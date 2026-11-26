/**
 * @file  sigma2ros_bridge.cpp
 * @brief Sigma2RosBridge 实现：参数装配、SDK 回调换算发布、上行指令下发
 *
 * 节点职责与数据通路见 sigma2ros_bridge.h。
 */

#include "sigma2ros_bridge.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "dxfd_protocol/dxfd_msg_origin_global_position.h"
#include "dxfd_protocol/dxfd_msg_model_info.h"
#include "dxfd_protocol/dxfd_msg_target_fusion_data.h"
#include "dxfd_protocol/dxfd_msg_weapon_spec.h"
#include "dxfd_protocol/dxfd_msg_weapon_fire_ack.h"

// header-only 工具（include/utils）
#include "geo_convert.h"
#include "units.h"
#include "yaw_convert.h"

#include "entity_naming.h"

namespace {

/** @brief 下行消息 msgID（协议定义） */
constexpr uint8_t kMsgOrigin       = DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION;
constexpr uint8_t kMsgModelInfo    = DXFD_MSG_ID_MODEL_INFO;
constexpr uint8_t kMsgTargetFusion = DXFD_MSG_ID_TARGET_FUSION_DATA;
constexpr uint8_t kMsgWeaponSpec   = DXFD_MSG_ID_WEAPON_SPEC;
constexpr uint8_t kMsgWeaponFireAck = DXFD_MSG_ID_WEAPON_FIRE_ACK;

} // namespace

Sigma2RosBridge::Sigma2RosBridge() : rclcpp::Node("sigma2ros_bridge")
{
    // ===== 参数 =====
    ue_ip_            = declare_parameter<std::string>("ue_ip", "127.0.0.1");
    ue_port_          = declare_parameter<int>("ue_port", 9990);
    local_port_       = declare_parameter<int>("local_port", 26530);
    pull_interval_ms_ = declare_parameter<int>("pull_interval_ms", 100);
    pull_method_      = static_cast<uint8_t>(declare_parameter<int>("pull_method", 1)); // 1 = MODEL_NAME
    loadout_timeout_ms_ = declare_parameter<int>("loadout_timeout_ms", 300);
    loadout_quiet_ms_   = declare_parameter<int>("loadout_quiet_ms", 50);
    home_source_      = declare_parameter<std::string>("home_source", "ue");
    home_wait_ms_     = declare_parameter<int>("home_wait_ms", 5000);
    home_lat_deg_     = declare_parameter<double>("home_lat_deg", 0.0);
    home_lon_deg_     = declare_parameter<double>("home_lon_deg", 0.0);
    home_alt_m_       = declare_parameter<double>("home_alt_m", 0.0);
    // Measured SigmaPI5 BigUAV MODEL_INFO altitude is metres, unlike the legacy
    // centimetre field contract. ORIGIN and other model types retain their units.
    biguav_altitude_scale_m_ = declare_parameter<double>("biguav_altitude_scale_m", 1.0);
    if (!std::isfinite(biguav_altitude_scale_m_) || biguav_altitude_scale_m_ <= 0.0)
        throw std::runtime_error("biguav_altitude_scale_m must be finite and positive");
    ns_               = declare_parameter<std::string>("namespace", "sigma");
    // 去首尾 '/'，得到话题根（空串 = 无顶层命名空间）
    while (!ns_.empty() && ns_.front() == '/') ns_.erase(0, 1);
    while (!ns_.empty() && ns_.back() == '/') ns_.pop_back();
    root_ = ns_.empty() ? "" : "/" + ns_;
    pull_contents_ = entity_naming::splitBy(declare_parameter<std::string>("pull_contents", ""), ',');

    // ===== 节点组件（声明顺序保证先于 sdk_ 析构）=====
    home_ = std::make_unique<HomeState>();
    topics_ = std::make_unique<EntityTopicManager>(*this, root_);
    topics_->registerCommand<MotionCommand>("motion_command",
        [this](int64_t raw, const MotionCommand& cmd) { onMotion(raw, cmd); });
    topics_->registerCommand<VelocityCommand>("velocity_command",
        [this](int64_t raw, const VelocityCommand& vel) { onVelocity(raw, vel); });
    topics_->registerCommand<ReferenceTrajectory>("reference_trajectory",
        [this](int64_t raw, const ReferenceTrajectory& traj) { onTrajectory(raw, traj); });
    topics_->registerCommand<WeaponFireCommand>("weapon_fire_command",
        [this](int64_t raw, const WeaponFireCommand& cmd) { onWeaponFire(raw, cmd); });

    // ===== 全局订阅 / 实体话题维护定时器 =====
    spawn_sub_ = create_subscription<SpawnModel>(root_ + "/spawn_model", 10,
        [this](const SpawnModel::SharedPtr msg) { onSpawnModel(*msg); });
    maintain_timer_ = create_wall_timer(std::chrono::milliseconds(kMaintainPeriodMs),
                                        [this] { topics_->maintain(); });

    // 载荷查询服务：请求-响应，UE 的多条 WEAPON_SPEC 由服务回调聚合后一次性应答
    loadout_srv_ = create_service<LoadoutQuery>(root_ + "/loadout_query",
        [this](const std::shared_ptr<LoadoutQuery::Request> req,
               std::shared_ptr<LoadoutQuery::Response> res) { handleLoadoutQuery(req, res); });

    // pull_contents 列出的实体在启动时即预开通指挥话题（不必等首条下行）
    for (const auto& s : pull_contents_)
    {
        entity_naming::EntityString e;
        if (!entity_naming::parseEntityString(s, e))
        {
            RCLCPP_WARN(get_logger(), "pull_contents 项 \"%s\" 无法解析为实体（格式 camp_squad_型号_model），跳过预开通",
                        s.c_str());
            continue;
        }
        const int64_t raw = getEntityID(e.type, e.camp, e.squad, e.model);
        const auto opened = topics_->preopenCommands(raw);
        if (!opened.empty())
        {
            std::string joined;
            for (const auto& topic : opened)
                joined += (joined.empty() ? "" : ", ") + topic;
            RCLCPP_INFO(get_logger(), "预开通指挥话题: %s", joined.c_str());
        }
    }

    // ===== SDK（成员声明顺序保证其最先析构：回调线程先于 publisher 析构停止）=====
    sdk_ = std::make_unique<SigmaSDKDemo>(ue_ip_, ue_port_, local_port_);

    // 下行回调：home 未就绪时自查跳过，就绪后直接换算发布
    sdk_->registerCallback(kMsgModelInfo,
        [this](const dxfd_message_t& m) { onModelInfo(m); });
    sdk_->registerCallback(kMsgTargetFusion,
        [this](const dxfd_message_t& m) { onTargetFusion(m); });
    sdk_->registerCallback(kMsgWeaponSpec,
        [this](const dxfd_message_t& m) { onWeaponSpec(m); });
    sdk_->registerCallback(kMsgWeaponFireAck,
        [this](const dxfd_message_t& m) { onWeaponFireAck(m); });

    // ===== home 来源（ue：等 ORIGIN 广播；cfg：直接用参数）=====
    if (home_source_ == "cfg")
    {
        home_->set(home_lat_deg_, home_lon_deg_, home_alt_m_);
        RCLCPP_INFO(get_logger(), "home 由参数加载: lat=%.7f lon=%.7f alt=%.2fm",
                    home_lat_deg_, home_lon_deg_, home_alt_m_);
        // 不注册 ORIGIN 回调：本地端口非 26530 收不到广播
    }
    else if (home_source_ == "ue")
    {
        if (home_wait_ms_ <= 0)
            throw std::runtime_error("home_wait_ms 必须为正数（当前 " +
                                     std::to_string(home_wait_ms_) + "）");
        sdk_->registerCallback(kMsgOrigin,
            [this](const dxfd_message_t& m) { onOrigin(m); });
        home_->waitReady(home_wait_ms_, get_logger());   // 超时 / Ctrl-C 均以异常结束构造
    }
    else
    {
        throw std::runtime_error("home_source 非法: \"" + home_source_ +
                                 "\"（应为 ue 或 cfg）");
    }

    // ===== 上行拉取定时器（请求-响应模式）=====
    if (!pull_contents_.empty())
    {
        pull_timer_ = create_wall_timer(std::chrono::milliseconds(pull_interval_ms_),
                                        [this] { pullOnce(); });
    }
    RCLCPP_INFO(get_logger(), "sigma2ros_bridge 就绪: UE=%s:%d local=%d, 话题根=%s, pull=%zu 项 @ %dms",
                ue_ip_.c_str(), ue_port_, local_port_, root_.c_str(), pull_contents_.size(), pull_interval_ms_);
}

// ============================================================
// SDK 回调（sdk 解析线程上下文）：换算后交组件发布（查表直发/入队）
// ============================================================

void Sigma2RosBridge::logRawOnce(const char* source, int64_t rawSelf, int64_t rawTarget)
{
    static std::atomic<bool> logged{false};
    if (!logged.exchange(true))
        RCLCPP_INFO(get_logger(), "[%s] 首条原始 entityID: self=%lld target=%lld",
                    source, static_cast<long long>(rawSelf), static_cast<long long>(rawTarget));
}

void Sigma2RosBridge::onOrigin(const dxfd_message_t& msg)
{
    dxfd_origin_global_position_t o;
    dxfd_msg_origin_global_position_decode(&msg, &o);
    const bool first = !home_->ready();
    // 协议单位：经纬微度（1e-7 °/lsb）、高度厘米
    home_->set(o.locationLat * units::kMicroDegToDeg,
               o.locationLon * units::kMicroDegToDeg,
               o.locationAlt * units::kCmToM);
    if (first)
        RCLCPP_INFO(get_logger(), "收到 UE ORIGIN，home 参考系已建立");
}

void Sigma2RosBridge::onModelInfo(const dxfd_message_t& msg)
{
    dxfd_model_info_t info;
    dxfd_msg_model_info_decode(&msg, &info);
    logRawOnce("model_info", info.entityID);

    double hLat, hLon, hAlt;
    if (!home_->get(&hLat, &hLon, &hAlt))
        return;   // home 未就绪，topic 暂空

    SigmaModelType modelType;
    uint8_t camp, squad;
    uint32_t model;
    parseEntityID(info.entityID, modelType, camp, squad, model);
    const double altitudeScale = static_cast<unsigned>(modelType) == 0
        ? biguav_altitude_scale_m_ : units::kCmToM;
    double e, n, u;
    geo::llaToEnu(hLat, hLon, hAlt,
                  info.locationLat * units::kMicroDegToDeg,
                  info.locationLon * units::kMicroDegToDeg,
                  info.locationAlt * altitudeScale, &e, &n, &u);

    ModelInfo out;
    entity_naming::fillEntityId(out.entity_id, info.entityID);
    out.x = e;
    out.y = n;
    out.z = u;
    // UE 速度 cm/s -> m/s；偏航 UE 左手系角度 -> ENU 弧度（正东 0、逆时针为正）
    out.vx  = info.velocityX * units::kCmToM;
    out.vy  = -1.0f * info.velocityY * units::kCmToM;
    out.vz  = info.velocityZ * units::kCmToM;
    out.yaw = yaw_conv::ueDegToEnuRad(info.attitudeYaw);

    topics_->publishModelInfo(info.entityID, out);
}

void Sigma2RosBridge::onTargetFusion(const dxfd_message_t& msg)
{
    dxfd_target_fusion_data_t t;
    dxfd_msg_target_fusion_data_decode(&msg, &t);
    logRawOnce("target_fusion", t.selfEntityID, t.targetEntityID);

    double hLat, hLon, hAlt;
    if (!home_->get(&hLat, &hLon, &hAlt))
        return;   // home 未就绪，topic 暂空

    double e, n, u;
    geo::llaToEnu(hLat, hLon, hAlt,
                  t.locationLat * units::kMicroDegToDeg,
                  t.locationLon * units::kMicroDegToDeg,
                  t.locationAlt * units::kCmToM, &e, &n, &u);

    TargetFusion out;
    entity_naming::fillEntityId(out.self_id, t.selfEntityID);    // 探测方归属键（解码）
    entity_naming::fillEntityId(out.target_id, t.targetEntityID); // 目标本体 ID（解码）
    out.x = e;
    out.y = n;
    out.z = u;
    out.vx = t.velocityX * units::kCmToM;
    out.vy = -1.0f * t.velocityY * units::kCmToM;
    out.vz = t.velocityZ * units::kCmToM;
    out.yaw = yaw_conv::ueDegToEnuRad(t.attitudeYaw);
    out.health_value = t.healthValue;

    topics_->publishFusion(t.selfEntityID, out);
}

void Sigma2RosBridge::onWeaponSpec(const dxfd_message_t& msg)
{
    dxfd_weapon_spec_t spec;
    dxfd_msg_weapon_spec_decode(&msg, &spec);

    std::lock_guard<std::mutex> lk(loadout_mutex_);
    if (!loadout_waiting_ || spec.entity_id != loadout_entity_)
        return;   // 非当前查询的实体，丢弃残留应答

    WeaponSpec out;
    out.ammo_id       = spec.ammoID;
    out.total_count   = spec.totalCount;
    out.guidance_type = spec.guidanceType;
    out.range         = spec.range;
    out.damage        = spec.damage;
    out.name.assign(spec.name, strnlen(spec.name, sizeof(spec.name)));
    loadout_specs_.push_back(std::move(out));
    loadout_last_rx_ = std::chrono::steady_clock::now();
    loadout_cv_.notify_all();
}

void Sigma2RosBridge::onWeaponFireAck(const dxfd_message_t& msg)
{
    dxfd_weapon_fire_ack_t ack;
    dxfd_msg_weapon_fire_ack_decode(&msg, &ack);

    // UE 暂用原 projectileID 字段承载开火平台实体 ID
    const int64_t raw = ack.projectileID;
    if (raw == 0)
    {
        RCLCPP_WARN(get_logger(), "weapon_fire_ack 的 entity_id 为 0（seq=%u），无法归属，丢弃",
                    static_cast<unsigned>(ack.seq));
        return;
    }

    WeaponFireAck out;
    entity_naming::fillEntityId(out.entity_id, raw);
    out.seq         = ack.seq;
    out.result_code = ack.resultCode;
    out.ammo_id     = ack.ammoID;
    out.ammo_remain = ack.ammoRemain;

    topics_->publishWeaponFireAck(raw, out);
}

// ============================================================
// 订阅回调（spin 线程上下文）：指令 -> UE
// ============================================================

void Sigma2RosBridge::onMotion(int64_t raw, const MotionCommand& cmd)
{
    const int64_t msgRaw = entity_naming::entityToRaw(cmd.entity_id);
    if (msgRaw != 0 && msgRaw != raw)
        RCLCPP_WARN(get_logger(),
                    "motion_command 的 entity_id(%lld) 与话题路径实体(%lld)不符，以话题路径为准",
                    static_cast<long long>(msgRaw), static_cast<long long>(raw));

    double hLat, hLon, hAlt;
    if (!home_->get(&hLat, &hLon, &hAlt))
    {
        RCLCPP_WARN(get_logger(), "home 未就绪，丢弃 motion 指令 (type=%u camp=%u squad=%u model=%u)",
                    cmd.entity_id.model_type, cmd.entity_id.camp_id,
                    cmd.entity_id.squad_id, cmd.entity_id.model_id);
        return;
    }
    double latDeg, lonDeg, altM;
    geo::enuToLla(hLat, hLon, hAlt, cmd.x, cmd.y, cmd.z, &latDeg, &lonDeg, &altM);
    // 业务侧米/ENU -> UE 侧微度整型 + 厘米
    sdk_->SendMotionCommand(raw,
                            static_cast<float>(cmd.speed),
                            static_cast<int32_t>(std::llround(latDeg * units::kDegToMicroDeg)),
                            static_cast<int32_t>(std::llround(lonDeg * units::kDegToMicroDeg)),
                            static_cast<float>(altM * units::kMToCm));
}

void Sigma2RosBridge::onVelocity(int64_t raw, const VelocityCommand& vel)
{
    const int64_t msgRaw = entity_naming::entityToRaw(vel.entity_id);
    if (msgRaw != 0 && msgRaw != raw)
        RCLCPP_WARN(get_logger(),
                    "velocity_command 的 entity_id(%lld) 与话题路径实体(%lld)不符，以话题路径为准",
                    static_cast<long long>(msgRaw), static_cast<long long>(raw));

    if (!home_->ready())
    {
        RCLCPP_WARN(get_logger(), "home 未就绪，丢弃 velocity_command (type=%u camp=%u squad=%u model=%u)",
                    vel.entity_id.model_type, vel.entity_id.camp_id,
                    vel.entity_id.squad_id, vel.entity_id.model_id);
        return;
    }

    // msg ENU 米/弧度 -> UE （ESU）米/度，这里UE又变成米单位了（速度为相对 origin 的局部坐标，无需 LLA 换算）
    float vx = static_cast<float>(vel.vx);
    float vy = static_cast<float>(-1.0f * vel.vy);
    float vz = static_cast<float>(vel.vz);
    float yaw = static_cast<float>(yaw_conv::enuRadToUeDeg(vel.yaw));
    float yaw_rate = static_cast<float>(-vel.yaw_rate * 180.0 / M_PI);
    float ax = static_cast<float>(vel.ax);
    float ay = static_cast<float>(-1.0f * vel.ay);          
    float az = static_cast<float>(vel.az);
    
    sdk_->SendVelocityCommand(raw, vel.flags, 0, vx, vy, vz, yaw, 
                                yaw_rate, ax, ay, az);
}


void Sigma2RosBridge::onTrajectory(int64_t raw, const ReferenceTrajectory& traj)
{
    const int64_t msgRaw = entity_naming::entityToRaw(traj.entity_id);
    if (msgRaw != 0 && msgRaw != raw)
        RCLCPP_WARN(get_logger(),
                    "reference_trajectory 的 entity_id(%lld) 与话题路径实体(%lld)不符，以话题路径为准",
                    static_cast<long long>(msgRaw), static_cast<long long>(raw));

    if (!home_->ready())
    {
        RCLCPP_WARN(get_logger(), "home 未就绪，丢弃 reference_trajectory (type=%u camp=%u squad=%u model=%u, %zu 点)",
                    traj.entity_id.model_type, traj.entity_id.camp_id,
                    traj.entity_id.squad_id, traj.entity_id.model_id, traj.points.size());
        return;
    }

    // msg ENU 米/弧度 -> UE 局部厘米/度（轨迹点为相对 origin 的局部坐标，无需 LLA 换算）
    std::vector<trajPoint> pts;
    pts.reserve(traj.points.size());
    for (const auto& p : traj.points)
    {
        trajPoint tp{};
        tp.x_cm      = static_cast<float>(p.x * units::kMToCm);
        tp.y_cm      = static_cast<float>(-1.0f * p.y * units::kMToCm);
        tp.z_cm      = static_cast<float>(p.z * units::kMToCm);
        tp.yaw_deg   = static_cast<float>(yaw_conv::enuRadToUeDeg(p.yaw));
        tp.speed_ms  = p.speed;
        tp.time_s    = p.time;
        tp.curvature = p.curvature;
        pts.push_back(tp);
    }

    sdk_->SendReferenceTrajectory(raw, traj.flags, pts);   // >200 点 SDK 内部自动分片
}

void Sigma2RosBridge::onWeaponFire(int64_t raw, const WeaponFireCommand& cmd)
{
    const int64_t msgRaw = entity_naming::entityToRaw(cmd.self_id);
    if (msgRaw != 0 && msgRaw != raw)
        RCLCPP_WARN(get_logger(),
                    "weapon_fire_command 的 self_id(%lld) 与话题路径实体(%lld)不符，以话题路径为准",
                    static_cast<long long>(msgRaw), static_cast<long long>(raw));

    // target_id 全零 = 按坐标开火，否则 UE 以目标实体 ID 为准（坐标字段忽略）
    const int64_t targetRaw = entity_naming::entityToRaw(cmd.target_id);
    int32_t targetLat = 0;
    int32_t targetLon = 0;
    float   targetAlt = 0.0f;
    if (targetRaw == 0)
    {
        double hLat, hLon, hAlt;
        if (!home_->get(&hLat, &hLon, &hAlt))
        {
            RCLCPP_WARN(get_logger(), "home 未就绪，丢弃 weapon_fire_command 坐标开火 (shooter=%lld)",
                        static_cast<long long>(raw));
            return;
        }
        double latDeg, lonDeg, altM;
        geo::enuToLla(hLat, hLon, hAlt, cmd.x, cmd.y, cmd.z, &latDeg, &lonDeg, &altM);
        targetLat = static_cast<int32_t>(std::llround(latDeg * units::kDegToMicroDeg));
        targetLon = static_cast<int32_t>(std::llround(lonDeg * units::kDegToMicroDeg));
        targetAlt = static_cast<float>(altM);   // 协议 targetAlt 单位为米
    }

    // 提前量 msg 弧度 -> 协议度
    sdk_->SendWeaponFireCmd(raw, targetRaw, cmd.ammo_id,
                            cmd.fire_mode, cmd.salvo_count, cmd.seq,
                            cmd.fire_delay_ms, targetLat, targetLon,
                            targetAlt,
                            static_cast<float>(cmd.lead_angle * 180.0 / M_PI));
}

// ============================================================
// 服务回调（spin 线程上下文）：载荷查询
// ============================================================

void Sigma2RosBridge::handleLoadoutQuery(const std::shared_ptr<LoadoutQuery::Request> req,
                                         std::shared_ptr<LoadoutQuery::Response> res)
{
    const int64_t raw = entity_naming::entityToRaw(req->entity_id);
    if (raw == 0)
    {
        res->success = false;
        res->message = "entity_id 不能为全零";
        return;
    }

    const auto timeout = std::chrono::milliseconds(loadout_timeout_ms_);
    const auto quiet   = std::chrono::milliseconds(loadout_quiet_ms_);

    {
        std::lock_guard<std::mutex> lk(loadout_mutex_);
        loadout_waiting_ = true;
        loadout_entity_  = raw;
        loadout_specs_.clear();
        loadout_last_rx_ = std::chrono::steady_clock::now();
    }

    sdk_->SendLoadoutQuery(raw);

    // WEAPON_SPEC 由 SDK 解析线程回填；此处等待“静默窗口到期”或“总超时”
    std::unique_lock<std::mutex> lk(loadout_mutex_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true)
    {
        const auto now = std::chrono::steady_clock::now();
        if (!loadout_specs_.empty() && now - loadout_last_rx_ >= quiet)
            break;   // 本批收完
        if (now >= deadline)
            break;   // 无应答或收包超过总超时

        auto wake = deadline;
        if (!loadout_specs_.empty())
            wake = std::min(deadline, loadout_last_rx_ + quiet);
        loadout_cv_.wait_until(lk, wake);
    }

    const bool got_any = !loadout_specs_.empty();
    res->loadout.entity_id = req->entity_id;
    res->loadout.ammos     = std::move(loadout_specs_);
    res->success           = got_any;
    if (!got_any)
        res->message = "timeout: UE 未返回 weapon_spec";

    loadout_waiting_ = false;
    loadout_entity_  = 0;
    loadout_specs_.clear();
}

void Sigma2RosBridge::onSpawnModel(const SpawnModel& cmd)
{
    double hLat, hLon, hAlt;
    if (!home_->get(&hLat, &hLon, &hAlt))
    {
        RCLCPP_WARN(get_logger(), "home 未就绪，丢弃 spawn 指令 (model=%s camp=%d squad=%d model=%d)",
                    cmd.model_name.c_str(), cmd.camp_id, cmd.squad_id, cmd.model_id);
        return;
    }
    double latDeg, lonDeg, altM;
    geo::enuToLla(hLat, hLon, hAlt, cmd.x, cmd.y, cmd.z, &latDeg, &lonDeg, &altM);
    // msg ENU -> UE 侧微度整型 + 厘米；yaw ENU 弧度 -> UE 左手系度
    // roll/pitch=0、scale=1：与 CXX Transmitter::spawnModel 的固定参数一致
    sdk_->SendSpawnModel(cmd.model_name,
                         cmd.camp_id, cmd.squad_id, cmd.model_id, cmd.spawn_type,
                         static_cast<int32_t>(std::llround(latDeg * units::kDegToMicroDeg)),
                         static_cast<int32_t>(std::llround(lonDeg * units::kDegToMicroDeg)),
                         static_cast<float>(altM * units::kMToCm),
                         0.0f, 0.0f,
                         static_cast<float>(yaw_conv::enuRadToUeDeg(cmd.yaw)),
                         1.0f, 1.0f, 1.0f);

    // 新实体纳入拉取列表（spin 线程内：订阅回调与拉取定时器同线程，无竞争），
    // 否则 MODEL_INFO 为拉取驱动，节点不请求就永远收不到该实体的下行数据
    const std::string content = std::to_string(cmd.camp_id) + "_" + std::to_string(cmd.squad_id)
                              + "_" + cmd.model_name + "_" + std::to_string(cmd.model_id);
    if (std::find(pull_contents_.begin(), pull_contents_.end(), content) == pull_contents_.end())
    {
        pull_contents_.push_back(content);
        RCLCPP_INFO(get_logger(), "spawn 实体已纳入拉取列表: %s", content.c_str());
    }
    if (!pull_timer_)
    {
        pull_timer_ = create_wall_timer(std::chrono::milliseconds(pull_interval_ms_),
                                        [this] { pullOnce(); });
        RCLCPP_INFO(get_logger(), "拉取定时器已创建: %dms", pull_interval_ms_);
    }
}

// ============================================================
// 上行拉取泵（spin 线程定时器上下文）
// ============================================================

void Sigma2RosBridge::pullOnce()
{
    for (const auto& c : pull_contents_)
        sdk_->SendRequestSimData(pull_method_, c);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int rc = 0;
    try
    {
        rclcpp::spin(std::make_shared<Sigma2RosBridge>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sigma2ros_bridge"), "节点异常退出: %s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
