#ifndef SIGMA2ROS_BRIDGE_H
#define SIGMA2ROS_BRIDGE_H

/**
 * @file  sigma2ros_bridge.h
 * @brief Sigma UE 仿真 ROS2 纯桥接节点（不创建 WorldState/Interface，直连 SigmaSDKDemo）
 *
 * 数据通路：
 *   下行  SDK 回调解码 MODEL_INFO / TARGET_FUSION -> utils 换算 ENU -> 直接发布 msg（不落库）
 *   上行  rclcpp Timer 定时 SendRequestSimData（请求-响应拉取）
 *         订阅 .../motion_command -> utils 换算 LLA -> SendMotionCommand 下发 UE
 *         订阅 .../velocity_command -> utils 换算 ESU 米/度 -> SendVelocityCommand 下发 UE
 *         订阅 .../reference_trajectory -> utils 换算局部厘米/度 -> SendReferenceTrajectory 下发 UE
 *         订阅 .../weapon_fire_command -> utils 换算 LLA -> SendWeaponFireCmd      下发 UE
 *         服务 {root}/loadout_query -> SendLoadoutQuery + 聚合 WEAPON_SPEC -> 一次性应答
 *         订阅 .../spawn_model    -> utils 换算 LLA -> SendSpawnModel  下发 UE（生成/渲染模型），
 *                                    并把新实体串自动纳入拉取列表（MODEL_INFO 为拉取驱动，
 *                                    不请求就收不到该实体下行；列表空时顺带创建拉取定时器）
 *
 * 话题树与惰性开通机制详见 entity_topics.h；实体命名/解析详见 entity_naming.h；
 * home 状态与等待详见 home_state.h。本文件只负责装配：参数、订阅/定时器、
 * SDK 创建与回调注册、home 来源、指令换算下发。
 *
 * home 两种来源（参数 home_source）：
 *   ue  等 UE 的 ORIGIN 广播，最多等 home_wait_ms（默认 5000），超时 FATAL 退出
 *   cfg 直接使用 home_lat_deg / home_lon_deg / home_alt_m 参数（UE 广播不可达时用）
 * home 未就绪期间：下行数据跳过（topic 暂空），上行指令丢弃并告警。
 *
 * 线程模型：SDK 解析线程回调（下行）与 rclcpp 执行器（上行/定时器）并发，
 * 共享状态由 HomeState 的原子量/锁与 EntityTopicManager 内部机制保护。
 */

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sigmaSDK.h"

#include "entity_topics.h"
#include "home_state.h"

#include "sigma2ros_bridge/msg/loadout.hpp"
#include "sigma2ros_bridge/msg/model_info.hpp"
#include "sigma2ros_bridge/msg/motion_command.hpp"
#include "sigma2ros_bridge/msg/reference_trajectory.hpp"
#include "sigma2ros_bridge/msg/spawn_model.hpp"
#include "sigma2ros_bridge/msg/target_fusion.hpp"
#include "sigma2ros_bridge/msg/velocity_command.hpp"
#include "sigma2ros_bridge/msg/weapon_fire_command.hpp"
#include "sigma2ros_bridge/msg/weapon_fire_ack.hpp"
#include "sigma2ros_bridge/msg/weapon_spec.hpp"
#include "sigma2ros_bridge/srv/loadout_query.hpp"


class Sigma2RosBridge : public rclcpp::Node
{
    using ModelInfo     = sigma2ros_bridge::msg::ModelInfo;
    using TargetFusion  = sigma2ros_bridge::msg::TargetFusion;
    using MotionCommand = sigma2ros_bridge::msg::MotionCommand;
    using VelocityCommand = sigma2ros_bridge::msg::VelocityCommand;
    using SpawnModel    = sigma2ros_bridge::msg::SpawnModel;
    using ReferenceTrajectory = sigma2ros_bridge::msg::ReferenceTrajectory;
    using WeaponFireCommand = sigma2ros_bridge::msg::WeaponFireCommand;
    using WeaponFireAck     = sigma2ros_bridge::msg::WeaponFireAck;
    using WeaponSpec         = sigma2ros_bridge::msg::WeaponSpec;
    using Loadout            = sigma2ros_bridge::msg::Loadout;
    using LoadoutQuery       = sigma2ros_bridge::srv::LoadoutQuery;

public:
    /** @brief 读取参数、装配组件与订阅、创建 SDK 并按 home_source 建立 home 参考系 */
    Sigma2RosBridge();

private:
    // ============================================================
    // SDK 回调（sdk 解析线程上下文）：换算后交组件发布（查表直发/入队）
    // ============================================================

    /** @brief 各数据源首条消息打印原始 64 位 entityID，便于核对解码/排查旧进程串台 */
    void logRawOnce(const char* source, int64_t rawSelf, int64_t rawTarget = 0);

    void onOrigin(const dxfd_message_t& msg);
    void onModelInfo(const dxfd_message_t& msg);
    void onTargetFusion(const dxfd_message_t& msg);

    /** @brief WEAPON_SPEC 回调：仅回填当前 loadout 查询的聚合缓冲，不直接发布 */
    void onWeaponSpec(const dxfd_message_t& msg);

    /** @brief WEAPON_FIRE_ACK 回调：按 seq -> 开火平台映射发布到该实体目录 */
    void onWeaponFireAck(const dxfd_message_t& msg);

    // ============================================================
    // 订阅回调（spin 线程上下文）：指令 -> UE
    // ============================================================

    /** @brief 航点话题回调：实体由话题路径决定（raw），msg 内 entity_id 仅作一致性校验（全零视为未填） */
    void onMotion(int64_t raw, const MotionCommand& cmd);

    /** @brief 速度话题回调：实体由话题路径决定（raw），msg 内 entity_id 仅作一致性校验（全零视为未填） */
    void onVelocity(int64_t raw, const VelocityCommand& vel);

    /** @brief 轨迹话题回调：实体由话题路径决定（raw），msg 内 entity_id 仅作一致性校验（全零视为未填） */
    void onTrajectory(int64_t raw, const ReferenceTrajectory& traj);

    /** @brief 开火话题回调：开火平台由话题路径决定（raw），msg 内 self_id 仅作一致性校验（全零视为未填） */
    void onWeaponFire(int64_t raw, const WeaponFireCommand& cmd);


    void onSpawnModel(const SpawnModel& cmd);

    // ============================================================
    // 服务回调（spin 线程上下文）：载荷查询
    // ============================================================

    /** @brief 发 loadout_query 并阻塞聚合多条 WEAPON_SPEC（静默窗口/超时后应答） */
    void handleLoadoutQuery(const std::shared_ptr<LoadoutQuery::Request> req,
                            std::shared_ptr<LoadoutQuery::Response> res);

    // ============================================================
    // 上行拉取泵（spin 线程定时器上下文）
    // ============================================================

    void pullOnce();

    // ============================================================
    // 成员（声明顺序即析构顺序的逆序：sdk_ 必须最后声明、最先析构）
    // ============================================================

    // 参数
    std::string ue_ip_;
    int ue_port_          = 9990;
    int local_port_       = 26530;
    int pull_interval_ms_ = 100;
    uint8_t pull_method_  = 1;
    int loadout_timeout_ms_ = 300;   // loadout 查询总超时：期间无任何 WEAPON_SPEC 即失败
    int loadout_quiet_ms_   = 50;    // 收到 spec 后静默多久视为“本批收完”
    std::string home_source_ = "ue";
    int home_wait_ms_     = 5000;
    double home_lat_deg_  = 0.0;
    double home_lon_deg_  = 0.0;
    double home_alt_m_    = 0.0;
    double biguav_altitude_scale_m_ = 1.0;
    std::string ns_;
    std::string root_;
    std::vector<std::string> pull_contents_;

    // 节点组件（先于 sdk_ 析构：publisher/subscription 生命周期短于 SDK 回调线程）
    std::unique_ptr<HomeState> home_;
    std::unique_ptr<EntityTopicManager> topics_;

    // ROS IO（全局）
    rclcpp::Subscription<SpawnModel>::SharedPtr spawn_sub_;
    rclcpp::Service<LoadoutQuery>::SharedPtr loadout_srv_;
    rclcpp::TimerBase::SharedPtr maintain_timer_;
    rclcpp::TimerBase::SharedPtr pull_timer_;

    // loadout 聚合：服务线程等待，SDK 解析线程回填
    std::mutex loadout_mutex_;
    std::condition_variable loadout_cv_;
    bool loadout_waiting_ = false;
    int64_t loadout_entity_ = 0;
    std::vector<WeaponSpec> loadout_specs_;
    std::chrono::steady_clock::time_point loadout_last_rx_;

    // SDK：最后声明 = 最先析构（回调线程先于 publisher 析构停止，避免 use-after-free）
    std::unique_ptr<SigmaSDKDemo> sdk_;
};

#endif // SIGMA2ROS_BRIDGE_H
