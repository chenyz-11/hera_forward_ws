/**
 * @file    sigmaSDK.cpp
 * @brief   SigmaSDKDemo 类实现 — UDP 通信、协议解析、回调分发与指令发送
 */

#include "sigmaSDK.h"
#include "dxfd_protocol/dxfd_msg_reference_trajectory.h"

#include <algorithm>

// ============================================================
// 构造 / 析构
// ============================================================

/**
 * @brief 构造函数：初始化 UDP 通信并启动消息监听双线程
 * @param ip        远端服务器 IP 地址
 * @param port      远端服务器端口号
 * @param localPort 本地绑定端口号
 */
SigmaSDKDemo::SigmaSDKDemo(std::string ip, int port, int localPort)
{
    // ============================
    // 1. 创建 UDP socket
    // ============================
    _socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_socketFd < 0)
    {
        printf("create socket failed\n");
        return;
    }

    // 设置 1 秒接收超时，确保退出时能检查 _shouldExit
    struct timeval tv_out;
    tv_out.tv_sec  = 1;
    tv_out.tv_usec = 0;
    setsockopt(_socketFd, SOL_SOCKET, SO_RCVTIMEO, &tv_out, sizeof(tv_out));

    // 绑定本地端口
    memset(&_localAddr, 0, sizeof(_localAddr));
    _localAddr.sin_family      = AF_INET;
    _localAddr.sin_port        = htons(localPort);
    _localAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(_socketFd, (struct sockaddr*)&_localAddr, sizeof(_localAddr)) == -1)
    {
        printf("bind socket failed, local port -> %d\n", localPort);
        return;
    }

    // 设置远端地址（用于 sendto）
    memset(&_remoteAddr, 0, sizeof(_remoteAddr));
    _remoteAddr.sin_family      = AF_INET;
    _remoteAddr.sin_addr.s_addr = inet_addr(ip.c_str());
    _remoteAddr.sin_port        = htons(port);

    _addrLen = sizeof(_localAddr);

    printf("UDP socket initialized, local:%d remote:%s:%d\n", localPort, ip.c_str(), port);

    // ============================
    // 2. 启动双线程
    // ============================
    _rawRecvThread = std::thread(&SigmaSDKDemo::rawRecvLoop, this);
    _parseThread   = std::thread(&SigmaSDKDemo::parseLoop, this);
}

/**
 * @brief 析构函数：安全停止监听线程，关闭并释放 UDP 资源
 */
SigmaSDKDemo::~SigmaSDKDemo()
{
    // 设置退出标志，通知两个工作线程停止
    _shouldExit = true;

    // 关闭 socket 以唤醒阻塞在 recvfrom 的接收线程
    if (_socketFd >= 0)
        close(_socketFd);

    // 等待两个线程安全退出
    if (_rawRecvThread.joinable())
        _rawRecvThread.join();
    if (_parseThread.joinable())
        _parseThread.join();

    printf("SigmaSDKDemo destroyed\n");
}

// ============================================================
// 回调注册与清除
// ============================================================

/**
 * @brief 注册消息回调 — 同一 msgID 支持挂载多个回调函数
 * @param msgID 消息类型 ID（协议定义的消息号）
 * @param cb    回调函数指针
 */
void SigmaSDKDemo::registerCallback(uint8_t msgID, MsgCallback cb)
{
    std::lock_guard<std::mutex> lock(_callbackMutex);
    _callbacks[msgID].push_back(cb);
    printf("callback registered for msgID: %d\n", (int)msgID);
}

/**
 * @brief 清除指定 msgID 下的所有已注册回调
 * @param msgID 消息类型 ID
 */
void SigmaSDKDemo::clearCallbacks(uint8_t msgID)
{
    std::lock_guard<std::mutex> lock(_callbackMutex);
    _callbacks.erase(msgID);
}

// ============================================================
// 发送指令：仿真数据请求
// ============================================================

/**
 * @brief 发送仿真数据请求指令
 * @param method  查询方式（按模型名称 MODEL_NAME 或按模型类型 MODEL_TYPE）
 * @param content 查询内容字符串，格式如 "阵营_小队_类型_ID"
 */
void SigmaSDKDemo::SendRequestSimData(uint8_t method, std::string content)
{
    dxfd_request_sim_data_t rSimData;
    dxfd_message_t msg;
    uint8_t tmpBuffer[DXFD_MAX_PAYLOAD_LEN];

    // 初始化结构体
    memset(&rSimData, 0, sizeof(dxfd_request_sim_data_t));

    // 填充请求参数
    rSimData.queryMethod = method;
    int count = content.length() > QUERY_CONTENT_LEN ? QUERY_CONTENT_LEN : content.length();
    memcpy(rSimData.queryContent, content.c_str(), count);
    rSimData.contentLength = count;

    // 协议编码
    dxfd_msg_request_sim_data_encode(&msg, CONTROLLER_ID, 1, rSimData);
    int len = dxfd_msg_to_send_buffer(&msg, tmpBuffer);

    // 通过 UDP 发送
    sendUdp(tmpBuffer, len);
}

// ============================================================
// 发送指令：实体运动控制
// ============================================================

/**
 * @brief 发送实体运动控制指令
 * @param id    目标实体 ID（64 位复合 ID）
 * @param speed 运动速度（m/s），暂未控制
 * @param lat   目标纬度坐标（整型）
 * @param lon   目标经度坐标（整型）
 * @param alt   目标高度坐标（浮点型 单位米）
 */
void SigmaSDKDemo::SendMotionCommand(int64_t id, float speed, int32_t lat, int32_t lon, float alt)
{
    dxfd_motion_command_t motionCommand;
    dxfd_message_t msg;
    uint8_t tmpBuffer[DXFD_MAX_PAYLOAD_LEN];

    // 初始化结构体
    memset(&motionCommand, 0, sizeof(dxfd_motion_command_t));

    // 填充运动控制参数
    motionCommand.entityID     = id;
    motionCommand.locationLon  = lon;
    motionCommand.locationLat  = lat;
    motionCommand.locationAlt  = alt;
    motionCommand.controlMode  = 1;      // 控制模式：1 = 目标点导航
    motionCommand.speed        = speed;     // 默认速度 20 m/s

    // 协议编码
    dxfd_msg_motion_command_encode(&msg, CONTROLLER_ID, 1, motionCommand);
    int len = dxfd_msg_to_send_buffer(&msg, tmpBuffer);

    // 通过 UDP 发送
    sendUdp(tmpBuffer, len);
}

// ============================================================
// 发送指令：参考轨迹
// ============================================================

/**
     * @brief 发送参考轨迹指令（>DXFD_REF_TRAJ_MAX_POINTS 点自动分片：首包 SET + 后续 APPEND）
     * @param id 目标实体 ID
     * @param flags 轨迹状态类型掩码（第一位为 yaw_deg，第二位为 speed_ms，第三位为 time_s，第四位为 curvature）
     * @param points 参考轨迹点列表
     */
void SigmaSDKDemo::SendReferenceTrajectory(int64_t id,  uint8_t flags, const std::vector<trajPoint>& points)
{
    dxfd_message_t msg;
    uint8_t tmpBuffer[DXFD_MAX_PAYLOAD_LEN];

    const size_t total = points.size();
    size_t sent = 0;
    uint16_t seq = 0;

    while (sent < total || seq == 0)   // 空轨迹也发一包 SET(count=0)，对齐"全量替换"语义
    {
        const size_t chunk = std::min<size_t>(total - sent, DXFD_REF_TRAJ_MAX_POINTS);

        dxfd_traj_point_t trajPoints[DXFD_REF_TRAJ_MAX_POINTS];
        for (size_t i = 0; i < chunk; ++i)
        {
            const trajPoint& src = points[sent + i];
            trajPoints[i].x_cm      = src.x_cm;
            trajPoints[i].y_cm      = src.y_cm;
            trajPoints[i].z_cm      = src.z_cm;
            trajPoints[i].yaw_deg   = src.yaw_deg;
            trajPoints[i].speed_ms  = src.speed_ms;
            trajPoints[i].time_s    = src.time_s;
            trajPoints[i].curvature = src.curvature;
        }

        dxfd_msg_reference_trajectory_encode(&msg, CONTROLLER_ID, 1,
                                             id,
                                             seq == 0 ? DXFD_TRAJ_CMD_SET : DXFD_TRAJ_CMD_APPEND,
                                             flags,
                                             seq,
                                             (sent + chunk < total) ? 1 : 0,
                                             trajPoints,
                                             static_cast<uint16_t>(chunk));
        int len = dxfd_msg_to_send_buffer(&msg, tmpBuffer);
        sendUdp(tmpBuffer, len);

        sent += chunk;
        ++seq;
    }
}


// ============================================================
// 发送指令：模型标签设置
// ============================================================

/**
 * @brief 发送模型标签设置指令
 * @param entityID 目标实体 ID
 * @param tags     标签字符串（逗号分隔），如 "test,uav,fixedwing"
 */
void SigmaSDKDemo::SendSetModelTags(int64_t entityID, std::string tags)
{
    dxfd_set_model_tag_t modelTag;
    dxfd_message_t msg;
    uint8_t tmpBuffer[DXFD_MAX_PAYLOAD_LEN];

    // 初始化结构体
    memset(&modelTag, 0, sizeof(dxfd_set_model_tag_t));

    // 检查标签长度是否越界
    if (tags.length() > TAG_CONTENT_LEN)
    {
        perror("model tags length overflow");
        return;
    }

    // 填充标签参数
    modelTag.entityID       = entityID;
    modelTag.contentLength  = tags.length();
    memcpy(modelTag.tagContent, tags.c_str(), tags.length());

    // 协议编码
    dxfd_msg_set_model_tag_encode(&msg, CONTROLLER_ID, 1, modelTag);
    int len = dxfd_msg_to_send_buffer(&msg, tmpBuffer);

    // 通过 UDP 发送
    sendUdp(tmpBuffer, len);
}

// ============================================================
// 发送指令：模型生成
// ============================================================

/**
 * @brief 发送模型生成（Spawn）指令
 * @param modelType 模型类型名称
 * @param modelID   模型 ID
 * @param campID    阵营 ID
 * @param squadID   队伍 ID
 * @param spawnType 生成方式：0 = 仅渲染，1 = 参与运动模拟
 * @param lat       初始纬度坐标（整型）
 * @param lon       初始经度坐标（整型）
 * @param alt       初始高度坐标（浮点型，单位米）
 * @param roll      初始横滚角（°）
 * @param pitch     初始俯仰角（°）
 * @param yaw       初始偏航角（°）
 * @param scaleX    X 轴缩放系数
 * @param scaleY    Y 轴缩放系数
 * @param scaleZ    Z 轴缩放系数
 */
void SigmaSDKDemo::SendSpawnModel(std::string modelType, int campID, int squadID, int modelID, int spawnType,
                                  int32_t lat, int32_t lon, float alt,
                                  float roll, float pitch, float yaw,
                                  float scaleX, float scaleY, float scaleZ)
{
    dxfd_spawn_model_t spawnModel;
    dxfd_message_t msg;
    uint8_t tmpBuffer[DXFD_MAX_PAYLOAD_LEN];

    // 初始化结构体
    memset(&spawnModel, 0, sizeof(dxfd_spawn_model_t));

    // 填充模型生成参数
    spawnModel.modelID        = modelID;
    spawnModel.squadID        = squadID;
    spawnModel.campID         = campID;
    spawnModel.spawnType      = spawnType;
    memcpy(spawnModel.modelType, modelType.c_str(), modelType.length());
    spawnModel.locationLat    = lat;
    spawnModel.locationLon    = lon;
    spawnModel.locationAlt    = alt;
    spawnModel.attitudeRoll   = roll;
    spawnModel.attitudePitch  = pitch;
    spawnModel.attitudeYaw    = yaw;
    spawnModel.scaleX         = scaleX;
    spawnModel.scaleY         = scaleY;
    spawnModel.scaleZ         = scaleZ;

    // 协议编码
    dxfd_msg_spawn_model_encode(&msg, CONTROLLER_ID, 1, spawnModel);
    int len = dxfd_msg_to_send_buffer(&msg, tmpBuffer);

    // 通过 UDP 发送
    sendUdp(tmpBuffer, len);
}

// ============================================================
// 发送指令：开火命令
// ============================================================

/**
 * @brief 发送开火命令
 * @param shooterID    开火平台实体 ID
 * @param targetID     目标实体 ID（=0 表示按坐标开火）
 * @param ammoID       弹药型号编号（0=当前装填/默认弹种）
 * @param fireMode     开火方式（FIRE_MODE：单发/连发/齐射）
 * @param salvoCount   齐射/连发数量（单发=1）
 * @param seq          命令序号（用于与 ACK 关联）
 * @param fireDelayMs  延时开火（毫秒，0=立即）
 * @param targetLat    目标纬度（targetID==0 时有效）*1e7
 * @param targetLon    目标经度（targetID==0 时有效）*1e7
 * @param targetAlt    目标高度（米）
 * @param leadAngle    提前量（度，0=自动解算）
 */
void SigmaSDKDemo::SendWeaponFireCmd(int64_t shooterID, int64_t targetID, uint32_t ammoID,
                                     uint8_t fireMode, uint8_t salvoCount, uint16_t seq,
                                     uint16_t fireDelayMs, int32_t targetLat, int32_t targetLon,
                                     float targetAlt, float leadAngle)
{
    dxfd_weapon_fire_cmd_t cmd;
    dxfd_message_t msg;
    uint8_t tmpBuffer[DXFD_MAX_PAYLOAD_LEN];

    memset(&cmd, 0, sizeof(cmd));
    cmd.shooterID   = shooterID;
    cmd.targetID    = targetID;
    cmd.ammoID      = ammoID;
    cmd.fireMode    = fireMode;
    cmd.salvoCount  = salvoCount;
    cmd.seq         = seq;
    cmd.fireDelayMs = fireDelayMs;
    cmd.targetLat   = targetLat;
    cmd.targetLon   = targetLon;
    cmd.targetAlt   = targetAlt;
    cmd.leadAngle   = leadAngle;

    dxfd_msg_weapon_fire_cmd_encode(&msg, CONTROLLER_ID, 1, cmd);
    int len = dxfd_msg_to_send_buffer(&msg, tmpBuffer);
    sendUdp(tmpBuffer, len);
}

// ============================================================
// 发送指令：载荷查询
// ============================================================

/**
 * @brief 发送载荷查询
 * @param entityID 目标实体 ID
 */
void SigmaSDKDemo::SendLoadoutQuery(int64_t entityID)
{
    dxfd_loadout_query_t query;
    dxfd_message_t msg;
    uint8_t tmpBuffer[DXFD_MAX_PAYLOAD_LEN];

    memset(&query, 0, sizeof(query));
    query.entityID = entityID;

    dxfd_msg_loadout_query_encode(&msg, CONTROLLER_ID, 1, query);
    int len = dxfd_msg_to_send_buffer(&msg, tmpBuffer);
    sendUdp(tmpBuffer, len);
}

// ============================================================
// 发送指令：速度控制指令
// ============================================================

/**
 * @brief 发送速度控制指令
 * @param entityID    目标实体 ID
 * @param flags       速度指令标志位（DXFD_VEL_FLAG 位掩码）
 * @param seq         命令序号
 * @param vx          东向速度（m/s）
 * @param vy          南向速度（m/s）
 * @param vz          天向速度（m/s，向上为正）
 * @param yawDeg      航向 setpoint（deg）
 * @param yawRateDegS 偏航率 setpoint（deg/s）
 * @param ax          东向加速度前馈（m/s²）
 * @param ay          南向加速度前馈（m/s²）
 * @param az          天向加速度前馈（m/s²）
 */
void SigmaSDKDemo::SendVelocityCommand(int64_t entityID, uint8_t flags, uint16_t seq,
                                       float vx, float vy, float vz, float yawDeg, float yawRateDegS,
                                       float ax, float ay, float az)
{
    dxfd_velocity_command_t cmd;
    dxfd_message_t msg;
    uint8_t tmpBuffer[DXFD_MAX_PAYLOAD_LEN];

    memset(&cmd, 0, sizeof(cmd));
    cmd.entity_id       = entityID;
    cmd.flags           = flags;
    cmd.seq             = seq;
    cmd.vx_m_s          = vx;
    cmd.vy_m_s          = vy;
    cmd.vz_m_s          = vz;
    cmd.yaw_deg         = yawDeg;
    cmd.yaw_rate_deg_s  = yawRateDegS;
    cmd.ax_m_s2         = ax;
    cmd.ay_m_s2         = ay;
    cmd.az_m_s2         = az;

    dxfd_msg_velocity_command_encode(&msg, CONTROLLER_ID, 1, cmd);
    int len = dxfd_msg_to_send_buffer(&msg, tmpBuffer);
    sendUdp(tmpBuffer, len);
}

// ============================================================
// 线程 1：原始数据接收（只收不解析）
// ============================================================

/**
 * @brief UDP 原始接收线程 — 阻塞接收字节流并写入共享队列
 *
 * @note  通过 _shouldExit 原子变量控制退出；
 *         socket 设置 1s 超时后每轮循环检查一次退出标志
 */
void SigmaSDKDemo::rawRecvLoop()
{
    uint8_t buffer[RECV_BUF_SIZE];

    while (!_shouldExit)
    {
        // 阻塞接收（带 1 秒超时）
        int recvBytes = recvfrom(_socketFd, buffer, sizeof(buffer), 0,
                                 (struct sockaddr*)&_localAddr, &_addrLen);

        if (recvBytes < 0)
        {
            // 超时或错误 — 检查是否应当退出
            if (!_shouldExit)
                continue;
            else
                break;
        }

        if (recvBytes > 0)
        {
            // 将接收到的字节写入共享队列（加锁保护）
            std::lock_guard<std::mutex> lock(_dataMutex);
            for (int i = 0; i < recvBytes; i++)
            {
                _rawDataVec.emplace_back(buffer[i]);
            }
        }
    }
}

// ============================================================
// 线程 2：协议解析 + 回调分发
// ============================================================

/**
 * @brief 协议解析线程 — 从共享队列取字节，逐字节喂给协议解析器
 *
 * @note  解析到完整消息后调用 dispatchCallbacks() 进行回调分发；
 *         每次批量取出队列中所有数据后清空，避免重复解析
 */
void SigmaSDKDemo::parseLoop()
{
    dxfd_message_t        msg;
    dxfd_message_status_t status;
    memset(&msg, 0, sizeof(msg));

    printf("ParseThread started\n");

    while (!_shouldExit)
    {
        std::vector<uint8_t> parseData;

        // -------- 从共享队列批量取出所有待解析数据 --------
        {
            std::unique_lock<std::mutex> lock(_dataMutex);
            if (_rawDataVec.size() > 0)
                parseData = _rawDataVec;
            _rawDataVec.clear();
        }

        // -------- 逐字节喂给协议解析器 --------
        for (int i = 0; i < parseData.size(); i++)
        {
            if (dxfd_msg_parse_char(&parseData.at(i), &msg, &status))
            {
                // 解析到一条完整消息 — 查回调表并分发
                dispatchCallbacks(msg);
            }
        }
    }

    printf("ParseThread exited\n");
}

// ============================================================
// 回调分发
// ============================================================

/**
 * @brief 按消息的 msgID 查找回调表，依次调用所有已注册回调
 * @param msg 已解析完成的协议消息（只读引用）
 *
 * @note  若 msgID 未注册任何回调，打印提示信息；
 *         回调在锁内执行，用户回调不宜包含长时间阻塞操作
 */
void SigmaSDKDemo::dispatchCallbacks(const dxfd_message_t& msg)
{
    std::lock_guard<std::mutex> lock(_callbackMutex);

    auto it = _callbacks.find(msg.msgID);
    if (it != _callbacks.end())
    {
        for (auto& cb : it->second)
        {
            cb(msg);
        }
    }
    else
    {
        // printf("no callback registered for msgID: %d\n", (int)msg.msgID);
    }
}

// ============================================================
// UDP 发送
// ============================================================

/**
 * @brief 通过 UDP socket 发送原始字节数据
 * @param data    待发送数据指针
 * @param dataLen 数据长度（字节）
 * @return        实际发送字节数，<0 表示发送失败
 *
 * @note  发送后 sleep 1ms 以降低瞬时发送速率，避免丢包
 */
int SigmaSDKDemo::sendUdp(void* data, int dataLen)
{
    int sz = sendto(_socketFd, data, dataLen, 0,
                    (struct sockaddr*)&_remoteAddr, sizeof(struct sockaddr_in));
    usleep(1000);   // 1ms 间隔，防止连续发送导致丢包
    return sz;
}
