#ifndef SIGMA_SDK_H
#define SIGMA_SDK_H

/**
 * @file    sigmaSDK.h
 * @brief   Sigma 仿真 SDK 接口类 — 封装 UDP 通信与协议解析，提供仿真控制指令发送能力
 *
 * @note    本类采用双线程架构：
 *          - 线程1 (rawRecvLoop) ：阻塞接收 UDP 原始字节流，写入共享缓冲区
 *          - 线程2 (parseLoop)   ：从共享缓冲区逐字节取出，喂给协议解析器，解析成功后查回调表并分发
 */

#include "dxfd_protocol/common.h"
#include "sigmaAssist.h"   // trajPoint、setTrajFlags 等轨迹辅助定义
#include <thread>
#include <string>
#include <map>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

#include <functional>

// ============================================================
// 缓冲区大小常量
// ============================================================

/** @brief 共享字节队列上限，防止接收过快导致内存溢出 */
#define MAX_QUEUE_SIZE (512 * 1024)

/** @brief 单次 recvfrom 接收缓冲区大小 */
#define RECV_BUF_SIZE  (20 * 1024)

// ============================================================
// 类型定义
// ============================================================

/** @brief 消息回调类型 — 解析到完整消息后按 msgID 查找并调用。使用 std::function 便于绑定 lambda/有状态对象 */
using MsgCallback = std::function<void(const dxfd_message_t&)>;

// ============================================================
// SigmaSDKDemo 类
// ============================================================

class SigmaSDKDemo
{
public:
    // ============================================================
    // 构造 / 析构
    // ============================================================

    /**
     * @brief 构造函数：创建 UDP socket 并启动双线程
     * @param ip        远端服务器 IP 地址
     * @param port      远端服务器端口号
     * @param localPort 本地绑定端口号（默认 10001）
     */
    SigmaSDKDemo(std::string ip, int port, int localPort = 26530);

    /** @brief 析构函数：安全停止线程并释放 socket 资源 */
    ~SigmaSDKDemo();

    // ============================================================
    // 公开接口：回调注册
    // ============================================================

    /** @brief 注册消息回调 — 同一 msgID 可注册多个回调，触发时依次调用 */
    void registerCallback(uint8_t msgID, MsgCallback cb);

    /** @brief 清除指定 msgID 下的所有已注册回调 */
    void clearCallbacks(uint8_t msgID);

    // ============================================================
    // 公开接口：发送仿真控制指令
    // ============================================================

    /**
     * @brief 发送仿真数据请求
     * @param method  查询方式
     * @param content 查询内容字符串
     */
    void SendRequestSimData(uint8_t method, std::string content);

    /**
     * @brief 发送实体运动控制指令
     * @param id    目标实体 ID
     * @param speed 运动速度（m/s），暂未控制
     * @param lat   目标纬度（整型）
     * @param lon   目标经度（整型）
     * @param alt   目标高度（浮点型 单位米）
     */
    void SendMotionCommand(int64_t id, float speed, int32_t lat, int32_t lon, float alt);

    /**
     * @brief 发送参考轨迹指令
     * @param id 目标实体 ID
     * @param flags 轨迹状态类型掩码（第一位为 yaw_deg，第二位为 speed_ms，第三位为 time_s，第四位为 curvature）
     * @param points 参考轨迹点列表
     */
    void SendReferenceTrajectory(int64_t id,  uint8_t flags, const std::vector<trajPoint>& points);

/**
     * @brief 发送模型标签设置指令
     * @param entityID 目标实体 ID
     * @param tags     标签字符串
     */
    void SendSetModelTags(int64_t entityID, std::string tags);

    /**
     * @brief 发送模型生成指令
     * @param modelType 模型类型名称
     * @param modelID   模型 ID
     * @param campID    阵营 ID
     * @param squadID   小队 ID
     * @param spawnType 生成方式：0 = 仅渲染，1 = 运动模拟
     * @param lat       初始纬度（整型）
     * @param lon       初始经度（整型）
     * @param alt       初始高度（浮点型 单位米）
     * @param roll      初始横滚角（°）
     * @param pitch     初始俯仰角（°）
     * @param yaw       初始偏航角（°）
     * @param scaleX    X 轴缩放系数
     * @param scaleY    Y 轴缩放系数
     * @param scaleZ    Z 轴缩放系数
     */
    void SendSpawnModel(std::string modelType, int campID, int squadID, int modelID, int spawnType,
                        int32_t lat, int32_t lon, float alt,
                        float roll, float pitch, float yaw,
                        float scaleX, float scaleY, float scaleZ);

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
    void SendWeaponFireCmd(int64_t shooterID, int64_t targetID, uint32_t ammoID,
                           uint8_t fireMode, uint8_t salvoCount, uint16_t seq,
                           uint16_t fireDelayMs, int32_t targetLat, int32_t targetLon,
                           float targetAlt, float leadAngle);

    /**
     * @brief 发送载荷查询
     * @param entityID 目标实体 ID
     */
    void SendLoadoutQuery(int64_t entityID);

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
    void SendVelocityCommand(int64_t entityID, uint8_t flags, uint16_t seq,
                             float vx, float vy, float vz, float yawDeg, float yawRateDegS,
                             float ax, float ay, float az);

private:
    // ============================================================
    // 线程 1：原始数据接收 — 只收不解析
    // ============================================================

    /** @brief UDP 接收循环（独立线程）— 阻塞读取并写入共享字节队列 */
    void rawRecvLoop();

    // ============================================================
    // 线程 2：协议解析 + 回调分发
    // ============================================================

    /** @brief 协议解析循环（独立线程）— 逐字节解析，完整消息触发回调分发 */
    void parseLoop();

    // ============================================================
    // 回调分发
    // ============================================================

    /**
     * @brief 按 msgID 查找回调表并依次调用
     * @param msg 已解析完成的协议消息
     */
    void dispatchCallbacks(const dxfd_message_t& msg);

    // ============================================================
    // UDP 发送
    // ============================================================

    /**
     * @brief 通过 UDP socket 发送原始字节数据
     * @param data    待发送数据指针
     * @param dataLen 数据长度（字节）
     * @return 实际发送字节数，<0 表示失败
     */
    int sendUdp(void* data, int dataLen);

private:
    // ============================================================
    // UDP Socket 相关成员
    // ============================================================

    int                _socketFd;    /**< UDP socket 文件描述符 */
    struct sockaddr_in _remoteAddr;  /**< 远端服务器地址（sendto 目标） */
    struct sockaddr_in _localAddr;   /**< 本地绑定地址（recvfrom 源） */
    socklen_t          _addrLen;     /**< sockaddr 结构体长度 */

    // ============================================================
    // 共享缓冲区（线程安全字节队列）
    // ============================================================

    std::vector<uint8_t> _rawDataVec;  /**< 原始接收字节队列 */
    std::mutex           _dataMutex;   /**< 保护 _rawDataVec 的互斥锁 */

    // ============================================================
    // 回调注册表
    // ============================================================

    std::mutex _callbackMutex;                                /**< 保护回调表的互斥锁 */
    std::map<uint8_t, std::vector<MsgCallback>> _callbacks;   /**< msgID → 回调列表 映射表 */

    // ============================================================
    // 线程管理
    // ============================================================

    std::thread        _rawRecvThread;  /**< 原始接收线程句柄 */
    std::thread        _parseThread;    /**< 协议解析线程句柄 */
    std::atomic<bool>  _shouldExit{false};  /**< 线程退出标志（原子变量，跨线程安全；必须显式初始化为 false，否则线程可能启动即退出） */
};

#endif // SIGMA_SDK_H
