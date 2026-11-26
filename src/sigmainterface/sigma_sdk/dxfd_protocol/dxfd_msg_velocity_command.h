//
// 速度控制指令（外部算法 → 车辆）
// 与 reference_trajectory 对齐：同一局部 ENU 坐标系（x=东, y=北, z=上）、同一 EntityID、同一 flags 模式。
// 用途：轨迹点之间的在线速度修正 / 实时速度控制。加速度为前馈量（DXFD_VEL_FLAG_ACCEL）。
//

#ifndef DXFD_MSG_VELOCITY_COMMAND_H
#define DXFD_MSG_VELOCITY_COMMAND_H

#include "protocol.h"

#define DXFD_MSG_ID_VELOCITY_COMMAND_LEN sizeof(dxfd_velocity_command_t)

// flags 位掩码：本次速度指令各分量是否生效（use 标志）
enum DXFD_VEL_FLAG {
    DXFD_VEL_FLAG_VELOCITY = 1 << 0,   // bUseVelocity: vx/vy/vz 有效
    DXFD_VEL_FLAG_YAW      = 1 << 1,   // bUseYaw: yaw_deg 有效
    DXFD_VEL_FLAG_YAW_RATE = 1 << 2,   // bUseYawRate: yaw_rate_deg_s 有效
    DXFD_VEL_FLAG_ACCEL    = 1 << 3,   // bUseAccel: ax/ay/az 有效（加速度前馈）
};

#pragma pack(push,1)
typedef struct __dxfd_velocity_command_t {
    int64_t entity_id;        // 目标车辆 EntityID（与 reference_trajectory.entity_id 一致）
    uint8_t  flags;           // DXFD_VEL_FLAG 位掩码
    uint16_t seq;             // 命令序号（回绕），供乱序/丢包判定
    float    vx_m_s;          // 局部东向速度（m/s），DXFD_VEL_FLAG_VELOCITY
    float    vy_m_s;          // 局部南向速度（m/s），DXFD_VEL_FLAG_VELOCITY
    float    vz_m_s;          // 局部天向速度（m/s，向上为正），DXFD_VEL_FLAG_VELOCITY
    float    yaw_deg;         // 航向 setpoint（deg），DXFD_VEL_FLAG_YAW
    float    yaw_rate_deg_s;  // 偏航率 setpoint（deg/s），DXFD_VEL_FLAG_YAW_RATE
    float    ax_m_s2;         // 东向加速度前馈（m/s²），DXFD_VEL_FLAG_ACCEL
    float    ay_m_s2;         // 南向加速度前馈（m/s²），DXFD_VEL_FLAG_ACCEL
    float    az_m_s2;         // 天向加速度前馈（m/s²），DXFD_VEL_FLAG_ACCEL
} dxfd_velocity_command_t;
#pragma pack(pop)

static inline uint16_t dxfd_msg_velocity_command_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_velocity_command_t cmd, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &cmd, sizeof(dxfd_velocity_command_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_VELOCITY_COMMAND;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_VELOCITY_COMMAND_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_velocity_command_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    int64_t entity_id, uint8_t flags, uint16_t seq,
    float vx, float vy, float vz, float yaw_deg, float yaw_rate_deg_s,
    float ax, float ay, float az, uint16_t length = 0)
{
    dxfd_velocity_command_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.entity_id = entity_id;
    cmd.flags = flags;
    cmd.seq = seq;
    cmd.vx_m_s = vx;
    cmd.vy_m_s = vy;
    cmd.vz_m_s = vz;
    cmd.yaw_deg = yaw_deg;
    cmd.yaw_rate_deg_s = yaw_rate_deg_s;
    cmd.ax_m_s2 = ax;
    cmd.ay_m_s2 = ay;
    cmd.az_m_s2 = az;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &cmd, sizeof(dxfd_velocity_command_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_VELOCITY_COMMAND;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_VELOCITY_COMMAND_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_velocity_command_decode(const dxfd_message_t* msg, dxfd_velocity_command_t* cmd)
{
    int len = msg->length < DXFD_MSG_ID_VELOCITY_COMMAND_LEN ? msg->length : DXFD_MSG_ID_VELOCITY_COMMAND_LEN;
    memset(cmd, 0, sizeof(dxfd_velocity_command_t));
    memcpy(cmd, &msg->payload[0], len);
}

#endif //DXFD_MSG_VELOCITY_COMMAND_H
