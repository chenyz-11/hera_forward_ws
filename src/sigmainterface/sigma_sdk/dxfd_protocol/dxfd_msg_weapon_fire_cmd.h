#ifndef DXFD_MSG_WEAPON_FIRE_CMD_H
#define DXFD_MSG_WEAPON_FIRE_CMD_H

#include "protocol.h"

#define DXFD_MSG_ID_WEAPON_FIRE_CMD_LEN sizeof(dxfd_weapon_fire_cmd_t)

#pragma pack(push,1)
// 开火命令：谁(shooterID) 用什么(ammoID) 打什么(targetID 或坐标) 怎么打(fireMode/延时)
typedef struct __dxfd_weapon_fire_cmd_t {
    int64_t  shooterID;     // 开火平台（entityID）
    int64_t  targetID;      // 目标实体（=0 表示按坐标开火）
    uint32_t ammoID;        // 弹药型号编号（0=当前装填/默认弹种）
    uint8_t  fireMode;      // FIRE_MODE：单发/连发/齐射
    uint8_t  salvoCount;    // 齐射/连发数量（单发=1）
    uint16_t seq;           // 命令序号，供 ACK 回显
    uint16_t fireDelayMs;   // 延时开火（毫秒，0=立即）
    int32_t  targetLat;     // 目标坐标（targetID==0 时有效）*1e7
    int32_t  targetLon;     // *1e7
    float    targetAlt;     // 米
    float    leadAngle;     // 提前量（度，0=自动解算）
} dxfd_weapon_fire_cmd_t;
#pragma pack(pop)

static inline uint16_t dxfd_msg_weapon_fire_cmd_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_weapon_fire_cmd_t cmd, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &cmd, sizeof(dxfd_weapon_fire_cmd_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_WEAPON_FIRE_CMD;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_WEAPON_FIRE_CMD_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_weapon_fire_cmd_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    int64_t shooterID, int64_t targetID, uint32_t ammoID,
    uint8_t fireMode, uint8_t salvoCount, uint16_t seq, uint16_t fireDelayMs,
    int32_t targetLat, int32_t targetLon, float targetAlt, float leadAngle, uint16_t length = 0)
{
    dxfd_weapon_fire_cmd_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.shooterID = shooterID;
    packet.targetID = targetID;
    packet.ammoID = ammoID;
    packet.fireMode = fireMode;
    packet.salvoCount = salvoCount;
    packet.seq = seq;
    packet.fireDelayMs = fireDelayMs;
    packet.targetLat = targetLat;
    packet.targetLon = targetLon;
    packet.targetAlt = targetAlt;
    packet.leadAngle = leadAngle;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, sizeof(dxfd_weapon_fire_cmd_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_WEAPON_FIRE_CMD;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_WEAPON_FIRE_CMD_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_weapon_fire_cmd_decode(const dxfd_message_t* msg, dxfd_weapon_fire_cmd_t* cmd)
{
    int len = msg->length < DXFD_MSG_ID_WEAPON_FIRE_CMD_LEN ? msg->length : DXFD_MSG_ID_WEAPON_FIRE_CMD_LEN;
    memset(cmd, 0, sizeof(dxfd_weapon_fire_cmd_t));
    memcpy(cmd, &msg->payload[0], len);
}

#endif //DXFD_MSG_WEAPON_FIRE_CMD_H
