#ifndef DXFD_MSG_WEAPON_FIRE_ACK_H
#define DXFD_MSG_WEAPON_FIRE_ACK_H

#include "protocol.h"

#define DXFD_MSG_ID_WEAPON_FIRE_ACK_LEN sizeof(dxfd_weapon_fire_ack_t)

#pragma pack(push,1)
// 开火回执：回显命令序号、返回结果码与弹药实例句柄
typedef struct __dxfd_weapon_fire_ack_t {
    uint16_t seq;           // 回显开火命令的 seq
    uint8_t  resultCode;    // FIRE_RESULT_CODE
    uint8_t  reserve;
    int64_t projectileID;   // 开火对象句柄（实例化弹药=弹药实体 entityID；机枪点射=burst 事件号，失败时为 0）
    uint32_t ammoID;        // 实际生效的弹药编号
    uint16_t ammoRemain;    // 本次消耗后的剩余量
} dxfd_weapon_fire_ack_t;
#pragma pack(pop)

static inline uint16_t dxfd_msg_weapon_fire_ack_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_weapon_fire_ack_t ack, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &ack, sizeof(dxfd_weapon_fire_ack_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_WEAPON_FIRE_ACK;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_WEAPON_FIRE_ACK_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_weapon_fire_ack_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    uint16_t seq, uint8_t resultCode, int64_t projectileID, uint32_t ammoID, uint16_t ammoRemain, uint16_t length = 0)
{
    dxfd_weapon_fire_ack_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.seq = seq;
    packet.resultCode = resultCode;
    packet.projectileID = projectileID;
    packet.ammoID = ammoID;
    packet.ammoRemain = ammoRemain;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, sizeof(dxfd_weapon_fire_ack_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_WEAPON_FIRE_ACK;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_WEAPON_FIRE_ACK_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_weapon_fire_ack_decode(const dxfd_message_t* msg, dxfd_weapon_fire_ack_t* ack)
{
    int len = msg->length < DXFD_MSG_ID_WEAPON_FIRE_ACK_LEN ? msg->length : DXFD_MSG_ID_WEAPON_FIRE_ACK_LEN;
    memset(ack, 0, sizeof(dxfd_weapon_fire_ack_t));
    memcpy(ack, &msg->payload[0], len);
}

#endif //DXFD_MSG_WEAPON_FIRE_ACK_H
