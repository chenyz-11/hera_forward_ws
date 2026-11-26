#pragma once
//
// Created by GuoJie on 2026/6/8.
//

#ifndef DXFDSIMULATION_DXFD_MSG_ALGO_HEARTBEAT_H
#define DXFDSIMULATION_DXFD_MSG_ALGO_HEARTBEAT_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_algo_heartbeat_t {
    uint8_t Identity;                 // 仿真系统/纯数字/半实物
} dxfd_algo_heartbeat_t;
#pragma pack(pop)

#define DXFD_MSG_ID_ALGO_HEARTBEAT_LEN sizeof(dxfd_algo_heartbeat_t)

static inline uint16_t dxfd_msg_algo_heartbeat_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_algo_heartbeat_t algo_heartbeat, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &algo_heartbeat, sizeof(dxfd_algo_heartbeat_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ALG_HEARTBEAT;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ALGO_HEARTBEAT_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_algo_heartbeat_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, uint8_t identity, uint16_t length = 0)
{
    dxfd_algo_heartbeat_t packet;
    memset(&packet, 0, sizeof(dxfd_algo_heartbeat_t));
    packet.Identity = identity;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, sizeof(dxfd_algo_heartbeat_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ALG_HEARTBEAT;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ALGO_HEARTBEAT_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_algo_heartbeat_decode(const dxfd_message_t* msg, dxfd_algo_heartbeat_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_ALGO_HEARTBEAT_LEN ? msg->length : DXFD_MSG_ID_ALGO_HEARTBEAT_LEN;
    memset(packet, 0, sizeof(dxfd_algo_heartbeat_t));
    memcpy(packet, &msg->payload[0], len);
}

#endif //DXFDSIMULATION_DXFD_MSG_ALGO_HEARTBEAT_H
