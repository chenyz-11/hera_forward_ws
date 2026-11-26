//
// Created by lewis on 2021/10/29.
//

#ifndef DXFD_MSG_HEARTBEAT_H
#define DXFD_MSG_HEARTBEAT_H

#include "protocol.h"


#pragma pack(push,1)
typedef struct __dxfd_heartbeat_t {
    uint8_t seq;
    uint16_t nodeCount; // equip node count
    uint16_t nodeList[SINGLE_LOWER_PC_MAX_NODE_COUNT];
    uint8_t runningStatus[SINGLE_LOWER_PC_MAX_NODE_COUNT]; 
    uint8_t reserve1;
    uint8_t reserve2;
    uint8_t reserve3;
} dxfd_heartbeat_t;
#pragma pack(pop)

#define DXFD_MSG_ID_HEARTBEAT_LEN sizeof(dxfd_heartbeat_t)

static inline uint16_t dxfd_msg_heartbeat_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, uint8_t seq,
                                                 uint16_t nodeCount, uint16_t* nodeList, uint8_t* runningStatus, 
                                                 uint8_t reserve1, uint8_t reserve2, uint8_t reserve3,uint16_t length = 0)
{
    dxfd_heartbeat_t packet;
    memset(&packet, 0, sizeof(dxfd_heartbeat_t));
    packet.seq = seq;
    packet.nodeCount = nodeCount;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;
    int cpySize = nodeCount > SINGLE_LOWER_PC_MAX_NODE_COUNT ? SINGLE_LOWER_PC_MAX_NODE_COUNT : nodeCount;
    memcpy(packet.runningStatus, runningStatus, SINGLE_LOWER_PC_MAX_NODE_COUNT);
    memcpy(packet.nodeList, nodeList, SINGLE_LOWER_PC_MAX_NODE_COUNT * 2);
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_HEARTBEAT_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_HEARTBEAT;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_HEARTBEAT_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_heartbeat_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, 
                                                 dxfd_heartbeat_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_HEARTBEAT_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_HEARTBEAT;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_HEARTBEAT_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_heartbeat_decode(const dxfd_message_t* msg, dxfd_heartbeat_t* heartbeat)
{
    int len = msg->length < DXFD_MSG_ID_HEARTBEAT_LEN? msg->length : DXFD_MSG_ID_HEARTBEAT_LEN;
    memset(heartbeat, 0, DXFD_MSG_ID_HEARTBEAT_LEN);
    memcpy(heartbeat, &msg->payload[0], len);
}

#endif //DXFD_MSG_HEARTBEAT_H
