//
// Created by lewis on 2021/10/29.
//

#ifndef DXFD_MSG_TRIGGER_INFO_H
#define DXFD_MSG_TRIGGER_INFO_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_trigger_info_t {
    uint8_t seq;
    uint8_t useTimeSync;
} dxfd_trigger_info_t;
#pragma pack(pop)

#define DXFD_MSG_ID_TRIGGER_INFO_LEN sizeof(dxfd_trigger_info_t)

static inline uint16_t dxfd_msg_trigger_info_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, uint8_t seq,
                                                 uint16_t useTimeSync,uint16_t length = 0)
{
    dxfd_trigger_info_t packet;
    memset(&packet, 0, sizeof(dxfd_trigger_info_t));
    packet.seq = seq;
    packet.useTimeSync = useTimeSync;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_TRIGGER_INFO_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_TRIGGER_INFO;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_TRIGGER_INFO_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_trigger_info_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, 
                                                 dxfd_trigger_info_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_TRIGGER_INFO_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_TRIGGER_INFO;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_TRIGGER_INFO_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_trigger_info_decode(const dxfd_message_t* msg, dxfd_trigger_info_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_TRIGGER_INFO_LEN? msg->length : DXFD_MSG_ID_TRIGGER_INFO_LEN;
    memset(packet, 0, DXFD_MSG_ID_TRIGGER_INFO_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif //DXFD_MSG_TRIGGER_INFO_H
