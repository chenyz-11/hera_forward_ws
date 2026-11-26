//
// Created by lewis on 2021/10/29.
//

#ifndef DXFD_MSG_MULTI_RENDER_DATA_H
#define DXFD_MSG_MULTI_RENDER_DATA_H

#include "protocol.h"
#include "dxfd_msg_render_data.h"

#pragma pack(push,1)
typedef struct __dxfd_multi_render_data_t {
    uint32_t seq;
    uint8_t count;
    dxfd_render_data_t renderData[SINGLE_LOWER_PC_MAX_NODE_COUNT];
} dxfd_multi_render_data_t;
#pragma pack(pop)

#define DXFD_MSG_ID_MULTI_RENDER_DATA_LEN sizeof(dxfd_multi_render_data_t)

static inline uint16_t dxfd_msg_multi_render_data_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                 uint32_t seq, uint8_t count, dxfd_render_data_t* data, uint16_t length = 0)
{
    dxfd_multi_render_data_t packet;
    memset(&packet, 0, sizeof(dxfd_multi_render_data_t));
    packet.seq = seq;
    packet.count = count;
    int cpySize = count > SINGLE_LOWER_PC_MAX_NODE_COUNT ? SINGLE_LOWER_PC_MAX_NODE_COUNT : count;
    memcpy(packet.renderData, data, cpySize * DXFD_MSG_ID_MULTI_RENDER_DATA_LEN);
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_MULTI_RENDER_DATA_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_MULTI_RENDER_DATA;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_MULTI_RENDER_DATA_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_multi_render_data_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, 
                                                         dxfd_multi_render_data_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_MULTI_RENDER_DATA_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_MULTI_RENDER_DATA;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_MULTI_RENDER_DATA_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_multi_render_data_decode(const dxfd_message_t* msg, dxfd_multi_render_data_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_MULTI_RENDER_DATA_LEN? msg->length : DXFD_MSG_ID_MULTI_RENDER_DATA_LEN;
    memset(packet, 0, DXFD_MSG_ID_MULTI_RENDER_DATA_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif //DXFD_MSG_MULTI_RENDER_DATA_H
