#ifndef DXFD_MSG_ORIGIN_GLOBAL_POSITION_H
#define DXFD_MSG_ORIGIN_GLOBAL_POSITION_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_origin_global_position_t {
    int32_t   locationLat;
    int32_t   locationLon;
    float     locationAlt;
} dxfd_origin_global_position_t;
#pragma pack(pop)

#define DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION_LEN sizeof(dxfd_origin_global_position_t)

static inline uint16_t dxfd_msg_origin_global_position_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, int32_t locationLat, int32_t locationLon, float locationAlt, uint16_t length = 0)
{
    dxfd_origin_global_position_t packet;
    memset(&packet, 0, sizeof(dxfd_origin_global_position_t));
    packet.locationLat = locationLat;
    packet.locationLon = locationLon;
    packet.locationAlt = locationAlt;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_origin_global_position_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_origin_global_position_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_origin_global_position_decode(const dxfd_message_t* msg, dxfd_origin_global_position_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION_LEN ? msg->length : DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION_LEN;
    memset(packet, 0, DXFD_MSG_ID_ORIGIN_GLOBAL_POSITION_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif
