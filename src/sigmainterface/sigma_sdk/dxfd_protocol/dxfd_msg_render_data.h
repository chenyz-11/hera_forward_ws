//
// Created by lewis on 2021/10/29.
//

#ifndef DXFD_MSG_RENDER_DATA_H
#define DXFD_MSG_RENDER_DATA_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_render_data_t {
    uint16_t dynamicModelID;
    uint32_t seq;
    int32_t  lat;
    int32_t  lon;
    float    alt;
    float    airspeed;
    float    groundSpeed;
    float    roll;
    float    pitch;
    float    yaw;
    float    elevator;
    float    aileron;
    float    rudder;
    float    throttle;
} dxfd_render_data_t;
#pragma pack(pop)

#define DXFD_MSG_ID_RENDER_DATA_LEN sizeof(dxfd_render_data_t)

static inline uint16_t dxfd_msg_render_data_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                 uint16_t dynamicModelID, uint32_t seq, int32_t lat, int32_t lon, float alt,
                                                 float airspeed, float groundSpeed,
                                                 float roll, float pitch, float yaw,
                                                 float elevator, float aileron, float rudder, float throttle,
                                                 uint16_t length = 0)
{
    dxfd_render_data_t packet;
    packet.dynamicModelID = dynamicModelID;
    packet.seq = seq;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.airspeed = airspeed;
    packet.groundSpeed = groundSpeed;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.elevator = elevator;
    packet.aileron = aileron;
    packet.rudder = rudder;
    packet.throttle = throttle;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_RENDER_DATA_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_RENDER_DATA;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_RENDER_DATA_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_render_data_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, dxfd_render_data_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_RENDER_DATA_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_RENDER_DATA;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_RENDER_DATA_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_render_data_decode(const dxfd_message_t* msg, dxfd_render_data_t* renderData)
{
    int len = msg->length < DXFD_MSG_ID_RENDER_DATA_LEN? msg->length : DXFD_MSG_ID_RENDER_DATA_LEN;
    memset(renderData, 0, DXFD_MSG_ID_RENDER_DATA_LEN);
    memcpy(renderData, &msg->payload[0], len);
}

#endif //DXFD_MSG_RENDER_DATA_H
