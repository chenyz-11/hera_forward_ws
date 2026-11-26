//
// Created by lewis on 2021/10/29.
//

#ifndef DXFDSIMULATION_DXFD_MSG_SENSOR_NOISE_H
#define DXFDSIMULATION_DXFD_MSG_SENSOR_NOISE_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_sensor_noise_t {
    uint8_t seq;
    uint8_t  sensorType;
    uint16_t channel_1;
    uint16_t channel_2;
    uint16_t channel_3;
    uint16_t channel_4;
    uint16_t channel_5;
    uint16_t channel_6;
} dxfd_sensor_noise_t;
#pragma pack(pop)

#define DXFD_MSG_ID_SENSOR_NOISE_LEN sizeof(dxfd_sensor_noise_t)

static inline uint16_t dxfd_msg_sensor_noise_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                    uint8_t seq, uint8_t sensorType, uint16_t channel_1, uint16_t channel_2,
                                                    uint16_t channel_3, uint16_t channel_4, uint16_t channel_5,
                                                    uint16_t channel_6, uint16_t length = 0)
{
    dxfd_sensor_noise_t packet;
    packet.seq = seq;
    packet.sensorType = sensorType;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.channel_4 = channel_4;
    packet.channel_5 = channel_5;
    packet.channel_6 = channel_6;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SENSOR_NOISE_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SENSOR_NOISE;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SENSOR_NOISE_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_sensor_noise_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                    dxfd_sensor_noise_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SENSOR_NOISE_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SENSOR_NOISE;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SENSOR_NOISE_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_sensor_noise_decode(const dxfd_message_t* msg, dxfd_sensor_noise_t* sensor_noise)
{
    int len = msg->length < DXFD_MSG_ID_SENSOR_NOISE_LEN ? msg->length : DXFD_MSG_ID_SENSOR_NOISE_LEN;
    memset(sensor_noise, 0, DXFD_MSG_ID_SENSOR_NOISE_LEN);
    memcpy(sensor_noise, &msg->payload[0], len);
}


#endif //DXFDSIMULATION_DXFD_MSG_SENSOR_NOISE_H
