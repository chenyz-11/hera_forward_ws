//
// Created by lewis on 2021/10/29.
//

#ifndef DXFD_MSG_ENVIRONMENT_PARAMS_H
#define DXFD_MSG_ENVIRONMENT_PARAMS_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_environment_params_t {
    uint16_t dynamicModelID;
    uint8_t seq;
    uint8_t paramType;
    float aglAltitude;      //  m
    float terrainElevation; //   m
    float northWindSpeed;   //ned n m/s
    float eastWindSpeed;    // ned e  m/s
    float groundWindSpeed;  //  ned  d  m/s
    float northGustSpeed;   //  ned n  m/s
    float eastGustSpeed;    //  ned  e  m/s
    float groundGustSpeed;   //  ned  d  m/s
    float slTemperature;    //  0
    float slPressure;       
    uint8_t reserve1;       
    uint8_t reserve2;       
    uint8_t reserve3;       
} dxfd_environment_params_t;
#pragma pack(pop)

#define DXFD_MSG_ID_ENVIRONMENT_PARAMS_LEN sizeof(dxfd_environment_params_t)

static inline uint16_t dxfd_msg_environment_params_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                 uint16_t dynamicModelID, uint8_t seq, uint8_t paramType, float aglAltitude, float terrainElevation,
                                                 float northWindSpeed, float eastWindSpeed, float groundWindSpeed,
                                                 float northGustSpeed, float eastGustSpeed, float groundGustSpeed,
                                                 float slTemperature, float slPressure,
                                                 uint8_t reserve1, uint8_t reserve2, uint8_t reserve3, uint16_t length = 0)
{
    dxfd_environment_params_t packet;
    packet.seq = seq;
    packet.dynamicModelID = dynamicModelID;
    packet.paramType = paramType;
    packet.aglAltitude = aglAltitude;
    packet.terrainElevation = terrainElevation;
    packet.northWindSpeed = northWindSpeed;
    packet.eastWindSpeed = eastWindSpeed;
    packet.groundWindSpeed = groundWindSpeed;
    packet.northGustSpeed = northGustSpeed;
    packet.eastGustSpeed = eastGustSpeed;
    packet.groundGustSpeed = groundGustSpeed;
    packet.slTemperature = slTemperature;
    packet.slPressure = slPressure;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;

    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_ENVIRONMENT_PARAMS_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ENVIRONMENT_PARAMS;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ENVIRONMENT_PARAMS_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_environment_params_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                          dxfd_environment_params_t packet, int16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_ENVIRONMENT_PARAMS_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ENVIRONMENT_PARAMS;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ENVIRONMENT_PARAMS_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_environment_params_decode(const dxfd_message_t* msg, dxfd_environment_params_t* environment_params)
{
    int len = msg->length < DXFD_MSG_ID_ENVIRONMENT_PARAMS_LEN ? msg->length : DXFD_MSG_ID_ENVIRONMENT_PARAMS_LEN;
    memset(environment_params, 0, DXFD_MSG_ID_ENVIRONMENT_PARAMS_LEN);
    memcpy(environment_params, &msg->payload[0], len);
}

#endif //DXFD_MSG_ENVIRONMENT_PARAMS_H
