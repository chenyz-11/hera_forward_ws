//
// Created by lewis on 2021/10/29.
//

#ifndef DXFDSIMULATION_DXFD_MSG_VEHICLE_INITIAL_CONDITION_H
#define DXFDSIMULATION_DXFD_MSG_VEHICLE_INITIAL_CONDITION_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_vehicle_initial_condition_t {
    uint16_t dynamicModelID; 
    uint8_t seq;
    uint8_t paramType;
    int32_t longitude; // *1e7
    int32_t latitude;  // *1e7
    float    altitude;  //  m
    float    roll;      // ° ±90
    float    pitch;     // ° ±90
    float    yaw;       // ° 0-360  // TODO: to test
    float    xSpeed;    // m/s, ned
    float    ySpeed;    // m/s, ned
    float    zSpeed;    // m/s, ned
    float    climbRate; // m/s
    uint8_t  reserve1;
    uint8_t  reserve2;
    uint8_t  reserve3;
} dxfd_vehicle_initial_condition_t;
#pragma pack(pop)

#define DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION_LEN sizeof(dxfd_vehicle_initial_condition_t)

static inline uint16_t dxfd_msg_vehicle_initial_condition_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                 uint16_t dynamicModelID, uint8_t seq, uint8_t paramType,
                                                 int32_t longitude, int32_t latitude, float altitude,
                                                 float roll, float pitch, float yaw,
                                                 float xSpeed, float ySpeed, float zSpeed, float climbRate,
                                                 uint8_t reserve1, uint8_t reserve2, uint8_t reserve3, uint16_t length = 0)
{
    dxfd_vehicle_initial_condition_t packet;
    packet.paramType = paramType;
    packet.dynamicModelID = dynamicModelID;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.xSpeed = xSpeed;
    packet.ySpeed = ySpeed;
    packet.zSpeed = zSpeed;
    packet.climbRate = climbRate;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_vehicle_initial_condition_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                                dxfd_vehicle_initial_condition_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_vehicle_initial_condition_decode(const dxfd_message_t* msg, dxfd_vehicle_initial_condition_t* vehicleInitialCondition)
{
    int len = msg->length < DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION_LEN? msg->length : DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION_LEN;
    memset(vehicleInitialCondition, 0, DXFD_MSG_ID_VEHICLE_INITIAL_CONDITION_LEN);
    memcpy(vehicleInitialCondition, &msg->payload[0], len);
}

#endif //DXFDSIMULATION_DXFD_MSG_VEHICLE_INITIAL_CONDITION_H
