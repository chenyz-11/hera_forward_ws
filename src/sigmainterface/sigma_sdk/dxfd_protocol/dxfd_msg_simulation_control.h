//
// Created by lewis on 2021/10/29.
//

#ifndef DXFDSIMULATION_DXFD_MSG_SIMULATION_CONTROL_H
#define DXFDSIMULATION_DXFD_MSG_SIMULATION_CONTROL_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_simulation_control_t {
    uint16_t dynamicModelID;
    uint8_t  seq;     
    uint8_t  controlType;     // DXFD_SIM_CONTROL_CMD_TYPE
    uint8_t  componentID;   // vehicle child software id
    uint8_t  simMode;       // DXFD_SIM_MODE
    uint8_t  simSpeed;
    uint8_t  renderDataOutRate;
    int32_t longitude;
    int32_t latitude;
    float     altitude;
    float    roll;
    float    pitch;
    float    yaw;
    uint8_t  nameLength;
    char     modelName[64];
    uint64_t algorithmIP;   // algor ip address
    uint16_t algorithmPort;      //algor port
} dxfd_simulation_control_t;
#pragma pack(pop)

#define DXFD_MSG_ID_SIMULATION_CONTROL_LEN sizeof(dxfd_simulation_control_t)

static inline uint16_t dxfd_msg_simulation_control_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                           uint16_t dynamicModelID, uint8_t seq, uint8_t controlType,
                                                          uint8_t componentID, uint8_t simMode,  uint8_t simSpeed, 
                                                          uint8_t renderDataOutRate, int32_t longitude, int32_t latitude, float altitude,
                                                          float roll, float pitch, float yaw, uint8_t nameLength, char* modelName, uint64_t algorithmIP, uint16_t algorithmPort,
                                                          uint16_t length = 0)
{
    dxfd_simulation_control_t packet;
    packet.dynamicModelID = dynamicModelID;
    packet.seq = seq;
    packet.componentID = componentID;
    packet.simMode = simMode;
    packet.controlType = controlType;
    packet.simSpeed = simSpeed;
    packet.renderDataOutRate = renderDataOutRate;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.roll = roll;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.nameLength = nameLength;
    uint8_t tmpLength = nameLength > sizeof(packet.modelName) ? sizeof(packet.modelName) : nameLength;
    memcpy(packet.modelName, modelName, tmpLength);
    packet.algorithmIP = algorithmIP;
    packet, algorithmPort = algorithmPort;
    length = DXFD_MSG_ID_SIMULATION_CONTROL_LEN - sizeof(packet.modelName) + tmpLength;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SIMULATION_CONTROL_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SIMULATION_CONTROL;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SIMULATION_CONTROL_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_simulation_control_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                          dxfd_simulation_control_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SIMULATION_CONTROL_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SIMULATION_CONTROL;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SIMULATION_CONTROL_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_simulation_control_decode(const dxfd_message_t* msg,
                                                      dxfd_simulation_control_t* simulation_control)
{
    int len = msg->length < DXFD_MSG_ID_SIMULATION_CONTROL_LEN ? msg->length : DXFD_MSG_ID_SIMULATION_CONTROL_LEN;
    memset(simulation_control, 0, DXFD_MSG_ID_SIMULATION_CONTROL_LEN);
    memcpy(simulation_control, &msg->payload[0], len);
}

#endif //DXFDSIMULATION_DXFD_MSG_SIMULATION_CONTROL_H
