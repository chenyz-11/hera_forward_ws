//
// Created by lewis on 2021/10/29.
//

#ifndef DXFD_MSG_SPAWN_MODEL_H
#define DXFD_MSG_SPAWN_MODEL_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_spawn_model_t {
   
    uint8_t   campID;
    uint8_t   squadID;
    char      modelType[64];
    uint32_t modelID;
    uint8_t   spawnType = 0;
    int32_t   locationLat;
    int32_t   locationLon;
    float   locationAlt;
    float     attitudeRoll;
    float     attitudePitch;
    float     attitudeYaw;
    float     scaleX;
    float     scaleY;
    float     scaleZ;
} dxfd_spawn_model_t;
#pragma pack(pop)

#define DXFD_MSG_ID_SPAWN_MODEL_LEN sizeof(dxfd_spawn_model_t)

static inline uint16_t dxfd_msg_spawn_model_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, uint8_t campID,
                                                                                        uint8_t squadID, char* modelType, uint32_t modelID, uint8_t spawnType, 
                                                                                        int32_t locationLat, int32_t locationLon, float locationAlt, 
                                                                                        float attitudeRoll, float attitudePitch, float attitudeYaw,
                                                                                        float scaleX, float scaleY, float scaleZ, uint16_t length = 0)
{
    dxfd_spawn_model_t packet;
    memset(&packet, 0, sizeof(dxfd_spawn_model_t));
    packet.squadID = squadID;
    memcpy(packet.modelType, modelType, 64);
    packet.campID = campID;
    packet.modelID = modelID;
    packet.spawnType = spawnType;
    packet.locationLat = locationLat;
    packet.locationLon = locationLon;
    packet.locationAlt = locationAlt;
    packet.attitudeRoll = attitudeRoll;
    packet.attitudePitch = attitudePitch;
    packet.attitudeYaw = attitudeYaw;
    packet.scaleX = scaleX;
    packet.scaleY = scaleY;
    packet.scaleZ = scaleZ;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SPAWN_MODEL_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SPAWN_MODEL;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SPAWN_MODEL_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_spawn_model_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                                                        dxfd_spawn_model_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SPAWN_MODEL_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SPAWN_MODEL;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SPAWN_MODEL_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_spawn_model_decode(const dxfd_message_t* msg, dxfd_spawn_model_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_SPAWN_MODEL_LEN ? msg->length : DXFD_MSG_ID_SPAWN_MODEL_LEN;
    memset(packet, 0, DXFD_MSG_ID_SPAWN_MODEL_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif //DXFD_MSG_SPAWN_MODEL_H

