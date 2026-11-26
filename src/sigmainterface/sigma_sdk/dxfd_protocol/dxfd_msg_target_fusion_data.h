#ifndef DXFD_MSG_TARGET_FUSION_DATA_H
#define DXFD_MSG_TARGET_FUSION_DATA_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_target_fusion_data_t {
    int64_t selfEntityID;
    int64_t targetEntityID;
    int32_t ModelID;
    float   healthValue;
    int32_t   locationLat;
    int32_t   locationLon;
    float     locationAlt;
    float       attitudeRoll;
    float       attitudePitch;
    float       attitudeYaw;
    float       velocityX;
    float       velocityY;
    float       velocityZ;
    float       sizeX;
    float       sizeY;
    float       sizeZ;
} dxfd_target_fusion_data_t;
#pragma pack(pop)

#define DXFD_MSG_ID_TARGET_FUSION_DATA_LEN sizeof(dxfd_target_fusion_data_t)

static inline uint16_t dxfd_msg_target_fusion_data_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, int64_t selfEntityID, int64_t targetEntityID, int32_t ModelID,
    float healthValue, int32_t   locationLat, int32_t   locationLon, float   locationAlt,
    float attitudeRoll, float attitudePitch, float attitudeYaw,
    float velocityX, float velocityY, float velocityZ,
    float sizeX, float sizeY, float sizeZ, uint16_t length = 0)
{
    dxfd_target_fusion_data_t packet;
    memset(&packet, 0, sizeof(dxfd_target_fusion_data_t));
    packet.selfEntityID = selfEntityID;
    packet.targetEntityID = targetEntityID;
    packet.ModelID = ModelID;
    packet.healthValue = healthValue;
    packet.locationLat = locationLat;
    packet.locationLon = locationLon;
    packet.locationAlt = locationAlt;
    packet.attitudeRoll = attitudeRoll;
    packet.attitudePitch = attitudePitch;
    packet.attitudeYaw = attitudeYaw;
    packet.velocityX = velocityX;
    packet.velocityY = velocityY;
    packet.velocityZ = velocityZ;
    packet.sizeX = sizeX;
    packet.sizeY = sizeY;
    packet.sizeZ = sizeZ;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_TARGET_FUSION_DATA_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_TARGET_FUSION_DATA;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_TARGET_FUSION_DATA_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_target_fusion_data_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_target_fusion_data_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_TARGET_FUSION_DATA_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_TARGET_FUSION_DATA;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_TARGET_FUSION_DATA_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_target_fusion_data_decode(const dxfd_message_t* msg, dxfd_target_fusion_data_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_TARGET_FUSION_DATA_LEN ? msg->length : DXFD_MSG_ID_TARGET_FUSION_DATA_LEN;
    memset(packet, 0, DXFD_MSG_ID_TARGET_FUSION_DATA_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif
