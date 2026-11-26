#ifndef DXFD_MSG_MODEL_INFO_H
#define DXFD_MSG_MODEL_INFO_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_model_info_t {
    int64_t entityID;
    uint8_t seq;
    int32_t   locationLat;
    int32_t   locationLon;
    float   locationAlt;
    float       attitudeRoll;
    float       attitudePitch;
    float       attitudeYaw;
    float       velocityX;
    float       velocityY;
    float       velocityZ;
    float       sizeX;
    float       sizeY;
    float       sizeZ;
    float       healthValue;   // 生命值
} dxfd_model_info_t;
#pragma pack(pop)

#define DXFD_MSG_ID_MODEL_INFO_LEN sizeof(dxfd_model_info_t)

static inline uint16_t dxfd_msg_model_info_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, int64_t entityID, uint8_t seq,
                                                                                int32_t   locationLat,  int32_t   locationLon, float   locationAlt,
                                                                                float attitudeRoll, float attitudePitch, float attitudeYaw,
                                                                                float velocityX, float velocityY, float velocityZ,
                                                                                float sizeX, float sizeY, float sizeZ, float healthValue, uint16_t length = 0)
{
    dxfd_model_info_t packet;
    memset(&packet, 0, sizeof(dxfd_model_info_t));
    packet.entityID = entityID;
    packet.seq = seq;
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
    packet.healthValue = healthValue;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_MODEL_INFO_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_MODEL_INFO;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_MODEL_INFO_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_model_info_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                                                    dxfd_model_info_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_MODEL_INFO_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_MODEL_INFO;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_MODEL_INFO_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_model_info_decode(const dxfd_message_t* msg, dxfd_model_info_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_MODEL_INFO_LEN ? msg->length : DXFD_MSG_ID_MODEL_INFO_LEN;
    memset(packet, 0, DXFD_MSG_ID_MODEL_INFO_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif //DXFD_MSG_MODEL_INFO_H
