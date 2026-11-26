#ifndef DXFD_MSG_MOTION_COMMAND_H
#define DXFD_MSG_MOTION_COMMAND_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_motion_command_t {
    int64_t entityID; 
    uint8_t  controlMode;
    float      speed;
    int32_t  locationLat; 
    int32_t  locationLon;
    float  locationAlt;
} dxfd_motion_command_t;
#pragma pack(pop)

#define DXFD_MSG_ID_MOTION_COMMAND_LEN sizeof(dxfd_motion_command_t)

static inline uint16_t dxfd_msg_motion_command_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, int64_t entityID, uint8_t controlMode, float speed,
                                                                                                int32_t locationLat, int32_t locationLon, float locationAlt, uint16_t length = 0)
{
    dxfd_motion_command_t packet;
    memset(&packet, 0, sizeof(dxfd_motion_command_t));
    packet.entityID = entityID;
    packet.controlMode = controlMode;
    packet.locationLat = locationLat;
    packet.locationLon = locationLon;
    packet.locationAlt = locationAlt;
    packet.speed = speed;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_MOTION_COMMAND_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_MOTION_COMMAND;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_MOTION_COMMAND_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_motion_command_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                                                                dxfd_motion_command_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_MOTION_COMMAND_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_MOTION_COMMAND;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_MOTION_COMMAND_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_motion_command_decode(const dxfd_message_t* msg, dxfd_motion_command_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_MOTION_COMMAND_LEN ? msg->length : DXFD_MSG_ID_MOTION_COMMAND_LEN;
    memset(packet, 0, DXFD_MSG_ID_MOTION_COMMAND_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif //DXFD_MSG_MOTION_COMMAND_H


