#ifndef DXFD_MSG_CAMERA_CONTROL_H
#define DXFD_MSG_CAMERA_CONTROL_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_camera_control_t {
    float roll;
    float pitch;
    float yaw;
} dxfd_camera_control_t;
#pragma pack(pop)

#define DXFD_MSG_ID_CAMERA_CONTROL_LEN sizeof(dxfd_camera_control_t)

static inline uint16_t dxfd_msg_camera_control_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, float roll,
    float pitch, float yaw, uint16_t length = 0)
{
    dxfd_camera_control_t packet;
    memset(&packet, 0, sizeof(dxfd_camera_control_t));
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_CAMERA_CONTROL_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_CAMERA_CONTROL;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_CAMERA_CONTROL_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_camera_control_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_camera_control_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_CAMERA_CONTROL_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_CAMERA_CONTROL;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_CAMERA_CONTROL_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_camera_control_decode(const dxfd_message_t* msg, dxfd_camera_control_t* heartbeat)
{
    int len = msg->length < DXFD_MSG_ID_CAMERA_CONTROL_LEN ? msg->length : DXFD_MSG_ID_CAMERA_CONTROL_LEN;
    memset(heartbeat, 0, DXFD_MSG_ID_CAMERA_CONTROL_LEN);
    memcpy(heartbeat, &msg->payload[0], len);
}

#endif //DXFD_MSG_HEARTBEAT_H
