#ifndef DXFD_MSG_SYSTEM_CONTROL_CMD_ACK_H
#define DXFD_MSG_SYSTEM_CONTROL_CMD_ACK_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_system_control_cmd_ack_t {
    uint8_t recvstatus;	//ack system control command, 0:None,1:recv
} dxfd_system_control_cmd_ack_t;
#pragma pack(pop)

#define DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK_LEN sizeof(dxfd_system_control_cmd_ack_t)

static inline uint16_t dxfd_msg_system_control_cmd_ack_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    uint8_t status, uint16_t length = 0)
{
    dxfd_system_control_cmd_ack_t packet;
    memset(&packet, 0, sizeof(dxfd_system_control_cmd_ack_t));
    packet.recvstatus = status;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_system_control_cmd_ack_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_system_control_cmd_ack_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_system_control_cmd_ack_decode(const dxfd_message_t* msg, dxfd_system_control_cmd_ack_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK_LEN ? msg->length : DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK_LEN;
    memset(packet, 0, DXFD_MSG_ID_SYSTEM_CONTROL_CMD_ACK_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif
