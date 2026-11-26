#ifndef DXFD_MSG_FAULT_CMD_H
#define DXFD_MSG_FAULT_CMD_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_fault_cmd_t {
    uint32_t  modelID;
    uint8_t  fault_type;
    float    time;
    bool     isTime;
} dxfd_fault_cmd_t;
#pragma pack(pop)

#define DXFD_MSG_ID_FAULT_CMD_LEN sizeof(dxfd_fault_cmd_t)

static inline uint16_t dxfd_msg_fault_cmd_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, uint32_t modelID,
    uint8_t fault_type, float time, bool isTime, uint16_t length = 0)
{
    dxfd_fault_cmd_t packet;
    memset(&packet, 0, sizeof(dxfd_fault_cmd_t));
    packet.modelID = modelID;
    packet.fault_type = fault_type;
    packet.time = time;
    packet.isTime = isTime;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_FAULT_CMD_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_FAULT_CMD;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_FAULT_CMD_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_fault_cmd_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_fault_cmd_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_FAULT_CMD_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_FAULT_CMD;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_FAULT_CMD_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_fault_cmd_decode(const dxfd_message_t* msg, dxfd_fault_cmd_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_FAULT_CMD_LEN ? msg->length : DXFD_MSG_ID_FAULT_CMD_LEN;
    memset(packet, 0, DXFD_MSG_ID_FAULT_CMD_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif