#ifndef DXFD_MSG_ALGORITHM_CONTROL_CMD_H
#define DXFD_MSG_ALGORITHM_CONTROL_CMD_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_algorithm_control_cmd_t {
	uint8_t command;	//control algorithm start or stop
	uint8_t algorithmMode; //distributed or centralized
	uint8_t algorithmTask;	//Set algorithm execution tasks
	uint8_t autopilotMode; //set autopilot offboard or mission
	uint8_t autopilotArm;	//arm or disarm
    uint8_t autopilotType[SINGLE_LOWER_PC_MAX_NODE_COUNT];    //autopilot model param type
} dxfd_algorithm_control_cmd_t;
#pragma pack(pop)

#define DXFD_MSG_ID_ALGORITHM_CONTROL_CMD_LEN sizeof(dxfd_algorithm_control_cmd_t)

static inline uint16_t dxfd_msg_algorithm_control_cmd_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, uint8_t command, uint8_t algorithmMode, uint8_t algorithmTask, uint8_t autopilotMode, uint8_t autopilotArm, const uint8_t* autopilot_type, uint16_t length = 0)
{
    dxfd_algorithm_control_cmd_t packet;
    memset(&packet, 0, sizeof(dxfd_algorithm_control_cmd_t));
    packet.command = command;
    packet.algorithmMode = algorithmMode;
    packet.algorithmTask = algorithmTask;
    packet.autopilotMode = autopilotMode;
    packet.autopilotArm = autopilotArm;
    memcpy(packet.autopilotType, autopilot_type, SINGLE_LOWER_PC_MAX_NODE_COUNT);
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_ALGORITHM_CONTROL_CMD_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ALGORITHM_CONTROL_CMD;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ALGORITHM_CONTROL_CMD_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_algorithm_control_cmd_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_algorithm_control_cmd_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_ALGORITHM_CONTROL_CMD_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ALGORITHM_CONTROL_CMD;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ALGORITHM_CONTROL_CMD_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_algorithm_control_cmd_decode(const dxfd_message_t* msg, dxfd_algorithm_control_cmd_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_ALGORITHM_CONTROL_CMD_LEN ? msg->length : DXFD_MSG_ID_ALGORITHM_CONTROL_CMD_LEN;
    memset(packet, 0, DXFD_MSG_ID_ALGORITHM_CONTROL_CMD_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif
