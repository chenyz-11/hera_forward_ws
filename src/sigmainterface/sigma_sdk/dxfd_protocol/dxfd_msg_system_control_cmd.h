#ifndef DXFD_MSG_SYSTEM_CONTROL_CMD_H
#define DXFD_MSG_SYSTEM_CONTROL_CMD_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_system_control_cmd_t {
    uint8_t command;	//control simulation system start or stop, 0:None,1:start,2:stop,3:restart
    uint8_t is_all_vehicle_offboard;
    uint8_t is_all_vehicle_armed;
    uint8_t is_algo_running;
} dxfd_system_control_cmd_t;
#pragma pack(pop)

#define DXFD_MSG_ID_SYSTEM_CONTROL_CMD_LEN sizeof(dxfd_system_control_cmd_t)

static inline uint16_t dxfd_msg_system_control_cmd_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                          uint8_t command, uint8_t is_all_vehicle_offboard, uint8_t is_all_vehicle_armed, uint8_t is_algo_running, uint16_t length = 0)
{
    dxfd_system_control_cmd_t packet;
    memset(&packet, 0, sizeof(dxfd_system_control_cmd_t));
    packet.command = command;
    packet.is_all_vehicle_offboard = is_all_vehicle_offboard;
    packet.is_all_vehicle_armed = is_all_vehicle_armed;
    packet.is_algo_running = is_algo_running;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SYSTEM_CONTROL_CMD_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SYSTEM_CONTROL_CMD;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SYSTEM_CONTROL_CMD_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_system_control_cmd_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                          dxfd_system_control_cmd_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SYSTEM_CONTROL_CMD_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SYSTEM_CONTROL_CMD;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SYSTEM_CONTROL_CMD_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_system_control_cmd_decode(const dxfd_message_t* msg, dxfd_system_control_cmd_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_SYSTEM_CONTROL_CMD_LEN ? msg->length : DXFD_MSG_ID_SYSTEM_CONTROL_CMD_LEN;
    memset(packet, 0, DXFD_MSG_ID_SYSTEM_CONTROL_CMD_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif
