//
// Created by fulei on 2026/1/13.
//

#ifndef SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ALGO_WAYPOINTS_H
#define SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ALGO_WAYPOINTS_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_algo_waypoints_t {
    uint8_t id;
    uint8_t count;
    int32_t waypoints_lat[20];
    int32_t waypoints_lon[20];
    float waypoints_alt[20];
} dxfd_algo_waypoints_t;
#pragma pack(pop)

#define DXFD_MSG_ID_ALGO_WAYPOINTS_LEN sizeof(dxfd_algo_waypoints_t)

static inline uint16_t dxfd_msg_algo_waypoints_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                     dxfd_algo_waypoints_t waypoints, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &waypoints, sizeof(dxfd_algo_waypoints_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ALG_WAYPOINTS;  // 需要定义对应的消息ID
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ALGO_WAYPOINTS_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_algo_waypoints_decode(const dxfd_message_t* msg, dxfd_algo_waypoints_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_ALGO_WAYPOINTS_LEN ? msg->length : DXFD_MSG_ID_ALGO_WAYPOINTS_LEN;
    memset(packet, 0, sizeof(dxfd_algo_waypoints_t));
    memcpy(packet, &msg->payload[0], len);
}



#endif //SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ALGO_WAYPOINTS_H
