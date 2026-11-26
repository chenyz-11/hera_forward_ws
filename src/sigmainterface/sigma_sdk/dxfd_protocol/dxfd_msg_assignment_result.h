//
// Created by fulei on 2026/1/13.
//

#ifndef SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ASSIGNMENT_RESULT_H
#define SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ASSIGNMENT_RESULT_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_assignment_result_t {
    int64_t target_id;
    uint8_t uav_count;
    uint8_t uav_id[20];
} dxfd_assignment_result_t;
#pragma pack(pop)

#define DXFD_MSG_ID_ASSIGNMENT_RESULT_LEN sizeof(dxfd_assignment_result_t)

static inline uint16_t dxfd_msg_assignment_result_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                       dxfd_assignment_result_t result, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &result, sizeof(dxfd_assignment_result_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ASSIGNMENT_RESULT;  // 需要定义对应的消息ID
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ASSIGNMENT_RESULT_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_assignment_result_decode(const dxfd_message_t* msg, dxfd_assignment_result_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_ASSIGNMENT_RESULT_LEN ? msg->length : DXFD_MSG_ID_ASSIGNMENT_RESULT_LEN;
    memset(packet, 0, sizeof(dxfd_assignment_result_t));
    memcpy(packet, &msg->payload[0], len);
}


#endif //SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ASSIGNMENT_RESULT_H
