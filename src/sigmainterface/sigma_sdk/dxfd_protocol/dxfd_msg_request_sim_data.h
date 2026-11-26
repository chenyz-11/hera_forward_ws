//
// Created by lewis on 2021/10/29.
//

#ifndef DXFD_MSG_REQUEST_SIM_DATA_H
#define DXFD_MSG_REQUEST_SIM_DATA_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_request_sim_data_t {
    uint8_t queryMethod;
    char     queryContent[QUERY_CONTENT_LEN];
    uint8_t contentLength;
} dxfd_request_sim_data_t;
#pragma pack(pop)

#define DXFD_MSG_ID_REQUEST_SIM_DATA_LEN sizeof(dxfd_request_sim_data_t)

static inline uint16_t dxfd_msg_request_sim_data_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, uint8_t seq,
                                                                                             uint8_t queryMethod, char* queryContent, uint8_t contentLength, uint16_t length = 0)
{
    dxfd_request_sim_data_t packet;
    memset(&packet, 0, sizeof(dxfd_request_sim_data_t));
    packet.queryMethod= queryMethod;
    packet.contentLength = contentLength;
    uint8_t tmpLen = contentLength > QUERY_CONTENT_LEN ? QUERY_CONTENT_LEN : contentLength;
    memcpy(packet.queryContent, queryContent, tmpLen);
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_REQUEST_SIM_DATA_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_REQUEST_SIM_DATA;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_REQUEST_SIM_DATA_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_request_sim_data_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                                                             dxfd_request_sim_data_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_REQUEST_SIM_DATA_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_REQUEST_SIM_DATA;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_REQUEST_SIM_DATA_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_request_sim_data_decode(const dxfd_message_t* msg, dxfd_request_sim_data_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_REQUEST_SIM_DATA_LEN ? msg->length : DXFD_MSG_ID_REQUEST_SIM_DATA_LEN;
    memset(packet, 0, DXFD_MSG_ID_REQUEST_SIM_DATA_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif //DXFD_MSG_REQUEST_SIM_DATA_H
