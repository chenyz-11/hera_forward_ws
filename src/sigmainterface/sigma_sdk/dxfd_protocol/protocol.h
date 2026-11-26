//
// Created by lewis on 2021/10/29.
//

#ifndef DXFDSIMULATION_PROTOCOL_H
#define DXFDSIMULATION_PROTOCOL_H

#include "dxfd_msg_id.h"
#include <stdbool.h>
#include <stdint.h>
#include <cstring>

#define DXFD_PROTOCOL_HEAD 0xDF
#define DXFD_PROTOCOL_LEN 8
#define DXFD_MAX_PAYLOAD_LEN 65535 - DXFD_PROTOCOL_LEN
#define DXFD_PACKET_MAX_LEN 65535
#define CONTROLLER_ID 255

#pragma pack(push,1)
typedef struct __dxfd_message_t {
    uint8_t  head = 0;
    uint8_t  upperID = 0;
    uint16_t lowerID = 0;
    uint8_t  msgID = 0;
    uint16_t length =0;
    uint8_t  payload[DXFD_MAX_PAYLOAD_LEN];
    uint8_t  checksum;
} dxfd_message_t;
#pragma pack(pop)

#define _DXFD_PAYLOAD(msg) ((const char *)(&((msg)->payload[0])))
#define _DXFD_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload[0])))

typedef enum {
    DXFD_MSG_PARSE_STATE_UNINIT = 0,
    DXFD_MSG_PARSE_STATE_GOT_HEAD,
    DXFD_MSG_PARSE_STATE_GOT_LENGTH_1,
    DXFD_MSG_PARSE_STATE_GOT_LENGTH_2,
    DXFD_MSG_PARSE_STATE_GOT_CONTROLLER_ID,
    DXFD_MSG_PARSE_STATE_GOT_SIMULATOR_ID_1,
    DXFD_MSG_PARSE_STATE_GOT_SIMULATOR_ID_2,
    DXFD_MSG_PARSE_STATE_GOT_MSGID,
    DXFD_MSG_PARSE_STATE_GOT_PAYLOAD,
    DXFD_MSG_PARSE_STATE_GOT_CHECKSUM,
} dxfd_msg_parse_state_t; ///< The state machine for the comm parser

typedef struct __dxfd_message_status_t {
    dxfd_msg_parse_state_t  status = DXFD_MSG_PARSE_STATE_UNINIT;
    uint16_t recvPayloadSize = 0;
} dxfd_message_status_t;

static inline uint8_t calculateChecksum(dxfd_message_t* msg)
{
    uint8_t checksum = 0;
    int i = 0;
    while(i < msg->length)
    {
        checksum += msg->payload[i];
        i++;
    }
    return checksum;
}

static inline uint16_t dxfd_msg_to_send_buffer(const dxfd_message_t* msg, uint8_t* buf)
{
    uint16_t length = msg->length;
    buf[0] = msg->head;
    buf[1] = msg->upperID;
    buf[2] = msg->lowerID & 0xff;
    buf[3] = (msg->lowerID >> 8) & 0xff;
    buf[4] = msg->msgID;
    buf[5] = msg->length & 0xff;
    buf[6] = (msg->length >> 8) & 0xff;
    memcpy(&buf[7], _DXFD_PAYLOAD(msg), length);
    buf[length + DXFD_PROTOCOL_LEN - 1] = msg->checksum;
    return DXFD_PROTOCOL_LEN + msg->length;
}

// TODO 如何判断新的一包数据要解包，除了判断是否为新定义的msg

static inline uint8_t dxfd_msg_parse_char(uint8_t* buf, dxfd_message_t* msg, dxfd_message_status_t* s)
{
    uint8_t c = *buf;
    if (msg->head == 0) s->status = DXFD_MSG_PARSE_STATE_UNINIT;
    switch (s->status) {
        case DXFD_MSG_PARSE_STATE_UNINIT:
            if(c == 0xDF)
            {
                s->status = DXFD_MSG_PARSE_STATE_GOT_HEAD;
                msg->head = c;
            }
            break;
        case DXFD_MSG_PARSE_STATE_GOT_HEAD:
            s->status = DXFD_MSG_PARSE_STATE_GOT_CONTROLLER_ID;
            msg->upperID = c;
            break;
        case DXFD_MSG_PARSE_STATE_GOT_CONTROLLER_ID:
            s->status = DXFD_MSG_PARSE_STATE_GOT_SIMULATOR_ID_1;
            msg->lowerID = c;
            break;
        case DXFD_MSG_PARSE_STATE_GOT_SIMULATOR_ID_1:
            s->status = DXFD_MSG_PARSE_STATE_GOT_SIMULATOR_ID_2;
            msg->lowerID = ((uint16_t)c << 8) + msg->lowerID;
            break;
        case DXFD_MSG_PARSE_STATE_GOT_SIMULATOR_ID_2:
            s->status = DXFD_MSG_PARSE_STATE_GOT_MSGID;
            msg->msgID = c;
            break;
        case DXFD_MSG_PARSE_STATE_GOT_MSGID:
            s->status = DXFD_MSG_PARSE_STATE_GOT_LENGTH_1;
            msg->length = c;
            break;
        case DXFD_MSG_PARSE_STATE_GOT_LENGTH_1:
            s->status = DXFD_MSG_PARSE_STATE_GOT_LENGTH_2;
            msg->length = ((uint16_t)c << 8) + msg->length;
            break;
        case DXFD_MSG_PARSE_STATE_GOT_LENGTH_2:
            s->status = DXFD_MSG_PARSE_STATE_GOT_PAYLOAD;
            msg->payload[0] = c;
            s->recvPayloadSize = 1;
            break;
        case DXFD_MSG_PARSE_STATE_GOT_PAYLOAD:
            if (s->recvPayloadSize == msg->length)
            {
                if(c != calculateChecksum(msg))
                {
                    s->status = DXFD_MSG_PARSE_STATE_UNINIT;
                } else {
                    s->status = DXFD_MSG_PARSE_STATE_UNINIT;
                    s->recvPayloadSize = 0;
                    msg->checksum = c;
                    return 1;
                }
            } else {
                msg->payload[s->recvPayloadSize] = c;
                s->recvPayloadSize++;
            }
            break;
        default:
            break;
    }
    return 0;
}

#endif //DXFDSIMULATION_PROTOCOL_H
