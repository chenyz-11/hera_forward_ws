//
// Created by lewis on 2021/10/29.
//

#ifndef DXFD_MSG_SET_MODEL_TAG_H
#define DXFD_MSG_SET_MODEL_TAG_H

#include "protocol.h"
#define TAG_CONTENT_LEN 128
#pragma pack(push,1)
typedef struct __dxfd_set_model_tag_t {
    int64_t entityID;
    char     tagContent[TAG_CONTENT_LEN];
    uint8_t contentLength;
} dxfd_set_model_tag_t;
#pragma pack(pop)

#define DXFD_MSG_ID_SET_MODEL_TAG_LEN sizeof(dxfd_set_model_tag_t)

static inline uint16_t dxfd_msg_set_model_tag_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID, int64_t entityID,
                                                                                        char* tagContent, uint8_t contentLength, uint16_t length = 0)
{
    dxfd_set_model_tag_t packet;
    memset(&packet, 0, sizeof(dxfd_set_model_tag_t));
    packet.entityID = entityID;
    packet.contentLength = contentLength > TAG_CONTENT_LEN ? TAG_CONTENT_LEN : contentLength;
    memcpy(packet.tagContent, tagContent, packet.contentLength);
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SET_MODEL_TAG_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SET_MODEL_TAG;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SET_MODEL_TAG_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_set_model_tag_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                                                        dxfd_set_model_tag_t packet, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, DXFD_MSG_ID_SET_MODEL_TAG_LEN);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_SET_MODEL_TAG;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_SET_MODEL_TAG_LEN;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_set_model_tag_decode(const dxfd_message_t* msg, dxfd_set_model_tag_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_SET_MODEL_TAG_LEN ? msg->length : DXFD_MSG_ID_SET_MODEL_TAG_LEN;
    memset(packet, 0, DXFD_MSG_ID_SET_MODEL_TAG_LEN);
    memcpy(packet, &msg->payload[0], len);
}

#endif //DXFD_MSG_SET_MODEL_TAG_H
