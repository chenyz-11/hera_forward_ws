#ifndef DXFD_MSG_LOADOUT_QUERY_H
#define DXFD_MSG_LOADOUT_QUERY_H

#include "protocol.h"

#define DXFD_MSG_ID_LOADOUT_QUERY_LEN sizeof(dxfd_loadout_query_t)

#pragma pack(push,1)
// 载荷查询：请求某实体搭载的弹药清单（应答为多条 WEAPON_SPEC，一个弹药一条）
typedef struct __dxfd_loadout_query_t {
    int64_t  entityID;    // 要查询载荷的实体
} dxfd_loadout_query_t;
#pragma pack(pop)

static inline uint16_t dxfd_msg_loadout_query_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_loadout_query_t query, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &query, sizeof(dxfd_loadout_query_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_LOADOUT_QUERY;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_LOADOUT_QUERY_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_loadout_query_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    int64_t entityID, uint16_t length = 0)
{
    dxfd_loadout_query_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.entityID = entityID;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, sizeof(dxfd_loadout_query_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_LOADOUT_QUERY;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_LOADOUT_QUERY_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_loadout_query_decode(const dxfd_message_t* msg, dxfd_loadout_query_t* query)
{
    int len = msg->length < DXFD_MSG_ID_LOADOUT_QUERY_LEN ? msg->length : DXFD_MSG_ID_LOADOUT_QUERY_LEN;
    memset(query, 0, sizeof(dxfd_loadout_query_t));
    memcpy(query, &msg->payload[0], len);
}

#endif //DXFD_MSG_LOADOUT_QUERY_H
