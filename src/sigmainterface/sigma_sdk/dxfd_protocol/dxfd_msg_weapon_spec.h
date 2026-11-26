#ifndef DXFD_MSG_WEAPON_SPEC_H
#define DXFD_MSG_WEAPON_SPEC_H

#include "protocol.h"

#define DXFD_WEAPON_SPEC_NAME_LEN 32
#define DXFD_MSG_ID_WEAPON_SPEC_LEN sizeof(dxfd_weapon_spec_t)

#pragma pack(push,1)
// 弹药属性（单条，一包一个实体的一种弹药）
typedef struct __dxfd_weapon_spec_t {
    int64_t  entity_id;                         // 实体ID（携带该弹药的实体）
    uint32_t ammoID;                            // 弹药ID
    uint16_t totalCount;                        // 该类型弹药总携带量
    uint8_t  guidanceType;                      // GUIDANCE_TYPE（制导方式）
    uint8_t  reserve;
    float    range;                             // 射程 (m)
    float    damage;                            // 威力/毁伤量
    char     name[DXFD_WEAPON_SPEC_NAME_LEN];   // 弹药型号名
} dxfd_weapon_spec_t;
#pragma pack(pop)

static inline uint16_t dxfd_msg_weapon_spec_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_weapon_spec_t spec, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &spec, sizeof(dxfd_weapon_spec_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_WEAPON_SPEC;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_WEAPON_SPEC_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_weapon_spec_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    int64_t entity_id, uint32_t ammoID, uint16_t totalCount, uint8_t guidanceType,
    float range, float damage, const char* name, uint16_t length = 0)
{
    dxfd_weapon_spec_t spec;
    memset(&spec, 0, sizeof(spec));
    spec.entity_id = entity_id;
    spec.ammoID = ammoID;
    spec.totalCount = totalCount;
    spec.guidanceType = guidanceType;
    spec.range = range;
    spec.damage = damage;
    if (name) {
        int n = (int)strlen(name);
        if (n >= DXFD_WEAPON_SPEC_NAME_LEN) n = DXFD_WEAPON_SPEC_NAME_LEN - 1;
        memcpy(spec.name, name, n);
    }
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &spec, sizeof(dxfd_weapon_spec_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_WEAPON_SPEC;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_WEAPON_SPEC_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_weapon_spec_decode(const dxfd_message_t* msg, dxfd_weapon_spec_t* spec)
{
    int len = msg->length < DXFD_MSG_ID_WEAPON_SPEC_LEN ? msg->length : DXFD_MSG_ID_WEAPON_SPEC_LEN;
    memset(spec, 0, sizeof(dxfd_weapon_spec_t));
    memcpy(spec, &msg->payload[0], len);
}

#endif //DXFD_MSG_WEAPON_SPEC_H
