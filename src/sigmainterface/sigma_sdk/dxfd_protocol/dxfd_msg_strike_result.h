#ifndef DXFD_MSG_STRIKE_RESULT_H
#define DXFD_MSG_STRIKE_RESULT_H

#include "protocol.h"

#define DXFD_MSG_ID_STRIKE_RESULT_LEN sizeof(dxfd_strike_result_t)

#pragma pack(push,1)
// 打击结果：命中/脱靶/被拦截与毁伤量（与 target healthValue 联动）
typedef struct __dxfd_strike_result_t {
    int64_t  shooterID;     // 开火平台
    int64_t  targetID;      // 目标实体
    int64_t projectileID;   // 开火对象句柄（实例化弹药=弹药实体 entityID；机枪点射=burst 事件号）
    uint8_t  resultType;    // STRIKE_RESULT_TYPE
    uint8_t  reserve;
    uint16_t hitCount;      // 命中发数（导弹=1，机枪=实际命中发数）
    float    damageValue;   // 毁伤量
    int32_t  hitLat;        // 命中点 *1e7
    int32_t  hitLon;        // *1e7
    float    hitAlt;        // 米
} dxfd_strike_result_t;
#pragma pack(pop)

static inline uint16_t dxfd_msg_strike_result_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    dxfd_strike_result_t result, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &result, sizeof(dxfd_strike_result_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_STRIKE_RESULT;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_STRIKE_RESULT_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_strike_result_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
    int64_t shooterID, int64_t targetID, int64_t projectileID, uint8_t resultType, uint16_t hitCount,
    float damageValue, int32_t hitLat, int32_t hitLon, float hitAlt, uint16_t length = 0)
{
    dxfd_strike_result_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.shooterID = shooterID;
    packet.targetID = targetID;
    packet.projectileID = projectileID;
    packet.resultType = resultType;
    packet.hitCount = hitCount;
    packet.damageValue = damageValue;
    packet.hitLat = hitLat;
    packet.hitLon = hitLon;
    packet.hitAlt = hitAlt;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, sizeof(dxfd_strike_result_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_STRIKE_RESULT;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_STRIKE_RESULT_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_strike_result_decode(const dxfd_message_t* msg, dxfd_strike_result_t* result)
{
    int len = msg->length < DXFD_MSG_ID_STRIKE_RESULT_LEN ? msg->length : DXFD_MSG_ID_STRIKE_RESULT_LEN;
    memset(result, 0, sizeof(dxfd_strike_result_t));
    memcpy(result, &msg->payload[0], len);
}

#endif //DXFD_MSG_STRIKE_RESULT_H
