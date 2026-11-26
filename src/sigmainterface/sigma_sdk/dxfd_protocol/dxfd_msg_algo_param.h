//
// Created by fulei on 2026/1/11.
//

#ifndef SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ALGO_PARAM_H
#define SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ALGO_PARAM_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_algo_param_t {
    float height;                 // 飞行高度
    float formation_dist;         // 编队间距
    uint8_t formation_size;       // 小队成员数量
    uint8_t area_column_count;         // 东西方向上，分配多少个小队
    uint8_t area_row_count;         // 南北方向上，分配多少个小队
    int32_t origin_lon;
    int32_t origin_lat;
    float    origin_alt;
    int32_t area_ne_lon;
    int32_t area_ne_lat;
    int32_t area_sw_lon;
    int32_t area_sw_lat;
    float area_dist;              // 生成的蛇形航线的间距
    uint8_t formation_type;         //编队队形
} dxfd_algo_param_t;
#pragma pack(pop)

#define DXFD_MSG_ID_ALGO_PARAM_LEN sizeof(dxfd_algo_param_t)

static inline uint16_t dxfd_msg_algo_param_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                  dxfd_algo_param_t algo_param, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &algo_param, sizeof(dxfd_algo_param_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ALG_PARAM;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ALGO_PARAM_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_algo_param_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                  float height, float formation_dist, uint8_t formation_size,
                                                  uint8_t area_column_count, uint8_t area_row_count, int32_t origin_lon, int32_t origin_lat, float origin_alt,
                                                   int32_t area_ne_lon,int32_t area_ne_lat,int32_t area_sw_lon,int32_t area_sw_lat,
                                                   float area_dist, uint8_t formation_type, uint16_t length = 0)
{
    dxfd_algo_param_t packet;
    memset(&packet, 0, sizeof(dxfd_algo_param_t));
    packet.height = height;
    packet.formation_dist = formation_dist;
    packet.formation_size = formation_size;
    packet.area_column_count = area_column_count;
    packet.area_row_count = area_row_count;
    packet.origin_lon = origin_lon;
    packet.origin_lat = origin_lat;
    packet.origin_alt = origin_alt;
    packet.area_ne_lon = area_ne_lon;
    packet.area_ne_lat = area_ne_lat;
    packet.area_sw_lon = area_sw_lon;
    packet.area_sw_lat = area_sw_lat;
    packet.area_dist = area_dist;
    packet.formation_type = formation_type;
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, sizeof(dxfd_algo_param_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_ALG_PARAM;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_ALGO_PARAM_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_algo_param_decode(const dxfd_message_t* msg, dxfd_algo_param_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_ALGO_PARAM_LEN ? msg->length : DXFD_MSG_ID_ALGO_PARAM_LEN;
    memset(packet, 0, sizeof(dxfd_algo_param_t));
    memcpy(packet, &msg->payload[0], len);
}

#endif //SWARM_ALGORITHM_COLLECTION_DXFD_MSG_ALGO_PARAM_H
