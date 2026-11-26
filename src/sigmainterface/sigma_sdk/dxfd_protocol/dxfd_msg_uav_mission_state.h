//
// Created by fulei on 2026/1/12.
//

#ifndef SWARM_ALGORITHM_COLLECTION_DXFD_MSG_UAV_MISSION_STATE_H
#define SWARM_ALGORITHM_COLLECTION_DXFD_MSG_UAV_MISSION_STATE_H

#include "protocol.h"

#pragma pack(push,1)
typedef struct __dxfd_uav_mission_state_t {
    uint8_t count;
    uint8_t uav_id_vec[200];          // 飞机 id 列表
    uint8_t uav_state_vec[200];       // 0、1、2、3、4 分别是无状态、起飞、巡航、搜索、打击
} dxfd_uav_mission_state_t;
#pragma pack(pop)

#define DXFD_MSG_ID_UAV_MISSION_STATE_LEN sizeof(dxfd_uav_mission_state_t)

/**
 * @brief 将 UAV 任务状态数据编码到消息中（结构体参数版本）
 * @param msg 消息指针
 * @param upperID 上位机 ID
 * @param lowerID 下位机 ID
 * @param uav_mission_state UAV 任务状态结构体
 * @param length 消息长度，默认为 0 表示自动计算
 * @return 返回整个消息包的长度
 */
static inline uint16_t dxfd_msg_uav_mission_state_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                         const dxfd_uav_mission_state_t &uav_mission_state, uint16_t length = 0)
{
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &uav_mission_state, sizeof(dxfd_uav_mission_state_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_UAV_MISSION_STATE;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_UAV_MISSION_STATE_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

/**
 * @brief 将 UAV 任务状态数据编码到消息中（逐个参数版本）
 * @param msg 消息指针
 * @param upperID 上位机 ID
 * @param lowerID 下位机 ID
 * @param count 无人机数量
 * @param uav_id_vec 无人机 ID 数组
 * @param uav_state_vec 无人机状态数组
 * @param length 消息长度，默认为 0 表示自动计算
 * @return 返回整个消息包的长度
 */
static inline uint16_t dxfd_msg_uav_mission_state_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                         uint8_t count, const uint8_t* uav_id_vec, const uint8_t* uav_state_vec, uint16_t length = 0)
{
    dxfd_uav_mission_state_t packet;
    memset(&packet, 0, sizeof(dxfd_uav_mission_state_t));

    // 设置无人机数量，最多 200 个
    packet.count = count > 200 ? 200 : count;

    // 复制无人机 ID 数据，最多 200 个
    if (uav_id_vec != NULL) {
        for (int i = 0; i < packet.count; i++) {
            packet.uav_id_vec[i] = uav_id_vec[i];
        }
    }

    // 复制无人机状态数据，最多 200 个
    if (uav_state_vec != NULL) {
        for (int i = 0; i < packet.count; i++) {
            packet.uav_state_vec[i] = uav_state_vec[i];
        }
    }

    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, sizeof(dxfd_uav_mission_state_t));
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_UAV_MISSION_STATE;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length > 0 ? length : DXFD_MSG_ID_UAV_MISSION_STATE_LEN;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

/**
 * @brief 从消息中解码 UAV 任务状态数据
 * @param msg 消息指针
 * @param packet 输出的 UAV 任务状态结构体
 */
static inline void dxfd_msg_uav_mission_state_decode(const dxfd_message_t* msg, dxfd_uav_mission_state_t* packet)
{
    int len = msg->length < DXFD_MSG_ID_UAV_MISSION_STATE_LEN ? msg->length : DXFD_MSG_ID_UAV_MISSION_STATE_LEN;
    memset(packet, 0, sizeof(dxfd_uav_mission_state_t));
    memcpy(packet, &msg->payload[0], len);
}


#endif //SWARM_ALGORITHM_COLLECTION_DXFD_MSG_UAV_MISSION_STATE_H
