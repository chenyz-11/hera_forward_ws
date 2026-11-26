//
// 参考轨迹下发协议（外部算法 → 车辆 SetReferenceTrajectory）
// 变长消息：头部 + count 个轨迹点。长轨迹用 seq + has_more 分片，
// UE 侧按 command 分发到 Set/Append/Clear。
//

#ifndef SWARM_ALGORITHM_COLLECTION_DXFD_MSG_REFERENCE_TRAJECTORY_H
#define SWARM_ALGORITHM_COLLECTION_DXFD_MSG_REFERENCE_TRAJECTORY_H

#include "protocol.h"

// 头部固定长度（字节）
#define DXFD_MSG_ID_REFERENCE_TRAJECTORY_HEADER_LEN sizeof(dxfd_reference_trajectory_t)
// 单个轨迹点长度（字节）
#define DXFD_TRAJ_POINT_LEN sizeof(dxfd_traj_point_t)
// 单包点上限（安全截断）
#define DXFD_REF_TRAJ_MAX_POINTS 200

// command 取值
enum DXFD_TRAJ_COMMAND {
    DXFD_TRAJ_CMD_SET    = 0,   // 全量替换（映射 SetReferenceTrajectory）
    DXFD_TRAJ_CMD_APPEND = 1,   // 追加（映射 AppendReferencePoint）
    DXFD_TRAJ_CMD_CLEAR  = 2,   // 清空（映射 ClearReferenceTrajectory）
};

// flags 位掩码：整条轨迹统一是否携带可选字段
enum DXFD_TRAJ_FLAG {
    DXFD_TRAJ_FLAG_YAW       = 1 << 0,   // yaw_deg 有效
    DXFD_TRAJ_FLAG_SPEED     = 1 << 1,   // speed_ms 有效
    DXFD_TRAJ_FLAG_TIME      = 1 << 2,   // time_s 有效
    DXFD_TRAJ_FLAG_CURVATURE = 1 << 3,   // curvature 有效
};

#pragma pack(push,1)
// 轨迹点。x/y/z 为相对 origin 的局部坐标（米），轴映射与系统内 spawn/render 的 ENU→world 一致
typedef struct __dxfd_traj_point_t {
    float x_cm;         // 局部东向（厘米）
    float y_cm;         // 局部北向（厘米）
    float z_cm;         // 高度（厘米）
    float yaw_deg;     // 航向（deg），DXFD_TRAJ_FLAG_YAW
    float speed_ms;    // 速度（m/s），DXFD_TRAJ_FLAG_SPEED
    float time_s;      // 相对下发时刻（s），DXFD_TRAJ_FLAG_TIME
    float curvature;   // 曲率（1/m），DXFD_TRAJ_FLAG_CURVATURE
} dxfd_traj_point_t;

// 消息头。轨迹点紧跟头部之后存于 payload
typedef struct __dxfd_reference_trajectory_t {
    int64_t entity_id;   // 目标车辆 EntityID
    uint8_t  command;    // DXFD_TRAJ_COMMAND
    uint8_t  flags;      // DXFD_TRAJ_FLAG 位掩码
    uint16_t seq;        // 分片序号（回绕）
    uint8_t  has_more;   // 1=后续还有分片，0=最后一片
    uint16_t count;      // 本包点数
} dxfd_reference_trajectory_t;
#pragma pack(pop)

static inline uint16_t dxfd_msg_reference_trajectory_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                            dxfd_reference_trajectory_t header, const dxfd_traj_point_t* points)
{
    if (header.count > DXFD_REF_TRAJ_MAX_POINTS) header.count = DXFD_REF_TRAJ_MAX_POINTS;
    uint16_t payload_len = DXFD_MSG_ID_REFERENCE_TRAJECTORY_HEADER_LEN
                         + (uint16_t)header.count * DXFD_TRAJ_POINT_LEN;

    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &header, DXFD_MSG_ID_REFERENCE_TRAJECTORY_HEADER_LEN);
    if (header.count > 0 && points)
        memcpy(_DXFD_PAYLOAD_NON_CONST(msg) + DXFD_MSG_ID_REFERENCE_TRAJECTORY_HEADER_LEN,
               points, (size_t)header.count * DXFD_TRAJ_POINT_LEN);

    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_REFERENCE_TRAJECTORY;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = payload_len;
    msg->checksum = calculateChecksum(msg);
    return msg->length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_reference_trajectory_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                            int64_t entity_id, uint8_t command, uint8_t flags,
                                                            uint16_t seq, uint8_t has_more,
                                                            const dxfd_traj_point_t* points, uint16_t count)
{
    dxfd_reference_trajectory_t header;
    memset(&header, 0, sizeof(dxfd_reference_trajectory_t));
    header.entity_id = entity_id;
    header.command = command;
    header.flags = flags;
    header.seq = seq;
    header.has_more = has_more;
    header.count = count;
    return dxfd_msg_reference_trajectory_encode(msg, upperID, lowerID, header, points);
}

// 返回实际解出的点数；points_out 需至少 DXFD_REF_TRAJ_MAX_POINTS 大小，可为 NULL（仅要头部时）
static inline uint16_t dxfd_msg_reference_trajectory_decode(const dxfd_message_t* msg,
                                                            dxfd_reference_trajectory_t* header,
                                                            dxfd_traj_point_t* points_out)
{
    uint16_t hdr_len = DXFD_MSG_ID_REFERENCE_TRAJECTORY_HEADER_LEN;
    memset(header, 0, sizeof(dxfd_reference_trajectory_t));

    if (msg->length < hdr_len)
        return 0;   // 包不完整，无有效点

    memcpy(header, &msg->payload[0], hdr_len);

    uint16_t avail = msg->length - hdr_len;
    uint16_t count = avail / DXFD_TRAJ_POINT_LEN;
    if (count > DXFD_REF_TRAJ_MAX_POINTS) count = DXFD_REF_TRAJ_MAX_POINTS;
    header->count = count;

    if (count > 0 && points_out)
        memcpy(points_out, &msg->payload[hdr_len], (size_t)count * DXFD_TRAJ_POINT_LEN);

    return count;
}

#endif //SWARM_ALGORITHM_COLLECTION_DXFD_MSG_REFERENCE_TRAJECTORY_H