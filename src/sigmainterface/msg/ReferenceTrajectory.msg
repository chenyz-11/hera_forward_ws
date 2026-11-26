# 参考轨迹下发：订阅后经 SDK SendReferenceTrajectory 下发 UE（>200 点自动分片 SET+APPEND）
# entity_id 仅作一致性校验（全零视为未填），目标实体以话题路径为准
# flags 位掩码：bit0 yaw_rad 有效 / bit1 speed_ms / bit2 time_s / bit3 curvature
EntityId entity_id
uint8 flags
ReferenceTrajectoryPoint[] points
