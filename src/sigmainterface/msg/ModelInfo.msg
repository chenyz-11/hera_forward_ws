# 实体状态：MODEL_INFO 协议解码并换算为 ENU 直角坐标
# 位置/速度单位 米 / 米每秒，yaw 为 ENU 弧度（正东 0、逆时针向北转为正）
# entity_id 已由节点经 parseEntityID 解码为分量（见 EntityId.msg）
EntityId entity_id
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
float64 yaw
