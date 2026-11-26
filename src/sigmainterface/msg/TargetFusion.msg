# 目标信息：TARGET_FUSION 协议解码并换算为 ENU 直角坐标
# self_id 为探测方ID，target_id 为目标本体 ID（均已解码为分量，见 EntityId.msg）
# 单位与 ModelInfo 相同（米 / 米每秒 / ENU 弧度）
EntityId self_id
EntityId target_id
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
float64 yaw
float64 health_value
