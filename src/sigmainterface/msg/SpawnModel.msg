# 生成指令：订阅后经 SDK SendSpawnModel 在 UE 场景生成/渲染模型
# 目标点为 ENU 直角坐标（米），yaw 为 ENU 弧度；节点内部换算 LLA 微度 + 厘米高度 + UE 左手系度
# roll/pitch 固定 0、scale 固定 1（与 CXX Transmitter::spawnModel 一致）
string model_name       # UE 模型类型名（如 "BTR82"）
int32 camp_id           # 生成实体的阵营/小队/模型号（与 motion_command 的 EntityId 命名空间一致）
int32 squad_id
int32 model_id
int32 spawn_type        # UE 生成方式（协议透传，与 CXX spawnModel 同语义）
float64 x
float64 y
float64 z
float64 yaw
