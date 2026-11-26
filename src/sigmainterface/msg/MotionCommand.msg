# 运动指令：订阅后经 SDK SendMotionCommand 下发 UE
# 目标点为 ENU 直角坐标（米），节点内部换算为 LLA 整形微度 + 厘米高度
# entity_id 分量由节点经 getEntityID 重编码回协议 64 位 ID 后下发
# speed 单位 m/s（UE 侧当前暂忽略速度，仅走航点）
EntityId entity_id
float64 x
float64 y
float64 z
float64 speed
