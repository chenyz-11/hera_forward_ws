# 开火命令：订阅后经 SDK SendWeaponFireCmd 下发 UE
# 开火平台由话题路径决定（raw），msg 内 self_id 仅作一致性校验（全零视为未填）
# target_id 全零表示按坐标开火，此时 x/y/z（ENU 米）经 home 换算为 LLA 微度 + 米高度
# target_id 非零时 UE 以实体 ID 为准，坐标字段忽略
EntityId self_id
EntityId target_id
uint32 ammo_id          # 弹药型号编号（0=当前装填/默认弹种）
uint8 fire_mode         # FIRE_MODE：0=单发 1=连发 2=齐射
uint8 salvo_count       # 齐射/连发数量（单发=1）
uint16 seq              # 命令序号（用于与 ACK 关联）
uint16 fire_delay_ms    # 延时开火（毫秒，0=立即）
float64 x               # 目标点 ENU 东向（米），target_id 全零时有效
float64 y               # 目标点 ENU 北向（米）
float64 z               # 目标点 ENU 天向（米）
float32 lead_angle      # 提前量（弧度，0=自动解算）
