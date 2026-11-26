# 开火回执：UE 每收到一次 weapon_fire_command 自动回一帧，靠 seq 与命令对齐
# 发布在该开火平台的实体目录下；当前 UE 侧用原 projectileID 字段承载实体 ID
EntityId entity_id      # 开火平台实体 ID（UE 暂用 projectileID 字段代替）
uint16 seq              # 回显开火命令的 seq
uint8 result_code       # FIRE_RESULT_CODE：0=成功 1=无目标 2=无弹药 3=射程外 4=射界外 5=武器未就绪
uint32 ammo_id          # 实际生效的弹药编号
uint16 ammo_remain      # 本次消耗后的剩余量
