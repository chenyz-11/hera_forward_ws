# 单种弹药属性：WEAPON_SPEC 协议解码
# 一次 loadout_query 会返回该实体搭载的多条 WeaponSpec，由节点聚合进 Loadout
uint32 ammo_id          # 弹药型号编号
uint16 total_count      # 该类型弹药总携带量
uint8 guidance_type     # GUIDANCE_TYPE（制导方式）
float64 range           # 射程（米）
float64 damage          # 威力/毁伤量
string name             # 弹药型号名
