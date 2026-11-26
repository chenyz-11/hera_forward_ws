# 实体标识：由协议 64 位 entityID 经 sigmaAssist.h parseEntityID 解码
# 编码规则：[model_type:16][camp_id:8][squad_id:8][model_id:32（实际 16 位有效）]
# model_type 取值见 SDK SigmaModelType 枚举（0=SULA89A 22=BTR82 25=M1A2 等）
uint16 model_type
uint8 camp_id
uint8 squad_id
uint32 model_id
