# sigma2ros\_bridge 功能自检命令

原作者：https://gitee.com/beijing-daxiang-airlines/sigmainterface

## 0. 启动

```bash
ros2 launch sigma2ros_bridge sigma2ros_bridge.launch.py
```

等价写法（可执行文件名是 `sigma2ros_bridge_node`，节点名是 `sigma2ros_bridge`）：

```bash
ros2 run sigma2ros_bridge sigma2ros_bridge_node --ros-args \
  --params-file $(ros2 pkg prefix sigma2ros_bridge)/share/sigma2ros_bridge/config/bridge_params.yaml
```

以下命令均以默认配置为例：`namespace: "sigma"`、`pull_contents: "0_0_BigUAV_1"`。
实体目录为 `/sigma/camp_0/squad_0/BigUAV_1`。

```bash
ros2 node list                                  # 应出现 /sigma2ros_bridge
ros2 service list | grep loadout                # 应出现 /sigma/loadout_query
ros2 topic list | grep camp_                    # 实体话题出现说明已开通
ros2 topic info /sigma/camp_0/squad_0/BigUAV_1/motion_command   # Subscription count 应为 1
```

> `pull_contents` 中的实体会在启动时预开通全部指令话题；未列出的实体要等首条 `model_info` 到达后才会开通。

## 1. 订阅 model\_info

```bash
ros2 topic echo /sigma/camp_0/squad_0/BigUAV_1/model_info
ros2 topic hz   /sigma/camp_0/squad_0/BigUAV_1/model_info
```

检查字段：`entity_id` 分量、`x/y/z`（ENU 米）、`vx/vy/vz`（m/s）、`yaw`（ENU 弧度）。

## 2. 订阅 target\_fusion

```bash
ros2 topic echo /sigma/camp_0/squad_0/BigUAV_1/target_fusion
```

该话题发布在**探测方**实体目录下，`self_id` 为探测方，`target_id` 为目标本体。

## 3. 发布 motion\_command

```bash
ros2 topic pub --once \
  /sigma/camp_0/squad_0/BigUAV_1/motion_command \
  sigma2ros_bridge/msg/MotionCommand \
  '{entity_id: {model_type: 0, camp_id: 0, squad_id: 0, model_id: 1},
    x: 100.0, y: 0.0, z: 50.0, speed: 10.0}'
```

`x/y/z` 为 ENU 目标点（米）；需要 home 已就绪（收到 ORIGIN 或 `home_source: cfg`）。

## 4. 发布 velocity\_command

```bash
ros2 topic pub --once \
  /sigma/camp_0/squad_0/BigUAV_1/velocity_command \
  sigma2ros_bridge/msg/VelocityCommand \
  '{entity_id: {model_type: 0, camp_id: 0, squad_id: 0, model_id: 1},
    flags: 3,
    vx: 10.0, vy: 0.0, vz: 0.0,
    yaw: 0.0, yaw_rate: 0.0,
    ax: 0.0, ay: 0.0, az: 0.0}'
```

`flags` 位掩码：1=速度有效、2=yaw 有效、4=yaw\_rate 有效、8=加速度前馈有效（`3` = 速度 + 航向）。
`vx/vy/vz` 为 ENU（东/北/天，m/s），`yaw` 为 ENU 弧度。

持续发送示例：

```bash
ros2 topic pub -r 10 \
  /sigma/camp_0/squad_0/BigUAV_1/velocity_command \
  sigma2ros_bridge/msg/VelocityCommand \
  '{entity_id: {model_type: 0, camp_id: 0, squad_id: 0, model_id: 1},
    flags: 3, vx: 10.0, vy: 0.0, vz: 0.0, yaw: 0.0, yaw_rate: 0.0,
    ax: 0.0, ay: 0.0, az: 0.0}'
```

## 5. 发布 reference\_trajectory（现成 5 点直线段）

```bash
ros2 topic pub --once \
  /sigma/camp_0/squad_0/BigUAV_1/reference_trajectory \
  sigma2ros_bridge/msg/ReferenceTrajectory \
  '{entity_id: {model_type: 0, camp_id: 0, squad_id: 0, model_id: 1},
    flags: 7,
    points: [
      {x: 0.0,  y: 0.0, z: 50.0, yaw: 0.0, speed: 5.0, time: 0.0, curvature: 0.0},
      {x: 10.0, y: 0.0, z: 50.0, yaw: 0.0, speed: 5.0, time: 2.0, curvature: 0.0},
      {x: 20.0, y: 0.0, z: 50.0, yaw: 0.0, speed: 5.0, time: 4.0, curvature: 0.0},
      {x: 30.0, y: 0.0, z: 50.0, yaw: 0.0, speed: 5.0, time: 6.0, curvature: 0.0},
      {x: 40.0, y: 0.0, z: 50.0, yaw: 0.0, speed: 5.0, time: 8.0, curvature: 0.0}
    ]}'
```

`flags` 位掩码：1=yaw、2=speed、4=time、8=curvature（`7` = yaw + speed + time）。
轨迹点为 ENU 米 / 弧度；`speed` m/s、`time` 秒、`curvature` 1/m。超过 200 点由 SDK 自动分片。

## 6. 发布 weapon\_fire\_command（按坐标开火）

```bash
ros2 topic pub --once \
  /sigma/camp_0/squad_0/BigUGV_1/weapon_fire_command \
  sigma2ros_bridge/msg/WeaponFireCommand \
  '{self_id: {model_type: 3, camp_id: 0, squad_id: 0, model_id: 1},
    target_id: {model_type: 8, camp_id: 1, squad_id: 0, model_id: 1},
    ammo_id: 1, fire_mode: 0, salvo_count: 1, seq: 1, fire_delay_ms: 0,
    x: 100.0, y: 0.0, z: 50.0, lead_angle: 0.0}'
```

`target_id` 全零 = 按坐标开火（`x/y/z` 为 ENU 目标点，需要 home 就绪）；`fire_mode`：0=单发、1=连发、2=齐射；`lead_angle` 弧度（0=自动解算）。

按目标实体开火（坐标字段忽略）：

```bash
ros2 topic pub --once \
  /sigma/camp_0/squad_0/BigUAV_1/weapon_fire_command \
  sigma2ros_bridge/msg/WeaponFireCommand \
  '{self_id: {model_type: 0, camp_id: 0, squad_id: 0, model_id: 1},
    target_id: {model_type: 0, camp_id: 0, squad_id: 0, model_id: 2},
    ammo_id: 0, fire_mode: 0, salvo_count: 1, seq: 2, fire_delay_ms: 0,
    x: 0.0, y: 0.0, z: 0.0, lead_angle: 0.0}'
```

## 7. 订阅 weapon\_fire\_ack

先执行第 6 节的开火命令，再订阅该开火平台的回执：

```bash
ros2 topic echo /sigma/camp_0/squad_0/BigUGV_1/weapon_fire_ack
```

`seq` 与下发的开火命令一致；`result_code`：0=成功、1=无目标、2=无弹药、3=射程外、4=射界外、5=武器未就绪；`ammo_remain` 为本次消耗后的剩余量。

## 8. 请求 loadout

```bash
ros2 service call /sigma/loadout_query \
  sigma2ros_bridge/srv/LoadoutQuery \
  '{entity_id: {model_type: 0, camp_id: 0, squad_id: 0, model_id: 1}}'
```

成功时 `success: true`，`loadout.ammos` 为该实体全部弹药；无应答时 `success: false`、`message: "timeout..."`。

## 9. 接口自查

```bash
ros2 interface show sigma2ros_bridge/msg/ModelInfo
ros2 interface show sigma2ros_bridge/msg/TargetFusion
ros2 interface show sigma2ros_bridge/msg/MotionCommand
ros2 interface show sigma2ros_bridge/msg/VelocityCommand
ros2 interface show sigma2ros_bridge/msg/ReferenceTrajectory
ros2 interface show sigma2ros_bridge/msg/WeaponFireCommand
ros2 interface show sigma2ros_bridge/msg/WeaponFireAck
ros2 interface show sigma2ros_bridge/msg/Loadout
ros2 interface show sigma2ros_bridge/srv/LoadoutQuery
```


### 当前 SigmaPI5 BigUAV 运行包兼容性（2026-11-26 实测）

- `VelocityCommand` 的线上消息号固定为 **38**，payload 为 43 字节。原先隐式枚举得到的 37 被当前 Windows DLL 忽略。编码布局及校验和由 `velocity_wire_contract` 回归测试检查。
- 当前 BigUAV（model_type=0）MODEL_INFO 的 altitude 实际按米返回，因此 `biguav_altitude_scale_m` 默认为 `1.0`。旧版真正按厘米返回时需显式设 `0.01`。ORIGIN、其他型号及发送侧坐标约定不因此改变；不要把此兼容处理推广到未验证型号。
- 无人机需要持续的速度/零速度消息。任务完成后应继续保持位置与高度；不能停止发送后仍假定平台会永久悬停。Windows 原有的失联 Failsafe 保留。
- 六机入口：`ros2 launch hera_forward uav_formation_switch.launch.py`。该入口使用原有宽编队→单列→宽编队几何和反馈速度控制，不依赖未验证的无人机目标点/整条轨迹执行通道。
