# 六架 BigUAV 编队运动

入口是 `uav_formation_switch.launch.py`。实际执行“前进并形成宽编队 → 单列 → 恢复宽编队 → 保持宽编队再前进 → 持续悬停”，保留已有 Hungarian 分配与编队几何算法。当前 Windows 运行包已通过单机、双机和六机完整实测。

先用 Windows 桌面的“仅六机启动”建立原固定六机场景，然后运行：

```bash
source /home/cyz/hera_forward_ws/install/setup.bash
export ROS_DOMAIN_ID=183
ros2 launch hera_forward uav_formation_switch.launch.py
```

默认分段前进距离 50 m，宽编队间距 15 m，单列间距 20 m，最后保持宽编队前进 50 m。参考速度 0.6 m/s，水平指令上限 1.0 m/s，垂直指令上限 0.4 m/s，控制频率 20 Hz。全流程约 8 分钟，完成后继续保持位置和高度。

高度是相对平台 HomeOrigin 的 ENU 高度，默认 5 m；约束默认 2–8 m。目标高度必须严格位于上下限内，例如：

```bash
ros2 launch hera_forward uav_formation_switch.launch.py \
  flight_height_m:=5.5 min_height_m:=2.0 max_height_m:=8.0
```

结束任务时先请求受控降落：

```bash
ros2 service call /hera/uav/land std_srvs/srv/Trigger '{}'
```

程序确认接地后释放控制，让平台原有机制解除武装；看到控制台 `LANDED` 再按 Ctrl+C。任务自然完成时保持发包，不会因停止发送而自动掉落。直接关闭桥接或网络中断仍由平台原有 Failsafe 处理，未关闭该保护。

`/hera/uav/hold` 可中断当前轨迹并保持位置；`/hera/uav/start` 可在 READY/HOLD/COMPLETE 时从当前实测位置开始新任务。`/hera/uav/status` 发布阶段、目标、六机反馈和数据目录。六机状态缺失、间距不足或跟踪误差过大时，任务锁定在 SAFETY_HOLD，不自动续跑。

执行起点来自六机实时反馈；不会把模型传送到 YAML 坐标。`config` 中三份 BigUAV 初始配置已重新读取当前平台状态，将之前错误的约 0.05 m 高度记录修正为实际约 5 m；原水平布局保留。若要从原始布局重放，正常关闭 Windows 场景并重新使用桌面启动文件。

`detour_preview_only:=true` 保留原路径预览功能，不发送正式运动指令。旧 `sample_step_m` 仅用于预览采样；执行固定细采样并按反馈调整公共进度。执行模式保持 `flight_height_m`，旧 `fixed_altitude` 参数保留兼容。

本次测试数据：`/home/cyz/hera_swarm_ws/artifacts/sigma_uav_motion_fix_20261126/REPORT.md`。位置数据来自实际 Windows MODEL_INFO，并以原生模型坐标和 Armed/FlightMode 交叉核对；平台速度字段未作为实际运动证据。
