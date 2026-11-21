# UE 虚拟云台：第一阶段

UE 每个编辑器 tick 积分两轴云台，图像采集、D-FINE 和网页刷新均不充当云台时钟。
`tools/ue/virtual_gimbal.py` 实现 pan/pitch 角度、角速度、角加速度、指令延迟、限位和失锁制动。
固定 90° 水平视场角用于本阶段；变焦与飞行跟随不在第一阶段验收范围。

## 运行

在 ROS 环境中安装更新后的 UE 服务，重启 UE 编辑器：

```bash
rosrun sar_yolo_detector install_ue_capture.py --project /your/UE/project --install-startup
roslaunch sar_yolo_detector ue_dfine_v3_live.launch \
  ue_project:=/your/UE/project pose_source:=kinematic seconds:=180
```

正式 PX4 仿真使用 `pose_source:=px4`，先启动对应 SITL/MAVROS。
网页默认 `http://127.0.0.1:8787`，选择 Smart/Fusion 的目标或使用手动 ROI。
取消选择会经过制动后保持当前云台角度。设置 `virtual_gimbal:=false` 可回到固定姿态采集。
参数 `gimbal_config:=/your/config.json` 指定轴约束。

默认 `config/ue/virtual_gimbal.json`：pan ±160°、pitch −89° 至 +45°、
最大角速度 30°/s、角加速度 90°/s²、指令延迟 80 ms、测量有效期 1.2 s。
这些是可配置的仿真参数；原 Gazebo 的 6°/s 配置可在同一测试中作为较慢执行器对照。

## 数据契约

相机桥保存实际捕获姿态/视场角/时间戳。`SmartTrackerState` 中的选定中心像素只有同时
满足当前实测、可用于控制、非预测/非陈旧/非暂定、时间新鲜、图像 frame_id 和采集记录
完全对应时，才能换算世界视线。重复/乱序帧不刷新视线。使用采集姿态换算，避免将旧图像
的误差直接加到当前云台角度上。此规则继承已有 Gazebo 网页云台实现的时间语义。

UE 根据世界视线与机体航向计算机械 pan，执行延迟和运动约束，再将实际相机旋转作为
反馈。不会从 UE 目标位置、目标 ID 或真值推导控制方向。

- ROS 选定测量：`/<uav>/sar_yolo_detector/dfine_v3/state`
- ROS 实际云台反馈：`/<uav>/gimbal/state`（JSON String，20 Hz 读取；真实更新频率取决于 UE tick）
- UE 独立指令/反馈：`Saved/AirGroundExchange/gimbal_command.json`、`gimbal_feedback.json`
- 相机 TF：每张图像携带其实际捕获姿态，与图像时间戳一致

`monotonic_stamp` 用于同一 UE 执行器内部的间隔和加速度分析，不能与 ROS 主机的单调时钟绝对值比较；`wall_stamp` 用于反馈新鲜度。

反馈包含 `mode`（HOLD/TRACKING/LOST）、角度、角速度、`saturated`、应用指令序号、
UE 实测旋转、反馈时间和 `feedback_fresh`。TRACKING 表示执行器收到有效视线，不能单独
等同于视觉目标当前仍有效；视觉有效性仍由 SAR state 报告。超过测量有效期会制动并进入
LOST；不使用预测框延长实测有效期。编辑器长时间停顿时限制补积分时长，避免恢复后跳转。

## 验证

```bash
python3 "$(rospack find sar_yolo_detector)/test/test_ue_virtual_gimbal.py"
```

单元和合成闭环覆盖角速度/加速度/延迟、机械限位、失锁及取消、重复/乱序/过期测量、
采集姿态换算、机体航向补偿以及不同目标角速度。真实 UE 验收还必须通过网页选择产生
真实视觉测量，检查相机实际转动、目标回中和取消制动；合成像素闭环不能替代这一项。

使用真实 UE 图像与网页自动验收（独立测试命名空间；会选择/取消画面中的目标）：

```bash
# 终端 1：悬停观察运动目标，避免航线变化干扰云台验收
roslaunch sar_yolo_detector ue_dfine_v3_live.launch ue_project:=/your/UE/project \
  scene:="$(rospack find sar_yolo_detector)/config/ue/gimbal_acceptance_scene.json" \
  pose_source:=kinematic UAV_NAME:=gimbal_check web_port:=8788 seconds:=240
# 终端 2
rosrun sar_yolo_detector verify_ue_gimbal.py --namespace gimbal_check \
  --web-url http://127.0.0.1:8788 --output /tmp/ue-gimbal-check
```

验证器保存选择前后图像、视觉状态、实际反馈和 result.json，并检查持续锁定、目标回中、
真实相机旋转、转速和取消制动。飞行器位置保持属于此验收场景，生产默认任务路线不变。
