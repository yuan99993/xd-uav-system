# xd_uav_track

`xd_uav_track` 把 PixEagle/现有 ROS 包中与“选定目标框跟随”直接相关的 Tracker 和
Follower 能力合并到一个 ROS 包中：

```text
DetectionArray（0/1/多个候选）-> 关联、Kalman、目标选择 -> Follower -> 速度向量
云台当前姿态（GM 模式）-> 校准/滤波/视轴解算 --/              -> XD 控制器 ENU 速度参考
```

本包既支持固定机载相机，也支持 PixEagle 的两种 GM（云台）Follower。GM 模式只读取
云台姿态并完成视轴到机体系的解算，不向云台下发控制命令，也不包含厂商 SDK、串口驱动
或光电吊舱管理。它也不执行 PX4 模式切换、解锁、起降、返航、控制租约、状态估计或
底层动力学控制。
这些职责分别留给上游检测器、`xd_uav_state_estimators`、`xd_uav_controller` 和
`xd_uav_control_manager`。

## 输入和坐标

```text
/uavX/track/detections      xd_uav_track/DetectionArray（唯一感知输入）
/uavX/track/gimbal_state    xd_uav_track/GimbalState（仅 GM 模式需要）
```

每条消息可以包含零个、一个或多个候选框，图像原点在左上角。Tracker 将框中心归一化到 PixEagle
约定的 `[-1, 1]` 坐标：`x` 向右为正，`y` 向下为正，图像中心为 `(0, 0)`。它负责：

- 框尺寸、有限值和置信度校验；
- 带迟滞的置信度锁定，减少阈值附近抖动；
- 目标中心、尺寸和中心速度的时间滤波；
- `track_id` 切换检测、输入超时和跟踪状态维护。

节点内部始终使用同一套轨迹管理：零个候选时执行遮挡预测，一个候选时直接建立或更新
轨迹，多个候选时进行关联和选择。每个目标具有独立的恒速度 Kalman 状态，支持
稳定 detector ID、IoU/中心距离关联、appearance embedding 余弦重识别、
tentative/confirmed/occluded/lost 生命周期、短时遮挡预测和手动/自动选目标。
Tracker 不再订阅图像或 CameraInfo；检测框、图像尺寸和可选三维位置统一由
`xd_uav_detect` 或其他上游感知节点提供。

多目标输出及选目标：

```text
/uavX/track/tracks          xd_uav_track/TrackStateArray
/uavX/track/select_track    xd_uav_track/SelectTrack
```

`SelectTrack` 可以直接给 `target_id`，也可以用 normalized ROI 选择与区域 IoU 最大的轨迹。
没有手动选择且 `auto_select: true` 时，会选质量最高的 confirmed 轨迹。

`GimbalState` 使用弧度，沿用 PixEagle 的云台约定：偏航向右为正、俯仰向下为正、滚转
顺时针为正。`follower/gimbal` 可配置安装方式 `HORIZONTAL`、`VERTICAL` 或
`TILTED_45`，以及零偏、反向、死区和角度滤波。GM 模式要求目标框与云台姿态都未超时。

## Follower 模式

| `follower/profile` | 输出类型 | 核心行为 |
|---|---|---|
| `mc_velocity_ground` | 机体系速度 | 地面目标前后/左右居中，可选受控下降 |
| `mc_velocity_position` | 机体系速度 | 前向/侧向为零，以偏航和升降保持目标居中 |
| `mc_velocity_distance` | 机体系速度 | 前向为零，以侧移和升降保持目标居中 |
| `mc_velocity_chase` | 机体系速度 | 前向渐变追逐，协调转弯或侧移，并做垂直居中 |
| `gm_velocity_chase` | 机体系速度 | 根据云台偏航/俯仰角做 PID 追逐，支持恒速或俯仰变速 |
| `gm_velocity_vector` | 机体系速度 | 将云台光轴直接变换为机体系三维速度向量 |
| `fw_velocity_vector` | 机体系速度 | 固定翼目标空速方向与爬升率组成的单一三维速度向量 |

七个模式全部输出速度，保留或适配 PixEagle Follower 的独立 PID、积分限幅、速度 EMA 和偏航处理
（死区、速度缩放、变化率限制、EMA）。追逐模式在丢失目标后立刻清零水平、垂直和
偏航控制，并按 `forward_ramp_rate` 将前向速度减到配置值。

固定翼模式不在本包中执行 L1、TECS、机体角速度或油门控制。它以配置的 `airspeed`
作为水平速度模长：框在右侧时把速度方向向右偏，框在左侧时向左偏；框在上方时输出
正爬升率，框在下方时输出负爬升率。发布给控制器的 `PositionTarget` 只启用
`velocity.x/y/z`，显式忽略 yaw 和 yaw_rate。实际航向、滚转、俯仰、空速限制及油门均由
`xd_uav_controller` 的固定翼控制器完成。

运行时切换模式：

```bash
rosservice call /uav1/track/set_profile \
  "profile: 'mc_velocity_distance'"
```

切换时会重置 PID 积分，并受最短切换间隔约束；多旋翼模式之间的速度命令按
`safety/profile_switch/blend_duration_sec` 平滑过渡。GM 姿态超时后可自动回退到对应的
固定相机速度模式，姿态恢复后自动返回请求的 GM 模式。

## 输出及 XD 栈衔接

```text
/uavX/track/velocity_body              geometry_msgs/TwistStamped
/uavX/track/command                    xd_uav_track/FollowerCommand
/uavX/track/status                     xd_uav_track/TrackStatus
/uavX/control/reference/setpoint       mavros_msgs/PositionTarget
```

`command` 是所有 7 个速度模式统一、带有效性的权威输出。`velocity_body`
使用 ROS FLU 机体系：`x` 前、`y` 左、`z` 上，`angular.z` 为逆时针
正偏航率。节点只从 `/uavX/state_estimator/main/odom` 读取当前航向，把 FLU 水平速度
旋转到 odom 的 ENU 坐标，随后发布现有 `xd_uav_controller` 的速度参考。

Follower 内的速度上限用于限制制导意图；最终速度、加速度、加加速度和可行性约束仍由
`xd_uav_controller` 完成。估计器有效性、OFFBOARD、起降和飞行失效保护仍由现有状态
估计与 control manager 负责，因此本包不重复这些配置和操作。

收到第一条有效框前不会发布控制参考。普通速度模式取得目标后若目标丢失，会发布有意的
减速或零速度参考。固定翼不能原地停止，因此 `fw_velocity_vector` 会立即停止刷新参考，
由 `xd_uav_controller` 在外部参考超时后进入其配置的 loiter。若估计器里程计超时，则停止
向 controller 发布参考，并在状态消息中报告原因。

多目标 Kalman 协方差过大时，Follower 会按 `safety/uncertainty` 对速度渐进限幅，达到
`abort_ratio` 后停止制导；appearance 重识别成功后也会从较低速度渐进恢复。若候选带有
body forward/right/down 的相对三维位置和速度，追逐模式还能启用米制距离误差与目标速度
前馈。以上状态都能从 `TrackStatus` 的 `target_predicted`、`tracking_quality`、
`uncertainty_scale`、`association_method` 和 `relative_state_active` 查看。

## 启动

```bash
UAV_NAME=uav1 roslaunch xd_uav_track track.launch
```

Tracker 默认处于安全停止状态。识别节点可以提前持续发布框，飞机、估计器和控制器准备好
后再调用 `StartTracker`：

```bash
rosservice call /uav1/track/start_tracker "start: true"
```

停止跟随（普通速度模式发布一次零参考；固定翼模式停止刷新并等待下游进入 loiter）：

```bash
rosservice call /uav1/track/start_tracker "start: false"
```

独立于 tracker 启停的紧急制导停止：

```bash
rosservice call /uav1/track/emergency_stop "data: true"
# 排除风险后解除
rosservice call /uav1/track/emergency_stop "data: false"
```

状态可通过 `/uav1/track/status` 的 `tracker_active`、`tracking_state` 和
`invalid_reason` 字段确认。确需保持旧的启动即跟随行为时，可以显式设置
`tracker_enabled_at_startup:=true`。

只需要机体系速度、不直接连接现有控制器时：

```bash
roslaunch xd_uav_track track.launch publish_control_reference:=false
```

所有输入输出话题都可以直接从 launch 指定，例如：

```bash
roslaunch xd_uav_track track.launch \
  body_frame:=base_link \
  detections_topic:=/perception/detections \
  gimbal_state_topic:=/gimbal/attitude \
  body_velocity_topic:=/guidance/body_velocity \
  follower_command_topic:=/guidance/follower_command \
  status_topic:=/guidance/status \
  state_topic:=/uav1/state_estimator/main/odom \
  reference_topic:=/uav1/control/reference/setpoint
```

示例单候选输入：

```bash
rostopic pub -r 20 /uav1/track/detections xd_uav_track/DetectionArray \
"{header: {frame_id: 'camera_optical_frame'}, image_width: 640, image_height: 480,
candidates: [{track_id: 1, class_id: 0, track_id_is_stable: true,
bbox: [400, 180, 500, 300], has_bbox: true, confidence: 0.9}],
image_source: 'front_rgb', detector_name: 'example'}"
```

仿真红色目标检测节点可直接运行，不需要单独的 launch：

```bash
python3 src/xd_uav_detect/scripts/red_box_detector.py
```

脚本向 `/uav1/detect/input/detections_2d` 发布单候选；`xd_uav_detect` 负责雷达融合和转发
到 `/uav1/track/detections`。参数可在命令行覆盖，例如：

```bash
python3 src/xd_uav_detect/scripts/red_box_detector.py \
  _minimum_area_px:=800 _saturation_min:=120 _publish_debug_image:=false
```

GM 模式的云台姿态示例：

```bash
rostopic pub -r 20 /uav1/track/gimbal_state xd_uav_track/GimbalState \
"{yaw_rad: 0.20, pitch_rad: 0.10, roll_rad: 0.0, valid: true}"
```

固定翼速度模式示例：

```bash
UAV_NAME=uav1 roslaunch xd_uav_track track.launch \
  follower_profile:=fw_velocity_vector
```

也可以在节点运行时通过 `/uav1/track/set_profile` 切换。

所有可调项集中在 `config/track.yaml`，并按阅读顺序分为 `vehicle`、`runtime`、
`interfaces`、`tracker`、`follower` 和 `safety`。固定翼目标空速只在
`follower/fw_velocity_vector/airspeed` 配置；实际空速测量、
空速上下限和油门控制属于 `xd_uav_controller`，本包不再重复订阅或配置。
