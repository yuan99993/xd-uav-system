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

与仓库内 `gm_control` 联动时，使用
`gm_control/launch/xd_track_gimbal_control.launch`。该适配把本包输出中
`selected: true` 的 `TrackState` 转成云台控制框，同时把 `gm_control/GimbalState`
的角度制反馈转换成本包所需的弧度制姿态。不要同时启动 `gm_control` 自带的
`bbox_tracker_node.py`，否则两个节点会竞争同一个目标框话题。

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
`config/track.yaml` 中的 `tracker/enabled_at_startup: true`。

只需要机体系速度、不直接连接现有控制器时：

```yaml
runtime:
  publish_control_reference: false
```

所有输入输出话题保留在 `config/track.yaml` 的 `interfaces` 中。例如只需把对应值改成：

```yaml
interfaces:
  input:
    detections: "perception/detections"
    gimbal_state: "gimbal/attitude"
    vehicle_state: "state_estimator/main/odom"
  output:
    body_velocity: "guidance/body_velocity"
    follower_command: "guidance/follower_command"
    status: "guidance/status"
    control_reference: "control/reference/setpoint"
```

相对话题会自动进入 `/UAV_NAME/...`；只有确实需要跨命名空间连接时才填写绝对话题。

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
python3 src/add_red_box_scripts/red_box_detector.py --uavs uav1 uav2
```

脚本分别向 `/<uav>/detect/input/detections_2d` 发布一个或多个候选；各机的
`xd_uav_detect` 负责雷达融合和转发到 `/<uav>/track/detections`。参数可在命令行覆盖，例如：

```bash
python3 src/add_red_box_scripts/red_box_detector.py --uavs uav1 uav2 \
  _minimum_area_px:=800 _saturation_min:=120 _publish_debug_image:=false
```

GM 模式的云台姿态示例：

```bash
rostopic pub -r 20 /uav1/track/gimbal_state xd_uav_track/GimbalState \
"{yaw_rad: 0.20, pitch_rad: 0.10, roll_rad: 0.0, valid: true}"
```

固定翼速度模式示例：

```yaml
follower:
  profile: "fw_velocity_vector"
```

也可以在节点运行时通过 `/uav1/track/set_profile` 切换。

所有可调项集中在 `config/track.yaml`，并按阅读顺序分为 `runtime`、`interfaces`、
`tracker`、`follower` 和 `safety`。固定翼目标空速只在
`follower/fw_velocity_vector/airspeed` 配置；实际空速测量、
空速上下限和油门控制属于 `xd_uav_controller`，本包不再重复订阅或配置。

## 生产多目标/ReID 边界

正式视觉链路中，`sar_yolo_detector` 只产生 bbox、类别和置信度，并直接发布
`DetectionArray`；其 `track_id=-1`、`track_id_is_stable=false`，避免额外的桥接进程和
一次 ROS 序列化。标准 `vision_msgs` 或旧输入仍可显式使用兼容桥接器。
`xd_uav_track` 是唯一的稳定身份分配器，并维护确认、遮挡、丢失和重捕获状态。
不要把 `xd_smart_tracker_integration.launch` 接入这条链路，否则会重新引入双重
ID 管理。

`launch/vision_tracking_stack.launch` 的 `enable_reid`（默认 true）会启用无状态
外观编码。SAR YOLO 默认采用 `inline_reid:=true`：直接复用检测器已解码的图像，并在
同一进程的独立 latest-only 工作线程中流水处理，不再为每个相机重复传输和反序列化
整幅图像。接入其他检测器时可设置 `inline_reid:=false`，恢复独立的
`reid_encoder_node.py`。两种模式都只写入
`DetectionCandidate.appearance_embedding`，不分配 ID、不保存丢失目标。模型和类别
契约在 `config/reid.yaml`：默认 `xd_vehicle_train7` 对 car/ar-car/tank 使用
Open Model Zoo `vehicle-reid-0001` 的 512 维车辆域深度特征，并以 hybrid 描述子作为
模型加载失败时的降级路径。Market1501 的行人 OSNet 仅可在明确选择 `coco_person`
时启用，而且其外观证据权重低于车辆。编码不可用时轨迹管理仍可退化为空间、运动和
世界坐标关联。

轨迹关联采用类别硬门控、Kalman 马氏距离、IoU/中心距离和有限外观 gallery 的两阶段
全局分配。低置信检测只维持已有轨迹，不会创建新 ID。锁定目标进入短时失检时会保持
其公开 ID 和 coast 预测，直到 `tracker/selection/locked_target_release_timeout_sec`
超时或人工重新选择，避免自动跳到画面中的另一目标。

相似车辆场景采用并行的普通 ReID + 按需 Group-ReID。普通关联是低开销主通道；当前
两名代价过近，或类别、尺寸比例、混合颜色/纹理特征、相机来源等粗属性同时匹配多个
身份时，才进入组内二次判别。`group_id` 仅在跟踪器内部使用，同组车辆仍拥有不同公开
`track_id`。活动轨迹超时后，已确认身份进入有界的长期库（默认 60 s）；库中保留多样
外观原型、按采集时刻姿态投影的世界位置/速度、增长协方差和身份置信度。长期恢复必须
同时通过世界位置创新、外观绝对门限和第一/第二名分差，
并连续确认三帧后才恢复 `control_measurement_ready`。候选接近时不会强行占用旧 ID。
关联器会在类别、运动、世界坐标门控后，将稀疏二分图拆为独立连通分量再执行
Hungarian；稳定 detector ID 和世界量测均使用帧内索引，长期库转移 embedding gallery
使用移动语义。因此目标数量较多但空间上分离时，不会为全部无效组合支付一个巨型稠密
分配矩阵的三次复杂度。

车辆喷涂号、OCR、二维码或 AprilTag 可通过可选
`interfaces/input/identity_hints`（`IdentityHintArray`）接入。配置
`tracker/identity_hints/enabled: true` 后，提示必须与检测时间及 bbox 重叠一致；同一采集
时间的重复消息只计一次。编号连续出现至少两次才固化，高置信编号冲突直接拒绝身份关联，
单次低质量结果不会覆盖长期身份。跨固定相机/云台交接仍要求世界坐标创新门控；一致实体
编号可替代受视角影响的外观否决，但绝不单独绕过空间门控。

固定相机和云台相机各自维护局部 tracklet。启用
`tracker/global_identity/enabled` 后，已锁定目标可以在两路相机间保持同一公开 ID，
但前提是类别相同、采集时刻的世界坐标量测通过控制器的时间/协方差创新门控；两端都有
embedding 时还必须不与 `appearance_cosine` 矛盾。仅凭相似外观绝不允许跨相机合并。
真实录制数据的 ID switch、误关联和重捕获验收格式见
[`docs/tracking_recording_acceptance.md`](docs/tracking_recording_acceptance.md)。

实机视觉相机可直接接入 `vision_tracking_stack.launch` 的
`fixed_image_topic/fixed_camera_info_topic` 或
`gimbal_image_topic/gimbal_camera_info_topic`。检测、ReID 和定位消息沿用相机采集
时间戳；内联模式不需要图像时间近邻查找，独立模式使用 50 ms 同步窗、最多 80 ms
的到达顺序等待。两者都使用单槽最新结果和 750 ms 输出新鲜度门限，避免低算力设备
积压旧帧。车辆默认使用不依赖 PyTorch 的 ONNX 深度模型，同帧相同模型的多个 ROI
合并为一次 batch；模型来源、摘要和车辆域限制记录在 `models/reid/README.md`。
`xd_vehicle_onnx` 是同一生产配置的显式别名。行人深度 OSNet 同样支持 ROI batch，
hybrid 仅作为车辆模型初始化失败时的安全降级。

`.pt` YOLO 仍运行在工作区 `.venv-sar-gpu`（或 `SAR_YOLO_PYTHON` 指定的
环境）中。该环境必须完整安装 `sar_yolo_detector/requirements-smart-tracker.txt`；
尤其不能从 ROS Noetic 的 Python 3.8 目录借用 `netifaces` 到 Python 3.10 环境，否则
节点虽能加载模型，但实际相机 TCPROS 连接会失败。

`vision_tracking_stack.launch` 是组合入口，因此部署镜像还应显式包含
`sar_yolo_detector` 和 `xd_uav_detect`。二者都依赖本包的消息，不能反向写入本包的
`package.xml`，否则 catkin 会形成循环依赖；应在系统级 rosdep/镜像清单中同时安装
这三个包。

连接实机相机前可离线检查当前类别/ReID 合同；深度或 ONNX profile 会同时校验模型
类型、路径和 SHA-256；默认车辆 profile 会校验随包安装的 ONNX 权重：

```bash
rosrun xd_uav_track validate_tracking_config.py \
  --reid-config $(rospack find xd_uav_track)/config/reid.yaml \
  --reid-profile xd_vehicle_train7
```

采集上线验收证据时可另开终端运行：

```bash
roslaunch xd_uav_track record_tracking_evidence.launch UAV_NAME:=uav1 \
  output_prefix:=tracking_evidence_01
```

该入口只录制两路原图、相机内参、检测各阶段、轨迹、状态、里程计和云台状态，
不会启动飞控或改变控制参考。
