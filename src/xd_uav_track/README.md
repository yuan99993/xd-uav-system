# xd_uav_track

完整启动、三模式、接口字段、网页接入和跨主机部署说明见
[感知使用与部署指南](docs/perception_user_deployment_guide.md)。该指南区分已实现能力、
当前云台/LRF 依赖缺口和真实场景验收边界，部署时优先按它核对。

**仅部署感知推荐** `roslaunch xd_uav_perception_bringup perception.launch`。
入口使用 [统一现场配置](config/perception_deployment.yaml)，提供 detect/track/fusion、
身份、锁定、选中目标的有界轨迹和失联状态，不启动飞行或仿真。节点内部禁止
发布 follower/速度/reference，旧启动服务不能越过隔离。设备驱动、航线和
运动学由外部系统负责。详见指南第 12.5 节；不要与兼容入口重复运行。

三模式、原帧 ROI/UID 选择、有界轨迹与注册 YOLO 模型切换接口见
[网页合同](docs/perception_web_contract.md)及[阶段验收](docs/perception_stage_6_7_8_acceptance.md)。
[网页视频与交互接口](docs/perception_web_video_interface.md)涵盖原分辨率叠框视频、位置表、点击/ROI 选择和模式/模型开关；[接口文件](web/perception_api.json)可交给网页端集成。
严格跨源身份/时延验收尚未通过，兼容 vision_tracking_stack 入口暂保持
`enable_perception_modes:=false`，联调显式设 true。纯感知组合直接启用三模式，
初始 detect，在现场配置中修改 initial_mode。
网页/Gazebo 联调入口为 `perception_gazebo_web_test.launch`，部署依赖组合见
`xd_uav_perception_bringup`。功能回归不等于实景精度认证。

兼容机型/定位编排入口是
[`vision_tracking_stack.launch`](launch/vision_tracking_stack.launch)。它默认使用
train7 TensorRT FP16、单实例双相机检测、预测 ROI、像素辅助、车辆 ReID 和
tracker 统一 ID 管理；不启动 Gazebo 或 PX4。它默认订阅固定相机与云台两路，
可先于设备驱动启动，任一路图像上线即自动进入识别/跟踪，不会额外加载第二份模型。
设备驱动本身并不会凭空启动此 launch；部署时应将它加入无人机的 bringup，且每机只启动一次。
目前工作区没有固定相机/云台的设备级图像发布 launch。将来接入驱动后，
由统一的设备 bringup 包含一次本 launch，再包含实际驱动 launch；不要让两路驱动各自
启动一套识别模型。示例（设备 launch 路径换成真实驱动）：

```xml
<include file="$(find xd_uav_track)/launch/vision_tracking_stack.launch">
  <arg name="UAV_NAME" value="uav1"/>
</include>
<!-- 此处包含固定相机及/或云台的真实设备驱动 launch -->
```

识别和多目标公开 ID 会随图像自动运行；飞行跟随/控制仍需明确选择并启动 tracker，
不会仅因相机上线而自动解锁或下发飞行指令。
只识别时传
`enable_tracking:=false`，仍从每路 `detect/input/*/detections_2d` 输出检测。

```bash
roslaunch xd_uav_track vision_tracking_stack.launch UAV_NAME:=uav1
roslaunch xd_uav_track vision_tracking_stack.launch UAV_NAME:=uav1 enable_tracking:=false
```

以上两条是互斥的启动方式，不要同时运行。真实设备还需把各相机图像、内参、
TF 和云台状态话题映射到该 launch 的参数；没有标定不能声称已完成 metric 锁定。
固定翼传 `vehicle_type:=fixedwing`；不使用云台时传
`enable_sar_gimbal:=false enable_gimbal_lrf:=false`。云台/LRF 的测距标定默认
`calibrated: false`，未标定时只允许安全降级，不把背景测距当目标距离。旧四种组合 launch 是
同一入口的兼容别名，不再运行上游 SmartTracker。Gazebo 演示使用
[`moving_uav_vehicle_search_demo.launch`](launch/moving_uav_vehicle_search_demo.launch)，
其中的场景高度和运动模型不能直接用于实机。
正式入口也默认发布 `/<UAV_NAME>/tracking_visualization`：固定相机单独
在线时显示一路，两路在线时并排显示；设
`enable_tracking_visualization:=false` 可在无显示需求的机载部署中省去
叠框计算。固定相机优先全局搜索，云台对已选中目标取得推理排队优先级；
云台失去可信轨迹后自动回退固定相机。内部检测、轨迹和控制话题不变。
目标框中心点另发布到 `/<UAV_NAME>/track/target_pixels_by_source`。
每条消息含相机来源、采集时间、原图宽高，以及每个已确认目标的公开 ID、
类别和 `(x,y)`。坐标以**对应相机原图左上角为原点**，`x` 向右、`y` 向下；
框采用 `[x_min,y_min,x_max,y_max)`，中心为
`((x_min+x_max)/2, (y_min+y_max)/2)`，允许半像素。`predicted` 字段标明
短暂失检后的预测点，不能将它当作新一次视觉检测。双相机组合画面不缩放：
固定相机位于左侧，云台位于右侧，较矮画面仅在下方补空白；目标点仍使用各自
原图坐标，不使用组合画面的偏移坐标。
验证机载云台时用
[`moving_uav_vehicle_search_gimbal_demo.launch`](launch/moving_uav_vehicle_search_gimbal_demo.launch)：
它沿用同一张地图和车辆路线，同时开启固定下视相机与云台相机、共享 YOLO 推理和
tracker：全局搜索及目标目录由 `search_fixed` 提供；操作员确认目标后，用
`/moving_uav_search/track/set_target_mode` 指定 `image_source: search_fixed` 和公开
`target_id`，云台再转向该实体，并可用两路观测持续更新它。启动时先全局搜索，不会因自动
选中的框直接追逐；切回 `mode: all` 即可恢复。默认从 58 m 高度开始、最低保持 54 m，
下降速度限制为 0.35 m/s，机体偏航上限为 0.12 rad/s。云台 ROI
仍关闭。若目标落在云台中心视线的机腹安全半径内，控制器保持高度并水平错开；固定相机
仍负责维持目标世界位置，不会把云台看不到误判为目标丢失。安全半径按云台最小离轴角、
当前高度和可配置余量计算，默认余量为 2 m。

该演示默认只打开一个组合可视化窗口，输出到
`/moving_uav_search/tracking_visualization`，固定相机和云台的跟踪画面并排显示。每个来源中每条
轨迹只绘制一个框，框上的类别名来自 tracker 已确认的类别；可视化复用同一份 YOLO 检测结果，不会
为显示类别额外触发检测。原有机器话题 `track/detections`、`track/tracks` 和
`track/tracks_by_source` 保持不变，两个来源各自的叠框话题仍保留作诊断。

图像窗口使用独立的 Qt 配置目录，避免恢复旧的浮动置顶停靠布局。在 WSLg 桌面启动时应使用
桌面登录用户而非 root 身份，以保证窗口管理器能正常移动和关闭窗口。

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
/uavX/track/metric_target   xd_uav_track/MetricTarget（共享世界/里程计坐标）
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
| `fw_metric_pursuit` | 机体系速度 | 使用共享世界坐标目标进行固定翼 metric 接近，不要求图像框 |
| `fw_metric_orbit` | 机体系速度 | 使用共享世界坐标目标进行固定翼定半径 metric 盘旋 |

九个模式全部输出速度，保留或适配 PixEagle Follower 的独立 PID、积分限幅、速度 EMA 和偏航处理
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
`uncertainty_scale`、`association_method`、`relative_state_active` 以及
`metric_target_valid`、`metric_active`、`orbit_active` 查看。

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

世界坐标任务模式示例：

```yaml
follower:
  profile: "fw_metric_pursuit"   # 或 fw_metric_orbit
interfaces:
  input:
    metric_target: "track/metric_target"
```

`fw_metric_*` 使用 `state_estimator/main/frames/world/odom` 和目标消息的
`header.frame_id` 作为同一共享世界坐标，不需要把任务坐标手工转换成飞机相对坐标。

也可以在节点运行时通过 `/uav1/track/set_profile` 切换。

所有可调项集中在 `config/track.yaml`，并按阅读顺序分为 `runtime`、`interfaces`、
`tracker`、`follower` 和 `safety`。固定翼目标空速只在
`follower/fw_velocity_vector/airspeed`（metric 模式也可在
`follower/fw_metric_pursuit` 或 `follower/fw_metric_orbit` 下覆盖）配置；实际空速测量、
空速上下限和油门控制属于 `xd_uav_controller`，本包不再重复订阅或配置。

不启动 Gazebo/PX4 的实拍视频检测与 ID 回归入口见
[`docs/video_replay.md`](docs/video_replay.md)：支持单路、双路同步回放，
以及有 MOT 真值时的轻量指标计算。

### 感知模式 V2（分阶段启用）

正式共享 YOLO 入口已经接入与飞行 `all/follow` 状态完全分离的视觉模式管理器。
为保留现有链路的 A/B 回退，当前需显式启用：

```bash
roslaunch xd_uav_track vision_tracking_stack.launch \
  enable_perception_modes:=true
```

主要相对接口位于 `/<uav>/perception/`：

- `mode/set`：`SetPerceptionModeCommand`，支持 `detect/track/fusion`；
- `target/select`：`SelectPerceptionTargetCommand`，支持公共 ID、原图点或任意 ROI；
- `command_result`：区分 `ACCEPTED/APPLIED/REJECTED/FAILED`；
- `state`：锁存的会话、版本、模式、选择、实际 SOT 算法和实际模型状态；
- `targets`：统一数值/显示数据源，包含原图像素框、左上角原点的中心点、归一化框和
  `detector/visual_tracker/prediction` 来源。

ROI 命令必须携带原始相机源以及 `frame_seq` 或 `capture_stamp`；指定帧已离开有界缓存
时返回 `STALE_FRAME`，不会把旧 ROI 套到最新帧。人工框先以 `unknown` 类别持续跟踪，
只有与同一 YOLO 目标连续唯一重叠后才确认类别。新 overlay 严格使用目标快照对应的
原图帧，不再自行移动框，因此画面框和 `perception/targets` 坐标一致。

点击选择也必须指定原始帧；匹配使用该帧的轨迹快照而非最新目标列表。已有 ID 的选择
会初始化实际 SOT；detect 中成功选择会进入 track。共享 YOLO 在 track 已稳定跟踪时
默认将选中来源降为 1 FPS 检测锚定、其他来源降为 0.5 FPS（检测器参数
`~track_anchor_fps/~track_standby_fps`），失联恢复原全图搜索；fusion 保持正常全图检测
并对选中 SOT 做同帧几何/外观/歧义校验。失效视觉证据标 lost/stale，不冒充实测。

人工 ROI 通过 C++ `track/manage_visual_identity` 注册为同一全局实体；连续三帧通过
唯一检测和外观门控后，普通轨迹继承人工 ID，原世界状态、深度特征库和跨源映射迁入
该实体。切回 detect 保留有界后台视觉跟踪（最多 8 个），身份 TTL 默认 60 秒。
跨相机继承仍要求可靠世界定位，不靠相似颜色强行合并；图像跟踪本身没有三维控制权。

失锁输出为 `lost/stale` 身份状态，旧框和中心点清零；overlay 不绘制失锁框，只显示
`LOST ID`。有效框带 `DET/SOT` 标记，真实短时预测带 `PRED` 虚线，默认最多 0.5 秒
（包括传输延迟）。相机停更会主动发出一次清除快照，不保留“仍在锁定”的旧实线框。

三维位置使用独立的 FRD `position_frame/position_stamp`，不是图像 `header.frame_id`。
纯 SOT、预测、失锁和过期测距的 `position_valid=false`；`age_ms` 是真实图像观测的
发布年龄，未知为 -1。部署需配置/发布正确的 `body_frd_frame` TF，并同步更新新增字段的
`TrackState` 生成消息后重启相关节点，不能混用旧 MD5 消息。

完整合同和当前完成边界见
[`perception_modes_and_model_switch_plan.md`](docs/perception_modes_and_model_switch_plan.md)。
运行时模型目前可查询但尚不可热切换；不得调用旧 SmartTracker 切换服务冒充新链路。

### 车辆 ReID、低分框与短时漏框

`sar_yolo_detector/config/xd_vehicle_detection.yaml` 保留置信度 ≥0.10 的框。
Tracker 的 ByteTrack 两阶段关联先处理 ≥0.50 的框，再用 0.10–0.50 的框更新
已有轨迹；低分框不能创建新 ID。train7 的四种车辆标签
`car(0)/ar-car(1)/tank(2)/m142(3)` 均可在运动、位置与外观门控通过后
沿用同一 ID。仅靠两框接近不会合并目标。固定相机预测 ROI 使用同一类别
兼容规则，避免二次检测成功却因 YOLO 类别翻转被丢弃。
轨迹确认前累计类别置信度，确认后将类别作为实体属性固定；后续 YOLO 临时把
其他车辆类别时，检测框仍可更新原轨迹的位置，但不会改写公开
类别和 ID。原始 YOLO 类别仍保留在检测话题中，方便诊断。可通过
`tracker/association/compatible_class_ids` 调整允许互换的类别集合；不要把
外观和运动差异很大的类别无条件放在同一集合中。
叠框画面在轨迹未确认时只显示 ID 与跟踪状态；确认后才显示固定的类别。

车辆外观默认使用经哈希校验的 `vehicle-reid-0001.onnx` 和 OpenCV DNN。
`sar_yolo_detector` 的独立 ReID 工作线程对同帧 ROI 批量编码，并在
`/diagnostics` 报告 `reid_runtime_*` 与 `reid_ms_ewma`。目标短暂漏检时，
`tracking_benchmark.yaml` 最多显示 2 秒标注为 `PRED` 的 Kalman 预测框；
预测框不是新的检测结果，也不会作为新测量更新目标。
双坦克地图的非退化回归门槛与执行方式见
[tracking_benchmark_standard.md](docs/tracking_benchmark_standard.md)。

本机 `.venv-sar-gpu` 安装了 `onnxruntime-gpu==1.23.2`。RTX 3050 上该车辆
ONNX 的两框批次稳态实测为 OpenCV CPU 84–94 毫秒、ONNX Runtime CPU
110–127 毫秒、ONNX Runtime CUDA 约 1.1 秒。所以生产配置
`xd_vehicle_train7` 仍使用 OpenCV CPU；
`xd_vehicle_onnx` 是显式 CUDA 配置，可在完成本机延迟验证后通过
`fixed_reid_profile:=xd_vehicle_onnx` 选择。ReID 加速只影响外观身份关联，
YOLO 检测模型和 TensorRT engine 不受这个配置切换影响。

### 多目标持续像素跟踪

共享检测节点对每条符合类别条件的活动轨迹持续运行稀疏 LK 光流，运动速度为
光流搜索提供有界提前量；没有增加另一种颜色或公开 ID。匹配 YOLO 框时由
检测框重新锚定；无匹配框时才把
合格光流结果作为 0.45 置信度候选补入。该值低于 0.50 新轨迹创建门限，因此
光流只更新已有 ID，不创建身份，也不进入 ReID 特征库。前后向误差、运动一致性、
速度、时间间隔、状态新鲜度及与其他检测框碰撞检查仍会拒绝不可信位移；YOLO
保持运行以搜索新目标和校正漂移。
