# xd_uav_detect 解耦版使用说明

更新：2026-09-13

## 1. 包的职责

`xd_uav_detect` 是二维识别结果之后的米制定位层：外部 YOLO/识别节点发布二维候选框，本包按
YAML 选择一种传感器几何，将候选转换为机体系 FRD 位置，并可并行发布局部世界坐标。

定位方式与飞机类型没有绑定关系。固定翼、旋翼或地面测试台只要能提供所选方法要求的消息和
TF，就能使用同一节点。`UAV_NAME` 只控制 ROS 命名空间和默认 frame 前缀。

包内把职责拆成互不依赖的节点：`xd_uav_detect_node` 负责定位，`gimbal_control_node` 负责标准
两轴云台命令和状态。前者不依赖后者；没有可控云台时，仍可单独使用任一种定位方法。

本包不负责 YOLO 推理、自动扫描、目标跟随、厂商协议、任务分配、WGS84 转换、航迹规划或
飞行控制。未来的搜索/跟随策略应作为包内独立节点向公共云台接口发命令，不写进定位算法。

## 2. 编译与环境

环境为 ROS Noetic、Python 3。由工作区根目录编译：

```bash
cd /home/promise/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2 --pkg xd_uav_detect
source devel/setup.bash
```

本包不要求安装厂商云台 SDK。真实设备应由独立驱动或适配器转换为标准 ROS 消息与 TF。

### 2.1 统一 Gazebo 模型目录

`models/` 保存四个模型，既有 MRS/PX4 功能仍从原路径启动，不会被本包自动替换：

| 模型 URI | 内容 |
|---|---|
| `model://x500` | 当前 MRS/PX4 x500 的渲染快照 |
| `model://plane` | 当前 PX4 plane 的原样快照 |
| `model://x500_gimbal` | x500 加 yaw/pitch 光电相机和单束 Range |
| `model://plane_gimbal` | plane 加相同光电载荷 |

独立 world 使用这些模型前设置：

```bash
export GAZEBO_MODEL_PATH="$(rospack find xd_uav_detect)/models:${GAZEBO_MODEL_PATH}"
```

然后在 SDF world 中引用，例如：

```xml
<include>
  <uri>model://x500_gimbal</uri>
  <name>x500_gimbal</name>
  <pose>0 0 1 0 0 0</pose>
</include>
```

四个模型都保留其上游飞行能力。`x500_gimbal/model.sdf` 是当前 MRS `x500` 的完整渲染快照，
保留旋翼动力、IMU、GPS、MAVLink 和流体阻力插件，只额外增加吊舱；`plane_gimbal/plane.sdf`
同样保留 PX4 plane 的气动面、推进器、IMU 和 MAVLink 插件。两个吊舱版本都没有世界固定关节。

直接通过 `model://` 加载的渲染 SDF 按单机 `uav1` 和默认 MAVLink 端口配置，适合本包演示。
需要改变 UAV 名称或用于多机时，应从 `x500_gimbal/x500_gimbal.sdf.jinja` 重新渲染，并为
每架飞机分配独立 namespace 和 MAVLink 端口；不能复制一份静态 SDF 后只改模型名。吊舱关节名
为 `gimbal_yaw_joint`、`gimbal_pitch_joint`，真实飞行仿真还必须发布拍摄时刻的
`body <- gimbal_laser` TF。

当前 MRS spawner 不能直接用 `UAV_TYPE=x500_gimbal`：它把 SDF 模板名、`PX4_SIM_MODEL`、
ROMFS 所属包和 MRS 机型配置名绑定为同一个值，而现有 PX4 ROMFS/MRS 配置只认识 `x500`。
本包不修改 spawner，而是提供独立单机入口：

```bash
roslaunch xd_uav_detect demo.launch mode:=px4 gui:=true
```

该可选飞行演示要求当前工作区已经提供 `gazebo_ros`、`mrs_gazebo_common_resources`、
`mrs_uav_gazebo_simulation` 和 `mrs_uav_px4_api`；它们不是 detect 核心定位节点的运行依赖。
该 launch 直接用 `gazebo_ros/spawn_model` 加载规范 `x500_gimbal/model.sdf`，同时复用现有 MRS
PX4 固件 launch 和 MAVROS launch。Gazebo 模型名是 `uav1`，PX4/MRS airframe 名仍是 `x500`；
PX4 instance 1、TCP 4561、UDP 14561 和 MAVROS `14006@14005` 已对齐。它不会启动
`mrs_drone_spawner`、MRS core、控制器、自动解锁或自动起飞。

检查底座连通：

```bash
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav1/gimbal/range
```

正常结果应为 `connected: True`，并能收到 frame 为 `uav1/gimbal_laser` 的 Range。若要在此
飞行底座上运行 detect，还必须增加真实的动态 `body <- gimbal_laser` TF；本 launch 不使用
Gazebo 真值冒充生产 TF。完整来源和边界见 `models/README.md`。

### 2.2 飞行状态下的吊舱定位演示

以下入口把上一节的 PX4 飞行底座和传感器定位演示组合起来，仍然不使用
`mrs_drone_spawner`：

```bash
roslaunch xd_uav_detect demo.launch mode:=flight gui:=true
```

它额外启动现有 MRS hw API、GPS/baro 状态估计、MPC 控制栈和预飞检查，随后自动解锁、进入
OFFBOARD 并起飞到约 1.5 m。`gimbal_range_demo.py` 等到 Gazebo 真值高度超过 0.8 m 后才开始
正前方命中、yaw/pitch 命中和无返回三个阶段。这里的 Gazebo 真值 TF 仅供演示，不是生产接法。

看到 `/uav1/mavros/state` 为 `armed: True`、`mode: "OFFBOARD"` 后，可以发布一个绝对位置参考。
例如令飞机在 GPS/baro 原点坐标系中移动到 `(3, 0, 1.5)`：

```bash
rostopic pub -1 /uav1/control_manager/reference mrs_msgs/ReferenceStamped \
  "{header: {stamp: now, frame_id: 'uav1/gps_baro_origin'}, \
    reference: {position: {x: 3.0, y: 0.0, z: 1.5}, heading: 0.0}}"
```

每次发布一个新绝对位置即可移动，MRS tracker 会在到达后继续悬停。可观察：

```bash
rostopic echo /uav1/estimation_manager/uav_state
rostopic echo /uav1/gimbal/range
rostopic echo /uav1/detect/detections
rostopic echo /uav1/detect/detections_world
```

正常结束时先请求降落，确认落地后再在 roslaunch 终端按 `Ctrl-C`：

```bash
rosservice call /uav1/uav_manager/land
```

该组合入口是单机仿真演示，不代表完整系统集成；它使用已有两轴云台位置闭环，但没有实现自动
扫描、目标跟随、航迹规划或真实吊舱驱动。Gazebo classic 与完整 MRS core 对 CPU 时序较敏感；
若 MRS 因状态超时进入 failsafe land，
不要继续发送位置参考。

### 2.3 当前入口分类

公开 launch 已收敛为四个，避免把演示层误当生产依赖：

| 当前文件 | 分类 | 用途 |
|---|---|---|
| `detect.launch` | 正式 | 单独启动定位节点 |
| `gimbal_control.launch` | 正式 | 单独启动两轴云台控制节点 |
| `detect_track.launch` | 兼容 | 定位输出镜像到当前 track 输入 |
| `demo.launch mode:=sensor` | demo | 固定旋翼传感器场景 |
| `demo.launch mode:=px4` | demo 底座 | PX4/MAVROS，不自动起飞 |
| `demo.launch mode:=flight` | demo | PX4/MRS 自动起飞、移动与吊舱定位 |

原来的三个演示入口已经删除并由 `demo.launch mode:=sensor|px4|flight` 取代；demo 专用参数位于
`config/demo/`。三个正式定位配置不合并，因为它们的输入与标定契约不同。

当前 YAML 也不是定位算法数量：`lidar_camera.yaml`、`camera_ground_plane.yaml`、
`gimbal_laser_range.yaml` 是三种正式定位契约；`gimbal_control.yaml` 是独立控制参数；
`multirotor_detect.yaml`、`fixedwing_detect.yaml` 是旧命名兼容；
`config/demo/gimbal_flight_autostart.yaml` 只供飞行演示。

## 3. 稳定输入输出

默认以 `UAV_NAME:=uav1` 为例：

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 输入 | `/uav1/detect/input/detections_2d` | `xd_uav_track/DetectionArray` | 外部识别器二维候选 |
| 输入 | 方法相关 | `CameraInfo`、图像、点云或 `Range` | 由 YAML 显式绑定 |
| 输出 | `/uav1/detect/detections` | `xd_uav_track/DetectionArray` | 稳定旧接口，三维位置为 FRD |
| 输出 | `/uav1/detect/detections_world` | `xd_uav_detect/WorldDetectionArray` | 新增世界位置和协方差 |
| 输出 | `/uav1/detect/status` | `std_msgs/String` | 当前方法、有效数量和失败原因 |
| 输出 | `/uav1/detect/debug/image` | `sensor_msgs/Image` | 带框与状态的调试图像 |

旧输出的 `relative_position_body` 永远使用 `[forward, right, down]`，单位 m；不得在此字段写入
世界坐标。`position_covariance` 同样在 FRD 中。定位失败时二维候选仍会转发，但
`range_valid=false`、`has_relative_position_body=false`。

世界输出的 `header.stamp` 保持检测图像拍摄时间，`header.frame_id` 是 `frames/world`。每个元素
通过 `source_candidate_index` 对应旧数组中的同序号候选。缺少检测时刻的 `world <- body` TF
时，只会令 `position_valid=false`，不会影响旧 FRD 输出。

## 4. 选择定位方法

只通过 YAML 的 `localization/method` 选择算法：

| 方法 | 正式配置 | 适用传感器 |
|---|---|---|
| `lidar_camera` | `config/lidar_camera.yaml` | 相机 + PointCloud2 |
| `camera_ground_plane` | `config/camera_ground_plane.yaml` | 相机 + 载体姿态/高度 + 水平地面假设 |
| `gimbal_laser_range` | `config/gimbal_laser_range.yaml` | 云台相机 + 单束激光测距仪 |

启动通式：

```bash
roslaunch xd_uav_detect detect.launch \
  UAV_NAME:=uav1 \
  body_frame:=uav1/base_link \
  world_frame:=uav1/local_origin \
  config:=$(rospack find xd_uav_detect)/config/gimbal_laser_range.yaml
```

同一架飞机切换方法时只换配置文件和相应传感器/TF，不需要修改节点源码或设置
`platform_type`。

### 4.1 LiDAR + camera

必要输入：二维框、相机图像、CameraInfo、PointCloud2。点云与图像按时间戳选择最近样本，点云
通过标定外参投影到图像，算法选择框内最近有效深度簇。

必要几何：

- `calibration/translation_xyz` 与 `rotation_ypr`：LiDAR 到相机的离线标定结果；
- 检测时存在 `body <- lidar` TF；
- CameraInfo 与识别框对应同一成像模型。

关键门限在 `fusion/`：点云时间差、距离范围、框收缩比例、最小聚类点数和深度聚类容差。

```bash
roslaunch xd_uav_detect detect.launch \
  UAV_NAME:=uav1 \
  config:=$(rospack find xd_uav_detect)/config/lidar_camera.yaml
```

### 4.2 Camera + ground plane

必要输入：二维框、图像和 CameraInfo。无需 LiDAR。算法在拍摄时刻取得相机姿态，将框内配置的
锚点射线与 `frames/world` 中的水平面 `z=ground_plane_z_m` 求交。

必要 TF：

```text
world <- camera
body  <- camera
```

两条 TF 默认都必须能在检测时间戳查询。`tf_use_latest_on_failure=false` 是安全默认值，避免高速
运动时拿最新姿态替代拍摄姿态。接近地平线、交点在相机后方或超出距离门限时输出无效。

```bash
roslaunch xd_uav_detect detect.launch \
  UAV_NAME:=uav1 \
  config:=$(rospack find xd_uav_detect)/config/camera_ground_plane.yaml
```

### 4.3 Gimbal camera + single-beam Range

必要输入：二维框、云台相机图像、CameraInfo 和 `sensor_msgs/Range`。Range 的射线遵循标准
ROS 约定，沿 `header.frame_id` 的 +X 轴。检测时间戳处必须存在：

```text
camera optical frame <- laser frame
body frame           <- laser frame
world frame          <- body frame     # 仅世界输出需要
```

节点把激光端点投影到相机图像。只有端点落入恰好一个候选框时，才把斜距赋给该候选；框外或
同时命中多个框均失效关闭。`gimbal_range/maximum_time_difference_sec` 限制 Range 与图像检测的
时间差，量程边界、非有限数、零时间戳、空 frame 和缺 TF 都不会产生有效三维位置。

噪声参数：

- `range_stddev_m`：测距径向标准差；
- `angular_stddev_rad`：视轴角误差；
- `position_stddev_m`：安装位置/外参的附加标准差；
- `bbox_gate_margin_px`：视轴投影与框边缘的像素余量。

仓库中的数值是保守仿真示例，不代表真实吊舱标定结果。

### 4.4 两轴云台控制接口

当前模型和控制器为 yaw + pitch 两自由度。yaw 绕竖直轴旋转，pitch 绕随 yaw 转动的横轴旋转；
相机和激光传感器都属于 `gimbal_pitch` link，因此云台带着整套光电载荷一起运动，不存在支架
固定而只有相机单独转动的情况。两轴能够覆盖水平搜索、俯仰指向和未来目标跟随。roll 只在需要
主动地平线稳定时增加；接口与定位均通过关节名/TF 解耦，未来升级三轴不会改变 detect 输出。

启动独立控制节点：

```bash
roslaunch xd_uav_detect gimbal_control.launch UAV_NAME:=uav1
```

公共接口：

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 输入 | `/uav1/gimbal/command` | `xd_uav_detect/GimbalCommand` | 外部位置、速度或回中命令 |
| 输出 | `/uav1/gimbal/state` | `xd_uav_detect/GimbalState` | 实测/指令角、模式、有效性和状态原因 |
| 内部 | `/uav1/joint_states` | `sensor_msgs/JointState` | 仿真或设备适配器的关节反馈 |
| 内部 | `/uav1/gimbal/set_joint_trajectory` | `trajectory_msgs/JointTrajectory` | 当前 Gazebo PID 执行后端 |

位置命令单位为 rad，角度相对机体；正负方向由模型关节轴定义。位置目标会保持到下一条命令，
并按命令中的正速率上限或 YAML 默认上限渐进到达；超范围角度会被安全夹紧。示例：

```bash
rostopic pub -1 /uav1/gimbal/command xd_uav_detect/GimbalCommand \
  "{mode: 0, yaw: 0.6, pitch: -0.25, yaw_rate: 0.8, pitch_rate: 0.5}"
```

仿真模型与默认控制配置的机械范围为 yaw `[-π,+π]`、pitch `[-π/2,+π/2]`。下面只使用公共
命令话题展示全范围，不依赖演示脚本。先转到 yaw 正限位，待其到位后再发负限位；第二段会沿
反方向扫过约 360°。这是有机械限位的全范围扫掠，不是能够跨过接缝无限同向旋转的 continuous
joint：

```bash
# 终端 1：启动模型和控制器，但关闭自动发命令的定位演示序列
roslaunch xd_uav_detect demo.launch mode:=sensor gui:=true \
  run_demo_sequence:=false

# 终端 2：以下命令逐条执行
# yaw +180°
rostopic pub -1 /uav1/gimbal/command xd_uav_detect/GimbalCommand \
  "{mode: 0, yaw: 3.14159, pitch: 0.0, yaw_rate: 0.65, pitch_rate: 0.65}"

# 等待约 6 秒到位，然后 yaw +180° -> -180°，完成约 360°扫掠
rostopic pub -1 /uav1/gimbal/command xd_uav_detect/GimbalCommand \
  "{mode: 0, yaw: -3.14159, pitch: 0.0, yaw_rate: 0.65, pitch_rate: 0.65}"

# pitch 向上 +90°、向下 -90°
rostopic pub -1 /uav1/gimbal/command xd_uav_detect/GimbalCommand \
  "{mode: 0, yaw: 0.0, pitch: 1.5708, yaw_rate: 0.65, pitch_rate: 0.65}"
rostopic pub -1 /uav1/gimbal/command xd_uav_detect/GimbalCommand \
  "{mode: 0, yaw: 0.0, pitch: -1.5708, yaw_rate: 0.65, pitch_rate: 0.65}"

# 回中
rostopic pub -1 /uav1/gimbal/command xd_uav_detect/GimbalCommand \
  "{mode: 2}"
```

大角度指向时，相机可能看到机腹、支架或地面，这是实际安装几何造成的视场遮挡，不表示命令
失效。可通过 `/uav1/gimbal/state` 的 `yaw`、`pitch` 和 `commanded_*` 字段确认到位情况。

速度命令的 `yaw_rate`、`pitch_rate` 单位为 rad/s，必须持续刷新；超过默认 0.5 s 未收到新命令
就保持当前位置。回中命令忽略其余数值：

```bash
rostopic pub -r 10 /uav1/gimbal/command xd_uav_detect/GimbalCommand \
  "{mode: 1, yaw_rate: 0.3, pitch_rate: 0.0}"

rostopic pub -1 /uav1/gimbal/command xd_uav_detect/GimbalCommand \
  "{mode: 2}"
```

用 `Ctrl-C` 停止连续速度命令，随后控制器会超时保持。默认限制和关节/话题映射位于
`config/gimbal_control.yaml`。真实吊舱应增加一个包内或设备侧适配节点：接收公共命令、调用厂商
SDK，并发布关节反馈和动态 TF；不应让定位节点直接调用厂商协议。

模型使用本包的 `libxd_uav_detect_gimbal_joint_controller.so` 在物理更新线程设置关节 PID 目标，
不会暂停 Gazebo 世界或直接改模型位姿。不要替换成 Gazebo Classic 的
`libgazebo_ros_joint_pose_trajectory.so`：该插件设置关节时会切换全局 physics enabled 状态，已在
完整 PX4/MRS 飞行复验中造成状态估计跳变和 failsafe；无飞控的固定夹具无法暴露这一风险。

## 5. 二维识别器接入要求

识别器发布 `xd_uav_track/DetectionArray`：

- `header.stamp` 必须是图像拍摄时间，不是推理结束时间；
- `header.frame_id` 应与 CameraInfo 的 optical frame 一致；
- 填写 `image_width`、`image_height`；
- 每个候选填写像素 `bbox=[xmin,ymin,xmax,ymax]` 并置 `has_bbox=true`，或填写 `[0,1]`
  图像比例坐标 `normalized_bbox=[cx,cy,w,h]`；
- 推荐填写 `track_id`、`class_id`、`confidence` 及 detector provenance；
- 识别器不应把世界坐标写入 `relative_position_body`。

## 6. 与 xd_uav_track 一起运行

当前分支的 track 默认订阅 `/uav1/track/detections`，而冻结的 detect 主输出必须继续是
`/uav1/detect/detections`。组合入口会同时发布二者，消息内容完全相同：

```bash
roslaunch xd_uav_detect detect_track.launch \
  UAV_NAME:=uav1 \
  detect_config:=$(rospack find xd_uav_detect)/config/gimbal_laser_range.yaml
```

`track_detections_topic:=track/detections` 是默认兼容镜像；已有系统若已把 track 输入重映射到
`detect/detections`，可传 `track_detections_topic:=""` 关闭镜像。此兼容方式不要求修改 track
或 task_allocate。

## 7. Gazebo 光电吊舱可视化演示

演示使用完整 `x500_gimbal` 旋翼模型：MRS/PX4 x500 机体下方安装蓝色 yaw 轴和橙色 pitch
载荷，并带 Gazebo camera、单束 ray/range；原 x500 飞行插件仍在模型内，场景中另有红色实体
目标。此 launch 只验收吊舱传感器和 detect，因此把世界重力设为零且不启动 PX4；为避免
MAVLink/动力插件在没有飞控时终止 Gazebo，launch 从同一目录加载 `sensor_demo.sdf`。这个夹具
保持 x500 外形、惯性、实体云台和传感器，但禁用飞行插件并固定机体；规范的
`x500_gimbal/model.sdf` 不固定且仍保留全部飞行插件。该演示不是飞行联调入口。脚本循环三个阶段：

1. `forward_hit`：吊舱正前方，目标中心 10 m，碰撞面测距约 9.5 m；
2. `yaw_pitch_hit`：真实 yaw/pitch 关节转动，目标移到实际激光轴上，FRD 和世界坐标随 TF 旋转；
3. `no_return_fail_closed`：目标移出射线，Range 到达无返回边界，三维有效标志清除。

启动 GUI：

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/catkin_ws/devel/setup.bash
roslaunch xd_uav_detect demo.launch mode:=sensor gui:=true
```

Gazebo 中直接观察云台和红色目标。另开终端查看数值：

```bash
rostopic echo /uav1/gimbal/range
rostopic echo /uav1/detect/detections
rostopic echo /uav1/detect/detections_world
rostopic echo /uav1/gimbal/state
```

查看带框调试图像：

```bash
rqt_image_view /uav1/detect/debug/image
```

默认每阶段 7 秒并循环。可改为每阶段 12 秒、运行一轮后保持：

```bash
roslaunch xd_uav_detect demo.launch mode:=sensor \
  gui:=true stage_duration:=12 loop:=false
```

按 `Ctrl-C` 结束，roslaunch 会关闭本次启动的 Gazebo/ROS 节点。演示的二维框由确定性 fixture
根据真实 CameraInfo 和已知目标尺寸生成；真实图像会被实际解码，Range 来自 Gazebo 碰撞射线，
但这里不宣称运行了产品 YOLO。演示脚本本身也通过公共 `GimbalCommand` 接口转动关节，不直接
调用 Gazebo 的模型配置服务。

### 7.1 从 dev 同步的目标场景工具

`origin/dev@e763836` 在仓库级 `add_red_box_scripts` 目录提供了四个正式测试工具。本分支为了让
detect 能独立交付，把四个正式版本放入 `scripts/demo/` 并由 CMake 安装；红框和红方块脚本
字节级原样复制，两个车辆脚本只增加 catkin devel wrapper 所需的同目录导入保护，算法、参数和
默认话题均未改写。编译并加载工作区后可使用 `rosrun`，不依赖开发者个人绝对路径：

| 工具 | 用途 | 必需环境 |
|---|---|---|
| `red_box_detector.py` | HSV 红色目标检测并发布二维 `DetectionArray` | ROS 图像、OpenCV、`xd_uav_track` |
| `spawn_red_boxes.py` | 按坐标、矩形或搜索多边形生成红色实体方块 | Gazebo；搜索多边形模式额外需要 task_allocate 消息 |
| `spawn_random_vehicles.py` | 在搜索多边形中生成随机 Gazebo 车辆 | Gazebo 车辆模型和 task_allocate 消息 |
| `spawn_yolo_vehicle_targets.py` | 生成适合固定翼下视/旋翼平视 YOLO 的大型车辆 | 同上 |

查看完整参数：

```bash
rosrun xd_uav_detect red_box_detector.py --help
rosrun xd_uav_detect spawn_red_boxes.py --help
rosrun xd_uav_detect spawn_random_vehicles.py --help
rosrun xd_uav_detect spawn_yolo_vehicle_targets.py --help
```

生成两个红色实体目标；`--replace` 会删除同前缀旧目标，因此只应在明确要刷新演示场景时使用：

```bash
rosrun xd_uav_detect spawn_red_boxes.py \
  --position 5,5 --position 15,15 --ground-z 0 --replace
```

不连接 Gazebo 的只读布局预览：

```bash
rosrun xd_uav_detect spawn_red_boxes.py \
  --count 6 --area=0,20,0,20 --ground-z 0 --seed 7 --dry-run
```

从一架或多架飞机的相机生成二维候选；下视相机可逐机覆盖：

```bash
rosrun xd_uav_detect red_box_detector.py --uavs uav1 uav2 \
  --image-topic uav1=/uav1/down_camera/image_raw
```

默认发布到 `/<uav>/detect/input/detections_2d`，正好是当前三个定位 backend 的统一输入；标注图
发布到 `/<uav>/track/red_detector/debug_image`。红色检测器只产生二维框，不负责定位或稳定目标
ID，三维位置由本包选定的 backend 计算，稳定 ID 仍由 track 或上游跟踪器负责。

按 `/task_allocate/search_areas` 生成目标时，先启动生成脚本等待消息，再发布一次性
`SearchAreaArray`。该模式会在运行时导入 `xd_uav_task_allocate` 消息，但 detect 的正式定位节点
没有增加对 task_allocate 的编译或运行依赖；不使用搜索区工具时无需启动任务包。

```bash
rosrun xd_uav_detect spawn_random_vehicles.py \
  --count 10 --ground-z 0 --replace --seed 7

rosrun xd_uav_detect spawn_yolo_vehicle_targets.py \
  --count 6 --models bus fire_truck --ground-z 0 --replace --seed 7
```

车辆脚本从 `--model-root`、`GAZEBO_MODEL_PATH`、`~/.gazebo/models` 和系统 Gazebo 模型目录依次
查找模型。它们只负责布置目标，不运行 YOLO。远端 `sar_yolo_detector` 已通过消息桥发布同一个
`/<uav>/detect/input/detections_2d`，因此接入真实 YOLO 时继续启动其桥接 launch，无需复制或
修改 YOLO 包，也无需改 detect：

```bash
roslaunch sar_yolo_detector xd_smart_tracker_integration.launch UAV_NAME:=uav1
```

远端 `spawn_red_boxes copy.py` 是文档明确标记的过期副本，功能少于正式脚本，因此不纳入本包。
同目录的 Typhoon UDP 视频桥和旧 `gm_control` Gazebo 适配器属于特定飞机/旧控制包的系统集成，
不属于 detect 定位能力：其图像输入可由任意标准 ROS Image 替代，云台能力则由当前独立的
`GimbalCommand/GimbalState`、真实 Camera/Range 和 x500/plane 载荷模型覆盖。为保持四模型契约
和避免重新绑定旧 `gm_control`，本包不复制这两个特定适配器。

## 8. 诊断顺序

没有有效三维位置时依次检查：

```bash
rostopic echo -n 1 /uav1/detect/status
rostopic hz /uav1/detect/input/detections_2d
rostopic hz /uav1/gimbal_camera/camera_info
rostopic hz /uav1/gimbal/range
rosrun tf tf_echo uav1/base_link uav1/gimbal_laser
```

常见状态 `reason` 与日志告警：

| reason | 含义 |
|---|---|
| `camera intrinsics unavailable` | 尚未收到合法 CameraInfo |
| `synchronized laser range unavailable` | Range 缺失或时间差超限 |
| `invalid laser range` | 时间戳/frame/数值或量程边界无效 |
| `camera/body laser TF unavailable` | 检测时刻的动态 TF 不完整 |
| `laser axis does not identify exactly one detection` | 框外或多框歧义 |
| ROS_WARN：`world/body TF unavailable` | 仅世界输出失败，FRD 仍可有效；定位状态可仍为 `ok` |

## 9. 真实吊舱上线清单

上线前必须确认：

- 相机、Range 和飞控时钟属于同一 ROS 时间基准；
- Range 的 `header.stamp`、`header.frame_id`、`min_range`、`max_range` 正确；
- 相机内参对应实际分辨率和去畸变图像；
- 相机与激光的静态外参经过标定；
- 云台角编码器以足够频率发布动态 `body <- laser` TF，并保留查询所需历史；
- `world <- body` TF 使用与下游任务层一致的局部世界坐标；
- 噪声参数由实测数据标定；
- 用遮挡、无返回、时间跳变和 TF 中断验证 fail-closed 行为。

只有完成上述硬件验证后，才能宣称真实光电吊舱定位可用；本仓库当前完成的是标准接口、算法、
自动回归和真实 Gazebo 传感器验收。

## 10. 回归测试

```bash
cd /home/promise/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make -j2 run_tests_xd_uav_detect
catkin_test_results build/test_results/xd_uav_detect
```

当前最终回归为 45 tests、0 errors、0 failures、0 skipped，包含控制核心、backend factory、
模型生成一致性、定位几何、节点失效保护、真实 Gazebo camera/ray/关节验收，以及 dev 目标工具
的安装入口、无 Gazebo dry-run 和红色区域提取。
