# xd_uav_detect

本包接收外部识别器的二维候选，并按 YAML 选择定位方法；定位方式与固定翼/旋翼机型无关。
包内另提供与定位节点独立的两轴光电云台控制接口及 Gazebo 执行后端。

完整接入、TF、参数、诊断和 Gazebo 演示说明见 `docs/USAGE.md`。
四个统一 Gazebo 模型的来源、用途与飞行集成边界见 `models/README.md`。

本包还保留了 `origin/dev` 后续增加的四个正式目标场景工具：红色目标二维识别、红色方块生成、
搜索区随机车辆生成和 YOLO 车辆目标生成。脚本位于 `scripts/demo/`，源自
`origin/dev@e763836:src/add_red_box_scripts/` 的对应正式版本；车辆脚本仅增加 catkin 安装态
导入保护。安装后可直接通过 `rosrun xd_uav_detect <脚本名>` 使用，完整命令与可选依赖见
`docs/USAGE.md`。

## 输出契约

- `/<uav>/detect/detections`：原 `xd_uav_track/DetectionArray` 接口，`relative_position_body` 始终为机体系 FRD（m）。
- `/<uav>/detect/detections_world`：新增 `xd_uav_detect/WorldDetectionArray`，在检测时刻输出配置 world frame 中的位置和协方差。

`detect_track.launch` 另将同一旧消息镜像到 `/<uav>/track/detections`，以兼容当前 track 输入配置；该镜像可通过 `track_detections_topic` 改名或置空关闭。

世界 TF 缺失只使世界候选无效，不改变旧 FRD 输出。定位失败时保留二维候选并清除三维有效标志。

## 定位方法

| 方法 | 配置 | 输入 |
|---|---|---|
| `lidar_camera` | `config/lidar_camera.yaml` | 2D 框、CameraInfo、PointCloud2、标定/TF |
| `camera_ground_plane` | `config/camera_ground_plane.yaml` | 2D 框、CameraInfo、地面高程、拍摄时刻 TF |
| `gimbal_laser_range` | `config/gimbal_laser_range.yaml` | 2D 框、CameraInfo、`sensor_msgs/Range`、拍摄时刻 TF |

旧 `multirotor_detect.yaml` 和 `fixedwing_detect.yaml` 保留为兼容入口；`UAV_NAME` 只用于命名空间和 frame 前缀，不参与方法选择。

## 光电定位契约

光电模式要求 Range 使用传感器坐标系 +X 视轴，并在检测时间戳处存在 `camera <- laser` 与 `body <- laser` TF。激光端点投影必须只落入一个候选框；无匹配或多框歧义均按失效保护处理。示例噪声参数不是实机标定值。

## 两轴云台控制

本包同时提供与定位节点独立的 `gimbal_control_node`。公共接口为
`/<uav>/gimbal/command`（`GimbalCommand`）和 `/<uav>/gimbal/state`
（`GimbalState`），支持位置、速度和回中三种命令。当前机械模型为 yaw + pitch 两自由度：
yaw 负责水平搜索/指向，pitch 负责俯仰指向；相机和单束激光都固定在 pitch 载荷上，始终一起
运动。仿真默认范围为 yaw ±180°、pitch ±90°，可直接发布公共命令完成全范围扫掠；具体命令
见 `docs/USAGE.md`。roll 轴本轮不实现，但定位算法只依赖 TF，因此未来增加三轴模型不需要改变
检测输出契约。

控制节点的公共命令不依赖 Gazebo；当前仿真后端通过内部
`/<uav>/gimbal/set_joint_trajectory` 驱动模型。真实设备仍需适配器把公共命令映射到厂商协议，
并把编码器状态和 TF 回传。控制接口不包含自动扫描、目标跟随或厂商 SDK。

## 启动与验证

基于 MRS/PX4 `x500_gimbal` 旋翼模型的可视化光电吊舱演示：

```bash
roslaunch xd_uav_detect demo.launch mode:=sensor gui:=true
```

```bash
roslaunch xd_uav_detect detect.launch \
  UAV_NAME:=uav1 \
  config:=$(rospack find xd_uav_detect)/config/gimbal_laser_range.yaml
```

```bash
cd /home/promise/catkin_ws
catkin_make -j2 --pkg xd_uav_detect
catkin_make -j2 run_tests_xd_uav_detect
catkin_test_results build/test_results/xd_uav_detect
```

`test/gazebo/gimbal_range.test` 使用 x500 机体外形及惯性、真实 Gazebo camera、ray/range 插件、
yaw/pitch revolute joints 和实体碰撞目标，并读取 x500 base/gimbal link 真值验证正前方、
组合云台角、FRD/世界输出及移出视轴无返回。真实吊舱由包外驱动/适配器提供标准 Range、
CameraInfo 和动态 TF；本包不内嵌厂商 SDK，也不负责 YOLO 推理、自动扫描、目标跟随、
跟踪、任务分配或飞行控制。

规范 `models/x500_gimbal/model.sdf` 完整保留当前 MRS/PX4 x500 的飞行插件；无 PX4 的传感器
演示显式使用同目录 `sensor_demo.sdf` 夹具。当前 MRS spawner 尚不能把新 Gazebo 模板名与
既有 PX4 airframe 名 `x500` 分开，完整飞行接入边界见 `models/README.md`。

不使用 MRS spawner 的单机 PX4/MAVROS 模型入口：

```bash
roslaunch xd_uav_detect demo.launch mode:=px4 gui:=true
```

该可选入口复用工作区现有的 MRS PX4/MAVROS launch 和 Gazebo 世界资源，但不使用
`mrs_drone_spawner`；它只启动飞行底座，不自动起飞，也不代替上面的吊舱定位演示。

飞行中的完整吊舱定位演示：

```bash
roslaunch xd_uav_detect demo.launch mode:=flight gui:=true
```

该入口在同一 Gazebo 中组合规范 `x500_gimbal`、PX4/MAVROS、MRS core、自动起飞和
`gimbal_range_demo.py`。起飞稳定后可向 `/uav1/control_manager/reference` 发布
`mrs_msgs/ReferenceStamped` 移动飞机；命令示例和安全停止方法见 `docs/USAGE.md`。
