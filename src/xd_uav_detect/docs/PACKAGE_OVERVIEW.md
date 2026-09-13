# xd_uav_detect 包说明

更新：2026-09-13

## 职责与边界

xd_uav_detect 是外部二维识别器和跟踪/任务层之间的米制定位层。它接收二维框，按 YAML 显式选择定位方法，输出目标的机体系 FRD 坐标；同时可并行输出局部世界坐标。包内另有独立的两轴云台控制模块，定位节点不依赖控制节点。它不运行 YOLO，不实现自动扫描/目标跟随，不负责任务分配、GPS 转换、规划或飞机控制。

定位方法只取决于可用传感器和几何模型，不取决于固定翼或旋翼机型。UAV_NAME 只用于 ROS 命名空间与默认 frame 前缀。

## 包内模块

| 模块 | 职责 | 稳定边界 |
|---|---|---|
| `xd_uav_detect_node` | 三种传感器定位、FRD 与世界并行输出 | 标准检测消息、CameraInfo/Range/PointCloud2、TF |
| `gimbal_control_node` | yaw/pitch 位置、速度、回中及失效保护 | `GimbalCommand`、`GimbalState` |
| `models/` 与演示 | x500/plane 及带吊舱派生模型、Gazebo 后端 | joint state 与内部 JointTrajectory |
| 目标场景工具 | 红框二维识别、红方块/车辆生成 | 标准检测消息与 Gazebo 服务 |

当前机械自由度为 yaw + pitch，相机和激光共同安装在 pitch link 上。未来若需要 roll 地平线稳定，
可扩展控制消息/执行适配器和模型；定位算法仍只消费传感器消息与 TF，不需知道云台轴数。

`scripts/demo/` 中的 `red_box_detector.py`、`spawn_red_boxes.py`、
`spawn_random_vehicles.py` 和 `spawn_yolo_vehicle_targets.py` 同步自
`origin/dev@e763836` 的四个正式目标测试脚本；前两个原样复制，后两个只适配 catkin 安装态
的兄弟模块导入。它们是可选演示工具，不被定位节点加载：
红框检测只发布统一二维候选，生成脚本只布置 Gazebo 目标，因此没有把机型、YOLO 或任务分配
重新耦合进定位 backend。

## 当前内部结构

M6/M7 已采用一个 ROS 包、多个内部模块：

| 层 | 目标职责 | 扩展方式 |
|---|---|---|
| ROS 编排层 | 订阅、缓存、时间戳、TF、发布和兼容镜像 | 统一调用 backend，不持有具体算法对象 |
| localization backend | 每种传感器组合的几何、协方差 | 实现 backend 并在编译期 factory 注册 |
| gimbal control | 公共命令/状态、限制、watchdog | 执行后端与策略分离 |
| device/simulation adapter | Gazebo PID 或未来厂商协议 | 不改变公共命令和定位接口 |
| strategy（未来） | 扫描、搜索、跟随 | 独立节点发布 `GimbalCommand` |
| payload model | 公共云台机构和可选 Camera/Range profile | `models/common/eo_gimbal.sdf.jinja` |

公开入口为 `detect.launch`、`gimbal_control.launch` 和统一
`demo.launch mode:=sensor|px4|flight`；`detect_track.launch` 单列为下游兼容入口。三个正式定位
YAML 继续保留，避免把互不适用的标定和门限塞入巨型配置。测试、消息、许可证、四个可加载模型和
运行时渲染 SDF 是必要交付物，不以减少文件数字为由删除。源码按 `src/localization`、
`src/gimbal`、`src/gazebo`，脚本按 `scripts/demo|model`，测试按 `test/unit|ros|gazebo|model`
分层；demo 专用配置位于 `config/demo`。

## 方法与输入

| localization/method | 输入 | 几何 |
|---|---|---|
| lidar_camera | 2D 框、CameraInfo、PointCloud2、标定/TF | 框内最近有效深度簇 |
| camera_ground_plane | 2D 框、CameraInfo、拍摄时刻 TF、地面高程 | 像素射线与水平地面相交 |
| gimbal_laser_range | 2D 框、CameraInfo、sensor_msgs/Range、拍摄时刻 TF | 单束激光端点定位 |

正式配置分别为 config/lidar_camera.yaml、config/camera_ground_plane.yaml 和 config/gimbal_laser_range.yaml。旧 multirotor_detect.yaml 与 fixedwing_detect.yaml 只作为兼容入口保留。

gimbal_laser_range 不绑定厂商 SDK。外部适配器必须提供：检测图像时间戳、相机内参、带有效时间戳与 frame 的标准 Range，以及检测时刻的 camera <- laser、body <- laser TF。激光采用 Range 规定的 +X 轴。只有一个候选框覆盖激光端点投影时才赋予距离；多框歧义或视轴框外均失效关闭。

## 输出契约

旧输出 `/<uav>/detect/detections` 的类型仍为 xd_uav_track/DetectionArray。成功候选的 relative_position_body 永远是 [forward,right,down] FRD，单位 m；position_covariance 同样在 FRD 中。失败时保留二维候选并清除三维有效性。xd_uav_track 与 xd_uav_task_allocate 不需要修改。

当前分支的 xd_uav_track 默认订阅 `/<uav>/track/detections`。组合入口 detect_track.launch 会把完全相同的旧消息可选镜像到该话题，同时继续发布主 `/detect/detections`；参数 `track_detections_topic` 可改名或置空关闭。该兼容层位于 detect 内，不修改下游包。

新输出 `/<uav>/detect/detections_world` 的类型为 xd_uav_detect/WorldDetectionArray。header.stamp 保持原图拍摄时间，header.frame_id 是配置世界 frame。每个元素保留源候选索引、track/class/confidence 和 provenance，并包含世界位置、世界协方差和 position_valid。检测时刻 world <- body TF 缺失时只让世界元素无效，不影响旧 FRD 输出。

禁止将世界坐标写入 relative_position_body，否则当前任务层会再次执行 FRD 到世界的转换。

## 失效语义

以下情况均转发二维候选但不产生新的有效三维位置：内参缺失、检测或 Range 零时间戳、Range 过期/非有限/位于量程边界、frame 缺失或不匹配、TF 缺失、视轴未落入框、多框歧义，以及各原有后端的点云簇或地面交点失败。

## 验证状态

2026-09-13 验证包括：

- catkin_make -j2 --pkg xd_uav_detect 通过；
- 原 LiDAR-camera 与 ground-plane 单元回归和新增 gimbal 几何测试通过；
- ROS 节点 fail-closed 测试覆盖零/过期时间戳、量程边界、框外、多框、缺 TF，以及缺世界 TF 时 FRD 独立可用；
- Gazebo Classic 场景使用完整 x500 外形、真实 camera 与 ray/range 插件、实体碰撞目标和实际
  yaw/pitch revolute joints；测试读取 x500 base/gimbal link 真值生成动态 TF，验证 9.5 m
  正前方测距、组合 yaw/pitch、FRD/世界输出和移出射线无返回；
- 测试夹具由真实 CameraInfo 和已知实体尺寸生成严格对应的确定性框，不伪造 Range。
- `demo.launch mode:=sensor` 可在 Gazebo GUI 中循环展示正前方命中、yaw/pitch 命中和无返回
  fail-closed；完整使用、接口、诊断和实机接入步骤见 `docs/USAGE.md`。
- `models/` 统一保存 MRS/PX4 x500、PX4 plane 及各自的光电吊舱版本；演示已从独立载荷试验台
  升级为 `x500_gimbal` 旋翼机。规范吊舱模型保留全部上游飞行插件，无 PX4 演示另用同目录
  明确标识的 `sensor_demo.sdf` 夹具；既有 MRS/PX4 运行路径保持不变。
- 两轴控制核心覆盖限位、限速、速度 watchdog、非法命令、无反馈和启动期命令保留；演示与
  Gazebo 自动验收均通过公共 `GimbalCommand` 控制实际关节。
- 包内 Gazebo JointController PID 插件替代会暂停全局物理的通用 pose trajectory 插件；完整
  PX4/MRS 飞行复验在云台三阶段后保持 connected/armed/OFFBOARD 和约 1.52 m 稳定悬停，随后
  land 成功解锁。
- dev 后续增加的四个正式目标测试工具已放入本包并加入安装与 smoke test；旧副本和
  Typhoon/旧 `gm_control` 专用系统适配器不属于 detect 核心，未引入新的机型耦合。

M8 后 `run_tests_xd_uav_detect` 汇总 45 tests、0 errors、0 failures、0 skipped。

真实设备尚未验证。设备话题、时间戳质量、相机—激光外参和噪声参数必须由实际驱动与标定提供。
