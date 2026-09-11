# xd_uav_planning

XD-UAV 的统一 ROS1 规划层。任务层只向本包发送目标或任务路径，本包按机型选择唯一后端，
校验后再向 controller 输出执行参考。本包已经自包含其演示所需的 PX4/Gazebo、状态估计和
控制栈组合资源，不依赖旧 `xd_uav_system_integration` 或 `xd_uav_sead`。

```text
xd_uav_task_allocate -> xd_uav_planning -> xd_uav_controller
```

## 正式入口

`planning.launch` 只启动规划层，不启动仿真、PX4/MAVROS、状态估计、controller、自动起飞或
任务发布器。

```bash
# 多旋翼：EGO-Swarm backend
roslaunch xd_uav_planning planning.launch \
  UAV_NAME:=uav1 vehicle_type:=multirotor \
  common_frame:=world output_frame:=uav1/odom

# 固定翼：Path backend，不启动任何 EGO 节点
roslaunch xd_uav_planning planning.launch \
  UAV_NAME:=uav1 vehicle_type:=fixedwing \
  common_frame:=world
```

正式公共接口：

```text
/<uav>/planning/goal          geometry_msgs/PoseStamped（多旋翼输入）
/<uav>/planning/task_path     nav_msgs/Path（固定翼执行输入）
/<uav>/planning/route_preview nav_msgs/Path（任务层原始路线预览）
/<uav>/planning/adjusted_path nav_msgs/Path（固定翼禁飞区调整后路径）
/planning/no_fly_zones         xd_uav_planning/NoFlyZoneArray（全机共享禁飞区输入）
/planning/no_fly_zone_markers  visualization_msgs/MarkerArray（RViz 禁飞区显示）
/<uav>/planning/cancel        xd_uav_task_allocate/CancelPlanning（两类飞机）
/<uav>/planning/status        xd_uav_task_allocate/PlannerStatus
/<uav>/planning/healthy       std_msgs/Bool
/<uav>/planning/diagnostics   diagnostic_msgs/DiagnosticArray
```

任务包现已使用 `planning/route_preview` 作为纯预览，固定翼执行输入只使用 `planning/task_path`。
任务层 publisher、状态入口、取消交接和 YAML 说明见
[任务层接入手册](docs/TASK_PLANNING_INTEGRATION.md)。

## 后端边界

### multirotor / EGO-Swarm

规划包负责 EGO 的目标校验、状态/轨迹 bridge、点云适配、健康门、引用仲裁和
`PlannerStatus`。EGO 私有 `PositionCommand` 只有在 frame、时间戳、载机状态和健康条件有效后
才进入 controller。

EGO 的内部 frame 固定为 `world`；规划层拒绝错误 frame 和非有限坐标。收到新目标后先关闭
参考输出，只有该目标产生时间戳不早于目标接收时刻的新 EGO candidate，才获取 reference mux
所有权并重新开放输出，避免上一条轨迹的缓存命令泄漏。`owner_acquisition_timeout` 默认 5 s，
超时会明确上报失败。首次从 `none` 获取 EGO owner 可由 `allow_initial_ego_owner` 放行，后续
SEAD/EGO owner 切换仍执行位置和速度跳变检查。

EGO 地图大小、高度边界和动态约束可从正式入口配置。默认恢复为 `30×30×8 m`、
`max_vel=3.5 m/s`、`max_acc=2.5 m/s²`，点云默认使用本工作区的 `fastlio/points` 话题；
滚动地图默认开启，需要固定地图时可传入 `rolling_map_enabled:=false`。占据地图
在飞机接近水平边界前重置有限体素缓存并将窗口中心移动到当前位置；任务目标、TF 和轨迹
继续使用 `world` 坐标。`rolling_map_margin_m` 默认 6 m，局部更新范围恢复为
18×18×10 m，障碍物膨胀半径为 0.65 m。重定位会清空旧占据缓存，必须依靠后续传感器数据
重建，因此实机启用前需要验证重定位期间的障碍重建和轨迹安全性。

官方 sequential swarm 的瞬时 `MultiBsplines` 交接仍由本包 `swarm_handoff_relay.py` 外部增强：
锁存并周期重发启动链，同时根据 `/broadcast_bspline` 更新完整前驱轨迹，避免后机因晚订阅或
首条消息早于 odometry 而永久卡在 `SEQUENTIAL_START`。

### fixedwing / Path

固定翼后端不启动 EGO，也不读取点云。它使用通过 ROS 话题收到的禁飞区，在任务 Path 首次
下发前以及飞行中的每次合法禁飞区更新后进行固定翼转弯约束路径调整：

```text
planning/no_fly_zones --┐
planning/task_path ------+-> initial/online no-fly planning -> planning/adjusted_path
ControlState -----------┘                                  -> control/reference/path
                                                           (atomic Path replacement)
controller/path_status -> planning/status
```

后端校验固定翼 `ControlState`、frame、时间戳、有限数值、路径点数和最小线段长度，并把
controller 私有 path ID 映射回任务 goal ID。禁飞区与 Path 必须使用同一个 `common_frame`；
受阻路径按配置的最小转弯半径和净距生成 Dubins 绕飞路径，最终连续路径校验失败时返回
`BLOCKED`，不向 controller 下发原始冲突路径。飞行中 UPSERT、REMOVE 或 CLEAR 会从当前
实测位置截取原任务的剩余段，重新计算并以一条新 Path 原子替换 controller 当前路径；沿原
任务的进度只前进不后退。任务层可调用 `planning/cancel` 释放当前 planning 控制权；在线
重规划无安全解时仍调用 control manager 的 `cancel_offboard` 失效保护，禁止继续执行已知
冲突旧路径。

禁飞区可以在任务 Path 前或飞行中发布。所有固定翼后端共享同一个全局话题，一条消息可以携带多个
区域。数组头部的 `frame_id` 会作为未填写区域头部的默认坐标系。下面示例一次新增两个永久矩形禁飞区：

```bash
rostopic pub -1 /planning/no_fly_zones xd_uav_planning/NoFlyZoneArray "{header: {stamp: now, frame_id: 'world'}, zones: [{schema_version: 1, operation: 0, zone_id: 7101, enabled: true, zone_type: 0, min_altitude: 0.0, max_altitude: 100.0, valid_until: {secs: 0, nsecs: 0}, polygon: {points: [{x: 80.0, y: -20.0, z: 0.0}, {x: 110.0, y: -20.0, z: 0.0}, {x: 110.0, y: 20.0, z: 0.0}, {x: 80.0, y: 20.0, z: 0.0}]}}, {schema_version: 1, operation: 0, zone_id: 7102, enabled: true, zone_type: 0, min_altitude: 0.0, max_altitude: 100.0, valid_until: {secs: 0, nsecs: 0}, polygon: {points: [{x: 160.0, y: -20.0, z: 0.0}, {x: 190.0, y: -20.0, z: 0.0}, {x: 190.0, y: 20.0, z: 0.0}, {x: 160.0, y: 20.0, z: 0.0}]}}]}"
```

删除 `zone_id=7101`，数组中只放一个 REMOVE 更新：

```bash
rostopic pub -1 /planning/no_fly_zones xd_uav_planning/NoFlyZoneArray "{header: {stamp: now, frame_id: 'world'}, zones: [{schema_version: 1, operation: 1, zone_id: 7101, enabled: false, zone_type: 0, min_altitude: 0.0, max_altitude: 0.0, valid_until: {secs: 0, nsecs: 0}, polygon: {points: []}}]}"
```

`operation=2, zone_id=0` 清空全部区域；也可以把多个 UPSERT、REMOVE 或 CLEAR 更新放在同一个数组中，
后端按数组顺序处理，并只触发一次在线换路。
`valid_until=0` 表示在 REMOVE/CLEAR 前永久有效。禁飞区 `header.stamp` 仅作消息元数据，
不会因为为 0 或消息延迟而拒绝；消息使用 `dubins` Python 模块；当前工作区
系统 Python 已具备该模块，但 Ubuntu/ROS 的 rosdep 数据库没有 `python3-dubins` 安装键，部署
新机器时需由系统镜像或项目依赖清单显式提供。

固定翼可在发 Path 前等待 `planning/healthy`；多旋翼的首个 EGO command 由首目标触发，因此
不能用该健康话题阻止多旋翼首目标发布。完整时序见
[任务层接入手册](docs/TASK_PLANNING_INTEGRATION.md)，禁飞区协议、在线换路、配置与验收记录见
[固定翼动态禁飞区规划说明](docs/FIXEDWING_DYNAMIC_NOFLY.md)。

## 演示

单机 EGO：

```bash
rosrun xd_uav_planning ego_demo.sh start single
rosrun xd_uav_planning ego_obstacle_demo.sh goal 6 0 1
rosrun xd_uav_planning ego_demo.sh status
rosrun xd_uav_planning ego_demo.sh land
rosrun xd_uav_planning ego_demo.sh stop
```

三机 EGO-Swarm：

```bash
rosrun xd_uav_planning ego_demo.sh start swarm 3
rosrun xd_uav_planning ego_demo.sh status
rosrun xd_uav_planning ego_demo.sh land all
rosrun xd_uav_planning ego_demo.sh stop
```

三机 launch 使用已经验证的 preset 目标。`land all` 只负责降落，必须继续执行 `stop` 才会
关闭 ROS、PX4 和 Gazebo。完整操作见 [EGO 演示手册](docs/EGO_FULL_DEMO_RUNBOOK.md)。

固定翼 SITL：

```bash
# 带 Gazebo GUI
roslaunch xd_uav_planning fixedwing_sitl_demo.launch gui:=true

# 无 GUI 自动验收
roslaunch xd_uav_planning fixedwing_sitl_demo.launch gui:=false

# 真实话题依次发布禁飞区和300 m任务Path，并验收实际绕飞
roslaunch xd_uav_planning fixedwing_sitl_demo.launch \
  gui:=false enable_nofly:=true path_length:=300

# 飞行中真实新增禁飞区，飞机完成绕飞并越过安全边界后再删除，显示规划可视化
roslaunch xd_uav_planning fixedwing_sitl_demo.launch \
  gui:=true dynamic_nofly:=true visualize:=true path_length:=300 \
  zone_center_fraction:=0.45
```

验收完成后会请求受控降落，结果锁存在
`/uav1/planning/fixedwing_acceptance/result`。可视化同时显示任务路线、历次替换路线、当前
controller 路线、实际航迹和活动禁飞区；关闭或停止节点时会在
`/tmp/xd_uav_planning_visualizations/<时间>/` 保存 `summary.png`、`events.json` 和
`trajectory.csv`。无显示器时自动使用 Agg 后端，仍会保存验收图。

## 目录

```text
launch/planning.launch   唯一正式产品入口
launch/internal/         只供 planning.launch 组合的内部组件
launch/internal/runtime/ planning 演示使用的 PX4、估计器与控制栈组合
launch/demo/             SITL 和演示组合
config/                  正式后端配置
config/demo/             仿真、RViz 和演示专用配置
config/runtime/          planning 演示使用的底层接线配置
scripts/                 正式运行节点
scripts/demo/            演示管理和验收脚本
scripts/runtime/         planning 演示使用的运行辅助节点
docs/                    操作及上下层接入手册
```

起降、OFFBOARD、状态估计和底层控制仍分别属于 control manager、estimator 和 controller。
`xd_uav_task_allocate` 是只读上游。`ego-planner-swarm` 仅带有 construction 分支移植来的最小
滚动地图补丁；其余项目适配位于本包。旧 `xd_uav_system_integration` 与 `xd_uav_sead` 仅作为
legacy 源码保留，不属于正式 `task_allocate -> planning -> controller` 链路。

## 旧 EGO bridge 恢复

独立的旧 `xd_uav_ego_bridge` 已由本包取代，不再保留在当前源码树中。需要查看或恢复旧包时，
使用兼容提交 `6c50f8eb3726f765b0162c2e7477c2adda1311f1`；该提交同时包含旧 bridge、
原样保留的 `xd_uav_system_integration` 以及已经完成移植的 `xd_uav_planning`。例如只查看旧包：

```bash
git show 6c50f8eb3726f765b0162c2e7477c2adda1311f1:src/xd_uav_ego_bridge/package.xml
```
