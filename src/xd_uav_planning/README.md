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
/<uav>/planning/status        xd_uav_task_allocate/PlannerStatus
/<uav>/planning/healthy       std_msgs/Bool
/<uav>/planning/diagnostics   diagnostic_msgs/DiagnosticArray
/<uav>/planning/cancel        xd_uav_task_allocate/CancelPlanning（两类飞机）
```

任务包现已使用 `planning/route_preview` 作为纯预览，固定翼执行输入只使用 `planning/task_path`。
任务层 publisher、状态入口、取消交接和 YAML 说明见
[任务层接入手册](docs/TASK_PLANNING_INTEGRATION.md)。

## 后端边界

### multirotor / EGO-Swarm

规划包负责 EGO 的目标校验、状态/轨迹 bridge、点云适配、健康门、引用仲裁和
`PlannerStatus`。EGO 私有 `PositionCommand` 只有在 frame、时间戳、载机状态和健康条件有效后
才进入 controller。

仓库中的 EGO manual-target 入口已解除固定 `z=1.0` 的硬编码，保留请求中的有限 XYZ。官方 sequential swarm 的瞬时
`MultiBsplines` 交接由本包 `swarm_handoff_relay.py` 外部增强：锁存并周期重发启动链，同时根据
官方 `/broadcast_bspline` 更新完整前驱轨迹。这样后机即使晚订阅或首条消息早于 odometry，也
不会永久卡在 `SEQUENTIAL_START`，无需修改第三方源码。

EGO 的内部 frame 固定为 `world`；规划层仍严格拒绝错误 frame 或非有限坐标，但允许地图范围内
的任意三维目标高度。

### fixedwing / Path

固定翼后端不启动 EGO，也不读取点云：

```text
planning/task_path -> control/reference/path
controller/path_status -> planning/status
```

后端校验固定翼 `ControlState`、frame、时间戳、有限数值、路径点数和最小线段长度，并把
controller 私有 path ID 映射回任务 goal ID。它执行任务层提供的几何 Path，不宣称提供 EGO
避障或 SEAD 动态禁飞区重规划。

固定翼可在发 Path 前等待 `planning/healthy`；多旋翼的首个 EGO command 由首目标触发，因此
不能用该健康话题阻止多旋翼首目标发布。完整时序见接入手册。

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
```

验收完成后会请求受控降落，结果锁存在
`/uav1/planning/fixedwing_acceptance/result`。

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
`xd_uav_task_allocate` 是只读上游；`ego-planner-swarm` 是只读第三方依赖，项目适配全部位于
本包。旧 `xd_uav_system_integration` 与 `xd_uav_sead` 仅作为 legacy 源码保留，不属于正式
`task_allocate -> planning -> controller` 链路。

## 旧 EGO bridge 恢复

独立的旧 `xd_uav_ego_bridge` 已由本包取代，不再保留在当前源码树中。需要查看或恢复旧包时，
使用兼容提交 `6c50f8eb3726f765b0162c2e7477c2adda1311f1`；该提交同时包含旧 bridge、
原样保留的 `xd_uav_system_integration` 以及已经完成移植的 `xd_uav_planning`。例如只查看旧包：

```bash
git show 6c50f8eb3726f765b0162c2e7477c2adda1311f1:src/xd_uav_ego_bridge/package.xml
```
