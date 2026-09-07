# xd_uav_planning

XD-UAV 的统一 ROS1 规划层。任务层只向本包发送目标或任务路径，本包按机型选择唯一后端，
校验后再向 controller 输出执行参考。

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
```

现有任务包的 `planning/mission_path` 是路线预览，不是固定翼执行输入。远端
`xd_uav_task_allocate dev@e763836` 仍把固定翼路径直接发布给 controller，因此两个包原样共启
尚未正式打通。任务层维护者需要修改的 publisher、状态入口、YAML 和验收步骤见
[任务层接入手册](docs/TASK_PLANNING_INTEGRATION.md)。

## 后端边界

### multirotor / EGO-Swarm

规划包负责 EGO 的目标校验、状态/轨迹 bridge、点云适配、健康门、引用仲裁和
`PlannerStatus`。EGO 私有 `PositionCommand` 只有在 frame、时间戳、载机状态和健康条件有效后
才进入 controller。

当前 EGO manual-target 只能执行 `world z=1.0 m` 的实时目标；其他高度会返回 `FAILED`，不会
静默飞错。恢复官方 EGO、外置 swarm 适配和任意三维目标属于下一阶段。

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
launch/demo/             SITL 和演示组合
config/                  正式后端配置
config/demo/             仿真、RViz 和演示专用配置
scripts/                 正式运行节点
scripts/demo/            演示管理和验收脚本
docs/                    操作及上下层接入手册
```

起降、OFFBOARD、状态估计和底层控制仍分别属于 control manager、estimator 和 controller。
`xd_uav_task_allocate` 是只读上游，本轮没有修改；`ego-planner-swarm` 是第三方依赖，本轮也没有
修改。
