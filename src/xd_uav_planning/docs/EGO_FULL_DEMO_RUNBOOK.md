# EGO 单机与三机演示完整使用手册

本文档对应 `xd_uav_planning/scripts/demo/ego_demo.sh`。脚本统一管理 Gazebo、RViz、
PX4/MAVROS、自研 estimator/manager/controller、EGO bridge 和 EGO-Planner。三机模式使用
真实 Gazebo world 与各机 Ouster 点云，并启用 EGO-Swarm 轨迹交换和互避。

第三方 `ego-planner-swarm` 保持官方 `92fe9f7` 原样。规划包通过外部轨迹交接中继处理后机
晚订阅、消息早于 odometry 以及前驱重规划刷新，不依赖第三方源码补丁。

## 1. 启动前检查

演示固定使用 `ROS_MASTER_URI=http://localhost:11311`，不能与 SEAD、MRS 或另一套 ROS
仿真同时运行。若 SEAD 仍在运行，先在 SEAD validation 目录执行 `./kill.sh`。

```bash
rosrun xd_uav_planning ego_demo.sh status
```

可选环境变量：

- `GUI=false`：不打开 Gazebo GUI。
- `RVIZ=false`：不打开 RViz。
- `XD_UAV_DEMO_READY_TIMEOUT=180`：调整 readiness 超时秒数。
- `XD_UAV_WS=/home/promise/catkin_ws`：显式指定 Catkin 工作区。

脚本会自动 source ROS Noetic 和工作区 overlay，不需要提前手工 source。通过 `rosrun`
启动时优先使用当前 `CMAKE_PREFIX_PATH` 中的非系统 Catkin overlay；直接执行源码脚本时会
向上查找外层工作区。若机器上同时存在多个工作区，用 `XD_UAV_WS` 明确指定所需工作区。

## 2. 单机演示

启动单机。以下两条命令等价，推荐统一入口：

```bash
rosrun xd_uav_planning ego_demo.sh start single
# 或简写
rosrun xd_uav_planning ego_demo.sh start
```

`start` 会等待 MAVROS、状态估计、GPS/local alignment、自研控制链、EGO odometry 和自动
起飞就绪后才返回。此时飞机保持起飞点，EGO 尚未取得 reference owner；收到有效目标并产生
轨迹后才会自动切换为 EGO。等待期间不要重复启动。

向 uav1 下发 `world` 坐标系目标：

```bash
rosrun xd_uav_planning ego_obstacle_demo.sh goal 6 0 1
```

单机目标必须使用上面的 `ego_obstacle_demo.sh goal X Y Z`。当前官方 EGO manual-target
后端只支持 `world z=1.0 m`；其他高度会由规划层明确拒绝。默认目标 `(6,0,1)` 已验证。

查看状态、降落和停止：

```bash
rosrun xd_uav_planning ego_demo.sh status
rosrun xd_uav_planning ego_demo.sh land
rosrun xd_uav_planning ego_demo.sh stop
```

## 3. 三机 EGO-Swarm 演示

启动验证过的三架 x500：

```bash
rosrun xd_uav_planning ego_demo.sh start swarm 3
```

`start multi 3` 是兼容别名。其他数量当前不属于正式验证配置。三机使用
`worlds/ego_multi_obstacles.world`，出生点、传感器和命名空间相互隔离；规划器 ID 为
0/1/2，并传递完整 `MultiBsplines` 前驱链。

三机当前显式使用经过实飞验证的 EGO preset 模式。启动完成前会自动执行以下 world 目标，
不需要另发目标命令：

- uav1：`(12,-4,1.5)`
- uav2：`(12,0,1.5)`
- uav3：`(12,4,1.5)`

`goal UAV X Y Z` 和 `goals formation` 是旧脚本兼容命令，但 preset 模式不会消费运行时目标，
因此不要用它们判断三机改点是否成功。三机动态改点将在后续切换到 manual-target swarm
模式并重新实飞验证后开放。

## 4. 应该观察什么

Gazebo 中应看到三架 x500 自动起飞，并在墙体和柱体组成的真实地图中运动。RViz 固定
frame 为 `world`，默认显示：

- 各机 odometry；
- Ouster 原始障碍点云和膨胀地图；
- EGO 目标、局部轨迹和执行轨迹；
- 三机交换后的 EGO-Swarm 轨迹。

三机应分别前往三个 preset 目标，而不是聚集到同一点。互避来自 EGO-Swarm 对其他飞机
时参数化 B 样条轨迹的约束，不是 bridge 中的临时位置偏移。

状态检查：

```bash
rosrun xd_uav_planning ego_demo.sh status
```

正常时三架 MAVROS 都应显示 `connected: True`、`armed: True`，飞行阶段通常为
`OFFBOARD`。`status` 只显示飞控摘要；更详细的启动失败原因看运行日志。

## 5. 正常停止与立即清理

推荐先降落，再停止整套仿真：

```bash
rosrun xd_uav_planning ego_demo.sh land all
rosrun xd_uav_planning ego_demo.sh stop
```

也可只降落一架：

```bash
rosrun xd_uav_planning ego_demo.sh land uav2
```

`land` 不关闭 Gazebo、RViz 或 ROS 节点；`stop` 才会关闭脚本拥有的 ROS graph，并按
INT、TERM、KILL 分级回收 roslaunch、Gazebo、PX4 和 MAVROS。需要立即结束演示时可以
直接执行：

```bash
rosrun xd_uav_planning ego_demo.sh stop
```

该停止命令同时兼容单机和三机：检测到三机实例时停止三机，否则转交单机入口。

停止后核验：

```bash
rosrun xd_uav_planning ego_demo.sh status
ps -eo pid,ppid,stat,cmd | rg '[r]oslaunch|[r]osmaster|[g]zserver|[g]zclient|[p]x4|[m]avros'
```

`status` 应报告 demo 未运行，进程检查中不应再有本演示进程。

## 6. 日志与常见问题

- 单机日志：`/tmp/xd_uav_ego_demo/roslaunch.log`
- 三机日志：`/tmp/xd_uav_ego_multi_demo/roslaunch.log`

### 启动提示已有 ROS graph

说明 `localhost:11311` 已被 SEAD、MRS 或另一套演示占用。先用对应脚本正常停止原仿真，
不要在同一 ROS master 上叠加启动。

### 提示找不到 `devel/setup.bash`

先确认工作区已经完成编译且 `/home/promise/catkin_ws/devel/setup.bash` 存在。默认目录不是
该路径时，可执行：

```bash
XD_UAV_WS=/实际/catkin工作区 rosrun xd_uav_planning ego_demo.sh start swarm 3
```

`XD_UAV_WS` 填工作区根目录，不要填 `src`、`devel` 或包目录。

### Gazebo 正常但 RViz 没有点云

先确认 RViz fixed frame 是 `world`，再检查三机 `planning/healthy`。三机模式依赖 Gazebo
Ouster 的真实传感器点云，不会启动人工点云发布器。

### 下发目标后不移动

先运行 `status`，随后检查日志中 estimator、local alignment、EGO bridge 和 planner 是否
健康。单机目标必须使用 `world` 且当前高度必须为 `1.0 m`；超出地图规划范围或落在障碍物
内的目标可能无法生成安全轨迹。三机 preset 演示当前不接受运行时改点。

### stop 后 Gazebo 仍存在

再次执行同一个 `rosrun xd_uav_planning ego_demo.sh stop`，再用上面的进程命令确认。不要
直接删除 runtime 目录，因为其中的 PID 文件用于脚本识别和回收其所属进程。

## 7. 功能边界

当前正式验收覆盖一架或三架四旋翼、Gazebo Ouster 障碍感知、自研控制闭环，以及三机
EGO-Swarm 轨迹交换和互避。它不等同于固定翼 SEAD 禁飞区功能，也不验证 FAST-LIO、真机
传感器、真实通信链路或任意数量无人机。
