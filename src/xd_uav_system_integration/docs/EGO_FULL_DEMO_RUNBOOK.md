# EGO 单机与三机演示完整使用手册

本文档对应 `xd_uav_system_integration/scripts/ego_demo.sh`。脚本统一管理 Gazebo、RViz、
PX4/MAVROS、自研 estimator/manager/controller、EGO bridge 和 EGO-Planner。三机模式使用
真实 Gazebo world 与各机 Ouster 点云，并启用 EGO-Swarm 轨迹交换和互避。

## 1. 启动前检查

演示固定使用 `ROS_MASTER_URI=http://localhost:11311`，不能与 SEAD、MRS 或另一套 ROS
仿真同时运行。若 SEAD 仍在运行，先在 SEAD validation 目录执行 `./kill.sh`。

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_system_integration
scripts/ego_demo.sh status
```

可选环境变量：

- `GUI=false`：不打开 Gazebo GUI。
- `RVIZ=false`：不打开 RViz。
- `XD_UAV_DEMO_READY_TIMEOUT=180`：调整 readiness 超时秒数。
- `XD_UAV_WS=/home/promise/catkin_ws`：显式指定 Catkin 工作区。

脚本会自动 source ROS Noetic 和工作区 overlay，不需要提前手工 source。

## 2. 单机演示

启动单机。以下两条命令等价，推荐统一入口：

```bash
scripts/ego_demo.sh start single
# 或简写
scripts/ego_demo.sh start
```

`start` 会等待 MAVROS、状态估计、GPS/local alignment、自研控制链、EGO bridge、EGO
planner 和自动起飞全部就绪后才返回。等待期间不要重复启动。

向 uav1 下发 `uav1/odom` 坐标系目标：

```bash
scripts/ego_obstacle_demo.sh goal 6 0 1
```

当前统一脚本的单机 `goal` 会转交给旧单机入口以外的命令只有 status/land/stop；因此单机
目标使用上面这条 `ego_obstacle_demo.sh goal X Y Z`。默认演示目标 `(6,0,1)` 已验证。

查看状态、降落和停止：

```bash
scripts/ego_demo.sh status
scripts/ego_demo.sh land
scripts/ego_demo.sh stop
```

## 3. 三机 EGO-Swarm 演示

启动验证过的三架 x500：

```bash
scripts/ego_demo.sh start swarm 3
```

`start multi 3` 是兼容别名。其他数量当前不属于正式验证配置。三机使用
`worlds/ego_multi_obstacles.world`，出生点、传感器和命名空间相互隔离；规划器 ID 为
0/1/2，并传递完整 `MultiBsplines` 前驱链。

发布预设编队目标：

```bash
scripts/ego_demo.sh goals formation
```

对应 world 坐标目标为：

- uav1：`(12,-4,1.5)`
- uav2：`(12,0,1.5)`
- uav3：`(12,4,1.5)`

单独修改某架飞机的 world 坐标目标：

```bash
scripts/ego_demo.sh goal uav2 12 2 1.5
```

格式是 `goal UAV X Y Z`，UAV 只能是 `uav1`、`uav2` 或 `uav3`。演示地图规划范围约为
x/y 各 `[-15,15] m`，目标必须留在范围内并避开实体内部；推荐终点高度 `1.5 m`。

## 4. 应该观察什么

Gazebo 中应看到三架 x500 自动起飞，并在墙体和柱体组成的真实地图中运动。RViz 固定
frame 为 `world`，默认显示：

- 各机 odometry；
- Ouster 原始障碍点云和膨胀地图；
- EGO 目标、局部轨迹和执行轨迹；
- 三机交换后的 EGO-Swarm 轨迹。

三机收到 `goals formation` 后应分别前往三个目标，而不是聚集到同一点。互避来自 EGO-
Swarm 对其他飞机时参数化 B 样条轨迹的约束，不是 bridge 中的临时位置偏移。

状态检查：

```bash
scripts/ego_demo.sh status
```

正常时三架 MAVROS 都应显示 `connected: True`、`armed: True`，飞行阶段通常为
`OFFBOARD`。`status` 只显示飞控摘要；更详细的启动失败原因看运行日志。

## 5. 正常停止与立即清理

推荐先降落，再停止整套仿真：

```bash
scripts/ego_demo.sh land all
scripts/ego_demo.sh stop
```

也可只降落一架：

```bash
scripts/ego_demo.sh land uav2
```

`land` 不关闭 Gazebo、RViz 或 ROS 节点；`stop` 才会关闭脚本拥有的 ROS graph，并按
INT、TERM、KILL 分级回收 roslaunch、Gazebo、PX4 和 MAVROS。需要立即结束演示时可以
直接执行：

```bash
scripts/ego_demo.sh stop
```

该停止命令同时兼容单机和三机：检测到三机实例时停止三机，否则转交单机入口。

停止后核验：

```bash
scripts/ego_demo.sh status
ps -eo pid,ppid,stat,cmd | rg '[r]oslaunch|[r]osmaster|[g]zserver|[g]zclient|[p]x4|[m]avros'
```

`status` 应报告 demo 未运行，进程检查中不应再有本演示进程。

## 6. 日志与常见问题

- 单机日志：`/tmp/xd_uav_ego_demo/roslaunch.log`
- 三机日志：`/tmp/xd_uav_ego_multi_demo/roslaunch.log`

### 启动提示已有 ROS graph

说明 `localhost:11311` 已被 SEAD、MRS 或另一套演示占用。先用对应脚本正常停止原仿真，
不要在同一 ROS master 上叠加启动。

### Gazebo 正常但 RViz 没有点云

先确认 RViz fixed frame 是 `world`，再检查三机 `ego/system_healthy`。三机模式依赖 Gazebo
Ouster 的真实传感器点云，不会启动人工点云发布器。

### 下发目标后不移动

先运行 `status`，随后检查日志中 estimator、local alignment、EGO bridge 和 planner 是否
健康。目标必须使用正确 frame：单机入口使用 `uav1/odom`，三机统一入口使用 `world`；
超出地图规划范围或落在障碍物内的目标可能无法生成安全轨迹。

### stop 后 Gazebo 仍存在

再次执行同一个 `scripts/ego_demo.sh stop`，再用上面的进程命令确认。不要直接删除 runtime
目录，因为其中的 PID 文件用于脚本识别和回收其所属进程。

## 7. 功能边界

当前正式验收覆盖一架或三架四旋翼、Gazebo Ouster 障碍感知、自研控制闭环，以及三机
EGO-Swarm 轨迹交换和互避。它不等同于固定翼 SEAD 禁飞区功能，也不验证 FAST-LIO、真机
传感器、真实通信链路或任意数量无人机。
