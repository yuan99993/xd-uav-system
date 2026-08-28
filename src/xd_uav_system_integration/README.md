# xd_uav_system_integration

XD-UAV 的组合层，负责 reference 仲裁、健康门、profile 校验以及 PX4/EGO/FAST-LIO 的显式
接线，不实现控制器、估计器或规划算法。

## 正式入口

- `world_registration.launch`：共享 world 注册。
- `reference_integration.launch`：baseline、健康合取和 reference mux。
- `px4_control_stack.launch`：PX4 状态估计、TF、controller/manager 和 reference 接线。
- `ego_px4_runtime.launch`：EGO 与 bridge。
- `ego_obstacle_demo.sh`：单机 EGO 障碍绕飞的 start/goal/status/land/stop 入口。
- `ego_demo.sh`：统一的单机/三机 EGO 演示入口；三机模式使用 Gazebo Ouster
  观测真实 world 障碍，不启动人工障碍点云节点。
- `docs/EGO_FULL_DEMO_RUNBOOK.md`：单机/三机启动、目标、观察、停止、日志和故障排查手册。
- `fastlio_ground_switch.launch`、`fastlio_shadow.launch`：FAST-LIO 影子验证与受控接入。

默认控制权为 `none`。候选来源必须同时满足消息新鲜度、健康和切换连续性约束，才能通过
`reference_mux_node.py` 发布 canonical setpoint。旧实验 profile、场景和分阶段脚本已删除，
不再作为产品入口。

```bash
catkin_make -j2 --pkg xd_uav_system_integration
```

单机演示会自动启动 Gazebo、RViz、PX4/MAVROS、状态估计、控制链和 EGO：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_system_integration
scripts/ego_obstacle_demo.sh start
scripts/ego_obstacle_demo.sh goal 6 0 1
scripts/ego_obstacle_demo.sh status
scripts/ego_obstacle_demo.sh land
scripts/ego_obstacle_demo.sh stop
```

三机 EGO-Swarm 正式演示：

```bash
scripts/ego_demo.sh start swarm 3
scripts/ego_demo.sh goals formation
scripts/ego_demo.sh goal uav2 12 2 1.5
scripts/ego_demo.sh status
scripts/ego_demo.sh land all
scripts/ego_demo.sh stop
```

不带模式时 `scripts/ego_demo.sh start` 保持单机行为。三机模式固定使用
`worlds/ego_multi_obstacles.world`，每架 x500 搭载 Gazebo Ouster。三架 EGO
使用连续 ID 0/1/2，按顺序传递完整 MultiBsplines 链，并启用 EGO-Swarm
轨迹互避；`start multi 3` 保留为 `start swarm 3` 的兼容别名。

`goals formation` 会分别向三机发布 `(12,-4,1.5)`、`(12,0,1.5)`、
`(12,4,1.5)` 的 world 坐标目标；外侧航线仍绕过真实墙体，但保留紧急恢复余量。也可以使用 `goal uavN X Y Z` 单独更新某架
飞机的目标；演示地图的规划范围约为 x/y 各 `[-15,15] m`，目标应留在范围内。
这里的避碰由 EGO-Swarm 使用各机交换的时参数化 B 样条轨迹完成，不是三套互不通信
的单机规划器。

`start` 只有在 MAVROS、estimator、GPS/local alignment、manager 和 EGO 全链健康且自动
起飞完成后才返回。不要在 readiness 等待期间重复启动。RViz 默认显示无人机里程计、障碍
点云、膨胀地图和 EGO 轨迹；`stop` 会等待 MRS spawner、Gazebo、PX4/MAVROS 完整退出。

## 停止与清理

正常演示结束时，推荐先让飞机受控降落，再关闭整套 ROS/PX4/Gazebo 环境：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_system_integration
scripts/ego_demo.sh land all
scripts/ego_demo.sh stop
```

`land all` 只命令三架飞机降落，不会关闭 Gazebo、RViz 或 ROS 节点；也可用
`land uav1` 只降落指定飞机。`stop` 才会关闭脚本拥有的 ROS graph，并依次等待
roslaunch、MRS spawner、Gazebo、PX4/MAVROS 退出；如果只想立即结束仿真，可以直接执行
`scripts/ego_demo.sh stop`。该命令同时兼容单机和三机：脚本检测到三机实例时关闭三机，
否则转交给单机停止入口。

停止后可执行：

```bash
scripts/ego_demo.sh status
ps -eo pid,ppid,stat,cmd | rg '[r]oslaunch|[r]osmaster|[g]zserver|[g]zclient|[p]x4|[m]avros'
```

`status` 应显示 demo 未运行，进程检查应无本演示残留。日志分别保存在
`/tmp/xd_uav_ego_multi_demo/roslaunch.log`（三机）和
`/tmp/xd_uav_ego_demo/roslaunch.log`（单机）。
