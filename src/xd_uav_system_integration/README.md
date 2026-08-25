# xd_uav_system_integration

XD-UAV 的组合层，负责 reference 仲裁、健康门、profile 校验以及 PX4/EGO/FAST-LIO 的显式
接线，不实现控制器、估计器或规划算法。

## 正式入口

- `world_registration.launch`：共享 world 注册。
- `reference_integration.launch`：baseline、健康合取和 reference mux。
- `px4_control_stack.launch`：PX4 状态估计、TF、controller/manager 和 reference 接线。
- `ego_px4_runtime.launch`：EGO 与 bridge。
- `ego_obstacle_demo.sh`：单机 EGO 障碍绕飞的 start/goal/status/land/stop 入口。
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

`start` 只有在 MAVROS、estimator、GPS/local alignment、manager 和 EGO 全链健康且自动
起飞完成后才返回。不要在 readiness 等待期间重复启动。RViz 默认显示无人机里程计、障碍
点云、膨胀地图和 EGO 轨迹；`stop` 会等待 MRS spawner、Gazebo、PX4/MAVROS 完整退出。
