# xd_uav_system_integration

XD-UAV 的系统接线层，负责 PX4/Gazebo 启动、共享 world 注册、状态估计与控制栈组合、
FAST-LIO 接入和仿真辅助节点；不实现任务分配、规划算法或控制律。

主要入口：

- `px4_control_stack.launch` / `px4_control_stack_guarded.launch`
- `px4_gazebo_single.launch`
- `world_registration.launch`
- `fastlio_ground_switch.launch` / `fastlio_shadow.launch`

统一规划层（EGO-Swarm 四旋翼后端、固定翼 Path 后端、规划健康门和 reference 仲裁）已迁入
`xd_uav_planning`。EGO 单机/多机演示也从该包启动：

```bash
rosrun xd_uav_planning ego_demo.sh start single
rosrun xd_uav_planning ego_demo.sh start swarm 3
```

本包仍被规划层作为通用 PX4、估计器、控制链和 Gazebo 启动底座使用。

```bash
catkin_make -j2 --pkg xd_uav_system_integration
```
