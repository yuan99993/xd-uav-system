# xd_uav_ego_bridge

EGO-Swarm 与 XD-UAV 通用接口之间的 ROS1 适配包。它负责 EGO 私有消息、坐标、点云和
健康检查，不启动 planner，不管理 OFFBOARD/起降，也不拥有 canonical 控制权。

```text
输入  control_manager/state      xd_uav_controller/ControlState
输出  ego/odometry               nav_msgs/Odometry
输入  ego/position_command       quadrotor_msgs/PositionCommand
输出  ego/reference_candidate    mavros_msgs/PositionTarget
输出  ego/bridge/healthy         std_msgs/Bool
输出  ego/bridge/diagnostics     diagnostic_msgs/DiagnosticArray
```

点云 adapter 使用消息时间戳和 TF 转换到显式公共规划 frame，并发布独立 sensing health。
状态、frame、TF、时间戳、四元数或有限值校验失败时停止有效输出。不得通过修改 frame 标签
或未经标定的 identity TF 制造兼容。

```bash
roslaunch xd_uav_ego_bridge ego_bridge.launch UAV_NAME:=uav1
roslaunch xd_uav_ego_bridge ego_pointcloud_adapter.launch UAV_NAME:=uav1
```

只有 `xd_uav_system_integration` 的 reference mux 明确选择并通过健康门后，candidate 才能
进入 controller。
