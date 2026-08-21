# xd_uav_controller

四旋翼与固定翼控制器。节点只消费统一状态和参考，输出机体角速度与归一化推力/油门；
不调用 MAVROS 服务，也不负责 OFFBOARD、解锁或安全状态机。

## 接口

```text
输入  /uavX/control_manager/state         xd_uav_controller/ControlState
输入  /uavX/control/reference/setpoint    mavros_msgs/PositionTarget
输入  /uavX/control/reference/trajectory  trajectory_msgs/MultiDOFJointTrajectory
输出  /uavX/controller/command             xd_uav_controller/ControlCommand
内部  /uavX/controller/internal/command    xd_uav_controller/InternalCommand
```

`PositionTarget.header.frame_id` 必须有效；允许 frame、TF 时效和输入超时由
`config/common_config.yaml` 控制。四旋翼支持逐轴位置、速度和加速度组合；固定翼将有效
参考统一转换为 course、course-rate、空速、高度和爬升率。固定翼不能悬停，参考超时后进入
等待盘旋。

## 启动与测试

```bash
roslaunch xd_uav_controller controller.launch UAV_NAME:=uav1 vehicle_type:=fixedwing
catkin_make -j2 --pkg xd_uav_controller
```

公共、四旋翼和固定翼参数分别位于 `config/common_config.yaml`、`multirotor.yaml`、
`fixedwing.yaml`。正常操作只调用 control manager 的公开服务。
