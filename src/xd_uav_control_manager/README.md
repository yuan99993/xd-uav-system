# xd_uav_control_manager

统一状态适配、安全监督和 PX4 OFFBOARD 流程。节点启动后保持 `STANDBY`，不会自动切模式、
解锁或起飞。

## 状态机与服务

```text
STANDBY -> WAIT_STATE -> PRESTREAM -> REQUEST_OFFBOARD -> ACTIVE
ACTIVE -> LANDING -> disarm -> STANDBY
持续输入失效 -> FAILSAFE
```

```text
/uavX/control_manager/offboard
/uavX/control_manager/cancel_offboard
/uavX/control_manager/takeoff
/uavX/control_manager/land
/uavX/control_manager/land_home
/uavX/control_manager/cancel_land
/uavX/control_manager/reset_failsafe
```

管理器校验 odom、IMU、加速度、固定翼空速、PX4 状态和控制器输出，进入 ACTIVE 后才向
`/uavX/mavros/setpoint_raw/attitude` 发布。固定翼降落使用进近、下滑、拉平和滑跑，不使用
四旋翼垂直降落逻辑。

## 启动

```bash
roslaunch xd_uav_control_manager multirotor_system.launch UAV_NAME:=uav1
roslaunch xd_uav_control_manager fixedwing_system.launch UAV_NAME:=uav1
```

OFFBOARD 流程和安全参数分别在 `config/offboard.yaml`、`config/safety.yaml`。PX4 自身的
RC、datalink 和 OFFBOARD-loss 参数不由本包修改。
