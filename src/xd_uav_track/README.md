# xd_uav_track

将二维/三维检测候选关联成稳定轨迹，并生成多旋翼或固定翼速度制导参考。它不负责检测、
状态估计、OFFBOARD、起降或底层控制。

## 接口

```text
输入  /uavX/track/detections       xd_uav_track/DetectionArray
输入  /uavX/track/gimbal_state     xd_uav_track/GimbalState
输出  /uavX/track/tracks           xd_uav_track/TrackStateArray
输出  /uavX/track/status           xd_uav_track/TrackStatus
输出  /uavX/control/reference/setpoint  mavros_msgs/PositionTarget
服务  /uavX/track/select_track
服务  /uavX/track/set_profile
```

支持固定相机、云台和固定翼速度向量 profile。输入或估计器失效时停止发布；固定翼由控制器
在参考超时后进入 loiter。速度和可行性最终约束仍由 `xd_uav_controller` 负责。

```bash
roslaunch xd_uav_track track.launch UAV_NAME:=uav1
```
