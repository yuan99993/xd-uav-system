# xd_uav_track

`launch/track.launch` 是本包的跟踪入口，默认加载唯一主配置 `config/track.yaml`。检测器、状态估计和飞控控制器由外部系统启动。

## 启动

沿用 YAML 中的 `follower/profile`：

```bash
roslaunch xd_uav_track track.launch UAV_NAME:=uav1
```

也可以在启动时覆盖 profile：

```bash
roslaunch xd_uav_track track.launch UAV_NAME:=uav1 follower_profile:=fw_metric_pursuit
roslaunch xd_uav_track track.launch UAV_NAME:=uav1 follower_profile:=fw_metric_orbit
```

`follower_profile` 留空时使用 `track.yaml` 的值。若想让后续启动都使用同一模式，修改该文件里的 `follower/profile` 即可。其它 profile 对应的参数也集中在这个文件中。

## 固定翼米制模式

- `fw_metric_pursuit`：使用米制目标做固定翼追踪/接近。
- `fw_metric_orbit`：围绕米制目标按 `follower/target_guidance/orbit/radius_m` 设定半径盘旋。

切换 profile 时，节点会按配置自动选择 `detect/detections` 和世界系里程计输入。两个模式都需要有效的 `xd_uav_track/MetricTarget`，默认输入话题是 `/<UAV_NAME>/track/metric_target`。观测超时或质量不满足门限时，节点按 `track.yaml` 的目标丢失策略处理。

Tracker 默认停止控制输出。确认检测器、里程计、控制器和目标输入都已就绪后，再启动跟踪：

```bash
rosservice call /uav1/track/start_tracker "start: true"
```

## `direct_control` 点到点模式

`direct_control` 使用共享世界坐标系中的 `MetricTarget`，不需要相机检测或本机 track ID。未进入到达半径时，按 `follower/target_guidance/direct_control/speed_mps` 以恒定三维速度朝目标飞行；进入 `arrival_radius_m` 配置的半径后切换到 XYZ 位置保持并报告 `arrived`。目标 Z 同样参与制导。

`task_execute` 会在执行期间持续刷新该目标，避免长距离飞行时目标过期。启用此 profile 前，必须确认 `target_pose.header.frame_id` 与飞机里程计共享同一坐标系。

```bash
rosservice call /uav1/track/set_profile "profile: 'direct_control'"
rosservice call /uav1/track/start_tracker "start: true"
rostopic pub -1 /uav1/track/metric_target xd_uav_track/MetricTarget \
  "{header: {frame_id: 'world'}, target_id: 1, valid: true, pose: {pose: {position: {x: 20.0, y: 5.0, z: 8.0}}}}"
```

正式任务中由 `xd_uav_task_execute` 设置 profile、启动 tracker 并发送目标点；`xd_uav_task_allocate` 可在目标分配时直接启动这一任务。
