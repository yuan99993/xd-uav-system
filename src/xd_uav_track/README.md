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
