# xd_uav_single_tf_manager

每架无人机一个实例的 TF 管理包。

```text
uavX/local_origin -> uavX/odom -> uavX/base_link -> sensor frames
```

- 本包发布 `local_origin -> odom` 和配置的传感器安装 TF。
- `odom -> base_link` 由 state estimator 独占发布。
- `local_alignment` 支持 `body_pose` 与 `direct_transform` 两种修正输入。
- GPS 适配器可把 MAVROS 全球位置转换为标准全局修正；无全局修正时可以发布 identity
  fallback，但 valid 保持 false。
- child frame 冲突、非法 frame 或过大跳变会 fail-closed。

```bash
roslaunch xd_uav_single_tf_manager single_tf.launch UAV_NAME:=uav1
rostopic echo /uav1/single_tf_manager/local_alignment_valid
```

参数位于 `config/single_tf.yaml` 和 `config/gps_alignment.yaml`。
