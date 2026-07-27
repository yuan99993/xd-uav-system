# xd_uav_single_tf_manager

单机TF管理包。每架飞机在自己的`/uavX`命名空间中启动一个实例。

## TF职责

默认标准链为：

```text
uavX/local_origin
└── uavX/odom
    └── uavX/base_link
        └── uavX/lidar_link
            └── uavX/lidar_imu_link
```

所有权划分如下：

- `local_origin -> odom`：由本包发布。
- `odom -> base_link`：由状态估计器独占发布，本包不再转发。
- 雷达等安装TF：由本包的`static_transforms`发布。
- 各定位源自己的虚拟机体与来源原点：由`dynamic_transforms`发布。

默认还会得到：

```text
uavX/odom -> uavX/mavros_estimated_base_link
uavX/odom -> uavX/fastlio_estimated_base_link
uavX/odom -> uavX/mavros_origin
uavX/odom -> uavX/fastlio_origin
```

来源虚拟TF只用于观察和坐标变换，不会占用标准`base_link`，也不会参与控制器主输入。
节点启动时会检查所有child frame；两个规则占用同一个child时会直接停止，避免双父节点。

## local_origin到odom

本包不读取GPS、经纬度或磁航向。世界管理器只负责`world -> local_origin`，单机管理器
只负责局部层。

没有回环或全局修正时：

```yaml
local_alignment:
  enabled: true
  fallback_identity: true
  correction_topic: ""
```

此时发布单位变换`local_origin -> odom`，表示尚未发生局部校正，并保证标准TF链完整；
`local_alignment_valid`保持为`false`。

后续接入回环或全局修正时，`correction_topic`应提供`local_origin`下`base_link`的
`nav_msgs/Odometry`，`odometry_topic`提供主估计的`odom`下`base_link`。管理器计算：

```text
T_local_origin_odom =
    T_local_origin_base_link × inverse(T_odom_base_link)
```

结果通过跳变检查与平滑后发布。

## 自定义TF规则

静态安装关系：

```yaml
static_transforms:
  - name: base_to_sensor
    enabled: true
    parent_frame: base_link
    child_frame: sensor_link
    translation: [0.0, 0.0, 0.1]
    rotation_rpy: [0.0, 0.0, 0.0]
```

话题驱动的动态关系：

```yaml
dynamic_transforms:
  - name: slam_estimate
    enabled: true
    type: odometry
    topic: state_estimator/sources/slam/odom
    parent_frame: odom
    child_frame: slam_estimated_base_link
    invert: false
```

消息类型支持：

- `odometry`：`nav_msgs/Odometry`
- `pose_stamped`：`geometry_msgs/PoseStamped`
- `transform_stamped`：`geometry_msgs/TransformStamped`

不含`/`的frame会自动加当前飞机前缀。`parent_frame`和`child_frame`会覆盖消息自带名称，
使TF树所有权始终由配置明确决定。

## 启动与状态

```bash
UAV_NAME=uav1 roslaunch xd_uav_single_tf_manager single_tf.launch
```

```text
/uav1/single_tf_manager/local_alignment_valid
/uav1/single_tf_manager/diagnostics
```
