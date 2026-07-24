# xd_uav_sigle_tf_manager

单机TF管理包。目录和包名保留用户指定的`sigle`拼写。每架飞机在自己的`/uavX`
命名空间中启动一个实例。

## TF所有权

默认负责：

```text
uavX/local_origin
└── uavX/odom
    └── uavX/base_link
        └── uavX/lidar_link
            └── uavX/lidar_imu_link
```

- `local_origin -> odom`：无修正输入时发布单位变换；有局部修正输入时自动反算并平滑更新。
- `odom -> base_link`：直接消费一个`nav_msgs/Odometry`话题。
- 雷达安装TF：由`static_transforms`配置。
- 来源或其他算法TF：由`dynamic_transforms`配置。

节点启动时会检查所有child frame。两个规则配置同一个child会直接报错退出，避免双父节点。

单独启动本包时，`odom -> base_link`默认读取
`/uavX/mavros/local_position/odom`。通过整套定位launch启动时，会自动覆盖为
`/uavX/state_estimator/main/odom`，因此估计器切换主源后TF也随主源切换。

## 自定义话题驱动TF

在`config/single_tf.yaml`中增加：

```yaml
dynamic_transforms:
  - name: my_slam_origin
    enabled: true
    type: odometry
    topic: my_slam/odometry
    parent_frame: odom
    child_frame: my_slam_origin
    invert: false
```

支持的消息类型：

- `odometry`：`nav_msgs/Odometry`
- `pose_stamped`：`geometry_msgs/PoseStamped`
- `transform_stamped`：`geometry_msgs/TransformStamped`

不含`/`的frame会自动加当前飞机前缀，例如`odom`变成`uav2/odom`。话题也建议使用
相对名称，使配置可以在不同`uavX`之间复用。

`parent_frame`和`child_frame`会覆盖消息自带frame，确保TF树由配置明确控制。

## 局部校正层

本包不读取经纬度、GPS原点或磁航向，这些数据不属于单机局部TF管理职责。

没有局部修正算法时：

```yaml
local_alignment:
  enabled: true
  fallback_identity: true
  correction_topic: ""
```

此时持续发布单位变换`local_origin -> odom`，用于保证整条TF链连通。单位变换表示当前
还没有全局或回环校正，并不是伪造了一份校正结果；因此
`local_alignment_valid`仍为`false`。

如果后续接入回环、全局定位或其他局部校正算法，把`correction_topic`配置为其
`nav_msgs/Odometry`话题。该消息的pose必须表示`local_origin -> base_link`，管理器会结合
主里程计中的`odom -> base_link`计算：

```text
T_local_origin_odom =
    T_local_origin_base_link × inverse(T_odom_base_link)
```

随后对结果进行跳变检查和平滑，再发布`local_origin -> odom`。

## 启动

```bash
UAV_NAME=uav1 roslaunch xd_uav_sigle_tf_manager single_tf.launch
```

有效性和诊断：

```text
/uav1/single_tf_manager/main_transform_valid
/uav1/single_tf_manager/local_alignment_valid
/uav1/single_tf_manager/diagnostics
```
