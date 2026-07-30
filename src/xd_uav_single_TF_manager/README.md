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
uavX/odom -> uavX/mavros_origin -> uavX/mavros_estimated_base_link
uavX/odom -> uavX/fastlio_origin -> uavX/fastlio_estimated_base_link
```

来源虚拟TF只用于观察和坐标变换，不会占用标准`base_link`，也不会参与控制器主输入。
节点启动时会检查所有child frame；两个规则占用同一个child时会直接停止，避免双父节点。

## local_origin到odom

TF计算核心不直接解析GPS。包内独立的`gps_global_alignment_node`负责把MAVROS全球
经纬度转换成标准`local_origin -> base_link`里程计；`single_tf_manager_node`只消费
标准修正消息并发布局部TF。

没有GPS、回环或其他全局修正时：

```yaml
local_alignment:
  enabled: true
  fallback_identity: true
  correction_topic: ""
```

此时发布单位变换`local_origin -> odom`，表示尚未发生局部校正，并保证标准TF链完整；
`local_alignment_valid`保持为`false`。

### body_pose输入模式

GPS、RTK、动捕、UWB等通常提供`local_origin`下`base_link`的位姿。配置：

```yaml
local_alignment:
  input_mode: body_pose
  correction_topic: global_alignment/gps/body_odometry
  correction_valid_topic: global_alignment/gps/valid
  odometry_topic: state_estimator/main/odom
```

`correction_topic`和`odometry_topic`都使用`nav_msgs/Odometry`。管理器计算：

```text
T_local_origin_odom =
    T_local_origin_base_link × inverse(T_odom_base_link)
```

结果通过跳变检查与平滑后发布。

消息坐标系必须是：

```text
修正消息：uavX/local_origin -> uavX/base_link
main消息：uavX/odom -> uavX/base_link
```

### direct_transform输入模式

带回环的SLAM或图优化模块可能已经直接提供`local_origin -> odom`。配置：

```yaml
local_alignment:
  input_mode: direct_transform
  correction_topic: localization_correction/local_to_odom
  correction_valid_topic: localization_correction/local_to_odom_valid
```

此时修正消息仍使用`nav_msgs/Odometry`，但坐标系必须是：

```text
uavX/local_origin -> uavX/odom
```

管理器不会订阅`odometry_topic`，直接检查、限跳和平滑该修正。启动时可传入
`gps_alignment_enabled:=false`关闭GPS适配器。

## GPS全局修正

默认启动GPS适配器，固定翼和旋翼使用同一组MAVROS话题：

```text
/uavX/mavros/global_position/gp_origin
/uavX/mavros/global_position/global
/uavX/mavros/imu/data
/uavX/mavros/estimator_status
```

其中`gp_origin`定义`local_origin`的WGS84经纬高，`global`提供飞机当前全球位置，
`imu/data`提供ROS ENU/FLU姿态。适配器使用GeographicLib统一转换ENU，输出：

```text
/uavX/global_alignment/gps/body_odometry
/uavX/global_alignment/gps/valid
/uavX/global_alignment/gps/diagnostics
```

适配器会检查GPS Fix、位置协方差、消息超时、经纬高有效性和运动学跳变。
`require_estimator_status`默认关闭以兼容仿真；真机确认PX4稳定发布估计状态后建议开启。

固定翼和旋翼是否真正可用不由机型名称决定，而由MAVROS是否加载`global_position`插件、
PX4是否具有有效全球位置以及上述`valid`输出决定。

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

关闭GPS适配器并等待其他直接修正：

```bash
UAV_NAME=uav1 roslaunch xd_uav_single_tf_manager single_tf.launch \
  gps_alignment_enabled:=false
```

```text
/uav1/single_tf_manager/local_alignment_valid
/uav1/single_tf_manager/diagnostics
/uav1/global_alignment/gps/body_odometry
/uav1/global_alignment/gps/valid
/uav1/global_alignment/gps/diagnostics
```
