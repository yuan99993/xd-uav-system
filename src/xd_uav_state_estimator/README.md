# xd_uav_state_estimator

> 此包保留为拆分前的参考实现。新的运行结构使用
> `xd_uav_state_estimators`、`xd_uav_sigle_tf_manager`和
> `xd_uav_world_tf_manager`；不要同时启动新旧估计器。

这是从MRS状态估计器思路中剥离出的ROS1无人机状态估计器。运行接口使用标准ROS消息，不依赖MRS manager、HW API、PX4 API、nodelet、MRS插件或`mrs_msgs`，目前只保留`mrs_lib::LKF`作为平移滤波器的数学后端。

本包的职责是从一个当前可靠的外部定位源获取状态，完成统一坐标、滤波、IMU高频预测和异常检查，再发布固定格式的标准状态。它不在内部融合多个定位源；如果需要视觉、激光和GNSS融合，应先在外部完成同步与融合，再把融合结果作为一个`nav_msgs/Odometry`定位源接入。

## 状态模型

- 水平状态：`[x, y, vx, vy, ax, ay]`
- 高度状态：`[z, vz, az]`，使用独立高度滤波器
- 航向状态：`[yaw, yaw_rate]`，使用独立角度滤波器并处理正负π跨越
- 横滚、俯仰：优先使用新鲜IMU姿态，定位源姿态作为回退
- 高频预测：经过坐标旋转、重力补偿和静止偏置估计的IMU加速度
- 状态修正：当前活动定位源的`nav_msgs/Odometry`
- 延迟修正：在固定时间窗内回退到测量时刻，修正后重放IMU预测

高度、水平位置、水平速度、垂直速度和航向可以针对每个定位源分别启用。例如，一个只提供平面SLAM的数据源可以关闭`use_altitude`和`use_vertical_velocity`。

## 启动与多机命名空间

```bash
cd /home/kzy/xd-uavsystem-test
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export UAV_NAME=uav1
roslaunch xd_uav_state_estimator local_odom.launch
```

多机时在不同终端或tmux pane中分别设置环境变量：

```bash
UAV_NAME=uav1 roslaunch xd_uav_state_estimator local_odom.launch
UAV_NAME=uav2 roslaunch xd_uav_state_estimator local_odom.launch
```

`UAV_NAME`不要包含开头的`/`。所有配置中的相对话题会自动解析到对应的`/uavX`命名空间。

## 定位源选择

定位源数量直接决定工作方式，不再配置`single`或`primary_fallback`模式：

- `localization_sources`中启用一个源时自然作为单源估计器。
- 启用多个源时按照`role`和`priority`自动选择活动源；主源超时、协方差异常或连续创新越界时切换到可靠备用源。

每个来源会经过`waiting -> healthy -> active`状态。连续异常后进入`quarantined`，隔离时间结束后仍需经过`recovering`稳定样本确认，不能用一条偶然恢复的消息立即接管。`minimum_active_time`和`cooldown`用于避免来源之间反复抖动。

运行时可以通过服务固定主输出使用的来源：

```bash
# main/odom切换到FAST-LIO；目标源必须已经健康且未被隔离
rosservice call /uav1/state_estimator/switch_source \
  "source_name: 'fastlio'"

# 取消手动指定，恢复按role和priority自动选择
rosservice call /uav1/state_estimator/switch_source \
  "source_name: 'auto'"
```

指定某个来源后，该来源保持为首选，不会被更高优先级来源立即抢回；若它失效，估计器仍会自动选择其他健康来源保证安全。它恢复健康后会重新成为主源。切换在目标源下一帧有效数据到达时完成，并使用该源的`alignment`策略保持`main/odom`连续。

所有定位源统一放在一个配置文件：

```bash
export UAV_NAME=uav1
roslaunch xd_uav_state_estimator local_odom.launch \
  localization_config:=$(rospack find xd_uav_state_estimator)/config/localization/localization.yaml
```

单源时只保留一个名称，多源时继续添加名称：

```yaml
switch_back_to_primary: true
source_switching:
  minimum_active_time: 1.0
  cooldown: 0.5

localization_sources: [vio, lidar_slam]

sources:
  vio:
    enabled: true
    adapter: odometry
    topic: vio/odometry
    role: primary
    priority: 0
    use_position_xy: true
    use_altitude: true
    use_velocity_xy: true
    use_vertical_velocity: true
    use_heading: true
    twist_in_body_frame: false
    timeout: 0.25
    alignment:
      mode: align_on_activation
      samples: 10
    frame_transform:
      use_tf_for_child_frame: true
      require_tf: true
      timeout: 0.02
    covariance:
      use_message: true
      position_xy: 0.02
      position_z: 0.05
      velocity_xy: 0.03
      velocity_z: 0.05
      heading: 0.03
    reliability:
      max_position_variance: 100.0
      max_velocity_variance: 100.0
      max_heading_variance: 10.0
      max_consecutive_rejections: 5
      quarantine_duration: 1.0
      recovery_min_samples: 20
      recovery_stable_time: 0.5

  lidar_slam:
    enabled: true
    adapter: odometry
    topic: liosam/mapping/odometry
    role: fallback
    priority: 1
    use_position_xy: true
    use_altitude: true
    use_velocity_xy: false
    use_vertical_velocity: false
    use_heading: true
    twist_in_body_frame: false
    timeout: 0.50
    alignment:
      mode: align_on_activation
      samples: 10
    frame_transform:
      use_tf_for_child_frame: true
      require_tf: true
      timeout: 0.05
```

`covariance`中的数值是消息没有提供有效协方差时使用的方差。估计器同时使用绝对创新门限和NIS统计门限，`reliability`用于拒绝明显失真的来源，而不是进行多源加权融合。

仓库默认的`config/localization/localization.yaml`已经包含`mavros_local`和`fastlio`两项。FAST-LIO配置订阅相对话题`fastlio/Odometry`，节点处于`/uav1`命名空间时即对应`/uav1/fastlio/Odometry`。由于FAST-LIO原始Odometry不提供稳定完整的协方差，该源默认使用配置中的固定测量方差，并且不使用其速度字段。

每个来源拥有独立的平移和航向滤波器，并发布：

```text
/uav1/state_estimator/sources/<source_name>/odom
/uav1/state_estimator/sources/<source_name>/valid
```

这些输出全部转换到统一的`uav1/odom`坐标系，便于直接比较。活动源另外修正具备延迟回溯和切换连续性处理的主滤波器，唯一主结果发布在`main/odom`。

### 坐标原点处理

- `identity`：输入已经位于估计器的`uavX/odom`坐标系。
- `align_on_first_measurement`：第一次使用时把该源的原点对齐到当前估计状态。
- `align_on_activation`：每次从待命切换为活动源时重新对齐，适合原点不同的备用定位源。

`alignment/samples`决定使用多少帧计算对齐。平移采用中位数，航向采用圆周均值，从而避免把第一帧噪声直接写入坐标偏移。对齐只校正坐标系间的航向差和平移差，不会把飞行器当时的横滚、俯仰误认为地图坐标系倾斜。

如果SLAM里程计的`child_frame_id`是`lidar_link`或`camera_link`，Adapter会查询TF并将传感器位姿、杆臂速度和角速度转换到`uavX/base_link`。启用`require_tf`后，缺少外参的消息会被拒绝，不会再把传感器frame直接当作机体frame。

## 延迟测量

`config/local_odom.yaml`中的配置：

```yaml
max_localization_delay: 0.10
delayed_measurement:
  enabled: true
  history_duration: 1.0
```

估计器保存最近一段平移滤波状态和IMU预测输入。SLAM消息时间戳落后于当前滤波时间时，会回退到对应历史状态、执行修正，再重放后续IMU输入。超过`max_localization_delay`或历史窗口的数据会被拒绝。航向会根据来源角速度补偿到当前时刻，避免用旧航向直接修正当前状态。

## 失效处理边界

估计器持续发布锁存话题：

```text
/uav1/state_estimator/localization_valid   std_msgs/Bool
/uav1/state_estimator/state_valid          std_msgs/Bool
/uav1/state_estimator/status               xd_uav_state_estimator/EstimatorStatus
```

只有活动定位源新鲜、未被隔离且所有滤波状态有限时，`localization_valid`才为`true`。短时间定位丢失时状态进入`DEAD_RECKONING`，仍可在`max_dead_reckoning_time`内输出预测状态；超过时限后进入`LOST`、`state_valid=false`并停止发布Odometry和TF。

结构化状态包括`WAITING`、`RUNNING`、`DEGRADED`、`DEAD_RECKONING`和`LOST`，同时提供活动来源、定位年龄、纯预测时间、切换次数和失败原因。估计器不会直接调用MAVROS降落服务；控制器或独立安全节点应使用`state_valid`和结构化状态决定返航、悬停或降落。

## 标准接口

使用`UAV_NAME=uav1`时：

- 输入 `/uav1/mavros/imu/data`（`sensor_msgs/Imu`）
- 输入 配置指定的定位话题（`nav_msgs/Odometry`）
- 可选输入 `/uav1/control_input`（`geometry_msgs/AccelStamped`）
- 输出 `/uav1/state_estimator/main/odom`（供控制器使用的主状态）
- 输出 `/uav1/state_estimator/sources/<source_name>/odom`（每个来源的独立滤波状态）
- 输出 `/uav1/state_estimator/sources/<source_name>/valid`（每个来源的独立有效性）
- 输出 `/uav1/state_estimator/localization_valid`（定位可用性）
- 输出 `/uav1/state_estimator/state_valid`（标准状态是否仍可用于控制）
- 输出 `/uav1/state_estimator/status`（结构化估计器状态）
- 输出 `/uav1/state_estimator/acceleration`（滤波加速度）
- 输出 `/uav1/state_estimator/imu_acceleration`（处理后的IMU预测输入）
- 输出 `/uav1/state_estimator/innovation`（位置和速度创新量）
- 输出 `/uav1/state_estimator/diagnostics`（总状态及每个来源状态）
- 服务 `/uav1/state_estimator/reset`（`std_srvs/Trigger`）
- 服务 `/uav1/state_estimator/switch_source`（指定主定位源或恢复`auto`）

查看当前活动源和可靠性：

```bash
rostopic echo -n 1 /uav1/state_estimator/diagnostics
rostopic echo /uav1/state_estimator/localization_valid
rostopic echo /uav1/state_estimator/status
```

## TF所有权

推荐结构：

```text
world -> uav1/local_origin -> uav1/odom -> uav1/base_link
```

当前PX4命名空间启动文件默认关闭MAVROS的local-position TF，状态估计器默认作为`uav1/odom -> uav1/base_link`的唯一发布者：

```bash
UAV_NAME=uav1 roslaunch xd_uav_state_estimator local_odom.launch
```

只有在不启动状态估计器、需要单独测试MAVROS TF时，才显式使用PX4启动参数`mavros_publish_tf:=true`，同时将估计器设为`publish_tf:=false`。同一个`odom -> base_link`只能有一个发布者。

`local_odom.launch`默认同时启动`gps_world_alignment_node`：

- `/uavX/mavros/global_position/global`提供WGS84位置。
- `/uavX/mavros/global_position/gp_origin`提供该飞机的PX4本地原点。
- `/uavX/mavros/global_position/compass_hdg`提供顺时针北向磁航向，并转换为ROS ENU yaw。
- `/uavX/state_estimator/main/odom`提供连续的本地机体位姿。

节点根据上述数据发布固定的`world -> uavX/local_origin`和缓慢校正的
`uavX/local_origin -> uavX/odom`。GPS噪声只改变全局对齐层，不直接改变控制器使用的
`odom -> base_link`。其锁存有效状态为：

```text
/uav1/gps_world_alignment/valid
```

共享世界坐标的WGS84基准配置在`config/gps_world_alignment.yaml`。仿真默认值与PX4
Zurich home一致；真机必须修改成任务场地统一基准。真机磁航向还应通过
`heading_offset`加入磁偏角和安装角修正。若不需要GPS全局层，可用
`enable_gps_world_alignment:=false`关闭；PX4单独运行时则可使用
`publish_static_global_tf:=true`恢复静态的两层TF。

估计器还会把各来源内部的坐标对齐结果发布为来源原点TF，例如：

```text
uav1/odom -> uav1/fastlio_origin
```

该变换来自`align_on_activation`计算的平移和航向差，不是固定零。FAST-LIO自身不再把真实`lidar_imu_link`挂到其原点下，因此不会与`base_link -> lidar_link -> lidar_imu_link`传感器安装树形成双父节点。可通过`publish_source_origin_tf`关闭这类来源原点TF。

## 接入雷达SLAM前的要求

1. SLAM发布带有效时间戳和协方差的`nav_msgs/Odometry`。
2. `header.frame_id`填写SLAM地图或里程计坐标系。
3. `child_frame_id`填写真实的`lidar_link`，不要伪装成`base_link`。
4. 通过URDF或`static_transform_publisher`提供`base_link <-> lidar_link`静态外参。
5. 如果SLAM不提供可信速度，将`use_velocity_xy`和`use_vertical_velocity`设为`false`。
6. 原点与PX4不同的SLAM使用`align_on_activation`和至少10帧对齐样本。
7. 避免SLAM和估计器同时发布同一个动态`odom -> base_link` TF。

## 编译和测试

```bash
cd /home/kzy/xd-uavsystem-test
catkin_make
catkin_make run_tests_xd_uav_state_estimator
catkin_test_results --verbose
catkin_make install
```

状态空间排列参考`mrs_uav_state_estimators`，保留的`mrs_lib`依赖采用BSD-3-Clause许可证。
