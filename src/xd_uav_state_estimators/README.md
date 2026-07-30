# xd_uav_state_estimators

多来源无人机状态估计包。除`world`外，节点、话题和坐标系均遵循`uavX`命名空间。

## 模块划分

本包包含两个相互独立的模块：

- `odometry_adapter_manager_node`：按照配置把多个`nav_msgs/Odometry`输入转换成六种独立修正消息。
- `multi_source_estimator_node`：融合修正量、维护来源健康状态、切换主源并输出连续状态。

适配器本身不决定某个修正量属于哪个定位源。管理节点从`sources.yaml`一次加载全部
适配器，估计器再在同一个文件中自由组合它们的输出。

六类标准修正话题为：

```text
/uavX/state_estimator_inputs/<适配器名称>/position_xy
/uavX/state_estimator_inputs/<适配器名称>/position_z
/uavX/state_estimator_inputs/<适配器名称>/velocity_xy
/uavX/state_estimator_inputs/<适配器名称>/velocity_z
/uavX/state_estimator_inputs/<适配器名称>/heading
/uavX/state_estimator_inputs/<适配器名称>/yaw_rate
```

对应消息分别为`PositionXY`、`PositionZ`、`VelocityXY`、`VelocityZ`、
`Heading`和`YawRate`。适配器负责：

- 按`nav_msgs/Odometry`规范解释pose和twist。
- 使用TF把传感器参考点换算到`uavX/base_link`。
- 处理安装杆臂引起的线速度差异。
- 把速度转换到明确的参考坐标系。
- 补充缺失或非法的协方差，并把六个修正量分别发布。

## 定位源配置

`config/estimator.yaml`只保存滤波器、创新门限和切换策略。
`config/sources.yaml`保存Odometry适配器、定位源及修正输入。

```yaml
odometry_adapters:
  mavros:
    input_topic: mavros/local_position/odom
    body_frame: base_link
    parent_frame_override: mavros_origin
    child_frame_override: ""

  fastlio:
    input_topic: fastlio/Odometry
    body_frame: base_link
    parent_frame_override: fastlio_origin
    child_frame_override: lidar_imu_link

localization_sources: [mavros, fastlio]

sources:
  mavros:
    priority: 0
    correction_defaults:
      timeout: 0.25
      required: true
    corrections:
      - position_xy: mavros/position_xy
      - position_z: mavros/position_z
      - velocity_xy:
          topic: external_velocity/velocity_xy
          timeout: 0.30
          required: false
      - heading: mavros/heading
    alignment:
      mode: align_on_activation
    republish_in_frames: [local_origin, world]
```

规则如下：

- `odometry_adapters`中的每一项都会创建一个内部适配器。
- 适配器名称自动决定`state_estimator_inputs/<适配器名称>/*`输出路径。
- `body_frame`、`parent_frame_override`和`child_frame_override`中的简单名称会自动补上`uavX/`。
- `localization_sources`中的来源一定加载，不再使用每个来源的`enabled`。
- `priority`数值越小，自动选源优先级越高。
- 相对修正话题自动补成`state_estimator_inputs/<配置值>`。
- 字符串写法继承`correction_defaults`；映射写法可覆盖单项参数。
- 同一种修正类型可以配置多次，并分别来自不同适配器。
- `required: true`的修正量失效会使整个来源失效；可选修正失效只停止使用自身。

每个修正输入都有独立的超时、创新拒绝、连续异常隔离和稳定恢复状态。估计器不会暗中
使用未在`corrections`中列出的测量。

## 估计与切换

每个来源拥有独立的三轴位置、速度、加速度和航向滤波状态。main另外拥有一套持续存在
的滤波状态；切换来源只改变后续进入main的修正输入，不会用新来源重新初始化main。
IMU用于高频预测；定位修正先经过数值、坐标系、时间戳、协方差、绝对创新和NIS检查。

切换来源时，估计器计算来源原点相对标准`odom`的平移和航向对齐量。main原有的位置、
速度、加速度、航向和偏航角速度全部保留，再由新来源的后续测量逐步修正。因此即使新
来源没有配置速度修正，切换瞬间也不会把main速度清零。对齐量同时以普通消息发布，
交给单机TF管理器显示来源原点。

主TF由估计器独占发布：

```text
uavX/odom -> uavX/base_link
```

这样服务切换、自动故障接管和内部对齐会直接反映到标准控制TF上，并且不会与单机TF
管理器重复发布。

## 输出

以`uav1`为例：

```text
/uav1/state_estimator/main/odom
/uav1/state_estimator/main/acceleration
/uav1/state_estimator/main/frames/world/odom
/uav1/state_estimator/sources/<来源>/odom
/uav1/state_estimator/sources/<来源>/frames/world/odom
/uav1/state_estimator/sources/<来源>/valid
/uav1/state_estimator/sources/<来源>/alignment
/uav1/state_estimator/localization_valid
/uav1/state_estimator/state_valid
/uav1/state_estimator/status
/uav1/state_estimator/diagnostics
```

`sources/<来源>/odom`不是公共`odom`下的副本，而是该来源自身原点下的独立估计。例如：

```text
sources/mavros/odom：  uav1/mavros_origin  -> uav1/base_link
sources/fastlio/odom： uav1/fastlio_origin -> uav1/base_link
```

来源滤波器内部仍统一在`uav1/odom`中运行；发布单来源结果时会逆用该来源的对齐量，
恢复到来源原点下。单机TF管理器可把消息的`base_link`改名为
`mavros_estimated_base_link`或`fastlio_estimated_base_link`后广播，避免多个来源
争用标准`base_link`。

`republish_in_frames`控制每个来源的额外坐标系输出；
`main_republish_in_frames`控制主来源输出。只有标准主TF会被本包广播，坐标系重发布均为
`nav_msgs/Odometry`话题。

所有输出Odometry的twist都按消息规范在`child_frame_id`，即`base_link`中表达。
`main/acceleration`使用`geometry_msgs/AccelWithCovarianceStamped`，其线加速度在
`uavX/odom`中表达，已经过主滤波器估计且不包含重力。角加速度由IMU三轴角速度差分、
低通滤波后从机体系旋转到`uavX/odom`，因此线加速度和角加速度遵守同一个`frame_id`。

## 切换服务

```bash
rosservice call /uav1/state_estimator/switch_source "source_name: 'fastlio'"
rosservice call /uav1/state_estimator/switch_source "source_name: 'mavros'"
rosservice call /uav1/state_estimator/switch_source "source_name: 'auto'"
```

手动指定源失效时仍会临时选择其他健康源保证安全；指定源恢复后会重新接管。

## 启动

只启动配置文件中的全部Odometry适配器：

```bash
roslaunch xd_uav_state_estimators odometry_adapter_manager.launch \
  UAV_NAME:=uav1
```

只启动估计器：

```bash
UAV_NAME=uav1 roslaunch xd_uav_state_estimators estimator.launch
```

启动配置中的适配器、估计器和单机TF管理器：

```bash
UAV_NAME=uav1 roslaunch xd_uav_state_estimators uav_localization_stack.launch
```

如果上游已经直接发布六类标准修正消息，可添加`start_odometry_adapters:=false`。
增加新的Odometry算法时，只需修改`sources.yaml`中的`odometry_adapters`和`sources`，
不再修改launch。
共享world管理器在整个系统中只需另外启动一次。
