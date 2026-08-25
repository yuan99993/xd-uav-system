# xd_uav_state_estimators

多来源无人机状态估计包。除`world`外，节点、话题和坐标系均遵循`uavX`命名空间。

## 模块划分

本包包含两个相互独立的模块：

- `odometry_adapter_manager_node`：按照配置把多个`nav_msgs/Odometry`输入转换成六种独立修正消息。
- `multi_source_estimator_node`：分别估计各来源的完整状态、维护来源健康状态、切换主源并输出连续状态。

适配器本身只负责标准化输入。管理节点从`sources.yaml`一次加载全部适配器，每个
定位源用自己列出的修正和公共IMU维护一套完整滤波状态；`main`不跨来源拼接状态分量。

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
    reference_frame: base_link

  fastlio:
    input_topic: fastlio/Odometry
    reference_frame: lidar_imu_link

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
- 适配器只读取输入Odometry的时间戳和数值，不依赖消息中的frame名称。
- 适配器名称自动生成`uavX/<名称>_origin`，`reference_frame`简单名称自动补上`uavX/`。
- `reference_frame`表示输入数值描述的机上参考点；适配器通过静态TF将其换算到`uavX/base_link`。
- `localization_sources`中的来源一定加载，不再使用每个来源的`enabled`。
- `priority`数值越小，自动选源优先级越高。
- 相对修正话题自动补成`state_estimator_inputs/<配置值>`。
- 话题中的第一段名称自动作为数据提供者，用来把该输入标准化到`odom`；通常每个
  来源只列出同名适配器的话题。Fast-LIO使用其自身ESKF发布的Odometry速度；确实
  没有速度测量的其他来源仍可由位置更新和公共IMU估计速度。
- 字符串写法继承`correction_defaults`；映射写法可覆盖单项参数。
- 同一种修正类型可以配置多次，并分别来自不同适配器。
- `required: true`的修正量失效会使整个来源失效；可选修正失效只停止使用自身。
- `align_on_activation`来源首次收齐必需修正时会立即对齐，即使它尚未成为主源。
- 任一必需修正超过自身`timeout`没有消息后，该来源会暂时失效，但不会立即
  改变来源原点。数据恢复并收齐完整修正后，估计器使用新观测重置运动状态，
  避免用断流期间的预测状态拒绝真实观测。
- 只有连续断流超过`reliability/session_reset_timeout`，才认为定位提供者可能
  已经重启并结束当前定位会话；重新接入后才重新建立来源原点。
- 来源原点在同一次定位会话内保持固定；切换main来源只改变测量选择，不会再次
  移动`<来源>_origin`。
- 切源时估计器另外计算一个只在main内部使用的handover偏移，使切换瞬间的main
  位置和航向连续；该偏移不发布为TF，也不改变任何来源坐标系。

每个修正输入都有独立的超时、创新拒绝、连续异常隔离和稳定恢复状态。估计器不会暗中
使用未在`corrections`中列出的测量。

## 估计与切换

每个来源拥有独立的三轴位置、速度、加速度和航向滤波状态。IMU用于各来源的高频预测；
定位修正先经过数值、坐标系、时间戳、协方差和绝对创新检查。NIS硬门控默认关闭；
因为碰撞、触地和急刹时恒加速度模型短时失配，并不表示定位传感器故障。只有显式设置
`innovation_gate/hard_reject_nis: true`后才使用NIS硬门控。`main`不再对活动
来源的position、velocity等字段做第二次融合，而是整包接管活动来源已经完成滤波的
状态和协方差。

切换来源时，估计器根据切换前main状态与新来源完整状态计算仅供main使用的连续性偏移。
位置和航向在切换瞬间保持连续，后续运动完全跟随新来源。Fast-LIO的位置和速度均来自
它自己的ESKF；没有速度修正的其他来源在首次接入时继承当时main的速度、加速度和偏航
角速度作为滤波初值，再由自身位置与公共IMU继续估计。来源原点对齐量仍以普通消息发布，
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
