# xd_uav_state_estimators

多定位源状态估计包。节点必须运行在`/uavX`命名空间中，只发布状态消息，不发布任何TF。

## 职责

- 为每个定位源维护独立的三轴位置/速度/加速度Kalman状态和航向状态。
- 使用IMU比力和角速度进行高频预测、重力补偿及异常加速度限制。
- 根据绝对创新和NIS拒绝异常修正。
- 对连续异常来源执行`quarantined -> recovering -> healthy`恢复流程。
- 根据`primary/fallback`、优先级、超时和服务请求选择主来源。
- 激活新来源时重新计算平移/航向对齐，保持`main/odom`连续。
- 发布每个来源、主来源、状态有效性、诊断以及来源对齐消息。
- 可查询已有TF并额外发布`base_link`在`world`等指定坐标系下的Odometry话题。

本包不会广播`odom -> base_link`或来源原点TF。TF所有权全部交给
`xd_uav_sigle_tf_manager`。

## 两类配置

- `config/estimator.yaml`：滤波器、IMU、创新门限、失效预测和切换策略。
- `config/sources.yaml`：来源名称、话题、使用的状态维度、协方差、外参和可靠性。

增加定位来源时只编辑`sources.yaml`：

```yaml
localization_sources: [mavros_local, fastlio, vio]

sources:
  vio:
    enabled: true
    topic: vio/odometry
    role: fallback
    priority: 2
    timeout: 0.3
    use_position_xy: true
    use_altitude: true
    use_velocity_xy: true
    use_vertical_velocity: true
    use_heading: true
    twist_in_body_frame: true
```

所有相对话题都解析在当前`/uavX`命名空间下。

## 输出

以`uav1`为例：

```text
/uav1/state_estimator/main/odom
/uav1/state_estimator/sources/<name>/odom
/uav1/state_estimator/sources/<name>/valid
/uav1/state_estimator/sources/<name>/alignment
/uav1/state_estimator/localization_valid
/uav1/state_estimator/state_valid
/uav1/state_estimator/status
/uav1/state_estimator/diagnostics
```

`alignment`是`geometry_msgs/TransformStamped`消息，只是供单机TF管理器消费的普通话题。

## 指定坐标系下的位置话题

在`sources.yaml`中配置：

```yaml
frame_outputs:
  - name: world
    enabled: true
    target_frame: world
    topic: state_estimator/frames/world/odom
    lookup_timeout: 0.02
```

输出为：

```text
/uav1/state_estimator/frames/world/odom
```

该功能只查询`world -> base_link`，不会发布或修改TF。

## 切换来源

```bash
rosservice call /uav1/state_estimator/switch_source "source_name: 'fastlio'"
rosservice call /uav1/state_estimator/switch_source "source_name: 'mavros_local'"
rosservice call /uav1/state_estimator/switch_source "source_name: 'auto'"
```

手动来源失效时仍会临时使用其他健康来源；指定来源恢复后会重新接管。

## 启动

只启动估计器：

```bash
UAV_NAME=uav1 roslaunch xd_uav_state_estimators estimator.launch
```

启动一架飞机的估计器和单机TF管理器：

```bash
UAV_NAME=uav1 roslaunch xd_uav_state_estimators uav_localization_stack.launch
```

共享world管理器应在整个系统中另外启动一次。
