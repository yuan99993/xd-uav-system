# xd_uav_state_estimators

带命名空间的多来源状态估计包。

## 组件

- `odometry_adapter_manager_node`：把各来源 `nav_msgs/Odometry` 转成位置、速度、航向和
  yaw-rate 修正消息。
- `multi_source_estimator_node`：为每个来源维护独立状态，进行健康判断、隔离、恢复和主源切换。

配置分为 `config/sources.yaml` 和 `config/estimator.yaml`。每个 adapter 必须声明
`input_topic` 与 `reference_frame`；来源的必需修正超时后整源失效。主输出不会跨来源拼接
不同状态分量，切源时使用内部偏移保持连续。

## 输出与启动

```text
/uavX/state_estimator/main/odom
/uavX/state_estimator/main/acceleration
/uavX/state_estimator/status
/uavX/state_estimator/state_valid
/uavX/state_estimator/localization_valid
```

```bash
roslaunch xd_uav_state_estimators uav_localization_stack.launch UAV_NAME:=uav1
```

`odom -> base_link` 由 estimator 独占发布；其他包不得重复发布该 TF。
