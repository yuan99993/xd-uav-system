# xd_uav_world_tf_manager

全局多机world坐标管理包。整个ROS系统只启动一个实例，不放在任何`uavX`命名空间中。

## 配置世界

`config/world.yaml`定义：

- `world/frame_id`
- 世界WGS84基准经纬高，可手动配置或从话题读取
- 地理ENU平面在world中的位置
- 地理ENU平面相对world的航向
- 接入world的飞机名称列表
- 每架飞机的原点来源、额外位置偏移和航向偏移

飞机清单：

```yaml
vehicles: [uav1, uav2, uav3]
```

每架飞机对应：

```yaml
vehicle_configs:
  uav1:
    enabled: true
    mode: gps_origin
    origin_topic: /uav1/mavros/global_position/gp_origin
    local_origin_frame: uav1/local_origin
    position_offset: [0.0, 0.0, 0.0]
    yaw_offset: 0.0
```

修改清单后无需修改代码。

## 原点模式

`world/datum`决定地理ENU平面的WGS84基准点，也就是world地理原点。默认从配置读取：

```yaml
world:
  datum:
    mode: fixed
    latitude: -37.7177543
    longitude: 144.8345184
    altitude: -6.4093
```

也可以改为订阅话题（默认`/world/datum`）：

```yaml
world:
  datum:
    mode: topic
    topic: /world/datum
```

话题类型为`sensor_msgs/NavSatFix`，使用其中的WGS84经纬高；`STATUS_NO_FIX`消息会被忽略。如果发布端只在收到请求后发布一次，需要先启动world TF manager，再触发该请求，否则非 latched 的消息不会补发。收到有效消息前，依赖地理原点的`gps_origin`和`geodetic`飞机不会发布TF；收到后会计算并发布。运行中datum更新时，也会重新计算这些飞机的world位置。`fixed`模式下仍需配置经纬高。

- `gps_origin`：订阅`geographic_msgs/GeoPointStamped`并投影到共享world。
- `geodetic`：直接在配置中填写该飞机本地原点经纬高。
- `fixed`：直接配置world下的`fixed_position`和`fixed_yaw`，适合无GPS仿真。

节点发布：

```text
world -> uav1/local_origin
world -> uav2/local_origin
...
```

所有当前有效的静态TF会作为一个集合重新锁存，保证晚启动的TF监听器也能收到全部飞机。

## 启动

```bash
roslaunch xd_uav_world_tf_manager world_tf.launch
```

状态：

```text
/world_tf_manager/vehicles/uav1/valid
/world_tf_manager/diagnostics
```

## 多机启动顺序

```bash
# 全系统一次
roslaunch xd_uav_world_tf_manager world_tf.launch

# 每架飞机一次
UAV_NAME=uav1 roslaunch xd_uav_state_estimators uav_localization_stack.launch
UAV_NAME=uav2 roslaunch xd_uav_state_estimators uav_localization_stack.launch
```

每架飞机可以提供自己的估计器和单机TF配置文件，world管理器只负责把它们的
`local_origin`接入共享世界。
