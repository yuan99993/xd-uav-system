# xd_uav_world_tf_manager

全系统单实例的多机 world 注册包，发布：

```text
world -> uav1/local_origin
world -> uav2/local_origin
...
```

`config/world.yaml` 定义 world 基准、飞机清单和每机原点。原点模式包括：

- `gps_origin`：订阅 MAVROS GPS origin；
- `geodetic`：配置经纬高；
- `fixed`：直接配置 world 位置和 yaw，适合仿真。

```bash
roslaunch xd_uav_world_tf_manager world_tf.launch
```

本包只把各机 `local_origin` 接入共享 world，不发布机内 `odom -> base_link`。
