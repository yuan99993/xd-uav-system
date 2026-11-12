# xd_uav_formation

本包提供领机跟随（`leader_follow`）和一致性编队（`consensus`）。用一个 launch 启动 YAML 中配置的成员节点，再通过 `/formation/set_enabled` 选择模式并启停。飞机成员、领机、槽位、航线和速度参数都在配置文件中维护。

编队节点向各机的 `/<uav>/control/reference/setpoint` 发布位置和速度参考，不负责解锁、切换 OFFBOARD 或控制油门。领机跟随时，领机继续由自己的导航或轨迹输入控制。

## 启动

```bash
roslaunch xd_uav_formation formation.launch
```

默认使用 `config/formation_three_fixedwing.yaml` 和 `config/common.yaml`。启动服务就绪后，只需指定模式并启用；成员从 YAML 读取：

```bash
rosservice call /formation/set_enabled "mode: 'leader_follow' enabled: true"
rosservice call /formation/set_enabled "mode: 'consensus' enabled: true"
```

一次只启用一种模式。切换模式时，管理器会先停止先前启用的队伍。停止当前所有编队节点不需要指定模式或成员：

```bash
rosservice call /formation/set_enabled "enabled: false"
```

也可以使用专用停止服务：

```bash
rosservice call /formation/stop "{}"
```

## 配置

默认机队配置 [formation_three_fixedwing.yaml](config/formation_three_fixedwing.yaml) 同时包含 `leader_follow` 和 `consensus` 两节。领机跟随部分配置领机名称、僚机列表和相对领机航向的 `[前向, 左向, 上向]` 米制偏移；一致性部分配置成员、队形、间距和共同航线。当前配置以 `uav1` 为领机，僚机为 `uav2`、`uav3`。

[common.yaml](config/common.yaml) 保存两种模式共用的控制周期、状态超时、固定翼速度限制、追赶增益等参数。机队文件中的同名参数会覆盖公共值。更换机型或队伍时，复制并修改机队 YAML，再通过 launch 参数加载：

```bash
roslaunch xd_uav_formation formation.launch \
  config:=/path/to/your_formation.yaml \
  common_config:=/path/to/your_common.yaml
```

一致性编队中的 `common_frame`、航点和高度应按当前仿真/飞行场景修改。默认航线仅作演示，启动一致性模式前请替换为当前世界坐标系中的安全航线。领机跟随不使用一致性编队的航点。

`common.yaml` 列出了当前统一 launch 可调的共用参数和管理器参数：

| 参数 | 用途 |
| --- | --- |
| `service_timeout` | 管理器等待成员启停服务的最长时间 |
| `rate` | 编队参考更新频率 |
| `state_timeout`、`mavros_timeout`、`tf_max_age` | 状态、MAVROS 和 TF 的最大允许过去时间 |
| `future_stamp_tolerance` | 状态、编队中心和 TF 时间戳允许超前 ROS 当前时间的范围；当前配置为 5 秒 |
| `tf_lookup_timeout`、`tf_buffer_cache_time` | TF 查询等待时间和缓存时长 |
| `external_quiet_time` | 启动前等待外部参考静默的时间 |
| `minimum_slot_distance`、`fixedwing_minimum_slot_distance` | 通用及固定翼最小槽位距离 |
| `fixedwing_min_speed`、`fixedwing_max_speed`、`multirotor_max_speed` | 各机型参考速度范围/上限 |
| `fixedwing_min_groundspeed_for_course` | 固定翼地速低于此值时，若空速有效且安全，则用机头方向估算队形中心航向 |
| `catchup_gain`、`max_catchup_speed` | 领机跟随位置误差的追赶补偿 |
| `max_turn_rate`、`yaw_offset` | 转弯补偿转率上限和机头偏角修正 |
| `reference_topic`、`team_cancel_topic`、`preempt_topic`、`task_goal_topic` | 参考、队伍取消和任务接管话题；队伍取消话题中的 `{team_id}` 会替换成配置的队伍标识 |

默认固定翼速度范围为 15–45 m/s，固定翼槽位距离至少 30 m。它们必须与实际机型、动力学模型和控制器限制相符；更换飞机后应先核对并修改这些值。

`future_stamp_tolerance` 同时用于状态消息、编队中心和 TF 的时间戳检查。当前 [common.yaml](config/common.yaml) 设置为 5 秒；可在该文件中调整，修改后需重启统一 launch 才会被编队节点读取。代码中的默认值仅作为缺少配置时的后备值。节点启动日志会打印实际加载的超时和 TF 参数。

## 启动条件与状态

- 所有成员应有有效的本机状态、MAVROS 状态和公共 TF；坐标系和航点需一致。
- 同队飞机必须使用相同机型和飞行阶段，且不能处于阶段转换或起降等飞行动作中。
- 各编队成员须已解锁并处于 OFFBOARD。领机跟随的领机也须已解锁并有有效导航状态。
- 固定翼须处于前飞状态，槽位距离须满足最小距离要求；VTOL/倾转旋翼成员须处于相同飞行阶段。

各机状态话题为 `/<uav>/formation_follower/status` 或 `/<uav>/formation_consensus/status`，可用于查看 `active`、`inactive` 及停止原因。外部导航/任务接管、状态或 TF 超时、失去 OFFBOARD 等情况会使编队节点停止发布参考。
