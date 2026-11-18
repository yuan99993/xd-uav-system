# xd_uav_formation

本包提供领机跟随（`leader_follow`）和一致性编队（`consensus`）。默认 launch 启动一个 C++ 编队管理进程；一致性编队每次根据服务请求中的 `members` 创建本次所选飞机的控制器和遥测订阅，不受 YAML 成员数量限制。切换成员集合时，管理器先停止旧队伍，再按新名单重建；未选飞机不进入该队的状态、中心和槽位计算。

编队节点向各机的 `/<uav>/control/reference/setpoint` 发布位置和速度参考，不负责解锁、切换 OFFBOARD 或控制油门。领机跟随时，领机继续由自己的导航或轨迹输入控制。

## 启动

```bash
roslaunch xd_uav_formation formation.launch
```

默认使用 `config/formation_10_fixedwing.yaml` 和 `config/common.yaml`。YAML 为模式提供控制参数；一致性编队的成员名单必须在每次启动请求中指定，可以多于配置文件以前列出的成员数量：

```bash
rosservice call /formation/set_enabled "mode: 'leader_follow' enabled: true"
rosservice call /formation/set_enabled "{mode: 'consensus', members: ['uav1', 'uav3', 'uav8'], enabled: true}"
```

`consensus` 至少选择两架飞机，`members` 的顺序用于分配槽位。重新请求另一组成员时会先停止旧组再启动新组；停止当前所有编队仍可省略模式和成员。

一次只启用一种模式。每架飞机仍有位于其命名空间下的编队状态服务和参考话题，但整个包只运行一个编队管理进程；不需要为每架飞机单独运行 `roslaunch`。C++ 管理器使用 ROS 原生异步线程池（至少 2 个线程，CPU 核数较多时使用约一半核心），定时控制和遥测回调不受 Python GIL 限制。停止当前所有编队控制器不需要指定模式或成员：

```bash
rosservice call /formation/set_enabled "enabled: false"
```

也可以使用专用停止服务：

```bash
rosservice call /formation/stop "{}"
```

## 配置

默认机队配置 [formation_10_fixedwing.yaml](config/formation_10_fixedwing.yaml) 同时包含 `leader_follow` 和 `consensus` 两节。领机跟随部分配置领机名称、僚机列表和相对领机航向的 `[前向, 左向, 上向]` 米制偏移；一致性部分配置队形形状、间距、共同航线和控制参数，不配置成员名单。`FormationControl.srv` 请求中的 `members` 是本次一致性编队的唯一成员来源。

[common.yaml](config/common.yaml) 保存两种模式共用的控制周期、状态超时、固定翼速度限制、追赶增益等参数。机队文件中的同名参数会覆盖公共值。更换机型或队伍时，复制并修改机队 YAML，再通过 launch 参数加载：

```bash
roslaunch xd_uav_formation formation.launch \
  config:=/path/to/your_formation.yaml \
  common_config:=/path/to/your_common.yaml
```

一致性编队中的 `common_frame`、航点和高度应按当前仿真/飞行场景修改。默认航线仅作演示，启动一致性模式前请替换为当前世界坐标系中的安全航线。领机跟随不使用一致性编队的航点。

一致性编队的 `shape` 支持以下自动槽位布局，成员顺序按服务请求 `members` 中的顺序分配：

| `shape` | 队形 | 槽位说明 |
| --- | --- | --- |
| `circle` | 圆形 | 成员均匀分布在环上 |
| `rectangle_compact` | 紧凑矩形/网格 | 按成员数量生成紧凑网格 |
| `line` | 一字横队 | 沿航向左右展开，首个成员位于左侧；`line_abreast` 是别名 |
| `column` | 纵队 | 固定翼按航线弧长前后错开并共用同一路径，首个成员位于最前；`single_file` 是别名 |
| `arrow` | 箭头/V形 | 首个成员位于尖端，其余成员从左侧开始交替分配到两翼；`vee`、`v_shape` 是别名 |

这些布局由本次选中的成员数实时生成；脱队后剩余成员会重新分配对应数量的槽位。`manual` 仍只用于领机跟随的自定义偏移，不是可用于一致性编队的自动形状。

### 运行时选择队形和航线

管理器接收两路标准 ROS 消息，话题名称固定且不包含队形名：

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/formation/shape` | `std_msgs/String` | `circle`、`rectangle_compact`、`line`、`column` 或 `arrow`；也接受上表中的别名 |
| `/formation/path` | `nav_msgs/Path` | 按顺序排列的航点；`header.frame_id` 指明坐标系，空的单点 frame 使用 Path 的 frame |

例如，发送箭头队形：

```bash
rostopic pub -1 /formation/shape std_msgs/String "data: 'arrow'"
```

航点先变换到 `common_frame`，然后按收到的点列作为实际航线使用；该消息代表最终路径，不会再按 YAML 的 `corner_radius` 改写。`nav_msgs/Path` 本身没有循环标志，因此运行时路径沿用 YAML 配置的 `loop_path`；固定翼配置默认开启循环，管理器会在末点后自动接回首点，不要求发送端重复首点。若需要开放路径，应在 profile 中将 `loop_path` 设为 `false`。闭合航线应提供足够密集、转弯平滑的路径点。

固定算法参数从 `common.yaml` 与当前 profile 读取；配置文件缺少必需的数值参数时，管理器会报出缺少的参数并停止启动，不再静默回退到代码默认值。航点仍可通过 `/formation/path` 在运行时更新。

两条指令可以在启动一致性编队前发送，也可以在编队运行中发送。运行中收到有效更新后，所有成员会清除旧集结计划，并按新队形或航线重新集结。未发送指令时，继续使用 YAML 中的默认队形和航线。固定话题面向当前一致性编队管理器；形状名称不需要拼入话题名。

`common.yaml` 列出了当前统一 launch 可调的共用参数和管理器参数：

| 参数 | 用途 |
| --- | --- |
| `service_timeout` | 管理器等待成员启停服务的最长时间 |
| `rate` | 编队参考更新频率 |
| `state_timeout`、`mavros_timeout`、`tf_max_age` | 状态、MAVROS 和 TF 的最大允许过去时间 |
| `future_stamp_tolerance` | 状态、编队中心和 TF 时间戳允许超前 ROS 当前时间的范围；当前配置为 30 秒 |
| `tf_lookup_timeout`、`tf_buffer_cache_time` | TF 查询等待时间和缓存时长 |
| `external_quiet_time` | 启动前等待外部参考静默的时间 |
| `minimum_slot_distance`、`fixedwing_minimum_slot_distance` | 领机跟随时限制僚机槽位距领机的距离；一致性编队时检查成员槽位之间的最小间距 |
| `fixedwing_min_speed`、`fixedwing_max_speed` | 固定翼编队使用的速度范围，也用于计算转弯时的速度余量 |
| `fixedwing_max_catchup_speed` | 领机跟随时，落后僚机最多增加的水平速度 |
| `multirotor_max_speed` | 多旋翼参考水平速度上限 |
| `fixedwing_min_groundspeed_for_course` | 固定翼地速低于此值时，若空速有效且安全，则用机头方向估算队形中心航向 |
| `catchup_gain`、`catchup_ki`、`catchup_kd` | 槽位追赶 PID 的 P、I、D 增益；`catchup_deadband`、`catchup_integral_limit` 和 `catchup_derivative_filter_time_constant` 控制死区、积分限幅和微分滤波 |
| `multirotor_max_catchup_speed` | 多旋翼追赶修正速度上限 |
| `yaw_offset` | 机头偏角修正 |
| `reference_topic`、`team_cancel_topic`、`preempt_topic`、`task_goal_topic` | 参考、队伍取消和任务接管话题；队伍取消话题中的 `{team_id}` 会替换成配置的队伍标识 |

每个机队 YAML 的 `consensus` 段还可独立设置集结阶段参数：

| 参数 | 用途 |
| --- | --- |
| `acquire_consensus_gain` | 集结/合并阶段的位置误差增益，用于横向收敛和沿航线速度追赶；进入 `following` 后改用 `consensus_gain` |
| `acquire_max_consensus_velocity` | 集结/合并阶段允许的横向修正速度上限，单位 m/s |
| `acquire_max_speed_adjustment` | 集结/合并阶段追赶或等待时，相对航线速度的最大速度调整量，单位 m/s |

这些参数只作用于 `separating`、`gathering`、`merging` 阶段；可在各自机队 YAML 的 `consensus` 段独立调整。未配置时分别沿用 `consensus_gain`、`max_consensus_velocity`；各机队文件中的当前值可以分别设置。

固定翼编队（包括处于固定翼前飞阶段的 VTOL）在集结时保持共享计划中的统一航向，先收拢队形，再合并高度并进入航线。各机直接根据同一组成员状态计算槽位误差：落后加速、超前减速、横向偏离则向槽位收拢；到位后追赶修正逐渐归零。每架机独立按自身误差计算修正量，避免所有落后成员统一使用最大追赶量导致反复超调。水平参考使用速度制导，高度仍使用位置反馈，避免后方槽位位置使超前飞机掉头追点。收到成员主动脱队状态后，该成员不再参与中心和速度计算；其余成员按剩余数量重新分配槽位并重组队形。

一致性控制不设 6 架或 8 架的数量分支：新增成员使用同一套共享槽位、追赶和航线控制逻辑，按全队均值计算误差，不随成员数增加而改回环形邻居控制。成员数和成员顺序由每次服务请求的 `members` 决定；领航跟随的协同等待也不再因总数超过 6 架而关闭。当前仍为单队控制，尚未实现分组编队。

圆形队形的半径会随成员数增加而扩大；较大的队形需要更大的转弯空间和更宽的内外侧速度范围。每个节点仍接收所有成员状态，队伍规模增加也会增加总体通信和计算开销，因此取消代码数量限制不代表任意规模均能保持相同飞行效果。

固定翼一致性编队根据当前队形的最大槽位半径和 `fixedwing_min_speed`/`fixedwing_max_speed` 计算可用的转弯角速度，再与 `formation_heading_rate_limit` 取较小值。队形越宽，允许的转弯角速度越小，转弯会更缓；槽位旋转速度补偿在共同速度平滑后叠加，避免滤波削弱内外侧速度差。转弯时各机速度不同是保持队形所需，配置的 `cruise_speed` 是编队中心的基准速度。

以下参数也位于各机队 YAML 的 `consensus` 段：

| 参数 | 用途 |
| --- | --- |
| `formation_heading_rate_limit` | 航线飞行时槽位航向变化率的全局上限，单位 rad/s；固定翼一致性编队还会根据队形跨度和配置速度范围进一步限幅 |
| `formation_speed_slew_rate` | 单机水平速度指令变化率上限，单位 m/s² |
| `formation_course_slew_rate` | 单机水平航向指令变化率上限，单位 rad/s |
| `route_correction_gain` | 编队完成集结及高度合并后，队伍向航线收敛的位置误差增益 |
| `max_route_correction_velocity` | 队伍向航线收敛的横向速度修正上限，单位 m/s，与成员回槽修正分开限幅 |

领航跟随使用槽位位置误差的 PID 计算追赶速度；固定翼只沿槽位速度方向调整速度大小，横向回槽由位置制导处理。`catchup_gain`、`catchup_ki`、`catchup_kd` 分别是 P、I、D 增益。固定翼转弯槽位速度先由领机速度、转弯和槽位偏置计算，再额外叠加追赶额度 `fixedwing_max_catchup_speed`。飞机自身控制器和动力学决定实际能达到的速度。多旋翼参考速度受 `multirotor_max_speed` 限制。

`future_stamp_tolerance` 同时用于状态消息、编队中心和 TF 的时间戳检查。当前 [common.yaml](config/common.yaml) 设置为 30 秒；可在该文件中调整，修改后需重启统一 launch 才会被编队节点读取。代码中的默认值仅作为缺少配置时的后备值。节点启动日志会打印实际加载的超时和 TF 参数。

每种模式只建立一组全队状态、MAVROS、中心和状态话题订阅，再把消息放入共享缓存供各机控制器读取；控制周期仍按配置的 `rate` 更新每架飞机的参考。未启用的一致性成员降低中心状态的发布频率。高频输入使用最新值队列，状态心跳最多为 1 Hz，并在状态变化时立即发布。

默认 C++ 启动入口为 `formation.launch`，可执行文件为 `formation_manager_cpp`。原 Python 管理器和控制器仍完整保留，可用 `roslaunch xd_uav_formation formation_python.launch` 启动，供对照和回归参考；性能优化前的 Python 源码快照也保存在 [backup/python_before_optimization](backup/python_before_optimization/)。C++ 版本保持原控制算法、参数和 ROS 接口，减少了逐机复制的只读配置/槽位表，并通过共享遥测订阅与本地快照服务全队控制。当前未给出基准测试数据，因此不标注 CPU 降幅比例。

## 启动条件与状态

- 所有成员应有有效的本机状态、MAVROS 状态和公共 TF；坐标系和航点需一致。
- 同队飞机必须使用相同机型和飞行阶段，且不能处于阶段转换或起降等飞行动作中。
- 各编队成员须已解锁并处于 OFFBOARD。领机跟随的领机也须已解锁并有有效导航状态。
- 领机跟随不要求领机先到达导航目标；领机处于 `ACTION_NAVIGATE` 时可以启动。领机状态无效、未解锁或正在起降等阻塞动作时仍会拒绝启动。
- 固定翼须处于前飞状态，槽位距离须满足最小距离要求；VTOL/倾转旋翼成员须处于相同飞行阶段。

各机状态话题为 `/<uav>/formation_follower/status` 或 `/<uav>/formation_consensus/status`，可用于查看 `active`、`inactive` 及停止原因。外部导航/任务接管、状态或 TF 超时、失去 OFFBOARD 等情况会使编队节点停止发布参考。

外部控制接管某架飞机时，该机停止发布编队参考并通知其余成员将它排除；控制器在接受有效新目标后，会忽略该机连接中残留的旧编队消息，避免新目标被覆盖后因编队流中断而进入等待盘旋。只有显式重新启用对应编队节点，才允许它再次接管该机。领航跟随和一致性编队均使用此规则，不需要先停止整队。
