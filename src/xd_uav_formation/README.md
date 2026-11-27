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

`consensus` 至少选择两架飞机。默认 C++ 版本按 `members` 的顺序均衡分组，再根据当前位置用匈牙利算法分配槽位；请求顺序不再直接决定飞机占据哪个槽位。重新请求另一组成员时会先停止旧组再启动新组；停止当前所有编队仍可省略模式和成员。

一次只启用一种模式。每架飞机仍有位于其命名空间下的编队状态服务和参考话题，但整个包只运行一个编队管理进程；不需要为每架飞机单独运行 `roslaunch`。C++ 管理器使用 ROS 原生异步线程池（至少 2 个线程，CPU 核数较多时使用约一半核心），定时控制和遥测回调不受 Python GIL 限制。停止当前所有编队控制器不需要指定模式或成员：

```bash
rosservice call /formation/set_enabled "enabled: false"
```

也可以使用专用停止服务：

```bash
rosservice call /formation/stop "{}"
```

## 配置

默认机队配置 [formation_10_fixedwing.yaml](config/formation_10_fixedwing.yaml) 同时包含 `leader_follow` 和 `consensus` 两节。领机跟随部分配置领机名称、僚机列表和相对领机航向的 `[前向, 左向, 上向]` 米制偏移；一致性部分配置队形形状、间距、分组人数、共同航线和控制参数，不配置成员名单。`FormationControl.srv` 请求中的 `members` 是本次一致性编队的唯一成员来源。

[common.yaml](config/common.yaml) 保存两种模式共用的控制周期、状态超时、固定翼速度限制、追赶增益等参数。机队文件中的同名参数会覆盖公共值。更换机型或队伍时，复制并修改机队 YAML，再通过 launch 参数加载：

```bash
roslaunch xd_uav_formation formation.launch \
  config:=/path/to/your_formation.yaml \
  common_config:=/path/to/your_common.yaml
```

一致性编队中的 `common_frame`、航点和高度应按当前仿真/飞行场景修改。默认航线仅作演示，启动一致性模式前请替换为当前世界坐标系中的安全航线。领机跟随不使用一致性编队的航点。

一致性编队的 `shape` 支持以下自动槽位布局。下表中的首个槽位指几何布局中的索引，不代表请求中的首架飞机：

| `shape` | 队形 | 槽位说明 |
| --- | --- | --- |
| `circle` | 圆形 | 成员均匀分布在环上 |
| `rectangle_compact` | 紧凑矩形/网格 | 按成员数量生成紧凑网格 |
| `line` | 一字横队 | 沿航向左右展开，首个槽位位于左侧；`line_abreast` 是别名 |
| `column` | 纵队 | 固定翼按航线弧长前后错开并共用同一路径，首个槽位位于最前；`single_file` 是别名 |
| `arrow` | 箭头/V形 | 首个槽位位于尖端，其余槽位从左侧开始交替排列到两翼；`vee`、`v_shape` 是别名 |

这些布局用于组间和组内两层，形状保持一致；两层分别按小组数和组内成员数生成槽位。`group_size` 是每组目标人数，成员会按请求顺序均衡分组；设为 `1` 时每架飞机各自成组，不会跟随其他组员的外部控制点。`group_spacing: 0` 表示根据组内队形跨度自动增加组间间距；也可以设置一个更大的固定间距。`manual` 仍只用于领机跟随的自定义偏移，不是可用于一致性编队的自动形状。

默认 C++ 版本在集结前，以共同坐标系中的水平直线距离为代价做一对一匈牙利匹配。`group_size: 1` 时最小化全队到槽位的总水平距离；多机一组时先优化各组中心到组间槽位的总距离，再分别优化每组成员到组内槽位的总距离，保留原有组员关系。代价使用集结共同航向旋转后的槽位；固定翼优先采用当前成员飞行方向，多旋翼沿用航线航向。这是距离分配，不是带避障或固定翼转弯半径约束的航迹规划。

分配结果由所有 C++ 控制器共享，集结和航线飞行使用同一份对应关系，正常飞行时不随距离变化反复换位。更新队形或航线、调整分组或重新启动编队时重新匹配；普通成员失效只重新匹配本组剩余成员，保留其他组的对应关系及空组的组间槽位。`/formation/<team_id>/acquisition_plan` 参数中的 `members[i]` 与 `slots[i]` 可用于查看优化后的集结对应关系。保留的 Python 对照版本仍按成员顺序分配。

运行时可通过 `/formation/group_size` 话题调整目标组人数，消息类型为 `std_msgs/UInt32`。主编队成员会重新均衡分组并集结到新槽位；已因外部控制离队的小组保留原成员关系：

```bash
rostopic pub -1 /formation/group_size std_msgs/UInt32 "data: 5"
```

当前编队的完整结构化快照发布在 `/formation/<team_id>/state`，消息类型为
`xd_uav_formation/FormationState`，使用 latched 发布并每 0.5 秒刷新。消息包含队伍
ID、模式、运行状态、队形、统一航线、成员列表、组间/组内间距、目标组人数和版本号。
`groups` 数组逐组列出组 ID、`uavs` 成员名单、是否已离队、锚点 UAV、锚点是否由外部控制，以及可用时的外部控制点。
顶层 `members` 列出全队 UAV；每个分组的 `uavs` 列出该组成员，Track 可用自身 UAV ID
匹配 `groups[].uavs` 找到组 ID 和组员名单。

```bash
rostopic echo /formation/team1/state
```

一致性编队中收到组员的外部位置控制点后，该组会离开组间队形，外控 UAV 成为临时锚点。离队组沿用脱离前匈牙利匹配后的组内槽位顺序；主编队重新分配组间槽位时，不会清掉离队组的槽位记录。离队组成员失效或更换锚点时，剩余飞机保留原槽位角色，避免因成员数减少而突然交换相对位置。

多旋翼收到外部位置点后，组员在共同控制点高度上按组内形状调整 X/Y 槽位。固定翼前飞时，组员的 X/Y 槽位随锚机的实际位置、航向和转弯运动更新，并输出速度、水平加速度及转向前馈；不会把同一个外部位置点拆成各机各自独立盘旋的固定目标。固定翼僚机的高度和垂直速度也跟随锚机的实际状态，并转换到各自坐标系后下发；外部控制点的最终高度只作为锚机的导航目标，避免锚机正在平缓升降或盘旋时，僚机提前冲向最终高度。多旋翼仍使用共同控制点高度。位置控制点保存在组状态中；锚点失效时由同组健康成员接替前往该控制点，其余组员继续跟随新锚机的实际运动，不会自行切回主航线高度。外部路径、轨迹、任务动作或 `preempt` 也会使该组离队，组员跟随锚点的实际状态。其余小组沿共同航线飞行。主编队中普通单机失效只会触发本组成员重新分配组内槽位，组间槽位按小组计权，不会因该组飞机数量减少而重新分组其他小组。

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

两条指令可以在启动一致性编队前发送，也可以在编队运行中发送。运行中收到有效队形更新后，仍在主编队中的小组会清除旧集结计划并重新集结；已因外部控制离队的小组只更新组内形状。路径更新作用于仍在主编队中的小组；离队小组继续跟随其外控锚点。未发送指令时，继续使用 YAML 中的默认队形和航线。固定话题面向当前一致性编队管理器；形状名称不需要拼入话题名。

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

这些参数用于集结阶段，固定翼在 `separating` 阶段暂停水平回槽修正，在 `gathering`、`merging` 阶段启用；可在各自机队 YAML 的 `consensus` 段独立调整。未配置时分别沿用 `consensus_gain`、`max_consensus_velocity`；各机队文件中的当前值可以分别设置。

固定翼编队（包括处于固定翼前飞阶段的 VTOL）集结时，优先用当前成员实际飞行方向的圆周均值确定共同航向，并以相同航向做槽位匹配；成员航向相互抵消时，使用名单中首个具有有效速度的成员航向。不会为了对准最近航段而在开始组队时直接改变整队的目标方向。`separating` 阶段先沿共同航向飞行并完成高度分层，暂不施加水平回槽修正；进入 `gathering` 后再收拢，随后合并高度并进入航线。

集结的水平修正同时使用槽位位置误差和相对全队平均速度的阻尼：正在快速接近槽位时提前减小修正，减少越过槽位后才反向拉回的情况。进入正常航线跟随后使用旋转槽位的运动前馈和位置误差修正。水平参考使用速度制导，高度仍使用位置反馈，避免后方槽位位置使超前飞机掉头追点。组间航线和速度按小组中心计算，组内成员失效时只更新该组槽位；外部接管时离队小组不再参与组间中心和速度计算。匈牙利算法只分配槽位，不规划固定翼的入队航迹；初始航向不同的成员仍需通过必要的转弯完成对齐。

一致性控制不设 6 架或 8 架的数量分支。新增成员使用同一套槽位、追赶和航线控制逻辑，组间误差按小组中心均值计算。成员数和成员顺序由每次服务请求的 `members` 决定；领航跟随的协同等待也不再因总数超过 6 架而关闭。

圆形队形的半径会随成员数增加而扩大；较大的队形需要更大的转弯空间和更宽的内外侧速度范围。每个节点仍接收所有成员状态，队伍规模增加也会增加总体通信和计算开销，因此取消代码数量限制不代表任意规模均能保持相同飞行效果。

默认 C++ 固定翼一致性编队根据所有活动成员的实际槽位偏置，统一规划中心前进速度和转弯角速度。槽位在编队坐标系中的速度为 `(中心前进速度 - 角速度 × 横向偏置, 航线横向修正 + 角速度 × 纵向偏置)`。内侧槽位过慢时，先将共同前进速度提高到满足 `fixedwing_min_speed` 所需的值，同时检查所有槽位的水平速度不超过 `fixedwing_max_speed`；只有无法同时满足时才降低转弯角速度。规划还保守要求槽位的前向速度分量不低于最小速度，避免靠侧飞或反向飞行满足速度模长。

共同运动按所有槽位的速度余量和 `formation_speed_slew_rate` 分配，优先满足转弯角速度变化及必要的共同提速，再用剩余变化率余量修正航线，最后恢复 `cruise_speed`。出弯降低角速度时先保留共同速度，让内侧加速、外侧减速，避免共同减速占用外侧退弯所需的减速能力。后两项不会再通过同一个缩放比例拖慢角速度变化；每步均检查所有活动槽位的水平速度上限、前向速度下限及编队坐标系内的速度变化率。角加速度仍按 `formation_speed_slew_rate / 最大槽位半径` 限制，停止角速度的计算包含当前周期的航向变化，避免按无法兑现的退弯能力计算停止位置。

曲率前馈随共同前进速度缩放，角速度仍受 `formation_heading_rate_limit` 限制。当队形跨度和速度范围无法支持航线曲率时，实际转弯会放缓，需要更大的转弯空间。共同运动由全队共享，同一时间戳下发布中心和生成控制目标时复用该状态，避免重复计算把转弯角速度清零。

各机只对自身回槽修正在编队坐标系中限速，再叠加共同运动和槽位旋转速度。输出同时启用水平加速度 `AX/AY` 和 `yaw_rate` 前馈；加速度包含共同提速、角加速度及向心项，`yaw_rate` 从各机最终速度与加速度计算，不直接照搬编队航向角速度。最终水平速度按配置的固定翼最小/最大速度限制，限幅后同步修正加速度；回槽误差较大时仍可能触发这层保护。高度继续使用位置反馈，水平位置仍交给编队层误差修正。纵队保留沿航线弧长跟随的逻辑，并从各机平滑后的速度生成运动前馈。转弯时各机速度不同是保持队形所需，`cruise_speed` 是共同前进速度的基准值，转弯时可以按槽位速度约束提高。该改动只涉及编队参考生成，不修改下层控制器或飞控参数。

固定翼非纵队的正常跟随对前后、横向回槽均加入线性相对速度阻尼，修正为 `consensus_gain * 位置误差 - 滤波后的相对速度`，不使用速度平方的减速预测项。相对速度先扣除共同角速度作用于各机实际位置差的旋转速度，再投影到编队坐标系，避免把内外侧正常速度差当作追赶误差；正常跟随的速度阻尼滤波时间常数使用 `catchup_derivative_filter_time_constant`。集结阶段仍使用原有的相对速度阻尼。

回槽修正在叠加到各机名义槽位速度之前，就根据剩余速度余量限幅；滤波和变化率限制后的内部状态也投影回可用范围。合成速度的前向分量保持不低于 `fixedwing_min_speed`，水平模长不超过 `fixedwing_max_speed`，并按实际发出的修正变化计算加速度。这样避免先用回槽修正将名义速度抵消到很低，再只抬高最终速度模长，却保留低速下过大的转向前馈。最后的速度限幅仍作为保护保留。共同转弯提速规划、纵队弧长跟随及速度上下限配置保持原有设置。

以下参数也位于各机队 YAML 的 `consensus` 段：

| 参数 | 用途 |
| --- | --- |
| `formation_heading_rate_limit` | 航线飞行时槽位航向变化率的全局上限，单位 rad/s；固定翼一致性编队还会根据队形跨度和配置速度范围进一步限幅 |
| `formation_speed_slew_rate` | 共同运动中槽位在编队坐标系内的速度变化率上限，单位 m/s²；另用于限制各机回槽修正和队形转弯角加速度 |
| `formation_course_slew_rate` | 限制回槽修正造成的航向变化；纵队仍用该值平滑航向，单位 rad/s |
| `route_correction_gain` | 编队完成集结及高度合并后，队伍向航线收敛的位置误差增益 |
| `route_velocity_damping` | 整队回航线的接近速度阻尼，非负、无量纲；未配置时为 0；当前 `formation_10_fixedwing.yaml` 取 0.8，需结合飞行响应校准 |
| `max_route_correction_velocity` | 队伍向航线收敛的横向速度修正上限，单位 m/s，与成员回槽修正分开限幅 |

整队回航线修正为 `route_correction_gain * 横向误差 - route_velocity_damping * 滤波后的接近速度`。接近速度使用共同虚拟中心相对于移动航线前视点的运动，并扣除编队坐标系旋转产生的误差变化；不能直接使用世界坐标中的横向速度，否则正常弯道运动也会被当作需要抑制的偏差。滤波时间常数使用 `catchup_derivative_filter_time_constant`。该项用于在接近航线时提前收回共同横向修正；当前机队保留 `consensus_gain: 0.4`、`max_consensus_velocity: 10.0`、`route_correction_gain: 0.4`、`max_route_correction_velocity: 8.0` 的回槽和回航线能力，阻尼初值尚需飞行验证。修改 YAML 后需重启编队管理器读取。

领航跟随使用槽位位置误差的 PID 计算追赶速度；固定翼只沿槽位速度方向调整速度大小，横向回槽由位置制导处理。`catchup_gain`、`catchup_ki`、`catchup_kd` 分别是 P、I、D 增益。固定翼转弯槽位速度先由领机速度、转弯和槽位偏置计算，再额外叠加追赶额度 `fixedwing_max_catchup_speed`。飞机自身控制器和动力学决定实际能达到的速度。多旋翼参考速度受 `multirotor_max_speed` 限制。

`future_stamp_tolerance` 同时用于状态消息、编队中心和 TF 的时间戳检查。当前 [common.yaml](config/common.yaml) 设置为 30 秒；可在该文件中调整，修改后需重启统一 launch 才会被编队节点读取。代码中的默认值仅作为缺少配置时的后备值。节点启动日志会打印实际加载的超时和 TF 参数。

每种模式只建立一组全队状态、MAVROS、中心和状态话题订阅，再把消息放入共享缓存供各机控制器读取；控制周期仍按配置的 `rate` 更新每架飞机的参考。未启用的一致性成员降低中心状态的发布频率。高频输入使用最新值队列，状态心跳最多为 1 Hz，并在状态变化时立即发布。

默认 C++ 启动入口为 `formation.launch`，可执行文件为 `formation_manager_cpp`。原 Python 管理器和控制器仍完整保留，可用 `roslaunch xd_uav_formation formation_python.launch` 启动，供对照和回归参考；性能优化前的 Python 源码快照也保存在 [backup/python_before_optimization](backup/python_before_optimization/)。C++ 版本沿用原控制参数和 ROS 接口，增加了上述匈牙利槽位分配，减少了逐机复制的只读配置/槽位表，并通过共享遥测订阅与本地快照服务全队控制。当前未给出基准测试数据，因此不标注 CPU 降幅比例。

## 启动条件与状态

Gazebo 仿真中，先确认 `/use_sim_time` 已设为 `true` 且 `/clock` 已有消息，再启动编队进程。若编队进程早于 Gazebo 启动，它可能仍使用系统时间，而飞机状态已使用仿真时间，造成持续的 `follower_state_stale`。此时在仿真时钟就绪后重启编队 launch；仅重试启用服务或增大 `state_timeout` 无法纠正时钟来源。PX4 的三机、六机 tmux 仿真配置已在编队启动前等待首条 `/clock` 消息。

- 所有成员应有有效的本机状态、MAVROS 状态和公共 TF；坐标系和航点需一致。
- 同队飞机必须使用相同机型和飞行阶段，且不能处于阶段转换或起降等飞行动作中。
- 各编队成员须已解锁并处于 OFFBOARD。领机跟随的领机也须已解锁并有有效导航状态。
- 领机跟随不要求领机先到达导航目标；领机处于 `ACTION_NAVIGATE` 时可以启动。领机状态无效、未解锁或正在起降等阻塞动作时仍会拒绝启动。
- 固定翼须处于前飞状态，槽位距离须满足最小距离要求；VTOL/倾转旋翼成员须处于相同飞行阶段。

各机状态话题为 `/<uav>/formation_follower/status` 或 `/<uav>/formation_consensus/status`，可用于查看 `active`、`inactive` 及停止原因。外部导航/任务接管、状态或 TF 超时、失去 OFFBOARD 等情况会使编队节点停止发布参考。

外部控制接管某架飞机时，该机停止发布编队参考并通知其余成员将它排除；控制器在接受有效新目标后，会忽略该机连接中残留的旧编队消息，避免新目标被覆盖后因编队流中断而进入等待盘旋。只有显式重新启用对应编队节点，才允许它再次接管该机。领航跟随和一致性编队均使用此规则，不需要先停止整队。
