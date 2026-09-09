# xd_uav_task_allocate

面向当前 XD UAV ROS1 工作空间的应急救援任务层。第一版采用中心协调器，负责：

- 将多个搜索多边形生成覆盖航线，并按预计飞行时间分给侦察机；四旋翼使用相邻折返，固定翼
  使用跨区跳行并在区域外插入满足最小转弯半径的 Dubins 连接；
- 直接读取 `xd_uav_detect` 的 `/uavX/track/detections`，无需启动 tracker 节点；
- 使用检测时间戳对应的 `world Odometry` 位姿，把 body FRD 目标转换到共享世界坐标；
- 对同机多帧和跨侦察机重复观测进行聚类，生成由本包维护的全局目标 ID；
- 自动侦察流程：纯四旋翼分配原始搜索区域；固定翼与四旋翼混合时，固定翼粗定位一旦形成
  稳定证据便立即发布局部区域，并派四旋翼按可配置范围逐区精搜；
- 只有精搜确认后的目标才建立救援任务，并按距离分给空闲作业机。

## 执行后端与避障边界

任务分配逻辑只决定“谁去哪里”。`mission.yaml` 中的 `planner/backend` 是默认执行后端；
每架飞机可用成员配置中的 `execution/backend` 覆盖，因此固定翼侦察机和旋翼工作机可以
连接不同适配器。同一架飞机仍然只启用一个后端，不会自动回退或重复下发。

正式配置为：

```yaml
planner:
  backend: planning
```

此时旋翼点目标和固定翼完整几何路径分别发布到：

```text
/<uav>/planning/goal       geometry_msgs/PoseStamped
/<uav>/planning/task_path  nav_msgs/Path
```

两类飞机统一从 planning 接收状态，并通过取消服务完成 pause/stop、到达以及视觉跟踪前的
控制权交接：

```text
/<uav>/planning/status  xd_uav_task_allocate/PlannerStatus
/<uav>/planning/cancel  xd_uav_task_allocate/CancelPlanning
```

任务层在该后端不创建 controller publisher，也不消费 controller 的 `PathStatus`。旋翼由
EGO-Swarm 生成避障轨迹；固定翼 planning 后端校验 Path 并转交固定翼 controller，当前仍不提供
固定翼点云避障或动态禁飞区重规划。

旋翼侦察机执行搜索航点 XYZ；工作机默认停在目标前 5 m，并把分配时的 world 高度写入 PZ，
保持明确的
垂直位置闭环，既不采用地面目标 Z，也不通过忽略 PZ 来关闭高度控制。默认根据飞机当前
位置到当前航点的 world ENU 水平方向设置 yaw，
使机头沿当前航段方向转动；水平距离过小时保持已有航向。旋翼根据新鲜有效的 world
Odometry，在目标容差内持续指定时间后判定 `REACHED`。固定翼整段搜索只有收到与当前
`goal_id`一致的控制器 `PathStatus.COMPLETED` 才推进到完成；不再用名义飞行时长判定。

旋翼的 EGO 地图是局部地图。若搜索区域距离起飞点较远，可在对应飞机配置中设置
`multirotor/waypoint_gap`，任务层会把长航段拆成不超过该距离的局部航点，再逐点发送给规划器：

```yaml
scouts:
  uav2:
    mobility_profile: hover
    multirotor:
      waypoint_gap: 10.0
      # 到此三维半径即推进下一个分段点；不要求停稳。
      waypoint_acceptance_radius_m: 1.0
```

这两个值只影响旋翼航点分段，不改变固定翼的完整 `task_path`。前者为最大分段长度，后者为
每个中间航点的三维到达半径；可在 `mission.yaml/planner/multirotor` 统一设置，也可按飞机覆盖。
工作机使用独立的到达条件：在 0.5 m 的停距目标容差内持续 1 s，且三维速度不超过
0.35 m/s，随后立即锁存当前位置保持，避免高速掠过停距点后继续撞向目标。

旋翼工作机前往远距离救援目标时也使用同一 `multirotor/waypoint_gap` 分段参数；中间航点
只保持任务执行状态，只有最后一个目标航点到达后才释放规划控制权并进入视觉交接。

`ego_swarm` 仅保留为旧配置的 planning 别名；新配置统一写 `planning`。路线可视化单独发布到
`/<uav>/planning/route_preview`，绝不能把预览 Path 当作固定翼执行输入。
`direct_controller_test` 仍保留作显式的无规划器测试后端；它会直发 controller，正式任务不要使用。

## 配置分层

- `config/mission.yaml`：共享坐标、目标去重、确认和分配参数；
- `config/scouts.yaml`：侦察机成员、world Odometry、健康状态、检测和规划话题；
- `config/workers.yaml`：作业机成员、world Odometry、健康状态和规划话题；
- `config/task_execution.yaml`：工作机到达后的完成方式和 TaskExecute 参数。

本包不配置或判断飞机类型，只用 `mobility_profile` 描述生成搜索路径和判断到达所需的运动语义：

```yaml
mobility_profile: hover     # hover 或 fixedwing
```

缺省值为 `hover`。VTOL 在当前任务中采用悬停导航时配置 `hover`，采用持续前飞、Dubins
航线和飞越到达语义时配置 `fixedwing`。协调器不读取 `ControlState` 的机型字段。

## 到达后的任务执行

`config/task_execution.yaml` 独立控制工作机到达后的完成语义。默认保持原行为，便于只验证
目标确认和任务分配：

```yaml
post_arrival:
  mode: arrive
```

此时工作机收到 `REACHED` 后立即完成任务，不要求启动 `xd_uav_task_execute`。需要验证
“到达后跟踪”时切换为：

```yaml
post_arrival:
  mode: task_execute
  default_task:
    task_type: track
    local_track_id: -1
    required_execution_sec: 10.0
```

协调器随后连接 `/<worker>/task_execute/execute`。工作机到达只表示进入
`EXECUTING`，只有 Action 成功才把任务和目标标为 `COMPLETED` 并释放工作机。Action 失败
默认把任务标为 `FAILED`，等待操作员调用 `retry_task`；也可将 `failure_policy` 设为
`retry` 自动重新排队。

任务参数按 `default_task → mobility_profile_overrides → class_overrides → worker_overrides`
依次覆盖。例如四旋翼可使用 `gm_velocity_chase`，固定翼可使用
`fw_velocity_vector`。`follower_profile: ""` 表示沿用 Track 自身配置，避免分配层强制切换。
暂停、停止、禁用飞机、取消任务、拒绝目标或工作机掉线时，协调器会取消对应 Action；暂停
会把未完成任务退回队列，恢复后重新分配。

也可以在启动时替换整份独立配置，而不修改 `mission.yaml`：

```bash
roslaunch xd_uav_task_allocate task_allocate.launch \
  post_arrival_config:=/absolute/path/to/task_execution.yaml
```

## 混合固定翼/旋翼机队

可从 [scouts_mixed_example.yaml](config/scouts_mixed_example.yaml) 复制混合侦察机配置。
默认的 `hierarchical_search/mode: auto` 根据侦察机类型选择任务链：纯四旋翼使用单级区域
分配；同时配置 fixedwing 和 multirotor 时自动执行下面的两阶段搜索：

```text
原始大区域 SearchArea
        ↓ 只分给 fixedwing
固定翼下视 detect 粗定位 + position_covariance
        ↓ 达到粗证据阈值，立即发布局部区域
GlobalTarget.VERIFYING → 空闲 multirotor 出发
        ↓ 按 XY 协方差生成局部方形割草机路线
四旋翼完整执行当前区域的低空精搜路线
        ↓ 发现目标可立即确认并派工作机，但不提前结束本区域
GlobalTarget.CONFIRMED → RescueTask → multirotor worker
        ↓ 当前路线结束后
下一个待搜索局部区域
```

启用示例：

```bash
roslaunch xd_uav_task_allocate task_allocate.launch \
  scout_config:=$(rospack find xd_uav_task_allocate)/config/scouts_mixed_example.yaml
```

两级模式下，原始输入的一个或多个大区域都只在固定翼之间分配，四旋翼侦察机等待粗定位。
默认 `verification/dispatch_policy: immediate`：固定翼候选达到粗证据阈值后立即发布局部区域，
并派空闲四旋翼出发，不等待固定翼搜完整个大区域。若确实需要严格分阶段，才改成
`after_coarse_complete`。

局部区域默认 `maximum_concurrent_regions: 1`，因此无论有多少空闲四旋翼，都必须完整搜索完
当前区域才取下一个队列区域。四旋翼在区域中确认一个目标不会提前结束路线；工作机可以立即
执行已确认目标，同时侦察四旋翼继续把该区域扫完。新粗目标中心若落在一个已发布且尚未结束的
局部区域内，会加入该区域的目标集合，不再生成重复区域和重复路线。

局部精搜范围支持两种方式：

- `radius_mode: covariance`：取固定翼 `position_covariance` 的最大 XY 主轴标准差乘
  `covariance_sigma`，再限制到 `minimum_radius_m` 与 `maximum_radius_m`；
- `radius_mode: fixed`：直接使用 `fixed_radius_m` 作为局部方形搜索区的半宽。

局部精搜航线的 Z 由 `verification/altitude_m` 设置，它是 `world` 中的绝对高度，不是相对
地面高度。当前 SITL 四旋翼统一在 4 m 起飞，因此默认也设为 `4.0`，避免接到区域后无意义地
爬升到 20 m。

若整个局部路线完成仍没有四旋翼确认，默认把该粗候选标记为 `STALE`，不会创建救援任务。

为了让粗定位有意义，固定翼的 `xd_uav_detect` 必须使用 `ground_plane` 模式并输出
`range_valid=true`、`has_relative_position_body=true` 和可信的 `position_covariance`；任务包
通过统一的 `DetectionArray` 接口接收观测；粗搜或精搜语义由侦察机的 `mobility_profile` 决定。

固定翼参数可以在 `mission.yaml/planner/fixedwing` 中统一设置，也可以在单机
`fixedwing` 段覆盖：

```yaml
scouts:
  fw1:
    mobility_profile: fixedwing
    execution:
      backend: planning
    coverage:
      nominal_speed_mps: 15.0
    fixedwing:
      coverage_altitude_m: 40.0
      coverage_lane_spacing_m: 30.0
      minimum_turn_radius_m: 30.0
      turn_waypoint_spacing_m: 5.0
      straight_lead_distance_m: 30.0
      waypoint_acceptance_radius_m: 20.0
      waypoint_altitude_tolerance_m: 10.0
      pass_cross_track_limit_m: 40.0
```

固定翼覆盖规划保留多边形内的全部平行扫描弦，但用 `0,N/2,1,N/2+1,...` 的跳行顺序拉开
连续反向航段，避免“小行距、大转弯直径”在每个端点生成三圆弧。每条扫描弦前后再延长
`straight_lead_distance_m`，确保飞机在进入区域前完成转弯、在区域内保持直飞。
`coverage_lane_spacing_m` 是固定翼传感器在 `coverage_altitude_m` 高度的有效地面覆盖宽度，
必须根据相机视场和所需重叠率设置；值为 0 时才沿用 `SearchArea.lane_spacing`。它与四旋翼
局部精搜的 `hierarchical_search/verification/lane_spacing_m` 相互独立。

转弯通常会离开搜索多边形，任务区域外必须预留至少与 `minimum_turn_radius_m` 和直线引导段
同量级的已确认净空；该几何约束不等于障碍物规划。`turn_waypoint_spacing_m` 只改变同一条
Dubins 曲线的离散密度，不会显著改变曲线长度或执行时间。
混合机队的区域负载按“运动语义对应路线长度 ÷ `coverage/nominal_speed_mps`”估算，因而不会
把同样长度机械地视为固定翼和旋翼具有相同完成时间。

救援任务分配与飞机类型及 `mobility_profile` 均无关。所有健康、在线且空闲的工作机都参与
距离分配。若以后确实需要表达载荷或任务能力，应增加独立的 capability 标签。

```yaml
workers:
  vtol1:
    mobility_profile: hover
```

每架飞机在 `localization/world_odometry_topic` 中指定估计器主状态重发布话题，例如
`/uav1/state_estimator/main/frames/world/odom`。该接口固定为 `nav_msgs/Odometry`，节点直接读取
`pose.pose.position`、`pose.pose.orientation` 和原始估计时间戳，并严格要求
`header.frame_id == world`。

`health/control_state_topic` 仍订阅 `xd_uav_controller/ControlState`，使用
`state_valid/localization_valid/odometry_fresh` 判断飞机是否允许参与分配，不读取其中的
位置与姿态，也不使用机型作为准入条件。只有 world Odometry 和健康状态都新鲜有效，飞机才可参与新任务。单次状态
抖动只暂时禁止新分配；连续超过 `allocation/worker_state_timeout_sec` 才释放正在执行的任务。

飞机位置链为：

```text
/uavX/state_estimator/main/frames/world/odom -> xd_uav_task_allocate
```

任务包不保存每架飞机的位置/航向偏移，也不再为飞机位置查询 `world <- odom`。TF2仅用于
把非 `world` 的搜索区域转换到 `world`。

`SearchAreaArray.header.frame_id` 必须填写。非 `world` 区域会先在消息时间戳转换到
`world`；如果该坐标系相对 `world` 明显倾斜，无法用单一搜索高度表示时会拒绝任务。

## 启动

```bash
source devel/setup.bash
roslaunch xd_uav_task_allocate task_allocate.launch
```

搜索区域由 `/task_allocate/search_areas` 下发，消息类型为
`xd_uav_task_allocate/SearchAreaArray`。输出包括：

```text
/task_allocate/global_targets  xd_uav_task_allocate/GlobalTargetArray
/task_allocate/rescue_tasks    xd_uav_task_allocate/RescueTaskArray
/task_allocate/mission_state   std_msgs/String（IDLE/LOADED/ACTIVE/PAUSED/ABORTED/COMPLETED）
/task_allocate/verification_areas    xd_uav_task_allocate/SearchAreaArray
/task_allocate/verification_markers  visualization_msgs/MarkerArray
```

RViz 中局部验证框按状态着色：黄色为等待、蓝色为正在搜索、绿色为搜完且确认过目标、红色为
完整搜完但未确认目标。默认 RViz 配置已订阅 `/task_allocate/verification_markers`。

下发搜索区域只加载任务并进入 `LOADED`，不会发布 `planning/goal`。现有控制系统完成解锁、
起飞并稳定后，由操作员显式开始：

```bash
rosservice call /task_allocate/start
```

默认要求 `scouts.yaml` 中全部侦察机的 world Odometry 与 ControlState 健康状态都有效；否则
服务返回失败，已加载区域不会丢失，飞机准备好后可以再次调用。成功后才生成/分配覆盖路线，
进入 `ACTIVE`；旋翼向当前执行后端发布第一个航点，固定翼测试后端一次发布整段几何路径。
由于现有状态消息没有可靠的“已经离地”字段，
`start` 必须由地面站在确认起飞后调用，任务包不会仅凭定位正常自动判定已经起飞。

## 任务生命周期与操作员服务

任务节点不再只有 `start`。完整状态转换为：

```text
IDLE --加载区域--> LOADED --start--> ACTIVE --完成--> COMPLETED
                              ACTIVE <--> PAUSED
                         ACTIVE/PAUSED --stop--> ABORTED
        reset（保留区域）-----------------------> LOADED
        clear_all（清除区域）-------------------> IDLE
```

生命周期服务均使用 `std_srvs/Trigger`：

| 服务 | 允许状态 | 行为与数据保留 |
|---|---|---|
| `/task_allocate/start` | `LOADED` | 根据飞机当前位置生成覆盖路线并开始执行 |
| `/task_allocate/pause` | `ACTIVE` | 进入 `PAUSED`；保留区域、航点索引、目标、验证和救援任务 |
| `/task_allocate/resume` | `PAUSED` | 重新发布原活动目标，从保存的航点和任务继续 |
| `/task_allocate/stop` | `ACTIVE/PAUSED` | 进入 `ABORTED`；保留区域和结果，终止活动目标，正在执行的救援任务标为失败 |
| `/task_allocate/reset` | 任意 | 必要时先安全停止；保留搜索区域，清空路线进度、目标、验证与救援任务，回到 `LOADED` |
| `/task_allocate/clear_all` | 任意 | 必要时先安全停止；清空区域和所有任务数据，回到 `IDLE` |
| `/task_allocate/restart` | 已加载区域 | 停止并清空本轮结果，使用原区域重新规划并立即尝试开始 |
| `/task_allocate/replan` | `ACTIVE/PAUSED` | 保留已发现目标和救援任务，从当前侦察机位置重新生成整个已加载区域的覆盖路线 |
| `/task_allocate/clear_results` | 非 `ACTIVE/PAUSED` | 仅清除融合目标、验证队列和救援任务，保留区域与已有路线快照 |
| `/task_allocate/cancel_all_tasks` | 任意 | 取消所有未完成救援任务，不停止仍在执行的搜索路线 |

常用调用：

```bash
rosservice call /task_allocate/pause "{}"
rosservice call /task_allocate/resume "{}"
rosservice call /task_allocate/stop "{}"
rosservice call /task_allocate/reset "{}"
rosservice call /task_allocate/clear_all "{}"
rosservice call /task_allocate/restart "{}"
rosservice call /task_allocate/replan "{}"
```

`planning` 后端暂停、停止、跳点、禁用飞机或切入视觉任务时，任务层先调用该机
`planning/cancel`。旋翼关闭 reference mux 并停发 EGO 参考，控制器进入自身超时保持；固定翼用
一次保持当前高度和飞行方向的参考立即替换活动 Path，随后进入控制器的 Loiter 保护。只有取消
成功，任务层才清理活动 goal 或把控制权交给 `xd_uav_task_execute`；恢复时重新发布保存目标/路径。

### 带参数的任务恢复服务

| 服务与类型 | 请求字段 | 行为 |
|---|---|---|
| `/task_allocate/get_state` (`GetMissionState`) | 无 | 返回状态、说明、区域/目标/验证/任务数量、各机路线进度和活动目标 |
| `/task_allocate/load_search_areas` (`LoadSearchAreas`) | `search_areas` | 加载 `SearchAreaArray` 并返回成功、失败原因和有效区域数量；原 topic 接口继续保留 |
| `/task_allocate/skip_waypoint` (`VehicleCommand`) | `vehicle_name` | 跳过指定侦察机当前航点；在 `PAUSED` 中跳过后等 `resume` 再继续 |
| `/task_allocate/set_vehicle_enabled` (`SetVehicleEnabled`) | `vehicle_name, enabled` | 操作员禁用/恢复飞机；禁用作业机会释放任务，禁用侦察机后通常应调用 `replan` |
| `/task_allocate/cancel_task` (`TaskCommand`) | `task_id` | 取消一个未完成救援任务并释放作业机 |
| `/task_allocate/retry_task` (`TaskCommand`) | `task_id` | 将未完成任务重新放回待分配队列 |
| `/task_allocate/reject_target` (`TargetCommand`) | `target_id` | 删除误检目标，并清理其验证路线、救援任务和作业机占用 |
| `/task_allocate/retry_verification` (`TargetCommand`) | `target_id` | 两级搜索中把目标重新放入四旋翼精搜队列 |

调用示例：

```bash
rosservice call /task_allocate/get_state "{}"
rosservice call /task_allocate/skip_waypoint "{vehicle_name: 'uav1'}"
rosservice call /task_allocate/set_vehicle_enabled "{vehicle_name: 'uav2', enabled: false}"
rosservice call /task_allocate/cancel_task "{task_id: 1}"
rosservice call /task_allocate/retry_task "{task_id: 1}"
rosservice call /task_allocate/reject_target "{target_id: 3}"
rosservice call /task_allocate/retry_verification "{target_id: 3}"
```

推荐优先使用 `/task_allocate/load_search_areas`，因为 service 会返回校验结果；继续使用
`/task_allocate/search_areas` topic 也兼容，但 topic 发布方无法直接得到拒绝原因。加载新区域会
清除上一轮目标和救援任务；`ACTIVE/PAUSED` 状态下拒绝替换区域，需先 `stop` 或 `reset`。

每个检测器 `track_id` 都按 `(source_uav, sensor_id, track_id)` 作为局部轨迹使用；相同稳定
轨迹优先关联原世界目标，不同轨迹仍可通过世界位置和协方差重新关联，因此跟踪中断换号或
多机分别编号不会直接变成新任务。具体 `class_id` 全部保留，不做类别合并；物理身份关联与
类别判断分开，同一稳定轨迹的偶发类别跳变只累计为类别证据，必须达到多帧类别占比和位置
散布门限后才能 confirmed。不同稳定轨迹在同一来源帧中同时出现时会记录为两个明确目标，
避免把靠近的同类物体误合并。

普通模式下，满足观测次数、持续时间、类别稳定度和世界位置稳定度后才 confirmed；两级搜索
模式下，固定翼证据只进入 VERIFYING，必须由四旋翼的稳定精定位证据确认。每个全局目标只
创建一个作业任务，任务创建前另有一个可配置的同类别近邻安全门，阻止残余轨迹分裂派出第二
个工作机；同帧已证明为不同轨迹的目标不受该安全门影响。候选目标超过
`stale_timeout_sec` 没有继续观测才会过期；已经 confirmed 并进入任务队列的目标不会在
等待作业机期间过期，仍持续参与世界坐标空间去重。

## 不驱动飞机的任务层测试

如需只验证ROS任务消息而完全不驱动飞机，应保持 backend 为 `planning`，但不要启动真实
planning，而是启动
`mock_ego_swarm.py`。它只模拟 `PLANNING/ACTIVE/REACHED` 状态，不读取点云、不生成轨迹、
不向 controller 发布任何内容，禁止把它用于真实飞行。可以为每架飞机启动一份：

```bash
rosrun xd_uav_task_allocate mock_ego_swarm.py _uav_name:=uav1
rosrun xd_uav_task_allocate mock_ego_swarm.py _uav_name:=uav2
rosrun xd_uav_task_allocate mock_ego_swarm.py _uav_name:=uav3
rosrun xd_uav_task_allocate mock_ego_swarm.py _uav_name:=uav4
```

在仿真已经发布各机 world Odometry 和 `ControlState` 后，下发并启动测试任务：

```bash
rosrun xd_uav_task_allocate mock_search_mission.py
rosservice call /task_allocate/start
```

也可以覆盖 `_areas_json` 同时下发多个区域。收到红色检测后，观察
`/task_allocate/global_targets` 和 `/task_allocate/rescue_tasks`，确认重复帧只形成一个
全局目标和一个作业任务。

## EGO-Swarm 接入约定

当前工作空间已由 `xd_uav_planning` 完成 EGO-Swarm 适配：

1. 订阅 `planning/goal`，把共享 frame 的三维目标交给 EGO-Swarm；
2. 将规划/执行状态映射成 `PlannerStatus`；
3. 将安全轨迹通过 reference mux 转成当前 controller 的流式 `PositionTarget`；
4. `planning/cancel` 关闭输出门并清空活动目标，交接成功后任务层才启动跟踪控制。

复制来的旧 SEAD 实现保存在 `legacy/`，不参与安装、启动和测试，也不会与原
`xd_uav_sead` Python 模块发生冲突。
