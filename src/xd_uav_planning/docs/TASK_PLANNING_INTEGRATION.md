# 任务层接入 `xd_uav_planning` 手册

更新：2026-09-10
适用基准：当前工作区 `xd_uav_planning` 与 `xd_uav_task_allocate`

## 1. 当前结论

正式数据流必须是：

```text
xd_uav_task_allocate -> xd_uav_planning -> xd_uav_controller
```

任务层只决定任务、载机和目标/路线；规划层是唯一允许向 controller 发布执行参考的上游。
当前规划包和任务包已经正式打通：固定翼和多旋翼都走完整 Path，多旋翼由 EGO 负责局部避障；两者在暂停、停止、
到达后执行和视觉跟踪交接前都先调用统一 cancel 服务释放 planning 控制权。

当前能力边界：

- 多旋翼：完整三维参考 Path、EGO 连续轨迹、健康门、控制权仲裁、取消和状态回报；高度不再
  固定为 `1.0 m`，但仍必须位于 EGO 地图和安全边界内。旧三维点目标接口继续兼容。
- 固定翼：接收完整几何 Path、严格校验、转发 controller，并把 controller 私有 path ID
  映射回任务 goal ID，并支持任务前/飞行中的动态禁飞区绕飞和剩余路径重规划；无安全解时
  返回 `BLOCKED` 并触发 `cancel_offboard` 失效保护。
- pause/stop/skip/disable/replan 与视觉任务交接均经过 `planning/cancel`，取消失败时任务层保持
  原状态并拒绝交接。

## 2. 启动责任

规划层正式入口：

```bash
roslaunch xd_uav_planning planning.launch \
  UAV_NAME:=uav1 \
  vehicle_type:=multirotor \
  common_frame:=world \
  output_frame:=uav1/odom
```

固定翼：

```bash
roslaunch xd_uav_planning planning.launch \
  UAV_NAME:=uav1 \
  vehicle_type:=fixedwing \
  common_frame:=world
```

`planning.launch` 只启动规划后端，不启动以下内容：

- Gazebo/PX4/MAVROS；
- state estimator、control manager 或 controller；
- 自动起飞；
- task allocator；
- 演示目标或验收发布器。

系统级 launch 应分别先组合仿真/真机、状态估计与控制栈，再为每架飞机包含一次
`planning.launch`，最后启动任务层。不得同时为同一架飞机启动两个 planning backend。

## 3. 正式公共接口

以下默认名称都可通过 `planning.launch` 参数覆盖。

| 方向 | 话题 | 类型 | 多旋翼 | 固定翼 |
|---|---|---|---:|---:|
| task -> planning | `/<uav>/planning/goal` | `geometry_msgs/PoseStamped` | 兼容 | 否 |
| task -> planning | `/<uav>/planning/task_path` | `nav_msgs/Path` | 默认 | 是 |
| planning -> task | `/<uav>/planning/status` | `xd_uav_task_allocate/PlannerStatus` | 是 | 是 |
| planning -> system | `/<uav>/planning/healthy` | `std_msgs/Bool` | 是 | 是 |
| planning -> operator | `/<uav>/planning/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 是 | 是 |
| task -> planning | `/<uav>/planning/cancel` | `xd_uav_task_allocate/CancelPlanning` | 是 | 是 |

`/<uav>/planning/route_preview` 是路线预览，不是执行输入，禁止重映射到 `task_path`。

controller 侧接口是规划层私有下游：

```text
/<uav>/control/reference/setpoint
/<uav>/control/reference/path
/<uav>/controller/path_status
```

任务层正式 backend 不得发布或订阅这些话题。

## 4. 输入消息约束

### 4.1 多旋翼 `nav_msgs/Path`

- `Path.header.frame_id` 必须等于该实例的 `common_frame`，默认 `world`。
- `Path.header.stamp` 必须为正且新鲜；默认超时 2.0 s，最多允许未来 0.25 s。
- 至少 2 个、最多 10000 个点；所有坐标必须有限；相邻点距离不得小于 0.05 m。
- 每个嵌套 `PoseStamped.header.frame_id` 可为空；非空时必须等于 `common_frame`。
- EGO 将整条剩余路径生成一个连续 global trajectory。路径中间点是几何约束，不是独立
  `REACHED` 事件；只有路径末端在位置和速度容差内稳定指定时间后才报告完成。
- 路径可包含回头、弯道和自交段；状态层使用带前向窗口的单调弧长投影，避免进度回跳。

### 4.2 多旋翼兼容 `PoseStamped`

- `header.frame_id` 必须等于该实例的 `common_frame`，默认 `world`。
- `position.x/y/z` 必须是有限数值。
- 仅使用兼容单点接口时，`flight_type` 必须为 `1`，即 EGO manual-target 模式；完整
  `nav_msgs/Path` 应使用 `flight_type=3`。
- `z` 会原样传给 EGO，不再被改写为 `1.0 m`；目标仍须处于配置的三维地图范围和安全空间内。
- `orientation` 当前不参与 EGO 目标规划，不应依赖它表达任务语义。

### 4.3 固定翼 `Path`

- `Path.header.frame_id` 必须等于该实例的 `common_frame`。
- `Path.header.stamp` 必须为正，默认必须在接收前 1.0 s 内，最多允许未来 0.02 s。
- 至少 2 个、最多 10000 个点。
- 每个 `PoseStamped.header.frame_id` 可为空；非空时必须等于 `common_frame`。
- 所有坐标必须有限；任意相邻点距离不得小于 0.05 m。
- 发路径时 `ControlState.vehicle_type` 必须是 `VEHICLE_FIXEDWING`，且 state、localization、
  odometry fresh 均有效；状态默认不得旧于 0.30 s。
- 同一个尚未终止的 `goal_id` 重复发布会被忽略，不会重复替换 controller 路径。

固定翼当前的过渡 goal ID 编码：

```python
path.header.seq = goal_id
for pose in path.poses:
    pose.header.seq = goal_id
```

必须在每个嵌套 `PoseStamped.header.seq` 中重复写入 ID，因为 rospy 发送时会重写顶层
`Header.seq`。这是兼容现有消息的过渡协议；后续应把 ID 提取到自有任务请求消息的显式字段。

## 5. 状态和健康语义

`PlannerStatus` 字段：

```text
std_msgs/Header header
uint32 goal_id
uint8 state
string detail
```

状态值：

| 值 | 名称 | 任务层动作 |
|---:|---|---|
| 0 | `IDLE` | 无活动目标 |
| 1 | `PLANNING` | 保持任务活动，等待执行 |
| 2 | `ACTIVE` | 标记任务执行中 |
| 3 | `REACHED` | 完成当前航点/任务并推进 |
| 4 | `BLOCKED` | 释放或重规划，不得当成到达 |
| 5 | `FAILED` | 释放载机并记录失败原因 |

任务层只处理与当前活动任务完全相同的 `goal_id`。过期或其他任务的状态必须忽略。
`planning/status` 是正式 backend 的唯一任务状态源，不能再同时消费 controller `PathStatus`。

`planning/healthy=True` 只表示当前后端的执行链正在产生可转发的有效输出，不等价于目标到达。
两种后端的首条命令时序不同：

- 固定翼可以在发 Path 前等待健康；它只依赖有效的固定翼 `ControlState`。
- 多旋翼首个 EGO command 要在目标下发后才产生，因此首目标前健康为 false 是正常状态。
  任务层必须先依据 control manager/定位状态发布目标，在收到 `PLANNING` 后观察
  `planning/healthy` 和 `ACTIVE`；禁止用该健康话题作为首目标的前置门，否则会循环等待。

两种机型都必须以 `PlannerStatus` 判断任务生命周期。

## 6. launch 参数

常用参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `UAV_NAME` | `uav1` | ROS 命名空间中的载机名 |
| `vehicle_type` | `multirotor` | `multirotor` 或 `fixedwing` |
| `common_frame` | `world` | 输入目标/路径的公共坐标系 |
| `output_frame` | `common_frame` | 多旋翼控制参考输出坐标系 |
| `ego_id` | `0` | EGO 实例编号，多机必须唯一 |
| `map_resolution` | `0.20 m` | EGO 占据栅格分辨率 |
| `map_size_x/y/z` | `36/36/9 m` | EGO 滚动栅格地图尺寸 |
| `local_update_range_x/y/z` | `14/14/7.5 m` | 每帧原生点云进入局部地图的范围 |
| `ground_height` | `-0.01` | EGO 地图下边界 |
| `virtual_ceil_height` | `8.0` | EGO 可飞上边界；高空任务需连同 map size 调大 |
| `flight_type` | `3` | EGO 路径模式；`1` 为兼容单点模式 |
| `goal_topic` | `/<uav>/planning/goal` | 多旋翼兼容单点输入 |
| `task_path_topic` | `/<uav>/planning/task_path` | 多旋翼/固定翼完整路径输入 |
| `path_timeout` | `2.0 s` | 多旋翼 Path 输入新鲜度 |
| `route_stall_timeout` | `12.0 s` | 完整 Path 沿程进度无变化后，从实测位置重新接入剩余路线的阈值；不限制整条路线总时长 |
| `route_stall_recovery_enabled` | `true` | 停滞时暂停旧轨迹输出，并重新发布裁掉已完成部分的剩余 Path |
| `route_stall_max_recoveries` | `0` | 连续恢复次数上限；`0` 表示由任务取消或到达状态结束，不因一次规划停滞永久判失败 |
| `route_tracking_weight` | `8.0` | EGO 局部重规划贴合完整 Path 的权重；真实障碍仍可触发局部绕行 |
| `replan_interval` | `0.25 s` | 正常执行时的局部轨迹更新周期 |
| `replan_lookahead_time` | `0.12 s` | 从旧轨迹未来状态接续，用于补偿规划/传输耗时 |
| `astar_resolution` | `0.20 m` | 障碍绕行前端搜索分辨率 |
| `astar_search_time` | `0.15 s` | 单次局部 A* 的硬超时 |
| `route_progress_epsilon_m` | `0.25 m` | 认为完整 Path 取得有效进度的最小弧长增量 |
| `path_progress_backtrack_m` | `1.0 m` | 路径进度允许的有限回退窗口 |
| `path_progress_search_window_m` | `25.0 m` | 路径进度前向搜索窗口 |
| `cloud_input_topic` | `/<uav>/fastlio/points` | EGO 使用的原生雷达点云；全局 `/fastlio/points` 需显式覆盖 |
| `cloud_input_frame` | `/<uav>/lidar_link` | 原生点云真实 `header.frame_id`，用于 TF 转换到 `common_frame` |
| `minimum_sensor_range` | `0.35 m` | 过滤雷达近距离盲区和机体附近自回波 |
| `cloud_obstacle_persistence` | `0.5 s` | 直接点云占据的短时保持，抑制遮挡/稀疏帧闪烁 |
| `status_topic` | `/<uav>/planning/status` | 统一任务状态输出 |
| `healthy_topic` | `/<uav>/planning/healthy` | 统一健康输出 |
| `diagnostics_topic` | `/<uav>/planning/diagnostics` | 统一诊断输出 |

点云、body frame 和 EGO 参数也由 `planning.launch` 显式暴露。真机接入时必须使用经过标定的
body-to-sensor 变换，不能把 frame 名称直接改写成另一个坐标系。

## 7. `xd_uav_task_allocate` 最小改造清单

以下工作应由任务层维护者在自己的包内完成；规划包不代改上游源码。

1. 在 coordinator 接受的 backend 枚举中新增正式 `planning`（也可以将 `ego_swarm` 重新定义为
   统一 planning backend，但固定翼和多旋翼必须采用相同的“不直连 controller”边界）。
2. `_configure_vehicle()`：
   - 多旋翼和固定翼都创建 `planner_task_path` publisher；多旋翼可保留 `planner_goal` 兼容接口；
   - 两种机型都订阅 `planner_status`；
   - 订阅 `planner_healthy`：固定翼可作为 Path 派发前条件，多旋翼只用于目标下发后的执行链
     监控，不能阻止首目标发布；
   - planning backend 不创建 controller setpoint/path publisher，也不订阅 `PathStatus`。
3. 保留 `_publish_direct_fixedwing_path()` 中现有路线生成和嵌套 goal-ID 编码逻辑，将最终发布者
   改成 `planner_task_path`。建议同时把函数重命名为 `_publish_fixedwing_task_path()`。
4. 搜索航线、验证路线和 planning worker 的路径都必须走同一个 `planner_task_path` publisher；
   多旋翼不再把路径采样点拆成独立 PoseStamped 目标。
5. `_planner_status_callback()` 继续作为唯一状态入口；删除 planning backend 内部的
   `_publish_direct_status()` 和 `_path_status_callback()` 路径。
6. 预览 Path 单独发布到 `route_preview`，不得把无活动 goal ID 的预览发送给 `task_path`。
7. pause、stop、skip waypoint、禁用飞机、取消任务及视觉任务交接，均先请求
   `planning/cancel`；服务失败时不清除 active goal，也不启动下一级控制器。

建议配置目标态：

```yaml
planner:
  backend: planning

scouts:
  uav1:
    vehicle_type: fixedwing
    execution:
      backend: planning
    topics:
      planner_task_path: /uav1/planning/task_path
      planner_status: /uav1/planning/status
      planner_healthy: /uav1/planning/healthy
      planner_cancel: /uav1/planning/cancel
      route_preview: /uav1/planning/route_preview

  uav2:
    vehicle_type: multirotor
    execution:
      backend: planning
    topics:
      planner_goal: /uav2/planning/goal
      planner_task_path: /uav2/planning/task_path
      planner_status: /uav2/planning/status
      planner_healthy: /uav2/planning/healthy
      planner_cancel: /uav2/planning/cancel
      route_preview: /uav2/planning/route_preview
```

这段 YAML 是任务层改造后的目标配置；当前 `origin/dev@e763836` 尚不识别 `planning`、
`planner_task_path`、`planner_healthy` 或 `route_preview`，不能直接复制到现有版本运行。

## 8. 过渡期 goal ID

现有 `PoseStamped` 没有显式 goal ID。多旋翼联机时，规划层通过只读服务
`/task_allocate/get_state` 的 `active_goals` 恢复当前载机的稳定 ID；单独演示时才回退使用
`PoseStamped.header.seq`。因此正式联机必须满足：

- task allocator 节点提供 `GetMissionState` 服务；
- `active_goals` 中存在 `<uav>:goal=<id>,...`；
- 目标发布与 active goal 状态更新顺序一致。

长期建议新增带显式 `uint32 goal_id` 的规划请求消息，从而删除服务查询和 Header.seq 过渡逻辑。
在新消息确定前，不应修改现有消息包所有权。

## 9. 联调验收

每架飞机至少检查：

```bash
rostopic type /uav1/planning/status
rostopic echo -n 1 /uav1/planning/healthy
rostopic echo /uav1/planning/status
```

多旋翼验收：

1. 未发目标时 controller 不接收伪造任务目标。
2. 发布合法目标后状态依次出现 `PLANNING -> ACTIVE -> REACHED`。
3. goal ID 始终等于任务层当前活动 ID。
4. EGO/点云/里程计失效时健康变 false，控制引用 fail-closed。

固定翼验收：

1. 任务层只在 `task_path` 出现一次完整路径，controller topic 的发布者只能是规划层。
2. 状态出现 `PLANNING -> ACTIVE -> REACHED`，controller 私有 path ID 不泄漏到任务层。
3. 错误 frame、过期时间戳、重复点、无效载机类型分别返回 `FAILED` 且不替换有效路径。
4. 任务真正产生位移、到达并进入约定的固定翼安全终态。

ROS graph 还必须确认不存在任务层到 controller 的直连：

```bash
rostopic info /uav1/control/reference/path
rostopic info /uav1/controller/path_status
```

## 10. 后续增强项

- 用专用请求消息替代 `PoseStamped.header.seq`/任务状态查询的过渡 goal ID；
- 完善 pause/resume/replan 服务和安全状态机；
- 多机产品级组合入口；
- EGO 第三方源码恢复官方原样及外部 swarm 适配。
