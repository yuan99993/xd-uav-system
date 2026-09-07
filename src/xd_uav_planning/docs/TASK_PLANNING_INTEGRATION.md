# 任务层接入 `xd_uav_planning` 手册

更新：2026-09-07
适用基准：`xd_uav_planning@feature/LTJ`、`xd_uav_task_allocate origin/dev@e763836`

## 1. 当前结论

正式数据流必须是：

```text
xd_uav_task_allocate -> xd_uav_planning -> xd_uav_controller
```

任务层只决定任务、载机和目标/路线；规划层是唯一允许向 controller 发布执行参考的上游。
当前规划包两种后端均已具备，固定翼 `task_path -> controller -> PlannerStatus` 已完成真实 SITL
验收。但远端任务包的固定翼 `direct_controller_test` 仍绕过规划层，因此两个包原样共启还不算
正式打通。本文第 7 节给出任务层维护者需要完成的最小改造。

当前能力边界：

- 多旋翼：实时点目标、EGO 轨迹、健康门、控制权仲裁和状态回报；受当前 EGO manual-target
  行为限制，只接受 `world z=1.0 m`。
- 固定翼：接收完整几何 Path、严格校验、转发 controller，并把 controller 私有 path ID
  映射回任务 goal ID；不提供 EGO 点云避障或 SEAD 动态禁飞区重规划。
- pause/cancel/replan 生命周期接口尚未实现，不能对外宣称完整任务控制已经打通。

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
| task -> planning | `/<uav>/planning/goal` | `geometry_msgs/PoseStamped` | 是 | 否 |
| task -> planning | `/<uav>/planning/task_path` | `nav_msgs/Path` | 否 | 是 |
| planning -> task | `/<uav>/planning/status` | `xd_uav_task_allocate/PlannerStatus` | 是 | 是 |
| planning -> system | `/<uav>/planning/healthy` | `std_msgs/Bool` | 是 | 是 |
| planning -> operator | `/<uav>/planning/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 是 | 是 |

`/<uav>/planning/mission_path` 是现有任务包的路线预览，不是执行输入。正式系统中建议最终更名
为 `/<uav>/planning/route_preview`；无论是否更名，都不得重映射到 `task_path`。

controller 侧接口是规划层私有下游：

```text
/<uav>/control/reference/setpoint
/<uav>/control/reference/path
/<uav>/controller/path_status
```

任务层正式 backend 不得发布或订阅这些话题。

## 4. 输入消息约束

### 4.1 多旋翼 `PoseStamped`

- `header.frame_id` 必须等于该实例的 `common_frame`，默认 `world`。
- `position.x/y/z` 必须是有限数值。
- 当前 `flight_type` 必须为 `1`，即 EGO manual-target 模式。
- 当前 `z` 必须为 `1.0 ± 0.05 m`；其他高度会立即返回 `FAILED`，不会静默改高度。
- `orientation` 当前不参与 EGO 目标规划，不应依赖它表达任务语义。

### 4.2 固定翼 `Path`

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
| `goal_topic` | `/<uav>/planning/goal` | 多旋翼点目标输入 |
| `task_path_topic` | `/<uav>/planning/task_path` | 固定翼执行路径输入 |
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
   - 多旋翼创建 `planner_goal` publisher；
   - 固定翼创建 `planner_task_path` publisher；
   - 两种机型都订阅 `planner_status`；
   - 订阅 `planner_healthy`：固定翼可作为 Path 派发前条件，多旋翼只用于目标下发后的执行链
     监控，不能阻止首目标发布；
   - planning backend 不创建 controller setpoint/path publisher，也不订阅 `PathStatus`。
3. 保留 `_publish_direct_fixedwing_path()` 中现有路线生成和嵌套 goal-ID 编码逻辑，将最终发布者
   改成 `planner_task_path`。建议同时把函数重命名为 `_publish_fixedwing_task_path()`。
4. 搜索航线和固定翼 worker 的短路径都必须走同一个 `planner_task_path` publisher。
5. `_planner_status_callback()` 继续作为唯一状态入口；删除 planning backend 内部的
   `_publish_direct_status()` 和 `_path_status_callback()` 路径。
6. 预览 Path 单独发布到 `route_preview`，不得把无活动 goal ID 的预览发送给 `task_path`。
7. 在 cancel 服务正式实现前，任务层对活动 planning backend 的 pause/stop/replan 必须明确拒绝，
   不能通过直接 controller hold 绕过规划层。

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
      route_preview: /uav1/planning/route_preview

  uav2:
    vehicle_type: multirotor
    execution:
      backend: planning
    topics:
      planner_goal: /uav2/planning/goal
      planner_status: /uav2/planning/status
      planner_healthy: /uav2/planning/healthy
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

## 10. 尚未完成

- 任务层上述代码交接及真实 `task_allocate -> planning -> controller` 双机型复验；
- 带稳定显式 goal ID 的请求消息；
- cancel/pause/resume/replan 服务和安全状态机；
- 多旋翼任意三维实时目标；
- 多机产品级组合入口；
- EGO 第三方源码恢复官方原样及外部 swarm 适配。

这些缺口没有被演示脚本掩盖；完成前不得宣称任务层和规划层已经完整即插即用。
