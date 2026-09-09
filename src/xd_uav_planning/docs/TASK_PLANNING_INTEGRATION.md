# 任务层接入 `xd_uav_planning` 手册

更新：2026-09-09
适用基准：当前工作区 `xd_uav_planning` 与 `xd_uav_task_allocate`

## 1. 当前结论

正式数据流必须是：

```text
xd_uav_task_allocate -> xd_uav_planning -> xd_uav_controller
```

任务层只决定任务、载机和目标/路线；规划层是唯一允许向 controller 发布执行参考的上游。
当前规划包和任务包已经正式打通：固定翼走完整 Path，多旋翼走 EGO 点目标；两者在暂停、停止、
到达后执行和视觉跟踪交接前都先调用统一 cancel 服务释放 planning 控制权。

当前能力边界：

- 多旋翼：实时三维点目标、EGO 轨迹、健康门、控制权仲裁、取消和状态回报；高度不再固定为
  `1.0 m`，但仍必须位于 EGO 地图和安全边界内。
- 固定翼：接收完整几何 Path、严格校验、转发 controller，并把 controller 私有 path ID
  映射回任务 goal ID；不提供 EGO 点云避障或 SEAD 动态禁飞区重规划。
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
| task -> planning | `/<uav>/planning/goal` | `geometry_msgs/PoseStamped` | 是 | 否 |
| task -> planning | `/<uav>/planning/task_path` | `nav_msgs/Path` | 否 | 是 |
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

### 4.1 多旋翼 `PoseStamped`

- `header.frame_id` 必须等于该实例的 `common_frame`，默认 `world`。
- `position.x/y/z` 必须是有限数值。
- 当前 `flight_type` 必须为 `1`，即 EGO manual-target 模式。
- `z` 会原样传给 EGO，不再被改写为 `1.0 m`；目标仍须处于配置的三维地图范围和安全空间内。
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
| `map_size_z` | `12.0` | EGO 栅格地图垂直尺寸 |
| `ground_height` | `-0.01` | EGO 地图下边界 |
| `virtual_ceil_height` | `10.0` | EGO 可飞上边界；高空任务需连同 map size 调大 |
| `goal_topic` | `/<uav>/planning/goal` | 多旋翼点目标输入 |
| `task_path_topic` | `/<uav>/planning/task_path` | 固定翼执行路径输入 |
| `status_topic` | `/<uav>/planning/status` | 统一任务状态输出 |
| `healthy_topic` | `/<uav>/planning/healthy` | 统一健康输出 |
| `diagnostics_topic` | `/<uav>/planning/diagnostics` | 统一诊断输出 |

点云、body frame 和 EGO 参数也由 `planning.launch` 显式暴露。真机接入时必须使用经过标定的
body-to-sensor 变换，不能把 frame 名称直接改写成另一个坐标系。

## 7. `xd_uav_task_allocate` 已实现的接入契约

协调器现已识别正式 `planning` 后端；旧值 `ego_swarm` 只作为兼容别名。该后端的行为是：

1. 多旋翼只发布 `planner_goal`，固定翼搜索和 worker 短路径只发布 `planner_task_path`。
2. 两种机型都只用 `planner_status` 推进 goal 生命周期，并配置 `planner_cancel`。
3. `route_preview` 仅用于可视化，不携带可执行 goal。
4. pause、stop、skip waypoint、禁用飞机、取消任务及视觉任务交接，均先请求 planning 取消；
   服务失败时不清除 active goal，也不启动下一级控制器。
5. `direct_controller_test` 只在显式配置时创建 controller publisher/订阅者，不是 planning 失败回退。

配置示例：

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
      planner_status: /uav2/planning/status
      planner_healthy: /uav2/planning/healthy
      planner_cancel: /uav2/planning/cancel
      route_preview: /uav2/planning/route_preview
```

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
- 为固定翼增加点云避障或动态禁飞区重规划；
- 对完整多机任务做长时间 SITL 与真机安全复验。
