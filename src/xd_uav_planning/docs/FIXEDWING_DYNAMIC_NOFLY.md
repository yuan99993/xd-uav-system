# 固定翼动态禁飞区规划说明

更新：2026-09-09
适用分支：`feature/LTJ`

包级入口、两种机型后端和公共话题总览请先看上级目录的
[README](../README.md)；本文只展开固定翼动态禁飞区功能。

## 1. 功能结论

`xd_uav_planning` 固定翼后端已经支持：

- 在任务 Path 首次下发 controller 前绕开已有禁飞区；
- 飞行中通过真实 ROS 话题新增、替换、删除或清空禁飞区；
- 以飞机实时位置为新起点，对原任务剩余路线重新规划并替换 controller Path；
- 删除或清空禁飞区后，重新连接到原任务的剩余路线；
- 无安全解时返回 `PlannerStatus.BLOCKED`，并请求 control manager 退出 OFFBOARD，禁止继续
  执行已知冲突旧路径。

此功能位于规划层，不启动或依赖 `xd_uav_sead`。SEAD 仅用于设计审计和行为参考，没有被
import、链接、包含或作为 launch 子项启动。

## 2. 数据流与包边界

```text
task Path ----------------------┐
NoFlyZoneArray batch updates ----+--> xd_uav_planning --> controller Path
fixed-wing ControlState ------┘                         replacement

controller PathStatus ------------> planning PlannerStatus
planning no-safe-path ------------> control_manager/cancel_offboard
```

运行时职责：

| 组件 | 本功能使用方式 |
|---|---|
| `xd_uav_planning` | 禁飞区校验/缓存、相交检查、Dubins 绕飞、剩余路径重规划、状态输出 |
| `xd_uav_controller` | 提供 `ControlState`/`PathStatus`，原子接收新的 `nav_msgs/Path` |
| `xd_uav_control_manager` | 仅在在线重规划无解时提供 `cancel_offboard` 失效保护 |
| `xd_uav_task_allocate` | 当前只提供既有 `PlannerStatus` 消息类型和上游任务 Path |
| `xd_uav_sead` | 无运行时或构建依赖 |

正式 `planning.launch` 本身只启动规划节点。SITL 演示额外组合 PX4、Gazebo、MAVROS、状态估计、
controller 和 control manager，这些是仿真/飞控闭环依赖，不是禁飞规划算法对 SEAD 的依赖。

## 3. ROS 接口

固定翼规划节点以 `uav1`、`uav2` 等不同命名空间运行，但禁飞区输入使用全局共享话题：

| 方向 | 话题 | 类型 |
|---|---|---|
| task -> planning | `/uav1/planning/task_path` | `nav_msgs/Path` |
| airspace -> all fixed-wing planners | `/planning/no_fly_zones` | `xd_uav_planning/NoFlyZoneArray` |
| planning -> RViz | `/planning/no_fly_zone_markers` | `visualization_msgs/MarkerArray` |
| state -> planning | `/uav1/control_manager/state` | `xd_uav_controller/ControlState` |
| planning -> controller | `/uav1/control/reference/path` | `nav_msgs/Path` |
| controller -> planning | `/uav1/controller/path_status` | `xd_uav_controller/PathStatus` |
| planning -> task | `/uav1/planning/status` | `xd_uav_task_allocate/PlannerStatus` |
| task -> planning | `/uav1/planning/cancel` | `xd_uav_task_allocate/CancelPlanning` |

禁飞区数组和任务 Path 必须使用同一个 `common_frame`。当前系统默认使用 `world`；实际系统以
各固定翼 `planning.launch` 的 `common_frame` 参数为准，不进行静默 frame 重标记。

### 3.1 批量新增或替换禁飞区

同一个非零 `zone_id` 再次 UPSERT 表示替换该区域。一次消息可以放多个更新：

```bash
rostopic pub -1 /planning/no_fly_zones xd_uav_planning/NoFlyZoneArray "{header: {stamp: now, frame_id: 'world'}, zones: [{schema_version: 1, operation: 0, zone_id: 7101, enabled: true, zone_type: 0, min_altitude: 0.0, max_altitude: 100.0, valid_until: {secs: 0, nsecs: 0}, polygon: {points: [{x: 80.0, y: -20.0, z: 0.0}, {x: 110.0, y: -20.0, z: 0.0}, {x: 110.0, y: 20.0, z: 0.0}, {x: 80.0, y: 20.0, z: 0.0}]}}, {schema_version: 1, operation: 0, zone_id: 7102, enabled: true, zone_type: 0, min_altitude: 0.0, max_altitude: 100.0, valid_until: {secs: 0, nsecs: 0}, polygon: {points: [{x: 160.0, y: -20.0, z: 0.0}, {x: 190.0, y: -20.0, z: 0.0}, {x: 190.0, y: 20.0, z: 0.0}, {x: 160.0, y: 20.0, z: 0.0}]}}]}"
```

数组头部的 `frame_id` 会补给区域中为空的 `header.frame_id`，所以每个区域不需要重复填写坐标系。

### 3.2 删除或清空禁飞区

```bash
rostopic pub -1 /planning/no_fly_zones xd_uav_planning/NoFlyZoneArray "{header: {stamp: now, frame_id: 'world'}, zones: [{schema_version: 1, operation: 1, zone_id: 7101, enabled: false, zone_type: 0, min_altitude: 0.0, max_altitude: 0.0, valid_until: {secs: 0, nsecs: 0}, polygon: {points: []}}]}"
```

清空全部区域：

```bash
rostopic pub -1 /planning/no_fly_zones xd_uav_planning/NoFlyZoneArray "{header: {stamp: now, frame_id: 'world'}, zones: [{schema_version: 1, operation: 2, zone_id: 0, enabled: false, zone_type: 0, min_altitude: 0.0, max_altitude: 0.0, valid_until: {secs: 0, nsecs: 0}, polygon: {points: []}}]}"
```

消息约束：

- `schema_version=1`，当前仅支持 `TYPE_NO_FLY=0`；
- UPSERT 的 `zone_id` 必须非零，CLEAR 的 `zone_id` 必须为零；
- 多边形至少 3 个点、不得自交、面积和坐标范围必须满足配置；
- `min_altitude < max_altitude`，使用与 Path 相同坐标系下的高度；
- `header.stamp` 不作为禁飞区新鲜度门槛；允许手动发布的零时间戳和延迟到达消息；
- `valid_until=0` 表示永久有效，直到 REMOVE/CLEAR；`header.stamp` 只保留为消息元数据，
  不参与消息新鲜度拒收。当前版本对非零 `valid_until` 做合法性
  校验，但为安全起见不会自动删除过期区域，仍应由上层显式 REMOVE/CLEAR。

## 4. 在线换路行为

backend 保存任务层原始 Path，而不是把上一次绕飞结果当成新任务。飞行中收到合法区域更新时：

1. 将实时位置投影到原任务折线，记录沿原路线的单调进度；
2. 生成“当前位置 + 原任务后续节点”的剩余路线；
3. 用当前有效禁飞区集合检查完整线段和高度重叠；
4. 路径受阻时，按当前航向、最小转弯半径和安全净距生成 Dubins 绕飞；
5. 对生成结果做连续采样净距检查；
6. 一次发布完整新 Path，controller 回调原子替换当前路径；
7. 将 controller 私有 path ID 状态映射回原任务 goal ID。

REMOVE/CLEAR 也会触发步骤 1～6，因此飞机会从当前位置回归原任务的剩余路线。重复任务
`goal_id` 在尚未终止时仍会被忽略，禁飞区更新不依赖任务层重发 Path。

规划器是带最终安全校验的启发式 Dubins 绕飞，不保证对所有复杂多边形组合找到全局可行解。
任务层主动取消时使用 `/uav1/planning/cancel`，固定翼后端发布安全释放参考并结束当前 Path。
找不到安全路径时不会回退到原冲突路线，而是 `BLOCKED + cancel_offboard`。

## 5. 配置

正式配置位于 `config/fixedwing_path.yaml`：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `turning_radius` | 35 m | 固定翼规划最小转弯半径 |
| `zone_clearance` | 15 m | 多边形水平安全净距 |
| `planning_sample_step` | 2 m | Dubins 生成和连续安全检查采样步长 |
| `zone_max_message_age` | 1 s | 兼容保留参数，当前不用于禁飞区消息拒收 |
| `zone_max_ttl` | 3600 s | 非零有效期允许的最大 TTL |
| `zone_max_vertices` | 64 | 单个多边形最大顶点数 |
| `dynamic_failure_cancel_service` | `control_manager/cancel_offboard` | 在线无解失效保护服务；空值表示禁用自动请求 |

转弯半径和净距必须依据具体飞机性能与任务安全要求配置，不能把 SITL 数值直接作为实机标定。

## 6. 演示与可视化

启动飞行中新增/删除禁飞区的完整演示：

```bash
roslaunch xd_uav_planning fixedwing_sitl_demo.launch \
  gui:=true dynamic_nofly:=true visualize:=true path_length:=300 \
  zone_center_fraction:=0.45
```

演示流程全部通过真实话题：起飞后先下发任务 Path，约 8% 航程时 UPSERT 禁飞区；飞机完成
绕飞并沿原任务方向越过多边形最远边界、规划净距和额外 5 m margin 后，才发布 REMOVE；随后
检查第二次 Path 替换、任务完成和受控降落。

可视化订阅正式话题，不直接访问 backend 内存。窗口显示：

- 灰色虚线：任务层原始 Path；
- 橙色细线：已被替换的 controller Path；
- 蓝色线：当前 controller Path；
- 绿色线：飞机实际任务航迹；
- 红色实心区域：活动禁飞区；
- 红色虚线区域：已经删除的禁飞区。

停止时保存到 `/tmp/xd_uav_planning_visualizations/<时间>/`：

- `summary.png`；
- `events.json`；
- `trajectory.csv`。

无显示器时设置 `MPLBACKEND=Agg` 或直接使用无 GUI 环境，仍会生成上述证据。

## 7. 已完成验证

- `catkin_make -j2 --pkg xd_uav_planning`：通过；
- planning 全量回归：39 tests、0 errors、0 failures、0 skipped；
- backend ROS 测试覆盖首次绕飞、飞行中 UPSERT 换路、REMOVE 恢复和 controller 状态映射；
- 300 m 固定翼动态 SITL：两次在线替换，规划最小净距 12.45 m（要求 10 m），任务位移
  281.9 m，正常完成路径、降落并解锁；
- 第二轮演示中禁飞区 x 范围约 `[199.5, 219.5] m`，飞机到达约
  `(238.2, 6.4, 25.2) m` 后才 REMOVE，证明区域保留到完成绕飞并越过安全边界。

本次只完成 SITL 与软件回归，尚未进行真实飞机、真实空域数据源或硬件在环验收。
