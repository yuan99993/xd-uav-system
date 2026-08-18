# xd_uav_task_allocate

面向当前 XD UAV ROS1 工作空间的应急救援任务层。第一版采用中心协调器，负责：

- 将多个搜索多边形生成割草机覆盖航线，并按工作量分给侦察机；
- 直接读取 `xd_uav_detect` 的 `/uavX/track/detections`，无需启动 tracker 节点；
- 使用检测时间戳对应的 `world Odometry` 位姿，把 body FRD 目标转换到共享世界坐标；
- 对同机多帧和跨侦察机重复观测进行聚类，生成由本包维护的全局目标 ID；
- 目标确认后建立一次救援任务，按距离分给空闲作业机；新目标、任务完成和飞机失效才触发增量重分配。

## 执行后端与避障边界

任务分配逻辑只决定“谁去哪里”。实际执行由 `mission.yaml` 中唯一启用的后端负责，
不会在运行中自动回退或同时向两个后端下发。

当前为了先验证搜索、目标去重和任务分配，配置为：

```yaml
planner:
  backend: direct_controller_test
```

该测试后端把 world 航点转换成控制器的一次性位置 `mavros_msgs/PositionTarget`（由控制器
内部锁存），发布到：

```text
/<uav>/control/reference/setpoint
```

它不读取点云、不规划绕障路径，只允许在已确认净空的仿真或测试场使用。侦察机执行
搜索航点 XYZ；工作机默认停在目标前 2 m，并把分配时的 world 高度写入 PZ，保持明确的
垂直位置闭环，既不采用地面目标 Z，也不通过忽略 PZ 来关闭高度控制。默认根据飞机当前
位置到当前航点的 world ENU 水平方向设置 yaw，
使机头沿当前航段方向转动；水平距离过小时保持已有航向。节点根据新鲜有效的 world
Odometry，在目标容差内持续指定时间后判定 `REACHED` 并推进下一个航点。

后续接入 EGO-Swarm 时，把 backend 改回 `ego_swarm`。该模式为每架飞机发布：

```text
/<uav>/planning/goal          geometry_msgs/PoseStamped
/<uav>/planning/mission_path  nav_msgs/Path（名义搜索路线/可视化）
```

`PoseStamped.header.seq` 是本包分配的 `goal_id`。预留的 EGO-Swarm 适配器完成局部规划和避障，
并回传：

```text
/<uav>/planning/status        xd_uav_task_allocate/PlannerStatus
```

届时只有 EGO-Swarm/轨迹适配器向 controller 发布最终安全轨迹，任务包不自动绕过
规划器。`direct_controller_test` 是显式选择的阶段性验证模式，不是规划失败回退。

## 配置分层

- `config/mission.yaml`：共享坐标、目标去重、确认和分配参数；
- `config/scouts.yaml`：侦察机成员、world Odometry、健康状态、检测和规划话题；
- `config/workers.yaml`：作业机成员、world Odometry、健康状态和规划话题。

每架飞机在 `localization/world_odometry_topic` 中指定估计器主状态重发布话题，例如
`/uav1/state_estimator/main/frames/world/odom`。该接口固定为 `nav_msgs/Odometry`，节点直接读取
`pose.pose.position`、`pose.pose.orientation` 和原始估计时间戳，并严格要求
`header.frame_id == world`。

`health/control_state_topic` 仍订阅 `xd_uav_controller/ControlState`，但只使用
`state_valid/localization_valid/odometry_fresh` 判断飞机是否允许参与分配，不再读取其中的
位置与姿态。只有 world Odometry 和健康状态都新鲜有效，飞机才可参与新任务。单次状态
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
/task_allocate/mission_state   std_msgs/String（IDLE/LOADED/ACTIVE/COMPLETED）
```

下发搜索区域只加载任务并进入 `LOADED`，不会发布 `planning/goal`。现有控制系统完成解锁、
起飞并稳定后，由操作员显式开始：

```bash
rosservice call /task_allocate/start
```

默认要求 `scouts.yaml` 中全部侦察机的 world Odometry 与 ControlState 健康状态都有效；否则
服务返回失败，已加载区域不会丢失，飞机准备好后可以再次调用。成功后才生成/分配覆盖路线，
进入 `ACTIVE` 并向当前执行后端发布第一个航点。由于现有状态消息没有可靠的“已经离地”字段，
`start` 必须由地面站在确认起飞后调用，任务包不会仅凭定位正常自动判定已经起飞。

红色检测脚本持续产生的无 ID 重复检测会先成为 candidate；默认两秒内五次观测，或两架
不同侦察机分别观测后才 confirmed，并且每个全局目标只创建一个作业任务。候选目标超过
`stale_timeout_sec` 没有继续观测才会过期；已经 confirmed 并进入任务队列的目标不会在
等待作业机期间过期，仍持续参与世界坐标空间去重。

## 不驱动飞机的任务层测试

如需只验证ROS任务消息而完全不驱动飞机，应临时把 backend 改为 `ego_swarm`，并启动
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

当前工作空间已有 EGO-Swarm 源码，但尚未启用适配。后续适配器需要：

1. 订阅 `planning/goal`，把共享 frame 的目标交给 EGO-Swarm；
2. 将 EGO-Swarm 的规划/执行状态映射成 `PlannerStatus`；
3. 将安全 B-spline/轨迹适配为当前 `xd_uav_controller` 接受的
   `trajectory_msgs/MultiDOFJointTrajectory` 或流式 `PositionTarget`；
4. 对状态超时、规划失败和碰撞风险执行悬停/盘旋，不允许自动回退到任务点控制。

复制来的旧 SEAD 实现保存在 `legacy/`，不参与安装、启动和测试，也不会与原
`xd_uav_sead` Python 模块发生冲突。
