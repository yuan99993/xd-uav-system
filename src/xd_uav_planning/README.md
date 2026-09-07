# xd_uav_planning

XD-UAV 的统一 ROS1 规划层。任务层始终使用同一组接口，规划层按已配置机型选择且只选择一个
执行后端：

```text
/<uav>/planning/goal          geometry_msgs/PoseStamped
/<uav>/planning/mission_path  nav_msgs/Path
/<uav>/planning/status        xd_uav_task_allocate/PlannerStatus
```

## 后端

### multirotor / EGO-Swarm

EGO 后端包含原 `xd_uav_ego_bridge` 的状态、轨迹、点云与健康适配，以及原
`xd_uav_system_integration` 中的 EGO 专属启动、健康合取和控制引用仲裁。EGO 的私有
`quadrotor_msgs/PositionCommand` 只有在状态、frame、时间戳和健康门全部有效后，才转换成
控制器候选。`ego_status_node.py` 把目标执行过程统一回报为 `PlannerStatus`；与任务分配器
联机时从只读的 `GetMissionState` 服务恢复稳定 `goal_id`，不依赖会被 rospy 重写的顶层
`PoseStamped.header.seq`。通用入口默认使用 EGO `flight_type=1`，因此任务层发布的实时目标
会先经过规划层校验，再转发给 EGO；启动不再依赖“预设目标自动飞行”。当前官方 EGO 的
manual-target 实现固定在 `world z=1.0 m`，规划层会立即拒绝其他高度而不是静默飞错高度。
需要任意三维目标时，应在后续官方化阶段增加外部三维目标适配能力。

```bash
roslaunch xd_uav_planning ego_bridge.launch uav_name:=uav1
roslaunch xd_uav_planning ego_pointcloud_adapter.launch UAV_NAME:=uav1
```

完整仿真入口：

```bash
rosrun xd_uav_planning ego_demo.sh start single
rosrun xd_uav_planning ego_demo.sh start swarm 3
```

通用启动入口按 `vehicle_type` 只实例化一个后端；固定翼分支不会启动任何 EGO 节点：

```bash
roslaunch xd_uav_planning planning_backend.launch vehicle_type:=multirotor UAV_NAME:=uav1
roslaunch xd_uav_planning planning_backend.launch vehicle_type:=fixedwing UAV_NAME:=uav1
```

### fixedwing / Path

固定翼后端不启动 EGO，也不读取点云。它严格校验任务层给出的 world 几何 Path、固定翼
`ControlState` 和时间新鲜度，再转发到控制器已有的 Path 接口，并把 `PathStatus` 映射回统一
规划状态。任务 ID 从各 `PoseStamped` 的嵌套 Header 读取，避免 rospy 重写顶层
`Path.header.seq`；控制器内部路径 ID 在 ACCEPTED 后单独绑定，再映射回稳定任务 ID：

```bash
roslaunch xd_uav_planning fixedwing_path_backend.launch UAV_NAME:=uav1
```

不经过 SEAD 运行时的 PX4 plane 端到端验收入口：

```bash
roslaunch xd_uav_planning fixedwing_sitl_demo.launch gui:=false
```

验收节点完成 OFFBOARD 起飞、规划 Path 转发、真实位移与 REACHED 校验，最后向 control manager
提交受控降落请求；结果锁存在 `/uav1/planning/fixedwing_acceptance/result`。

```text
planning/mission_path -> control/reference/path
controller/path_status -> planning/status
```

无效机型、无效定位、错误 frame、过期/非有限路径、退化线段或控制器拒绝都会 fail-closed，
不会用错误输入替换控制器当前有效参考。

## 边界

- `xd_uav_task_allocate` 是只读参考/上游任务包；本包兼容其现有接口，不修改其实现。
- `ego-planner-swarm` 是第三方规划器；当前第一阶段保留已验证本地版本，后续将在独立提交中
  恢复官方源码并把 swarm 启动可靠性适配外置。
- 固定翼第一版只执行任务层几何 Path，不宣称提供 EGO 点云避障或 SEAD 动态禁飞区重规划。
- 起降、OFFBOARD、状态估计和底层控制仍分别属于 control manager、estimator 和 controller。
