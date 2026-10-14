# `feature/LTJ` 相对 `dev` 的能力与接入说明

本文面向需要把 `feature/LTJ` 能力带到当前 `dev` 的维护者。编写和复核基线为：

- `origin/dev`: `968c4d8bcf43ed8e9be464f67c1fc7bd3c8811dd`
- LTJ 实现基线: `02ed952b4e213fd3e53430040f03492ef5e41ad3`

若远端提交已变化，应先重新做差异和构建验证，不要把本文的兼容结论直接套到新的基线。

## 主要新增能力

| 范围 | 相对当前 `dev` 的主要增量 |
|---|---|
| 地形与规划 | 保留 `dev` 已有的 EGO-Swarm、固定翼 Path 后端和动态禁飞区在线换路；新增 `terrain-navigation`、规范 GeoTIFF 地形航迹及后置安全复核、动态机队、多机时空冲突检查和统一放行。 |
| Web 与仿真 | 保留原单机 DEM/三维预览页，新增可离线编排、也可连接 ROS/PX4/Gazebo 的固定翼集群控制台；Web、规划器和 Gazebo 用 terrain ID、SHA-256、CRS 和 MSL 基准核对同一地形。在线 WGS84 区域会规范化为局部 UTM，上传 GeoTIFF 必须有有效水平 CRS，且高程为米制 MSL。 |
| 控制链 | 保留 `dev` 已有的多旋翼/固定翼/VTOL 控制、Path 跟踪及起降服务；新增带有效位的输出合同、PX4/ArduCopter 后端分层、quaternion attitude 输出和 ArduCopter 4.7.1 SITL 适配。业务服务仍位于 `/<uav>/control_manager/*`。 |
| 感知定位 | 保留 `dev` 已有的 LiDAR-camera、地面投影、云台单束激光三种定位方式、world 输出和两轴云台接口；新增 WGS84 输出、datum/TF 严格校验、无 MRS 的飞行演示及配置兼容整理。详见 [`xd_uav_detect` 说明](src/xd_uav_detect/README.md)。 |
| 定位与 TF | 状态估计器支持多来源、健康隔离和连续切换；TF 管理器明确 `local_origin -> odom -> base_link` 所有权，并支持每个适配器自己的机体参考 frame。 |
| 运行边界 | 规划、控制和演示入口不依赖 MRS core/manager；保留 `dev` 自带的 `ego-planner-swarm`，没有把 PX4、Gazebo 或 MRS 源码复制进这些包。 |

原 `dev` 的 controller/control manager 话题与服务名、detect 原输入输出和四个旧消息、planning 正式入口与禁飞区消息均保留。`ControlCommand.msg` 和 `ControlState.msg` 增加了字段，因此旧生成消息和旧二进制不能复用。

## 在 `dev` 上采用：完整目录替换

已验证的最小完整闭包是下面 8 个目录：

```text
src/xd_uav_planning
src/xd_uav_controller
src/xd_uav_control_manager
src/xd_uav_detect
src/xd_uav_single_tf_manager
src/xd_uav_state_estimators
src/terrain-navigation
src/web
```

必须替换目录，而不是用 `cp -r` 叠加文件；否则 LTJ 已删除的旧 MRS 脚本会残留。不要替换
`src/ego-planner-swarm`，继续使用 `dev` 自带版本。

推荐在全新、干净的克隆中执行：

```bash
git fetch origin
git switch -c integrate-ltj origin/dev
git status --short

git restore --source origin/feature/LTJ --staged --worktree -- \
  src/xd_uav_planning \
  src/xd_uav_controller \
  src/xd_uav_control_manager \
  src/xd_uav_detect \
  src/xd_uav_single_tf_manager \
  src/xd_uav_state_estimators \
  src/terrain-navigation \
  src/web

git diff --cached --stat
```

执行 `git restore` 前，`git status --short` 必须无输出。上面的命令会把 8 个路径在索引和工作树中都还原为 LTJ 版本，因此也会删除这些路径内仅由 `dev` 跟踪的旧文件；先在新分支操作并审查暂存差异。

覆盖后必须使用新的 `build`/`devel` 结果做完整构建，不得复用旧消息生成物。推荐使用全新 catkin 构建工作区；按团队方式清理生成目录后，基本构建入口为：

```bash
cd /path/to/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
```

Web 的 Conda/Node 首次准备命令只维护在 [`src/web/使用说明.md`](src/web/%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.md)，不要提交 Conda 环境、`node_modules`、构建结果或运行日志。

## `dev` 原功能兼容审计

已在独立的最新 `origin/dev` 副本中按上面的 8 目录执行真实替换，而不是只比较当前 LTJ
工作树。覆盖后的核心闭包完成从零构建，并连同 `dev` 原有的 `xd_uav_task_allocate`、
`xd_uav_sead`、EGO-Swarm 依赖和 world TF manager 运行自动测试；最终汇总为 328 tests、0 errors、
0 failures、0 skipped。目标包中 `dev` 原测试文件删除数为 0，四个核心包的公共
`.launch` 文件名删除数也为 0。

接口兼容边界如下：

- `NoFlyZone*`、`WorldDetection*` 以及原 controller 服务定义未改；原调用方重编后可继续使用。
- `ControlCommand.msg` 和 `ControlState.msg` 只在末尾增加字段，但 ROS1 消息 MD5 会改变；
  必须清理旧生成物并完整重编整个工作区，不能混用旧二进制节点。
- `detect.launch`、`detect/demo.launch`、`planning.launch` 和原 EGO/固定翼演示入口仍保留。
  原 MRS 专用演示内部件已换成仓库自己的 PX4/control-manager 组合，不再要求 MRS core/manager。
- 少数非公共源码路径发生迁移：planning 的自动起飞脚本移到 `xd_uav_control_manager`；
  MRS spawn 适配脚本被 `px4_sitl_vehicle.launch` 取代；detect 的旧 MRS 自动起飞脚本/配置被
  新 `demo.launch` 取代；红框演示脚本移到 `scripts/demo/`，但安装后的 `rosrun` 名称不变。
  仓库内调用点已经同步，若外部脚本直接写死这些源码路径，需要按新入口迁移。

因此，当前证据支持“完整替换这 8 个目录并全量重编”保持 `dev` 的公开能力；不支持把两套
生成目录或旧二进制混在一起运行，也不承诺兼容外部工具对包内私有脚本路径的硬编码。

## 最短演示流程

以下命令均从仓库根目录执行。Web 可以先独立运行，不要求 ROS/PX4/Gazebo：

```bash
scripts/fixedwing_mission.sh web start
```

打开 `http://127.0.0.1:5173/` 选择在线区域或上传 GeoTIFF，点击“启用当前地形并进入任务控制台”；然后启动执行系统：

```bash
scripts/fixedwing_mission.sh sim start --gui
```

已有激活地形或直接使用仓库 Davosdorf 样例时，也可一次启动：

```bash
scripts/fixedwing_mission.sh start --gui
```

常用观察和统一停止命令：

```bash
scripts/fixedwing_mission.sh status
scripts/fixedwing_mission.sh view uav1
scripts/fixedwing_mission.sh logs
scripts/fixedwing_mission.sh stop
```

页面用途和逐步操作以 [`src/web/使用说明.md`](src/web/%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.md) 为准。其他已验证入口不在本文重复展开：

- 单机/三机 EGO-Swarm：[`EGO_FULL_DEMO_RUNBOOK.md`](src/xd_uav_planning/docs/EGO_FULL_DEMO_RUNBOOK.md)
- 固定翼动态禁飞区：[`FIXEDWING_DYNAMIC_NOFLY.md`](src/xd_uav_planning/docs/FIXEDWING_DYNAMIC_NOFLY.md)
- 任务层接入：[`TASK_PLANNING_INTEGRATION.md`](src/xd_uav_planning/docs/TASK_PLANNING_INTEGRATION.md)
- 控制器与 control manager：[`xd_uav_controller/README.md`](src/xd_uav_controller/README.md)、[`xd_uav_control_manager/docs/USAGE.md`](src/xd_uav_control_manager/docs/USAGE.md)
- detect 定位、云台和 Gazebo 演示：[`xd_uav_detect/docs/USAGE.md`](src/xd_uav_detect/docs/USAGE.md)
- 多来源估计与 TF：[`xd_uav_state_estimators/README.md`](src/xd_uav_state_estimators/README.md)、[`xd_uav_single_tf_manager/README.md`](src/xd_uav_single_tf_manager/README.md)

## 关键新增或扩展接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/terrain_swarm/tasks` | `xd_uav_planning/TerrainTaskBatch` | 定义本批参与飞机、任务编号和标称速度。 |
| `/<uav>/planning/task_path` | `nav_msgs/Path` | 每机任务航点输入；固定翼地形规划的正式输入。 |
| `/terrain_swarm/status` | `xd_uav_planning/TerrainSwarmStatus` | 汇总每机规划状态和时空冲突。 |
| `/terrain_swarm/release` | `std_msgs/Bool` | 安全检查通过后统一放行或关闭执行输出。 |
| `/terrain_swarm/fleet` | `xd_uav_planning/TerrainFleetStatus` | 当前动态飞机实例及连接、解锁、落地状态。 |
| `/terrain_swarm/terrain_manifest` | `std_msgs/String` | 锁存执行端正在使用的规范地形身份。 |
| `/terrain_swarm/fleet/spawn` | `xd_uav_planning/SpawnTerrainVehicle` | 创建完整 PX4/MAVROS/控制/规划实例。 |
| `/terrain_swarm/fleet/remove` | `xd_uav_planning/RemoveTerrainVehicle` | 仅在安全落地、上锁条件满足时删除实例。 |
| `/<uav>/control_manager/{offboard,takeoff,land,land_home,cancel_land,cancel_offboard}` | ROS services | 对外飞行业务入口；不要直接调用 controller 内部服务。 |
| `/<uav>/detect/detections_world` | `xd_uav_detect/WorldDetectionArray` | 检测时刻的世界坐标和协方差。 |
| `/<uav>/detect/detections_geodetic` | `xd_uav_detect/GeodeticDetectionArray` | 可选 WGS84 经纬度、椭球高和 ENU 协方差。 |
| `/<uav>/gimbal/{command,state}` | `GimbalCommand` / `GimbalState` | 与 Gazebo 或厂商执行适配器解耦的两轴云台接口。 |

通用 planning 接口（`planning/status`、`planning/healthy`、`planning/cancel`、动态禁飞区等）及完整消息语义见 [`src/xd_uav_planning/README.md`](src/xd_uav_planning/README.md)，不要仅凭本表编写新的任务发布器。

## 已验证边界

上述 8 目录替换方法已在干净 `origin/dev` 副本上验证为与 LTJ 对应目录完全一致；混合工作区的核心闭包和直接 dev 消费包从零构建通过，EGO、固定翼和三机地形集群 launch 解析通过，相关 ROS 测试复测通过，Web 后端 38 项测试和 Vite production build 通过。覆盖兼容审计没有在该临时混合目录再次执行完整 Gazebo 飞行；固定翼三机 SITL 闭环是在 LTJ 工作区完成的。接入者仍应在自己的 PX4/Gazebo、网络和显卡环境中按使用说明做一次 GUI 与三机飞行验收。
