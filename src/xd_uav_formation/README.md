# xd_uav_formation

独立编队控制包，包含 **领机跟随** 和 **一致性编队** 两种模式。算法参考 `/home/kzy/swiftwing_ws` 对应的 `swarm_lf/Follower.py`、`ConsensusFollow.py`、`DesirePath.py`、`FormationFun.py`。这里保留领机槽位、转弯速度补偿、虚拟编队中心、环形邻接一致性修正和共同航线预瞄的计算方法。姿态与加速度闭环由本工程已有的 `xd_uav_controller` 负责；本包向每架飞机自己的 `/<uav>/control/reference/setpoint` 发布 `mavros_msgs/PositionTarget` 位置及速度参考，不直接解锁、切模式或控制油门。领机跟随时领机继续使用自己的导航输入，例如 `/uav1/control/reference/trajectory`；编队节点不向领机发布参考。

两种模式都要求同队飞机有相同的 `airframe_type` 和飞行阶段。支持多旋翼、固定翼、VTOL 和倾转旋翼；VTOL/倾转旋翼可在同为悬停或同为前飞时编队，阶段转换立即结束编队。固定翼默认槽位离队形中心至少 30 m，参考速度须在 11–24 m/s 之间，与本工程现有固定翼控制器默认空速范围一致；具体间距、速度和航线应根据机型调整。跨飞机的定位使用 TF，必须先启动各机本地 TF 和多机公共坐标系。

## 统一控制接口

启动时调用 `/formation/set_enabled`，通过 `mode` 和 `members` 选择要控制的编队节点。停止时调用全局接口 `/formation/stop`，不需要填写编队模式或成员名单；管理器会停用当前运行的全部编队节点。队伍标识由管理器从成员节点配置中推断并校验，不需要放进启动请求。三机一致性编队示例：

```bash
rosservice call /formation/set_enabled "mode: 'consensus' members: ['uav1', 'uav2', 'uav3'] enabled: true"
rosservice call /formation/stop "{}"
```

三机领机跟随时，`members` 只列出实际运行编队节点的僚机：

```bash
rosservice call /formation/set_enabled "mode: 'leader_follow' members: ['uav2', 'uav3'] enabled: true"
rosservice call /formation/stop "{}"
```

无论是领机跟随还是一致性编队，也无论编队有几架飞机，都用这一条命令停止当前所有编队：

```bash
rosservice call /formation/stop "{}"
```

三机 VTOL 的组合 launch 会自动启动统一服务节点。若使用 `consensus_member.launch` 或 `follower.launch` 分别启动飞机，先单独启动一次 `roslaunch xd_uav_formation formation_manager.launch`。如果管理器已由系统 launch 启动，可在组合 launch 中传入 `start_manager:=false`。管理器会从成员节点配置中读取队伍标识，并检查模式、队伍和成员是否一致；不一致时拒绝操作。

## 领机跟随

领机通过原有导航链飞行，每架僚机启动一个节点。`offset: [forward, left, up]` 是相对领机航向的米数；固定翼按地速航向旋转槽位，多旋翼按机体偏航旋转。领机转弯时目标速度包含 `ω × R(offset)` 补偿。

僚机落后于目标槽位时，在上述移动槽位速度上叠加 `catchup_gain × 水平位置误差`，误差缩小时追赶速度也随之下降。默认增量最多 3 m/s，最终水平参考速度对多旋翼限制在 5 m/s 以内，对固定翼限制在 11–24 m/s；这些数值可用 `catchup_gain`、`max_catchup_speed`、`multirotor_max_speed`、`fixedwing_min_speed`、`fixedwing_max_speed` 调整，并应与控制器限速一致。飞机达到自身速度上限后若仍追不上领机，编队不能保证恢复间距；本包不会修改领机的导航速度。

```bash
roslaunch xd_uav_formation follower.launch leader_name:=uav1 follower_name:=uav2 forward:=-5 left:=4
roslaunch xd_uav_formation follower.launch leader_name:=uav1 follower_name:=uav3 forward:=-5 left:=-4
roslaunch xd_uav_formation formation_manager.launch
rosservice call /formation/set_enabled "mode: 'leader_follow' members: ['uav2', 'uav3'] enabled: true"
```

三架 VTOL 可用独立 launch 分别启动悬停和前飞阶段的领机跟随。两种启动方式都以 `uav1` 为领机；领机继续由已有导航链控制，编队节点只控制 `uav2`、`uav3`。悬停槽位较近：

```bash
roslaunch xd_uav_formation leader_follow_three_vtol_hover.launch
rosservice call /formation/set_enabled "mode: 'leader_follow' members: ['uav2', 'uav3'] enabled: true"
```

停止时调用 `/formation/stop`。三架 VTOL 全部完成前飞转换后，结束悬停 launch，再启动前飞 launch；前飞槽位为 `[-90, 45, 0]` 和 `[-90, -45, 0]` 米：

```bash
roslaunch xd_uav_formation leader_follow_three_vtol_forward.launch
rosservice call /formation/set_enabled "mode: 'leader_follow' members: ['uav2', 'uav3'] enabled: true"
```

前飞 launch 启动前，三机都必须已经处于 `flight_regime: 2`、`vehicle_type: 1`，且 `transition_pending: false`。悬停时要求三机处于 `flight_regime: 0`、`vehicle_type: 0`。这些 launch 不负责切换飞行阶段；同一时间只运行一种编队模式。

固定翼或 VTOL 前飞应使用更大间距，例如 `forward:=-80 left:=40`。`shape:=circle` 或 `shape:=rectangle_compact` 可按 `follower_count`、`slot_index`、`spacing` 自动生成槽位；手工指定偏移时使用默认 `shape:=manual`。同队僚机应设置相同的 `leader_name` 或显式 `team_id`，但 `slot_index` 不同。

圆形槽位沿用 Swiftwing 的角度排列；为避免少量飞机时圆半径过小，这里将半径下限设为 `spacing`。

### 四架 VTOL 转前飞后的领机跟随

`launch/leader_follow_four_vtol_forward.launch` 以 `uav1` 为领机，启动 `uav2`、`uav3`、`uav4` 三个僚机节点。三个槽位相对领机前飞航向分别为 `[-90, 45, 0]`、`[-90, -45, 0]`、`[-150, 0, 0]` 米，形成尾随队形。领机仍由自身导航或轨迹控制；该启动文件不替领机生成航线，也不执行 VTOL 转换。

四架均进入前飞状态后，结束已有的同名僚机编队节点，再启动：

```bash
roslaunch xd_uav_formation leader_follow_four_vtol_forward.launch
rosservice call /formation/set_enabled "mode: 'leader_follow' members: ['uav2', 'uav3', 'uav4'] enabled: true"
```

领机跟随模式中 `uav1` 不属于服务请求的 `members`。可以用控制器已有的圆轨迹测试节点给领机一条持续航线，例如另开终端运行：

```bash
rosrun xd_uav_controller publish_fixedwing_trajectory.py \
  _uav_name:=uav1 _radius:=250.0 _airspeed:=18.0 _direction:=ccw
```

当前 `vtol_system.launch` 默认把**每架飞机的控制器**都接到全局 `/move_base_simple/goal`。直接向该话题发送 RViz 目标会同时影响多架飞机；只想给领机下目标时，使用上述 `/uav1/control/reference/trajectory` 接口，或在每架控制器启动时把 `simple_goal_topic` 分别指定为各自的专用话题。

先让飞机在悬停阶段拉开间距再转前飞；领机跟随会立即追向槽位，没有一致性编队的分高度集结阶段。固定翼速度限制仍由 `config/formation.yaml` 控制，测试时请观察空速和相互间距。停止时调用 `/formation/stop`。

## 一致性编队

每架飞机都运行一个节点，没有单独的领机。所有成员使用完全相同的 `members` 顺序、`team_id`、`shape`、`spacing`、航点及控制参数。统一服务按 `members` 顺序启动节点；第一架先建立集结计划，其余成员随后加入。若飞机尚未在槽位附近，编队先自动集结：按成员顺序分配不同高度，各机保持当前位置升至自己的高度，再横向飞往由当前机群位置计算的槽位，最后在槽位上合到航线高度。集结完成后才开始共同航线跟随。这个过程按 `members` 数量和 `circle`、`rectangle_compact` 槽位计算，不依赖四机演示的初始位置。

一致性编队的成员顺序仍由配置中的 `members` 决定；统一控制服务会根据成员节点配置校验队伍、模式和成员顺序，避免误启动另一组节点。自定义队伍也使用同一个 `/formation/set_enabled` 服务，请求只填写模式和成员列表；各节点的 `team_id` 仍需在配置中设成一致。

跟随阶段，节点从本机位置减去旋转后的槽位得到虚拟编队中心，向全队发布中心；用全队虚拟中心的平均值选择共同航线段和预瞄点，再结合环形邻居中心的拉普拉斯修正生成参考。每架飞机都参与路径跟踪。

若某机的虚拟中心落后于邻机，一致性修正会提高其参考速度；超前时降低速度。修正量默认最多 4 m/s，最终请求仍受上述多旋翼或固定翼速度范围限制。

`config/consensus.yaml` 是三机前飞示例。参与编队的飞机都必须在 `xd_uav_world_tf_manager/config/world.yaml` 中登记；两架多旋翼试跑请使用 `config/consensus_two_multirotor.yaml`：

```bash
roslaunch xd_uav_formation consensus_member.launch member_name:=uav1 config:=$(rospack find xd_uav_formation)/config/consensus_two_multirotor.yaml
roslaunch xd_uav_formation consensus_member.launch member_name:=uav2 config:=$(rospack find xd_uav_formation)/config/consensus_two_multirotor.yaml
roslaunch xd_uav_formation formation_manager.launch
rosservice call /formation/set_enabled "mode: 'consensus' members: ['uav1', 'uav2'] enabled: true"
```

如果启动三架，先在 world 配置中登记 `uav3` 并让它的 TF 有效，然后使用三机配置：

三架 VTOL 的一致性编队也分别提供悬停和前飞 launch。先启动与当前飞行阶段匹配的文件，等三机状态、TF、解锁和 OFFBOARD 均就绪后，只调用一次统一服务：

```bash
roslaunch xd_uav_formation consensus_three_vtol_hover.launch
rosservice call /formation/set_enabled "mode: 'consensus' members: ['uav1', 'uav2', 'uav3'] enabled: true"
# 停止时：rosservice call /formation/stop "{}"
```

前飞时确认 VTOL 已完成转换，再切换到前飞 launch 和对应服务：

```bash
roslaunch xd_uav_formation consensus_three_vtol_forward.launch
rosservice call /formation/set_enabled "mode: 'consensus' members: ['uav1', 'uav2', 'uav3'] enabled: true"
# 停止时：rosservice call /formation/stop "{}"
```

悬停配置的示例高度/速度为 20 m、2 m/s；前飞配置为 80 m、18 m/s，槽位间距 60 m。请按实际空域、飞机性能和控制器限速检查并调整对应 YAML。也可用 `config:=/path/to/your.yaml` 覆盖配置；服务请求中的 `members` 必须与各机加载的配置一致，队伍标识由管理器从配置读取。悬停要求三机都在悬停阶段，前飞要求三机都完成前飞转换；不要同时运行两种编队 launch。

四架多旋翼用 `config/consensus_four_multirotor.yaml`。四架 VTOL 在约 20 m 高度悬停时可用 `config/consensus_four_vtol_hover.yaml`；该配置沿 120 m 方形航线飞行，30 m 半径的圆弧连接转角，四个槽位构成半径 12 m 的菱形。`cruise_speed` 为 2 m/s，`lookahead_distance` 为 3 m；两者应配合控制器位置增益调节实际速度。起飞稳定后通过 `/formation/set_enabled` 请求启动，节点会从当前机位自动集结。两者都需要在 world 配置中登记四架飞机，并让控制状态和公共 TF 有效。VTOL 若要先切入前飞阶段，应结束悬停编队并为前飞重新设置速度、间距和航点。

```bash
roslaunch xd_uav_formation consensus_member.launch member_name:=uav1
roslaunch xd_uav_formation consensus_member.launch member_name:=uav2
roslaunch xd_uav_formation consensus_member.launch member_name:=uav3
roslaunch xd_uav_formation formation_manager.launch
rosservice call /formation/set_enabled "mode: 'consensus' members: ['uav1', 'uav2', 'uav3'] enabled: true"
```

先启动全部节点并等待状态、TF、邻居中心就绪，再通过统一服务启动。`config` 启动参数可指向自定义 YAML。对于多旋翼，应降低示例中的 `cruise_speed` 和 `spacing`；示例航点和 20 m/s 速度面向前飞演示，不适合直接用于所有机型。固定翼转弯半径和航点距离也应在仿真中按机型验证。

编队人数、形状和航线均从配置读取，不需要为每种人数修改算法。例如五架多旋翼沿三角航线飞行，可复制 `config/consensus_four_multirotor.yaml`，改为：

```yaml
team_id: team5
members: [uav1, uav2, uav3, uav4, uav5]
shape: circle                 # 也可设为 rectangle_compact
spacing: 10.0
waypoints:
  - [0.0, 0.0, 20.0]
  - [120.0, 0.0, 20.0]
  - [60.0, 105.0, 20.0]
loop_path: true
cruise_speed: 2.0
lookahead_distance: 10.0
```

五架节点加载同一份配置，`member_name` 分别设为 `uav1` 到 `uav5`；在 world TF 配置中登记全部成员。各机起飞并进入 OFFBOARD 后，只调用一次 `rosservice call /formation/set_enabled "mode: 'consensus' members: ['uav1', 'uav2', 'uav3', 'uav4', 'uav5'] enabled: true"`，系统会按五架飞机的实时位置分层集结，然后沿三角航线飞行。停止时调用 `rosservice call /formation/stop "{}"`。

### 四架 VTOL 转前飞后测试

使用 `config/consensus_four_vtol_forward.yaml`。这份配置仍是四架 VTOL，同为**前飞阶段**后按固定翼速度和转弯半径飞行；槽位间距 60 m，航线高度 80 m，速度 18 m/s，圆角半径 250 m。不要把悬停编队配置用于前飞。现有 `session_four_vtol_px4.yml` 的 `formation2` 窗口可能已经在运行其他编队节点；先结束该窗口的旧编队节点，避免相同 ROS 节点名冲突，再启动：

```bash
roslaunch xd_uav_formation consensus_four_vtol_forward.launch
```

四架 VTOL 建议先起飞至约 80 m 并稳定，确保最高约 125 m 的集结高度可用。当前四机出生点仅相隔 3 m，应在悬停阶段先拉开间距，再分别请求前飞转换；前飞编队的分层集结发生在转换**完成后**，不会保护转换过程。服务返回 `accepted: true` 仅表示受理，需要等 `/<uav>/control_manager/state` 中的 `flight_regime: 2`、`vehicle_type: 1`、`transition_pending: false`，且各机保持 OFFBOARD、水平地速不低于 11 m/s。转换命令中的 `target_regime: 1` 是**服务请求值**，与完成后的状态值 `flight_regime: 2` 不同：

```bash
for uav in uav1 uav2 uav3 uav4; do
  rosservice call /$uav/control_manager/set_flight_regime "target_regime: 1"
done
```

确认四架都完成前飞转换后，只调用一次：

```bash
rosservice call /formation/set_enabled "mode: 'consensus' members: ['uav1', 'uav2', 'uav3', 'uav4'] enabled: true"
```

节点先分高度并自动靠向槽位，再沿航线飞行。测试状态可看 `/<uav>/formation_consensus/status`，全队进入 `active:following` 表示集结完成。切回悬停前先调用 `/formation/stop`。固定翼分支已做 ROS 状态流程测试，尚需在 PX4 四机仿真中验证实际轨迹；首次测试应观察各机空速、间距和高度。

## 起停与任务接管

区域搜索任务接管前，先调用全局停止接口，并等服务成功返回，再启动搜索：

```bash
rosservice call /formation/stop "{}"
rosservice call /task_allocate/start "{}"
```

收到本机其他发布者的 `control/reference/setpoint`、`control/reference/path`、`control/reference/trajectory`，或 `task_execute/execute/goal` 时，该机立即停发编队参考，并通过 `/formation/<team_id>/cancel` 通知全队退出。控制器接受的 simple goal 会转换为本机的 `control/reference/setpoint`，因此也能触发接管；编队节点不直接订阅全局 `/move_base_simple/goal`。外部任务也可提前向 `formation/preempt` 发布一次 `std_msgs/String`。起飞、降落、返航、应急、失去 OFFBOARD、状态或 TF 超时、机型/阶段不一致、邻机中心丢失也会结束编队。退出后不会自动恢复，必须再次调用开始服务。领机跟随模式下，领机原有导航参考不算僚机的新任务。

状态见 `/<uav>/formation_follower/status` 或 `/<uav>/formation_consensus/status`，如 `active:following`、`inactive:task_goal:42`。普通失联/手动结束时，多旋翼尝试发布一条当前位置保持参考；新任务到达时停发参考，让接管功能的控制输入生效。固定翼停发后由已有控制器的参考超时策略接管。

配置项和默认值见 `config/formation.yaml`、`config/consensus.yaml`。`common_frame` 必须是所有飞机 TF 可达的同一 ENU 坐标系；航点也必须在该坐标系中。集结用 `acquire_tolerance`、`acquire_vertical_spacing`、`acquire_height_tolerance` 和 `acquire_timeout` 调整。悬停机型按高度分层后才横向移动；固定翼在保持前飞时分层并向槽位靠拢。起飞前仍须有足够初始间距，航线和高度层应位于可飞区域；本包没有通用障碍物避让能力。
