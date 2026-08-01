# 阶段 3 控制链复用与排障手册

本文记录 SEAD 接入 `xd_uav_control_manager`/`xd_uav_controller` 时已经踩过的坑、解决方法和可重复的人工验证流程。它面向 MRS Gazebo x500 仿真，不是真机参数手册。

## 控制链与边界

```text
mock_gcs / SEAD 任务
        -> /uav1/control/reference/setpoint
        -> xd_uav_control_manager
        -> xd_uav_controller
        -> /uav1/mavros/setpoint_raw/attitude
        -> PX4 OFFBOARD -> Gazebo x500
```

- `sead_xd_control.launch` 启动状态估计器、仓库 manager/controller 和 SEAD，不启动 PX4、MAVROS 或 Gazebo。
- MRS Gazebo spawner 可以作为 PX4+MAVROS 底座，但不能再启动 `mrs_uav_core`。MRS manager 和仓库 manager 都占用 `/uav1/control_manager`，并行时会冲突。
- `direct_mavros` 与 `xd_control_manager` 是互斥后端。联合验证时不要再启动任何其他会发 PX4 setpoint 的控制器。

## 已遇到的问题与解决方法

### 1. MRS core 与仓库 manager 冲突

现象：节点名、服务或控制输出互相覆盖，无法确定谁拥有 OFFBOARD 控制权。

原因：两套系统共用 `/uav1/control_manager` 命名空间，且都会接管 PX4。

处理：只借用 `mrs_uav_gazebo_simulation simulation.launch` 和 `/mrs_drone_spawner/spawn` 启动 Gazebo/PX4/MAVROS，再单独启动 `sead_xd_control.launch`。

### 2. x500 不受控爬升

现象：起飞到 2 m 后高度持续增加，即使 controller 输出最大下降加速度也无法下降。

原因：共享 `xd_uav_controller/config/multirotor.yaml` 的 `hover_throttle=0.7`，而当前 Gazebo x500 的实测悬停油门约为 0.5。

处理：不改共享 controller 配置，由 `xd_uav_sead/config/xd_control_x500.yaml` 为该仿真机型覆盖 `hover_throttle: 0.5`。该值不能直接套用于真机或其他机型。

### 3. 局部航点被转成数十万米

现象：输入 `(2, 0, 0)` 后，控制参考变成类似 `(642860, 4672856, ...)`，无人机高速飞离。

原因：原 SEAD 先用全局 origin 转 ECEF，再使用 MAVROS home 转本地 ENU。MRS MAVROS 配置屏蔽了 `home_position`，`home=[0,0,0]` 被当成真实原点。

处理：

- manager 后端明确把 waypoint 解释为 `uav1/odom` 中的本地 ENU 米制坐标。
- manager 后端的本地位置直接使用 MAVROS local odometry，不依赖 home。
- 旧 `direct_mavros` 路径保留原语义，但 home 无效时拒绝航点，不发送猜测坐标。

### 4. x500 落地后 manager 不自动上锁（已复测通过）

现象：无人机已落地，但 manager 仍停留在 `LANDING`，PX4 仍为 armed/OFFBOARD。

原因：x500 落地时 FCU 原点高度实测约 0.23–0.34 m，共享默认触地高度容差为 0.10 m，`landing_touchdown` 永远不会成立。

处理：x500 仿真覆盖设为 `touchdown_height_tolerance: 0.40`。2026-08-01 使用 0.7 m 低高度起飞完成复测：实际稳定高度约 0.77 m，manager 降落后确认触地、输出零推力，并在 PX4 最终报告 Landing detected 后完成上锁；manager 回到 `STANDBY`，MAVROS `armed: False`，最终本地高度约 -0.005 m。因此该阈值在当前 MRS Gazebo x500 配置下已验证有效。

本次仍观察到预期兜底路径：第一次普通上锁请求被 PX4 以尚未 landed 拒绝，manager 随后仅在已确认触地和持续零推力状态请求强制上锁。触地后 `mavros/position_z`、`velocity_z` 曾因新息超限隔离；速度修正先恢复，高度修正约十余秒后恢复，未导致空中提前上锁、manager `FAILSAFE` 或控制中断。

### 5. tmux 关闭后 PX4/MAVROS 仍残留

现象：再次启动时端口或 PX4 instance 冲突；`tmux kill-session` 之后仍能看到 PX4/MAVROS 进程。

原因：MRS spawner 派生进程不一定随 tmux pane 退出。

处理：每次启动前和关闭后都只读检查进程；仅终止本次测试已确认 PID，不使用宽泛 `pkill`、递归 kill 或未解析的通配符。

### 6. GeographicLib 导致限定构建失败

现象：CMake 找不到 GeographicLib module，但机器已有头文件和库。

处理：不安装新依赖，构建时指定现有 module 路径：

```bash
cd /home/promise/catkin_ws
catkin_make --pkg xd_uav_controller xd_uav_control_manager \
  xd_uav_single_tf_manager xd_uav_state_estimators xd_uav_sead \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_MODULE_PATH=/usr/share/cmake/geographiclib
```

### 7. 当前不阻塞的联调警告

以下警告已在阶段 3 人工飞行中观察到。它们来自估计器、manager 或 PX4 仿真适配，不是 SEAD 任务逻辑直接报错。若后续不影响飞行和任务效果，统一留到全部阶段完成后总联调处理。

1. `uav1/local_origin` TF 不存在：估计器默认尝试将主 odometry 额外重发布到 `uav1/local_origin`，当前控制链使用 `uav1/odom`，不依赖该辅助输出。当前影响为日志噪声和该辅助话题无效。
2. PX4 未识别落地后强制上锁：这是 manager 在 controller 已确认触地、持续零推力，但 PX4 `extended_state` 未报告落地时的安全兜底。只要它仅在实际落地后出现，且最终 `armed: False`，当前接受。
3. `mavros/position_z` 高度新息超限：表示预测高度与 MAVROS 高度测量连续偏差过大，高度修正会被短暂隔离并等待恢复。如果只在触地/上锁前后偶发，留待总联调；如果平稳悬停或水平飞行时重复出现，则不再延后。

需要提前升级处理的条件：

- 正常飞行中 `state_valid` 或 `localization_valid` 持续变为 `False`。
- manager 进入 `FAILSAFE`，或无人机出现明显高度跳变、失控或任务中断。
- 未真正触地时就触发零推力或强制上锁。
- 警告频率持续增加，或隔离后没有恢复记录。

## 人工可控验证流程

### 0. 前置检查

新开终端，以下每个终端都先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/catkin_ws/devel/setup.bash
```

确认没有旧的仿真与 manager：

```bash
tmux list-sessions
rosnode list
ps -eo pid,ppid,stat,cmd | grep -E 'gzserver|px4|mavros_node'
```

不要在不知道进程归属时直接 kill。

### 1. 启动 ROS master

终端 A：

```bash
roscore
```

### 2. 只启动 MRS Gazebo 底座

终端 B：

```bash
roslaunch mrs_uav_gazebo_simulation simulation.launch \
  world_name:=grass_plane gui:=true
```

终端 C，等 Gazebo 完全起来后：

```bash
rosservice call /mrs_drone_spawner/spawn "1 --x500"
```

注意：这里不运行 `one_drone/start.sh`，因为它还会启动 MRS core、MRS manager 和自动起飞。

检查 PX4/MAVROS：

```bash
rostopic echo -n 1 /uav1/mavros/state
```

正常前置是 `connected: True`、`armed: False`、`mode: MANUAL`。

### 3. 启动仓库控制链和 SEAD

终端 D：

```bash
UAV_NAME=uav1 roslaunch xd_uav_sead sead_xd_control.launch
```

启动后先不要立即起飞，等待：

```bash
rostopic echo -n 1 /uav1/state_estimator/state_valid
rostopic echo -n 1 /uav1/state_estimator/localization_valid
rostopic echo -n 1 /uav1/control_manager/status
```

前两项应为 `data: True`，manager 应处于可接受起飞的待机状态。

### 4. 人工下发起飞、航点和降落

终端 E，低高度起飞：

```bash
rosrun xd_uav_sead mock_gcs.py \
  _uav_name:=uav1 _cmd:=takeoff _alt:=1.0
```

`mock_gcs.py` 的 takeoff 历史默认是 120 m，仿真验证必须显式给 `_alt:=1.0`。也可绕过 GCS 协议直接测 manager：

```bash
rosservice call /uav1/control_manager/takeoff "altitude: 1.0"
```

检查已解锁且进入 OFFBOARD：

```bash
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav1/mavros/local_position/odom
```

下发 `uav1/odom` 本地 ENU 航点：

```bash
rosrun xd_uav_sead mock_gcs.py \
  _uav_name:=uav1 _cmd:=waypoint _x:=2.0 _y:=0.0 _z:=1.0
```

先检查参考值，再观察无人机：

```bash
rostopic echo -n 1 /uav1/control/reference/setpoint
rostopic echo -n 1 /uav1/mavros/local_position/odom
```

前者应显示 `frame_id: uav1/odom` 和接近 `(2,0,1)` 的 position。如果出现百米、千米或更大数值，不要继续任务，立即降落。

通过 SEAD 模式命令降落：

```bash
rosrun xd_uav_sead mock_gcs.py \
  _uav_name:=uav1 _cmd:=mode _mode:=LAND
```

或直接测 manager：

```bash
rosservice call /uav1/control_manager/land
```

观察：

```bash
rostopic echo /uav1/control_manager/status
rostopic echo /uav1/mavros/state
```

验收目标是 manager 离开 `LANDING`、PX4 最终 `armed: False`。该目标已于 2026-08-01 达成；以后仅在控制参数、x500 模型或 PX4/MAVROS 配置变化后需要重新验证。

### 5. 异常时的安全收尾

首选 manager 降落：

```bash
rosservice call /uav1/control_manager/land
```

如果无人机已经贴地但 manager 无法完成上锁，才使用 PX4 降落模式收尾：

```bash
rosservice call /uav1/mavros/set_mode \
  "base_mode: 0
custom_mode: 'AUTO.LAND'"
```

等 `/uav1/mavros/state` 显示 `armed: False` 后，再按 Ctrl-C 关闭终端 D、B 和 A。最后检查：

```bash
ps -eo pid,ppid,stat,cmd | grep -E 'gzserver|px4|mavros_node'
```

如仍有进程，先确认 PID 属于本次仿真，再单独终止。

## 可调整的操作空间

- 起飞高度：`_alt:=0.7` 或 `1.0`，首次复测不建议超过 2 m。
- 航点：`_x`/​`_y`/​`_z` 均是本地 ENU 米；`z <= 0` 会保持当前高度。
- 航点半径：`_radius`，当前单点 guide 路径主要使用位置参考，不应把 radius 当成 controller 精度参数。
- 控制参数：仅对 MRS Gazebo x500 调整 `xd_control_x500.yaml`。每次只改一项，记录起飞高度、稳态误差、下降速度和触地状态后再决定。
- 只测 controller/manager 时可直接调服务；验证 SEAD 协议链时使用 `mock_gcs.py`。
- 不要用 `mock_gcs.py _cmd:=arm`单独解锁 manager 后端；该后端的安全流程是调用 takeoff，由 manager 负责 OFFBOARD 和解锁。

## 三机 Formation 低空复现（2026-08-01 当前状态）

本节是本轮实际使用的复现步骤。它已验证三机起飞、共享坐标换算、TRAIL→VEE 状态转换和低空高度限制，但三机物理队形尚未通过验收：第二轮中 `uav2` 独立进入 `AUTO.LAND` 并解除武装。因此在解决该控制失效前，不得把本节称为成功演示，也不要继续叠加 SimpleStrike/GA 实飞。

所有终端先设置一致的本机 ROS 地址，避免 roscore 发布不可解析的主机名：

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```

1. 启动 `roscore`，再启动带可见窗口的 Gazebo 底座并生成三机：

```bash
roscore
roslaunch mrs_uav_gazebo_simulation simulation.launch gui:=true
rosservice call /mrs_drone_spawner/spawn '1 2 3 --x500'
```

2. 分别读取 `/gazebo/model_states` 和三个 `/uavX/mavros/local_position/odom`。对每架飞机计算：

```text
shared_frame_offset = Gazebo 世界位置 - MAVROS 本地位置
```

每次重新生成模型都必须重算，不能复制旧数值。随后为 `uav1`、`uav2`、`uav3` 各开一个终端，替换对应偏移：

```bash
roslaunch xd_uav_sead sead_xd_control.launch \
  UAV_NAME:=uav1 \
  config:=/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/config/sead_x500_three_uav.yaml \
  shared_frame_enabled:=true \
  shared_frame_offset_x:=<OFFSET_X> \
  shared_frame_offset_y:=<OFFSET_Y> \
  shared_frame_offset_z:=<OFFSET_Z>
```

3. 三机的 `state_valid`、`localization_valid` 都为 `True` 后再起飞：

```bash
rosservice call /uav1/control_manager/takeoff '{}'
rosservice call /uav2/control_manager/takeoff '{}'
rosservice call /uav3/control_manager/takeoff '{}'
```

确认三机均为 `armed: True`、`mode: OFFBOARD`。然后逐机发送相同的 TRAIL 配置；下面命令中的 `X` 依次替换为 1、2、3：

```bash
rostopic pub -1 /uavX/sead/command std_msgs/String \
  '{data: '\''{"msg_id":24,"info":{"enable":1,"shape":"TRAIL","leader_id":1,"spacing":4.0,"standoff":12.0,"safe_sep":2.0,"alt_step":0.5}}'\''}'
```

再逐机发送相同的低空集结点（点位应按当次出生/落地点选择，避免长距离飞行）：

```bash
rostopic pub -1 /uavX/sead/command std_msgs/String \
  '{data: '\''{"msg_id":26,"info":{"point_id":1,"point":[35.0,6.0,3.5],"loiter_radius":8.0}}'\''}'
```

4. 验收必须连续观察，而不是截取瞬时位置：

- 三机始终 `armed: True`、`mode: OFFBOARD`，且 manager 的 `state_valid/localization_valid` 不失效。
- JSONL 日志中三机槽位稳定为一组 `0/1/2`，出现 `formation_shape_switch`，最终 `formation_shape` 为 `VEE`。
- `formation/static_hold: true` 时进入 HOLD 后，各机 waypoint 不再随时间绕集结点旋转。
- 使用“本地 odom + 各机 shared offset”换算到同一坐标系，三机应连续保持 VEE 槽位，而非仅一瞬间接近。

本轮实际结果：22 项离线回归测试通过；三机低空限制有效，未再爬升到原固定翼默认 80 m；TRAIL→VEE 状态转换和槽位 `uav1=1/uav2=0/uav3=2` 可见；静态 HOLD 修复后 1、3 号不再生成盘旋目标，但 2 号机进入 `AUTO.LAND`，所以物理三机验收失败。下一次只需重现并定位 `uav2` 的 estimator/controller 失效，不重复单机起降和协议往返检查。

5. 无论成功或失败都先降落，再关闭控制链和 Gazebo：

```bash
rosservice call /uav1/control_manager/land '{}'
rosservice call /uav2/control_manager/land '{}'
rosservice call /uav3/control_manager/land '{}'
```

只有确认三机均 `armed: False` 后才关闭进程；manager 拒绝降落且飞机仍在空中时，使用对应 `/uavX/mavros/set_mode` 切换 `AUTO.LAND` 作为兜底。

### 本轮失败归因与下一次采证要求

当前不能把失败简单归因于电脑性能，也不能只归因于 SEAD：

- 第二次复飞中，`uav2/control_manager` 在仿真时刻约 `457.116` 进入 `ACTIVE`，约 10 秒后报告 `FAILSAFE: 飞行期间意外上锁`；同一轮 `uav1/uav3` 仍为 armed/OFFBOARD。这更像 `uav2` 的 PX4/起飞/落地状态或控制链单机异常，而不是三机同时因整机卡顿丢失控制。
- 更早一轮 `uav2/control_manager` 报告过 `输入持续失效: 估计器报告state_valid=false`；后续 rosout 还记录 `uav2/state_estimator` 的 `mavros/position_xy` 水平位置新息超限、拒绝并隔离。该证据指向 `xd_uav_state_estimators` 与 MAVROS odometry 的状态一致性问题，但尚不足以断定 estimator 本身有 bug：异常机动、落地点改变、时间跳变或仿真卡顿也可能先造成大新息。
- 三机都反复出现缺失 `uavX/local_origin` TF 的辅助重发布警告。当前主控制使用 `uavX/odom`，所以它不是已确认根因，但属于其他包/launch 的配置缺口，应记录而不是在 SEAD 中伪造 TF 或用常数掩盖。
- 本轮没有同步记录 Gazebo real-time factor、CPU/内存、`/clock` 连续性和各控制话题实测频率，因此“电脑性能不足”只能列为待验证假设。

下一次先做单独 `uav2` 的最小复现，不立即发送 Formation。起飞前后连续记录以下证据：

```bash
rostopic hz /clock
rostopic hz /uav2/mavros/local_position/odom
rostopic hz /uav2/state_estimator/main/odom
rostopic hz /uav2/mavros/setpoint_raw/attitude
rostopic echo /uav2/control_manager/status
rostopic echo /uav2/state_estimator/diagnostics
rostopic echo /uav2/mavros/extended_state
```

同时记录系统负载和 Gazebo real-time factor。若单独 `uav2` 也在约 10 秒后意外上锁，优先排查 PX4 landed 判定、第二次起飞时的本地高度/home 语义及 manager 状态机；若单机稳定而三机才失败，再比较 real-time factor、消息频率与 estimator 新息。未经证据不得通过放宽 estimator 阈值、延长 manager timeout、写死偏移/高度或禁用 failsafe 来“修好”演示。

### 配置参数与魔法数字边界

- 本轮出生点共享坐标偏移只作为 launch 参数传入，没有写死进源码；每次 spawn 必须重新测量。
- `minimum_altitude: 1.0`、`static_hold: true`、spacing/standoff 等位于专用 x500 YAML，并有字段名和使用范围，不应复制到真机或原固定翼配置。
- 新增 `static_hold` 默认 `false`，只在 x500 profile 开启，避免改写原固定翼盘旋语义。
- Formation 中仍存在原始固定翼算法遗留的常数，例如 TRAIL→VEE 的 `600.0` 切换距离以及过渡阶段的最小前进距离。它们不应继续通过新增硬编码适配多旋翼；下一步应提炼为带单位、默认值和机型 profile 的配置项，或设计明确的 vehicle profile。
- ROS bridge 的 shape 数值是线协议枚举，不是可任意调参的飞行常数；但当前在两个模块重复定义，后续应收敛到单一协议定义，避免再次出现 TRAIL 编码不一致。

## 快速诊断命令

```bash
rosnode list | grep -E 'state_estimator|control_manager|controller|sead_onboard'
rostopic info /uav1/mavros/setpoint_raw/local
rostopic info /uav1/mavros/setpoint_raw/attitude
rostopic echo -n 1 /uav1/state_estimator/diagnostics
rostopic echo -n 1 /uav1/control_manager/diagnostics
rostopic echo -n 1 /uav1/controller/command
```

正常 manager 后端下，SEAD 不应成为 `/uav1/mavros/setpoint_raw/local` 的 publisher；仓库 controller 应是 attitude setpoint 的唯一业务控制输出。
