# SEAD 全功能验证手册：三个终端、逐项独立执行

## 当前实测状态与下一步（2026-08-01 交接）

### 已确认

- V0–V2 的构建、launch、协议和核心算法回归已有有效通过记录；相关代码未变化时不重复。
- V3 单机起飞、航点、降落和自动上锁已经通过。
- 独立 uav2 在 OFFBOARD/armed 下稳定约 67.8 仿真秒；estimator 两个 valid 全程为 true，manager 无 FAILSAFE，单机没有复现意外上锁。
- 简化启动器已经实际调试，不只是静态编写：
  - `start.sh v4 --no-attach` 能从干净环境自动打开 Gazebo、生成 uav1/uav2、启动两套控制链；两机 MAVROS connected、两个 estimator valid 均为 true。未起飞。
  - `start.sh v5 --no-attach` 能生成三机，等待完整 Gazebo model 与三路 MAVROS odom，自动计算当次 offset，并启动三套控制链；三机 MAVROS connected、两个 estimator valid 均为 true。未起飞。
  - `kill.sh` 已实测能停止专用 tmux session，并清理本验证链派生的 PX4/MAVROS/Gazebo；最终残留检查通过。
- 调试中确认旧 PX4/MAVROS 残留会导致新 PX4 实例/端口冲突：表现为 odom 话题存在但无消息、MAVROS `connected=False`。`process_guard.sh` 现会在启动前阻止叠加。
- V5 offset 文件包含本轮唯一 run ID；旧计算进程或旧文件不能再触发新控制链。

### 尚未通过

- V4 只完成“启动与起飞前状态”验证，尚未执行两机起飞、60 秒保持、降落和同步采证。
- V5 只完成“仿真底座、自动 offset 与三套控制链”验证，尚未执行起飞、TRAIL、集结点、TRAIL→VEE 物理转换、连续队形保持和安全降落。
- V6 Airspace ROS 节点链、V7 DPGA 真实多节点分配、V8 SimpleStrike 真实多节点协商尚待按本文执行。
- DPGA 物理任务飞行、SimpleStrike 物理同步进场、固定翼实飞和真实 XBee 仍没有端到端证据。
- V10 QGC/UDP 属于阶段 4，当前不进入。

### 下一会话唯一首要任务

先执行 V4 两机并发稳定性，不直接进入 V5 飞行：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/tmux/validation
./start.sh v4
```

等待 `status` 窗口中 uav1/uav2 均为 `connected: True`、`state_valid: True`、`localization_valid: True`，再在 `commands` 窗口执行：

```bash
takeoff_all
```

保持约 60 秒并同步采集 RTF、系统负载、`/clock`、两路 odom、estimator 输出、attitude setpoint、extended_state 和 manager diagnostics；然后：

```bash
land_all
```

只有 V4 全程没有意外上锁、valid 失效或 FAILSAFE，且两机安全上锁，才进入 V5 物理 Formation。

## 0. 推荐方式：一条命令启动可见 tmux 工作区

手工三终端步骤保留在本文后半部分，便于审计和故障恢复。日常验证优先使用新增启动器：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/tmux/validation
./start.sh v4
```

把 `v4` 换成所需项目：

| 参数 | 内容 |
|---|---|
| `v3` | 单机起飞、航点、降落底座 |
| `v4` | 两机并发稳定性，当前下一项 |
| `v5` | 三机 Formation |
| `v6` | Airspace 无飞行节点验证 |
| `v7` | DPGA 无飞行多节点分配 |
| `v8` | SimpleStrike 无飞行多节点协商 |

启动器创建一个 `sead-validation` tmux session，其中只有三个窗口：

- `launch`：平铺显示 roscore、Gazebo、spawner、uav1、uav2、uav3；不适用的 pane 会明确显示“本场景不使用”。
- `status`：每两秒刷新 MAVROS、estimator 和 manager 摘要。
- `commands`：显示可输入的短命令菜单。

tmux 操作：

```text
Ctrl+B，再按 n       下一个窗口
Ctrl+B，再按 p       上一个窗口
Ctrl+B，再按 0/1/2   跳到 launch/status/commands
Ctrl+B，再按 d       退出界面但保持后台运行
```

重新进入：

```bash
tmux -L sead-validation attach -t sead-validation
```

`v3/v4/v5` 会启动 `simulation.launch gui:=true`，因此 Gazebo GUI 会独立弹出；`v6/v7/v8` 是无飞行验证，按设计不打开 Gazebo。

三机 PX4/MAVROS 是异步生成的，首次启动通常需要约 30–90 秒。不要因为某个 pane 暂时显示等待就重复运行 `start.sh` 或 `auto_offsets`；以 `status` 窗口最终出现 `connected: True` 和两个 valid 为 true 为准。

### commands 窗口的短命令

进入 `commands` 窗口后，不再复制长 service 或 JSON：

```bash
takeoff_all
land_all
```

V5 会在 spawn 三机后自动等待三路 MAVROS odom、计算本轮 offset 并启动三套控制链。等 `status` 窗口显示三机 valid 后，依次使用：

```bash
takeoff_all
trail_all
formation_point 35 6
land_all
```

如果 `launch` 窗口明确显示自动偏移失败，修复所报告的 PX4/MAVROS 问题后，才使用 `auto_offsets` 手动重试；不要在等待期间反复输入。偏移文件带有本轮唯一 run ID，旧文件不会触发新控制链。`formation_point 35 6` 中的 `35 6` 仍应根据当次出生位置选择，不能机械复用。

V6：

```bash
airspace_demo
```

V7：

```bash
dpga_demo
```

V8：

```bash
strike_demo
```

随时查看菜单：

```bash
help_sead
```

结束飞行项目时必须先在 `commands` 窗口执行 `land_all` 并确认全部上锁。然后按 `Ctrl+B`、`d` 退出 tmux，在普通终端执行：

```bash
./kill.sh
```

`start.sh` 会在启动前检查会占用本验证端口的 PX4/MAVROS/Gazebo 残留，发现残留就拒绝启动。`kill.sh` 停止专用 session 后，只按明确的 MRS x500 仿真进程特征清理派生 PX4/MAVROS/Gazebo，并再次核对；它不会执行宽范围 `pkill`。

## 1. 这份手册怎么使用

每个验证项都按三个普通终端编写，不要求额外打开窗口：

- **终端 1——启动**：用命名 tmux 会话启动 roscore、Gazebo 和各 UAV launch。
- **终端 2——状态**：查看 tmux 日志、ROS 状态、Gazebo 性能并记录 rosbag。
- **终端 3——指令**：spawn UAV、调用起降服务、发送 SEAD 命令。

`tmux new-session -d` 成功时不会在当前终端显示 launch 日志，这是正常行为。使用下面的方法查看：

```bash
tmux list-sessions
tmux capture-pane -pt <会话名> -S -100
```

Gazebo GUI 只由下面这个 launch 启动：

```bash
roslaunch mrs_uav_gazebo_simulation simulation.launch gui:=true
```

`sead_xd_control.launch` 不启动 Gazebo，它只启动指定 UAV 的 estimator、manager、controller 和 SEAD 节点。

## 2. 三个终端的统一初始化

每次新开终端，先复制执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export DEBUG=false
```

开始任何项目之前，在终端 2 检查旧进程：

```bash
tmux list-sessions
pgrep -af 'roscore|rosmaster|roslaunch|gzserver|gzclient|px4|mavros|sead_onboard|control_manager_node|controller_node|state_estimator'
```

如果发现无法确认归属的旧进程，不要叠加启动。

## 3. 通用安全规则

- 不启动 MRS core；这里只使用 MRS Gazebo spawner 作为 PX4+MAVROS 底座。
- 起飞前必须确认 `state_valid=True`、`localization_valid=True`、MAVROS `connected=True`。
- 任一 UAV 空中意外上锁、离开 OFFBOARD、estimator 失效、manager FAILSAFE、翻转或失控时，停止发送任务并立即降落。
- 不放宽 estimator 阈值、manager timeout 或 failsafe，不在 SEAD 内写死坐标掩盖问题。
- 每个飞行项目结束后必须确认所有 UAV `armed: False`，再关闭进程。

---

## V0：构建与 launch 静态检查

此项不启动 Gazebo，不飞行。

### 终端 1：构建

仅在相关代码变化后执行：

```bash
cd /home/promise/catkin_ws
catkin build xd_uav_msgs xd_uav_controller xd_uav_state_estimators xd_uav_control_manager xd_uav_sead
```

### 终端 2：检查 launch

```bash
roslaunch --nodes xd_uav_sead sead_onboard.launch
roslaunch --nodes xd_uav_sead sead_xd_control.launch
```

### 终端 3

本项不需要发送指令。

### 验收与结束

- 构建成功；
- 两个 launch 能列出节点；
- 不出现缺包、XML 或参数错误；
- 命令执行完即结束，无后台进程需要清理。

---

## V1：协议、ROS bridge 与 U2U 回归

此项不启动 Gazebo，不飞行。代码未变化时不重复运行。

### 终端 1：运行协议测试

```bash
cd /home/promise/catkin_ws
PYTHONDONTWRITEBYTECODE=1 nosetests3 -v src/xd-uavsystem-test/src/xd_uav_sead/test/test_rosbridge_protocol.py
```

### 终端 2

查看终端 1 的输出；所有用例应为 `ok`，最终失败数为 0。

### 终端 3

本项不需要发送指令。

### 验收与结束

基础命令、Swarm、Airspace、SEAD mission、Task Insert 和 U2U 路由测试全部通过。测试结束即完成，无后台进程。

---

## V2：Airspace、PathFollowing、GA/DPGA、Formation、SimpleStrike 算法回归

此项不启动 Gazebo，不飞行。它验证算法效果，不代表多机飞行通过。

### 终端 1：运行核心测试

```bash
cd /home/promise/catkin_ws
PYTHONDONTWRITEBYTECODE=1 nosetests3 -v src/xd-uavsystem-test/src/xd_uav_sead/test/test_core_functions.py
```

### 终端 2

查看终端 1 输出，确认 Airspace、Dubins 绕障、GA、DPGA、Formation 和 SimpleStrike 相关项均为 `ok`。

### 终端 3

本项不需要发送指令。

### 验收与结束

所有测试通过且无残留子进程。代码未变化时不重复运行。

---

## V3：单机起飞、航点、降落

此项已经通过；只有控制链、配置或环境变化时才复测。

### 终端 1：启动 roscore、Gazebo 和 uav1 控制链

```bash
tmux new-session -d -s sead-val-ros 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roscore'
tmux new-session -d -s sead-val-sim 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch mrs_uav_gazebo_simulation simulation.launch gui:=true'
```

Gazebo 窗口出现、终端 2 确认 spawner 就绪后，由终端 3 spawn：

```bash
rosservice call /mrs_drone_spawner/spawn '1 --x500'
```

然后回到终端 1 启动控制链：

```bash
tmux new-session -d -s sead-val-uav1 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:=uav1 config:=/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/config/sead_x500_three_uav.yaml shared_frame_enabled:=false'
```

### 终端 2：状态和日志

```bash
rosservice list | grep /mrs_drone_spawner/spawn
tmux capture-pane -pt sead-val-sim -S -60
tmux capture-pane -pt sead-val-uav1 -S -100
rostopic echo -n 1 /uav1/state_estimator/state_valid
rostopic echo -n 1 /uav1/state_estimator/localization_valid
rostopic echo -n 1 /uav1/mavros/state
```

只有两个 valid 为 true 且 MAVROS connected 时才能继续。

### 终端 3：指令

每条执行后等待动作稳定，再执行下一条：

```bash
rosservice call /uav1/control_manager/takeoff '{}'
```

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=waypoint _x:=2.0 _y:=0.0 _z:=1.0 _radius:=1
```

```bash
rosservice call /uav1/control_manager/land '{}'
```

### 验收与结束

验收：起飞稳定、飞向约 `(2,0,1)`、受控降落、最终 `armed: False`、manager 回到 STANDBY。

终端 1 清理：

```bash
tmux kill-session -t sead-val-uav1
tmux kill-session -t sead-val-sim
tmux kill-session -t sead-val-ros
```

终端 2 重新执行旧进程检查，确认无残留。

---

## V4：两机并发稳定性

这是当前应该执行的下一项。不发送 Formation 或航点。

### 终端 1：启动 roscore 和 Gazebo

```bash
tmux new-session -d -s sead-val-ros 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roscore'
tmux new-session -d -s sead-val-sim 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch mrs_uav_gazebo_simulation simulation.launch gui:=true'
```

### 终端 2：确认 Gazebo GUI 和 spawner

Gazebo 窗口应由 `sead-val-sim` 会话打开。检查：

```bash
rosservice list | grep /mrs_drone_spawner/spawn
tmux capture-pane -pt sead-val-sim -S -80
```

如果没有 Gazebo 窗口，再检查：

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
tmux show-environment -g | grep -E 'DISPLAY|WAYLAND_DISPLAY'
```

### 终端 3：生成两机

```bash
rosservice call /mrs_drone_spawner/spawn '1 2 --x500'
```

必须返回 `success: True`。

### 终端 1：启动两套控制链

下面两条均可直接复制：

```bash
tmux new-session -d -s sead-val-uav1 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:=uav1 config:=/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/config/sead_x500_three_uav.yaml shared_frame_enabled:=false'
```

```bash
tmux new-session -d -s sead-val-uav2 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:=uav2 config:=/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/config/sead_x500_three_uav.yaml shared_frame_enabled:=false'
```

### 终端 2：检查、查看日志并启动采证

```bash
tmux capture-pane -pt sead-val-uav1 -S -100
tmux capture-pane -pt sead-val-uav2 -S -100
rostopic echo -n 1 /uav1/state_estimator/state_valid
rostopic echo -n 1 /uav2/state_estimator/state_valid
rostopic echo -n 1 /uav1/state_estimator/localization_valid
rostopic echo -n 1 /uav2/state_estimator/localization_valid
```

四项均为 true 后执行：

```bash
tmux new-session -d -s sead-val-bag 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec rosbag record -O /tmp/sead_v4_two_uav.bag /clock /gazebo/performance_metrics /uav1/mavros/state /uav2/mavros/state /uav1/mavros/extended_state /uav2/mavros/extended_state /uav1/mavros/local_position/odom /uav2/mavros/local_position/odom /uav1/state_estimator/main/odom /uav2/state_estimator/main/odom /uav1/state_estimator/state_valid /uav2/state_estimator/state_valid /uav1/state_estimator/diagnostics /uav2/state_estimator/diagnostics /uav1/control_manager/status /uav2/control_manager/status /uav1/control_manager/diagnostics /uav2/control_manager/diagnostics /uav1/mavros/setpoint_raw/attitude /uav2/mavros/setpoint_raw/attitude'
```

### 终端 3：起飞、等待、降落

```bash
rosservice call /uav1/control_manager/takeoff '{}'
rosservice call /uav2/control_manager/takeoff '{}'
```

观察约 60 秒。终端 2 检查两机均 armed/OFFBOARD、manager ACTIVE、estimator valid。然后终端 3 执行：

```bash
rosservice call /uav1/control_manager/land '{}'
rosservice call /uav2/control_manager/land '{}'
```

### 验收与结束

两机最终都必须 `armed: False`。终端 2停止并检查 bag：

```bash
tmux send-keys -t sead-val-bag C-c
rosbag info /tmp/sead_v4_two_uav.bag
```

终端 1 清理：

```bash
tmux kill-session -t sead-val-bag 2>/dev/null || true
tmux kill-session -t sead-val-uav1
tmux kill-session -t sead-val-uav2
tmux kill-session -t sead-val-sim
tmux kill-session -t sead-val-ros
```

终端 2 核对无残留。

---

## V5：三机 Formation

只有 V4 通过后执行。V5 必须使用当次 spawn 后测得的 shared-frame offset。

### 终端 1：启动 roscore 和 Gazebo

```bash
tmux new-session -d -s sead-val-ros 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roscore'
tmux new-session -d -s sead-val-sim 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch mrs_uav_gazebo_simulation simulation.launch gui:=true'
```

### 终端 2：确认 spawner

```bash
rosservice list | grep /mrs_drone_spawner/spawn
```

### 终端 3：生成三机

```bash
rosservice call /mrs_drone_spawner/spawn '1 2 3 --x500'
```

### 终端 2：取得当次出生点数据

```bash
rostopic echo -n 1 /gazebo/model_states
rostopic echo -n 1 /uav1/mavros/local_position/odom
rostopic echo -n 1 /uav2/mavros/local_position/odom
rostopic echo -n 1 /uav3/mavros/local_position/odom
```

对每架机计算：

```text
offset_x = Gazebo世界x - MAVROS本地x
offset_y = Gazebo世界y - MAVROS本地y
offset_z = Gazebo世界z - MAVROS本地z
```

> 注意：下面三条命令是本手册唯一不能原样复制的命令。必须先把 `U1X`、`U1Y`、`U1Z` 等文字替换成你刚计算出的纯数字；不要保留尖括号或字母。

### 终端 1：启动三套控制链

uav1：

```bash
tmux new-session -d -s sead-val-uav1 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:=uav1 config:=/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/config/sead_x500_three_uav.yaml shared_frame_enabled:=true shared_frame_offset_x:=U1X shared_frame_offset_y:=U1Y shared_frame_offset_z:=U1Z'
```

uav2：

```bash
tmux new-session -d -s sead-val-uav2 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:=uav2 config:=/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/config/sead_x500_three_uav.yaml shared_frame_enabled:=true shared_frame_offset_x:=U2X shared_frame_offset_y:=U2Y shared_frame_offset_z:=U2Z'
```

uav3：

```bash
tmux new-session -d -s sead-val-uav3 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 DEBUG=false; exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:=uav3 config:=/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/config/sead_x500_three_uav.yaml shared_frame_enabled:=true shared_frame_offset_x:=U3X shared_frame_offset_y:=U3Y shared_frame_offset_z:=U3Z'
```

### 终端 2：检查三机

```bash
tmux capture-pane -pt sead-val-uav1 -S -80
tmux capture-pane -pt sead-val-uav2 -S -80
tmux capture-pane -pt sead-val-uav3 -S -80
rostopic echo -n 1 /uav1/state_estimator/state_valid
rostopic echo -n 1 /uav2/state_estimator/state_valid
rostopic echo -n 1 /uav3/state_estimator/state_valid
```

三机 valid 后再继续。

### 终端 3：起飞

```bash
rosservice call /uav1/control_manager/takeoff '{}'
rosservice call /uav2/control_manager/takeoff '{}'
rosservice call /uav3/control_manager/takeoff '{}'
```

### 终端 2：确认三机均 armed/OFFBOARD

```bash
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav2/mavros/state
rostopic echo -n 1 /uav3/mavros/state
```

### 终端 3：逐机发送 TRAIL 配置

下面是三条独立命令，不使用 `for` 循环：

```bash
rostopic pub -1 /uav1/sead/command std_msgs/String '{data: '\''{"msg_id":24,"info":{"enable":1,"shape":"TRAIL","leader_id":1,"spacing":4.0,"standoff":12.0,"safe_sep":2.0,"alt_step":0.5}}'\''}'
```

```bash
rostopic pub -1 /uav2/sead/command std_msgs/String '{data: '\''{"msg_id":24,"info":{"enable":1,"shape":"TRAIL","leader_id":1,"spacing":4.0,"standoff":12.0,"safe_sep":2.0,"alt_step":0.5}}'\''}'
```

```bash
rostopic pub -1 /uav3/sead/command std_msgs/String '{data: '\''{"msg_id":24,"info":{"enable":1,"shape":"TRAIL","leader_id":1,"spacing":4.0,"standoff":12.0,"safe_sep":2.0,"alt_step":0.5}}'\''}'
```

集结点必须选择在当次三机附近。将 `GX`、`GY` 替换成纯数字后，逐条发送：

```bash
rostopic pub -1 /uav1/sead/command std_msgs/String '{data: '\''{"msg_id":26,"info":{"point_id":1,"point":[GX,GY,3.5],"loiter_radius":8.0}}'\''}'
```

```bash
rostopic pub -1 /uav2/sead/command std_msgs/String '{data: '\''{"msg_id":26,"info":{"point_id":1,"point":[GX,GY,3.5],"loiter_radius":8.0}}'\''}'
```

```bash
rostopic pub -1 /uav3/sead/command std_msgs/String '{data: '\''{"msg_id":26,"info":{"point_id":1,"point":[GX,GY,3.5],"loiter_radius":8.0}}'\''}'
```

### 验收与结束

验收：三机持续 armed/OFFBOARD、estimator valid；槽位唯一；出现 TRAIL 到 VEE；共享世界坐标下连续保持 VEE。

终端 3 安全降落：

```bash
rosservice call /uav1/control_manager/land '{}'
rosservice call /uav2/control_manager/land '{}'
rosservice call /uav3/control_manager/land '{}'
```

三机均 `armed: False` 后，终端 1 逐条清理：

```bash
tmux kill-session -t sead-val-uav1
tmux kill-session -t sead-val-uav2
tmux kill-session -t sead-val-uav3
tmux kill-session -t sead-val-sim
tmux kill-session -t sead-val-ros
```

终端 2 核对无残留。

---

## V6：Airspace ROS 下发

首次只做节点与协议验证，不飞行。

### 终端 1：启动 roscore 和节点

```bash
tmux new-session -d -s sead-val-ros 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roscore'
tmux new-session -d -s sead-val-uav1 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:=uav1 use_simulation:=true sead_runtime_mode:=dpga_sead sead_control_mode:=position_waypoint'
```

### 终端 2：查看节点日志

```bash
tmux capture-pane -pt sead-val-uav1 -S -100
rostopic echo /uav1/sead/telemetry
```

`rostopic echo` 会持续占用终端；查看到 ACK 后按 `Ctrl+C` 返回。

### 终端 3：依次发送

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_clear
```

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_zone _zone_id:=1 _minAlt:=0.0 _maxAlt:=20.0 _points_json:='[[5,-2],[9,-2],[9,2],[5,2]]'
```

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_clear
```

### 验收与结束

日志报告 zone 1 已存储，遥测返回成功 ACK，clear 后区域归零。

终端 1 清理：

```bash
tmux kill-session -t sead-val-uav1
tmux kill-session -t sead-val-ros
```

---

## V7：DPGA SEAD 多节点分配

当前先验证无飞行多节点分配。物理飞行必须等待 V5 通过并重新审查任务坐标。

### 终端 1：启动 roscore 和三个节点

```bash
tmux new-session -d -s sead-val-ros 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roscore'
tmux new-session -d -s sead-val-uav1 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:=uav1 use_simulation:=true sead_runtime_mode:=dpga_sead sead_control_mode:=position_waypoint'
tmux new-session -d -s sead-val-uav2 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:=uav2 use_simulation:=true sead_runtime_mode:=dpga_sead sead_control_mode:=position_waypoint'
tmux new-session -d -s sead-val-uav3 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:=uav3 use_simulation:=true sead_runtime_mode:=dpga_sead sead_control_mode:=position_waypoint'
```

### 终端 2：观察

```bash
rostopic echo /sead/u2u
```

需要查看节点日志时按 `Ctrl+C`，然后执行：

```bash
tmux capture-pane -pt sead-val-uav1 -S -120
tmux capture-pane -pt sead-val-uav2 -S -120
tmux capture-pane -pt sead-val-uav3 -S -120
```

### 终端 3：逐机发送同一任务

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=sead_mission _targets_json:='[[20,0]]' _uav_type:=1 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
```

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav2 _cmd:=sead_mission _targets_json:='[[20,0]]' _uav_type:=2 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
```

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav3 _cmd:=sead_mission _targets_json:='[[20,0]]' _uav_type:=3 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
```

动态目标插入：

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=task_insert _x:=25 _y:=5 _task_type:=0
```

### 验收与结束

验收：三机有 U2U 状态交换；分配满足 Recon→类型1、Strike→类型2、BDA→类型3；Task Insert 被接受或排队后回放；进程不异常退出。

终端 1 清理：

```bash
tmux kill-session -t sead-val-uav1
tmux kill-session -t sead-val-uav2
tmux kill-session -t sead-val-uav3
tmux kill-session -t sead-val-ros
```

---

## V8：SimpleStrike 多节点协商

当前先验证无飞行协商。物理同步进场必须等待 V5 通过。

### 终端 1：启动 roscore 和三个节点

```bash
tmux new-session -d -s sead-val-ros 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roscore'
tmux new-session -d -s sead-val-uav1 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:=uav1 use_simulation:=true sead_runtime_mode:=simple_strike simple_strike_control_mode:=position_waypoint'
tmux new-session -d -s sead-val-uav2 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:=uav2 use_simulation:=true sead_runtime_mode:=simple_strike simple_strike_control_mode:=position_waypoint'
tmux new-session -d -s sead-val-uav3 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:=uav3 use_simulation:=true sead_runtime_mode:=simple_strike simple_strike_control_mode:=position_waypoint'
```

### 终端 2：观察

```bash
rostopic echo /sead/u2u
```

### 终端 3：逐机发送相同三个目标

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=sead_mission _targets_json:='[[20,-8],[24,0],[20,8]]' _uav_type:=2 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
```

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav2 _cmd:=sead_mission _targets_json:='[[20,-8],[24,0],[20,8]]' _uav_type:=2 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
```

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav3 _cmd:=sead_mission _targets_json:='[[20,-8],[24,0],[20,8]]' _uav_type:=2 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
```

### 验收与结束

验收：leader 产生一机一目标分配；uav2/uav3 返回 ACK；三机路径状态被聚合；leader 广播共同命中时间。

终端 1 清理：

```bash
tmux kill-session -t sead-val-uav1
tmux kill-session -t sead-val-uav2
tmux kill-session -t sead-val-uav3
tmux kill-session -t sead-val-ros
```

---

## V9：真实 XBee

此项必须具备真实 DigiMesh 设备、GCS 电台和真机安全授权；没有设备时不执行。

### 终端 1：启动硬件节点

只有完成串口、Node ID、64位地址、波特率、设备锁和真机急停检查后，才可执行：

```bash
tmux new-session -d -s sead-val-uav1 'source /opt/ros/noetic/setup.bash; source /home/promise/catkin_ws/devel/setup.bash; export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:=uav1 use_simulation:=false'
```

### 终端 2：状态

```bash
tmux capture-pane -pt sead-val-uav1 -S -150
```

必须看到设备发现和时间同步成功，不能只有节点存活。

### 终端 3：指令

硬件 GCS 指令必须由现场操作方案定义。本手册不在无硬件信息时给出解锁或飞行命令。

### 验收与结束

验收至少包括设备发现、时间同步、G2U、U2G、U2U、断链恢复和安全终止。结束时：

```bash
tmux kill-session -t sead-val-uav1
```

---

## V10：QGC/UDP

V10 属于阶段 4，当前尚未实现，因此没有可执行的启动和验收指令。

### 终端 1

当前不启动 QGC 或 UDP adapter。

### 终端 2

当前没有 UDP→ROS 状态可检查。

### 终端 3

当前不发送 QGC 任务。

### 验收与结束

只有完成版本化 UDP 协议、功能包外 adapter 和 QGC UI 后，才能补充 V10。`mock_gcs.py` 不能当作 QGC 集成成功证据。

---

## 4. 飞行异常时的统一降落

优先通过 manager：

```bash
rosservice call /uav1/control_manager/land '{}'
rosservice call /uav2/control_manager/land '{}'
rosservice call /uav3/control_manager/land '{}'
```

只对实际存在的 UAV 调用。只有某机 manager 拒绝且飞机仍在空中时，才使用对应 MAVROS 兜底，例如 uav2：

```bash
rosservice call /uav2/mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.LAND'"
```

确认全部 `armed: False` 后才能关闭 Gazebo 和 ROS。

## 5. 每项需要记录的结果

- V 编号、日期和 Git 状态；
- 实际执行的终端命令；
- UAV 数量和配置；
- 是否出现 estimator false、FAILSAFE、意外上锁或模式退出；
- Gazebo 中看到的物理效果；
- bag/log 路径；
- 最终是否安全上锁；
- 结论：已验证、部分验证、待验证或阻塞。

不得用测试文件通过替代飞行效果，也不得用瞬时截图替代连续协同效果。
