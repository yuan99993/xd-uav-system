# 飞控后端与 ArduCopter SITL 使用说明

本文说明如何演示 `xd_uav_controller` 与 `xd_uav_control_manager` 新增的
autopilot-neutral 输出合同、PX4/ArduCopter 后端选择，以及已经验证的
ArduCopter 4.7.1 起飞、位置、速度和降落流程。

## 支持范围

- PX4：保持原有 `OFFBOARD`、body-rate + normalized thrust 路径。
- ArduCopter：首阶段只支持 multirotor，使用 `GUIDED`、quaternion attitude +
  raw thrust，并忽略三轴 body-rate。
- ArduPlane、APM VTOL/tiltrotor 尚未验收，不属于当前支持范围。
- 仿真配置固定对应 `Copter-4.7.1`。真机不得直接套用 SITL 的悬停推力、降落高度源或
  强制上锁参数。
- 三维仿真使用本机已有 Gazebo Classic 11，以及工作区级固定插件
  `third_party/ardupilot_gazebo-classic`；不会安装新版 Gazebo 或修改 PX4 仿真环境。

本机插件固定在提交 `51907b9e72513db199a5ac57c99fa449bb5ae670`。若插件目录或构建
产物缺失，可在工作区级依赖目录恢复，不要复制进控制包：

```bash
cd ~/catkin_ws/third_party
git clone https://github.com/SwiftGust/ardupilot_gazebo.git ardupilot_gazebo-classic
cd ardupilot_gazebo-classic
git checkout 51907b9e72513db199a5ac57c99fa449bb5ae670
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --parallel 2
```

该插件对应 Gazebo Classic，不会替换系统 Gazebo。已构建的
`build/libArduPilotPlugin.so`由一键脚本按绝对工作区路径加载。

## 最简启动

脚本会同时启动 Gazebo Classic 11、ArduCopter SITL，以及 MAVROS、定位和控制链。它保持在前台，按
`Ctrl-C`会一起清理所有由它启动的进程：

```bash
~/catkin_ws/src/xd-uavsystem-test/src/xd_uav_control_manager/scripts/arducopter_demo.sh start
```

这个直接入口会自行加载 ROS 和工作区环境，启动终端无需再手工 `source`。
若改过工作区或 ArduPilot 位置，可用 `CATKIN_WS`/`ARDUPILOT_DIR` 覆盖默认值。

另一个终端可以查询或停止：

```bash
~/catkin_ws/src/xd-uavsystem-test/src/xd_uav_control_manager/scripts/arducopter_demo.sh status
~/catkin_ws/src/xd-uavsystem-test/src/xd_uav_control_manager/scripts/arducopter_demo.sh stop
```

Gazebo 窗口会显示 Iris 四旋翼。运行日志和PID只写入
`/tmp/xd_uav_arducopter_demo_$UID`。脚本不会解锁或起飞；下面的
状态检查、起飞、位置/速度移动和降落仍需由操作者显式执行。后续章节中的手动启动
方式保留用于逐层排障。

只需控制链而不需要三维界面时，可继续使用原来的内置物理模型：

```bash
~/catkin_ws/src/xd-uavsystem-test/src/xd_uav_control_manager/scripts/arducopter_demo.sh start --headless
```

## 1. 环境和构建

```bash
export CATKIN_WS="${CATKIN_WS:-$HOME/catkin_ws}"
cd "$CATKIN_WS"
source /opt/ros/noetic/setup.bash
catkin_make -j2 --pkg xd_uav_controller xd_uav_control_manager
source "$CATKIN_WS/devel/setup.bash"
```

固定版本 ArduPilot 源码应位于：

```text
$CATKIN_WS/third_party/ardupilot-Copter-4.7.1
```

其 `build/sitl/bin/arducopter` 必须已经完成构建。下面第 2～3 节的手工排障命令使用
ArduPilot 内置 quad 物理模型，不启动 Gazebo；通常演示直接使用上面的一键入口即可。

## 2. 手动启动 ArduCopter SITL

在终端 A 启动固定版本飞控：

```bash
export CATKIN_WS="${CATKIN_WS:-$HOME/catkin_ws}"
export ARDUPILOT_DIR="$CATKIN_WS/third_party/ardupilot-Copter-4.7.1"
cd "$ARDUPILOT_DIR"
env -u DISPLAY /usr/bin/python3 Tools/autotest/sim_vehicle.py \
  -v ArduCopter -f quad -N --no-mavproxy -w \
  --use-dir /tmp/arducopter_sitl \
  --add-param-file="$CATKIN_WS/src/xd-uavsystem-test/src/xd_uav_control_manager/config/sitl/arducopter.parm"
```

`-w`会重置这次 SITL 的参数，然后加载仓库内固定的 `GUID_OPTIONS=8` 与 MAVLink
遥测流率。该命令会持续运行，不要关闭终端 A。

## 3. 手动启动 MAVROS、定位和控制链

在终端 B 启动完整 ROS 链：

```bash
export CATKIN_WS="${CATKIN_WS:-$HOME/catkin_ws}"
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash"
roslaunch xd_uav_control_manager arducopter_sitl_system.launch UAV_NAME:=uav1
```

此入口使用独立 ROS master 时最安全。它默认发布基于单调时钟的 `/clock`；如果接入
Gazebo 或其他已经发布 `/clock` 的仿真器，必须追加：

```bash
use_steady_sim_time:=false
```

## 4. 起飞前检查

在终端 C 加载环境：

```bash
export CATKIN_WS="${CATKIN_WS:-$HOME/catkin_ws}"
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash"
```

依次检查连接、定位和控制状态：

```bash
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav1/state_estimator/status
rostopic echo -n 1 /uav1/control_manager/state
```

正常条件至少包括：

- MAVROS `connected: True`；
- estimator 的 `state_valid`、`localization_valid` 为 true；
- control manager 的 `state_valid`、`stable` 为 true；
- 起飞前 `armed: false`、`landed_state` 为地面状态。

还可以检查后端要求的实时参数：

```bash
rosservice call /uav1/mavros/param/get "param_id: 'GUID_OPTIONS'"
rosservice call /uav1/mavros/param/get "param_id: 'MOT_THST_HOVER'"
```

预期分别包含整数 `8` 和实数 `0.39`。manager 会在接管前再次只读核对，并请求
MAVLink 消息 245；任何一步失败都会拒绝进入 `GUIDED`。

## 5. 起飞和悬停

请求相对当前位置起飞 2 m：

```bash
rosservice call /uav1/control_manager/takeoff "altitude: 2.0"
```

观察模式、解锁和高度：

```bash
rostopic echo /uav1/mavros/state
rostopic echo /uav1/state_estimator/main/odom
```

正常结果是进入 `GUIDED`、解锁、连续爬升后稳定在目标高度附近。若高度持续快速上升、
定位失效或姿态异常，立即执行本文末尾的安全降落命令。

## 6. 位置目标演示

下面命令向公共 simple-goal 入口发送 `uav1/odom` 下的目标点。默认配置使用消息的
水平位置并保持当前高度：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"{header: {stamp: now, frame_id: 'uav1/odom'}, pose: {position: {x: 1.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}"
```

观察里程计 `pose.pose.position.x` 向约 1 m 移动并稳定：

```bash
rostopic echo /uav1/state_estimator/main/odom/pose/pose/position
```

该目标会锁存，不需要持续发布。

## 7. 速度目标演示

下面命令以 20 Hz 发布 3 秒：水平 X 速度为 -0.3 m/s，同时将 Z 位置保持为 2 m。
`3555`是“VX/VY + PZ、忽略其余轴和 yaw”的逐轴掩码：

```bash
timeout 3s rostopic pub -r 20 \
  /uav1/control/reference/setpoint mavros_msgs/PositionTarget \
"{header: {stamp: now, frame_id: 'uav1/odom'}, coordinate_frame: 1, type_mask: 3555, position: {z: 2.0}, velocity: {x: -0.3, y: 0.0, z: 0.0}}"
```

`timeout`结束时返回码 124 是预期行为。流式命令停止并超过 reference timeout 后，
多旋翼会捕获当前位置并恢复悬停，不会继续无限移动。

## 8. 受控降落

在当前位置降落：

```bash
rosservice call /uav1/control_manager/land
```

正常结果是连续下降、触地确认、零推力以及最终自动上锁。检查：

```bash
rostopic echo -n 1 /uav1/mavros/extended_state
rostopic echo -n 1 /uav1/mavros/state
```

最终应为地面状态且 `armed: false`。

如果正常控制链异常但 MAVROS 仍连接，可直接请求飞控原生 LAND：

```bash
rosservice call /uav1/mavros/set_mode "base_mode: 0
custom_mode: 'LAND'"
```

## 9. 结束和清理

1. 确认飞机已经落地且 `armed: false`。
2. 若用最简脚本启动，在启动终端按 `Ctrl-C`；也可在另一终端执行
   `arducopter_demo.sh stop`。脚本会同时停止 Gazebo、ROS 链与 SITL。
3. 若按第 2～3 节手工分层启动，则分别在 ROS launch 和 SITL 终端按 `Ctrl-C`。
4. 确认没有遗留进程：

```bash
ps -eo pid,ppid,stat,cmd | \
  grep -E '[a]rducopter|[g]zserver|[g]zclient|[m]avros|[r]oscore|[r]osmaster|[r]oslaunch'
```

没有输出即表示本次演示已清理完成。

## PX4 默认路径

原有 PX4 启动入口不变：

```bash
roslaunch xd_uav_control_manager multirotor_system.launch UAV_NAME:=uav1
```

它默认加载 `config/autopilot/px4.yaml`，controller 输出保持 body-rate，manager 使用
`OFFBOARD`/`POSCTL`。ArduCopter 参数只由选中的 ArduCopter backend 读取；即使参数服务器
残留无效的 `autopilot/arducopter/*` 值，也不能影响 PX4 后端启动。
