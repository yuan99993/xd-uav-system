# xd_uav_control_manager

状态适配、安全监督和飞控外部控制流程编排包。节点启动后只发布统一控制状态，不会
自动发送飞行控制量、切换模式或解锁。

可直接执行的PX4/ArduCopter启动、起飞、位置/速度演示和清理命令见
[`docs/USAGE.md`](docs/USAGE.md)。

固定版本ArduCopter SITL的一键入口为
`rosrun xd_uav_control_manager arducopter_demo.sh start`；默认使用本机 Gazebo Classic 11
显示 Iris，并启动 SITL 与控制链。它不会自动解锁或起飞，`Ctrl-C`会清理脚本启动的
全部进程；追加 `--headless` 可使用原来的无界面内置物理模型。

## 分层

控制链按职责分层：

- 控制器维护悬停、轨迹和起降参考，输出显式body-rate或quaternion attitude合同。
- 管理器显式启用控制输出，编排飞控模式、解锁、起飞和降落，并监督输入新鲜度。
- 统一控制状态携带MAVROS实时观测的`armed`；控制器据此在预发送阶段保持地面参考，
  只在飞控确认解锁后推进受限速/限加速度约束的起飞轨迹。
- `autopilot/`后端拥有模式名、退出模式、MAVROS mask和飞控能力检查；airframe
  `VehicleAdapter`仍独立处理多旋翼、固定翼和VTOL行为。
- PX4后端保持`OFFBOARD`、body-rate + thrust与`IGNORE_ATTITUDE`原行为。
- ArduCopter后端首阶段仅支持multirotor，发送quaternion + thrust并忽略三轴body-rate。

管理器不会修改飞控参数。PX4的`COM_RCL_EXCEPT`、OFFBOARD丢失动作和RC失效动作
仍必须在PX4启动参数中配置并验证。ArduCopter激活前会只读检查`GUID_OPTIONS` bit 3
（mask 8），确认`SET_ATTITUDE_TARGET.thrust`按原始0～1推力解释；同时读取
`MOT_THST_HOVER`并与控制器命令声明的悬停推力工作点核对。任一参数读取失败、能力位
未启用或推力基准超出容差时均 fail closed，不请求`GUIDED`。激活还会通过MAVROS显式
请求`EXTENDED_SYS_STATE`（MAVLink消息245）10 Hz；ArduPilot的`EXTENDED_STATUS`流
本身不包含该消息，请求失败同样拒绝接管。

## 状态机

```text
STANDBY
  -- offboard --> WAIT_STATE -> PRESTREAM -> REQUEST_ACTIVE_MODE -> ACTIVE
  -- takeoff  --> WAIT_STATE -> PRESTREAM -> REQUEST_ACTIVE_MODE
                                  -> REQUEST_ARM -> ACTIVE/悬停

ACTIVE -- land/land_home --> LANDING -> 上锁 -> STANDBY
LANDING -- cancel_land（确认仍在空中）--> ACTIVE
ACTIVE -- cancel_offboard/人工切模式 --> STANDBY
ACTIVE -- 持续输入失效 --> FAILSAFE
```

`STANDBY`不会向`mavros/setpoint_raw/attitude`发布消息。进入`ACTIVE`后持续发布
setpoint，从而维持所选飞控的主动控制模式。起飞结束后控制器保留同一个位置和yaw参考，不会自行
下降；只有调用`land`或`land_home`服务才建立下降参考。

管理器不会在检测到地面站主动切出主动控制模式后再次抢回模式，而是立即停止外部控制并
回到`STANDBY`。兼容接口`cancel_offboard`会先请求所选后端的`exit_mode`，确认飞控
接受模式请求后才停止setpoint。

短于`safety/controller_command/invalid_grace_duration`的单次输入抖动会暂时复用最后一条安全控制量，
不会立即退出主动控制。持续失效仍会停止输出，由飞控执行其配置的失联保护
策略；这是安全例外，不能承诺任何传感器故障下都永不降落。

## 服务

对外飞行业务接口只位于`/uavX/control_manager/*`。不要直接调用
`/uavX/controller/internal/command`，该服务仅用于管理器与控制器节点之间的编排。
每个roscpp节点自动生成的`get_loggers`和`set_logger_level`属于ROS日志管理接口，
不是飞行业务服务。

只进入并维持OFFBOARD，不解锁：

```bash
rosservice call /uav1/control_manager/offboard
```

取消OFFBOARD并默认交还给PX4 `POSCTL`：

```bash
rosservice call /uav1/control_manager/cancel_offboard
```

组合执行OFFBOARD、解锁和相对高度起飞：

```bash
rosservice call /uav1/control_manager/takeoff "altitude: 3.0"
```

四旋翼在当前位置原地受控降落：

```bash
rosservice call /uav1/control_manager/land
```

四旋翼先返回本次起飞位置，再受控降落：

```bash
rosservice call /uav1/control_manager/land_home
```

触地前取消当前降落，并保持解锁和主动控制模式：

```bash
rosservice call /uav1/control_manager/cancel_land
```

取消后四旋翼会捕获当前位置悬停，固定翼会沿当前航向平滑进入等待盘旋；之后和普通
`ACTIVE`状态一样，可以接收新的外部参考或再次调用降落。为避免触地后突然恢复推力，
控制器已经报告触地、管理器已经进入触地持续确认，或飞控不能实时确认飞机仍在空中时，
`cancel_land`会拒绝请求并继续原降落流程。

`land_home`使用控制器公共配置中的home语义。`home/mode: takeoff`会在起飞时记录
当前位置；仓库提供的`common_config.yaml`当前使用`fixed_local`，其
`home/fixed_position`和`home/fixed_yaw`定义固定Home。执行返航时，Home会按最新TF
转换到控制odom。它不是“最近一次落地点”，降落和控制器内部复位都不会重写固定Home。

两种降落都会生成限速连续轨迹，近地阶段自动减速。管理器同时使用飞控的
`mavros/extended_state`和控制器的独立触地判定。触地持续确认后，管理器锁存触地
状态并持续发送零姿态变化目标和零推力，先请求普通上锁。如果飞控落地检测异常并以
`not landed`拒绝普通请求，管理器等待配置的安全时间后，通过
`MAV_CMD_COMPONENT_ARM_DISARM(param2=21196)`执行受限强制上锁，直到
`mavros/state.armed == false`后才停止外部控制输出。强制上锁只有在本次飞行已经
确认处于空中、随后又持续满足触地条件时才可能执行。

固定翼的`land`和`land_home`使用进近点、直线下滑、拉平和滑跑，而不是四旋翼的
垂直下降。`land`在当前course前方建立临时接地点，`land_home`以公共Home三维位置为
接地点；若`home/use_home_yaw: true`，公共Home yaw同时作为跑道着陆方向。固定翼
即使收到PX4触地状态或控制器触地信号，地速高于
`safety/touchdown/fixedwing_max_groundspeed`时也不会停桨解锁。

复位FAILSAFE：

```bash
rosservice call /uav1/control_manager/reset_failsafe
```

当前固定翼降落属于SITL基础实现，不包含地形/跑道探测、侧风补偿和自动复飞，不能
直接用于真机。

## 话题

状态输入和适配输出保持不变：

```text
/uavX/state_estimator/main/odom
/uavX/state_estimator/main/acceleration
/uavX/state_estimator/status
/uavX/mavros/imu/data
/uavX/mavros/vfr_hud
/uavX/mavros/state
/uavX/mavros/extended_state
/uavX/controller/command

/uavX/control_manager/state
/uavX/control_manager/status
/uavX/control_manager/diagnostics
/uavX/mavros/setpoint_raw/attitude
```

统一状态适配器会把`main/odom.twist.linear`从机体系旋转到odom世界系。三轴body
rates来自IMU；固定翼空速来自`mavros/vfr_hud`。

## 配置

- `config/autopilot/px4.yaml`是默认后端；`config/autopilot/arducopter.yaml`选择
  `GUIDED`/`LOITER`和`GUID_OPTIONS`能力门。兼容服务名仍保留为`offboard`与
  `cancel_offboard`，但在ArduCopter配置下实际请求/退出的是对应GUIDED模式。
- `launch/arducopter_multirotor_system.launch`成对加载ArduCopter manager配置和
  controller attitude输出配置。真机应通过`controller_override_config`提供与机体
  `MOT_THST_HOVER`一致的控制器配置；不一致会被安全门拒绝。该入口只启动本项目
  控制层，不安装或启动ArduPilot SITL。
- `config/sitl/arducopter.parm`固定SITL的`GUID_OPTIONS=8`和Copter 4.7.1的`MAV1_*`
  SERIAL0遥测流率，避免依赖MAVProxy或地面站临时请求数据流。它只用于SITL，不能
  直接复制到真机；旧版`SR0_*`参数不适用于该固定固件。
- `launch/sitl/arducopter_sitl_system.launch`使用MAVROS官方APM插件配置，并把IMU
  `frame_id`覆盖为带飞机命名空间的`uavX/base_link`。ArduPilot内置物理模型不发布
  `/clock`，而WSL主机的实时时钟可能被Windows周期性步进；因此该SITL入口默认启用
  `steady_sitl_clock.py`，以单调时钟驱动ROS仿真时间，并把MAVROS timesync设为`NONE`。
  `use_steady_sim_time:=false`可关闭该行为；Gazebo等已有`/clock`发布者时必须关闭，禁止
  同一ROS master存在两个时钟源。真机入口不启用仿真时间，仍保留官方`MAVLINK`默认值。
  该launch同时可启动world TF、定位栈和ArduCopter控制栈。ArduPilot进程仍在ROS外
  独立启动，便于固定上游版本和清理进程。其控制器覆盖默认使用
  `config/arducopter_copter_4_7_sitl.yaml`，悬停推力`0.39`来自固定版本官方
  `Tools/autotest/default_params/copter.parm`，运行时仍会和FCU实值再次核对。

### ArduCopter SITL

已验证版本固定为`Copter-4.7.1`。推荐使用本文开头的`arducopter_demo.sh start`，它默认启动
Gazebo Classic 11、legacy Iris插件、SITL和ROS控制链；`start --headless`才使用ArduPilot内置
quad物理模型。分层手工启动、状态检查、飞行命令和清理步骤统一维护在
[`docs/USAGE.md`](docs/USAGE.md)，README不再复制第二套操作流程。

- `config/offboard.yaml`按职责分为三组：`offboard/stream/*`负责setpoint发送和预发送，
  `offboard/mode_request/*`负责主动模式请求重试与超时，`offboard/exit/*`是旧版退出参数的
  接管模式。
- `config/safety.yaml`按职责分为四组：`safety/inputs/*`检查传感器、估计器和飞控状态，
  `safety/controller_command/*`监督控制器输出，`safety/activation/*`控制进入主动模式前
  的稳定等待，`safety/touchdown/*`负责触地确认、零推力等待和上锁回退。
- 输入超时同时检查消息回调接收间隔和消息时间戳延迟。MAVROS FCU时间同步与Gazebo
  `/clock`之间的微小相位差由`safety/inputs/future_stamp_tolerance`吸收；超过该范围的
  未来时间戳仍按时钟异常处理，不能通过增大普通输入超时绕过。
- 真机使用前必须重新验证触地容差，并可通过
  `safety/touchdown/force_disarm/enabled: false`关闭强制上锁。固定翼还通过
  `safety/touchdown/fixedwing_max_groundspeed`限制可确认触地和解锁的最大地速。
  固定翼SITL地面静止时若`mavros/vfr_hud.airspeed`出现轻微负噪声，
  `safety/inputs/airspeed_negative_tolerance`范围内会按`0m/s`输入控制器；超过该负值
  容差仍判定为空速无效。
- 旧版扁平参数路径仍可在未配置新路径时兼容读取，并在启动时输出迁移提示；新配置和
  launch覆盖应使用上述分层路径。
- 控制器公共参数放在`xd_uav_controller/config/common_config.yaml`，四旋翼和固定翼
  参数分别放在`multirotor.yaml`与`fixedwing.yaml`。系统launch先加载公共配置，再加载
  机型配置。
