# 异构机架统一控制底座重构审计与实施设计

审计日期：2026-09-03

实施规范：`/home/kzy/xd_uav_system_control_base_refactor_for_codex.md`

本文件同时作为 Phase 0/1 审计结果和 Phase 2~9 的实施设计。后续改动严格限制在：

```text
/home/kzy/xd-uavsystem-test/src/xd_uav_controller
/home/kzy/xd-uavsystem-test/src/xd_uav_control_manager
```

旧 `xd_uav_controller`、`xd_uav_control_manager` 和其余业务包保持只读。

## 1. Phase 0 基线

### 1.1 Git 与工作树

```text
repository: /home/kzy/xd-uavsystem-test
branch: feature/xd-uav-construction
HEAD: 8223661e7cd748199724a3cff2d0d4d2a13759db
upstream relation: origin/dev ahead 19
```

审计开始时的未跟踪内容：

```text
src/tmux_start/session_one_vtol_px4.yml
src/xd_uav_control_manager/
src/xd_uav_controller/
```

其中 `tmux_start` 文件不属于本轮范围，不读取、不修改。两个 `_new` 目录是本次唯一实施基线，不使用旧包或历史版本覆盖。

### 1.2 隔离构建环境

隔离工作空间：

```text
/tmp/xd-uav-baseline-fKwev9
```

source space 只链接：

```text
xd_uav_controller
xd_uav_control_manager
xd_uav_state_estimators    # manager 的直接消息依赖
```

环境与命令：

```bash
source /opt/ros/noetic/setup.bash
cd /tmp/xd-uav-baseline-fKwev9
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_xd_uav_controller run_tests_xd_uav_control_manager
catkin_test_results --all build/test_results
```

构建结果：成功，两个 `_new` 节点和消息、服务均能生成并链接。

测试结果：28 tests，0 errors，2 failures，0 skipped。

通过的主要测试：

- 公共枚举解析、合法组合和 transition 源 backend 选择；
- VTOL observed regime 映射、service ACK 与 observed state 分离、timeout；
- `ControlState`/`ControlCommand`/`SetFlightRegime` 接口常量；
- multirotor anti-windup；
- fixed-wing controller；
- multirotor/fixed-wing control manager 生命周期。

既有 baseline failure：

1. `rosunit-controller_interface/test_reference_and_takeoff`
   - 期望 latched `reference_position_target.header.frame_id == uav1/local_origin`；
   - 实际读到 `uav1/odom`；
   - 同期日志显示 local alignment 尚未有效，需先消除测试发布/latched topic 的时序歧义，再判断是否为实现问题。
2. `rosunit-odom_landing/test_odom_landing_does_not_require_range`
   - `multirotor.yaml` 已配置新路径 `multirotor/landing/height_source: distance_sensor`；
   - 测试仍覆盖旧路径 `landing/height_source: odom`；
   - 参数加载优先读取新路径，因此旧路径不能覆盖已存在的新路径，导致降落被距离传感器准入拒绝。

这两个失败在 Phase 1 golden baseline 建立前单独修复，不与控制数学搬迁混合。

测试日志目录：

```text
/tmp/xd-uav-baseline-fKwev9/build/test_results
/home/kzy/.ros/log
```

## 2. 完整目录基线

忽略 Python 解释器生成的 `__pycache__/*.pyc` 后，审计开始时两个包共有 46 个文件；加入本审计文件后为 47 个。

```text
xd_uav_controller/
├── CMakeLists.txt
├── README.md
├── config/
│   ├── common_config.yaml
│   ├── fixedwing.yaml
│   ├── multirotor.yaml
│   ├── typhoon_h480_override.yaml
│   └── vtol.yaml
├── docs/
│   └── control_base_refactor_audit.md
├── include/xd_uav_controller/
│   └── control_types.h
├── launch/
│   └── controller.launch
├── msg/
│   ├── ControlCommand.msg
│   ├── ControlState.msg
│   └── PathStatus.msg
├── package.xml
├── scripts/
│   └── publish_fixedwing_trajectory.py
├── src/
│   └── controller_node.cpp
├── srv/
│   ├── InternalCommand.srv
│   └── Takeoff.srv
└── test/
    ├── controller.test
    ├── fixedwing_controller.test
    ├── multirotor_anti_windup.test
    ├── odom_landing.test
    ├── test_control_types.cpp
    ├── test_controller.py
    ├── test_fixedwing_controller.py
    ├── test_multirotor_anti_windup.py
    └── test_odom_landing.py

xd_uav_control_manager/
├── CMakeLists.txt
├── README.md
├── config/
│   ├── offboard.yaml
│   ├── safety.yaml
│   └── vtol.yaml
├── include/xd_uav_control_manager/
│   └── vtol_vehicle_adapter.h
├── launch/
│   ├── control_manager.launch
│   ├── fixedwing_system.launch
│   ├── multirotor_system.launch
│   └── vtol_system.launch
├── package.xml
├── src/
│   ├── control_manager_node.cpp
│   └── vtol_vehicle_adapter.cpp
├── srv/
│   └── SetFlightRegime.srv
└── test/
    ├── control_manager.test
    ├── fixedwing_control_manager.test
    ├── test_control_interfaces.cpp
    ├── test_control_manager.py
    └── test_vtol_vehicle_adapter.cpp
```

## 3. 最新控制输入能力矩阵

### 3.1 外部和内部 Reference 类型

| Reference/Input | ROS 消息及字段 | 入口与 adapter | Multirotor 路径 | Fixed-wing 路径 | 必须保留 |
|---|---|---|---|---|---|
| Masked position | `PositionTarget.position.{x,y,z}` + `IGNORE_P*` | `referencePositionTargetCallback()` → `positionTargetToSourceReference()` → TF normalization | 任意逐轴 P，与 V/A 可组合，进入 finite-horizon MPC | PXY 生成几何位置闭环，PZ 生成高度闭环 | 是 |
| Masked velocity | `PositionTarget.velocity.{x,y,z}` + `IGNORE_V*` | 同上 | 任意逐轴 V，限速后进入 MPC | VXY 提供 course/airspeed，VZ 提供 climb-rate | 是 |
| Masked acceleration | `PositionTarget.acceleration_or_force.{x,y,z}` + `IGNORE_AF*` | 同上；拒绝 `FORCE` | 支持纯 A 或 P/V+A，含反馈积分、jerk/saturation/anti-windup | 仅作前馈；水平 A 要求 VX/VY/AX/AY 完整，AZ 要求 PZ 或 VZ | 是 |
| Yaw | `PositionTarget.yaw` + `IGNORE_YAW` | 同上并经过惯性 frame 旋转 | 期望机头 yaw | 无 VXY 时可作 course 语义；有路径速度时以几何 course 为主 | 是 |
| Yaw rate | `PositionTarget.yaw_rate` + `IGNORE_YAW_RATE` | 同上并经过 frame 旋转 | yaw-rate feed-forward | course-rate feed-forward | 是 |
| Timed trajectory | `MultiDOFJointTrajectory.transforms[0]`、可选 `velocities[0]`、`accelerations[0]`、`time_from_start` | `referenceTrajectoryCallback()` → validation → linear/time interpolation → TF normalization | P/V/A/yaw/yaw-rate 轨迹跟踪 | P/V/A/yaw/yaw-rate 适配到 course/airspeed/altitude/climb-rate；结束后进入 loiter | 是 |
| Geometric path | `nav_msgs/Path.poses[].pose.position`，首 pose/header seq 用作 path id | `referencePathCallback()` → 去重/TF → 有序投影、重捕获、曲率估计 | lookahead P/V/yaw，末端减速，曲率生成 A/yaw-rate | 切线生成 course/airspeed/climb-rate，曲率前馈，完成后 loiter | 是 |
| Simple goal | `PoseStamped.position`、quaternion yaw | `simpleGoalCallback()` → 发布兼容 `PositionTarget` → point reference | XY point；按配置使用消息 Z 或保持当前目标高度 | point/heading 适配到 fixed-wing guidance | 是 |
| Idle/hold | 内部生成 | `captureIdleReference()` / `makeIdleReference()` | 捕获当前位置和 yaw 悬停 | 捕获高度、course、airspeed 后建立等待参考 | 是 |
| Takeoff | `Takeoff.srv.altitude` / `InternalCommand` | manager 发起，controller 内部 reference generator | 锁定 XY/yaw，平滑垂直起飞 | 起飞油门、rotate airspeed、climb pitch，完成后切相切 loiter | 是 |
| Land | `std_srvs/Trigger` / `InternalCommand` | manager 发起，controller 内部 reference generator | 原地垂直下降 | 当前 course 前方建立 approach/glideslope/flare/rollout | 是 |
| Return-home land | `std_srvs/Trigger` / `InternalCommand` | manager 发起，home TF adaptation | 水平制动返回 home 后垂直下降 | 配置 home 作为 touchdown，执行进近和滑跑 | 是 |
| Landing cancel | `InternalCommand.CANCEL_LANDING` | 捕获当前状态，清除降落 reference | 悬停 | 平滑进入 loiter | 是 |

### 3.2 当前不存在的输入

当前没有独立的 attitude、body-rate、thrust 或直接 pass-through 订阅入口。`body_rate` 和 `thrust` 是 `ControlCommand` 输出，不应在本轮凭空扩展为输入。

### 3.3 Reference 数据模型现状

当前 `Reference` 已实际包含：

```text
header/stamp/frame
position[xyz] + per-axis validity
velocity[xyz] + per-axis validity
acceleration[xyz] + per-axis validity
jerk[xyz] + group validity
yaw + validity
yaw_rate + validity
trajectory/path source flags
```

固定翼适配后形成 `FixedwingControlTarget`：

```text
course
course_rate_feedforward
airspeed
altitude_error
climb_rate_feedforward
vertical_acceleration_feedforward
altitude/course integrator enable flags
```

新的 `UnifiedReference` 必须至少完整承载这些真实能力，不添加无真实入口的字段。

## 4. 当前控制链

### 4.1 Multirotor

```text
PositionTarget / Trajectory / Path / SimpleGoal / Internal generator
  -> source Reference + masks
  -> frame allowlist/global-alignment/TF validation
  -> normalized Reference
  -> per-axis P/V/A validity and limiters
  -> finite-horizon axis MPC
  -> disturbance rejection integrators
  -> acceleration feedback + anti-windup
  -> jerk/acceleration/tilt/throttle/body-rate limits
  -> SO(3) attitude-to-body-rate
  -> ControlCommand(body_rate, thrust, validity, metadata)
```

### 4.2 Fixed-wing

```text
PositionTarget / Trajectory / Path / SimpleGoal / Internal generator
  -> source Reference + masks
  -> frame allowlist/global-alignment/TF validation
  -> normalized Reference
  -> fixed-wing reference adaptation
     (position closure, velocity course, curvature/course-rate,
      altitude/climb-rate, airspeed, loiter/takeoff/landing semantics)
  -> FixedwingControlTarget
  -> lateral course PI + roll loop
  -> vertical altitude/climb-rate/pitch loop
  -> energy/airspeed/throttle loop
  -> underspeed and bank load-factor protection
  -> attitude-to-body-rate and output limits
  -> ControlCommand(body_rate, thrust, validity, metadata)
```

### 4.3 Manager 到 PX4

```text
Odometry + IMU + acceleration + estimator status + VFR_HUD
+ MAVROS State + ExtendedState
  -> ControlState validity/freshness/regime/action
  -> controller reference -> selected control math -> ControlCommand
  -> manager generation/backend/validity checks
  -> input grace / last safe target
  -> MAVROS AttitudeTarget
  -> PX4
```

## 5. `vehicle_type` 与机型分支审计

### 5.1 Config parsing 和兼容接口

- Controller `loadParameters()`：优先 `airframe_type`，否则读取 deprecated `vehicle_type`。
- Manager `loadParameters()`：同样执行兼容解析。
- `controller.launch`、`control_manager.launch`：同时暴露两个参数。
- multirotor/fixed-wing 系统 launch 和既有测试仍主要使用 `vehicle_type`。
- `ControlState.msg`、`ControlCommand.msg` 保留 legacy `vehicle_type` 字段。

这些位置属于本轮允许保留的兼容边界。

### 5.2 必须收敛的运行逻辑

Manager 仍在以下运行路径直接比较原始字符串 `vehicle_type_ == "fixedwing"`：

- `startLanding()` 的降落类型和状态文本；
- `cancelLandingCallback()` 的恢复策略文本；
- touchdown 地速门限；
- touchdown 高速告警。

这会让合法别名 `fixed_wing` 走入多旋翼分支，也无法正确表达 VTOL 当前 regime，必须改为 AirframeType、FlightRegime 或 VehicleAdapter capability。

Controller 已主要使用 `activeBackend()`，但以下职责仍集中在单个 Node：

- `load_multirotor_`/`load_fixedwing_` 参数分组；
- external reference 对 fixed-wing 的能力判断；
- takeoff/landing/loiter 分支；
- control math 选择；
- diagnostics/controller 名称。

这些分支要迁移到 capability、ReferenceAdapter、ControllerBackend，不继续增加 VTOL 特判。

### 5.3 Testing

旧参数入口测试必须保留，以证明 deprecated compatibility。新增测试使用 `airframe_type`，并覆盖 `fixedwing` 与 `fixed_wing` 映射一致性。

## 6. 当前职责划分

### 6.1 `controller_node.cpp`

| 职责 | 当前函数/区域 |
|---|---|
| ROS I/O | constructor subscriptions/publishers/services/timer |
| 参数与配置 | `loadParameters()` |
| reference parsing | `positionTargetToSourceReference()`、trajectory/path/simple-goal callbacks |
| reference adaptation | TF functions、trajectory/path sampling、`updateFixedwingExternalSetpointGuidance()`、`fixedwingControlTarget()` |
| lifecycle reference generator | takeoff/landing/loiter/idle functions |
| state validation | `stateCallback()`、`validReference()`、freshness checks |
| multirotor math | MPC、integrators、limiters、`multirotorControl()` |
| fixed-wing math | path/course/energy/landing target、`fixedwingControl()` |
| output | `timerCallback()` and `ControlCommand` metadata |

单文件 5742 行，尚未形成规范要求的独立 backend 和 adapter。

### 6.2 `control_manager_node.cpp`

| 职责 | 当前函数/区域 |
|---|---|
| 公共 MAVROS lifecycle | mode/arm/disarm/force-disarm clients and retry |
| OFFBOARD | manager State、prestream、request/active/cancel |
| 公共 safety | input freshness、command validation、grace、failsafe |
| ControlState adapter | odometry/IMU/acceleration/airspeed/ExtendedState aggregation |
| multirotor/fixed-wing lifecycle | 当前仍由公共 `startLanding()` 和 controller internal service 隐式分支 |
| VTOL regime | `VtolVehicleAdapter::observe()` |
| VTOL transition | `setFlightRegimeCallback()` + MAVROS service |
| diagnostics | `publishDiagnostics()` |

公共 OFFBOARD、arming、state freshness 和 failsafe 继续留在 manager 公共层。机架差异迁入 VehicleAdapter。

## 7. 已完成与缺口矩阵

| 规范项 | 状态 | 审计结论 |
|---|---|---|
| `_new` 包名和依赖 | 已完成 | package/project/include/launch/test 命名一致 |
| Airframe/Regime/Action 公共值 | 部分完成 | C++ enum 在一处；ROS 常量存在；需要增加线上值一致性测试 |
| observed/requested 分离 | 部分完成 | service ACK 不伪造 observed regime；失败/完成后的 target/result 生命周期需完善 |
| UnifiedReference | 未完成 | 仍是 Node 内部 `Reference` |
| ReferenceAdapter | 未完成 | adapter 函数散落在 Node |
| ControllerBackend | 未完成 | 两套数学仍是 Node 方法 |
| BackendResolver | 部分完成 | `backendForRegime()` 未接收 airframe/transition status，错误信息能力不足 |
| bumpless handover | 部分完成 | 切换时重置两套状态，未以当前 state/reference 激活目标 backend |
| source backend transition output | 部分完成 | 常规 transition 保持 member `active_backend_`；缺完整 dropout/cache/generation 测试 |
| VehicleAdapter interface/capability | 未完成 | 仅有不完整 VTOL 类，无 multirotor/fixed-wing adapter 或 capability table |
| MAVROS VTOL transition | 部分完成 | 参数化 service、ACK 分层和 observed confirmation 已有；diagnostic/result/reset 不完整 |
| Action ownership protocol | 未完成 | `InternalCommand.srv` 无 action/phase/generation；controller 仍拥有顶层启动判断 |
| VehicleAction 真实来源 | 未完成 | ACTIVE 一律近似 HOLD，未消费 command reference metadata |
| action status/result | 未完成 | command 基本只产生 IDLE/ACTIVE，无 succeeded/failed/detail |
| action_generation rejection | 未完成 | manager 不验证 command action generation |
| VTOL takeoff | 部分完成 | HOVER regime 可复用 multirotor takeoff，但缺 adapter capability 和专用测试 |
| VTOL landing | 未完成且不安全 | FW land 会直接调用 fixed-wing landing，未先请求 MC |
| timeout fallback | 不符合 | 当前 transition timeout 无条件进入 FAILSAFE，未区分已回源稳定形态 |
| ExtendedState dropout | 部分完成 | manager 有 input grace；缺保持源 backend 的端到端测试 |
| VTOL config loading | 不符合 | controller `vtol.yaml` 复制两套 gains，launch 未按 common→两个 backend→vtol 加载 |
| 纯机型 rate/dt | 基本保留 | multirotor 100Hz、fixed-wing 50Hz；需 golden test 锁定 |
| diagnostics | 部分完成 | 缺 transition target/detail、backend/reference readiness/action generation 等 |
| golden numerical regression | 未完成 | 现有 rostest 多为范围断言，不是可重放序列 |
| 规范新增测试 | 大量缺失 | 无完整 VTOL manager/controller rostest、landing sequencing、dropout、generation handover |

## 8. 不可回归能力

### 8.1 公共输入与坐标系

- PositionTarget 逐轴 mask 和非法/未知 mask 拒绝；
- 只接受明确的 ROS ENU 惯性语义；
- reference frame allowlist、namespace-safe frame resolution；
- global alignment ready 门限；
- TF max age、future stamp 和短时 transform grace；
- point reference latch 与 streaming timeout 的不同语义；
- Path 有序投影，不能在相交或相邻路径段间错误跳跃；
- Path 去重、reacquisition、completion/status；
- trajectory 严格时间递增、字段一致性和插值。

### 8.2 Multirotor

- finite-horizon axis MPC 和固定 dt；
- position/velocity/acceleration 三类扰动抑制；
- acceleration feedback 去重力语义；
- actuator saturation 后 anti-windup；
- velocity/acceleration/jerk/tilt/body-rate/throttle limit；
- SO(3) attitude-to-rate；
- smooth takeoff；
- return-home braking envelope；
- odom 和 distance-sensor 两类垂直降落；
- distance sensor dropout 冻结高度而非盲降；
- touchdown AGL rate filtering。

### 8.3 Fixed-wing

- masked P/V/A 到 course/airspeed/altitude/climb-rate 的适配；
- streaming course-rate estimation/filter；
- path curvature preview 与 adaptive lookahead filter；
- course PI 和 altitude integral；
- climb-rate feedback 与 vertical acceleration pitch-rate feed-forward；
- bank load-factor pitch/throttle compensation；
- bank-protected airspeed、underspeed hysteresis 和 recovery；
- takeoff rotate/climb/loiter；
- landing approach capture、line guidance、glideslope、flare/rollout；
- touchdown groundspeed gate；
- cancel landing 后相切 loiter。

### 8.4 Manager safety

- message source stamp 与 receive time 双重 freshness；
- bounded future-stamp tolerance；
- estimator status gating；
- OFFBOARD prestream/retry/timeout；
- command invalid grace 和 last-safe target；
- 人工退出 OFFBOARD 后不抢回；
- touchdown confirm、zero-thrust、normal/force disarm；
- force disarm 仅在确认曾经在空中且持续触地后启用。

## 9. 选定设计

采用“增量抽离、行为先锁定”的方案。每个阶段先增加会失败的行为或回归测试，再搬迁最少代码使其通过。禁止同时重写控制数学。

### 9.1 Controller 边界

```text
ROS callback
  -> UnifiedReference（完整 masks/timing/source metadata）
  -> ReferenceAdapter（公共 normalization + regime/backend capability）
  -> BackendResolver（唯一选择点）
  -> ControllerBackend interface
       ├── MultirotorControllerBackend
       └── FixedWingControllerBackend
  -> ControlCommand metadata + output
```

Backend 接口固定提供：

```text
configure
supportsRegime
supportsReference
onActivate(current state, current compatible reference)
onDeactivate
reset
update(state, reference, dt)
```

`onActivate()` 初始化积分器、filter/history 和上一控制时间；不得从另一个 backend 继承内部状态。Node 是唯一 `ControlCommand` publisher。

### 9.2 Manager 边界

```text
Common ControlManager lifecycle/safety
  -> VehicleAdapter interface and capability
       ├── MultirotorVehicleAdapter
       ├── FixedWingVehicleAdapter
       └── VtolVehicleAdapter (also used by TILTROTOR)
```

Adapter 声明：

```text
supported_regimes
supports_transition
supports_vertical_takeoff
supports_vertical_landing
requires_airspeed_in_forward_flight
```

公共 MAVROS State、OFFBOARD、arming、setpoint streaming、input grace 和 failsafe 不复制进 adapter。

### 9.3 Action 协议

扩展 `InternalCommand.srv`，在保留 legacy `command/altitude` 的同时增加：

```text
vehicle_action
action_phase
action_generation
return_home
```

`action_phase` 使用公共常量明确表达 `NONE`、`START`、`HOLD`、`APPROACH`、`REQUEST_HOVER`、`WAIT_HOVER`、`VERTICAL_DESCENT`、`TOUCHDOWN_CONFIRM`、`DISARM`、`CANCEL` 和 `RESET`。未在当前 action 合法 phase 集合中的命令必须拒绝，不能落入默认行为。

Manager 接受新顶层动作时只递增一次 generation，并持续在 `ControlState` 发布。Controller 只为相同 generation 生成 reference，回填 `ACTION_IDLE/ACTIVE/SUCCEEDED/FAILED` 和 detail。Manager 同时结合 PX4 armed/mode/landed 状态确认完成。

Manager 通过 controller 的 `reference_type/reference_source` 维护 HOLD/NAVIGATE，但 failsafe、disarm、takeoff、land、return-home 的显式生命周期优先级更高。

### 9.4 VTOL transition 与 landing

稳定形态切换只由 `ExtendedState.vtol_state` 确认。service transport、PX4 ACK 和最终 observed result 分别记录。

```text
HOVER -> request FW -> keep Multirotor backend
transition to FW -> keep Multirotor backend
PX4 confirms FW -> generation++ -> activate FixedWing backend once

FORWARD -> request MC -> keep FixedWing backend
transition to MC -> keep FixedWing backend
PX4 confirms MC -> generation++ -> activate Multirotor backend once
```

VTOL landing 子阶段：

```text
NONE
REQUEST_HOVER
WAIT_HOVER
VERTICAL_DESCENT
TOUCHDOWN_CONFIRM
DISARM
COMPLETE / FAILED
```

FW 收到 land 时只建立 LAND action 并请求 MC，不调用 controller 降落 generator。确认 MC 后才发送相同 action generation 的 vertical-descent phase。HOVER 收到 land 可直接进入 vertical descent。转换失败不会触发固定翼 runway landing 或多旋翼垂降。

timeout 时：若 PX4 明确回到源稳定形态，则保持源 backend 和 OFFBOARD、记录 ERROR 并等待显式重试；仍处于 transition/UNKNOWN 才进入 FAILSAFE。

### 9.5 Config 和 launch

`controller/config/vtol.yaml` 只保留 VTOL 特有选择和频率策略，不复制 multirotor/fixed-wing gains。

VTOL 加载顺序固定为：

```text
common_config.yaml
multirotor.yaml
fixedwing.yaml
vtol.yaml
optional vehicle override
```

纯 multirotor/fixed-wing launch 和 dt 行为保持不变。VTOL Node 以 100Hz 发布，multirotor backend 按 100Hz 更新，fixed-wing backend 按 50Hz 更新，未到周期只允许在一个 backend 周期和 freshness 限制内复用结果。

## 10. Phase 2~9 文件级计划

### Phase 2：公共类型和 baseline 修复

- 修改 `include/xd_uav_controller/control_types.h`：补全 resolver 输入/结果、capability 和 action phase/result 类型。
- 修改 `msg/ControlState.msg`、`msg/ControlCommand.msg`：保留全部当前字段并锁定常量。
- 修改 `srv/InternalCommand.srv`：加入 action/phase/generation，保留旧字段。
- 修改两个包的 CMake/test：增加公共协议单测。
- 修复两个既有 baseline test 的确定性配置/时序，不改变控制数学。

### Phase 3：UnifiedReference 和 ReferenceAdapter

- 新增 `include/xd_uav_controller/unified_reference.h`。
- 新增公共 reference capability/validation 单元。
- 抽离 PositionTarget、trajectory、path、simple-goal 和 internal-source metadata；TF Buffer 的所有权仍在 Node，Node 查询并校验刚体变换后把变换值传给纯 adapter，adapter 不自行访问 ROS TF 服务。
- 为矩阵中每类输入增加真实行为回归。

### Phase 4：ControllerBackend

- 新增 `controller_backend.h`、`backend_resolver.h` 及测试。
- 新增 multirotor/fixed-wing backend 源文件；逐段移动现有算法，不改变公式、参数默认值或 dt 来源。
- 使用 golden ControlState/reference/time 序列比较 body-rate、thrust、validity 和 rejection。
- 增加双向 handover 和 source-backend transition 测试。

### Phase 5：VehicleAdapter

- 新增 `vehicle_adapter.h`、`multirotor_vehicle_adapter.*`、`fixedwing_vehicle_adapter.*`。
- 将机架能力、takeoff/landing 准入和 phase 所有权迁入 adapter。
- 保持 manager 公共 OFFBOARD/MAVROS safety 逻辑。

### Phase 6：完善 VtolVehicleAdapter

- 扩展现有 `vtol_vehicle_adapter.*`：capability、observed/requested/result、unsolicited transition、dropout recovery、timeout fallback。
- 参数化 MAVROS service client 保留在 manager 公共编排层；调用结果以明确的 transport/ACK/result 值传给 adapter，adapter 单测不依赖真实 PX4。
- 扩展 diagnostics 与一次性状态变化日志。

### Phase 7：VTOL backend 复用

- Controller 只由 BackendResolver 选择 backend。
- transition 期间缓存只适用于目标 backend 的新 reference，源 backend 保持最后一条新鲜兼容 reference。
- 确认稳定形态后执行 deactivate/reset/activate，并拒绝旧 regime generation。

### Phase 8：VTOL 起降

- Manager 增加独立 landing sub-phase。
- VTOL HOVER takeoff 复用 multirotor reference generator。
- FW landing 请求先转 MC，确认后垂直下降。
- 增加 VTOL manager/controller rostest 和 SITL launch 支持；SITL 是否真正飞通由本机可用 PX4/Gazebo 环境决定，不能用单元测试替代飞行验收。

### Phase 9：deprecated 收敛

- 删除生产运行路径对原始 `vehicle_type_` 字符串的判断。
- 保留参数 fallback、消息字段和明确 deprecation warning。
- 更新 README 和 launch 示例，保留 legacy compatibility 测试。

## 11. 验证策略

每项生产行为执行 RED→GREEN→REFACTOR：

1. 先写能捕获具体错误的测试；
2. 运行并确认因缺失行为失败；
3. 写最小实现；
4. 运行目标测试和两个包全量测试；
5. 数学搬迁额外运行 golden comparison。

最终验证至少包括：

```text
isolated catkin build
all controller gtest/rostest
all manager gtest/rostest
catkin_test_results --all
launch XML validation
deprecated parameter compatibility
multirotor and fixed-wing golden cases
VTOL request/idempotence/failure/timeout/dropout/handover/landing tests
git diff scope audit
```

SITL 验收单独报告环境、机型、PX4 参数、launch 命令和 MC→FW→MC→land 结果。没有真实运行证据时不会声称 SITL 完成。

## 12. 范围和非目标

本轮不修改：

```text
xd_uav_controller
xd_uav_control_manager
xd_uav_task_allocate
xd_uav_detect
xd_uav_track
xd_uav_state_estimators
tmux_start
```

`xd_uav_state_estimators` 仅作为隔离构建依赖，不产生改动。

不实现自动任务距离选 regime、自定义气动 transition、倾转舵机直控、ArduPilot、ROS 2、pluginlib 或 VTOL runway landing。

## 13. 本轮实施结果

审计后已在两个 `_new` 包内完成以下后端扩展，未修改生产 YAML：

- 公共类型及线上数值测试，并增加集中、可单测的 `BackendResolver`；
- 保留全部真实字段和 mask/source 元数据的 `UnifiedReference`；
- `ControllerBackend` 接口及 multirotor/fixed-wing 两个运行时 backend，
  切换时执行 `onDeactivate -> onActivate(reset)`；
- `VehicleAdapter` capability 接口及 multirotor/fixed-wing/VTOL 实现；
- `InternalCommand` 的 action、phase、generation、return-home 协议，controller
  拒绝旧代次或非法 action/phase 组合；
- manager 对 controller 输出的 airframe、regime、regime generation、action
  generation 和 active backend 进行统一合同校验；
- VTOL transition transport failure、ACK reject、accepted、completed、timeout
  分层结果，原始 ACK 和诊断信息不丢失；
- VTOL 降落子阶段：FW 收到 land 时只请求 MC，PX4 确认 HOVER 且 controller
  回报 multirotor backend 已接管后，才允许启动垂直降落 reference；
- 全部 reference 继续走原数值路径，同时回报准确的 `reference_type` 和
  `reference_source`；simple goal 在本地直接完成适配，消除 latched topic
  旧消息竞态。

隔离 Catkin 工作区已完成两个包的编译及 21 项新增/扩展 C++ 单元测试。
完整 rostest 的执行需要允许 ROS 读取网络接口并启动本地 roscore；本轮环境的
提权审批服务返回 404，因此不能把 rostest 或 PX4/Gazebo SITL 标记为已完成。
