# ROS 功能包改写 — 任务清单

> 生成时间：2026-07-28
> 来源：将 `src/SEAD` 和 `src/PixEagle` 改写为 xd-uavsystem-test 风格的 ROS1 catkin 功能包
> 风格依据：`src/xd-uavsystem-test/CLAUDE.md`

---

## 总体阶段

```
Phase 0: 分析现有代码依赖 → 已完成（见下方分析结果）
Phase 1: SEAD → xd_uav_sead（ROS Python 包）
Phase 2: PixEagle → xd_uav_pixeagle_tracker（Tracker + Follower 提取）
Phase 3: 集成测试
```

---

## Phase 1: SEAD → xd_uav_sead

SEAD 已经是 ROS Python 代码，改造量最小。

### 1.1: 创建包骨架

- [ ] 建立 `src/xd-uavsystem-test/src/xd_uav_sead/` 目录结构
- [ ] 创建 `package.xml`（depend: rospy, mavros_msgs, geometry_msgs, nav_msgs, sensor_msgs, std_srvs）
- [ ] 创建 `CMakeLists.txt`（纯 Python 包模板）
- [ ] 创建 `README.md`

### 1.2: 迁移源码

库模块（纯 Python，无 ROS 依赖）→ `src/xd_uav_sead/src/xd_uav_sead/`：
- [ ] `drone.py` — Drone 类，依赖 mavros_msgs
- [ ] `DPGA.py` — 路径规划算法
- [ ] `GA_SEAD_process.py` — SEAD 任务流程
- [ ] `airspace_manager.py` — 空域管理
- [ ] `formation_control.py` — 编队控制
- [ ] `pathFollowing.py` — 路径跟随
- [ ] `simple_strike.py` — 打击任务
- [ ] `communication_info.py` — XBee 通信

ROS 节点脚本 → `scripts/`：
- [ ] `onboard.py` → `scripts/sead_onboard_node.py`（添加 `#!/usr/bin/env python3` + `rospy.init_node` 包装）
- [ ] `xbee_send_formation_test.py` → `scripts/xbee_formation_test.py`

### 1.3: 创建 launch 和 config

- [ ] `launch/sead_onboard.launch`（UAV_NAME 命名空间 + 参数加载）
- [ ] `launch/sead_formation.launch`
- [ ] `config/sead_defaults.yaml`

### 1.4: 验证

- [ ] `catkin build xd_uav_sead` 通过
- [ ] `roslaunch xd_uav_sead sead_onboard.launch` 无 import 报错

**预估工作量**：~30 分钟

---

## Phase 2: PixEagle Tracker + Follower → xd_uav_pixeagle_tracker

这是核心难题。PixEagle 使用 MAVSDK（不是 mavros），内部有 127 个 Python 文件互相引用。

### 2.1: 现状分析（已完成）

Tracker/Follower 需要的 classes 文件：

| 核心文件 | 行数 | 依赖数 | 备注 |
|---|---|---|---|
| `smart_tracker.py` | 1435 | 9 | 依赖 detection backends (ultralytics/ncnn) |
| `follower.py` | 516 | 4 | 工厂模式，动态注册 |
| `tracker_output.py` | 347 | - | 数据结构 |
| `tracking_state_manager.py` | 1315 | - | 状态管理 |
| `motion_predictor.py` | 268 | - | 卡尔曼预测 |
| `appearance_model.py` | 540 | - | 外观特征 |
| `geometry_utils.py` | 117 | - | 纯计算，无外部依赖 |
| `target_loss_handler.py` | 658 | - | 目标丢失处理 |
| `kalman_box_tracker.py` | 279 | - | 卡尔曼框跟踪 |
| `tracking_roi.py` | 178 | - | ROI 管理 |
| `tracker_runtime_status.py` | 440 | - | 运行时状态 |

Follower 实现（在 `classes/followers/` 下）：

| 文件 | 行数 | 说明 |
|---|---|---|
| `base_follower.py` | 1126 | 基类，依赖 SafetyManager/SchemaManager/CircuitBreaker |
| `mc_velocity_chase_follower.py` | 2014 | 多旋翼速度追逐 |
| `mc_velocity_distance_follower.py` | 620 | 距离保持 |
| `mc_velocity_ground_follower.py` | 642 | 地面跟随 |
| `mc_velocity_position_follower.py` | 739 | 位置跟随 |
| `mc_attitude_rate_follower.py` | 1009 | 姿态角速率控制 |
| `fw_attitude_rate_follower.py` | 1183 | 固定翼 |
| `gm_velocity_chase_follower.py` | 1656 | 云台追逐 |
| `gm_velocity_vector_follower.py` | 1013 | 云台矢量 |
| `custom_pid.py` | 47 | PID 实现 |
| `yaw_rate_smoother.py` | 131 | 偏航平滑 |

Tracker 实现（在 `classes/trackers/` 下）：

| 文件 | 行数 | 说明 |
|---|---|---|
| `base_tracker.py` | 1078 | 基类 |
| `csrt_tracker.py` | 581 | OpenCV CSRT |
| `kcf_kalman_tracker.py` | 378 | KCF + 卡尔曼 |
| `dlib_tracker.py` | 417 | dlib 相关 |
| `gimbal_tracker.py` | 1001 | 云台追踪 |
| `custom_tracker.py` | 68 | 自定义 |
| `tracker_factory.py` | 90 | 工厂 |

基础设施依赖（需要一起带过来的非 tracker 文件）：

| 文件 | 行数 | 角色 |
|---|---|---|
| `parameters.py` | 821 | 全局参数管理，YAML 加载 |
| `setpoint_handler.py` | 1051 | 指令 schema 解析/验 |
| `command_safety.py` | 157 | 指令安全校验 |
| `command_intent.py` | 25 | 指令意图数据类 |
| `safety_types.py` | 172 | 安全类型定义 |
| `px4_interface_manager.py` | ~2000+ | **MAVSDK 飞控接口 — 必须改写为 mavros** |
| `circuit_breaker.py` | ? | 熔断器 |
| `safety_manager.py` | ? | 安全管理器 |
| `schema_manager.py` | ? | Schema 管理器 |
| `follower_logger.py` | ? | 日志 |
| `follower_config_manager.py` | ? | Follower 配置 |
| `follower_types.py` | ? | Follower 类型 |
| `detection_adapter.py` | ? | 检测适配 |
| `backends/` | ? | 检测后端 (ultralytics/ncnn) |

总计需要搬运 **~20-30 个文件**，并且需要做：

### 2.2: 核心改造项

- [ ] 决策：MAVSDK → mavros 改写策略（两个选项）
  - 选项 A：整体搬迁 MAVSDK 代码，保持 `mavsdk.System` 通信，只用 ROS 包管理（改动最小）
  - 选项 B：把 `px4_interface_manager` 完整改写为 mavros 发布/订阅模式（真正 ROS 化，改动大）
- [ ] 确认需要哪些 follower（你说只需要 tracker 和 follower，具体要 mc_velocity_chase + mc_attitude_rate？还是全部？）
- [ ] 确认 Detection Backend（ultralytics？ ncnn？还是都保留？）

### 2.3: 实施步骤（待确认策略后细化）

- [ ] 创建 `xd_uav_pixeagle_tracker/` 包骨架
- [ ] 创建 `package.xml` + `CMakeLists.txt`
- [ ] 复制核心 tracker/follower 代码 → `src/xd_uav_pixeagle_tracker/`
- [ ] 改写 `px4_interface_manager`：mavsdk → mavros（如选选项 B）
- [ ] 抽象接口层：让 follower 通过 ROS topic 而不是 MAVSDK 下发控制指令
- [ ] 重写 `parameters.py`：YAML → ROS param server
- [ ] 创建 ROS 节点入口 `scripts/pixeagle_tracker_node.py`
- [ ] 创建 `launch/pixeagle_tracker.launch`
- [ ] 创建 `config/pixeagle_tracker.yaml`
- [ ] `catkin build xd_uav_pixeagle_tracker` 通过

**预估工作量**：取决于策略
- 选项 A（保留 MAVSDK）：~1-2 小时
- 选项 B（完整 mavros 改写）：~4-8 小时

---

## Phase 3: 集成

- [ ] SEAD 和 PixEagle 可以 `catkin build` 通过
- [ ] 两包的 launch 文件可以正常运行 roslaunch
- [ ] 文档更新：顶层 README 说明两包的用途和启动方式

---

## 关键待决策问题（需你确认后推进）

1. PixEagle 策略：选选项 A（保留 MAVSDK）还是选项 B（改写成 mavros）？
2. 需要哪些 follower 模式？（全部 9 种还是少数几种？）
3. Detection backend 需要哪些？（ultralytics / ncnn / 都要？）
4. 命名：`xd_uav_sead` + `xd_uav_pixeagle_tracker` 可以吗？
