# PixEagle Tracker + Follower → ROS 功能包可行性分析

> 分析日期：2026-07-28
> 分析范围：`src/PixEagle/src/classes/` 下 tracker 和 follower 的所有直接和传递依赖
> 分析方法：逐文件深读 20 个核心文件，追踪完整的 import 依赖链和 MAVSDK 调用路径

---

## 一、总体结论

**可以改成 ROS1 功能包。** 核心逻辑与 MAVSDK/rospy 完全解耦，只有一处 MAVSDK 调用需要改写为 mavros 话题发布。

---

## 二、架构分析

### 2.1 数据流全景

```
Camera Frame
    │
    ▼
┌──────────────────────┐
│ SmartTracker         │  ← DetectionBackend (YOLO/ncnn)
│  检测 + 目标跟踪      │  ← TrackingStateManager (ID 持久化)
│  输出: TrackerOutput  │  ← MotionPredictor + AppearanceModel (短时遮挡)
└────────┬─────────────┘
         │ TrackerOutput (dataclass)
         ▼
┌──────────────────────┐
│ ConcreteFollower     │  ← 如 MCVelocityChaseFollower
│  PID 控制数学         │  ← 输入: TrackerOutput + 无人机当前状态
│  输出: CommandIntent  │  ← 输出: {vel_body_fwd, vel_body_right, ...}
└────────┬─────────────┘
         │ CommandIntent (dataclass: profile_name, fields=Dict[str,float])
         ▼
┌──────────────────────┐
│ OffboardCommander    │  ← 心跳循环，定期取 CommandIntent
│  setpoint 生命周期    │
└────────┬─────────────┘
         │
         ▼
┌──────────────────────────────────────────────────┐
│ PX4InterfaceManager                               │
│ send_velocity_body_offboard()                     │
│   → self.drone.offboard.set_velocity_body(...) ◄── 唯一 MAVSDK 调用
└──────────────────────────────────────────────────┘
         │ (替换为)
         ▼
┌──────────────────────────────────────────────────┐
│ mavros ROS 接口                                   │
│ Publisher → /uavX/mavros/setpoint_velocity/cmd_vel│
│ Service  → /uavX/mavros/set_mode (OFFBOARD)       │
└──────────────────────────────────────────────────┘
```

### 2.2 关键发现

1. **20 个核心文件的 import 谱系中，0 个 import MAVSDK，0 个 import rospy。** 全部是纯 Python 数值计算和数据结构。
2. MAVSDK 调用仅存在于 `px4_interface_manager.py` 的 2 个 dispatch 函数中。
3. Follower 输出的 `CommandIntent` 是 SI 单位的字典（`vel_body_fwd: float` 等），可以直接映射到 `geometry_msgs/TwistStamped`。

---

## 三、文件清单

### 3.1 Tracker 侧（检测 + 跟踪，输出 TrackerOutput）

| 文件 | 行数 | 依赖层次 | 职责 |
|---|---|---|---|
| `geometry_utils.py` | 117 | 零依赖 | OBB 多边形转换、射线法点-in-polygon、AABB 裁剪 |
| `detection_adapter.py` | 45 | 零依赖 | `NormalizedDetection` dataclass + 格式转换 |
| `kalman_box_tracker.py` | 279 | 零依赖 | SORT 族 7 状态卡尔曼滤波器 |
| `motion_predictor.py` | 268 | 零依赖 | EMA 速度预测 + 加速度感知运动学外推 |
| `appearance_model.py` | 540 | 零依赖 | HSV 直方图 + HOG 特征提取，余弦相似度 ReID |
| `tracking_roi.py` | 178 | 零依赖 | 手动 ROI/Polygon 验证与坐标转换 |
| `tracker_output.py` | 347 | 可选依赖 schema_manager | `TrackerOutput` + `TrackerDataType` |
| `tracker_runtime_status.py` | 440 | 零依赖 | Target 可见性/可用性评估，超时检测 |
| `tracking_state_manager.py` | 1315 | 依赖 kalman_box_tracker | 4 层匹配状态机 |
| `smart_tracker.py` | 1435 | 依赖上述全部 + DetectionBackend + Parameters | 主检测-跟踪管线 |

### 3.2 Follower 侧（控制指令生成，输出 CommandIntent）

| 文件 | 行数 | 依赖层次 | 职责 |
|---|---|---|---|
| `command_intent.py` | 25 | 零依赖 | `CommandIntent` 冻结 dataclass |
| `safety_types.py` | 172 | 零依赖 | 速度/高度/速率限制 + 安全行为枚举 |
| `command_safety.py` | 157 | 依赖 safety_types + Parameters(懒加载) | 指令值验证限幅 |
| `setpoint_handler.py` | 1052 | 依赖 command_safety, command_intent, safety_types | YAML schema 加载 + 字段验证 + CommandIntent 生产 |
| `followers/base_follower.py` | 1126 | 依赖 safety_manager, schema_manager, circuit_breaker, follower_logger, setpoint_handler | 所有 follower 的公共基础设施 |
| `follower.py` | 516 | 依赖 Parameters, SetpointHandler, 具体 follower | 工厂 + 管理封装 |
| 具体 follower 实现 | 500-2000 各 | 依赖 base_follower | PID 控制器，每种模式一个类 |

### 3.3 需要新增的 ROS 接口层（替代 px4_interface_manager.py）

| 替代内容 | ROS 接口 |
|---|---|
| `drone.offboard.set_velocity_body(VelocityBodyYawspeed(...))` | `rospy.Publisher` → `/uavX/mavros/setpoint_velocity/cmd_vel` (TwistStamped) |
| `drone.offboard.set_attitude_rate(AttitudeRate(...))` | `rospy.Publisher` → `/uavX/mavros/setpoint_raw/attitude` (AttitudeTarget) |
| `start_offboard_mode()` / `stop_offboard_mode()` | `rospy.ServiceProxy` → `/uavX/mavros/set_mode` (SetMode) |
| 无人机状态订阅（位置、姿态、速度） | `rospy.Subscriber` → `/uavX/mavros/local_position/odom` + `/uavX/mavros/imu/data` |

---

## 四、Python 版本兼容性

### 4.1 语法兼容性

| 语法特性 | 最低 Python | 影响文件 | 修复 |
|---|---|---|---|
| `X \| Y` (PEP 604, 无 `from __future__`) | 3.10 | **仅 1 处**: `setpoint_handler.py` 第 507 行 `str \| Path` | 改为 `Union[str, Path]` |
| `from __future__ import annotations` + `X \| Y` | 3.8 (惰性求值) | 4 个文件 | 无需修改 |
| `match`/`case` | 3.10 | 无 | — |
| `ParamSpec` | 3.10 | `api_execution.py`（非 tracker/follower 文件） | — |

**结论：改 1 行代码即可在 Python 3.8 上运行。**

### 4.2 当前环境

| 项目 | 值 |
|---|---|
| ROS Python | 3.8.10（系统） |
| PixEagle venv Python | 3.10.14（仅 PixEagle 目录下） |
| 迁移后 | 统一用系统 Python 3.8.10 |

---

## 五、依赖处理

| 依赖 | 当前来源 | 迁移方案 |
|---|---|---|
| `numpy` | apt (python3-numpy) | 不动 |
| `scipy` | pip `~/.local` → 改 apt | `apt install python3-scipy` |
| `matplotlib` | apt (python3-matplotlib) | 不动 |
| `dubins` | 无（SEAD 专用） | — |
| `pymap3d` | pip `~/.local` | 保持（无 apt 包） |
| `opencv-contrib-python-headless` | PixEagle pip | `pip install --user` |
| `mavsdk` | PixEagle pip | **不再需要，改用 mavros** |

---

## 六、文件搬运清单（预估 19-20 个文件）

```
src/xd_uav_pixeagle_tracker/
├── package.xml
├── CMakeLists.txt
├── README.md
├── config/
│   └── pixeagle_defaults.yaml
├── launch/
│   └── pixeagle_tracker.launch
├── scripts/
│   └── pixeagle_tracker_node.py          # ROS 节点入口 + 状态订阅 + publisher
└── src/xd_uav_pixeagle_tracker/
    ├── __init__.py
    ├── tracker/
    │   ├── smart_tracker.py              # 主检测-跟踪管线
    │   ├── tracker_output.py             # 输出数据结构
    │   ├── tracker_runtime_status.py     # 目标可用性评估
    │   ├── tracking_state_manager.py     # ID 持久化状态机
    │   ├── tracking_roi.py               # ROI 管理
    │   ├── kalman_box_tracker.py         # 卡尔曼滤波器
    │   ├── motion_predictor.py           # 运动预测
    │   ├── appearance_model.py           # 外观 ReID
    │   ├── geometry_utils.py             # 几何工具
    │   └── detection_adapter.py          # 检测结果适配
    ├── follower/
    │   ├── follower.py                   # 工厂
    │   ├── base_follower.py             # 基类
    │   ├── mc_velocity_chase_follower.py # 多旋翼速度追逐
    │   ├── mc_velocity_distance_follower.py
    │   ├── mc_velocity_ground_follower.py
    │   ├── mc_velocity_position_follower.py
    │   ├── mc_attitude_rate_follower.py
    │   ├── custom_pid.py                 # PID 实现
    │   ├── yaw_rate_smoother.py          # 偏航平滑
    │   └── (其他 follower 按需)
    ├── command/
    │   ├── command_intent.py             # CommandIntent dataclass
    │   ├── command_safety.py             # 安全校验
    │   ├── safety_types.py               # 限制类型定义
    │   └── setpoint_handler.py           # Schema 加载 + 字段验证
    ├── px4_ros_interface.py              # ★ 新写：MAVSDK → mavros 的桥接层
    └── parameters.py                     # YAML 配置加载（后续可迁移到 ROS param）
```

---

## 七、改写步骤（按优先级）

### MVP：import 能过，包能编译

1. [ ] 创建包骨架（package.xml + CMakeLists.txt + __init__.py）
2. [ ] 复制 19-20 个文件
3. [ ] 全局替换 `from classes.xxx` → `from xd_uav_pixeagle_tracker.xxx`
4. [ ] 修复 `setpoint_handler.py:507`：`str | Path` → `Union[str, Path]`
5. [ ] 修复 BOM/CRLF 编码问题（如果有）
6. [ ] `catkin_make` 验证通过 + Python 语法检查通过

### 第二阶段：MAVSDK → mavros（真正的 ROS 化）

7. [ ] 新写 `px4_ros_interface.py`：接收 CommandIntent → 发布 mavros 话题
8. [ ] 去掉 `mavsdk` 依赖
9. [ ] 写 ROS 节点入口：订阅 `/uavX/mavros/local_position/odom` + 调用 tracker + 调用 follower + 发布 setpoint
10. [ ] launch + config 文件

### 第三阶段：配置迁移（可选）

11. [ ] `parameters.py` 的 YAML 加载 → ROS param server
12. [ ] `follower_commands.yaml` → ROS param

---

## 八、待确认问题

1. **需要哪些具体的 follower 模式？** 全部 8 种还是只保留多旋翼的 3-4 种？
2. **Detection backend 需要哪些？** ultralytics？ncnn？两者都要？
3. **`parameters.py` 的 YAML 配置文件在哪里？** 改写成 ROS 包时需要把原始 config YAML 带过来。
4. **包名**：`xd_uav_pixeagle_tracker` 可以吗？
