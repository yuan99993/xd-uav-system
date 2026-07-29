# SEAD → xd_uav_sead ROS 功能包 — 任务清单

> 最后更新：2026-07-29
> [交接文档](HANDOFF.md) | [快速入门](SEAD_QUICKSTART.md) | [完善计划](IMPROVEMENT_PLAN.md) | [Phase4计划](PHASE4_PLAN.md)

---

## 总体进度: ~75%

```
██████████████████████████████████████

✅ Phase 1 ─ 基础移植       ████████████████████  100%
✅ Phase 2 ─ 双模通信       ████████████████████  100%
✅ Phase 3 ─ 包结构优化     ████████████████████  100%  (P0 bug已修复，测试通过)
✅ Phase 4 ─ 仿真验证       ██████░░░░░░░░░░░░░░   30%  (ROS层验证完成，PX4 SITL未跑)
⬜ Phase 5 ─ XBee 硬件实测  ░░░░░░░░░░░░░░░░░░░░    0%
```

### Phase 4 仿真验证进展

| # | 验证项 | 状态 |
|---|--------|------|
| 4.1 | rosbridge 消息闭环测试 (16/16 pass) | ✅ |
| 4.2 | rosbridge info 消息格式修复 | ✅ |
| 4.3 | sead_onboard launch 启动 + rosbridge 话题就绪 | ✅ |
| 4.4 | mock_gcs → command → telemetry 收发 | ✅ |
| 4.5 | launch 加 `use_simulation:=true` 避免 XBee 硬件崩溃 | ✅ |
| 4.6 | PX4 SITL + Gazebo 端到端 (sitl_gazebo CMake 不兼容暂跳过) | ⬜ |

### 本轮修复记录 (2026-07-29)

- [x] **P0**: 运行时模式参数 `os.environ.get()` → `rospy.get_param()`，launch 文件 param 设置现在生效
- [x] **P0**: `mock_gcs.py` 加入 `CMakeLists.txt` catkin_install_python
- [x] **P0**: `sead_onboard.launch` 加 `<param name="use_simulation" value="true"/>`
- [x] **P0**: rosbridge info 消息格式修复（pack 缺少 len 前缀）
- [x] **P1**: `sead_onboard_node.py` wildcard imports → 显式 import
- [x] **P1**: `drone.py` 重复 import 清理 (SetMode×3, CommandBool×2 → 1处集中)
- [x] **P1**: `drone.py` `from pathFollowing import *` → `import pathFollowing as pf`
- [x] **P1**: 双 `except Exception` 块合并为一个
- [x] **P1**: 裸 `except:` 全部改为 `except Exception:`（6处）
- [x] 二进制协议对照：rosbridge._serialize_info 与 communication_info.unpack_packet 逐字段一致
- [x] 5 个 launch 文件全部通过 roslaunch --ros-args
- [x] catkin_make 全量编译通过
- [x] rosbridge 16 项单元测试全部通过
- [x] sead_onboard + rosbridge + mock_gcs 端到端收发验证

---

## 移植策略（重要！）

**保留原有 XBee 硬件功能 + 新增 ROS 仿真通路。** 这是双模设计，不是替代：

| 模式 | 激活条件 | 通信后端 |
|------|---------|---------|
| 硬件模式 | digi.xbee SDK 已安装 + `~use_simulation:=false` | 真实 XBee S3B 900MHz |
| 仿真模式 | `~use_simulation:=true`（默认，无SDK时自动启用） | SeadRosBridge → ROS 话题 |

### 本轮修复记录 (2026-07-28)

- [x] `digi.xbee` import → try/except（SDK不存在时不崩）
- [x] `XBee_Devices` 恢复原始 64-bit 地址
- [x] `find_xbee_by_id()` 完整保留
- [x] 双模初始化逻辑（L391-410 of sead_onboard_node.py）
- [x] `fcntl` import 恢复
- [x] drone.py `while not self.frame_type` → 加限次重试
- [x] drone.py `set_stream_rate` → 加timeout
- [x] drone.py `__main__` sys.argv → rospy.get_param
- [x] rosbridge SEAD_mission 空target格式字符串bug修复
- [x] sead_onboard 硬编码 launch 包 → rospy param
- [x] mock_gcs 补全 12 种命令
- [x] 裸字符串/无用 import 清理
- [x] config 拆分: `gps_origin.yaml`, `sead_planner.yaml`

---

## Phase 1 ─ 基础移植 ✅ 100%

| # | 检查项 | 状态 |
|---|---|---|
| 1.1 | `package.xml` — format="2" | ✅ |
| 1.2 | `CMakeLists.txt` — catkin 纯 Python 包 | ✅ |
| 1.3 | `README.md` — 中文文档 | ✅ |
| 1.4 | `config/sead_defaults.yaml` | ✅ |
| 1.5 | 4 个 launch 文件 | ✅ |
| 1.6 | 9 个源文件: import改写 `classes.xxx` → `xd_uav_sead.xxx` | ✅ |
| 1.7 | `sead_onboard_node.py` — 节点入口 | ✅ |
| 1.8 | `sys.argv` → `rospy.get_param("~uav_name")` | ✅ |
| 1.9 | onboard.py: 12/12 函数完整保留 | ✅ |
| 1.10 | devel 软链接 | ✅ |
| 1.11 | 行尾符修复 (CRLF→LF, BOM清除) | ✅ |
| 1.12 | Python 语法检查: 10/10 通过 | ✅ |
| 1.13 | `catkin_make --pkg xd_uav_sead` | ✅ |
| 1.14 | `rospack find` + `roslaunch --ros-args` | ✅ |
| 1.15 | 原始 `src/SEAD/` **未被改动** | ✅ |

### 库模块逐文件验证

| 原始文件 | 移植位置 | 函数/类数 | 行数 | 状态 |
|---------|---------|-----------|------|------|
| `drone.py` (635L) | [drone/](src/xd_uav_sead/src/xd_uav_sead/drone/) | 24→24 | 644 | ✅ |
| `communication_info.py` (865L) | [comms/](src/xd_uav_sead/src/xd_uav_sead/comms/) | 28→28 | 865 | ✅ |
| `DPGA.py` (1134L) | [planning/](src/xd_uav_sead/src/xd_uav_sead/planning/) | 20→20 | 1134 | ✅ |
| `GA_SEAD_process.py` (1540L) | [planning/](src/xd_uav_sead/src/xd_uav_sead/planning/) | 28→36* | 1540 | ✅ |
| `pathFollowing.py` (388L) | [planning/](src/xd_uav_sead/src/xd_uav_sead/planning/) | 12→12 | 388 | ✅ |
| `airspace_manager.py` (82L) | [airspace/](src/xd_uav_sead/src/xd_uav_sead/airspace/) | 10→10 | 82 | ✅ |
| `formation_control.py` (2562L) | [formation/](src/xd_uav_sead/src/xd_uav_sead/formation/) | 70→72 | 2562 | ✅ |
| `simple_strike.py` (3842L) | [strike/](src/xd_uav_sead/src/xd_uav_sead/strike/) | 78→81 | 3842 | ✅ |
| `onboard.py` (1805L) | [scripts/](src/xd_uav_sead/scripts/) | 13→13 | 1827 | ✅ |

> *GA_SEAD_process.py 函数数差异是因为AST解析方式不同（嵌套def算进去了），实际内容一致。

---

## Phase 2 ─ 双模通信（XBee + ROS 仿真桥）⚠️ 70%

### 2.1 XBee 硬件路径
- [x] `digi.xbee` import → try/except
- [x] `XBee_Devices` 保持原始 64-bit 地址
- [x] `find_xbee_by_id()` 完整保留
- [ ] 硬件模式端到端测试

### 2.2 仿真路径
- [x] `SeadRosBridge` 类（[rosbridge.py](src/xd_uav_sead/src/xd_uav_sead/comms/rosbridge.py)）
- [x] `/uavX/sead/command` Subscriber
- [x] `/uavX/sead/telemetry` Publisher
- [x] `/uavX/sead/u2u` Pub/Sub
- [x] 14 种消息类型 `_serialize_info`
- [ ] 仿真模式端到端测试

### 2.3 双模切换
- [x] `~use_simulation` param（默认值跟随 `XBEE_HW_AVAILABLE`）
- [x] 仿真模式: `xbee = comms` (SeadRosBridge实例)
- [x] 硬件模式: `xbee = DigiMeshDevice` (真实XBee)
- [ ] 切换逻辑运行时验证

### 2.4 Mock GCS
- [x] 12 种命令: takeoff, arm, disarm, mode, waypoint, freq, mission_abort, origin, sead_mission, task_insert, airspace_zone, airspace_clear, swarm, formation
- [ ] 多步骤自动化脚本

---

## Phase 3 ─ 包结构优化 ⚠️ 30%

### 3.1 目录结构 ✅
```
src/xd_uav_sead/
├── drone/        drone.py
├── planning/     DPGA.py, GA_SEAD_process.py, pathFollowing.py
├── comms/        communication_info.py, rosbridge.py
├── formation/    formation_control.py
├── strike/       simple_strike.py
├── airspace/     airspace_manager.py
├── config/       sead_defaults.yaml, gps_origin.yaml, sead_planner.yaml
├── launch/       5 个 launch 文件
└── scripts/      sead_onboard_node.py, mock_gcs.py
```

### 3.2 配置拆分 ✅
- [x] `sead_defaults.yaml` — 控制模式
- [x] `gps_origin.yaml` — GPS 原点
- [x] `sead_planner.yaml` — GA + Dubins 参数

### 3.3 Launch 文件 ⚠️ 5个都有但未验证
- [x] `sead_onboard.launch` — roslaunch --ros-args 通过
- [ ] `sead_formation_demo.launch`
- [ ] `sead_strike_demo.launch`
- [ ] `sead_gazebo_demo.launch`
- [ ] `sead_waypoint_demo.launch`

---

## Phase 4 ─ PX4 SITL + 仿真验证 ⬜ 0%

- [ ] 确认 PX4 SITL 或 Gazebo 仿真环境可用
- [ ] 确认 mavros 连接到仿真飞控
- [ ] `roslaunch xd_uav_sead sead_gazebo_demo.launch` 完整流程
- [ ] mock GCS → 起飞 → 航点 → SEAD任务 (simple_strike)
- [ ] 遥测数据正确发布到 `/uavX/sead/telemetry`

---

## Phase 5 ─ XBee 硬件实测 ⬜ 0%

- [ ] XBee S3B 插入 USB
- [ ] `~use_simulation:=false roslaunch xd_uav_sead sead_onboard.launch`
- [ ] `find_xbee_by_id()` 搜到设备
- [ ] 硬件 GCS → XBee → onboard 完整链路
- [ ] 多机 XBee 编队通信
