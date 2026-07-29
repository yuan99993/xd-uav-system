# SEAD → xd_uav_sead ROS 功能包 — 任务清单

> 最后更新：2026-07-29
> [交接文档](HANDOFF.md) | [快速入门](SEAD_QUICKSTART.md) | [Phase4报告](PHASE4_TEST_REPORT.md)

---

## 总体进度: ~92%

```
██████████████████████████████████████████████

✅ Phase 1 ─ 基础移植       ████████████████████  100%
✅ Phase 2 ─ 双模通信       ████████████████████  100%
✅ Phase 3 ─ 包结构优化     ████████████████████  100%
✅ Phase 4 ─ 仿真验证       ██████████████████░░   90%  (OFFBOARD 切模式成功, waypoint 轨迹待验证)
⬜ Phase 5 ─ XBee 硬件实测  ░░░░░░░░░░░░░░░░░░░░    0%
```

---

## Phase 4 仿真验证进展

| # | 验证项 | 状态 |
|---|--------|------|
| 4.1 | rosbridge 消息闭环测试 (16/16 pass) | ✅ |
| 4.2 | sead_onboard launch 启动 + rosbridge 话题就绪 | ✅ |
| 4.3 | mock_gcs → command → telemetry 收发 | ✅ |
| 4.4 | 5 个 launch 文件 `use_simulation:=true` 补齐 | ✅ |
| 4.5 | MRS one_drone tmux 仿真环境集成 | ✅ |
| 4.6 | MRS Gazebo + PX4 SITL 下 sead_onboard 启动成功 | ✅ |
| 4.7 | info 命令 → telemetry 回传 base64 确认 | ✅ |
| 4.8 | freq 命令 → U2G 频率设置 | ✅ |
| 4.9 | airspace_clear 命令 → 清空空域管理器 | ✅ |
| 4.10 | sead_gazebo_demo.launch 一键启动 | ✅ |
| 4.11 | **drone.py frame_type 自动识别修复 (Quad/Fixed_wing)** | ✅ |
| 4.12 | **Quad 机型 OFFBOARD 切换成功** | ✅ |
| 4.13 | SimpleStrike 端到端任务（3目标） | ⬜ |
| 4.14 | DPGA 完整 SEAD 任务测试 | ⬜ |
| 4.15 | waypoint 实际飞行轨迹验证 | ⬜ |
| 4.16 | 多机 u2u 通信验证 | ⬜ |

---

## 新增修复 (本轮 2026-07-29 — 仿真实测)

| # | 文件 | 问题 | 修复 |
|---|------|------|------|
| **P0** | [drone.py](src/xd_uav_sead/src/xd_uav_sead/drone/drone.py) L75 | `frame_type` 初始值硬编码 `Fixed_wing`，fallback 也是 `Fixed_wing` → SITL 中 classifier 读不到 param 时永远走固定翼逻辑 | 初始值 `None`，fallback 改为 `Quad`，重试次数 20→5 |
| **P0** | [drone.py](src/xd_uav_sead/src/xd_uav_sead/drone/drone.py) L597 | `get_param` 返回 `False` 时 `if servo_1 is not None` 误判为 True → `False.integer` → AttributeError → 20次重试后 fallback Fixed_wing | 改为 `if servo_1 is not None and servo_1 is not False` |
| **P0** | [drone.py](src/xd_uav_sead/src/xd_uav_sead/drone/drone.py) L279 | `set_mode()` 只有 `Fixed_wing` 分支，Quad 完全无法切换模式 | 新增完整 `Quad` 分支：OFFBOARD/LAND/LOITER/TAKEOFF，OFFBOARD 前预热 setpoint 流 |

验证链路（2026-07-29）:
```
[classifier] SERVO1_FUNCTION = False
[classifier] param read failed, => Quad
[Drone] frame_type = FrameType.Quad
Quad: Switching to OFFBOARD...
Quad: OFFBOARD success.
PX4 mode: OFFBOARD ✅
```

### 历史修复汇总

- [x] **P0**: 运行时模式参数 `os.environ.get()` → `rospy.get_param()`
- [x] **P0**: `mock_gcs.py` 加入 `CMakeLists.txt` catkin_install_python
- [x] **P0**: `unpack_packet` 对 info 消息返回 None → 元组解包闪退
- [x] **P0**: 4 个副 launch 文件缺少 `use_simulation:=true`
- [x] **P1**: wildcard imports → 显式 import
- [x] **P1**: drone.py 重复 import 清理
- [x] **P1**: 双 `except Exception` 块合并 + 6处裸 except 修正
- [x] rosbridge info 格式修复, SEAD_mission 空 target bug
- [x] `while not self.frame_type` 无限忙等、`set_stream_rate` 无超时
- [x] `__main__` sys.argv → rospy.get_param, 硬编码 launch 包修复
- [x] XBee_Devices 64-bit 地址恢复

---

## 移植策略（重要！）

**保留原有 XBee 硬件功能 + 新增 ROS 仿真通路。** 双模设计，不是替代：

| 模式 | 激活条件 | 通信后端 |
|------|---------|---------|
| 硬件模式 | digi.xbee SDK 已安装 + `~use_simulation:=false` | 真实 XBee S3B 900MHz |
| 仿真模式 | `~use_simulation:=true`（默认，无SDK时自动启用） | SeadRosBridge → ROS 话题 |

---

## Phase 1 ─ 基础移植 ✅ 100%

| 原始文件 | 移植位置 | 行数 | 状态 |
|---------|---------|------|------|
| `drone.py` (635L) | drone/drone.py | 654* | ✅ |
| `communication_info.py` (865L) | comms/communication_info.py | 865 | ✅ |
| `DPGA.py` (1134L) | planning/DPGA.py | 1134 | ✅ |
| `GA_SEAD_process.py` (1540L) | planning/GA_SEAD_process.py | 1540 | ✅ |
| `pathFollowing.py` (388L) | planning/pathFollowing.py | 388 | ✅ |
| `airspace_manager.py` (82L) | airspace/airspace_manager.py | 82 | ✅ |
| `formation_control.py` (2562L) | formation/formation_control.py | 2562 | ✅ |
| `simple_strike.py` (3842L) | strike/simple_strike.py | 3842 | ✅ |
| `onboard.py` (1805L) | scripts/sead_onboard_node.py | 1872* | ✅ |

> *drone.py +67行：mavros 接口重构 + Quad 分支。onboard.py +67行：双模初始化 + rospy 集成。

---

## Phase 2 ─ 双模通信 ✅ 95%

- [x] XBee 硬件路径完整保留 (try/except, 64-bit 地址, find_xbee_by_id)
- [x] SeadRosBridge (374行, 14种消息类型)
- [x] `/uavX/sead/command|telemetry|u2u` 话题
- [x] 双模切换运行时验证
- [ ] XBee S3B 硬件实测

---

## Phase 3 ─ 包结构优化 ✅ 100%

5 个 launch 全部 `roslaunch --ros-args` 通过 + `use_simulation:=true` 补齐。

---

## Phase 4 ─ PX4 SITL + 仿真验证 ✅ 90%

已验证: info, freq, airspace_clear, **OFFBOARD 切模式**。
待验证: waypoint 轨迹, SimpleStrike, DPGA, 多机 u2u。

---

## Phase 5 ─ XBee 硬件实测 ⬜ 0%

需 XBee S3B 硬件。
