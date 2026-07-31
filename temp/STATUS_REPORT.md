# SEAD → xd_uav_sead 移植状态报告

> 日期：2026-07-29
> 供：项目汇报

---

## 一、总览

将 `src/SEAD/` 下 9 个 Python 文件的多无人机 SEAD 控制系统，移植为 xd 风格的 ROS1 catkin Python 功能包 `xd_uav_sead`。

**当前进度：~92%。代码移植完整，MRS Gazebo + PX4 SITL 仿真下 OFFBOARD 切模式成功。**

---

## 二、已完成

### Phase 1 — 基础移植 ✅ 100%
- 9 源文件搬运，import 改写，catkin_make 编译通过
- 原始 `src/SEAD/` **零改动**

### Phase 2 — 双模通信 ✅ 100%
- XBee 硬件代码完整保留 + SeadRosBridge 仿真桥 (374行)
- `/uavX/sead/command|telemetry|u2u` 话题
- 通过 `~use_simulation` param 一键切换

### Phase 3 — 代码质量修复 ✅ 100%
- os.environ→rospy.get_param, wildcard import 清理, 裸 except 修正, 双except合并
- 5 个 launch 全部补齐 use_simulation

### Phase 4 — 仿真验证 ✅ 90%

| 验证项 | 状态 |
|--------|------|
| rosbridge 16项协议测试 | ✅ |
| sead_onboard + rosbridge 话题就绪 | ✅ |
| info / freq / airspace_clear 端到端收发 | ✅ |
| MRS Gazebo + PX4 SITL 闭环 | ✅ |
| **Quad frame_type 自动识别 + OFFBOARD 切模式** | ✅ **本轮** |
| SimpleStrike / DPGA 任务执行 | ⬜ |
| waypoint 飞行轨迹 | ⬜ |

---

## 三、本轮关键修复 (2026-07-29 仿真实测)

| # | 文件 | 问题 | 修复 |
|---|------|------|------|
| 1 | drone.py uav_classifier | SITL 读不到 SERVO1_FUNCTION → get_param 返回 False → `False.integer` AttributeError → 20次重试后永远 fallback Fixed_wing | `if servo_1 is not None and servo_1 is not False` |
| 2 | drone.py 初始化 | frame_type 初始值 + fallback 都硬编码 Fixed_wing | 初始 None，fallback Quad |
| 3 | drone.py set_mode | 完全缺失 Quad 分支，无法切 OFFBOARD | 新增 Quad 分支：OFFBOARD/LAND/LOITER/TAKEOFF |

验证结果：`frame_type = FrameType.Quad → OFFBOARD success`

---

## 四、尚未完成

1. **SimpleStrike / DPGA 完整任务测试** — 收发链路+OFFBOARD 已通，需要实际飞行验证
2. **XBee 硬件实测** — 需 XBee S3B 硬件

---

## 五、下一步

1. mock_gcs waypoint 命令 → 确认飞机在 Gazebo 中移动到目标点
2. SimpleStrike 端到端 (3 目标，无 GA)
3. DPGA 完整 SEAD 任务
4. XBee 硬件模式验证

---

## 六、文件清单

```
src/xd_uav_sead/
├── CMakeLists.txt, package.xml, README.md
├── config/     sead_defaults.yaml, gps_origin.yaml, sead_planner.yaml
├── launch/     5 个 launch 文件 (全部验证通过)
├── scripts/    sead_onboard_node.py, mock_gcs.py
└── src/xd_uav_sead/
    ├── drone/drone.py         ← Quad OFFBOARD 分支 (本轮修复)
    ├── comms/                 communication_info.py, rosbridge.py
    ├── planning/              DPGA.py, GA_SEAD_process.py, pathFollowing.py
    ├── formation/             formation_control.py
    ├── strike/                simple_strike.py
    └── airspace/              airspace_manager.py
```

**17 个 Python 文件，catkin_make 编译通过，MRS Gazebo + PX4 SITL OFFBOARD 验证通过。**
