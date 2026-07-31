# 交接文档 —— xd_uav_sead 移植项目

> 整体进度 ~92%。读完本文档 + CLAUDE.md + TASK_LIST.md 即可接手。

---

## 1. 这是什么项目

将 [src/SEAD/](/home/promise/catkin_ws/src/SEAD/) 移植为 [src/xd-uavsystem-test/src/xd_uav_sead/](src/xd_uav_sead/) —— ROS1 catkin Python 功能包。

**双模设计**：保留 XBee 硬件代码 + 新增 ROS 仿真通路 (SeadRosBridge)。通过 `~use_simulation` param 切换。

---

## 2. 当前位置 (~92%)

- Phase 1-3 完整。Phase 4 MRS Gazebo + PX4 SITL 闭环通过。
- **OFFBOARD 切模式验证成功**（Quad 分支已修复）。
- info / freq / airspace_clear / mode(GUIDED→OFFBOARD) 四个命令端到端可用。
- 5 个 launch 全部补齐 use_simulation。

### 本轮关键修复 (2026-07-29)

drone.py 三个 P0 问题全部修复：

| 问题 | 根因 | 修复 |
|------|------|------|
| SITL 永远判为 Fixed_wing | get_param 返回 False → `False.integer` AttributeError → 20次 fallback Fixed_wing | `is not None and is not False` + fallback 改 Quad |
| Quad 无法切模式 | set_mode 只有 Fixed_wing 分支 | 新增 Quad OFFBOARD/LAND/LOITER/TAKEOFF |
| OFFBOARD 被拒绝 | 切 OFFBOARD 前未预热 setpoint 流 | Quad::OFFBOARD 先行 20 帧 setpoint 预热 |

### 仿真环境

- 启动: `bash temp/start_sim.sh` (MRS one_drone tmux)
- 停止: `bash temp/kill_sim.sh`
- ⚠️ 不动 PX4/Gazebo/MRS 源文件，不动 `src/SEAD/`

---

## 3. 下一步

### P0: waypoint 实际飞行
```bash
bash temp/start_sim.sh
source devel/setup.bash
roslaunch xd_uav_sead sead_onboard.launch
# 切 OFFBOARD
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=mode _mode:=GUIDED
# 飞 waypoint
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=waypoint _x:=30 _y:=10 _z:=30
```
注意: `guide_to_waypoint` 做了 GPS→ENU 坐标变换，SITL 中 GPS origin 与 mavros 原点可能不一致导致飞机乱飞。如果位置异常，优先排查 `drone.py` L544-L556 的坐标变换。

### P1: SimpleStrike 端到端（3 目标）
### P2: DPGA 完整 SEAD 任务
### P3: XBee S3B 硬件实测

---

## 4. 注意事项

1. **不要删 XBee 代码。** 仿真通路是附加的。
2. **`communication_info.py` 不要动。** 协议层，两边共用。
3. **原始 `src/SEAD/` 没改过。**
4. **`drone.py` 的 classifier 依赖 `get_param("SERVO1_FUNCTION")`** — 真正硬件上返回 integer=4 (固定翼) 或其他 (多旋翼)，SITL 返回 False。
5. **会话结束后必须关仿真** (`bash temp/kill_sim.sh`)。
