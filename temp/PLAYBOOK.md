# xd_uav_sead 移植实操手册 (Playbook)

> 完成日期: 2026-07-29 | 总体进度: ~79%
> 每次接手先读: CLAUDE.md → temp/HANDOFF.md → 本文档

---

## 1. 环境速查

| 项目 | 值 |
|------|-----|
| 工作空间 | `/home/promise/catkin_ws` (catkin_make) |
| 元仓库 | `/home/promise/catkin_ws/src/xd-uavsystem-test` |
| ROS 包路径 | `src/xd_uav_sead/` |
| 原始 SEAD | `/home/promise/catkin_ws/src/SEAD/` (零改动) |
| Python | `/usr/bin/python3` (3.8.10) — 裸 `python3`=pyenv 3.10 ❌ |
| 仿真 | MRS one_drone tmux → `/home/promise/catkin_ws/src/mrs_uav_gazebo_simulator/tmux/one_drone/start.sh` |
| PX4 | `/home/promise/PX4-Autopilot/` (零改动，bin 已编好) |
| mavros 话题 | `/uav1/mavros/*` (state, imu, odom, gps, battery, setpoint_raw/local, ...) |

## 2. 启动仿真

```bash
# 启动 (后台等就绪)
bash temp/start_sim.sh

# 停止
bash temp/kill_sim.sh
# 或者: tmux -L mrs kill-session -t simulation
```

**start_sim.sh 等待条件**: `/uav1/*` 话题出现即认为就绪。

**注意**: MRS 自动执行了 takeoff → 无人机启动时已 armed + OFFBOARD + 在空中 (z≈4.6m)。

## 3. 编译 + 启动 SEAD

```bash
source devel/setup.bash
catkin_make
roslaunch xd_uav_sead sead_onboard.launch
```

**5 个 launch 文件均可用; 都已加 `use_simulation:=true`。**

## 4. mock_gcs 测试命令

```bash
# 基本测试
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=info _text:="hello"
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=freq _freq:=2.0
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_clear

# takeoff (无人机已浮空则跳过)
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=takeoff _alt:=80

# waypoint (无人机高度>5m 时直入航点跟随)
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=waypoint _x:=50 _y:=0 _z:=80

# SEAD 任务 (3 目标 SimpleStrike)
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=sead_mission \
  _targets_json:='[[100,0],[200,50],[300,100]]'
```

**所有 12 种命令均可用**: takeoff, arm, disarm, mode, waypoint, freq, mission_abort, origin, sead_mission, task_insert, airspace_zone, airspace_clear, swarm, formation

## 5. 已验证 vs 未验证

### ✅ 已验证

| 项目 | 状态 |
|------|------|
| Python 语法 17/17 | 全部通过 |
| catkin_make 编译 | 通过 |
| roslaunch --ros-args 5/5 | 通过 |
| rosbridge 16项协议闭环测试 | 16/16 pass |
| sead_onboard 在 PX4 SITL 下启动 | 不崩溃 |
| rosbridge info 收发 | 正确 |
| rosbridge freq 收发 | 正确 (触发 formation 子进程) |
| rosbridge airspace_clear 收发 | 正确 |
| 5 个 launch use_simulation | 全部生效 |
| start_roslaunch → formation demo | 子进程存活 + 退出时清理 |

### ⚠ 已修复但未重新测试

| 项目 | 修复内容 |
|------|----------|
| waypoint 高度>5m 跳过 takeoff | `local_pose[2] > 5.0` 判断 (原始代码每次都走起飞) |

### ⬜ 未验证

| 项目 | 说明 |
|------|------|
| waypoint 实际飞行 | 命令层 OK，需要看 Gazebo 里无人机是否移动到目标坐标 |
| SEAD_mission 任务下发 | SimpleStrike / DPGA 未跑 |
| 多机 u2u | 单机环境无法测试 |
| XBee S3B 硬件 | 需要真实硬件 |
| GPS 坐标转换 | home/origin 坐标系设置需确认 |

## 6. 已知 Bug 和坑

### 1. digi.xbee SDK 已安装
- `XBEE_HW_AVAILABLE = True` — 不加 `use_simulation:=true` 就走 XBee 路径 → 崩溃
- **所有 launch 文件必须带 `use_simulation:=true`** (已全部补齐)

### 2. pyenv python3 vs /usr/bin/python3
- 裸 `python3` = `/home/promise/.pyenv/versions/3.10.14/bin/python3`
- ROS (catkin_make) = `/usr/bin/python3` (3.8.10)
- `pip3 install` = pyenv (装完 ROS 找不到)
- **始终用 `/usr/bin/python3`**

### 3. pgrep -f "sead" 的陷阱
- `pgrep -f "sead"` 会匹配到自己的命令行参数
- 用 `ps aux | grep sead_onboard` 替代

### 4. state_callback PX4→APM 模式映射
- 原始 SEAD 代码在 `drone.py:134-156` 做 PX4→APM 模式翻译
- AUTO.LOITER → "LOITER", OFFBOARD → "GUIDED"
- MRS 起飞后状态 = AUTO.LOITER → Drone.mode = "LOITER"
- onboard node 检查 mode != GUIDED → 触发 set_mode(OFFBOARD)
- **workaround**: waypoint 分支加了 `local_pose[2] > 5m` 时跳过 set_mode

### 5. gps_enu_callback 坐标变换
- 原始代码: `enu→ecef→enu` 依赖 home 和 origin
- origin 从 `rospy.get_param("~gps_origin_0_lat/lon")` 读取
- home 从 mavros `/uav1/mavros/home_position/home` 订阅
- MRS 下 home position 需要 mavros 发布 (当前无 publisher)
- **后果**: local_pose 可能不准确

### 6. info 消息 (msg_id=44) 在 unpack_packet 无分支
- `communication_info.py:762` 最后一个 elif 是 Task_Insert
- info 消息走到函数末尾隐式 return None
- **修复**: sead_onboard_node 先取 result 再判空
- rosbridge info 格式也修了 (加 len 前缀)

## 7. 文件清单

### 已修改 (git tracked)

| 文件 | 改动 |
|------|------|
| `CLAUDE.md` | +仿真 convention, kill_sim |
| `src/xd_uav_sead/scripts/sead_onboard_node.py` | P0/P1 多项修复 |
| `src/xd_uav_sead/launch/sead_onboard.launch` | +use_simulation |
| `src/xd_uav_sead/launch/sead_formation_demo.launch` | +use_simulation |
| `src/xd_uav_sead/launch/sead_strike_demo.launch` | +use_simulation |
| `src/xd_uav_sead/launch/sead_gazebo_demo.launch` | +use_simulation |
| `src/xd_uav_sead/launch/sead_waypoint_demo.launch` | +use_simulation |
| `temp/TASK_LIST.md` | 进度更新 |

### 新增文件

| 文件 | 用途 |
|------|------|
| `temp/start_sim.sh` | 一键启动 MRS 仿真 |
| `temp/kill_sim.sh` | 一键关闭仿真 |
| `temp/rosbridge_dataflow_test.py` | 16 项协议闭环测试 |
| `temp/SEAD_QUICKSTART.md` | 快速入门 |
| `temp/HANDOFF.md` | 交接文档 |
| `temp/PLAYBOOK.md` | 本文档 |

### 禁区 (零改动)

| 路径 | 原因 |
|------|------|
| `~/PX4-Autopilot/` | 用户 PX4 环境 |
| `~/catkin_ws/src/mrs_uav_gazebo_simulator/` | 用户 MRS 仿真 |
| `~/catkin_ws/src/SEAD/` | 原始 SEAD 源码 |

## 8. 下一步工作

1. **继续 Phase 4**: waypoint 飞行验证 (启动仿真→SEAD→发 waypoint→Gazebo 观察)
2. **SEAD_mission 测试**: 发 3 目标 SimpleStrike 任务
3. **坐标系统**: 确认 home/origin 设置 → 确保 ENU 转换正确
4. **多机仿真**: PX4 multi-UAV SITL
5. **Phase 5**: XBee S3B 硬件插上后测试 `use_simulation:=false`
