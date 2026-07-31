# xd_uav_sead — Quick Start

> 状态: Phase 4 OFFBOARD 验证通过 | 2026-07-29

## 一键启动

```bash
# 终端1: 启动仿真
cd ~/catkin_ws/src/xd-uavsystem-test && bash temp/start_sim.sh

# 终端2: SEAD 机载
source ~/catkin_ws/devel/setup.bash
roslaunch xd_uav_sead sead_onboard.launch

# 终端3: 控制指令
source ~/catkin_ws/devel/setup.bash

# 切 OFFBOARD (已验证通过)
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=mode _mode:=GUIDED

# 发 waypoint
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=waypoint _x:=30 _y:=0 _z:=30

# 其他已确认指令
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=info _text:="hello"
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=freq _freq:=2.0
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_clear
```

## 已验证命令

| 命令 | 效果 | 状态 |
|------|------|------|
| `mode GUIDED` | Quad OFFBOARD 切换 | ✅ |
| `info` | 节点回显 | ✅ |
| `freq` | U2G 频率设置 | ✅ |
| `airspace_clear` | 清空空域 | ✅ |
| `waypoint` | 航点飞行 | 待轨迹验证 |

## 关闭

```bash
pkill -f sead_onboard
bash temp/kill_sim.sh
```
