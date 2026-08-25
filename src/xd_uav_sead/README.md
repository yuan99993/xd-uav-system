# xd_uav_sead

ROS1 Noetic 下的 SEAD 移植包，保留 XBee 协议、ROS 仿真桥、DPGA、固定翼路径跟随、
编队、简化打击和禁飞区规划。

## 主要入口

- `scripts/sead_onboard_node.py`：机载主循环。
- `scripts/mock_gcs.py`：ROS 模拟地面站命令。
- `tmux/validation/start.sh`、`kill.sh`：V3–V9 仿真验证入口和安全清理。
- `msg/NoFlyZone.msg`：带版本、frame、高度和有效期的动态禁飞区消息。
- `launch/sead_fixedwing_xd_control.launch`：SEAD → 自研 manager/controller 控制链。

## 仿真验证

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/tmux/validation
./start.sh v4
# 在 commands pane 完成场景操作和降落后：
./kill.sh
```

场景含义、commands pane 操作、观察指标和清理要求见
`docs/SEAD_FULL_VALIDATION_RUNBOOK.md`。

固定翼动态禁飞区演示：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/tmux/validation
./start.sh v9
# 可选：指定禁飞区半边长（米）
./start.sh v9 --zone-half-size 18
# 无人值守验收
./start.sh v9 --no-attach --profile nominal
```

V9 自动完成起飞、任务下发、飞行中禁飞区插入、重规划绕飞、返航和 LOITER。默认
nominal 区域为 `36 m × 36 m`；运行期间的 ROS 动态更新方法和完整判据见使用手册。
真实 XBee/DigiMesh、GCS 电台与真机仍需单独硬件验收。

## 常用命令

发送任务：

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=sead_mission \
  _targets_json:='[[600,0]]' _uav_type:=2 _velocity:=15 _Rmin:=50 \
  _waypoint_radius:=40 _end_x:=800
```

动态禁飞区通过 `/uav1/dynamic_nofly_zone` 发布 `xd_uav_sead/NoFlyZone`。消息的
`header.frame_id` 必须与控制参考 frame 一致，过期、非法或无法安全重规划的输入会触发
fail-closed。

## 验证

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/catkin_ws/devel/setup.bash
catkin_make -j2 --pkg xd_uav_sead
catkin_make -j2 run_tests_xd_uav_sead
catkin_test_results build/test_results/xd_uav_sead
```

最近一次自动测试结果为 37 tests、0 failures。编译并发必须保持 `-j2`。
