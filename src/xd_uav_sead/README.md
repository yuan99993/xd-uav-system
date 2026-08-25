# xd_uav_sead

ROS1 Noetic 下的 SEAD 移植包，保留 XBee 协议、ROS 仿真桥、DPGA、固定翼路径跟随、
编队、简化打击和禁飞区规划。

## 主要入口

- `scripts/sead_onboard_node.py`：机载主循环。
- `scripts/mock_gcs.py`：ROS 模拟地面站命令。
- `tmux/validation/start.sh`、`kill.sh`：V3–V10 仿真验证入口和安全清理。
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

三固定翼共同动态禁飞区（V10）：

```bash
# 交互模式：打开 Gazebo、进入 tmux，并自动启动三机轨迹/NFZ 可视化
./start.sh v10 --zone-half-size 18
# 无人值守正式验收：不弹 GUI，等待三机聚合结果
./start.sh v10 --no-attach --profile nominal --zone-half-size 18
# 完成或中止后清理
./kill.sh
```

V10 启动 3 个独立 PX4 `plane` SITL、MAVROS、自研 estimator/manager/controller 和
SEAD。三机在 `world` 共享坐标中接收同一个 zone 9001；初始路径受影响的飞机必须产生
安全重规划，未受影响的飞机可保持原安全路径，但聚合验收至少要求一架真实重规划。当前
nominal 默认仍为半边长 18 m（`36 m × 36 m`）。运行期间可向
`/sead/v10/dynamic_nofly_zone` 发布同 frame 的 `NoFlyZone`；自动验收会周期刷新 9001，
人工接口测试应使用其他非零 zone ID。完整消息示例、判据和可视化说明见 Runbook。
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

最近一次 V10 相关纯自动测试为 46 tests、0 failures；ROS 集成测试在受限环境因网卡枚举
权限未启动，最终由真实三固定翼 SITL 验收覆盖。编译并发必须保持 `-j2`。
