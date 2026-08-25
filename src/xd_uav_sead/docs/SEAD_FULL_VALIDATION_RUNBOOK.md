# SEAD 全功能验证手册：`start.sh` 主线与可观察结果

本文只保留当前可执行的验证主线。旧版“三终端手工展开”的大量命令已删除；它们是启动器形成前的审计/故障恢复步骤，并非未知代码或测试输出残留。

## 通用启动、观察与清理

进入验证目录：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/tmux/validation
./start.sh v4
```

将 `v4` 换成 `v3`–`v10`。启动器创建专用 `sead-validation` tmux session：

- `launch`：roscore、Gazebo/spawner（仅飞行项）及各 UAV 节点日志；
- `status`：每两秒显示 MAVROS、estimator 和 manager 摘要；
- `commands`：输入本手册列出的短命令。

tmux 操作：

```text
Ctrl+B，再按 n/p       前后切换窗口
Ctrl+B，再按 0/1/2     跳到 launch/status/commands
Ctrl+B，再按 d         退出界面，进程继续运行
```

重新进入：

```bash
tmux -L sead-validation attach -t sead-validation
```

`v3/v4/v5` 会打开 Gazebo；三机首次启动可能需要 30–90 秒。不要因 pane 暂时等待而再次运行 `start.sh`。飞行前必须在 `status` 确认每架参与 UAV 都满足：

```text
connected: True
state_valid: True
localization_valid: True
```

结束飞行项时，先在 `commands` 执行 `land_all`，确认全部 `armed: False`，再 detach 并在普通终端执行：

```bash
./kill.sh
```

`start.sh` 会拒绝叠加到既有 PX4/MAVROS/Gazebo/ROS/SEAD 验证进程。`kill.sh` 会停止专用 tmux session，并按本验证链的明确特征清理和复查上述进程；不会做无范围的全局 `pkill`。

V3–V5 启动时，`start.sh` 还会等待本场景全部 UAV 同时满足 MAVROS `connected=True`、estimator 的 `state_valid=True`/`localization_valid=True`，并存在 manager takeoff 服务；上述条件连续 3 次成立后才返回或 attach。初始化期间会打印具体未就绪项。默认 180 秒超时，超时后脚本返回非零但保留 tmux 供诊断，不能在超时状态下继续下发任务。该门槛用于避免 estimator 初始化窗口中的首次起飞拒绝，不代表后续飞行任务已经通过。

### 实时可视化与证据保存

V3–V10 启动后，先在 `commands` 输入：

```bash
visualize
```

它会打开 SEAD 专用实时面板，且不向控制链发布任何消息：

- V3–V4：绘制各 UAV 的 MAVROS local ENU 轨迹；V5 自动读取并校验本轮 offset，绘制统一 shared ENU 轨迹、集结点、uav2 中心机及实际/理论 VEE 距离；
- V6：绘制禁飞区多边形、高度范围和接收事件；
- V7：绘制任务目标，并显示各节点 DPGA cost/chromosome 和 U2U 事件；
- V8：绘制一机一目标分配，并显示 ACK 集合和共同命中时间。
- V9：只绘制 uav1 蓝色实际轨迹和 `/uav1/dynamic_nofly_zone` 的红色多边形轮廓/填充，不叠加规划航线。黑色 `× T1` 是任务目标，蓝色实心点是飞机当前位置。
- V10：在统一 `world` 坐标中同时绘制 uav1/uav2/uav3 的真实轨迹，以及 `/sead/v10/dynamic_nofly_zone` 的共同红色区域；同样不叠加规划航线。

关闭窗口或运行 `kill.sh` 时会保存：

```text
.codex-tmp/visualizations/<场景>_<时间>/summary.png
.codex-tmp/visualizations/<场景>_<时间>/events.json
.codex-tmp/visualizations/<场景>_<时间>/trajectories.csv
.codex-tmp/visualizations/<V5时间>/offsets.env
```

建议在 `airspace_demo`、`dpga_demo`、`strike_demo` 之前启动面板，以免漏掉瞬时命令。PNG 用于快速查看，JSON/CSV 用于以后复盘和重新制图；V5 的 `offsets.env` 保证历史轨迹能恢复到当轮 shared frame。offset 文件缺失或 run ID 不匹配时 V5 面板拒绝启动，避免把三路不同 local odom 误画成编队几何。可视化进程异常不影响控制链。

## V0–V2：离线与协议回归

这三项没有 Gazebo 飞行画面，因此不存在“输入指令看飞机效果”。它们的效果是测试命令成功、节点/协议断言通过。代码未变化时采用已有记录，不重复执行。

若相关代码发生变化，应按包内测试清单重新运行对应单元测试和 launch 检查；不得用 V3–V8 的视觉结果替代 V0–V2。

## V3：单机起飞、航点、降落

启动：

```bash
./start.sh v3
```

等 uav1 connected 和两个 valid 均为 true，在 `commands` 输入：

```bash
visualize
takeoff_all
waypoint 2 0 1
land_all
```

可见效果：

- Gazebo 中 uav1 起飞，移动到指定局部航点附近，然后降落；
- `status` 中保持 armed/OFFBOARD，降落后变为 `armed: False`、manager STANDBY；
- `launch` 中不应出现持续 estimator invalid 或 manager FAILSAFE。

通过标准：航点响应明确、飞行期间闭环有效、最终安全上锁。随后执行 `./kill.sh`。

## V4：两机并发稳定性

启动：

```bash
./start.sh v4
```

确认 uav1/uav2 均 connected 且两个 valid 为 true，在 `commands` 输入：

```bash
visualize
takeoff_all
```

约保持 60 秒；本项不要发送航点、Formation、DPGA 或 SimpleStrike。观察：

- Gazebo 中两机各自在原地稳定悬停，无自行降落；
- `status` 中两机持续 armed/OFFBOARD、两个 valid 为 true、manager 不进入 FAILSAFE；
- 采证时同步检查 Gazebo RTF/系统负载、`/clock`、两路 MAVROS odom、estimator odom/valid/diagnostics、attitude setpoint、extended_state、manager status/diagnostics。

然后：

```bash
land_all
```

通过标准：保持段无意外上锁、持续 estimator 失效、FAILSAFE 或明显性能/时钟问题；最终两机均 `armed: False`。随后执行 `./kill.sh` 并确认无残留。

## V5：三机物理 Formation

启动：

```bash
./start.sh v5
```

启动器会等待三路 MAVROS odom、计算本轮 offset 并启动三套控制链。只有 `launch` 明确报告自动 offset 失败时，排除 PX4/MAVROS 问题后才输入 `auto_offsets`，不要在正常等待时反复调用。

三机均 connected/valid 后，在 `commands` 依次输入：

```bash
visualize
takeoff_all
trail_all
formation_point 35 6
```

可见效果：

- `trail_all` 更新编队配置；
- `formation_point` 后，Gazebo 中三机飞向集结区域并形成 VEE，随后稳定保持；
- `status` 中三机保持 armed/OFFBOARD、valid，不出现 FAILSAFE；
- `35 6` 应按当次出生位置选择，不能脱离空域机械复用。

观察足够时间后：

```bash
land_all
```

通过标准：三机形成目标几何且间距安全，保持期间闭环有效，最终全部 `armed: False`。随后执行 `./kill.sh`。

## V6：Airspace 分片、重组与存储

先在普通终端启动：

```bash
./start.sh v6
```

然后在 `commands` 输入：

```bash
visualize
airspace_demo
```

本项不启动 Gazebo，没有飞机运动画面。实际效果在 `launch` 的 uav1 pane：应依次看到 clear、分片完成/重组以及 store 日志，并能确认 Zone 1 的四个顶点及高度 `(0,20)`。

通过标准：区域被完整重组并存储，节点不崩溃。完成后 detach，执行 `./kill.sh`。

## V7：DPGA 无飞行三节点分配

先在普通终端启动：

```bash
./start.sh v7
```

然后在 `commands` 输入：

```bash
visualize
dpga_demo
```

等待三节点交换信息后，可再输入动态目标：

```bash
dpga_insert 25 5
```

本项不启动 Gazebo，效果在 `launch` 的 uav1/uav2/uav3 pane：

- src 1/2/3 持续交换 `/sead/u2u`；
- 三节点形成一致的三机类型化 chromosome/DPGA 视图；
- `dpga_insert` 后 `(25,5)` 出现在共享 `new_targets`；
- 无 MAVROS 时出现 waypoint rejected 属于本项预期边界，不代表 DPGA 协议失败。

通过标准：三节点视图一致、动态目标传播、节点不异常退出。物理任务飞行不由 V7 覆盖。完成后执行 `./kill.sh`。

## V8：SimpleStrike 无飞行协商

先在普通终端启动：

```bash
./start.sh v8
```

然后在 `commands` 输入：

```bash
visualize
strike_demo
```

本项不启动 Gazebo，效果在三个节点日志：

- uav1 leader 生成一机一目标分配；
- uav2/uav3 应用分配并返回 ACK；
- leader 显示 `assignment ack complete`；
- 三机路径状态被聚合，并接收、冻结同一个 `common_hit_time`；
- 无 MAVROS 时 waypoint/LOITER 超时属于无飞行边界。

通过标准：分配、ACK、路径聚合、共同时间传播全部完成，节点不崩溃。物理同步进场不由 V8 覆盖。完成后执行 `./kill.sh`。

## V9：固定翼动态禁飞区空地图 SITL

默认交互启动（创建 tmux，并在就绪后进入界面）：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/tmux/validation
./start.sh v9
```

默认 `nominal` 禁飞区半边长为 18 m，即 `36 m × 36 m`。禁飞区不是预先固定在世界坐标中：验收器先生成原始航线，再在飞机前方的原始航线上选取中心，因此原始航线必然穿过随后插入的区域。启动时可用米为单位覆盖半边长：

```bash
./start.sh v9 --zone-half-size 18
```

上例与当前 nominal 默认值相同，生成 `36 m × 36 m` 区域。半边长 25 m（`50 m × 50 m`）在一次旧规划配置下被安全拒绝，仍不作为默认演示值。`--zone-half-size` 只覆盖区域大小，不改变所选 profile 的任务距离、最小转弯半径或规划净空。三个 profile 未覆盖时的默认尺寸为：`nominal=36×36 m`、`relaxed1=30×30 m`、`relaxed2=24×24 m`。当前三个 profile 的最小转弯半径为 70/65/60 m，规划净空为 10/8/5 m；规划器从贴近请求净空且能够安全直达后续目标的候选开始，并对完整 Dubins 曲线做碰撞检查，验收仍独立要求真实轨迹不得进入红色多边形。nominal 已实飞通过，真实最小净空 39.36 m，完成到达 T1、返航和 `AUTO.LOITER`。

自动验收/无人值守启动：

```bash
./start.sh v9 --no-attach --profile nominal
```

`v9` 会自动完成 PX4 固定翼解锁、起飞、SEAD 航线下发、飞行中动态插入禁飞区、剩余航线重规划、真实轨迹绕飞、到达目标、返航和 `AUTO.LOITER` 收尾，不需要手工再发任务。交互式 `./start.sh v9` 会在 `commands` pane 启动时自动等待 ROS 并打开可视化，因此能够从仿真开始记录轨迹，不会遗漏手工开窗前的飞行段。再次输入下面的命令只会检查并复用已有节点，不会重复开窗：

```bash
visualize
```

`--no-attach` 默认不启动图形窗口；如需显式覆盖，可在启动前设置 `SEAD_VALIDATION_AUTO_VISUALIZE=true|false`。

面板只显示蓝色实际轨迹与红色动态禁飞区；关闭后保存 PNG/JSON/CSV。轨迹从跑道起飞时开始记录，而 SEAD 航线是在爬升完成、任务接管时才生成，两者起点本就不同，因此不再把规划线叠加到该视图。T1 在固定翼完成起飞爬升后才生成：以当时飞机位置为原点，沿实测地面航向向前放置 300 m（nominal）。可视化订阅的是锁存任务消息，所以打开窗口的第一帧就会看到 T1；这不表示飞机出生在 T1。验收终端最终必须出现 `success: true`，并同时满足：动态重规划发生、规划航线满足净空、实际轨迹未进入多边形、到达目标、返航、最终保持解锁的 `AUTO.LOITER`。完成后执行：

```bash
./kill.sh
```

仅在 nominal 的规划或真实净空确实失败时，才按顺序使用一次放宽配置，不可跳级掩盖控制故障：

```bash
./start.sh v9 --no-attach --profile relaxed1
./start.sh v9 --no-attach --profile relaxed2
```

### 飞行期间通过 ROS 话题更新禁飞区

动态接口是 `/uav1/dynamic_nofly_zone`，类型为 `xd_uav_sead/NoFlyZone`。对同一个非零 `zone_id` 再发送 `operation: 0`（UPSERT）会原子替换原多边形，因此可用来实时改变大小或位置。frame 必须是 `uav1/odom`；时间戳必须新鲜；`valid_until` 必须晚于 stamp 且 TTL 不超过 60 s；顶点应按边界顺序组成无自交多边形。下面示例读取仿真 `/clock`，把 zone 9100 更新为中心 `(225,-13)`、半边长 20 m 的 `40×40 m` 矩形，有效期 45 s：

```bash
clock_yaml="$(rostopic echo -n 1 /clock/clock)"
zone_now_s="$(awk '/secs:/ {print $2; exit}' <<<"$clock_yaml")"
zone_now_ns="$(awk '/nsecs:/ {print $2; exit}' <<<"$clock_yaml")"
zone_until_s=$((zone_now_s + 45))

rostopic pub -1 /uav1/dynamic_nofly_zone xd_uav_sead/NoFlyZone "
header:
  stamp: {secs: ${zone_now_s}, nsecs: ${zone_now_ns}}
  frame_id: 'uav1/odom'
schema_version: 1
operation: 0
zone_id: 9100
enabled: true
zone_type: 0
min_altitude: 0.0
max_altitude: 100.0
valid_until: {secs: ${zone_until_s}, nsecs: ${zone_now_ns}}
polygon:
  points:
    - {x: 205.0, y: -33.0, z: 0.0}
    - {x: 245.0, y: -33.0, z: 0.0}
    - {x: 245.0, y:   7.0, z: 0.0}
    - {x: 205.0, y:   7.0, z: 0.0}"
```

改变 `zone_id: 9100` 的四个顶点并再次执行同一 UPSERT，即可实时改变该区域大小。后续独立接口测试建议使用 9100 等自定义 ID。自动 v9 验收器自身使用 zone 9001，并约每 15 s 刷新它的 45 s TTL；因此在 v9 运行中手工覆盖 9001 会被验收器恢复，不能作为持久人工修改。要观察人工区域，可使用不同 ID；但新区域若让当前航迹无安全可行解，SEAD 会 fail-closed、拒绝继续使用旧航线，这是预期安全行为。

headless SITL 专用配置会回读确认 PX4 模拟电池保持 100%，并豁免无 RC/GCS 环境下 OFFBOARD 与 Hold 的链路丢失动作；这些旁路不用于真机。真实 XBee/DigiMesh、GCS 电台和真机仍是独立硬件验收，仿真结果不能替代射频链路、急停和设备拔插测试。

## V10：三固定翼共同动态禁飞区

交互演示（Gazebo 与可视化自动打开）：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/tmux/validation
./start.sh v10 --zone-half-size 18
```

无人值守聚合验收：

```bash
./start.sh v10 --no-attach --profile nominal --zone-half-size 18
```

V10 启动三架 PX4 `plane`，每架都有独立 MAVROS、estimator、manager/controller、
SEAD 和验收器。三机采用独立 SITL instance/MAVLink 端口，出生点横向分离，并分别爬升
到 30/40/50 m。任务发布前设置空中同步屏障，避免较早起飞的飞机在等待其他飞机时提前
飞过待插入区域；三条初始路径准备完成后，由 uav1 验收器在其原始路径前方选择 zone
9001，并通过锁存的 `/sead/v10/dynamic_nofly_zone` 以 `world` frame 向三机扇出。

判据不是强迫所有飞机做无意义绕行：初始路径受区域影响时必须产生不同的新路径且满足
规划净空；初始路径本来安全时允许保持原路径。无论是否重规划，每架都必须保持真实轨迹
不进入多边形、到达目标、返航并进入保持解锁的 `AUTO.LOITER`；聚合结果还要求至少一架
实际重规划，防止区域完全没有挑战任何航线。验收成功时终端显示：

```text
v10 三固定翼动态禁飞区验收全部通过（实际重规划 N/3）。
```

当前 nominal `36×36 m` 最终实测为 3/3 重规划并完整通过：uav1/uav2/uav3 的规划最小
净空分别为 48.24/43.03/43.00 m，真实最小净空为 81.74/45.19/44.27 m，耗时
106.9/80.9/81.2 s，最终均为 `AUTO.LOITER`。这些数值属于该轮证据，不是写入控制器的
阈值。

交互启动会像 V9 一样自动运行 `visualize`，从仿真早期记录三条真实轨迹和共同禁飞区；
再次输入 `visualize` 会复用已有节点。`--no-attach` 默认关闭 Gazebo GUI 和可视化窗口。
完成后执行：

```bash
./kill.sh
```

运行期公共动态接口为 `/sead/v10/dynamic_nofly_zone`，消息类型仍是
`xd_uav_sead/NoFlyZone`，但 `header.frame_id` 必须为 `world`。对同一 zone ID 做 UPSERT
即可实时改变大小/位置；自动验收会刷新 9001，因此人工接口测试使用 9100 等其他非零 ID。
消息时间戳、TTL、简单多边形和高度限制与 V9 相同。

## 异常处置与结论边界

飞行时若出现持续 estimator invalid、manager FAILSAFE、意外上锁、失控趋势或 RTF/时钟明显异常，立即停止下发新任务并执行 `land_all`；若常规降落不可用，再按现场安全流程处理。不要通过无依据地放宽安全阈值或加入 SEAD 魔法数字掩盖问题。V9/V10 的 2 秒 MAVROS state watchdog 是针对该话题约 1 Hz 发布周期设置的两周期门限；其余高频估计器、控制器和空速门限保持原值。V10 只证明共同禁飞区控制，不包含固定翼之间的轨迹互避。

若证据指向其他共享包缺陷，只记录复现条件、日志、影响和最小修改方案，未经授权不直接修改。阶段 3 的当前剩余边界是 DPGA/SimpleStrike 物理飞行、固定翼实飞、真实 XBee，以及已登记间歇性问题的统一归因。
