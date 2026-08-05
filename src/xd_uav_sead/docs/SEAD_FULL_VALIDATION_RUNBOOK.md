# SEAD 全功能验证手册：`start.sh` 主线与可观察结果

本文只保留当前可执行的验证主线。旧版“三终端手工展开”的大量命令已删除；它们是启动器形成前的审计/故障恢复步骤，并非未知代码或测试输出残留。

## 当前实测状态与下一步（2026-08-01）

| 项目 | 当前结论 | 下一步边界 |
|---|---|---|
| V0–V2 | 构建、launch、协议、核心算法回归通过 | 相关代码未变时不重复 |
| V3 | 单机起飞、航点、降落通过 | 无 |
| V4 | 两机并发稳定性通过，带已登记间歇性问题 | 后续统一归因 |
| V5 | 三机物理 Formation 通过 | 无 |
| V6 | Airspace 无飞行节点链通过 | 无 |
| V7 | DPGA 无飞行三节点链通过 | 物理任务飞行未验证 |
| V8 | SimpleStrike 无飞行协商通过 | 物理同步进场未验证 |
| V9 | 未执行 | 需要真实 DigiMesh/GCS 电台和真机授权 |
| V10 | 未执行 | 属于阶段 4，未获授权不进入 |

阶段门结论：V0–V8 已按各自定义的验收边界通过，因此阶段 3 核心仿真验收完成。V9、DPGA/SimpleStrike 物理任务飞行、固定翼实飞及间歇性问题归因作为硬件或扩展验收项延期，不得写成已验证，但不再阻挡阶段 4 的只读审计和协议设计。

关键实测摘要：

- V4 两机共同 `armed=True/OFFBOARD` 134.3 仿真秒；起飞完成至首个 `LANDING` 的 123.2 秒稳定段内，无 estimator valid=false、manager FAILSAFE、意外上锁或离开 OFFBOARD。两机最终均 `armed=False`、STANDBY。
- V4 `/clock` 约 250 Hz、无回退，CPU 平均忙碌 29.3%、峰值 62%，无 swap/I/O wait，未见性能或时钟故障证据。
- V5 Formation 命令后保持至降落共 217.0 仿真秒；最后 60 秒两翼到中心平均距离均为 4.999 m，翼间平均 6.559 m。三机最终均安全上锁。
- V6 成功重组并存储 Zone 1：四点 `(5,-2) (9,-2) (9,2) (5,2)`，高度 `(0,20)`。
- V7 三节点形成一致 chromosome，动态目标 `(25,5)` 进入共享 `new_targets`。
- V8 完成 leader 分配、两从机 ACK、路径状态聚合与共同命中时间冻结。
- V4 证据：`.codex-tmp/sead_v4_two_uav_20260801.bag`、`.codex-tmp/sead_v4_vmstat_20260801.log`；V5 证据：`.codex-tmp/sead_v5_formation_20260801.bag`。

已登记的间歇性问题：

- V4 第一次 `takeoff_all` 时 uav2 瞬时因控制状态无效拒绝；valid 恢复后重试成功。2026-08-05 定向复现确认 `start.sh` 返回后仍可能短暂出现单机 `state_valid=False`，现已增加连续 readiness gate；修复后 V5 启动返回即执行 `takeoff_all`，三机首次请求全部接受并进入 ACTIVE/OFFBOARD。触地末段两机短暂隔离 `mavros/velocity_z`，未进入 FAILSAFE 并安全上锁。
- V5 起飞初段 uav1/uav2 各一次 `PRESTREAM -> WAIT_STATE -> PRESTREAM` 后自动恢复；触地安全上锁后 uav2 valid 未在采集结束前恢复。
- 历史轮次中 uav2 独立 `AUTO.LAND`/意外上锁尚未在 2026-08-05 两轮 V5 定向运行中复现，不能宣称已由 readiness gate 修复；若再次出现，必须同步采集后单独归因。若同类问题在空中持续、触发 FAILSAFE 或影响安全，应立即终止对应验证。

## 通用启动、观察与清理

进入验证目录：

```bash
cd /home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/tmux/validation
./start.sh v4
```

将 `v4` 换成 `v3`–`v8`。启动器创建专用 `sead-validation` tmux session：

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

V3–V8 启动后，先在 `commands` 输入：

```bash
visualize
```

它会打开 SEAD 专用实时面板，且不向控制链发布任何消息：

- V3–V4：绘制各 UAV 的 MAVROS local ENU 轨迹；V5 自动读取并校验本轮 offset，绘制统一 shared ENU 轨迹、集结点、uav2 中心机及实际/理论 VEE 距离；
- V6：绘制禁飞区多边形、高度范围和接收事件；
- V7：绘制任务目标，并显示各节点 DPGA cost/chromosome 和 U2U 事件；
- V8：绘制一机一目标分配，并显示 ACK 集合和共同命中时间。

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

## V9：真实 XBee/DigiMesh

当前没有可安全执行的 `start.sh v9`。此项必须具备真实 DigiMesh 设备、GCS 电台、正确串口映射、真机安全区和明确授权；仿真日志不能替代射频链路验收。

满足条件后需要另行制定硬件检查表，至少覆盖地址映射、双向收发、丢包/重连、错误帧、急停和设备拔插恢复。在这些条件满足前，V9 保持未执行。

## V10：QGC/UDP 与阶段 4

当前没有 `start.sh v10`，也不是“没有效果可看”，而是它明确属于阶段 4。阶段 3 已交接完成；下一步先只读审计 QGC 产物/源码并设计 UDP→ROS 协议，协议确认后才实现独立适配器，最后再修改和验证 QGC。

## 异常处置与结论边界

飞行时若出现持续 estimator invalid、manager FAILSAFE、意外上锁、失控趋势或 RTF/时钟明显异常，立即停止下发新任务并执行 `land_all`；若常规降落不可用，再按现场安全流程处理。不要通过放宽安全阈值或加入 SEAD 魔法数字掩盖问题。

若证据指向其他共享包缺陷，只记录复现条件、日志、影响和最小修改方案，未经授权不直接修改。阶段 3 的当前剩余边界是 DPGA/SimpleStrike 物理飞行、固定翼实飞、真实 XBee，以及已登记间歇性问题的统一归因。
