# SEAD 原始功能映射与逐项复现记录

本文以 `/home/promise/catkin_ws/src/SEAD` 的实际代码为基准，不以旧 AI 文档或 launch 文件名推断功能已经可用。目标是在 ROS1 移植包中逐项确认：代码映射、运行入口、依赖、可观察效果和未覆盖风险。

## 验证口径

每项能力按以下层级记录，低层通过不代表高层通过：

1. **代码映射**：原始实现能在移植包中找到，语义变化已说明。
2. **静态验证**：Python 导入、语法、launch/XML 或协议离线检查通过。
3. **节点验证**：ROS 节点能启动，输入能到达对应分支，并产生预期日志、话题或状态。
4. **控制效果**：PX4/Gazebo 中出现预期飞行动作，且没有控制权冲突。
5. **协同效果**：多机通信、任务分配、编队或同步打击在多机运行中得到证据。

状态使用：`已验证`、`部分验证`、`待验证`、`不适用`。所有“已验证”都必须附可重复证据；仅存在代码不得写成已验证。

## 原始运行入口与系统边界

原始入口是 `onboard.py`。每架无人机运行一个进程，完成 XBee 设备发现、ROS/MAVROS 状态接入、GCS/U2U 数据处理、任务状态机和飞行控制。原始系统不是一组彼此独立的 demo：航点、编队、空域、DPGA SEAD 和 SimpleStrike 都由同一主循环按消息类型和 Mission 状态切换。

原始 `xbee_send_formation_test.py` 是独立的 GCS 侧硬件工具，用于通过 DigiMesh 单播发送 Swarm Command 和 Formation Point；它不是机载节点，也不是普通单元测试。

## 功能映射与当前状态

| 原始文件/能力 | 移植对应 | 预期可观察效果 | 当前状态 | 下一项最小验证 |
|---|---|---|---|---|
| `drone.py`：MAVROS 状态、模式、解锁、起飞、位置/速度控制 | `drone/drone.py`，增加 `xd_control_manager` 后端 | 状态有效；安全进入 OFFBOARD；产生唯一控制参考 | 部分验证：manager 后端起飞、1 m 悬停、本地航点和受控下降已通过 | 复测 0.40 m 触地阈值与最终自动上锁 |
| `communication_info.py`：G2U/U2G/U2U 二进制协议 | `comms/communication_info.py` + `comms/rosbridge.py` | 命令编码后被原协议正确解码；U2U 可跨 UAV 路由 | 部分验证：基础命令、Swarm、Airspace 修复后协议测试已通过；真实多节点路由待测 | 在独立 ROS master 上运行协议测试和双节点 U2U 路由检查 |
| `onboard.py`：统一机载入口与任务状态机 | `scripts/sead_onboard_node.py` | 单节点启动、按消息切换 Mode/Mission、正确调度各功能 | 部分验证：单机 takeoff/waypoint/land 分支已进入 | 分功能输入验证状态转换，避免用重复 launch 冒充不同功能 |
| `airspace_manager.py`：禁飞区增删、点内判断、规划器导出 | `airspace/airspace_manager.py` | Zone 分片组成合法区域；查询与规划输入一致 | 部分验证：区域增删、水平内外、高度边界和规划器导出已通过 | 验证 ROS Zone 下发后进入任务规划 |
| `pathFollowing.py`：固定翼路径跟随、期望点/速度、控制律 | `planning/pathFollowing.py` | 给定路径与位姿产生连续、有限且方向正确的引导量 | 部分验证：固定翼航点按路径顺序前向推进并在末点结束 | 补充连续引导量；固定翼实飞不与 x500 验收混合 |
| `GA_SEAD_process.py`：GA 分配、Dubins 路径、禁飞区绕行 | `planning/GA_SEAD_process.py` | 固定输入产生合法任务染色体和不穿越禁飞区的路径 | 无飞行效果已验证：Dubins 绕障通过；固定种子真实 GA 将单目标 Recon/Strike/BDA 分配给对应能力 UAV，适应度有限且为正 | 后续多机飞行中核对分配路径执行 |
| `DPGA.py`：GA 子进程、机间状态交换、任务计划与执行 | `planning/DPGA.py` | 多 UAV 达成任务分配并输出各自路径/控制目标 | 部分验证：多机输入和分配进程生命周期通过；真实 GA 小场景通过 | 后续验证 `plan_only` ROS 多节点路径执行；不能以离线测试证明完整分布式飞行 |
| `formation_control.py`：集结、TRAIL/VEE/ECHELON 等队形、避碰 | `formation/formation_control.py` | 多机状态输入产生稳定 slot、集结/转换/保持指令 | 无飞行效果已验证：8 种槽位、ASSEMBLE→HOLD、三机 TRAIL 角色锁定与居中 VEE 转换均通过 | 后续用可见 Gazebo 验证多机物理效果 |
| `simple_strike.py`：三目标分配、ACK、路径状态、共同命中时间 | `strike/simple_strike.py` | 3 机完成分配锁定、路径准备和同步进场 | 无飞行协同状态已验证：三个真实 manager 完成分配锁定、ACK 汇聚、三份路径状态聚合和共同命中时间广播 | 后续用多机飞行验证同步进场误差 |
| `xbee_send_formation_test.py`：硬件编队命令发送 | `tools/xbee_send_formation_test.py` | 在真实 DigiMesh 设备上向指定 UAV 发送两类合法包 | 代码映射已确认，硬件未验证 | 保留为硬件工具；无设备时只做 CLI/打包离线检查，不宣称硬件成功 |

## launch 审计

审计前共有 6 个 launch。引用、历史和运行契约核对后的处理结果：

- `sead_xd_control.launch` 是已验证的 x500 控制链组合入口，应保留。
- `sead_onboard.launch` 已参数化仿真/硬件选择、SEAD 运行模式和两个控制模式，作为统一机载入口保留。
- `sead_formation_demo.launch`、`sead_waypoint_demo.launch` 与通用入口实质相同且没有独立输入，已删除。
- `sead_gazebo_demo.launch` 不启动 Gazebo/PX4 且包含 120 m 可选自动起飞，已删除。
- `sead_strike_demo.launch` 固定选择当前 x500 manager 链不消费的 SwiftWing vector 输出，独立语义已由通用入口参数覆盖，已删除。

仓库现行代码没有引用被删除入口；搜索命中仅来自不作为权威状态的历史 `temp/` 资料。现在保留一个明确的通用机载入口和一个明确的 x500 manager 联调入口。功能复现由可重复的测试输入和观察项定义，不再用同构 launch 文件名冒充功能覆盖。

## `test/` 与 `tools/` 必要性

- `test/test_rosbridge_protocol.py` 被 `CMakeLists.txt` 的 `catkin_add_nosetests()` 注册，覆盖移植新增 ROS bridge 与原二进制协议的兼容性。它是有效回归测试，不是运行残留，当前应保留。后续若替换测试框架，也必须先提供等价覆盖。
- `tools/xbee_send_formation_test.py` 对应原始仓库同名硬件工具，被 `catkin_install_python()` 安装且由 README 说明。它承载原始能力的保留，不应因目录名为 `tools` 而删除。需要核对其内容是否与原始文件一致以及依赖失败时的行为。
- `scripts/__pycache__`、`test/__pycache__` 是本地 Python 缓存，不是源码能力或交付物；应由忽略规则排除，并可作为确定性冗余清理。

## 2026-08-01 离线复现结果

运行环境为 ROS Noetic overlay 与系统 Python 3.8.10。执行 `test/test_core_functions.py`，4 项测试全部通过：

- Airspace：区域增删、水平内外、高度范围和规划器导出。
- PathFollowing：固定翼航点从起点跳过已到达点，按顺序前进，并在终点后返回完成。
- Formation：VEE、左右梯队、TRAIL、TRIANGLE、WEDGE_WIDE、ARROW、INVERTED_VEE 均为 5 机产生唯一且满足 40 m 最小间距的有限槽位。
- Formation 状态演化：单机按每周期输出航点推进，虚拟编队中心在有限步内从 `ASSEMBLE` 进入 `HOLD`；测试不通过瞬移伪造状态跳转。
- Planner：起终点直线被矩形禁飞区阻挡时，生成多点 Dubins 绕行路径，所有采样点均位于多边形之外。
- DPGA 输入：乱序 UAV 字典稳定排序，completed/new targets 扁平合并，zones 原样进入 GA 输入。
- SimpleStrike：重复目标稳定去重，三机三目标产生确定的一机一目标分配。
- DPGA 生命周期：受控求解器读取初始输入、输出 `[fitness, solution]`，收到 `[44]` 后退出且不遗留队列数据。
- SimpleStrike 协议：Assignment、AssignmentAck、ReleaseTime、PathStatus 的字段和浮点值往返一致。
- 真实 GA 小场景：固定随机种子、1 目标、3 种能力 UAV、3 次迭代；输出 Recon→UAV1、Strike→UAV2、BDA→UAV3，适应度有限且为正。
- SimpleStrike 三节点：两个 follower 接受并锁定 leader 分配，leader 收齐 UAV2/UAV3 ACK；三份 ready path status 聚合后识别 UAV3 最晚、到达时间跨度 6 s，并广播可解码的共同命中时间。
- Formation TRAIL→VEE：三机直线队形锁定 UAV2 为中心，转换在 4 s 推进后完成，形状变为 VEE 并冻结以中心机为首的槽位顺序。

这些结果属于确定性离线功能效果，不等价于 ROS 多节点或多机飞行验证。

## 执行顺序

1. 纯算法/协议：Airspace、PathFollowing、GA、协议打包与 U2U 路由。
2. 单机 ROS：统一入口、Mode/Arm/Takeoff/Waypoint/Land、任务状态切换。
3. 单机飞行：x500 manager 后端和自动上锁收尾。
4. 多节点无飞行：Formation、DPGA、SimpleStrike 的消息与状态机协同。
5. 多机仿真：只有前述边界稳定后才验证编队和协同任务效果。

阶段 4 的 QGC/UDP 适配不在本文执行范围内。
