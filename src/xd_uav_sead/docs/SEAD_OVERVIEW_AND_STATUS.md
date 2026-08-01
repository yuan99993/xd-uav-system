# SEAD 原理、用途与当前完成度

本文面向未阅读过原始 `src/SEAD` 源码的操作者，解释系统用途、主要数据流、已经验证的能力和仍待验证的能力。完成度以代码和可重复证据为准，不以文件存在或 launch 名称推断。

## SEAD 是什么

SEAD 是 Suppression of Enemy Air Defenses，即“压制敌方防空系统”。原始项目让多架无人机分别运行同一个机载程序，通过 XBee DigiMesh 交换状态，由地面站下发目标、禁飞区和编队要求，再协同执行侦察、打击和战损评估。

当前 ROS1 移植保留原有算法和 XBee 路径，并增加 ROS 仿真通信及 `xd_uav_control_manager` 控制后端。仿真路径并不替代真实 XBee，也不能证明硬件已经可用。

```text
地面站：目标、禁飞区、任务和编队命令
                    |
                    v
每架 UAV 的 sead_onboard_node：通信、任务状态机、协同逻辑
                    |
         +----------+----------+
         |          |          |
         v          v          v
    Formation    GA/DPGA   SimpleStrike
      编队       任务分配     同步进场
         \          |          /
          +---------+---------+
                    |
                    v
       航点或速度/航向控制参考
                    |
                    v
xd_uav_control_manager -> xd_uav_controller -> PX4 -> Gazebo/真机
```

## 主要能力

### 航点与飞行控制

系统读取 MAVROS 的位置、姿态、飞行模式和武装状态，可执行起飞、航点、保持和降落。当前 x500 仿真使用 `xd_uav_control_manager` 后端：SEAD 只提交期望位置，manager 负责 OFFBOARD、解锁、状态机和安全收尾，controller 是姿态 setpoint 的唯一业务输出者。

### GA/DPGA 任务分配

SEAD 将任务区分为侦察、打击和战损评估，并按 UAV 能力进行分配。遗传算法同时考虑任务完成时间、总路径长度、阶段顺序和负载均衡。DPGA 让各 UAV 交换状态并各自运行分配过程。

### 路径规划与禁飞区

规划器使用 Dubins 路径，适配固定翼的最小转弯半径，并能检测和绕开多边形禁飞区。原项目以固定翼为主要对象，因此速度、高度、盘旋和转弯参数不能直接照搬到 x500 四旋翼。

### Formation 编队

支持 TRAIL、VEE、左右梯队、三角形等队形。飞机先飞向集结点，再分配编队槽位并进入保持状态；部分流程可从 TRAIL 转换到 VEE。

### SimpleStrike

三架 UAV 对三个目标进行稳定去重和就近分配，leader 收集 ACK 与各机剩余路径，计算共同命中时间，各机通过调整速度争取同步到达。

### 通信

- 原始硬件：XBee DigiMesh 二进制协议。
- 当前仿真：ROS GCS 命令、遥测和共享 `/sead/u2u` 机间总线。
- QGC/UDP：规划中的阶段 4，当前尚未实现端到端验收。

## 当前完成度

### 已有较强验证证据

- ROS1 Noetic 包结构、入口、依赖和主要源码映射。
- 基础命令、Swarm、Airspace、SEAD 和 SimpleStrike 协议往返。
- ROS 多节点 U2U 路由。
- 禁飞区增删、区域判断和 Dubins 绕障。
- 固定种子 GA 小场景的侦察、打击、评估分配。
- DPGA 子进程输入、输出和受控退出。
- 8 种编队槽位及 TRAIL 到 VEE 状态转换。
- SimpleStrike 三节点的分配、ACK、路径状态和共同命中时间。
- x500 单机起飞、悬停、航点、降落和自动上锁。
- SEAD 经仓库 manager/controller 控制 PX4 的单机闭环。
- 独立 uav2 在 OFFBOARD 下持续稳定飞行，未发生意外上锁或 estimator 状态失效。
- 两机并发稳定保持、降落和最终上锁。
- 三机 TRAIL→VEE 物理 Formation 连续保持和安全降落。
- V3–V8 专用实时可视化及 PNG/JSON/CSV 证据保存；V6 Airspace 保存链已有实际运行证据。

阶段 3 的核心目标——SEAD 接入现有 manager/controller，并在仿真中验证单机、两机稳定性、三机 Formation 和主要协同协议——已在当前验收边界内完成。

### 尚未完成或仅部分完成

1. **DPGA 多机任务飞行**：算法和三节点分配链已通过，但多架 Gazebo UAV 完整执行侦察、打击和评估路径仍是延期扩展项。
2. **SimpleStrike 物理同步进场**：软件协商已通过，三机实际共同到达误差是延期扩展项。
3. **真实 XBee**：代码路径存在，但需要真实 DigiMesh/GCS 电台和真机安全授权。
4. **固定翼飞行效果**：当前以 x500 验证控制接口；固定翼 Dubins 跟随、速度矢量和盘旋需要独立环境。
5. **间歇性/触地问题**：只在可恢复或安全上锁后出现的 valid 瞬态已登记，后续统一归因；若空中持续或触发 FAILSAFE，应立即升级。
6. **QGC/UDP**：阶段 4 尚未实施。

## 当前正确的下一步

阶段 3 按现有仿真边界结束，下一步进入阶段 4。第一步不是直接修改 QGC，而是只读审计 `src/Release` 与 `src/qgroundcontrol` 的版本/产物对应关系、已有 SEAD UI 或 UDP 代码，以及 SEAD 当前 ROS 命令接口；随后形成版本化最小 UDP 协议、坐标系/单位、任务 ID、校验、ACK/重试和错误处理方案。

协议与适配器边界确认后，先在 QGC 外实现并验证本地 UDP 发送器 → 独立 Python UDP→ROS 适配器 → `xd_uav_sead`，最后才申请修改 QGC UI/发送逻辑。阶段 3 延期项不冒充通过，也不阻塞这条软件集成主线。

更详细的功能证据见 `SEAD_FUNCTION_VERIFICATION.md`，控制链和历史故障见 `PHASE3_CONTROL_RUNBOOK.md`。
