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

这些结果中，算法和多节点测试不等于多机物理飞行已经通过。

### 尚未完成或仅部分完成

1. **三机物理编队**：软件状态能进入 TRAIL/VEE，但上次三机飞行中 uav2 单独失效，物理 VEE 连续保持尚未通过。
2. **多机故障归因**：独立 uav2 正常，问题可能与多实例并发、第二次起飞、Formation 后机动或消息时序有关。
3. **DPGA 多机任务飞行**：算法能产生分配，尚未证明多架 Gazebo UAV 能完整执行侦察、打击和评估路径。
4. **SimpleStrike 物理同步进场**：软件协商已通过，三机实际共同到达误差尚未验收。
5. **真实 XBee**：代码路径存在，但没有真实设备端到端证据。
6. **固定翼飞行效果**：当前主要用 x500 验证接口；固定翼 Dubins 跟随、速度矢量和盘旋需单独环境验证。
7. **QGC/UDP**：尚未进入阶段 4。

## 当前正确的下一步

完整验收顺序和三终端操作方法见 `SEAD_FULL_VALIDATION_RUNBOOK.md`。当前实际下一步是其中 V4“两机、无 Formation、一次起飞”对照测试；它只判断多实例并发是否足以触发 uav2 异常，不验证编队，也不重复已经通过的单机协议和离线测试。

若两机稳定，下一步才比较第二次起飞或最小三机 Formation；若两机已经出现异常，则优先分析同步 rosbag 中 PX4、estimator、manager、时钟和系统负载的时间关系。不得通过 SEAD 内硬编码、放宽 estimator 阈值、延长 manager timeout 或禁用 failsafe 掩盖问题。

更详细的功能证据见 `SEAD_FUNCTION_VERIFICATION.md`，控制链和历史故障见 `PHASE3_CONTROL_RUNBOOK.md`。
