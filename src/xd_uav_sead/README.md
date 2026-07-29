# xd_uav_sead

分布式多无人机 SEAD 任务控制包。通过 XBee 无线通信实现无人机间协同，集成 GA 任务分配、Dubins 路径跟随与禁飞区避让。

## 功能模块

| 模块 | 文件 | 职责 |
|---|---|---|
| 机载节点 | `scripts/sead_onboard_node.py` | 主循环：XBee 通信 → 命令解析 → 任务执行 |
| 无人机接口 | `drone.py` | mavros 通信：订阅状态/位姿，发布 setpoint_raw，调用服务 |
| 通信协议 | `communication_info.py` | XBee 二进制打包/解包 + 所有 Enum 定义 |
| GA 任务分配 | `DPGA.py` | `task_allocation_process` + `main_process`（控制循环） |
| GA 引擎 | `GA_SEAD_process.py` | GA 种群优化 + Dubins 碰撞检测 + 避让路径规划 |
| 路径跟随 | `pathFollowing.py` | CraigReynolds 路径跟随 + PID/LQR 控制器 |
| 禁飞区 | `airspace_manager.py` | AirspaceManager：禁飞区存取与导出 |
| 编队控制 | `formation_control.py` | FormationController：TRAIL/VEE 等队形 |
| 简化打击 | `simple_strike.py` | SimpleStrikeManager：3 目标简化打击（无 GA） |

## 外部依赖

- ROS: `rospy`, `mavros_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_srvs`, `tf`
- Python: `numpy`, `scipy`, `pymap3d`, `dubins`, `matplotlib`
- 硬件: XBee S3B 900MHz 无线模块（`digi.xbee` SDK）
- 仿真: ROS 话题 `/uavX/sead/command` / `/uavX/sead/telemetry`（SeadRosBridge 虚拟化，无需 XBee 硬件）

## 启动

```bash
UAV_NAME=uav1 roslaunch xd_uav_sead sead_onboard.launch
```
