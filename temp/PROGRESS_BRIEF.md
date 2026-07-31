# xd_uav_sead 移植 — 阶段性工作简报

---

## 一、当前阶段目标

将 [src/SEAD/](../src/SEAD/) 下的 9 文件 Python 多无人机协同 SEAD 系统移植为 xd 风格的 ROS1 catkin 功能包 `xd_uav_sead`，使其能在 PX4 SITL + Gazebo 仿真环境中运行，并**保留原有 XBee 硬件通信能力**。

**核心设计：双模通信架构** — 不删一行硬件代码，在既有 XBee 通路上额外增加 ROS 话题仿真通路。

---

## 二、已完成工作

### 2.1 通信层：XBee → ROS 双模桥接

原始系统完全依赖 Digi XBee S3B 900MHz 数传电台，通过自定义二进制协议进行 GCS↔UAV 和 UAV↔UAV 通信。仿真环境中没有 XBee 硬件，因此开发了 **SeadRosBridge**（[rosbridge.py](src/xd_uav_sead/src/xd_uav_sead/comms/rosbridge.py)），它完整模拟 XBee 的 `send_data_async` / `send_data_broadcast` / `read_data` 接口，内部通过 ROS Topic（`/uavX/sead/command`、`/uavX/sead/telemetry`、`/uavX/sead/u2u`）收发消息，二进制协议序列化格式完全保留不变。

切换方式：`~use_simulation:=true`（或 digi.xbee SDK 未安装时自动启用仿真模式）。

### 2.2 飞控接口层：MAVROS 适配修复

`drone.py`（[drone.py](src/xd_uav_sead/src/xd_uav_sead/drone/drone.py)）是全部飞行控制的枢纽，它封装 MAVROS 接口，需要对 PX4 SITL 正确适配。本阶段修复了三个关键问题：

### 2.3 命令注入工具：mock_gcs

开发 [mock_gcs.py](src/xd_uav_sead/scripts/mock_gcs.py) 作为仿真环境下的虚拟地面站，支持通过命令行参数发送各类控制指令（takeoff / mode / waypoint / sead_mission / formation_config / task_insert 等 16 种命令）。

### 2.4 已验证通过的链路

在 MRS one_drone Gazebo + PX4 SITL 环境下，以下命令端到端收发验证通过：

- info 文本消息的 GCS→UAV→GCS 回传
- freq 遥测频率设置
- airspace_clear 禁飞区清空
- **mode GUIDED → OFFBOARD 切换成功**（PX4 状态机确认）

---

## 文件清单

```
src/xd_uav_sead/
├── CMakeLists.txt, package.xml, README.md
├── config/         sead_defaults.yaml, gps_origin.yaml, sead_planner.yaml
├── launch/         5 个 launch (已验证 roslaunch 通过)
├── scripts/        sead_onboard_node.py, mock_gcs.py
└── src/xd_uav_sead/
    ├── drone/      drone.py           ← MAVROS 接口 + Quad OFFBOARD 分支
    ├── comms/      communication_info.py (XBee 协议, 不改), rosbridge.py (新增)
    ├── planning/   DPGA.py, GA_SEAD_process.py, pathFollowing.py
    ├── formation/  formation_control.py
    ├── strike/     simple_strike.py
    └── airspace/   airspace_manager.py
```

原始 `src/SEAD/` **零改动**。硬件 XBee 代码完整保留。
