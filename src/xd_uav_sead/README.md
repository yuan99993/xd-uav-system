# xd_uav_sead

`xd_uav_sead` 是原始 `catkin_ws/src/SEAD` 的 ROS1 Noetic 功能包移植。原有 XBee、任务协议、DPGA、路径跟随、编队、禁飞区和简化同步打击能力应继续保留；ROS 话题通信和仿真支持是附加路径，不得替代或破坏硬件路径。

## 代码映射

| 原始文件 | ROS 包位置 |
|---|---|
| `onboard.py` | `scripts/sead_onboard_node.py` |
| `drone.py` | `src/xd_uav_sead/drone/drone.py` |
| `communication_info.py` | `src/xd_uav_sead/comms/communication_info.py` |
| `DPGA.py`、`GA_SEAD_process.py`、`pathFollowing.py` | `src/xd_uav_sead/planning/` |
| `formation_control.py` | `src/xd_uav_sead/formation/` |
| `simple_strike.py` | `src/xd_uav_sead/strike/` |
| `airspace_manager.py` | `src/xd_uav_sead/airspace/` |

`src/xd_uav_sead/comms/rosbridge.py` 和 `scripts/mock_gcs.py` 是新增的无 XBee 仿真适配层。
原始硬件编队发送工具保留在 `tools/xbee_send_formation_test.py`。

## 通信模式

### ROS 仿真模式

```bash
UAV_NAME=uav1 roslaunch xd_uav_sead sead_onboard.launch
```

默认 launch 设置 `use_simulation=true`：

- GCS 命令：`/uavX/sead/command`，`std_msgs/String` JSON
- GCS 遥测：`/uavX/sead/telemetry`，`std_msgs/String` JSON envelope
- 机间总线：`/sead/u2u`，包含 `src`、`dst`、`broadcast` 和 base64 二进制协议数据

每架机必须使用唯一的协议 `uav_id`。默认映射兼容 `uav0→1` 和 MRS 风格 `uav1→1`；多机或混合命名时应在 launch 中显式设置 `uav_id`。

### XBee 硬件模式

将私有参数 `use_simulation` 设为 `false`。节点会沿用原始行为：扫描 `/dev/ttyUSB*` 和 `/dev/ttyACM*`、匹配 XBee node ID、持有设备锁、构造真实 `RemoteDigiMeshDevice` 地址并执行时间同步。

硬件依赖 Digi XBee Python SDK。未经真实设备验证，不应声称硬件链路已经通过端到端测试。

## 配置

- `config/sead_defaults.yaml`：运行模式、XBee 和编队配置
- `config/sead_planner.yaml`：GA、Dubins、空域和打击默认值
- `config/gps_origin.yaml`：ENU/LLA 原点

标准 launch 会加载这三个文件。GA 的 `population_size`、`time_interval` 和 FormationConfig 的主要字段已接入运行代码；任务报文中显式提供的飞行速度、转弯半径和航点半径仍优先作为任务输入。

## Mock GCS 示例

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=mode _mode:=GUIDED
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=waypoint _x:=100 _y:=50 _z:=80
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=sead_mission \
  _targets_json:='[[100,0],[200,50],[300,100]]'
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=formation_config \
  _shape:=VEE _spacing:=220
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_zone \
  _zone_id:=1 _points_json:='[[0,0],[100,0],[100,100],[0,100]]'
```

## PX4/Gazebo 与控制器边界

本包的 launch 不负责启动 PX4 或 Gazebo。外部可以使用 MRS 启动链，也可以使用经验证的独立 PX4 SITL + Gazebo 链。

`Drone` 保留两种互斥后端：`direct_mavros` 保留原始路径，`xd_control_manager` 只向 `/uavX/control/reference/setpoint` 提交期望，由 manager 负责 OFFBOARD、解锁、起飞和降落。不得让两条路径同时向 PX4 输出。

已有 PX4、MAVROS 和 Gazebo 时，启动仓库控制链与 SEAD：

```bash
UAV_NAME=uav1 roslaunch xd_uav_sead sead_xd_control.launch
```

该 launch 会启动 `xd_uav_state_estimators`、`xd_uav_control_manager`/`xd_uav_controller` 和 SEAD。如果借用 MRS 仿真，只启动 Gazebo/spawner 部分；不要同时启动 `mrs_uav_core`，因为两套 manager 会占用同一 `/uavX/control_manager` 命名空间。

manager 后端下，SEAD waypoint 是 `uavX/odom` 中的本地 ENU 米制坐标；`z <= 0` 表示保持当前高度。直连 MAVROS 后端仍保留原有原点转换语义，但在未收到有效 home position 时会拒绝航点。

`config/xd_control_x500.yaml` 是 MRS Gazebo x500 的仿真专用覆盖；其 hover throttle 和触地阈值不应直接套用于其他机型或真机。

完整的人工起飞—航点—降落操作、安全收尾和已知问题见 [`docs/PHASE3_CONTROL_RUNBOOK.md`](docs/PHASE3_CONTROL_RUNBOOK.md)。

`sead_gazebo_demo.launch` 默认只启动 SEAD 节点，不自动发送起飞命令。仅在明确需要且外部仿真没有自动起飞逻辑时设置 `send_takeoff:=true`。

## 依赖与验证

ROS 依赖见 `package.xml`。其他 Python 运行依赖包括 `pymap3d`、`dubins` 和硬件模式使用的 `digi.xbee`；安装前应确认使用 ROS Noetic 的系统 Python 3 环境。

协议回归测试：

```bash
PYTHONDONTWRITEBYTECODE=1 nosetests3 -v \
  src/xd_uav_sead/test/test_rosbridge_protocol.py
```

测试覆盖基础 GCS 命令、SEAD mission、Swarm、Airspace Zone、U2U 过滤以及 GCS/UAV 路由。端到端飞行、真实 XBee 和多机任务仍需分别验证。
