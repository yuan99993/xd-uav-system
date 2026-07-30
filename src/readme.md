# Tracker / Follower ROS1 功能包说明

## 1. 功能概览

本工作区包含两个面向 PX4、Gazebo 和 MRS 的 ROS1 Noetic 功能包：

- `tracker`：接收外部目标检测、特征点或 ROI 信息，完成稳定目标跟踪，并向下游输出归一化图像误差。
- `follower`：接收 `tracker` 的误差和 PX4/MAVROS 状态，计算机体系速度与偏航角速度指令；可直接发布给 MAVROS，也可发布给 MRS `control_manager`。

系统不负责 PX4 的解锁、OFFBOARD 切换或任务起飞。它只提供安全限幅后的跟踪速度建议与标准 ROS 接口，飞控状态机和最终执行权限应由控制器/MRS 系统管理。

## 2. 总体数据流

```text
外部检测器 / 特征识别 / 手工输入
        |
        | tracker/ExternalInput
        v
tracker_node
  - 输入统一化
  - 目标关联、滤波、短时预测
  - 计算归一化图像误差
        |
        | /tracker_node/normalized_error
        | tracker/NormalizedError
        v
follower_node <----- MAVROS/PX4 telemetry
  - PID 制导、速度爬升、平滑与安全限制
  - 目标丢失保护
        |
        +--> /mavros/setpoint_velocity/cmd_vel
        |    geometry_msgs/TwistStamped
        |
        +--> /uavX/control_manager/velocity_reference_in
        |    mrs_msgs/VelocityReferenceStamped
        |
        +--> /follower_node/controller_feedback
             follower/ControllerFeedback
```

## 3. Tracker 原理与算法

`tracker` 的角色是将不同来源的感知结果转换成稳定、统一的跟踪结果，而不是绑定某一种检测网络。上游可使用 YOLO、OpenCV、深度网络、特征点匹配或人工选框，只要将结果发布为 `tracker/ExternalInput` 即可。

核心处理链路如下：

1. **输入统一化**：支持像素边界框、归一化边界框、归一化特征点与 ROI；统一转换为内部检测表示。
2. **混合目标关联**：使用目标 ID、空间 IoU 和中心距离进行关联，在检测器目标 ID 短暂丢失时尽量维持同一目标。
3. **卡尔曼状态估计**：对边界框位置和尺度进行预测/校正，降低检测框抖动。
4. **运动预测**：使用 EMA 平滑的速度、加速度进行短时遮挡预测；同时提供受 EGO-Planner 思路启发的 B-spline 轨迹预测模块，用于更连续的长期运动趋势估计。
5. **误差计算**：以图像中心为期望位置，输出：
   - `error_x`：水平误差，范围 `[-1, 1]`；负数表示目标在左，正数表示目标在右。
   - `error_y`：垂直误差，范围 `[-1, 1]`；负数表示目标在上，正数表示目标在下。
   - `error_size`：目标尺寸误差；负数表示目标过小/较远，正数表示目标过大/较近。
6. **有效性标志**：输出 `error_valid`、`target_visible`、`is_estimated` 与 `dt_since_detection`，下游可区分真实检测、短时预测和失效数据。

### Tracker 主要接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 输入 | `/tracker_node/external_input` | `tracker/ExternalInput` | 外部边界框、特征点、ROI、置信度、类别和 `start_track`/`stop_track`/`reset` 命令 |
| 输出 | `/tracker_node/tracking_output` | `tracker/TrackingOutput` | 完整跟踪状态、边界框、速度、角度误差等 |
| 输出 | `/tracker_node/normalized_error` | `tracker/NormalizedError` | 供 follower 使用的归一化误差 |

`ExternalInput` 的关键字段：

```text
source: "bounding_box" | "feature_point" | "roi" | "external_detector"
bbox: [x1, y1, x2, y2]                  # 像素坐标
normalized_bbox: [cx, cy, width, height] # 范围 [0, 1]
feature_point: [x, y]                    # 范围 [0, 1]
feature_velocity: [vx, vy]               # 可选
confidence: 0.0 ~ 1.0
class_id: 类别 ID
command: "start_track" | "stop_track" | "reset" | ""
```

## 4. Follower 原理与算法

`follower` 基于 PixEagle 的速度追踪逻辑实现，输入为 `tracker` 输出的归一化误差，输出为机体系速度指令：

```text
body frame:
x = forward（前）
y = right（右）
z = down（下）
```

默认的 `coordinated_turn` 制导模式：

1. `error_x` 经过偏航 PID，得到偏航角速度。
2. 偏航角速度经过死区、变化率限制、EMA 平滑和低速缩放，降低抖动。
3. `error_y` 经过垂直 PID，得到 `velocity_down`。
4. 前向速度由 `VelocityRamper` 根据设定斜率逐步提升或在目标丢失时减速。
5. 可选的俯仰补偿修正由飞机前飞姿态造成的图像垂直偏差。
6. 可选的自适应俯冲/爬升模块根据目标垂直像素运动速率进行附加修正。
7. 受 EGO-Planner 可行性约束思路启发的速度、加速度和加加速度限制，与总速度、高度包络、偏航速率限制共同保证指令可执行。
8. 目标误差过期、检测无效、目标丢失或紧急停止时，输出会收敛至安全指令。

`sideslip` 模式可替代偏航制导：水平误差由横向 PID 产生 `velocity_right`，适合允许侧向机动的平台。

### Follower 的输入

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 输入 | `/tracker_node/normalized_error` | `tracker/NormalizedError` | tracker 输出的目标误差 |
| 输入 | `<mavros_ns>/imu/data` | `sensor_msgs/Imu` | 姿态/俯仰补偿 |
| 输入 | `<mavros_ns>/state` | `mavros_msgs/State` | 飞控状态 |
| 输入 | `<mavros_ns>/altitude` | `mavros_msgs/Altitude` | 相对高度与安全限制 |
| 输入 | `<mavros_ns>/local_position/pose` | `geometry_msgs/PoseStamped` | 本地位置 |
| 输入 | `<mavros_ns>/local_position/velocity_local` | `geometry_msgs/TwistStamped` | 实际速度与 RViz 可视化 |
| 输入 | `/follower_node/controller_command` | `follower/ControllerCommand` | 下游控制器的限速、执行反馈或覆盖指令 |

`<mavros_ns>` 默认是 `/mavros`，多机环境可配置为 `/uav1/mavros`、`/uav2/mavros` 等。

### Follower 的输出

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 输出 | `/follower_node/follower_command` | `follower/FollowerCommand` | 完整机体系速度、偏航速率及诊断 |
| 输出 | `/follower_node/follower_status` | `follower/FollowerStatus` | 跟随、目标丢失、高度安全等状态 |
| 输出 | `/follower_node/controller_feedback` | `follower/ControllerFeedback` | 给外部控制器的速度建议与有效性 |
| 输出 | `<mavros_ns>/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | 直接 PX4/MAVROS 速度接口，可关闭 |
| 输出 | `/<uav>/control_manager/velocity_reference_in` | `mrs_msgs/VelocityReferenceStamped` | MRS 控制器速度输入，可关闭 |
| 输出 | `/follower_node/velocity_command_odom` | `nav_msgs/Odometry` | RViz 指令速度显示 |
| 输出 | `/follower_node/velocity_command_marker` | `visualization_msgs/Marker` | RViz 指令箭头 |
| 输出 | `/follower_node/actual_velocity_marker` | `visualization_msgs/Marker` | RViz 实际速度箭头 |

### 与控制器的协作接口

控制器应订阅：

```text
/follower_node/controller_feedback
```

需要向 follower 回传执行状态、额外限速或临时覆盖时，发布：

```text
/follower_node/controller_command
```

其中 `ControllerCommand.velocity_limits` 的顺序为：

```text
[vx_min, vx_max, vy_min, vy_max, vz_min, vz_max]
```

即使控制器使用 `override_active` 覆盖指令，follower 仍会再次应用本地速度幅值、高度和偏航安全限制。

## 5. PX4、Gazebo、RViz 与 MRS 通信

### 5.1 坐标与符号转换

Follower 内部使用机体系 FLU/FRD 约定中的控制语义：前、右、下。MAVROS 本地速度反馈是 ENU，因此接收反馈时会将 `z` 取反转换为内部 NED 下向速度。

发布给 MAVROS 或 MRS 时：

```text
TwistStamped.linear.x / VelocityReference.velocity.x = velocity_forward
TwistStamped.linear.y / VelocityReference.velocity.y = -velocity_right
TwistStamped.linear.z / VelocityReference.velocity.z = -velocity_down
```

因此不要在外部控制器中再次对右向或下向速度重复取反。

### 5.2 PX4/Gazebo 直连模式

用于 SITL、Gazebo 或没有 MRS control_manager 的环境：

```text
tracker -> follower -> /mavros/setpoint_velocity/cmd_vel -> MAVROS -> PX4
```

此时启用 MAVROS 输出，关闭 MRS 输出。

### 5.3 MRS 协作模式

用于已有 MRS 控制器的环境：

```text
tracker -> follower -> /uavX/control_manager/velocity_reference_in -> MRS control_manager -> PX4
```

此时建议启用 MRS 输出并关闭直接 MAVROS 输出，避免两个节点同时向飞控下发速度指令。

### 5.4 RViz 检查

建议订阅以下话题：

```text
/follower_node/velocity_command_odom
/follower_node/velocity_command_marker
/follower_node/actual_velocity_marker
```

`velocity_command_marker` 是 follower 计算出的速度，`actual_velocity_marker` 来自 MAVROS 的真实速度反馈。若 Gazebo 中无人机未被解锁、未进入 OFFBOARD 或没有控制器实际执行速度指令，命令箭头可以变化而实际速度箭头和模型位置保持不变，这是正常现象。

## 6. 新环境部署

### 6.1 前置条件

目标环境应具备：

- Ubuntu 20.04
- ROS Noetic
- `catkin_tools`
- `rospy`、`geometry_msgs`、`sensor_msgs`、`nav_msgs`、`visualization_msgs`
- `mavros`、`mavros_msgs`
- MRS 模式额外需要 `mrs_msgs`，且其中必须存在 `mrs_msgs/VelocityReferenceStamped`

将 `tracker`、`follower` 放入目标工作区的 `src/` 目录；MRS 输出模式下也必须保证该工作区或其 overlay 中可找到 `mrs_msgs`。

### 6.2 一键检查并生成适配配置

```bash
source /opt/ros/noetic/setup.bash
cd ~/your_ros_workspace

bash src/tracker/scripts/deploy_mrs_adapter.sh --verify
```

脚本将构建 `tracker` 与 `follower`，检查 ROS 包和消息类型，并生成：

```text
<workspace>/.mrs_adapter/follower_adapter.yaml
```

该文件是机器本地的接口映射，不会覆盖 `follower/config/follower_params.yaml` 中的算法参数。

### 6.3 部署到 MRS 控制器环境

下面示例假设无人机命名空间为 `uav1`，MAVROS 位于 `/uav1/mavros`：

```bash
cd ~/your_ros_workspace

bash src/tracker/scripts/deploy_mrs_adapter.sh \
  --uav-name uav1 \
  --mavros-ns /uav1/mavros \
  --enable-mavros-output false \
  --enable-mrs-output true \
  --verify
```

通过后可一键启动 tracker 和 follower：

```bash
bash src/tracker/scripts/deploy_mrs_adapter.sh \
  --uav-name uav1 \
  --mavros-ns /uav1/mavros \
  --enable-mavros-output false \
  --enable-mrs-output true \
  --launch
```

对于 `uav3`，只需将 `--uav-name uav3`，`--mavros-ns /uav3/mavros` 一并替换即可。

### 6.4 部署到 PX4 SITL/Gazebo 直连环境

默认配置使用 `/mavros`，只发布 MAVROS 速度指令：

```bash
cd ~/your_ros_workspace
bash src/tracker/scripts/deploy_mrs_adapter.sh --launch
```

或者先生成配置，再分别启动：

```bash
source /opt/ros/noetic/setup.bash
source ~/your_ros_workspace/devel/setup.bash

roslaunch tracker tracker.launch

roslaunch follower follower_mrs_adapter.launch \
  adapter_config:="$HOME/your_ros_workspace/.mrs_adapter/follower_adapter.yaml"
```

组合启动图等价于：

```bash
roslaunch follower tracker_follower_mrs_adapter.launch \
  adapter_config:="$HOME/your_ros_workspace/.mrs_adapter/follower_adapter.yaml"
```

## 7. 构建、测试与运行检查

### 7.1 构建和基础语法检查

```bash
source /opt/ros/noetic/setup.bash
cd /home/promise/mrs_test
catkin build tracker follower

source devel/setup.bash
python3 -m py_compile \
  src/tracker/scripts/tracker_node.py \
  src/follower/scripts/follower_node.py

python3 src/tracker/test/test_tracker_core.py
python3 src/follower/test/test_follower_core.py
```

### 7.2 Tracker 外部输入测试

先启动 tracker 或组合启动图：

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/mrs_test/devel/setup.bash
roslaunch tracker tracker.launch
```

在另一终端执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/mrs_test/devel/setup.bash
rosrun tracker test_external_input.py
```

检查 tracker 输出：

```bash
rostopic echo /tracker_node/normalized_error
```

目标正常被检测时，重点观察：

```text
error_valid: true
target_visible: true
is_estimated: false
```

`error_x`、`error_y`、`error_size` 随测试边界框变化而变化是正常现象。

### 7.3 Follower 运行与状态检查

```bash
rosservice call /follower_node/start "data: true"

rostopic echo /follower_node/follower_status
rostopic echo /follower_node/follower_command
rostopic echo /follower_node/velocity_command_odom
```

停止或紧急停止：

```bash
rosservice call /follower_node/stop "data: true"
rosservice call /follower_node/emergency_stop "data: true"
```

### 7.4 接口连通性检查

```bash
rostopic info /tracker_node/normalized_error
rostopic info /follower_node/follower_command
rostopic info /follower_node/controller_feedback
rostopic info /follower_node/velocity_command_odom

# PX4/MAVROS 模式
rostopic info /mavros/setpoint_velocity/cmd_vel

# MRS 模式，以 uav1 为例
rostopic info /uav1/control_manager/velocity_reference_in
```

## 8. 关键配置文件

| 文件 | 作用 |
|---|---|
| `tracker/config/tracker_params.yaml` | tracker 参数模板 |
| `follower/config/follower_params.yaml` | PID、速度、高度、目标丢失和安全参数 |
| `follower/config/mrs_adapter_defaults.yaml` | 默认 MAVROS/MRS 端口映射 |
| `.mrs_adapter/follower_adapter.yaml` | 部署脚本生成的本机端口映射 |
| `follower/launch/follower_mrs_adapter.launch` | 加载算法参数和端口映射 |
| `follower/launch/tracker_follower_mrs_adapter.launch` | 同时启动 tracker 与 follower |

调整算法增益和安全边界时修改 `follower/config/follower_params.yaml`。调整无人机名称、MAVROS 命名空间或 MRS 速度输入话题时，优先重新运行 `deploy_mrs_adapter.sh`，不要修改算法配置文件。
