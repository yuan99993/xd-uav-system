# xd_uav_detect Gazebo 模型资产

本目录统一保存 detect 演示和载荷集成参考所需的四个模型，不替换现有 MRS/PX4 启动链的
模型路径：

| 目录 | 来源/用途 |
|---|---|
| `x500` | 当前 MRS/PX4 `x500` Jinja 在本机版本上渲染的 SDF 快照及完整 x500v2 网格 |
| `plane` | 当前 PX4 `plane` 模型的原样快照 |
| `x500_gimbal` | 完整 MRS/PX4 x500 飞行模型加 yaw/pitch、Gazebo camera 和单束 ray/range |
| `plane_gimbal` | 完整 PX4 plane 飞行模型加相同光电载荷 |

两个吊舱版本共用 `common/eo_gimbal.sdf.jinja` 这一份手写载荷定义。x500 与 plane 派生模板只
给出 parent link、安装位、模型资源 URI 和 namespace profile；Camera、Range、双轴机构、PID、
传感器开关与未来 roll 扩展点均由公共宏生成。四个模型 URI 与已渲染运行时 SDF 继续保留。

两个吊舱模型的可见外壳复用 PX4 Gazebo Classic 的 CGO3 云台网格：mount、vertical arm、
horizontal arm 和 camera 分别绑定到固定安装座、yaw link 与 pitch link。网格只用于渲染；
简化惯性、两轴关节和传感器定义保持独立，避免让精细外观影响 Gazebo 碰撞和飞行稳定性。
相机与单束激光都定义在 `gimbal_pitch` link 上，所以 yaw/pitch 关节带动的是完整传感器载荷。

来源版本：

- MRS x500：`mrs_uav_gazebo_simulator` commit
  `020cdcb0268fb1f98c128181355f8d4a827798a9`；原始模板保存在
  `x500/upstream_x500.sdf.jinja`。
- PX4 plane：`PX4-Autopilot` commit
  `46a12a09bf11c8cbafc5ad905996645b4fe1a9df`。
- CGO3 外观：PX4 Gazebo Classic `cgo3_camera` 模型的四个 remeshed STL；本包只复用外观，
  关节、惯性、传感器和 ROS 接口由本包定义。

两个上游项目均使用 BSD-3-Clause；许可证副本位于本目录。更新上游时应重新生成/复制基础模型，
再重新应用并验证吊舱增量，不能假定本地快照会自动同步。

让 Gazebo 发现这些模型：

```bash
export GAZEBO_MODEL_PATH="$(rospack find xd_uav_detect)/models:${GAZEBO_MODEL_PATH}"
```

之后可在 world 中使用 `model://x500`、`model://plane`、`model://x500_gimbal` 或
`model://plane_gimbal`。`demo.launch` 已自动设置此路径。

四个模型均保留上游飞行插件和非静态机体。`x500_gimbal/model.sdf` 相对 `x500/model.sdf`
只改变模型/资源 URI 并增加两个吊舱 link、两个 revolute joint、camera 和 ray/range；旋翼动力、
IMU、GPS、MAVLink、ground truth 与流体阻力插件均保留。`plane_gimbal/plane.sdf` 也保留原有
升阻力、舵面、推进器、IMU、导航和 MAVLink 插件。两种吊舱模型均没有世界固定关节。

`model.sdf`/`plane.sdf` 是按当前单机默认端口渲染的可运行快照；对应的
`x500_gimbal.sdf.jinja`/`plane_gimbal.sdf.jinja` 是保留上游参数入口的派生模板。演示 launch
在零重力、无 PX4 条件下加载同目录的 `sensor_demo.sdf`：它保留相同 x500 机体、惯性、实体
云台和传感器，仅移除需要飞控配合的插件并通过 `sensor_demo_world_anchor` 固定机体。该文件是
明确标识的 test fixture，不是第五个模型，也不能用于飞行；正式模型始终是 `model.config`
指向的 `model.sdf`。

修改公共载荷宏后，用以下命令刷新并核验三个运行时快照：

```bash
rosrun xd_uav_detect render_payload_models.py --write
rosrun xd_uav_detect render_payload_models.py
```

第二条是只读漂移检查，快照与 `models/common/eo_gimbal.sdf.jinja` 不一致时返回非零。新增飞机
派生模型时应调用同一 `eo_gimbal()` 宏，只增加飞机侧安装 profile；不要复制宏展开后的 link、
sensor 或 plugin。`camera_enabled`、`range_enabled` 可分别关闭载荷设备；`roll_enabled` 已预留
三轴机构生成路径，但当前控制消息和交付模型仍只启用 yaw/pitch。
两个 `*_gimbal` 模型的 detect 标准接口为：

- `/uav1/gimbal_camera/image_raw`
- `/uav1/gimbal_camera/camera_info`
- `/uav1/gimbal/range`
- frame `uav1/gimbal_camera_optical` 与 `uav1/gimbal_laser`
- joint `gimbal_yaw_joint` 与 `gimbal_pitch_joint`

两个 `*_gimbal` 模型还提供当前 Gazebo 控制后端：

- `/uav1/joint_states`：yaw/pitch 实际角与速度；
- `/uav1/gimbal/set_joint_trajectory`：包内控制节点到自有 Gazebo 关节 PID 插件的内部轨迹话题；
- `/uav1/gimbal/command`、`/uav1/gimbal/state`：运行 `gimbal_control.launch` 后供外部使用的
  稳定公共接口。

当前自由度为 yaw + pitch。它覆盖水平与俯仰指向；roll 地平线稳定留作以后按需求扩展。控制
配置在 `config/gimbal_control.yaml`，相机和激光定位仍只依赖标准消息与 TF。x500 吊舱经过两次
人工外观确认累计上移 4 cm，当前安装座、yaw link 和 pitch link 的 z 分别为 `0.02`、`-0.08`
和 `-0.1441` m；plane 安装位未随该调整改变。

执行插件 `libxd_uav_detect_gimbal_joint_controller.so` 使用 Gazebo `JointController` 的 PID
目标，不暂停世界、不直接写整机位姿。曾短暂试用的
`libgazebo_ros_joint_pose_trajectory.so` 在完整飞行模型中会切换全局物理状态并导致 PX4/MRS
状态估计跳变，已撤销；传感器固定夹具通过不能替代飞行稳定性验收。

真实飞行仿真接入时还需由载体/云台驱动发布拍摄时刻的 `body <- gimbal_laser` TF，并处理模型
命名空间、MAVLink 端口和多机隔离。

特别注意：当前 MRS spawner 将模板名同时作为 `PX4_SIM_MODEL`，并从模板所属 ROS 包寻找
`ROMFS`；MRS core 的配置文件也由 `UAV_TYPE` 拼接。因此把本目录加入 `extra_resource_paths`
后直接执行 `--x500_gimbal` 仍会失败。

旋翼单机通过 `launch/demo.launch mode:=px4` 绕过 spawner：它直接加载本目录规范 SDF，使用
PX4 instance 1，但把 PX4 airframe 保持为已有 `x500`，并启动对应 MAVROS。该入口已经验证
PX4 simulator TCP、MAVROS heartbeat、Gazebo 有限物理状态、Range 和云台关节状态；它不是
MRS core/控制器/自动起飞集成。固定翼吊舱模型仍只交付模型和标准传感器接口，不扩展到固定翼
云台或航向控制。

`launch/demo.launch mode:=flight` 在上述底座上额外复用现有 MRS core 和 autostart，提供
自动起飞以及 `/uav1/control_manager/reference` 位置参考入口，并同时运行吊舱定位演示。该入口
同样直接加载本目录 SDF，不调用或修改 spawner。

两个云台轴使用 `0.01 N·m·s/rad` 被动阻尼。此前的 `0.2` 相对 `1e-4/2e-4 kg·m²` 转动惯量
过大，PX4 actuator 微小扰动会令 Gazebo 数值发散；当前数值已通过基础 x500、无 PX4 吊舱版
和 PX4 联通吊舱版三组对照验证。它是仿真稳定参数，不代表真实云台控制器参数。
