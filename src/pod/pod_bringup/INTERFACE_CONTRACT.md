# 产品级接口契约

实际接口值由 `config/interfaces.yaml` 决定，并由 `launch/pod_system.launch` 加载到 `/pod` 参数空间。本文件说明执行规则，不重复维护 Topic 名称。

## 发布权

状态和最终执行命令必须只有一个正常发布者：

| 接口组 | 唯一正常发布者 |
|---|---|
| 相机图像与内参 | `pod_camera_driver` |
| 检测数组 | `pod_detector` |
| 全目标轨迹 | `pod_tracker_adapter` |
| 选中目标 | `pod_tracker_adapter`（任务管理器接入后由其统一编排） |
| 任务状态 | `pod_mission_manager` |
| 最终吊舱命令与控制租约 | `pod_command_arbiter` |
| 吊舱状态 | `pod_gimbal_driver` |
| 飞机归一化状态 | `pod_uav_adapter` |
| 目标地理位置 | `pod_geolocator` |
| 健康与事件聚合 | `pod_gateway` |

`pod_gimbal_controller` 和 UI 不能直接向 `/pod/gimbal/command` 写入。它们分别经内部的 `mission_command` 与 `manual_command` 输入 `pod_command_arbiter`。飞机自动控制默认关闭，且将来必须由 `pod_uav_adapter` 显式选择一个后端。

非紧急吊舱控制必须先经 `/pod/gimbal/manage_control_lease` 获得资源为 `gimbal` 的短时租约；
仲裁器仅转发当前租约持有者的手动或任务命令，安全命令和急停始终优先。任务管理器为
`pod_gimbal_controller` 申请并续期任务云台租约，任务结束后释放它。

`pod_uav_adapter` 是 Follower 到最终飞控后端的唯一产品写入者：它只接受
`command_valid`、授权有效、平台就绪且属于速度控制域的 `FollowerCommand`，并在
`/pod/uav/control_execution` 发布每个指令是否被实际后端接受、执行或安全置零。直接
MAVROS 后端在进入 OFFBOARD 前必须经 `~prepare_follow` 预发送零速度；任务管理器只在
该准备成功后申请 Follower 租约并启动跟随。

飞机自动跟随还必须通过 `/follower_node/manage_control_lease` 获取短时租约后才能
启动。租约需要由任务管理器持续续期；过期、急停、输入时延超限或飞控状态不满足时，
Follower 自动撤销有效指令。`control_output_backend` 只能选择一个后端，产品默认
`command_only`，由 XD/产品适配器继续执行单写入仲裁。

`pod_tracker_adapter` 是产品检测器与 Tracker 的兼容层：它无损将
`/pod/perception/detections` 转发至 `/tracker_node/detection_candidates`，读取
`/tracker_node/track_states` 后在 `/pod/perception/tracks` 发布带稳定 ID、生命周期和
预测状态的全目标列表。检测帧直通仅作为 Tracker 尚未就绪时的兼容降级。

产品点选通过 `/pod/mission/select_target` 进入适配器，再调用
`/tracker_node/select_track`；适配器只桥接产品意图，不回写 Tracker 的输出或控制
Topic。当前由适配器锁存发布 `/pod/target/selected`。后续引入完整
`pod_mission_manager` 时，应由任务管理器编排该服务并保持该公共接口名称不变，不能
形成第二个同名服务或状态发布者。

## 时间与坐标

- 视频、检测、轨迹、吊舱状态和地理结果的 `header.stamp` 使用采集/测量时间，而非界面或网络转发时间。
- `TargetTrack.image_source` 指明产生轨迹的 EO/IR 图像源；UI 以该字段选择叠加视频。
- `TrackState.predicted=true` 或 `measurement_valid=false` 表示当前帧没有检测测量；UI
  应使用预测样式显示，自动控制必须继续服从 Tracker 的测量就绪与协方差门限。
- 机体相对位置使用 `forward/right/down` 语义；世界坐标使用已有 TF2 命名空间链。
- EO/IR 检测必须填写 `DetectionArray.image_source`、`sensor_id`、检测器名称和模型版本。
  Tracker 对跨光谱切换默认拒绝；显式允许切换时会清空旧轨迹，要求任务层重新选择目标，
  禁止将 EO 的 ID 无条件继承到 IR。
- 启用 `pod_tracker_adapter` 的 body-LOS 补偿后，Follower 必须订阅
  `/pod/perception/body_normalized_error`。该接口将相机残差经实时吊舱姿态和安装外参转换
  到机体 FRD；吊舱状态过期时自动撤销控制测量资格。
- `tracker` 与 `follower` 也直接订阅 `/pod/gimbal/state`。Tracker 可在产品启动中因
  失效、限位、故障或超时撤销 body-LOS 角度；Follower 将同一状态作为飞行控制安全门。
  Follower 不再次旋转已经由 Adapter 补偿的 body-LOS，避免双重补偿。
- 厂商桥的输入/输出统一位于 `/pod/vendor/*`。`pod_camera_driver` 保留图像和 CameraInfo
  的采集时间，`pod_detector` 规范化 EO/IR 检测来源，`pod_gimbal_driver` 规范化云台状态
  和最终命令。真实协议适配只能在该边界内实现。
- HIL 使用 `pod_hil_validation` 的 `/pod/hil/run_preflight`。它只检查状态、坐标系、
  检测时间戳与 EO/IR 来源，不发布任何执行命令。
- `GeoTarget.valid=false` 时只能显示视线或无效状态，不能将该位置用于自动指向或任务决策。

## 未实现接口的行为

配置并不会伪造视频、检测、吊舱或 UAV 数据。节点在尚未实现或未连接硬件时应发布明确的诊断/事件，而非发布虚构的正常状态。
