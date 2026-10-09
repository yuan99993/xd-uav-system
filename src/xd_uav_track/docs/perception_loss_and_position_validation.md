# 第四、第五项整改与验收

日期：2026-10-01。按要求先完成第四项，29 项单元回归和真实 ROS 节点集成测试通过后，
才开始第五项。未修改 YOLO 识别权重、身份关联策略或飞行控制输出字段。

## 第四项：禁止旧框伪装锁定

- SOT 更新失败只返回无效结果，不再把上一帧框作为结果输出。
- 内部最后框可供恢复校验；对外 lost/stale 只保留身份，框、归一化框和像素中心清零，
  位置无效。不会重新编号，也不会将保留身份解释成仍有真实位置。
- 真实 Kalman 图像预测只能在 occluded 状态、有前序观测且未超过预测期限时展示。
  `~prediction_horizon_sec` 默认 0.5 秒，可设置 0～2 秒；最终期限包含传输延迟。
- overlay 明确显示 DET/SOT/PRED/LOST。PRED 为虚线框、空心中心；LOST 仅显示状态栏
  `LOST ID`，不画旧位置框。相机停更后主动发出一次清除快照。

## 第五项：位置来源与时间真实性

`TrackState` 新增字段：

| 字段 | 语义 |
|---|---|
| `last_observation_stamp` | 最后一次接受的真实图像观测时间，预测不更新 |
| `position_frame` | 机体 FRD 位置坐标系，独立于图像 header |
| `position_stamp` | 最后接受的真实度量定位时间，图像-only/预测/失联/跳点保持不更新 |

`body_frd_frame` 同时适配四旋翼和固定翼顶层入口，默认 `<UAV_NAME>/base_link_frd`。
不对 detector 已转换成 FRD 的位置再次翻转符号，不改变旧 `frames/body` 的 FLU 控制语义。

统一目标输出：

- `position_frame/position_stamp` 来自位置专属元数据，不取相机 header 或当前预测消息时间。
- 位置有效要求确认轨迹、非预测、测量可用、range 有效、配置 frame 一致、定位与原图时间
  一致、未超时，以及有限位置和对称半正定协方差。
- 纯 SOT 不输出假三维测量；预测/失联只可保留历史定位标签供诊断，必须位置无效。
- `age_ms` 计入发布延迟；未知观测时间为 -1，不伪装成 0。
- 物理跳点被保持时，保留原定位时间并撤销本帧 range 有效性，不将保持点冒充新测距。

## 当前版本回归结果

- Python：38 项模式、选帧、SOT、失锁显示、位置元数据及共享检测器测试通过。
- C++：21 项 MultiTrackManager、14 项 TrackletStitcher 测试通过。
- launch：1 项测试覆盖四旋翼/固定翼两种配置，确认 tracker 与模式管理器使用相同 FRD frame。
- 三个包 `xd_uav_track / sar_yolo_detector / xd_uav_detect` 低并发构建成功。
- 扩展的 `perception_visual_identity.test` 通过：实际 C++/Python ROS 节点验证前三项身份
  行为未回退；新增真实图像消失后的 lost/stale 清零框、FRD 位置标签/定位时间、纯 SOT
  不复用旧三维定位等断言。ROS 用例限时 60 秒，外层限时 90 秒。
- 测试节点及 ROS master 均已退出，无遗留测试后台进程。

合计 74 项单元/launch 校验，另有 1 项多断言 ROS 集成用例。
ROS 测试使用合成图像、模拟 detector 定位与 odometry；未启动 Gazebo/PX4/YOLO/GPU，
不能用本记录替代实拍精度、实际 TF/标定、长遮挡或实机飞行验收。

## 部署兼容注意

新增 TrackState 字段改变该消息及其嵌套消息/服务的 ROS MD5。相关节点必须同步生成
新消息并重新编译、重启；网页订阅端也应刷新消息描述。最终 PerceptionTarget 字段及控制
命令字段未改。旧 TrackState rosbag 需转换，或使用旧版本隔离回放，不能混接新节点。

运行位置参数：

- tracker：`frames/body_frd`；
- 模式管理器：`~body_frd_frame`、`~position_timeout_sec`（默认 1 秒）；
- 预测展示：`~prediction_horizon_sec`；相机停更：`~source_timeout_sec`。

部署仍需真实发布、标定正确的 FRD TF 链路。坐标标签正确不代表外参自动完成标定。
