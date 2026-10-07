# 高频跟踪升级的非退化验收

本次按“独立像素辅助 → 相机运动补偿 → 融合质量裁决 → TensorRT
数据路径”顺序实施；每阶段未完成可比回归，不进入下一阶段或替换默认路径。

## 最新无假山场景纠正与空窗采样（2026-10-06）

用户要求最新地图。此前 `tracking_benchmark.world` 是含假山的历史双坦克
基线，只能作为历史复测，不能代表最新演示场景。本次已改用
`perception_gazebo_web_test.launch` → `moving_uav_vehicle_search_demo.launch`
→ `moving_uav_vehicle_search_gimbal.world`。运行时调用 Gazebo
`get_world_properties` 确认模型仅包括 `concrete_ground`、
`search_quadrotor`、`search_gimbal_camera` 和 alpha/bravo/charlie 三辆车；
没有假山或旧遮挡块。世界 SHA-256：
`c3c13f223896c3c1e4bb769368ad2b5deae06cc8d831e68edb6a8a221a9c24d5`。

修正共享检测器限频为固定相位截止时刻推进，避免相机帧抖动导致每次
从当前时刻重新计时而累积延迟；丢失的时槽不会排队补发，切换帧率、
模式重置会清理截止时刻。没有提高配置的 YOLO 帧率，没有修改模型或
新轨迹/身份门限。网页仿真入口增加实验开关转发，仍默认关闭。

同一最新世界、train7 TensorRT、固定相机 3 FPS / 云台 2 FPS 检测上限，
两次顺序运行，每次采样约 45 秒；无 Gazebo GUI、无 rqt 窗口，保留网页
视频编码。两组都使用新的限频修复，区别仅为高频像素辅助开关。

| 节奏指标 | 旧像素路径 | 独立像素辅助 |
| --- | --- | --- |
| 固定相机目标输出最大帧间隔 | 0.74 s | 0.50 s |
| 固定相机目标输出 P95 帧间隔 | 0.51 s | 0.49 s |
| 固定相机目标输出平均采集频率 | 2.91 Hz | 3.50 Hz |
| 云台目标输出最大帧间隔 | 1.00 s | 0.50 s |
| 云台目标输出 P95 帧间隔 | 0.75 s | 0.49 s |
| 应用内图像回调到目标发布 P95 | 141.42 ms | 85.49 ms |

候选路径：工作线程收到/处理 370 帧，丢弃 0、异常 0，处理 P95
11.15 ms。固定相机的 158 个输出帧均至少有一个 confirmed 实测或视觉辅助
目标；**这不代表三个真值目标逐个确认率均为 100%**。云台因扫描视角变化，
仅 68/159 个输出帧有确认目标，不把其余空画面自动认作漏检。

原始记录：工作区 `logs/pixel_latest_scene_baseline_cadence.json` 和
`logs/pixel_latest_scene_cadence.json`。探针为
`test/measure_pixel_cadence.py`，缓存有界、使用 monotonic 截止时间。
这些报告只证明话题更新节奏，不包含单个真值目标的框定位误差、
ID switch、完全遮挡后的误关联或网页接收/编码延迟。两次运行的预热起点
未严格对齐，因此不能以目标计数差异证明精度提升。完整第一阶段验收仍
未通过，不能用本节替代下面历史标准或启用后续阶段。

复测入口（开启窗口需另设 `gui:=true`，会改变负载）：

```bash
cd /home/promise/mrs_test
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch --skip-log-check xd_uav_track perception_gazebo_web_test.launch \
  gui:=false enable_high_frequency_pixels:=true
```

## 保留现有基线

沿用 `config/tracking_benchmark_baseline.json` 和
`docs/tracking_benchmark_standard.md`，不降低门限，不覆盖旧报告。
历史报告 `logs/tracking_mountain_baseline.json` 实际成绩为：

| 指标 | 历史实际成绩 | 新方案最低要求 |
| --- | --- | --- |
| 双目标同时确认率 | 100% | 不低于 100% |
| 双目标存活率 | 100% | 不低于 100% |
| ID switch | 0 | 0 |
| 额外确认轨迹 | 0 | 0 |
| 最大确认/存活更新空窗 | 0.507 s | 不高于 0.507 s |
| 最大世界投影误差 | 4.015 m | 不高于 4.015 m |
| 持续时长 | 45.060 s | 覆盖完整路线，至少 45 s |

这些比旧文件的放行门限更严格：仅达到 98% 确认率、1 s 空窗或
8 m 误差，不能称为本次优化通过。已为 100% 或 0 的指标没有改善空间，
应保持；至少一个非饱和指标应改善，并同时检查资源和端到端时延不退化。
历史报告没有提供 P95 时延、CPU/GPU 和显存基线，不能编造比较数据。

## 可比性检查

本次检查发现旧基线内的场景、路线、假山和 TensorRT engine 哈希一致，
但 `tracking_benchmark_perception.launch` 与 `tracking_benchmark.yaml`
不再匹配旧冻结哈希。这两处不是本次升级修改；不能改写旧哈希后宣称
复现了历史成绩。

先冻结当前配置，用同一输入、模型、相机、路线与负载分别运行旧路径和
候选路径，保留两份原始报告与资产哈希。历史门限、上述实际成绩和
当前 A/B 结果都必须单独列明。仿真真值只用于评分，不向跟踪器输入。
像素辅助的无效三维位置不得当作定位误差为零，定位指标只对真实定位
测量评估，并单独报告有效定位覆盖率。

## 第一阶段当前状态与开关

新增 `LatestPixelWorker`：每来源一槽 latest-only、目标数上限、采集时刻
锚点、双向批量 LK、过期/失败关闭、原 ID 与真实检测时间保留。
输出通过模式管理器与原帧配对，不塞入 C++ 检测时间序列，避免迟到
YOLO 锚点因较新光流被拒绝。失败或边缘观测请求有界 ROI 重检测，
并保留周期性全图搜索。选中目标 SOT 仍沿用原实现。

正式入口及移动仿真入口的 `enable_high_frequency_pixels` 默认 **false**。
仅在测试时显式设为 true，且同时启用 `enable_perception_modes`。
共享检测器只在此开关启用时停止旧推理节奏内的像素辅助，避免重复计算。
未开启时保留旧路径，模型与公开消息不变。

当前轻量回归只验证功能和边界，不代表 Gazebo、实拍或本机性能达标。
相机运动补偿、融合裁决与 TensorRT 执行优化尚未进入验收阶段。

本次轻量验证：30 项模式/像素线程/失锁/定位测试通过，共享检测器
8 项测试通过；`rostest xd_uav_track perception_video_web.test
high_frequency_pixels:=true` 的真实 ROS/WebSocket 原尺寸视频与交互联调
通过（1 项集成测试，0 错误）。原始日志为
`logs/pixel_stage1_ros_contract.log`。该测试是合成输入，不加载 YOLO，
不能证明实际推理速度、识别召回、Gazebo 框覆盖率或 IDF1/HOTA。
测试启动的 ROS/WebSocket 子进程已随测试结束退出。

## 2026-10-06 第一阶段实测记录（仍未放行）

本轮补齐批量 LK 的尺度异常门控、颜色回退、有期限的运动预测、
跨模式 generation 隔离、原始测量框锚点与同帧反馈。C++ 新增只读内部
`track/internal/observations_by_source`，保留全局 ID，提供未经 Kalman
平滑的该帧观测框；旧公开消息和控制接口不变。共享检测器只接受同来源、
同 epoch、同采集时间和同分辨率的辅助快照，不复制辅助 ID 到 detector，
不将辅助标为 YOLO、不将预测用于新建身份。ROI 仅在失败/边缘/锚点冲突时
请求，按来源限频；周期性全图检测仍保留。

同时修复度量位置门控的平滑滞后冻结：物理跳变检查使用最后接受的原始
位置与其采集时间，而非滞后的 EWMA 与上一图像时间。原有异常拒绝测试
保留，新增恒速运动测试。长期冻结会制造错误失联和新 ID，因此必须在
本阶段一并修复，不能用延长失锁时间掩盖。

记录（均为真实 Gazebo + train7 FP16，不是合成检测）：

| 指标 | 当前旧路径初测 | 最新候选 |
| --- | --- | --- |
| 双目标确认率 | 96.90% | 100% |
| 双目标存活率 | 98.45% | 100% |
| ID switch | 1 | 0 |
| 额外确认轨迹计数 | 1 | 0 |
| 最大确认更新空窗 | 1.127 s | 0.585 s |
| 最大有效定位误差 | 7.398 m | 3.439 m |
| 不可用三维位置观测计数 | 9 | 7 |
| 同帧展示双目标覆盖率 | 未测 | 100% |

最新候选的应用相机回调开始到目标发布 P95 为 97.676 ms；不包含相机
曝光、网络接入或网页 PNG 编码，不能称为完整端到端 P95。外部观察者
可能先收到较小的 targets 消息，再收到同帧大图，故其延迟配对可能为空；
不得将缺失值填成零。此前约 15 ms 的外部配对值也不能与这个应用计时
直接比较。

原始结果：`logs/pixel_stage1_baseline_metrics.json`、
`logs/pixel_stage1_final_candidate_metrics.json`。首次旧路径报告的
`frozen_assets=false` 还包含新验收 manifest 初次录入时的 world 哈希笔误；
笔误已修复，未重新写旧报告。最新候选的资产校验为 true。
一个主动 ROI 重检试验产生额外轨迹，未予采用；保留失败报告
`logs/pixel_stage1_measured_anchor_metrics.json`，不能拿其较高展示覆盖率
掩盖身份管理退步。

**最新候选的 `passed` 仍为 false**：0.585 s 未达到 0.507 s；当前验收器
把 confirmed 的视觉辅助（明确 `range_valid=false`）也计入不可用三维
观测，计数不是“发布了错误的有效三维位置”。这一语义必须单独核查，
不能把辅助定位伪装成真实检测以消除计数，也不能在不说明口径的情况下
直接改判通过。

因此第一阶段仍处于验收整改，`enable_high_frequency_pixels` 默认 false；
第二至第四阶段没有开始实现。36 项 Python 相关回归、24 项 C++ 跟踪回归、
9 项共享检测器回归及更新后的真实 ROS/WebSocket 联调通过，但不替代
未通过的性能验收。联调日志为 `logs/pixel_stage1_final_ros_contract.log`。
