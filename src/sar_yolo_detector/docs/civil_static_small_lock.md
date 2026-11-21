# 民用静止小目标：人工选定后的可信视觉锁定

此入口用于 RGB 相机对民用无人机的被动观察。它保留四模式架构和 D-FINE-S V3；本阶段验证人工选定后的当前图像证据、连续跟踪与云台视线响应。人工 ROI 本身不是无人机类别证据，模型尚未确认时类别保持 `unknown`，不虚构分类、距离或三维位置。

## 环境与启动

沿用 `dfine_v3_deployment.md` 的 ROS Noetic、D-FINE、OpenCV contrib、UE 5.5 原生流插件与 PX4 环境。KCF/CSRT 必须实际可用，禁用模板替代。安装本包提供的 SarCameraStream 插件；旧二进制无法包含本次 TCP 生命周期修复。

```bash
source /opt/ros/noetic/setup.bash
source /path/to/workspace/devel/setup.bash
export PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot
roslaunch sar_yolo_detector ue_civil_static_lock.launch \
  ue_project:=/mnt/d/UE/Project/pro1 \
  python_executable:=/path/to/workspace/.venv-dfine/bin/python
```

网页入口为 `http://127.0.0.1:8787`。等检查点悬停后再留出至少 2 秒，让显示的采集帧也进入悬停时段，再切换 Fusion，在原始画面上紧贴目标拖框。民用场景明确 `mission.initial_yaw_ned_deg: 0`（机首朝北），任务控制器换算为 MAVROS ENU 航向；悬停时的位置数值残差不改变航向。默认使用 PX4 飞行动力学；纯视觉隔离测试可显式传 `pose_source:=kinematic`。默认固定水平视场角 90°，1920×1080 算法输入，原生持续 JPEG 传输（源质量 98，网页质量 90），采集请求 30 FPS。UE 的源 JPEG 通过 `/fixed_camera/image_raw/compressed` 直接进入算法的最新帧槽，算法工作线程解码，不增加一次 JPEG 重编码。原始 `Image` 话题仍可按需供录像或其他订阅者使用。网页预览独立压缩、限帧；输入图像没有为了网页缩小。实际吞吐取决于环境，须以运行记录为准。

## 数据流与有效性

* D-FINE、后处理、BoT-SORT、分类时间一致性、SmartTracker 状态管理仍执行；检测与选定目标分开调度。
* Fusion 保留 KCF/CSRT、选定目标光流、ROI 颜色分支，按各自有效性条件执行并报告证据；颜色模型连续失败后退出，不能继续对空背景投票。新增静止小目标验证作为人工小 ROI 的附加门槛，不能用弱颜色或局部框绕过它。
* 选框、相机运动历史、颜色模型从同一采集帧初始化；严格按原始 capture header 配对。弱检测框不能扩大精确的人工 ROI。
* 不可变的初始前景与环形背景用于核验；仅使用相机内参、测得姿态和当前像素，保存初选帧的内参、相机姿态及原始 ROI，直接计算初选到当前帧的旋转参考，不累积背景运动来代替空中目标的几何关系；当前前景像素仍须核验。初选姿态独立保存，不会随 TF 历史缓存淘汰而失效。此阶段的相机悬停域允许相对初选位置平移最多 0.25 米，超出即失效并要求重新选框，不套用地面投影。背景亮度整体变化通过局部对比度处理，不在线学习空背景。
* 初次必须通过五个不同采集时刻、跨至少 0.1 秒的验证才输出有效观测；在绝对标定参考可用时，5 秒初始化窗口内的启动停顿会重新收集五个新帧，期间控制仍无效，超时则要求重新选框。兼容增量相机补偿时，初始化首帧可以在 2 秒内利用连续、标定有效的姿态验证最初参考；这不是有效控制观测，等待期间控制仍无效。确认后超过 0.5 秒采集间隔即撤销，不能自动靠旧框恢复。
* 短暂像素异常立即撤销当前有效观测；只有姿态连续且 0.75 秒内重新通过两个不同新帧，才允许恢复同一身份。目标持续缺失、几何不连续或超时则需重新选框。
* 当前前景必须保有至少一半参考对比度（且不少于 8 灰度级），并通过有上限的误差门槛、邻域负匹配、几何和尺度检查。没有像素证据，检测候选也不能为人工小目标续命。
* 视觉证据不会刷新检测器时间戳、伪造 detector ID 或提升为分类概率。算法发布前也核验采集到输出不超过 0.5 秒，超时则所有选定目标输出无效；云台同样拒绝过期测量。网页的“已确认锁定”和命中计数必须同时满足当前控制有效、采集帧未过期、视频未中断。

初始 ROI 的短边至少 6 像素、长边至多 64 像素，投影后长边至多 96 像素；这个范围只是输入检查，并不保证每个尺寸或背景都可辨认。没有可分离前景的 ROI 应拒绝锁定。该验证器默认关闭，仅 `ue_civil_static_lock.launch` 开启，普通四模式入口保持原有设置。

## 复核

```bash
rosrun sar_yolo_detector run_ue_stress_suite.py \
  --project /mnt/d/UE/Project/pro1 \
  --python-executable /path/to/workspace/.venv-dfine/bin/python \
  --output /path/to/new/result-directory \
  --launch-file ue_civil_static_lock.launch \
  --cases civil_static_air_offcenter civil_static_air_h35 civil_static_air_dropout \
  --seconds 60 --no-record --roi-padding 2
```

评测器独立读取 UE 投影日志，仅在初始帧通过普通网页接口代替人工框选一次。感知与控制节点不读取目标真值。结果是给定初始 ROI 的跟踪与视线测试，使用投影三维包围框及中心射线可见性代理，不是自动检出率、像素级遮挡真值或真实无人机现场认证。

`static_lock_acceptance` 要求测试前已确认锁定、可见时长至少 10 秒、可见帧正确有效比例至少 90%、中心归一化误差 P95 不大于 0.10、可见时段最长有效观测间隔不超过 0.5 秒、零 ID 切换、目标不可见时零有效观测，且消失后最后一次收到有效观测不晚于 0.5 秒。旧 `tracking_acceptance` 原定义保留，它将目标消失后的尾段也算作间隔，因此消失负例应读取专用静止锁定指标。

`--no-record` 仅关闭额外的原始 ROS bag 录像负载，仍保存状态、时序、各分支证据、真值及初末帧。录像吞吐需独立复核，不与实时无录像结果混淆。

2026-11-16 的三组各 60 秒 PX4 实测已通过上述静止小目标验收，摘要见 [验收记录](validation/civil_static_px4_20261116.json)。三组可见时正确有效比例均为 100%，最长有效观测间隔分别为 0.237、0.239、0.198 秒；消失负例无不可见时错误续框。该结论不覆盖后续快速运动场景。
