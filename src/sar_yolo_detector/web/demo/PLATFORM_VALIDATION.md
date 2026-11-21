# 平台适配与最小回归

`config/platform/portable_baseline.yaml` 是智能跟踪节点私有命名空间的参数覆盖模板，对应当前演示参数。通过 `xd_smart_tracker_fusion_integration.launch platform_config:=/path/to/profile.yaml` 在原有节点参数之后加载，按实际相机和测量结果校准。相机话题和模型路径通过启动参数指定，推理选择参考 `config/dfine_v3.yaml`。图像订阅队列可通过 `input_image_queue_size` 配置，推理任务使用最新帧邮箱；当前仍需使用已有后端实现。切换后端前须先确认该后端及其依赖在目标平台可用。

从已 source 的工作区运行：

```bash
DFINE_PYTHON=/path/to/model/python "$(rospack find sar_yolo_detector)/web/demo/run_portable_regression.sh"
```

源码工作区也可直接运行 `web/demo/run_portable_regression.sh`。本机的 KCF/CSRT 在 `.venv-fusion/lib/python3.10/site-packages`，模型解释器为 `.venv-dfine/bin/python`；两者都要位于模型进程的 Python 搜索路径。该脚本测试融合、近邻门控、重选和辅助框几何。离线用例不能替代带真值的常规、高速、交会、重选在线测试，也不能据此宣称零误锁。

实时排查优先读取以下已有输出，节点前缀按实际命名空间替换：

- `.../pipeline_trace`，每秒包含采集时间、处理和拒绝计数、邮箱覆盖、调度及各阶段计数。对比相机原始时间戳间隔、处理进度和邮箱覆盖数，可以区分采集断档与处理积压。
- `.../fusion_loss_history` 和 `.../fusion_evidence`，用于检查状态转换、关联来源与诊断。检测超时可结合状态与阶段计数查看；`pipeline_trace` 记录 `capture_gap`、`selection_request`、`strong_takeover_veto`、`detector_gap_visual_fallback` 与 `control_capture_deadline_exceeded` 计数及 `important_stage_last` 中各事件的最近详情。它是每秒汇总而非逐事件日志，同一秒内多次事件需通过计数识别，不能恢复每条详情。
- `/diagnostics` 和检测节点 `model_status`，用于检查实际设备、耗时、回退和积压。以具体节点实际发布字段为准。

为后续在线测试单独记录设备、驱动、模型哈希、配置快照、相机时间戳、有效帧率、误锁与 ID 切换。不要将失效或空检测帧计入有效锁定。当前辅助框显示代码只作用于网页，它的数字 ID 是临时显示身份，不能作为跨平台真实身份真值。

## 封存参考与当前候选

`reports/dfine_round9/sealed_release.json` 对应 verification2 仿真实用参考版，严格门禁未全通过。封存快照 380 个文件的 SHA256 已核验，本候选不更改该归档。当前工作区不是封存版：相对封存清单，`CMakeLists.txt`、`launch/smart_tracker_fusion.launch`、`launch/xd_smart_tracker_fusion_integration.launch` 和 `scripts/smart_tracker_node.py` 已修改；`web/demo/`、`launch/ue_web_demo.launch`、`config/platform/portable_baseline.yaml` 为后续演示与候选配置。`pipeline_metrics.py`、`XDChainMetrics` 及 `verify_dfine_pipeline.py` 与封存清单一致。

2026-11-21 重启候选节点后，实时 trace 观测到两次重选和强接管拦截计数，约 8 秒内状态消息约 24.6 Hz。第二次重选的状态采样旧 ID 7 后切换至 900000001，未见新 ID 后回退；两次状态切换间的采集时间包含实际帧间隔，不作为真值或误锁率。详细记录在工作区 `runtime/jitter_investigation_20261121/restart_live_audit.json`，不属于封存验收记录。拦截记录的前四个候选为上下文摘要，不能保证恰好是被拦截候选。计数为每次融合步骤发生拦截的次数，不等同于候选数量。

当前待办：在目标平台实测后完成相机、推理后端和帧率节拍的配置适配；对采集断档和拦截的多次事件建立更完整的取证；以独立真值执行常规运动、高速、交会、重选及辅助框回归，单独记录平台参数、误锁、ID 切换、帧率与延迟。现有 ROS 队列与最新帧邮箱会在压力下丢旧保新，不保证每帧处理，也不能由一次仿真采样推断新平台结果。

## 网页编号与辅助框边界（2026-11-21 候选）

网页使用当前网页进程内按首次出现顺序分配的短数字 `display_id`；主框和辅助框共享编号空间。原有 `selected_id`、辅助框内部 `id` 保留用于接口与选框，`selected_identity_debug` / `identity_debug` 保存来源和可用的原始值。原始检测没有提供 `raw_id` 或 `det_id` 时显式为 null，不能把网页编号当作跨进程目标身份。网页刷新服务或辅助轨迹清除重建后编号可能改变。

只改变展示辅助层：检测刷新后用当前框，最多 0.35 秒以内可利用 LK 或最近检测估计速度延续，中心离开图像即移除，部分贴边框以图像交集绘制；可信点不足时短期速度延续不等于重新识别。默认最多 12 个辅助目标，单次辅助耗时超过显示周期时下一次最多 4 个。辅助轨迹不能直接更新 ROS 主锁定或强接管，但用户点击辅助框会调用原有 ROI 选框接口。

采样 `runtime/jitter_investigation_20261121/edge_audit_before_normalization.json` 中近边原始候选置信度均低于 0.50，网页过滤掉了它们；该数据没有目标真值，不能判断边缘车辆是否被正确检出。`id_normalized_metrics.json` 为短编号实时抽样，不是带真值验收。合成边界回归不等于实景边缘验收。当前摄像机位姿没有独立接入辅助延续，快速转镜头时速度外推可能漂移，0.35 秒后必须重新检测。

## 仿真收口与裁剪对照（2026-11-21）

verification2 仍为封存的仿真实用参考版，不以本次实验覆盖或改写。当前演示默认继续使用 256×256 起始裁剪和辅助层最多 0.35 秒无检测延续。限量光流延续试验已回退。384×384 每帧共享裁剪及每 5 次主检测切换至 384 的试验保存在工作区 `runtime/edge_evaluation_20261121/`，未合入默认配置。

复测采样保留了投影代理、同步网页框、原始候选和若干 JPEG 画面。画面人工抽查发现 384 试验中有车辆辅助框，也有道路上未见清晰目标的可疑框。每 5 次扩窗直接改变该次主检测输入和融合候选，并非独立辅助池。两次自动选框产生了不同的主 ID，不能排除目标或视角差异；采样结果不足以证明零误锁、零 ID 切换、误框率达标或两配置的因果优劣。目标平台测试时应使用同一主目标和独立标注真值重新对照，记录原始采集时间、有效帧率、延迟、误锁、ID 切换及辅助框误检。

`pipeline_trace` 的阶段计数及最近事件用于发现异常；同一秒内发生多次采集断档或强接管拦截时，应同步保存 `fusion_loss_history`、`fusion_evidence`、原始状态与相机消息并以采集时间关联。现有汇总不是完整逐事件历史；缺失上下文应明确标为未知。
