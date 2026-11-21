# UE 民用汽车录像与首次失败复盘

使用 UE5.5 的 AirGroundStressV1 地图和 PX4 巡检点悬停。三个配置分别覆盖单车横穿、相同车型相向交汇、汽车经过现有遮挡区域。单车与交汇配置的目标峰值速度为 20 m/s，遮挡配置为 6 m/s。验收以录像时的真值和遥测为准。检查评分窗口内的任务机实际速度，并确认遮挡场景出现中心射线从遮挡到清晰的转换。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PX4_AUTOPILOT_DIR=/home/promise/PX4-Autopilot
rosrun sar_yolo_detector run_ue_stress_suite.py \
  --project /mnt/d/UE/Project/pro1 --output "$PWD/reports/civil_car_new_capture" \
  --python-executable "$PWD/.venv-dfine/bin/python" \
  --launch-file ue_civil_moving_targets.launch --pose-source px4 \
  --stream-quality 98 --seconds 30 --roi-padding 2 \
  --cases civil_car_crossing_single civil_car_crossing_pair civil_car_crossing_occluded
```

对单次运行的 bag、采集配置和评测目录做联查。录制器启动晚于初始选框时，先检查初始选框采集戳是否在 bag 中，再判断是否适合用原始 ROI 重跑。

```bash
base="$PWD/reports/civil_car_new_capture"
case_id=civil_car_crossing_single
rosrun sar_yolo_detector inspect_ue_car_recording.py \
  --bag "$base/$case_id.bag" --capture "$base/${case_id}_live" \
  --evaluation "$base/${case_id}_eval" --output "$base/${case_id}_review"
```

`aligned.jsonl` 逐帧包含采集戳、图像序号、汽车身份和投影框、中心射线可见性、目标世界速度和像素速度估计、匹配的融合状态。匹配容差为 2 微秒。缺失同帧状态单独记为 `no_matched_output`。车速和像素速度采用前后帧差分，遮挡代理采用中心射线。PNG 是原始帧。MP4 用于人工查看，精确回放使用 bag。

现有 `ue_rgb_replay.launch` 提供隔离的 RGB/K/TF 回放，`capture_mapping.jsonl` 记录回放戳和原始戳。四模式比较时应分别启动并固定模式，在相同原始帧复用 ROI，验证该帧确实送达跟踪节点。网页端的绘制证据须另采，融合状态只能确认算法有没有发出当前框。

## 历史实测

在 `reports/civil_car_recording_review/v2_*` 中，6 m/s 普通和遮挡录像分别匹配到 712 和 678 张真值帧，第一个有评分的可见失败分别为图像序号 583 和 552。两帧的融合状态都返回 `no_trusted_visual_observation`。这两份旧录像的评分区间均未记录正确框。

最新 20 m/s 固定 90° 历史运行的首个已匹配可见失败是融合帧 1288。KCF 分支仍有效，选定目标光流不一致，颜色分支未通过门控，同帧没有新的检测器证据。融合仅保留预测，发布状态没有当前确认框。该帧的 RGB 未录制。日志能够确定首个匹配失败时确认框已在融合输出阶段消失，无法用该次记录逐像素复核或定位浏览器另外丢掉的框。

## 误锁与连续性计分

```bash
rosrun sar_yolo_detector score_ue_car_review.py \
  --review "$base/${case_id}_review" --evaluation "$base/${case_id}_eval" \
  --browser "$base/${case_id}_browser.json" --output "$base/${case_id}_score.json"
```

浏览器文件使用 `window.sarPresentationSamples()` 返回的 JSON 数组。网页记录原始图像和叠框各自的采集戳、Canvas 确认框和双 rAF 给出的帧龄区间。采样缓冲最多 512 条，采集端应在评分结束后及时导出。离线只评价能与真值对齐的记录。网页输出和 ROS 确认状态分别统计。无浏览器采样时省略 `--browser`，该部分结果为 null。

邻车和目标框使用投影框 IoU 0.25。两车 IoU 都达到门槛且差小于 0.05 时标为歧义。与可见车辆投影框完全无重叠记为背景代理。可见脱框把缺失同帧确认状态计入，连续时长以相邻采集时间估算。当前分支矛盾指 KCF 通过而选定目标光流报告不一致。报告同时列出具有分支诊断的帧数。遮挡后恢复需要原选定 ID，浏览器恢复时间带帧龄上下界。历史录像若缺少初始 ID 或实际绘制日志，对应恢复或网页指标为 null。

验收路线须独立于调参和复盘用的录像。核对原始选框帧已录入 bag，再固定同一 bag 和 ROI 比较修改前后的结果。

## 选框前录制的阶段检查

汽车 crossing 用例先等待 PX4 到达 inspect_01，再启动录制器。评测器等待录制器订阅图像后，使用此后的一帧选框。录制结束后自动校验初始原始帧、逐帧内参和同帧融合处理证据，导出 selected_raw.png、ROI、内参、原始像素哈希和模型路径及权重哈希。缓冲溢出或初始帧缺失均判定失败。

回放入口接受 selection_validation 参数，指向 selection_validation.json。它校验 bag 哈希并等待跟踪节点对初始帧的融合证据，写入 replay_summary.json。该参数验证输入帧处理，ROI 的实际提交仍需由同帧选框流程执行。

2026-11-19 第一轮新录制出现 rosbag 缓冲溢出，不能作为完整复现样本。该轮在跟踪节点就绪前已经开始录制。已将录制起点调整到 PX4 巡检悬停后，此调整仍需实测通过。首次脱框算法修改须等待该阶段通过。
