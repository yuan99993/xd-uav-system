# 第二阶段：独立检测调度与融合可信度

保持 Smart、Feature、Color、Fusion 四模式，以及 D-FINE-S V3、BoT-SORT、Smart 管理器、
KCF/CSRT、颜色学习、LK、相机补偿、融合仲裁、去重和 SAR→XD 原有链路。

## 独立调度

检测线程与当前图像线程通过有界 latest-only 邮箱通信。Fusion 的局部跟踪在每张新图像
更新，不等待全图检测。相同采集帧的晚到检测仅进行修正，不再次推进 KCF/CSRT/颜色。
`AsyncPipeline/fusion_detector_max_fps` 控制 Fusion 全图检测上限，V3 默认 8 Hz；0 表示不限制。
Smart 保留原检测调度，Feature/Color 单独模式仍无需检测器。

既有异步路径曾只允许检测采集时间恰好等于当前图像的身份更新。在高帧率下，大量有效
晚到包因此不能更新选定身份，最终触发丢失。本阶段改为：

1. 原始 D-FINE/BoT-SORT 检测通过原 Smart 管理器的选定身份验证。
2. 检测包在有效期内，且确实被当前 FlowDetections 接纳。
3. 选定身份的框经过前后向光流、运动/尺度/外观检查，传播到当前图像。
4. Fusion 使用这个当前视觉位置与原始检测时间；不把旧检测框标成当前 detector 实测。

没有光流支持、过期/重复包、其他目标的检测和空检测都不能刷新选定目标的检测时间。
SAR→XD 的原始 detector 观测仍保留原采集头；当前视觉输出独立标为 visual_tracker。

## 分支证据

`/<uav>/sar_yolo_smart_tracker_dfine_v3/fusion_evidence` 中的
`diagnostics.branches` 分别包含 `feature/flow/color/detector` 的 accepted、score 和 reason。
score 是分支证据分数，不是经过校准的目标存在概率。

- Feature：原跟踪器响应、选定外观相似度、纹理质量及位置/尺度检查。
- Flow：前后向支持、同身份和时效检查；视觉证据冲突时明确拒绝。
- Color：ROI 直方图响应、前景纯度、局部背景对比、连续性及选定外观检查。
- Detector：原分数、身份/外观/关联门控，以及原始采集时间。

颜色与周围背景越相似，分数越低；不再将低颜色质量抬高到固定的最低置信度。
颜色与特征位置一致时，颜色混合比例最多 20%，并随实际可信度降低。其他身份、
不满足外观或位置条件的颜色框不能参与融合。

## 可选 NanoTrack 与对比

默认仍为 KCF。包内 NanoTrack v2 是可选局部跟踪器，模型文件、来源、许可证、SHA256
均位于 `models/nanotrack_v2`。模型加载及初始化失败会明确失败。

```bash
roslaunch xd_uav_track dfine_v3_four_modes.launch feature_tracker_algorithm:=nano
# UE 完整入口也支持 feature_tracker_algorithm:=nano / csrt / kcf
rosrun sar_yolo_detector benchmark_local_trackers.py --output /tmp/local-tracker-comparison
```

对比工具将 KCF/CSRT/NanoTrack 运行在相同初始框、相同两组序列上，并导出视频与几何真值。
序列使用本项目 UE 图像中的目标纹理，包含运动、尺度变化、背景变化和短时完全遮挡，
属于受控合成测试，不能代替真实 UE 地图精度评估。独立跟踪器在遮挡时仍可能错误返回
成功，因此候选性能不能用于绕过 Fusion 的有效性门控。OSTrack-256 也通过原生网络与作者提供的
`vitb_256_mae_ce_32x4_ep300` 权重完成同序列离线对比。它不作为生产 ROS 后端。
原生代码、权重由用户显式提供，研究依赖单独安装：

```bash
# 在独立比较环境中安装，不修改生产运行环境
python -m pip install -r "$(rospack find sar_yolo_detector)/requirements-ostrack-benchmark.txt"
rosrun sar_yolo_detector benchmark_local_trackers.py --output /tmp/four-trackers \
  --ostrack-source /your/OSTrack --ostrack-checkpoint /your/OSTrack_ep0300.pth.tar
```

本机受控序列测得的 p95 单帧延迟约为：KCF 1.4–5.9 ms、CSRT 21.6–24.5 ms、
NanoTrack 9.8–10.7 ms（以上 CPU），OSTrack-256 36.6–42.0 ms（CUDA）。
这里记录完整裁剪/跟踪更新耗时，OSTrack 显式同步 CUDA；两种设备、合成场景和默认参数
下的结果不能外推为通用排名。CSRT/NanoTrack 在完全遮挡时也会返回错误的成功标志。
因此保持默认 KCF，可显式选择 NanoTrack，并保留所有融合门控。

## 复核

```bash
# 启动专用测试命名空间，避免与实际相机同时发布
roslaunch xd_uav_track dfine_v3_four_modes.launch UAV_NAME:=deploy_check web_port:=8788
rosrun sar_yolo_detector verify_dfine_pipeline.py --namespace deploy_check \
  --web-url http://127.0.0.1:8788 --input-fps 30 --per-mode-sec 10 --output /tmp/fusion-check.json
```

验证器检查各阶段实际调用、四模式网页选择、至少 75% 的持续有效网页锁定、Kalman/外观
活跃，以及 SAR→XD 类别一致性，同时保存逐阶段 trace、网页和 Fusion 分支证据。
相机输入 30 FPS 是该回放测试的设置，不是对任意 UE 地图/硬件的帧率保证。
