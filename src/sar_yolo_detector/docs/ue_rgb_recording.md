# RGB 录像、分析与四模式回放

第 5 阶段空地视线与 PX4 验收通过后实施本阶段。UE 通道的来源为 `UE.SceneCapture2D.FinalColorLDR`，实际 ROS 图像为 RGB/BGR 三通道。`camera/source` 明确记录 `modality=rgb`、`thermal_sensor_model=false`、来源、采集会话、光学坐标系及编解码通道。当前没有红外传感器响应模型或真实红外跟踪验收；灰度、色表、伪彩色都不能生成该能力。后续红外应使用真实传感器数据、响应与噪声模型、独立模型/评估，再接入已验证的数据契约。

## 录制实际输入与分析证据

需要 ROS Noetic 的 rosbag、tf2_msgs、numpy 和 OpenCV。录制/导出/回放工具使用系统 ROS Python（`/usr/bin/python3`，需与 Noetic 的 rosbag/roslz4 对应），D-FINE 继续使用独立模型解释器。完整算法回放还需 [D-FINE-S V3 环境](../requirements-dfine-v3.txt) 中的 Python 3.10、PyTorch/CUDA 和 contrib OpenCV KCF/CSRT；可用 `python_executable` 指定模型解释器。先启动默认 PX4/UE 正式入口，确认实时画面，再运行：

```bash
roslaunch sar_yolo_detector ue_rgb_record.launch \
  output:=$HOME/.ros/sar_rgb/session.bag seconds:=75
```

录制器是独立只读进程，不改变飞行、模型或跟踪调度。它要求实际 UE RGB 来源声明，拒绝把已有回放再次录成实时传感器证据。记录内容包括原始 Image、CameraInfo、相机 TF、采集来源、任务状态、云台反馈、几何证据、SAR 状态/Fusion 证据/算法计数、SAR→XD 候选及 XD 轨迹。

完成后创建 `.bag.manifest.json`，保存来源会话、文件 SHA256、帧数、实际时间跨度/平均帧率和逐帧配对统计。每幅图像必须具有自己采集时刻的 K、尺寸、光学坐标系及相机姿态；缺失或冲突会明确失败，不用最新 K 或固定 FOV 补齐。索引仅保存有界的小型标定/姿态元数据，图像逐帧读取。录制文件不覆盖已有会话。

ROS bag 保留算法实际收到的消息与像素；采集源默认使用质量 90 的 JPEG，若需原始像素通道，可在正式入口指定 `stream_codec:=bgra8`。原始采集时间与 ROS bag 接收时间分别保存，不把接收时间当作采集时间。

## 导出视频用于查看和标注分析

```bash
rosrun sar_yolo_detector export_ue_rgb.py \
  --bag $HOME/.ros/sar_rgb/session.bag --output $HOME/.ros/sar_rgb/session.mp4
```

MP4 使用平均实际采集帧率，便于逐段查看；`.mp4.frames.jsonl` 保留每帧原始纳秒时间、K、尺寸、相机位置/四元数和 bag 接收时间。时间精确分析以此侧录和 bag 为准，不能用固定帧率 MP4 的播放时刻代替实际可变采样时间。导出不改变颜色域，也不添加红外能力标签。

## 重新运行 D-FINE-S V3 与原四模式

使用独立 ROS master/命名空间运行，便于与实时仿真分别分析：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11457
roslaunch xd_uav_track dfine_v3_rgb_replay.launch \
  bag:=$HOME/.ros/sar_rgb/session.bag \
  output:=$HOME/.ros/sar_rgb/replay_result web_port:=8788
```

打开 `http://127.0.0.1:8788`，可使用 Smart / Feature / Color / Fusion 及网页选定、ROI 学习。入口仍为包内 D-FINE-S V3 和原有完整 SAR→XD 算法链路。回放先等待真正的算法图像订阅者就绪，再播放完整录像；避免模型冷启动期间丢掉开头内容。

回放只发布新命名空间下的 RGB、对应 CameraInfo、独立世界坐标系下的相机 TF 和“RGB recording replay”来源声明。保存的检测框、轨迹、类别判断、任务及云台/飞行反馈供分析，不重发给算法冒充新计算。回放不启动 PX4、UE 相机控制或目标飞行追逐。记录中的旧选框/模式状态不会自动执行，用户在同一录像上重新选择；需要精确复现某次初始化时，可对照原始状态与采集时间在相同图像 ROI 初始化。

原采集间隔按 `rate:=1.0` 保留，整个 Image/K/TF 元组同步平移到本次 ROS 墙钟；`capture_mapping.jsonl` 逐帧保留原始时间与回放时间映射。`rate` 可设 0.1–4，慢放可用于分析，不能把慢放结果报成实时吞吐。帧不会因回放工具内部采样被省略；运行中的正式算法仍按自己的最新帧调度处理，并报告真实拒绝/替换计数。回放结果的处理延迟不是原始相机端到端延迟。

普通 MP4 缺少采集内参/姿态时只能进行图像层分析；不能把它补成有真实空间位置或红外能力的传感器录像。现有通用视频入口保留，新入口使用带完整配对证据的 RGB bag。

## 核验

```bash
rosrun sar_yolo_detector verify_ue_rgb_replay.py \
  --namespace rgb_replay --seconds 30 --output /tmp/replay_isolation.json
rosrun sar_yolo_detector verify_dfine_pipeline.py \
  --namespace rgb_replay --web-url http://127.0.0.1:8788 --external-camera \
  --output /tmp/replay_four_modes.json
```

前者只读检查实际图像与同帧 K/TF、来源和回放隔离；后者通过真实网页选择验证四模式，并要求 D-FINE/NMS/BoT-SORT、Smart 管理器、特征/颜色/光流、Fusion 仲裁与 SAR→XD 实际调用。需要在录像结束前完成观察窗口；EOF 后旧画面不会冒充持续采集。录制原始模型误检仍保留，不据回放流程通过宣称分类或真实红外准确率。

2026-11-15 验收：1971 帧原始 RGB/K/TF 配对与 rate=1 完整重放通过；网页四模式锁定采样 92.5%–97.5%，全部必要算法调用已核验。包含 PX4/MAVROS 原始记录的另一片段实际核验 284/284 帧配对，未重发控制数据。最终 242 项针对性回归与 4 项 ROS 桥接测试通过。详见 [upgrade_acceptance_20261114.json](upgrade_acceptance_20261114.json) 的 stage6。
