# PixEagle / train7.pt 四模式融合使用指南

更新：2026-10-09。基线 Gitee feature/xd-uav-construction 提交 `3db09b001e903f70ca56815b325dba18d40fb756`。

方案定位更新：用户于2026-10-09选定本目录SAR与同级XD为后续正式方案。文件名、工作区路径保留历史experiment命名；
不改变原有默认模式，不代表UE/实机上线验收已经完成。完整正式部署合同见下方网页适配指南。

新网页/跨主机接入见 [网页接口适配指南](web_frontend_integration_guide.md)与
[接口绑定 JSON](web_frontend_interface_manifest.json)；该文档描述本实验实际四模式接口，不是旧 perception/* 网页。

## 修改范围

当前正式方案工作区：`/home/promise/mrs_test/sar_fusion_experiment.wlpAs3`。两包重新导出，只有 SAR 修改；XD 源码、消息、服务和 launch 与上游逐文件一致。
旧 `/home/promise/mrs_test/src` 下两包未改。旧实验备份在 `/home/promise/mrs_test/sar_fusion_backup_20261009.CpqpkW`，含旧 build/devel。
没有推送 Gitee，没有保存密钥。本次没有搬迁源码；不要同时将两份同名包加入同一catkin src，避免重复包。

## 四种模式

| 服务值 | 启动参数 | 算法 |
|---|---|---|
| smart_tracker | SmartTracker | 原 PixEagle YOLO + BoT-SORT |
| feature_tracker | FeatureTracker | 原人工 ROI + KCF/CSRT/MIL/模板兼容路径 |
| color_tracker | ColorTracker | 原 ROI 颜色学习或配置颜色全图搜索 |
| fusion_tracker | FusionTracker | 全图 YOLO + 选中单目标 KCF/CSRT + 运动、类别父组及弱外观门控 |

默认由原 YAML 的 Default_Tracker 决定，当前 SmartTracker；tracking_mode 留空不覆盖配置。
融合未选目标时输出全图 YOLO；选择后该目标只输出一个最终融合框，其他目标保持 YOLO 输出。
不是全目标 KCF 池，也不是跨相机身份层。原三模式未被替换。

## 模型与环境

固定使用 `models/xd_vehicle_train7/xd_vehicle_train7.pt`。
SHA-256：`8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54`。
PyTorch CUDA，不转 TensorRT、不改权重。新版上游检测门限 0.10，类别 [0,1,2,3]（car/ar-car/tank/m142）不变。
KCF/CSRT 使用 OpenCV CPU；HSV 是弱证据，不是坦克深度 ReID。
模型不能 group/world writable；CUDA/完整性失败明确报错，不静默退回 CPU。

### 同一车辆出现不同类别重叠框

车辆配置启用 `SmartTracker/SMART_TRACKER_AGNOSTIC_NMS: true`，IoU 门限为 `0.60`。
在 BoT-SORT 分配轨迹 ID 之前抑制 train7 四个车辆类别的重复候选，避免 tank/m142 各建一个框。
此配置只适用于当前全部为车辆的类别合同；以后加入人员等类别时，不应直接照搬全类别 NMS。

融合入口另有保守防线：不同子类属于同一父组、IoU 至少 0.75、中心偏差在小框宽高的 20% 内，才视为重复。
已验证的选中目标优先保留，其余按置信度选择；相邻但未高度重叠的车辆不会仅因距离近被合并。
原帧候选缓存、数值输出、云台单目标输出和叠框保持一致；`fusion_evidence` 的
`suppressed_class_duplicates` 记录该帧融合防线删除的框数，不包含 YOLO 内部 NMS 数量。
同类竞争框仍交由原有身份歧义门控，不通过几何去重强行合并身份。

去重不能判断 tank/m142 哪个类别才是真值，也不能修复模型漏检或仿真域差异。
Gazebo 的低多边形纹理与真实训练图像存在差异；UE 上需用当地相机录像验证类别混淆、召回及 ID switch，
不能把此处的仿真联通测试当作 UE 或实机识别精度认证。

本机 .venv-fusion 使用 OpenCV contrib 4.10，复用本机 GPU 依赖，其 .pth 不能直接复制到其他主机。
源码工作区包装器可自动选择 .venv-fusion，其次 .venv-sar-gpu；显式 python_executable 优先，跨主机/安装空间部署建议明确配置。
新主机独立环境先安装 requirements-smart-tracker.txt，移除普通 OpenCV wheel 后安装 opencv-contrib-python-headless==4.10.0.84。
实际检查 cv2.TrackerKCF_create / TrackerCSRT_create（或 cv2.legacy）以及 torch.cuda.is_available()。
融合禁止模板静默替代 KCF/CSRT；原独立 feature 模式的回退不变。

## 构建与启动

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/mrs_test/devel/setup.bash
catkin_make -C /home/promise/mrs_test/sar_fusion_experiment.wlpAs3 -j1 -l1 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release \
  -DSAR_YOLO_ENABLE_TENSORRT=OFF
source /home/promise/mrs_test/sar_fusion_experiment.wlpAs3/devel/setup.bash
rospack find sar_yolo_detector
roslaunch sar_yolo_detector xd_smart_tracker_fusion_integration.launch \
  UAV_NAME:=uav1 instance_name:=smart tracking_mode:=FusionTracker \
  input_image_topic:=camera/image_raw camera_info_topic:=camera/camera_info \
  image_width:=1920 image_height:=1080 \
  python_executable:=/home/promise/mrs_test/sar_fusion_experiment.wlpAs3/.venv-fusion/bin/python
```

包须解析到实验副本。旧入口位于 archive/launch，原样保留；新增唯一命名入口避免同名 launch 冲突。
只有新融合入口显式启用 enable_fusion_mode；旧归档入口仍运行原三模式，拒绝切入融合，避免旧 tracked 桥接把视觉辅助误当 YOLO。
按 UE 相机实际宽高、话题、frame 和时间配置；使用 /clock 时同步 use_sim_time。
入口只启动 SAR 和桥接，不启动飞行、定位、XD tracker 或云台设备控制器。默认把 UI 选中的单框转成
`/uav1/track/selected_detections`，供单目标跟踪使用；需要让 XD 感知链处理全部融合候选时，覆盖
`bridge_input_topic:=sar_yolo_detector/smart/tracked_detections/observations bridge_input_type:=fusion`
并把 `output_detections_topic` 指向对应的 `detect/input/.../detections_2d`。
双相机分别设 instance_name、输入、输出、image_source、sensor_id；每个实例加载一份模型，不是共享推理。

## 接口

### 正式 Gazebo 地图联调

仅从实验工作区 overlay 启动，地图和模型资源读取正式 XD 包，XD 源码不修改：

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/mrs_test/devel/setup.bash
source /home/promise/mrs_test/sar_fusion_experiment.wlpAs3/devel/setup.bash
roslaunch sar_yolo_detector formal_gazebo_fusion_test.launch \
  python_executable:=/home/promise/mrs_test/sar_fusion_experiment.wlpAs3/.venv-fusion/bin/python
```

默认使用正式 worlds/moving_uav_vehicle_search.world（纯色地面、三辆坦克），4 Hz、640×480 固定相机。
只启动一份 PT/CUDA 模型、实验 XD tracker、已有地面定位、Gazebo和一个 rqt 叠框窗口，不启动 PX4 或网页。
无人机使用现有运动学场景驱动；测试参数限制高度 28～36 m、搜索速度 1 m/s，避免模型重复加载及高速出画干扰。
资源位置可由 formal_track_package/world 参数覆盖。窗口可独立关闭，不设置顶或自动重启。

该入口 namespace=moving_uav_search，instance_name=fusion_fixed，因此选择服务为
`/moving_uav_search/sar_yolo_smart_tracker_fusion_fixed/select`，图像为
`/moving_uav_search/sar_yolo_detector/fusion_fixed/annotated`。
SAR 选择只选择图像融合目标，不自动改变 XD 的飞行跟随目标，不能把两层 ID 或控制选择混为一谈。

45 秒联调原始报告：工作区 logs/formal_gazebo_fusion_metrics_20261009.json。
178 帧输入、177 帧叠框，169 个有效投影位置，选中公开 ID 34 保持，90 帧 fused、7 帧 yolo、2 帧 visual；另有12帧 ambiguous、4帧 lost。
处理 P95=75.16 ms，原图/输出均640×480；这是链路测试，不是严格准确率或零失锁验收，出画/纯色背景造成的歧义需要另行分析。

默认 UAV_NAME=uav1，instance_name=smart。话题相对 /uav1，服务相对 /uav1/sar_yolo_smart_tracker_smart。

| 话题 | 类型/用途 |
|---|---|
| down_camera/image_raw | 默认 sensor_msgs/Image，可覆盖 |
| down_camera/camera_info | 默认 sensor_msgs/CameraInfo，可覆盖 |
| sar_yolo_detector/smart/tracked_detections | 原 TrackedDetection2DArray，所有最终观测框/类别/ID |
| sar_yolo_detector/smart/selected_tracked_detections | 原 TrackedDetection2DArray，一个最终选中框；无可信观测为空 |
| sar_yolo_detector/smart/tracked_detections/fusion | 新 FusionDetection2DArray，原目标结构 + 来源/最近 YOLO 时间/实际宽高 |
| sar_yolo_detector/smart/annotated | sensor_msgs/Image，同帧原像素叠框 |
| sar_yolo_detector/smart/state | 原 SmartTrackerState，选择、有效性、模型/设备/耗时 |
| sar_yolo_detector/smart/select_pixel | 原 PointStamped，像素点击；负坐标取消 |
| track/selected_detections | xd_uav_track/DetectionArray，默认桥接 UI 选中的单框，供 StartTracker 使用 |
| detect/input/detections_2d | xd_uav_track/DetectionArray，可选的全候选定位链输入 |
| sar_yolo_smart_tracker_smart/fusion_evidence | String JSON：公开/局部 ID、来源、原因、最近 YOLO 时间、耗时 |
| sar_yolo_detector/manual/initial_roi | 原 RegionOfInterest；feature/color 初始化手动跟踪器；fusion_tracker 中按最新缓存帧手动初始化 KCF |

| 私有服务 | 类型/用途 |
|---|---|
| switch_tracking_mode | 原 SwitchTrackingMode，新增支持 fusion_tracker |
| select | 原 SelectSmartTrack，当前帧 ID/点击/归一化 ROI/取消 |
| select_fusion_roi | 新 SelectFusionROI，原帧 header + 像素 ROI；唯一 YOLO 框可作为 KCF 初始框 |
| select_fusion_manual_roi | 同一 SelectFusionROI 请求；始终用手画框初始化 KCF，后续再由附近 YOLO 检测校正 |
| switch_model | 原 SwitchSmartTrackerModel；切换后清空融合选择 |

```bash
rosservice call /uav1/sar_yolo_smart_tracker_smart/switch_tracking_mode "{mode: fusion_tracker, color_name: ''}"
rosservice call /uav1/sar_yolo_smart_tracker_smart/select "{use_target_id: true, target_id: 5}"
rosservice call /uav1/sar_yolo_smart_tracker_smart/select "{clear_selection: true}"
```

旧 select 和 stamp=0 点击是最新帧接口，不适用于延迟网页。延迟点击须带显示原图完整 stamp/seq/frame_id。
延迟框选用 select_fusion_roi，先查原帧再匹配目标；它保留现有 YOLO 优先初始化行为。新增
select_fusion_manual_roi 使用完全相同的原帧请求合同，但跳过 YOLO 初始框吸附，直接用手画 ROI 启动 KCF。
后续 YOLO 仍在全图推理。手选 KCF 尚未绑定 YOLO 时，候选框中心落在本次手选 ROI 内（或与 ROI 的 IoU 不低于 0.15）才可开始首次关联；已在 ROI 内开始确认的同一 YOLO ID 可继续完成确认，即使检测框随后移出 ROI。首次 YOLO 身份确认后，清除该 ROI 限制，后续按位置、尺度、外观、类别和歧义门控融合。
每次新的手动画框都会重置启动门控并记录新的 ROI；此前选框的区域不会限制新选目标，也不会在进入融合后继续限制当前目标。
最多24帧/24 MiB，最大年龄0.75秒，三项限制同时生效。
请求header取与显示画面按stamp/frame_id配对的 fusion_evidence.capture_header，不能用不同发布者重写的seq。
过期、未来、错帧和越界请求均拒绝；select_fusion_roi 对多检测 ROI 拒绝，select_fusion_manual_roi 则按手画框启动，但后续歧义候选不会绑定为目标。
select_fusion_roi 的大 ROI 含唯一检测目标时使用紧框；无检测时可初始化未知类别 -1。
select_fusion_manual_roi 始终使用原始手画框作为 KCF 初始框，即使框内已有一个或多个 YOLO 候选；此时初始类别和检测身份未知，后续由区域内通过门控的 YOLO 结果确认，不默认伪称 tank。
第三方 UI 若只能发布 `sensor_msgs/RegionOfInterest`，也可向 `sar_yolo_detector/manual/initial_roi` 发布框：在 `fusion_tracker` 模式下后端以最新且未过期的缓存原帧为 KCF seed，YOLO 仍继续校正。该旧消息不带 header，因此不能保证与延迟显示帧精确对应；需要帧绑定时应调用 `select_fusion_manual_roi`。
中心在 detection.bbox.center.x/y（新消息为 target.detection），原图左上角为原点，x 右、y 下。
叠框与数值共用最终框、原始宽高/header，不缩图或独立修框。桥接 bbox 是 xmin/ymin/xmax/ymax，floor/ceil 取整最多一像素差。
不同 ROS1 发布者会独立计数 header.seq，输入与输出序号不保证相同；严格框选回传所显示叠框的 header，不猜测其他话题的序号。

## 融合和失联语义

- 按采集时间差预测；检查运动、尺度、车辆父组、HSV 相似度与纹理质量，局部 ID 相同也不绕过门控。
- 最佳/次佳过近、两条不同局部轨迹同时可见时拒绝拼接；一致才加权，不取框并集。
- 手选 KCF 等待首次 YOLO 关联时，候选须落在本次手选 ROI；同一 YOLO ID 可完成连续确认后解除 ROI 限制。之后使用常规身份、位置、尺度、外观和歧义门控，局部 ID 重建需同一候选连续三帧确认，公开锁定 ID 保持。
- 0.10～0.50 弱框仅维持当前已绑定且局部框一致的可信身份，不能恢复新 ID 或更新外观锚点。
- 车辆父组类别跳变不换 ID；选中类别暂保持，不是完整类别时域滤波。
- 手选且尚无 YOLO 关联时，持续输出通过 KCF 自身边界/跳变安全检查的逐帧框，不再让融合外观门控冻结 KCF；KCF 连续失败后仅预测0.35秒，之后 selected 流为空。YOLO 关联后，漏检时视觉辅助仍限1.5秒；短时重获窗口8秒，身份记忆最多60秒，长时重获须严格门控和3帧确认。详见网页指南第19节。
- prediction/lost/ambiguous 不输出有效检测或实线框，selected 流为空。
- yolo/fused 对应 detector，visual 对应 visual_tracker；辅助不刷新最近 YOLO 时间。
- measurement_current 仅当前 YOLO 为真；合格 visual 可短时 control_measurement_ready=true，其余 false。
- 重复/乱序不推进确认；帧间隔超过 0.75 秒不直接推进局部跟踪；尺寸变化清理 BoT-SORT/GMC、融合选择和缓存。
- 模式/模型切换清理融合选择，不承诺身份跨切换/重启恢复。原独立 feature/color 配置类别及手动 ID 语义保留。

## 兼容与验收边界

所有原 msg/srv 字节及 ROS1 MD5 保持上游不变。只新增 FusionDetection2D、FusionDetection2DArray、SelectFusionROI。
桥接新增 input_type=fusion，复用 XD 已有 DetectionCandidate.measurement_source；XD 不改。
原 vision/tracked 输入及所有旧输出/选择服务保留。新入口保留最新上游 selected 流和 gimbal_visual_servo 话题参数；角度、健康、控制权仍由设备控制负责。
未知人工 ROI 的 XD 全局注册、跨相机合并和长期 ReID 不在此次单包修改范围。

测试和真实模型链路报告见 fusion_validation_20261009.md；validate_fusion_runtime.py 带超时，不是准确率测试。
仍需 UE 同视频 A/B：IoU、ID switch、误跟背景、空窗、P95 端到端延迟及 CPU/GPU，包含相机/云台运动和遮挡。
当前同步 YOLO + 局部跟踪，KCF 频率仍受推理节奏约束；不宣称高频独立图像线程、坦克深度 ReID或上线认证。
