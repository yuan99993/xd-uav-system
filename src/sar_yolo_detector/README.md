# sar_yolo_detector

本包当前保留的主启动链是 `launch/xd_smart_tracker_fusion_integration.launch`。
默认使用 C++ tracker 和 C++ TensorRT 推理服务，并向 XD 感知输入转发所选目标。
Python tracker/provider 仅在显式设置 `tracker_node_type` 与 `shared_model_server_type` 时使用。
独立 SmartTracker 跟踪模式已移除；其配置命名空间仍保存 FusionTracker 使用的 YOLO 模型、类别和推理参数。

```bash
roslaunch sar_yolo_detector xd_smart_tracker_fusion_integration.launch \
  tracker_node_type:=sar_yolo_smart_tracker_cpp \
  shared_model_server_type:=sar_yolo_shared_model_server_cpp \
  UAV_NAME:=uav1 \
  input_image_topic:=down_camera/image_raw \
  camera_info_topic:=down_camera/camera_info
```

启动文件依赖以下配置：

- `config/smart_tracker.yaml`：YOLO 检测后端基础设置
- `config/smart_tracker_xd_vehicle.yaml`：车辆类别、跟踪和手动选择设置
- `config/vehicle_botsort.yaml`：BoT-SORT 设置
- `config/fusion_tracker.yaml`：FusionTracker 设置

默认模型为 `models/xd_vehicle_train7/xd_vehicle_train7_tegra_512_batch3.engine`，并在启动时校验
SHA-256。启动多个无人机实例时，共享推理服务按模型摘要复用同一份 GPU 模型；每架无人机的跟踪状态仍然独立。

FusionTracker、FeatureTracker 和 ColorTracker 共用同一个图像接收与帧调度节点。
每帧按需缓存灰度/HSV 转换；没有待传播的光流目标且本次 YOLO 结果为空时，跳过无用的灰度转换和 LK 初始化。
OpenCV 与 PyTorch CPU 线程数默认限制为 1，可在对应运行配置中调到 2–4；该限制用于防止多个 ROS/推理进程相互争抢 CPU。

模式行为：

- `fusion_tracker` 会按 `inference_max_fps` 请求 YOLO 检测，并为每个 YOLO 目标维护一个模板框。每帧在各目标附近做单尺度匹配，多尺度校正按配置限频；UI 选择只决定哪个目标输出到 `selected_tracked_detections` 供云台跟踪。目标完全不可见时会标记丢失，不发布旧框。可通过点选/框选，或调用 `auto_select` 开始自动选择。尚未选中目标时，所有YOLO/模板目标仍会发布。
- `feature_tracker` 和 `color_tracker` 不请求 YOLO 推理；共享推理服务仍保持运行，模型不会因切换模式而卸载。ColorTracker 配置了 `red/green/blue/white/black` 时，会立即把全图色块作为候选框发布到 `tracked_detections`，但不会自动选中；只有 UI 点选/框选或显式开启 `auto_select` 后，`selected_tracked_detections` 才发布目标。颜色为空或为 `custom` 时先等待 ROI，再按 ROI 学到的颜色特征跟踪。
- 切回 `fusion_tracker` 时会检查共享 YOLO 服务。若服务不可用，模式切换会失败并返回服务地址。节点的 `fusion_evidence` 会提供 `detector_updated` 与 `inference_error`，用于区分本帧没有新推理、推理失败和推理成功但未检出目标。

主要输出：

- `sar_yolo_detector/smart/tracked_detections`：当前跟踪结果
- `sar_yolo_detector/smart/selected_tracked_detections`：选中目标，默认转发到 `track/selected_detections`
- `sar_yolo_detector/smart/annotated`：带标注的图像
- `detect/input/detections_2d`：由可选的检测桥接实例转发的选中目标

`launch/xd_detection_bridge.launch` 是上述入口使用的桥接节点配置。`launch/formal_gazebo_fusion_test.launch`
用于围绕主启动链运行 Gazebo 验证。

为保持同一工作区内其他 `xd_uav_track` 启动文件可用，本包仍保留多源检测入口及其配置。ROS 消息和服务定义也继续保留，
以兼容依赖这些接口的同级包。旧归档启动链、独立 SAR/任务 C++ 节点、训练/任务工具以及未使用的 profile 配置已移除。

构建：

```bash
source /opt/ros/noetic/setup.bash
cd /path/to/catkin_ws
catkin build sar_yolo_detector
source devel/setup.bash
```
