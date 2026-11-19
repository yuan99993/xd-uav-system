# sar_yolo_detector

本包当前保留的主启动链是 `launch/xd_smart_tracker_fusion_integration.launch`。
它启动 FusionTracker、共享 TensorRT 推理服务，以及向 XD 感知输入转发所选目标的桥接节点。
独立 SmartTracker 跟踪模式已移除；其配置命名空间仍保存 FusionTracker 使用的 YOLO 模型、类别和推理参数。

```bash
roslaunch sar_yolo_detector xd_smart_tracker_fusion_integration.launch \
  UAV_NAME:=uav1 \
  input_image_topic:=down_camera/image_raw \
  camera_info_topic:=down_camera/camera_info
```

启动文件依赖以下配置：

- `config/smart_tracker.yaml`：YOLO 检测后端基础设置
- `config/smart_tracker_xd_vehicle.yaml`：车辆类别、跟踪和手动选择设置
- `config/vehicle_botsort.yaml`：BoT-SORT 设置
- `config/fusion_tracker.yaml`：FusionTracker 设置

默认模型为 `models/xd_vehicle_train7/xd_vehicle_train7_tegra_512.engine`，并在启动时校验
SHA-256。启动多个无人机实例时，共享推理服务按模型摘要复用同一份 GPU 模型；每架无人机的跟踪状态仍然独立。

FusionTracker、FeatureTracker 和 ColorTracker 共用同一个图像接收与帧调度节点。
每帧按需缓存灰度/HSV 转换；没有待传播的光流目标且本次 YOLO 结果为空时，跳过无用的灰度转换和 LK 初始化。
OpenCV 与 PyTorch CPU 线程数默认限制为 1，可在对应运行配置中调到 2–4；该限制用于防止多个 ROS/推理进程相互争抢 CPU。

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
