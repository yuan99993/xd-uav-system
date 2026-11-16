# sar_yolo_detector

本包当前保留的主启动链是 `launch/xd_smart_tracker_fusion_integration.launch`。
它启动 SmartTracker Fusion、共享 TensorRT 推理服务，以及向 XD 感知输入转发所选目标的桥接节点。

```bash
roslaunch sar_yolo_detector xd_smart_tracker_fusion_integration.launch \
  UAV_NAME:=uav1 \
  input_image_topic:=down_camera/image_raw \
  camera_info_topic:=down_camera/camera_info
```

启动文件依赖以下配置：

- `config/smart_tracker.yaml`：SmartTracker 基础设置
- `config/smart_tracker_xd_vehicle.yaml`：车辆类别、跟踪和手动选择设置
- `config/vehicle_botsort.yaml`：BoT-SORT 设置
- `config/fusion_tracker.yaml`：FusionTracker 设置

默认模型为 `models/xd_vehicle_train7/xd_vehicle_train7_tegra_512.engine`，并在启动时校验
SHA-256。启动多个无人机实例时，共享推理服务按模型摘要复用同一份 GPU 模型；每架无人机的跟踪状态仍然独立。

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
