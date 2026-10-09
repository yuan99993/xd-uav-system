# 已归档的旧启动入口

`archive/launch/` 保留旧 COCO、救援、热红外、OpenCV 和上游 SmartTracker
启动文件，仅供历史参考；它们不随正常安装空间发布，也不是车辆识别/跟踪入口。
归档文件之间的 `include` 已改为指向本目录。部分历史入口还依赖当前工作区
未安装的外部包，不能保证直接运行。
旧单路 Python 检测器、启动包装器及对应历史测试保存在 `archive/scripts/`
和 `archive/test/`；正式单路入口现在复用共享检测器实现。

`launch/xd_smart_tracker_integration.launch` 还提供 UI 点击选框接口：

- `sar_yolo_detector/$(arg instance_name)/tracked_detections` 持续发布所有 YOLO 框。
- UI 将 `geometry_msgs/PointStamped` 发布到
  `sar_yolo_detector/$(arg instance_name)/select_pixel`；`point.x` 和 `point.y`
  使用原始图像像素坐标。
- UI 在同一 `select_pixel` 话题上发送 `point.x < 0` 或 `point.y < 0` 的
  `geometry_msgs/PointStamped` 可取消选框；选中框高亮随之消失，云台的单目标
  话题立即发布空数组。
- `selected_tracked_detections` 是 `TrackedDetection2DArray`，选中后每帧只含
  该目标；未点击或 YOLO 当前帧未检出目标时为空。UI 重新点击可切换目标。
- `annotated` 是唯一图像输出，保留全部检测框，并用红色粗框高亮所选目标。
- 云台节点不由该 launch 启动。它的输入参数预设为单目标话题，且在 UI 首次
  选中目标前该话题只发空数组，因此云台不会提前接管。

车辆任务统一从 `xd_uav_track/launch/vision_tracking_stack.launch` 启动。
该入口默认使用 train7 TensorRT FP16 检测、单实例双相机调度、预测 ROI、
像素辅助和 `xd_uav_track` 的统一身份管理；`enable_tracking:=false` 为纯识别模式。
Gazebo 演示另用 `moving_uav_vehicle_search_demo.launch`，不应作为实机入口。
