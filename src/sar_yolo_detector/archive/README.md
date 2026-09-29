# 已归档的旧启动入口

`archive/launch/` 保留旧 COCO、救援、热红外、OpenCV 和上游 SmartTracker
启动文件，仅供历史参考；它们不随正常安装空间发布，也不是车辆识别/跟踪入口。
归档文件之间的 `include` 已改为指向本目录。部分历史入口还依赖当前工作区
未安装的外部包，不能保证直接运行。
旧单路 Python 检测器、启动包装器及对应历史测试保存在 `archive/scripts/`
和 `archive/test/`；正式单路入口现在复用共享检测器实现。

车辆任务统一从 `xd_uav_track/launch/vision_tracking_stack.launch` 启动。
该入口默认使用 train7 TensorRT FP16 检测、单实例双相机调度、预测 ROI、
像素辅助和 `xd_uav_track` 的统一身份管理；`enable_tracking:=false` 为纯识别模式。
Gazebo 演示另用 `moving_uav_vehicle_search_demo.launch`，不应作为实机入口。
