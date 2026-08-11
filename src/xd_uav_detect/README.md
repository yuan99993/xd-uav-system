# xd_uav_detect

`xd_uav_detect` 是 `xd_uav_track` 前面的统一感知层。它接收外部识别器提供的一个或多个
二维框，将激光雷达点投影到图像，在框内选择最近的有效深度点簇，补充目标相对三维位置
和协方差，然后统一发布：

```text
/uav1/track/detections    xd_uav_track/DetectionArray
```

数据流：

```text
外部二维检测 ───────────────────────────────┐
相机 Image + CameraInfo ────────────────────┤
LiDAR PointCloud2 + T_camera_lidar + TF ────┴─> xd_uav_detect
                                                     │
                                             DetectionArray
                                                     │
                                               xd_uav_track
```

## 外部检测接口

外部识别器向下面的话题发布二维候选：

```text
/uav1/detect/input/detections_2d    xd_uav_track/DetectionArray
```

输入可以只有一个候选，也可以同一帧包含多个候选。识别器应当填写 `header.stamp`、图像
宽高、bbox、confidence、class_id；如果识别器已经提供可靠三维位置，融合节点会保留它，
不会覆盖。

没有同步点云、标定未启用、框内没有足够点或 TF 不可用时，节点仍会发布原二维候选，只把
`range_valid` 保持为 false。因此图像跟踪不会因为雷达暂时失效而中断。

## 标定参数

将 `velo2cam_calibration` 生成的 `velo2cam_tf` 六个数直接填入
[detect.yaml](config/detect.yaml)，不需要手工计算矩阵，也不使用生成文件中的 frame 名：

```yaml
calibration:
  calibrated: true
  translation_xyz: [x, y, z]
  rotation_ypr: [yaw, pitch, roll]
```

例如生成文件中：

```text
args="-0.253366 -0.0435222 0.185053 0.0100813 0.00255071 -0.00259498 ..."
```

前三个数填入 `translation_xyz`，后三个数原样填入 `rotation_ypr`。节点内部负责转换为
相机投影需要的光学坐标变换。

目标三维位置最终通过 TF 转换到 `frames/body` 的 ROS FLU 坐标，再转换为
`DetectionCandidate` 约定的 forward/right/down。TF 中必须存在：

```text
frames/body <- PointCloud2.header.frame_id
```

## CameraInfo

`Image.width/height` 用于填写输出图像尺寸；`CameraInfo` 仍用于获取投影所需的 fx、fy、cx、
cy。当前投影模型适用于与 CameraInfo 的 `P`/`K` 对应的去畸变图像。若外部检测基于原始
畸变图像，应先使用 `image_proc` rectification，或后续加入畸变模型投影。
如果外部检测框使用等比例或非等比例缩放后的图像坐标，节点会按 DetectionArray 中的
`image_width/image_height` 自动缩放针孔内参。

## 启动

```bash
UAV_NAME=uav1 roslaunch xd_uav_detect detect.launch \
  point_cloud_topic:=/实际点云话题
```

同时启动感知和跟踪：

```bash
UAV_NAME=uav1 roslaunch xd_uav_detect detect_track.launch \
  point_cloud_topic:=/实际点云话题
```

状态查看：

```bash
rostopic echo /uav1/detect/status
rostopic echo /uav1/track/detections
```

状态中的 `fused` 表示本帧成功获得三维位置的候选数量。

## 点云投影调试图像

节点发布原始相机图像、按深度着色的雷达投影点、检测框、目标距离和融合状态：

```text
/uav1/detect/debug/image    sensor_msgs/Image
```

使用下面命令查看：

```bash
rqt_image_view /uav1/detect/debug/image
```

只有该图像存在订阅者时节点才执行绘制，不查看时不会产生额外的逐点绘图开销。色条范围、
点大小和显示降采样可在 `visualization` 配置段调整。

## 红色目标仿真检测

红色识别脚本现在属于感知包，直接发布统一二维候选：

```bash
python3 /home/kzy/xd-uavsystem-test/src/xd_uav_detect/scripts/red_box_detector.py
```

脚本会把同一帧中所有满足面积和尺寸阈值的独立红色区域分别发布为候选框；
`maximum_targets:=0` 表示不限制数量。脚本只负责检测，稳定目标 ID 由
`xd_uav_track` 的多目标关联模块分配。

默认输出 `/uav1/detect/input/detections_2d`。即使尚未填写雷达标定参数，融合节点仍会把
二维检测透传到 `/uav1/track/detections`，供 `xd_uav_track` 使用。
