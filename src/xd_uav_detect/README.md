# xd_uav_detect

`xd_uav_detect` 是 `xd_uav_track` 前面的统一三维感知层，现支持两种定位方法：

- 多旋翼：把激光雷达点投影到相机，在二维框内选取最近有效点簇。
- 固定翼：用下视相机的检测框中心射线与水平地面平面求交。

两种方法都输出同一个接口：

```text
/<uav>/track/detections    xd_uav_track/DetectionArray
```

目标相对位置写入每个 `DetectionCandidate` 的 `relative_position_body`，坐标约定为
`forward/right/down`，并设置 `has_relative_position_body=true`、`range_valid=true` 和
`position_covariance`。因此 `xd_uav_track` 以及后续任务分配不需要针对固定翼修改输入接口。

## 输入检测框

外部识别模型向下面的话题发布检测结果：

```text
/<uav>/detect/input/detections_2d    xd_uav_track/DetectionArray
```

需要填写：

- `header.stamp`：原始下视图像的拍摄时间；固定翼姿态变化快，不能使用推理完成时间。
- `image_width`、`image_height`。
- 每个候选的 `bbox`/`has_bbox`，或 `normalized_bbox`/`has_normalized_bbox`。
- `confidence`、`class_id`，以及识别器已有的其他字段。

如果上游已经填写可靠的三维位置，本节点会保留而不覆盖。定位条件不满足时，二维候选仍会
发布，但 `range_valid` 保持为 false。

## 多旋翼配置

配置文件为 [multirotor_detect.yaml](config/multirotor_detect.yaml)：

```bash
roslaunch xd_uav_detect detect.launch \
  UAV_NAME:=uav1 \
  config:=$(rospack find xd_uav_detect)/config/multirotor_detect.yaml
```

将 `velo2cam_calibration` 生成的六个数复制到：

```yaml
calibration:
  calibrated: true
  translation_xyz: [x, y, z]
  rotation_ypr: [yaw, pitch, roll]
```

点云三维位置通过 TF 转到 `frames/body` 的 ROS FLU 坐标，再转换成消息要求的 FRD。TF
中必须存在：

```text
frames/body <- PointCloud2.header.frame_id
```

## 固定翼下视定位

配置文件为 [fixedwing_detect.yaml](config/fixedwing_detect.yaml)：

```bash
roslaunch xd_uav_detect detect.launch \
  UAV_NAME:=uav1 \
  config:=$(rospack find xd_uav_detect)/config/fixedwing_detect.yaml
```

固定翼模式不订阅点云。节点将框内锚点反投影成相机光线，并在 `frames/world` 中与
`z = ground_projection/ground_plane_z_m` 相交。必须满足：

```text
frames/world <- 下视相机 optical frame
frames/body  <- 下视相机 optical frame
```

推荐让 `CameraInfo.header.frame_id` 填写准确的相机 optical frame，并保持配置中的
`frames/camera: ""`；也可以显式设置该参数。`frames/world` 必须使用 z 轴向上的局部世界
坐标系；与任务分配联用时应和 `mission/shared_frame` 一致（本工程为 ENU `world`）。
`ground_plane_z_m` 是该世界坐标系中的绝对 z，
并不是飞机离地高度。

固定翼实机使用前至少要确认：

- 下视图像与 `CameraInfo` 来自同一相机和同一成像尺寸；若检测使用缩放图像，节点会按
  `DetectionArray.image_width/image_height` 缩放内参。
- 当前针孔反投影要求检测使用与 `CameraInfo.P/K` 对应的去畸变图像；若模型输入为原始
  畸变图像，应先用 `image_proc` 矫正。
- 相机外参已发布到 TF，且 TF 缓存覆盖检测消息的拍摄时间。
- `ground_plane_z_m` 符合任务区域的地面高程；地形起伏较大时，水平平面模型会产生系统
  误差，应接入 DEM 或测距信息后再执行真实任务。
- `box_anchor_y_ratio` 默认 0.5（框中心）。若识别目标的位置定义在框底，可设为 1.0。

`pixel_stddev_px`、`ground_height_stddev_m` 和 `position_stddev_m` 会传播到输出协方差。
`minimum_ray_plane_angle_deg` 会拒绝过于接近地平线的射线，避免位置误差失控。

## 同时启动跟踪

多旋翼（默认）：

```bash
roslaunch xd_uav_detect detect_track.launch UAV_NAME:=uav1
```

固定翼：

```bash
roslaunch xd_uav_detect detect_track.launch \
  UAV_NAME:=uav1 \
  detect_config:=$(rospack find xd_uav_detect)/config/fixedwing_detect.yaml
```

## 状态与调试图像

```bash
rostopic echo /uav1/detect/status
rostopic echo /uav1/track/detections
rqt_image_view /uav1/detect/debug/image
```

状态中的 `method` 为 `lidar_camera` 或 `ground_plane`，`metric` 表示本帧成功获得三维位置
的候选数量。多旋翼调试图显示雷达投影深度；固定翼调试图显示检测框、解算距离和地面投影
状态。

## 红色目标仿真检测

```bash
python3 src/add_red_box_scripts/red_box_detector.py --uavs uav1 uav2
```

脚本只负责发布二维候选；稳定目标 ID 由 `xd_uav_track` 分配。固定翼仿真时还必须提供正确
的下视相机 `CameraInfo` 和拍摄时刻 TF，才能得到三维位置。
