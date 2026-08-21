# xd_uav_detect

`xd_uav_track` 前的统一感知层：接收外部二维检测，按相机标定把 LiDAR 点投影到图像，
为候选补充相对三维位置和协方差。点云、TF 或标定不可用时仍转发二维候选，并将
`range_valid` 置 false。

```text
输入  /uavX/detect/input/detections_2d  xd_uav_track/DetectionArray
输入  Image / CameraInfo / PointCloud2
输出  /uavX/track/detections             xd_uav_track/DetectionArray
输出  /uavX/detect/status
输出  /uavX/detect/debug/image
```

标定与话题配置位于 `config/detect.yaml`；`translation_xyz` 和 `rotation_ypr` 必须来自实际
标定，不得用 frame relabel 代替。

```bash
roslaunch xd_uav_detect detect.launch UAV_NAME:=uav1
roslaunch xd_uav_detect detect_track.launch UAV_NAME:=uav1
```
