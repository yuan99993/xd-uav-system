# 观测语义、部署与网页/Gazebo 联调更新

日期：2026-10-04。本文区分代码实现、接口回归和身份/实景验收。

## 观测合同

`DetectionCandidate.measurement_source`：

- 空字符串：兼容旧发布器，按 detector 处理。
- `detector`：真实 YOLO 全图或 ROI 检测。
- `visual_tracker`：像素辅助图像测量，只更新已有框，不创建/确认新身份，不更新类别/ReID 原型。
- 未知字符串：拒绝，不默认为实测。

`TrackState` 新增 `measurement_source` 与 `last_detection_stamp`。
`last_observation_stamp` 表示最近接受的图像测量，可能是视觉辅助；
`last_detection_stamp` 仅由真实 detector 更新。视觉辅助输出不是 Kalman 预测：
`detected=false, predicted=false, measurement_source=visual_tracker`。
视觉质量与 YOLO 置信度分开管理；视觉辅助不制造三维测量，定位节点和 tracker 都会
拒绝将其作为新 metric 证据。网页分别显示 DET/SOT/PRED/LOST。

新增消息字段改变 ROS MD5：必须同时构建并重启发布/订阅包。旧 rosbag 需做兼容转换，
不能直接连接旧 C++ 二进制与新 Python 消息。

## 来源时间段

重复时间戳和普通小范围乱序继续拒绝。模式管理器、共享 detector 与 C++ tracker
对超过 5 秒的回退建立新的来源时序段。模式管理器清除该来源帧、轨迹快照、图像证据
与 SOT 模板，关闭其轨迹段并将选择状态标为丢失；其他来源的证据/身份保留。
共享推理通过 generation 使旧推理/ReID 结果失效，并在批次边界清除临时缓存。
模式管理器与 detector 拒绝超过 ROS 当前时间 1 秒的未来帧，防止回退前迟到帧污染新段。

这是 ROS 同一时基下的恢复，不是设备独立 uptime 时钟的自动标定。
小于门限的复位需设备驱动重建合法 ROS 采集时间；不能把任意乱序都当重启。

## 部署安全与依赖

本机 Noetic rosbridge 的 `address` 原来只改变 URL，底层 listenWS 未传 interface。
`bound_rosbridge_websocket.py` 复用上游协议实现，但明确绑定 TCP interface。
实际检查 HTTP 8080 与 WebSocket 9090 都是 `127.0.0.1`，不是全网卡。
远程部署需显式地址、受控网络与认证/TLS 反向代理；允许列表不等于身份认证。

`xd_uav_perception_bringup` 是薄组合包，声明 tracker、detector、localizer 的运行依赖。
部署请一起提供它；不反向添加造成环的 tracker → detector 依赖。
本机已安装模型、消息、脚本、网页和资源，并从 install 启动实际共享 TensorRT/Gazebo 链路。

## 联调入口

仅在工作区启动：

```bash
cd /home/promise/mrs_test
source /opt/ros/noetic/setup.bash
source install/setup.bash
export SAR_YOLO_PYTHON=/home/promise/mrs_test/.venv-sar-gpu/bin/python
roslaunch xd_uav_track perception_gazebo_web_test.launch \
  gui:=false metrics_dir:=/home/promise/mrs_test/logs/my_perception_test
```

浏览器访问 `http://localhost:8080/`，命名空间为 `moving_uav_search`，点击连接。
固定/云台两个来源使用同一 `perception/video` 话题，各自保留 640×480 原分辨率。
GUI 默认关闭，避免 Gazebo GUI 和桌面 Qt 叠框争用算力；`gui:=true` 可打开地图。
开启云台时自动选择含云台模型的地图。没有桌面显示需求时不启动组合画面节点。

一次有时限的事务测试：

```bash
python3 src/xd_uav_track/scripts/validate_perception_web_runtime.py \
  --rotations 100 --observe-seconds 15 \
  --output logs/model_rotation_report.json
```

测试会真实切换模型，期间 train/train4 的误检特点仍存在；结束成功后恢复 train7。
请求 ID 每次运行随机生成，避免碰撞旧去重缓存。ROS 初始化期限 15 秒，WebSocket
连接期限 20 秒，主体总期限 240 秒；不会无限等待。测试读取视频，不复制一套 YOLO。
`--rotations 0 --select-target` 可测试真实目标选择、局部 SOT 与公开 UID 保持。

`passed` 仅代表事务、视频同帧与所选显存增长门限，不代表 HOTA/IDF1 或实机认证。

## 仿真定位适配与边界

固定/云台投影统一使用框中心。旧配置一个用中心、一个用底部，会把同一辆车投影为
不同点。本场景下 bbox 到车辆参考点的误差远大于纯像素噪声，因此仿真配置的位置
标准差设为 1 m；保留世界距离、协方差上限、最佳/次佳歧义门控和三次连续确认。
2 FPS 云台的确认间隔改为 1.25 秒，不降低确认次数。
这些是仿真物体/地平面代理参数，不是实机云台/LRF 的标定结果。

首次无可用世界位置时保留后续拼接资格；有拼接候选但仍在确认/歧义阶段的目标标为
tentative，不当成已确认身份显示。仍需长期遮挡、真实交叉标注与不同光照的独立数据验收。

实测低负载视频到 WebSocket 的 P95 存在 320–440 ms，尚不能宣称达到 200 ms。
该场景相机输入 4 FPS、检测 2–3 FPS，也不证明 15 Hz 观测目标已达到。
