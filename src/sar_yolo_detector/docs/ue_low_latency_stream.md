# UE 持续相机通道

UE 5.5 编辑器中的 SarCameraStream 原生插件对 SceneCapture2D 进行异步 GPU 回读，通过 loopback TCP 持续发送带采集时间、尺寸、实际姿态、视场角的 SRF1 帧。帧不经过磁盘。ROS 消费者独立接收、解码和发布；云台反馈、控制、场景任务、网页显示分别调度。

## 环境与安装

Windows UE5.5、Python Editor Script Plugin、Editor Scripting Utilities；WSL/Linux ROS Noetic、Python3、numpy、OpenCV，以及两包已有 D-FINE-S V3 运行环境。随包原生编辑器二进制对应 UE5.5.4 / BuildId 37670630；其他引擎构建使用包内源码编译。

```bash
rosrun sar_yolo_detector install_ue_capture.py \
  --project /mnt/d/UE/Project/pro1 --engine /mnt/d/UE/UE_5.5 \
  --install-startup --install-stream-plugin
```

安装前校验二进制 SHA256；指定 `--engine` 时同时校验 BuildId。启用插件后重启编辑器。已有场景与启动脚本保留；项目描述文件有备份。

需要重新编译时，在 Windows PowerShell 运行包内 `scripts/build_ue_stream_plugin.ps1 -EngineDir D:\UE\UE_5.5`。需要 VS2022 C++/Windows SDK，或通过 `-AutoSDKRoot` 指定已有 UE AutoSDK。脚本只编译 Win64 编辑器模块并生成新的完整性清单。安装目录的脚本可用 `-PluginDir` 指定 `share/sar_yolo_detector/ue_plugin/SarCameraStream`。

## 启动

```bash
roslaunch sar_yolo_detector ue_dfine_v3_live.launch \
  ue_project:=/mnt/d/UE/Project/pro1 \
  transport:=tcp capture_fps:=30 stream_codec:=jpeg web_fps:=5
```

默认 `pose_source:=px4` 启动所属 SITL/MAVROS/任务并读取实际飞行姿态；需设置 `PX4_AUTOPILOT_DIR`。已有外部实例可用 `start_px4:=false start_mission:=false`。显式 `pose_source:=kinematic` 使用理想任务航线且不启动 SITL。详见 [空地目标几何与 PX4 入口](ue_ground_air_geometry.md)。四模式、Smart/BoT-SORT 身份管理、特征/颜色/LK 分支及 SAR→XD 数据流仍由原有算法处理。

`stream_codec:=bgra8` 是无损原始像素；`jpeg` 使用质量 90、降低传输带宽。`capture_fps` 可设 1–60，实际帧率取决于渲染、回读和计算负载。`web_fps` 仅控制浏览器预览。旧文件通道可显式选择 `transport:=files`。

默认自动选择直接 TCP；不支持 WSL localhost 转发时使用 Windows 二进制 stdio 中继。可以显式指定 `stream_connection:=direct` 或 `windows-relay`。本通道监听本机回环地址，启动时使用随机会话令牌。

## 时间与积压控制

- 采集元数据在渲染提交时固定，异步完成后仍属于原帧。
- 原生回读最多 3 个在途槽，已完成帧与接收端各只有一个最新槽；过期帧和乱序帧被丢弃。
- ROS Image、CameraInfo 和相机 TF 共用采集时间，光学坐标系标识一致。
- UE/ROS 时钟通过七次往返估计偏差与误差上限，并保留 1 ms 的 UTC 分辨率误差；不把网络延迟强制归零。
- 云台只接受有相应采集姿态的当前可信视觉测量；本地视觉更新与检测器更新分开，预测框不能刷新控制。
- 输出目录 `capture_summary.json`、`frame_timing.json`、`control_timing.json`、`actuator_timing.json` 分别记录采集→ROS、采集→控制输出、采集→UE 执行器应用时间。`--stream-only` 不生成逐帧图像。
- 编辑器纹理/着色器首次编译会暂停渲染，应在验收前完成；超时或断流会明确失败并让云台停止续用旧测量。

完整链路验收使用 `verify_ue_gimbal.py` 和 `verify_dfine_pipeline.py --external-camera`，后者不会发布测试图片覆盖真实相机。

## 独立帧率核验

在独立测试命名空间中选定目标后：

```bash
rosrun sar_yolo_detector verify_ue_stream.py --namespace ue_test \
  --web-url http://127.0.0.1:8787 --seconds 15 --require-control --cpp-probe \
  --output /tmp/ue_stream_cadence.json
```

C++ 只读探针统计真实图像、CameraInfo、唯一处理帧、有效视觉状态和反馈。网页 `/api/telemetry` 返回与当前图像相同的数值信息而不重复传输 PNG，便于观测时避免大图像探针自身的 Python/GIL 丢帧。ROS 原始图像仍由正式算法消费。

高帧率下的原帧 ROI 缓存按 32 帧、48 MiB、0.75 秒三重限制保留，未延长可选择图像的时间门限。推理数学库默认限制线程数（OMP/MKL 2、OpenBLAS 1），可以用同名环境变量显式覆盖。旋转/镜头补偿持续保留四边形像素支持区域，测量修正后再投影到当前帧取框，避免反复取外接矩形造成人工膨胀。详见 [虚拟镜头与视场角验收](ue_virtual_zoom.md)。
