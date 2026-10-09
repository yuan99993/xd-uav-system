# 检测、跟踪与融合：使用、跨主机部署和网页接入指南

校订日期：2026-10-07。以当前 `xd_uav_track`、`sar_yolo_detector` 源码、消息定义和 launch 为准。

本指南描述已经实现的接口，不把实施方案中的设想当作现有能力。它不是实机飞行操作规程，也不代表三种模式已经取得完整实拍、长遮挡和跨相机验收。

建议阅读顺序：首次操作看第 1、3、4 节；网页开发看第 5～11 节；部署另一台主机看第 10、12～14 节；测试和排错看第 15、16 节。

目录：

- [三种模式与算法](#3-三种业务模式与算法选择)
- [启动和显示](#4-启动先单路再双路)
- [当前 Gazebo 与网页操作](#46-当前-gazebo--网页联调入口)
- [公共接口](#5-公共接口总表)
- [命令与原帧选择](#6-三类命令共用的事务合同)
- [目标坐标与观测状态](#7-统一目标输出像素和三维位置)
- [运行状态](#8-perceptionstate网页恢复与可操作状态)
- [轨迹](#9-目标轨迹输出)
- [模型和 TensorRT](#10-yolo-模型注册与事务式切换)
- [网页接入](#11-网页端接入和视频帧适配)
- [跨主机安装](#12-跨主机部署依赖安装空间与模型适配)
- [独立感知入口](#125-独立感知部署不绑定运动学或飞行方案)
- [配置适配](#13-配置适配清单)
- [内部接口](#14-内部话题设备输入与控制边界)
- [诊断与回归](#15-性能诊断与低负载回归)
- [错误码和上线检查](#16-错误处理已知缺口与上线检查)

## 1. 先明确默认行为与目前的限制

2026-10-04 收尾实现和实际网页/Gazebo结果见
[补齐验收报告](perception_completion_acceptance_2026_10_04.md)与
[观测/部署/联调更新](perception_observation_and_regression_update.md)。
注意跨源 ID 与 200 ms 时延仍未通过，不能把接口联调当作上线认证。

本次更新包含：融合选中目标的转弯连续性修复、逐次视频更新复测、网页
模型下拉选择保持、实际 TensorRT 模型轮换测试和独立高频像素辅助的
实验开关。修复详情见 [融合锁定恢复记录](fusion_selected_lock_recovery.md)，
整体升级门限见 [高频辅助验收记录](high_frequency_upgrade_acceptance.md)。
四项算法升级**尚未全部完成严格验收**，不要把这次局部修复理解为全部
性能门限、实拍身份准确率或实机控制已经通过。

2026-10-07 联合效率优化不改变上述能力边界：缩短共享检测器的 GPU 锁
范围、仅复制保留的像素锚点、复用全局匹配临时数组，并增加 GPU 等待/
整批处理诊断；模型、阈值、ID 与公共消息合同保持不变。细节及测试范围
见第 15.5 节。接口清单按当前正式链路核对，旧独立演示合同单独列示，
不能把包内存在的 srv 文件理解为默认已启动的服务。

1. **独立感知部署推荐 `xd_uav_perception_bringup/perception.launch`**，配置与命令见第 12.5 节。它只运行检测、身份、视觉锁定、轨迹、失联状态和可选网页，不启动仿真或飞行策略。每机只运行一套共享 YOLO，不要两路相机分别加载模型。
2. 新的纯感知入口直接启动三模式管理器，初始 `detect`。兼容 `xd_uav_track/vision_tracking_stack.launch` 的 `enable_perception_modes` 仍默认 `false`，联调需显式 `true`；其默认 follower/定位编排与纯感知入口不同。功能可部署不等于跨源身份、严格时延或实景精度已全部认证。
3. 初始模式是 **`detect`**；纯感知配置使用 `perception_mode_manager.initial_mode`。兼容入口可传 `initial_perception_mode:=fusion`；运行中均通过模式命令切换。
4. `detect` 也是带稳定 ID 和短时视觉补偿的多目标感知，不是每帧给目标重新编号。`enable_tracking:=false` 则是另一种“仅 YOLO 检测”启动方式，**不提供本指南中的完整三模式、稳定身份、人工 ROI 和轨迹**。
5. 感知选择与飞行控制分开。网页选中目标、切换 `track/fusion` 不会自动解锁、起飞、启动云台电机或启用 follower；本指南不开放这些控制权限。
6. 新模式下提供原分辨率叠框视频 `perception/video`，网页可显示类别、ID、中心点和观测状态，支持点击/拖拽、模式和模型切换。视频用无损 PNG 与同帧目标快照打包；真实相机/云台驱动仍需另外部署。
7. 当前外部依赖包 `xd_uav_detect` 的主节点只接受 `lidar_camera`、`ground_plane`，而兼容入口的默认云台配置填写 `gimbal_lrf`，可能报方法不支持。**纯感知入口默认不启动这个定位器**，两路检测直接送入身份层，输出原图像素位置；未接可靠定位时 `position_valid=false`。这不是云台/LRF 三维定位已经接通。
8. 模型列表中的活动模型 ID 已修正。网页切换成功仍以命令终态和 `perception/state.active_model_id/model_epoch` 为准，不能只看下拉选项或文件名。
9. `enable_high_frequency_pixels` 是另一独立实验开关，正式入口默认 false；只有同时启用 tracking、perception_modes 与此开关才启动模式管理器的独立 latest-only 像素工作线程。既有像素辅助与此线程不是同一个执行路径。第 4.6 节复测显式开启，不代表生产默认已改变。
10. detect/track/fusion 是感知模式，不绑定高度、悬停、航线或机型运动策略。当前演示的总览升高/悬停只属于 Gazebo driver；大地图部署不能直接把该 driver 当任务规划器。外部飞行模块接入边界见第 13.4 节。

网页接入增加了传输、显示和交互，不替换 YOLO/跟踪算法或识别权重。兼容入口保留旧总开关，纯感知入口直接提供三模式。视频与接口文件见 [网页视频与交互说明](perception_web_video_interface.md)。

## 2. 包的职责和完整数据链

```text
固定/云台相机 Image ──→ sar_yolo_detector：共享 YOLO、ROI/像素辅助、异步外观编码
                              │ 各源 DetectionArray
                              ▼
                        xd_uav_detect：可选定位、2D 转发
                              │ detect/detections
                              ▼
                  xd_uav_track C++：局部轨迹、全局实体、ReID/轨迹片段拼接
                              │ track/tracks_by_source
                              ▼
原图 Image ──→ perception_mode_manager：detect/track/fusion、SOT、事务、统一目标与轨迹
                              │
                       targets / trajectory / state
                              ├──→ 同源叠框 → tracking_visualization
                              └──→ rosbridge → 网页
```

- YOLO 负责目标框、类别和检测置信度；正式链路的身份所有权属于 `xd_uav_track`。上游候选不是最终实体 ID。
- ReID 的特征编码可以在共享检测进程的独立工作线程执行，但轨迹特征库、重捕获与公开身份仍由 tracker 管理。
- C++ 多目标主干保留两阶段高/低置信关联、全局分配、实际时间差 Kalman、外观/运动门控、长期身份和 Tracklet Stitching。
- 新模式管理器在此基础上增加单目标局部跟踪和观测裁决，**没有用 KCF 替换多目标主干**。
- 显式开启高频辅助后，模式管理器可在 YOLO 推理之间独立推进已确认目标，按来源保存有界同帧光流证据；YOLO 仍负责发现目标和纠正漂移。输入帧率、计算负载和证据质量限制更新频率，不能据此保证每帧必有框。
- 当前车辆 ReID 为 `vehicle-reid-0001.onnx`，默认 `opencv_dnn` CPU、512 维车辆域特征；不是坦克专用验证模型。模型不可用时可退化为 hybrid 弱外观线索。不要把默认 ReID 宣称为 TensorRT GPU 推理。
- 两路没有可靠共同世界坐标、时间同步与门控证据时，可以分别跟踪，但不能保证同一物体在两个相机中拥有同一个全局 ID。

## 3. 三种业务模式与算法选择

| 行为 | `detect` 检测 | `track` 跟踪 | `fusion` 融合 |
|---|---|---|---|
| 默认用途 | 全局搜索、发现和多目标身份维持 | 对一个指定实体持续局部跟踪 | 全局发现 + 对选中实体局部跟踪 |
| 输出对象 | 各来源全部有效目标 | 仅选中目标；无选择时目标数组可为空 | 各来源全部目标，合入选中目标 SOT 结果 |
| 正常全图 YOLO | 按正常调度持续运行 | 跟踪正常时，选中源默认上限 1 Hz，其他源默认上限 0.5 Hz | 按正常调度持续运行 |
| 无选择或目标丢失 | 继续搜索 | 恢复正常搜索调度 | 继续全图搜索 |
| 局部 SOT | 不要求主动选择；既有人工视觉身份可有界后台维护 | 选中目标 SOT，由低频检测作同帧几何、运动和外观校验 | 选中目标 SOT 与检测、几何、运动、弱外观共同校验 |
| 轨迹输出 | 不新增选中对象的连续轨迹；旧历史仍受 TTL 限制 | 选中实体的一条有界轨迹 | 选中实体的一条有界轨迹 |

正常检测调度的正式入口默认 `detector_fps=4.0`，是每源调度上限，不是实际吞吐保证。track 锚定频率还受原调度上限约束；局部 SOT 随可处理的新鲜图像运行，不等于 1 Hz。detect/fusion 也不会关闭既有像素辅助。

track 也使用检测锚点校验 SOT，而不是彻底停用检测后只盲跟模板；fusion 的主要区别是保持正常全图发现并输出全部目标。

模式转换规则：

- 在 detect 中成功按 ID、点击或 ROI 选择，自动进入 track；想要融合，可再发 fusion 命令。
- 在 fusion 中选择目标仍保持 fusion。
- 模式真正改变或新模型成功提交后自动软初始化：清临时帧/证据/轨迹缓存，可信同帧下重建选中 SOT；公开 ID 和长期身份保留。详见 [切换后的自动初始化](perception_web_video_interface.md#成功切换后的自动初始化)。
- 切换进入 detect 会取消当前主动选择，但不抹掉现有 C++ 实体与尚有效的后台人工视觉身份。
- `clear` 仅取消选择，不改变模式；track 随后进入 `WAITING_SELECTION`。
- 当前每机只有一个主动选中目标和一条主动轨迹；detect/fusion 的多目标框不等于多条同时控制的 SOT 轨迹。
- 原 `track/set_target_mode` 的 `all/follow` 属于旧选择/跟随语义，不是这三种模式，网页不要混用。

`tracker_algorithm` 是另一组枚举：

| 值 | 行为 |
|---|---|
| `auto` | 依次尝试 CSRT、KCF、MIL；均不可用时才尝试模板跟踪 |
| `csrt` | 显式请求 CSRT；不可用则拒绝，不悄悄换算法 |
| `kcf` | 显式请求 KCF；不可用则拒绝 |
| `mil` | 显式请求 MIL；不可用则拒绝 |
| `template` | 模板匹配回退；低纹理 ROI 可能初始化失败 |

实际执行算法应读取 `state.active_tracker_algorithm`，不能仅依据请求字段。SOT 不识别类别、不证明长期身份，也不能保证完全遮挡期间真实贴框。

## 4. 启动：先单路，再双路

以下命令互为不同启动方案，不要在同一 UAV 命名空间中全部同时运行。相机驱动须提前或随后持续发布图像；本入口不创建图像、不启动 Gazebo/PX4，也不弹出图像窗口。

### 4.1 初始化环境

本机示例；换主机时替换工作区和 Python 路径：

```bash
cd /home/promise/mrs_test
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export SAR_YOLO_PYTHON="$PWD/.venv-sar-gpu/bin/python"
```

前提是上述虚拟环境确实已经安装并可运行。安装空间部署改为 source `install/setup.bash`，不要同时叠加旧工作区的 devel 环境。

### 4.2 单固定相机：三模式入口

```bash
roslaunch xd_uav_track vision_tracking_stack.launch \
  UAV_NAME:=uav1 \
  enable_perception_modes:=true \
  enable_gimbal_lrf:=false \
  enable_sar_gimbal:=false
```

默认订阅 `/uav1/fixed_camera/image_raw`，初始 detect。没有标定、点云或有效 TF 时，固定路可仅转发 2D 候选；此时三维位置必须视为无效。

如果实际驱动的话题不同，直接适配名称，不改代码：

```bash
roslaunch xd_uav_track vision_tracking_stack.launch \
  UAV_NAME:=drone_a \
  enable_perception_modes:=true \
  enable_gimbal_lrf:=false enable_sar_gimbal:=false \
  fixed_image_topic:=/camera_front/image_raw \
  fixed_camera_info_topic:=/camera_front/camera_info \
  fixed_image_source:=front_rgb \
  fixed_sensor_id:=front_camera
```

以 `/` 开头的输入为绝对话题；不以 `/` 开头的名称位于 UAV 命名空间内。`image_source` 是业务来源标识，不是 ROS topic，也不是 TF frame。

### 4.3 固定相机 + 云台：当前可用的纯视觉配置

先在自己的部署包中保存下面的 YAML，例如 `config/gimbal_visual_only.yaml`。这是配置示例，不是包内已存在的文件：

```yaml
localization:
  method: lidar_camera
  enabled: false
calibration:
  calibrated: false
interfaces:
  output:
    detections_3d: detect/detections
    status: detect/gimbal/status
    debug_image: detect/gimbal/debug_image
visualization:
  enabled: false
```

必须同时设置 `method: lidar_camera` 和 `enabled: false`：保留不支持的 `gimbal_lrf` 再关闭 enabled，仍会在构造时失败。输出必须接 `detect/detections`，不能落到节点默认的 `track/detections`，否则 tracker 收不到该路观测。图像、内参、来源和 sensor_id 由顶层 launch 覆盖。

```bash
roslaunch xd_uav_track vision_tracking_stack.launch \
  UAV_NAME:=uav1 \
  enable_perception_modes:=true \
  gimbal_detect_config:=/absolute/path/to/gimbal_visual_only.yaml
```

该方案支持两路视觉检测和各源跟踪，**不提供云台激光目标定位，也不自动转动真实云台**。没有可信世界位置时，跨相机同实体 ID 合并仍会保守拒绝。

### 4.4 固定翼与云台单路

固定翼固定相机入口应同时选择正确定位与 tracker 配置，不能只换 vehicle_type：

```bash
roslaunch xd_uav_track vision_tracking_stack.launch \
  UAV_NAME:=fw1 vehicle_type:=fixedwing \
  enable_perception_modes:=true \
  enable_gimbal_lrf:=false enable_sar_gimbal:=false \
  track_config:="$(rospack find xd_uav_track)/config/fixedwing_metric_orbit.yaml" \
  fixed_detect_config:="$(rospack find xd_uav_detect)/config/fixedwing_fixed_camera.yaml"
```

此命令仍不授权飞行；地面投影需要实际 CameraInfo、相机安装 TF、采集时刻位姿和正确地平面。视觉测试不需要绕飞制导时，应使用自己的不发布控制 reference 的 tracker 配置。

仅云台纯视觉时，在第 4.3 节命令中加 `enable_fixed_camera:=false enable_sar_fixed:=false`；保留 gimbal 两个开关为 true。

四种旧 `tracking_multirotor_*`、`tracking_fixedwing_*` 包装入口目前没有完整转发 `enable_perception_modes`，不能直接给它们传这个参数。新接入使用正式入口，或在自己的 bringup 中明确转发参数。

### 4.5 图像显示与网页桥

在另一个已 source 工作区环境的终端，只选一种显示方式即可：

```bash
rosrun rqt_image_view rqt_image_view /uav1/fixed_camera/tracking_overlay
```

双路合成画面使用 `/uav1/tracking_visualization`。合成图像保留各源原始分辨率，并排放置；右路坐标仍是右路原图坐标，不是合成画面的偏移坐标。

推荐一次启动网页 HTTP 与 WebSocket 桥，不会重复启动检测器：

```bash
roslaunch xd_uav_track perception_web_app.launch \
  UAV_NAME:=uav1 http_port:=8080 websocket_port:=9090
```

浏览器打开 `http://127.0.0.1:8080`，检查 WS 地址和 UAV 命名空间后连接。页面从 `/perception_runtime.json` 读取部署端口和 UAV，提供独立固定/云台视频、类别/ID/位置表、UID/点击/ROI 选择、模式/模型开关、取消选择和轨迹状态。视频无输入或过期时显示等待/LOST，不冒充实时画面。

算法栈需启用 `enable_perception_modes:=true enable_web_video:=true`；可以设置 `enable_tracking_visualization:=false` 不生成桌面合成画面，网页同帧叠框仍可用。网页入口本身不创建相机，也不启动算法或飞行。

只需 WebSocket 桥时使用 `perception_web_bridge.launch`，不要与 web_app 重复启动。静态页面也可单独托管：

```bash
roslaunch xd_uav_track perception_web_bridge.launch UAV_NAME:=uav1
# 以下命令在另一个终端执行：
python3 -m http.server 8080 --bind 127.0.0.1 \
  --directory "$(rospack find xd_uav_track)/web"
```

这种静态托管没有运行配置文件，手填 `ws://127.0.0.1:9090` 和 `uav1` 即可。网页的 canvas 原尺寸等于原图尺寸，CSS 可缩放显示；框选始终转换回原始像素坐标，并绑定实际显示的帧。

### 4.6 当前 Gazebo + 网页联调入口

以下入口**同时启动仿真、共享检测、tracker、三模式、原分辨率网页视频
和 HTTP/WS**。不要与第 4.2～4.5 节另起的同名栈并行；已有演示运行时
直接使用网页，不要再次启动。仅在本机工作区测试，不是 PX4 实机飞行入口。

```bash
cd /home/promise/mrs_test
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export SAR_YOLO_PYTHON="$PWD/.venv-sar-gpu/bin/python"
roslaunch --skip-log-check xd_uav_track perception_gazebo_web_test.launch \
  gui:=true enable_high_frequency_pixels:=true
```

`--skip-log-check` 避免启动时扫描大型 ROS 日志目录；不关闭程序日志。
当前包含 `moving_uav_vehicle_search_gimbal.world`：水泥色地面、移动无人机、
固定/云台相机和三辆连续运动坦克，没有旧假山。该入口默认 gui=false，
上面显式开启 Gazebo；不另开 rqt 窗口。命名空间固定为
`/moving_uav_search`，来源是 `search_fixed`、`search_gimbal`，不是正式设备
配置里的 `fixed_rgb`、`gimbal_rgb`。默认 train7 TensorRT FP16、单共享
实例，固定源正常检测上限 3 Hz、云台 2 Hz；相机输入是 4 Hz，当前原图
640×480。频率是配置上限，不能作为端到端吞吐保证。

浏览器打开 `http://localhost:8080/`；WS 为 `ws://localhost:9090`。
两者默认仅绑定 localhost。页面会读取 `moving_uav_search` 运行配置，
首次连接时检查命名空间。操作顺序：

1. 等待视频、`model_ready=true` 和实际模型 train7；初始 detect。
2. 查看全部已确认目标的类别、ID、原图中心坐标。类别未确认只显示 ID。
3. 测融合时先切 fusion，再点击目标或冻结原帧后拖拽 ROI；选中框为绿色，
   其余有效框继续显示。若在 detect 中选择，会自动进入 track，而不是 fusion。
4. track 只输出选中目标；取消后等待选择，不能把空数组理解为检测器崩溃。
5. 云台画面中选择目标时，此仿真 driver 可按新鲜图像误差限速转动云台；
   真实设备仍需独立驱动和控制权适配。选中固定路目标不等于自动交接云台。
6. 切模型：选择 train/train4/train7 → 点击切换 → 等 APPLIED 和顶部实际
   模型/epoch 更新。下拉框是待提交选择，顶部才是实际运行模型。
7. 更新页面代码后 `Ctrl+F5` 再连接。页面刷新不停止算法，也不重新起 Gazebo；
   manager/仿真整套重启才建立新会话。旧 UID 不应跨会话复用。

本入口 `enable_high_frequency_pixels` 默认仍 false；本轮修复复测显式
开启 true。需要基线对照可改为 false，切换该启动参数需重启整套入口，
它不是网页的 detect/track/fusion 按钮。

## 5. 公共接口总表

以 UAV_NAME=uav1 为例；其他部署仅替换前缀，消息类型不变。

| 完整接口 | 类型 | 使用方向 | 说明 |
|---|---|---|---|
| `/uav1/perception/mode/set` | `xd_uav_track/SetPerceptionModeCommand` | 客户端发布 | 三模式切换，非锁存命令 |
| `/uav1/perception/target/select` | `xd_uav_track/SelectPerceptionTargetCommand` | 客户端发布 | clear、ID/UID、点击、ROI |
| `/uav1/perception/model/set` | `xd_uav_track/SetDetectorModelCommand` | 客户端发布 | 选择注册 model_id，不接受任意文件路径 |
| `/uav1/perception/command_result` | `xd_uav_track/PerceptionCommandResult` | 客户端订阅 | 阶段和终态，非锁存 |
| `/uav1/perception/state` | `xd_uav_track/PerceptionState` | 客户端订阅 | 锁存的当前会话、模式、模型、选择和事务状态 |
| `/uav1/perception/targets` | `xd_uav_track/PerceptionTargetArray` | 客户端订阅 | 各图像源的目标快照，非锁存 |
| `/uav1/perception/video` | `xd_uav_track/PerceptionVideoFrame` | 客户端订阅 | 原分辨率 PNG 叠框图与同帧 targets，按来源分流、非锁存 |
| `/uav1/perception/trajectory` | `xd_uav_track/TargetTrajectoryArray` | 客户端订阅 | 选中实体的有界轨迹，锁存 |
| `/uav1/perception/state/get` | `xd_uav_track/GetPerceptionState` | 客户端调用服务 | 空请求，返回 success/message/state |
| `/uav1/perception/models/list` | `sar_yolo_detector/ListDetectorModels` | 客户端调用服务 | 空请求，返回模型列表和产物信息 |

`models/list` 由共享检测器直接提供；不需要额外列表代理节点。旧 `/uav1/sar_yolo_multi_source_detection/list_models` 是兼容服务，网页应使用公共名称。

验证接口而不改变状态：

```bash
rostopic type /uav1/perception/targets
rosservice call /uav1/perception/state/get "{}"
rosservice call /uav1/perception/models/list "{}"
rostopic echo -n 1 /uav1/perception/state
rosmsg show xd_uav_track/SelectPerceptionTargetCommand
rosmsg show xd_uav_track/PerceptionTargetArray
```

消息完整定义以包内 [msg](../msg) 和 [srv](../srv) 为准。当前是 ROS 1 类型接口，不是 REST，不是 ROS 2 DDS 接口。

## 6. 三类命令共用的事务合同

### 6.1 公共字段

| 字段 | 类型 | 填写要求 |
|---|---|---|
| `request_id` | string | 唯一且 1～128 字符；同请求重试完整复用，不能改任何字段 |
| `session_id` | string | 读取当前 state，必须相同；manager 重启会改变 |
| `expected_state_version` | uint64 | 当前已确认 state 的版本；在真正执行时检查 |
| `issued_at` | ROS time | 推荐使用新鲜 state.header.stamp，或同一 ROS 时间源 |
| `ttl_sec` | float32 | 有限值 0～120 秒；建议 10～20 秒；0 不做普通 TTL 过期检查 |

不要用浏览器 `Date.now()` 直接替代 ROS 时间，尤其 `/use_sim_time=true` 时。issued_at 为零会绕过时间过期检查，因此正式客户端应填写非零、同时间基准的时间。

三类命令共用一个串行 FIFO，默认容量 16。提交过程：

```text
读取 state → 订阅 command_result → 发命令 → QUEUED → ACCEPTED → APPLIED
                                                               或 REJECTED/FAILED
                 等待终态对应的新 state → 再发依赖它的下一条命令
```

- QUEUED 仅表示排队，ACCEPTED 仅表示开始执行；只有 APPLIED 表示提交完成。
- APPLIED 可能先于新 state 到达。下一命令必须等 `state_version >= result.state_version`，并确认会话仍相同。
- 前一命令改变版本后，排队的旧版本命令会返回 STALE_STATE；服务端不会自动改写请求版本。
- 最近 128 个终态有内存幂等缓存；没有跨重启持久化，也不能保证离开缓存后的无限期重试去重。
- `command_result` 附带 `request_id/session_id/command_type/stage/error_code/message/state_version`。command_type 为 `mode`、`model`、`target_select`。
- 事务进行中读 `state.active_request_id/queued_commands/transaction_state`；IDLE、EXECUTING、MODEL_PREPARING 与业务模式是不同概念。

### 6.2 模式与模型附加字段

- `SetPerceptionModeCommand.mode`：`detect`、`track`、`fusion`。
- `SetDetectorModelCommand.model_id`：当前注册的 `train`、`train4`、`train7`。当前消息**没有** preferred_backend 或客户端模型路径字段。

### 6.3 选择字段与实际行为

| 字段 | 类型 | 用途 |
|---|---|---|
| `selection_type` | string | clear / track_id / point / roi |
| `track_id` | int32 | 数值全局实体 ID；使用 UID 时建议 -1 |
| `target_uid` | string | 当前会话的 `session_id:global_id`；有值时作为权威选择身份 |
| `image_source` | string | 目标所在原始图像源，例如 fixed_rgb，不能填话题名 |
| `frame_seq` | uint32 | 对应原始图像帧号；0 可表示未指定序号 |
| `capture_stamp` | ROS time | 原始图像采集时间；不能使用当前浏览器时间代替 |
| `point_norm` | float32[2] | 原图归一化 [x,y] |
| `roi_norm` | float32[4] | 原图归一化 [left,top,width,height] |
| `tracker_algorithm` | string | auto / csrt / kcf / mil / template |

选择示例要点：

1. **按 UID**：selection_type=track_id，target_uid 取 targets，track_id=-1，指定图像源。可不指定帧，服务端寻找最新新鲜轨迹及对应原图。不能用上一次节点会话的 UID。
2. **点击**：必须绑定原帧，点击同帧 confirmed/occluded 目标；点击空白不会新建身份。相互重叠的框可能按较小框选择，歧义场景建议按明确 UID 选择。
3. **ROI**：先定位原图与该帧轨迹；唯一匹配既有目标则选它，否则注册新的 C++ 全局视觉身份。新身份类别先 unknown/class_valid=false，不要求先被 YOLO 识别。
4. **clear**：取消主动选择，其他选择字段不生效，不改变 mode。保留的历史身份并不等于仍在控制目标。
5. 人工身份的后续 YOLO 绑定必须通过同帧、类别/运动/外观证据和连续确认；不能把附近任意检测框立即合入人工身份。

### 6.4 原帧缓存与延迟画面

点击/ROI 必须指定 frame_seq 和/或 capture_stamp；两者都给时必须指向同一原帧。时间匹配默认 1 ms，不能将邻近帧当原帧。

图像缓存默认 **15 帧，不是 15 秒**。30 FPS 输入仅约 0.5 秒，10 FPS 约 1.5 秒；可配置 2～60 帧，但原图缓存会占内存。相机停更默认超过 1 秒会拒绝继续选择。

浏览器画面很旧时，正确结果是 STALE_FRAME，不能改成“最新框碰巧重叠就选择”。应减少视频延迟、增加有界缓存或请求新帧，不能放宽身份安全规则来伪造选择成功。

### 6.5 终端切换模式：不手填会话和版本

下面直接获取当前状态、订阅结果后提交一次 fusion 命令。要切换其他模式，替换 PERCEPTION_MODE；该示例只改变感知，不启动飞行跟随：

```bash
PERCEPTION_UAV=uav1 PERCEPTION_MODE=fusion python3 - <<'PY'
import os
import threading
import time
import uuid
import rospy
from xd_uav_track.msg import SetPerceptionModeCommand, PerceptionCommandResult
from xd_uav_track.srv import GetPerceptionState

rospy.init_node("perception_cli", anonymous=True)
root = "/" + os.environ["PERCEPTION_UAV"].strip("/") + "/perception/"
rospy.wait_for_service(root + "state/get", timeout=10)
get_state = rospy.ServiceProxy(root + "state/get", GetPerceptionState)
response = get_state()
if not response.success:
    raise RuntimeError(response.message)
state = response.state
request = uuid.uuid4().hex
done, terminal = threading.Event(), []

def result_callback(result):
    if (result.request_id == request and result.session_id == state.session_id
            and result.stage in ("APPLIED", "REJECTED", "FAILED")):
        terminal.append(result)
        done.set()

subscriber = rospy.Subscriber(root + "command_result", PerceptionCommandResult,
                              result_callback, queue_size=10)
publisher = rospy.Publisher(root + "mode/set", SetPerceptionModeCommand, queue_size=1)
deadline = time.monotonic() + 5
while ((publisher.get_num_connections() == 0 or subscriber.get_num_connections() == 0)
       and time.monotonic() < deadline):
    time.sleep(0.02)
if publisher.get_num_connections() == 0 or subscriber.get_num_connections() == 0:
    raise RuntimeError("模式命令/结果话题未建立连接")
publisher.publish(SetPerceptionModeCommand(
    request_id=request, session_id=state.session_id,
    expected_state_version=state.state_version,
    issued_at=state.header.stamp, ttl_sec=20.0,
    mode=os.environ["PERCEPTION_MODE"]))
if not done.wait(10):
    raise RuntimeError("未收到终态；先查询 state，勿假定命令未执行")
print(terminal[-1])
if terminal[-1].stage != "APPLIED":
    raise RuntimeError(terminal[-1].error_code + ": " + terminal[-1].message)
print(get_state().state)
PY
```

## 7. 统一目标输出、像素和三维位置

### 7.1 数组是逐源快照，不是跨相机增量事件

`PerceptionTargetArray` 字段：

| 字段 | 含义 |
|---|---|
| `header.stamp` | 对应原图采集时间 |
| `header.frame_id` | 图像帧标签；不代表所有三维 position 的坐标系 |
| `frame_seq` | 原图帧号，优先使用它，不假设 header.seq 与原图相同 |
| `image_source`、`image_width`、`image_height` | 本条快照的来源和原图尺寸 |
| `session_id`、`state_version` | 会话及生效状态版本 |
| `model_epoch`、`model_id` | 产生该快照的模型代次与 ID |
| `mode` | 输出所属业务模式 |
| `targets` | 本来源当前目标列表；空数组也是有效清除更新 |

前端用 `image_source → 最新快照` 存储，收到某一源时替换该源列表，不要把每条数组追加到总列表。再按 target_uid 生成全局目录、按 source+UID 绘制各相机框。同一全局 ID 在两路出现不意味着两路像素坐标相同。

### 7.2 每个 PerceptionTarget 的字段

| 字段 | 使用说明 |
|---|---|
| `target_uid`、`track_id` | 会话内公开身份；ID 不能当作仿真车辆模型名或永久数据库主键 |
| `class_id`、`class_name`、`class_valid` | 类别确认后才能显示为可靠类别；未确认显示“类别待确认”或仅 ID |
| `detection_confidence` | 检测置信度，不是 SOT 质量 |
| `tracking_quality` | 跟踪质量，不是 YOLO 类别置信度 |
| `bbox_xyxy` | int32[4]，原图 [xmin,ymin,xmax,ymax)，右下边界不含 |
| `center_px` | float32[2]，原图框中心，可为半像素 |
| `bbox_cxcywh_norm` | float32[4]，归一化 [cx,cy,w,h]，不是 ROI 的 left/top 格式 |
| `selected` | 当前主动选择标记 |
| `lifecycle_state` | 当前公开输出主要为 confirmed/occluded/lost；内部 tentative 轨迹会过滤；不能仅凭存在 ID 认定有效锁定 |
| `measurement_source` | detector / visual_tracker / prediction / stale |
| `last_detection_stamp` | 最后接受的真实图像观测；名称虽含 detection，也可由视觉跟踪更新 |
| `age_ms` | 最后图像观测至服务器生成输出的年龄，包含上游处理/投递；-1 表示未知，不应当作 0 |
| `position_valid` | 三维定位的权威有效标志 |
| `position_frame`、`position_stamp` | 三维位置独立坐标系和最后真实定位时间 |
| `position_xyz` | float32[3]，有效时为该 FRD frame 的相对米制位置 |
| `position_covariance` | float32[9]，行优先 3×3，单位 m² |

像素约定：**原图左上角原点，x 向右，y 向下**。不是显示窗口左上角，不是图像中心。新 perception 归一化坐标为 [0,1]；不要与旧 follower 内部 [-1,1] 图像误差混用。

```text
cx = (xmin + xmax) / 2       cy = (ymin + ymax) / 2
normalized_center = [cx / image_width, cy / image_height]
roi_norm = [xmin/W, ymin/H, (xmax-xmin)/W, (ymax-ymin)/H]
```

将上述 ROI 转换用于框选时，仍必须携带同一原帧引用；不能直接把 bbox_cxcywh_norm 代入 roi_norm。

### 7.3 观测、预测与失联的显示规则

| measurement_source / 状态 | 推荐显示 | 可解释为当前实测位置？ |
|---|---|---|
| detector，有效 | `类别 ID 数值` 实线，选中绿色、未选中蓝青色 | 是，来自接受的检测图像证据；3D 仍看 position_valid |
| visual_tracker，有效 | 同样仅 `类别 ID 数值` 实线 | 是，视觉跟踪图像证据；不能推断有新的 YOLO 检测或三维定位 |
| prediction，期限内 occluded | 同样仅类别与 ID，黄橙色虚线 | 否，是短时预测 |
| stale / lost | 状态栏 `LOST ID`，不画旧位置实线框 | 否，仅保留身份 |

框上已删除 DET/SOT/PRED 后缀；只是显示简化，`measurement_source`、
生命周期和有效性字段仍保留。下游必须读取这些字段，不能仅凭类别/ID
文字或“框还在”判定实测。类别由身份层确认，显示端不重复识别类别。

融合选中目标不会因一次 SOT 失败就让 stale 覆盖同帧可靠同实体观测。
同帧检测偏离但当前 SOT 通过质量/外观校验时，可拒绝几何修正并继续
当前真实视觉观测；有 LK 时还要求一致。身份歧义、外观冲突不靠最近框
强行恢复，失锁也不自动改选别的公开 ID。顶部状态栏先画，再画目标框，
避免转弯进入顶部区域时框被背景覆盖。详见融合恢复记录；这是保守恢复
机制，不是永久锁死保证。

默认图像预测期限 0.5 秒，可配 0～2 秒，期限计入传输延迟。失联目标会清零框、中心和归一化框；**这些零值不是目标在画面原点**。相机停更还会主动发布清除快照，网页自身也要有图像和数据超时策略。

age_ms 不包含服务器发布之后新增的 rosbridge→浏览器网络排队时间；前端必须再检查实际收包/显示新鲜度，不能只相信消息内的年龄。

已有 ID 稳定不等于框永远准确。完全遮挡、出画、模板漂移或两辆极相似车辆冲突时，正确行为可能是 lost/ambiguous，而不是强行换目标。

### 7.4 三维 FRD 与 TF

有效 position 默认位于 `<UAV_NAME>/base_link_frd`：x 前、y 右、z 下，单位米，是相对机体位置，不是 camera optical 坐标、地图世界坐标或经纬度。

- 图像 optical 常用 x 右、y 下、z 前；ROS base_link 常用 FLU，x 前、y 左、z 上。三者不能仅改 frame 字符串当作变换。
- body_frame/body_frd_frame/world_frame 只是命名参数，不会自动发布 TF。设备/估计器/TF 管理器必须提供实际安装和采集时刻动态链路。
- 显示 position_frame，不要用 targets.header.frame_id 代替。
- 默认定位新鲜度 1 秒；SOT 与预测不是新测距。旧三维值不能用当前消息时间重新盖章。
- position_valid=false 时不使用 xyz 控制，即使数值非零；不把纯图像框转换为虚构目标距离。
- 固定相机 ground_plane 定位需要有效地平面和非近水平射线；山区、台阶或车体中心高度会引入系统误差，必须标定协方差。
- 云台相机安装和角度必须随采集时间可查；激光背景回波不等于目标距离。当前主定位节点的 LRF 接线缺口见第 1、16 节。

## 8. PerceptionState：网页恢复与可操作状态

| 字段组 | 内容与前端用途 |
|---|---|
| `header`、`session_id`、`state_version` | 会话、状态版本和状态心跳时间；重连先同步 |
| `mode`、`operating_state` | detect/track/fusion 与 ACTIVE/WAITING_SELECTION/TRACKING/LOST |
| `selection_active`、`selected_target_uid`、`selected_track_id`、`selected_image_source` | 当前主动身份及来源 |
| `requested_tracker_algorithm`、`active_tracker_algorithm` | 请求算法与真正初始化的算法 |
| `active_model_id`、`active_model_sha256`、`active_model_backend`、`model_epoch` | 实际模型、源 PT 摘要、后端与代次 |
| `model_ready` | 共享模型加载与预热就绪，不是相机健康或召回率证明 |
| `last_error` | 最近诊断文本，不能替代命令终态；成功锚定也可能填 accepted，不以非空直接判失败 |
| `active_request_id`、`queued_commands`、`transaction_state` | 串行事务进度 |

`ACTIVE` 不代表目标已锁定、视频新鲜或模型就绪。至少分别检查：连接/state 心跳、model_ready、对应来源的图像时间和 target.age_ms、生命周期/观测来源，以及 position_valid。

detect 的 operating_state 为 ACTIVE；无选择的 track 为 WAITING_SELECTION，fusion 为 ACTIVE；已有选择且 SOT 有效为 TRACKING，失效/断流为 LOST。selection_active=true 只表示仍保留主动选择，LOST 时也可以为 true，不能据此画真实锁定框。

模式 manager 重启后 session 改变，旧 UID、命令版本、未完成请求必须作废并提示重新选择。ID 没有跨重启持久化保证。不要只用数值 ID 在历史数据库中认定同一辆车。

## 9. 目标轨迹输出

`TargetTrajectoryArray` 包含 header、session_id、state_version、mode、model_epoch、target_uid、track_id、类别/确认状态、active、status、retention_sec、maximum_points 和 points。

每个 `TargetTrajectoryPoint` 包含：capture_stamp、frame_seq、segment_id、segment_reason、image_source、measurement_source、observed、predicted、center_px、center_norm，以及独立 position_valid/frame/stamp/xyz/covariance。

| 默认参数 | 值 | 作用 |
|---|---|---|
| `~trajectory/retention_sec` | 60 s | 时间保留窗口，最大 120 s |
| `~trajectory/maximum_points` | 600 | 数量上限，最大 1200 |
| `~trajectory/sample_hz` | 10 Hz | 接受轨迹点的采样上限，不改变检测频率 |
| `~trajectory/publish_hz` | 2 Hz | 发布完整有界轨迹快照 |
| `~trajectory/gap_sec` | 1 s | 观测长间隔的断段阈值 |

- points 是完整有界快照，不是增量；网页收到后替换，不要无限追加。
- 按 image_source 和 segment_id 分段绘制；源切换、丢失、长间隔、重新选择不应被直线连接。
- observed/predicted 区分图像实测与真实短时预测。LOST 不追加零坐标假点。
- 当前像素轨迹和 FRD 相对位置**不是惯性世界轨迹**；飞行时不能把不同时间的 FRD 点直接连接成地图运动路径。
- 换实体或成功切换模式/模型清空本次临时轨迹；取消选择后已有点仍受 TTL 限制。轨迹锁存不等于永久保存，持久化需另行记录。

## 10. YOLO 模型注册与事务式切换

### 10.1 当前注册模型

注册表：`sar_yolo_detector/config/model_registry.yaml`。路径相对 sar 包解析，适合不同工作区；不要写死 `/home/promise`。

启动会核对注册表中**全部**已登记 PT 和已有 engine 的文件/摘要，不只核对当前活动模型。只复制 train7，却保留引用 train/train4 的完整表，可能在启动时失败。应复制完整登记产物，或用自己的 detector 编排加载裁剪注册表；顶层目前没有 model_registry 参数。注册产物必须在 sar 包根目录内，外部注册表位置可自定义，但不能由网页任意引用包外模型。产物保留正常只读权限，例如 0644，避免 symlink 或 group/world writable 文件被完整性校验拒绝。

| model_id | PT 文件相对路径 | 类别顺序 | validated |
|---|---|---|---|
| train | models/xd_vehicle_latest/xd_vehicle_latest.pt | car、ar-car、tank | false |
| train4 | models/xd_vehicle_train4/xd_vehicle_train4.pt | car、ar-car、tank、m142 | false |
| train7 | models/xd_vehicle_train7/xd_vehicle_train7.pt | car、ar-car、tank、m142 | false |

正式默认仍为 train7。旧 train 出现过较大误检风险，不因某次检测框更多就认定精度更高。validated=false 表示尚无相应实景精度验收，不表示文件无法加载。

网页下拉框表示**准备切换的模型**，顶部“实际模型”表示运行中的模型。
正常状态心跳、模式切换或选目标不应覆盖未提交的下拉选择；实际模型/
epoch 或服务端 session 改变后才同步下拉框。选择模型后点击切换，等到
APPLIED 及实际模型更新再判断成功；失败保留旧模型。更新网页文件后应
刷新浏览器，已经打开的页面不会自动替换其 JavaScript。

2026-10-06 修正了状态心跳反复将选择重置到 train7 的问题。当前 Gazebo
进程实测 train4 → train → train7 均 APPLIED、实际模型及 epoch 同步；
另用原生网页客户端经 WebSocket 实测 train4 → train7，也全部通过。
记录分别为工作区 `logs/model_switch_dropdown_live.json` 和
`logs/model_switch_web_client_live.json`。仅验证切换合同，不构成模型精度验收。

`models/list` 返回字段：

- response：success、message、models、active_model_id、loaded_artifact_path、loaded_artifact_sha256、active_backend、model_epoch。
- 每个 DetectorModelInfo：model_id、display_name、pt_path、pt_sha256、class_names、input_size、engine_prepared、engine_path、engine_sha256、validated。
- `engine_prepared=true` 是注册产物文件/摘要准备状态，不证明它兼容另一台 GPU，也不是精度认证。
- `response.active_model_id` 返回实际活动模型，已覆盖 train/train4/train7 回归。切换是否生效以 APPLIED 与对应版本的 state 为准；列表只是只读快照。

### 10.2 切换流程与安全失败

```text
校验请求/注册 ID → 校验 PT 与 engine 摘要 → GPU 显存预检
      → 后台加载候选 → 两次预热 → 校验实际类别 → 共享推理完整批次边界原子提交
      → 两路 model_epoch 更新 → 清旧图像/ReID/ROI/像素结果 → 重新确认类别
```

准备期间旧模型继续运行；失败保留旧模型，不先卸载旧模型腾显存。模型提交保留几何/公开身份，但清理旧类别与外观证据，新类别暂未知，等待新 epoch 重新确认。

- 当前候选显存预算默认 768 MiB，保留余量 256 MiB；需要同时容纳旧模型、候选和工作空间。
- 候选准备墙钟上限 28 秒，manager 等待上限 30 秒。
- 在线切换只使用**已注册且预构建的 engine**。NOT_PREPARED 不会在线触发长时间转换。
- 双相机共同切换，不支持同一个共享实例中固定相机 train7、云台 train4。
- 提交后前端按 session/epoch 丢弃旧结果；跨 epoch 不沿用旧类别名。

### 10.3 换主机后的 TensorRT 准备

PT 是源权重，生产默认推理的是 FP16 engine。engine 与 GPU 架构、TensorRT/CUDA 版本、输入尺寸和 batch 约束相关；本机生成的文件不能作为所有主机通用产物。Jetson/不同 GPU 应在目标环境准备并回归。

目标机离线转换示例，确保先关闭本项目 GPU 演示，串行构建：

```bash
cd /path/to/deployment_workspace
export SAR_YOLO_PYTHON="$PWD/.venv-sar-gpu/bin/python"
"$SAR_YOLO_PYTHON" src/sar_yolo_detector/scripts/export_xd_vehicle_tensorrt.py \
  --model src/sar_yolo_detector/models/xd_vehicle_train7/xd_vehicle_train7.pt \
  --output src/sar_yolo_detector/models/xd_vehicle_train7/xd_vehicle_train7.engine \
  --imgsz 640 --workspace 0.5 --builder-optimization-level 1
```

脚本可能更新源权重旁的 ONNX/engine，保留原 PT，输出 engine 摘要和 `.engine.json` 清单；在部署副本中构建，避免覆盖正在使用或需要保留的旧产物。train/train4 需要切换时也分别准备。

重新生成后同步**部署注册表与启动参数**中的 engine_sha256、engine_source_model_sha256；PT 未变则不要改 PT 摘要。顶层 launch 默认摘要属于本机现有 engine，新 engine 不能继续冒用旧摘要。

启动自动转换和在线模型切换不同：启动时 `auto_export_engine=true` 可在无匹配 engine 时构建指纹缓存；在线 model/set 不构建。对于生产部署，建议提前离线准备并设 `auto_export_engine:=false`。已存在但与目标平台不兼容的显式 engine 不应仅靠打开自动导出赌它自行恢复，应在目标平台重新构建、校验并设置路径。

`inference_backend:=ultralytics` 可明确用现有 PT 作 GPU PyTorch A/B 回退；这不等于 CPU 模式。正式顶层没有 use_gpu 参数，CPU 部署需要自己的 detector 编排及频率验收，不能照搬 TensorRT 默认命令。

若首次迁移先验证目标平台的自动缓存，可显式避免加载包内旧平台 engine：

```bash
roslaunch xd_uav_track vision_tracking_stack.launch \
  UAV_NAME:=uav1 enable_perception_modes:=true \
  enable_gimbal_lrf:=false enable_sar_gimbal:=false \
  engine_path:="" auto_export_engine:=true
```

首次转换可能持续数分钟，不能把它放在起飞或网页操作的关键路径。启动缓存不会自动写回模型注册表；后续网页切换仍需要准备并登记目标机 engine。

## 11. 网页端接入和视频帧适配

### 11.1 现有客户端

包内 [web/perception_client.js](../web/perception_client.js) 是无 CDN 依赖的原生 WebSocket 客户端，[web/index.html](../web/index.html) 是视频交互页面，[web/perception_api.json](../web/perception_api.json) 是机器可读接口清单。配套 [perception_video.js](../web/perception_video.js) 处理 PNG/同帧元数据/坐标映射，[perception_ui.js](../web/perception_ui.js) 处理页面交互。只要 bridge 名称空间和 URL 对应即可使用，不依赖固定 `/home/promise` 路径。

以下示例在已加载 perception_client.js 的浏览器模块中执行：

```javascript
const client = new PerceptionClient("ws://127.0.0.1:9090", "uav1", {
  onState: state => console.log("运行状态", state),
  onTargets: snapshot => console.log("逐源快照", snapshot),
  onVideo: frame => console.log("同帧视频", frame.targets.image_source, frame.width, frame.height),
  onVideoLost: (source, reason) => console.warn("视频不可用", source, reason),
  onTrajectory: path => console.log("有界轨迹", path),
  onModels: response => console.log("候选模型", response.models),
  onResult: result => console.log("请求阶段", result),
  onConnection: online => console.log("状态已同步", online),
  onError: error => console.error(error)
});

async function applyAndSync(promise) {
  const result = await promise;
  // 客户端的 REJECTED/FAILED 也会 resolve，必须检查 stage。
  if (result.stage !== "APPLIED") {
    throw new Error(`${result.error_code}: ${result.message || "命令失败"}`);
  }
  const deadline = performance.now() + 2000;
  while (performance.now() < deadline) {
    if (client.state && client.state.session_id !== result.session_id) {
      throw new Error("节点会话已改变，需要重新同步");
    }
    if (client.state && client.state.state_version >= result.state_version) return result;
    await new Promise(resolve => setTimeout(resolve, 20));
  }
  throw new Error("已收到终态，但尚未同步新 state");
}

// 收到新鲜 onState 且 client.online=true 后，由按钮分别触发：
async function enterFusion() { return applyAndSync(client.mode("fusion")); }
async function selectTarget(source, uid) { return applyAndSync(client.select(source, uid)); }
async function switchModel(id) { return applyAndSync(client.model(id)); }

async function selectRegion(snapshot, roi) {
  const reference = {seq: snapshot.frame_seq, stamp: snapshot.header.stamp};
  return applyAndSync(client.region(snapshot.image_source, reference, roi));
}
async function selectPoint(snapshot, point) {
  const reference = {seq: snapshot.frame_seq, stamp: snapshot.header.stamp};
  return applyAndSync(client.region(snapshot.image_source, reference, point, true));
}
async function clearSelection() {
  return applyAndSync(client.command("target/select", {
    selection_type: "clear", track_id: -1, target_uid: "", image_source: "",
    frame_seq: 0, capture_stamp: {secs: 0, nsecs: 0},
    point_norm: [0, 0], roi_norm: [0, 0, 0, 0], tracker_algorithm: "auto"
  }));
}
```

使用视频时先加载 perception_video.js，再加载 perception_client.js。select/region 包装方法请求 auto；需要明确算法时，用 command 传完整 SelectPerceptionTargetCommand 字段。默认 TTL=20 秒、请求等待 35 秒、最多 16 条串行待办，断线约 500 ms 后重连。客户端等待 APPLIED 和对应版本 state 后才完成成功事务；REJECTED/FAILED 仍须检查 stage。

实际页面点击/框选应保存 `PerceptionVideo.identity(frame)`，作为 select 的第三个参数或 region 的第五个参数，防止排队期间模式/模型改变后仍提交旧画面。region 的原帧引用使用 `frame.targets.frame_seq` 与 `frame.targets.header.stamp`，不能使用视频最外层 header.seq。

### 11.2 不使用现有客户端时的 rosbridge JSON

先订阅 state/result，再发布命令。以下是协议结构，尖括号内容和时间必须替换，不是可原样执行的命令：

```json
{
  "op": "publish",
  "topic": "/uav1/perception/mode/set",
  "msg": {
    "request_id": "web-unique-request-001",
    "session_id": "<state.session_id>",
    "expected_state_version": 1,
    "issued_at": {"secs": 0, "nsecs": 0},
    "ttl_sec": 20.0,
    "mode": "fusion"
  }
}
```

expected_state_version 和 issued_at 替换为当前 state 对应值。新客户端先 advertise 正确消息类型，再 publish；服务使用 op=call_service、service=/uav1/perception/state/get、args={}，解析 service_response.values。

**不能向 mode/set 发送 std_msgs/String，也不能仅发 `{mode:"track"}` 而省略事务字段。**

### 11.3 视频与数值必须绑定同一帧

默认 bridge 开放 `perception/video`（PerceptionVideoFrame），消息上限 32 MiB，不开放任意 raw Image。每包包含无损 PNG 与同一帧的 PerceptionTargetArray；以包内 targets 作为视频框和当帧位置表的权威来源。详细字段见 [视频接口说明](perception_web_video_interface.md)。

每个实际显示的视频帧同时提供 image_source、frame_seq、capture_stamp(secs/nsecs)、原图宽高、session/version/epoch 和 frame_revision。后端不缩放原图；PNG 未叠绘区域无损保留。不能仅靠一个无时间信息的 MJPEG 画面，将最新 targets 强行套上去。

视频内嵌 targets 已原子配对，独立 targets 可更新目录但不覆盖视频的当帧框。自建客户端应按 source+frame_seq+stamp 配对；没有对应帧就跳过或丢弃，不能在新图上显示旧框仍标 DET。顶层 header.seq 会被 rospy 改成发布序号；采集帧号从 targets.frame_seq 读取。

原图到 CSS/canvas 的变换需要剔除缩放、黑边、裁剪和双路偏移。无裁剪、等比缩放情况下：

```text
scale = min(viewport_width / W, viewport_height / H)
pad_x = (viewport_width - W * scale) / 2
pad_y = (viewport_height - H * scale) / 2
original_x = (local_display_x - pad_x) / scale
original_y = (local_display_y - pad_y) / scale
point_norm = [original_x / W, original_y / H]
```

local_display 为扣除 DOM 边框/位置后的画面内坐标。canvas backing pixels 与 CSS pixels 不同或 devicePixelRatio≠1 时也要转换。超出原图有效区域的点击拒绝，而不是夹到边缘冒充合法目标。

双路合成图只用于观看更方便；两路帧可能不是同一采集时刻，合成图头不能证明左右相机分别的帧引用。可靠网页框选优先使用独立原图/元数据，而不是只使用合成图 header。

### 11.4 重连、陈旧数据与安全

- 重连先恢复订阅，再取 state/get 和 models/list；旧请求在同会话内原样重发，不自动更新 session、版本或帧引用。
- session 改变则清空旧 UID 和选择；客户端旧请求结束为 STALE_SESSION，让操作者重新决定。
- 前端按 source 保存最后 capture stamp，拒绝采集时间严格早于当前帧的乱序结果。**同采集时间的新 revision 修正、类别补充和 LOST 清框必须接受**。排序先采集时间、同时间再 revision；overlay 重启后的新采集帧可接受较小 revision。session/epoch/state_version/mode 不符的数据不显示为当前有效帧，未来版本数据只有限缓存等待 state。
- 客户端已加入 state 墙钟心跳超时（默认 3.5 秒）和每源视频超时（默认 2.5 秒）；超时或状态改变清除有效画面/选择权限。client.online 仍不等于 detector 健康，应同时查看 model_ready 和诊断。
- `disconnect()` 会自动重连；`close()` 才永久停止并结束 pending 请求。
- 幂等终态重放受最近 128 条内存缓存限制，缓存命中在 TTL/版本检查之前返回；未完成请求仍受 TTL 检查，节点重启后缓存消失。不要把客户端超时当成服务端一定没执行。

默认桥监听 127.0.0.1:9090。跨主机最简单的受控方式是 SSH 隧道：

```bash
ssh -N -L 9090:127.0.0.1:9090 -L 8080:127.0.0.1:8080 user@uav-host
```

本地访问 localhost 的 HTTP/WS；部署 Web 服务时用 HTTPS+WSS 和鉴权代理。桥的 topic/service 白名单不是用户认证，不要把 address=0.0.0.0 的裸 ws 放到公网。默认只开放上述 8 话题、2 服务，无飞控话题、内部模型命令或参数读写。数值-only 部署设 bridge 的 enable_video=false，可恢复 7 话题白名单。

## 12. 跨主机部署：依赖、安装空间与模型适配

### 12.1 不要只复制两个 Python 脚本

纯视觉运行至少需要 `xd_uav_track`、`sar_yolo_detector` 及 ROS/cv_bridge/消息依赖；推荐同时安装 `xd_uav_perception_bringup`，由它声明完整组合依赖。该组合包保留 `xd_uav_detect` 依赖用于可选定位，但纯感知入口不运行定位器。网页另需 rosbridge_server；三维定位和飞行功能另需设备、状态估计和控制系统。tracker 还保留兼容仿真/飞行入口的运行依赖，因此完整包的依赖安装可能包含其他 XD 系统包，不能仅复制两个源码目录就忽略 manifest。

当前 tracker package.xml 对跨包运行依赖并未完全覆盖：顶层 launch 使用 sar_yolo_detector、xd_uav_detect，但不能假定仅安装 xd_uav_track 的声明依赖就能带齐它们。部署清单应显式包含这些包，并在干净安装环境核验。

ROS Noetic 风格的消息/构建环境是当前基线。以下命令在目标工作区中执行，依赖齐备后编译，默认限制并发：

```bash
cd /path/to/deployment_workspace
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build xd_uav_track xd_uav_detect sar_yolo_detector -j 2 -p 1
source devel/setup.bash
```

rosdep 不会替你安装完整 CUDA、TensorRT、PT 推理环境或实际云台驱动；某些 XD 系统包不是公开 rosdep 项，必须由同系统源码提供。不要从旧工作区偷用生成消息绕过缺失依赖。

### 12.2 GPU Python 与 SOT Python 是不同运行环境

共享检测包装器按以下顺序选 Python：

1. SAR_YOLO_PYTHON。
2. 兼容环境变量 SAR_YOLO_SMART_PYTHON。
3. 可发现的工作区 `.venv-sar-gpu/bin/python`。
4. python3。

在安装空间自动推断工作区路径不可靠，**显式设 SAR_YOLO_PYTHON 最稳妥**。工作区/安装前缀移动后不要保留原主机路径。

不要直接复制本机 `.venv-sar-gpu` 目录到另一台主机：当前环境的解释器还依赖 `/home/promise/miniconda3/envs/pixeagle`。目标机应重新建立环境，或使用完整且经 ABI/ROS 验证的容器部署；不能只移动 venv 目录。

detector 环境需 torch、ultralytics、NumPy/OpenCV、ROS 消息/rospy/cv_bridge 可导入，以及与主机匹配的 TensorRT bindings 与 libnvinfer。requirements-smart-tracker.txt 覆盖旧兼容功能，包含行人训练等非车辆必需依赖；不要盲目把其中所有依赖都当最低部署要求。

requirements-tensorrt-cu12.txt 针对本机 TensorRT 10.1/CUDA 12 系绑定，仅安装 bindings，**不是 TensorRT 系统运行库安装器**。另一台机器的版本不同时需要对应环境的依赖方案，不能直接覆盖厂商 TensorRT 或 Jetson PyTorch。

perception_mode_manager/SOT 使用其 catkin Python 环境，不一定是 SAR 虚拟环境。CSRT/KCF 是否可用取决于该解释器的 OpenCV contrib；只向 SAR 环境装 contrib，不会自动改变 manager 的 OpenCV。检查解释器/Python ABI，避免 cv_bridge 和 NumPy/OpenCV 二进制不兼容。

detector 环境的只读检查示例：

```bash
"$SAR_YOLO_PYTHON" -c 'import sys, torch, cv2, rospy, cv_bridge; from xd_uav_track.msg import PerceptionState; from sar_yolo_detector.srv import ListDetectorModels; from sar_yolo_detector.pixeagle.backends.tensorrt_compat import import_tensorrt; print(sys.executable, torch.__version__, torch.cuda.is_available(), cv2.__version__, import_tensorrt().__version__)'
```

另外用实际运行 manager 的 Python 检查 `cv2.TrackerCSRT_create` 或 `cv2.legacy.TrackerCSRT_create` 是否存在。每台目标机锁定已验收的依赖版本，保留解释器路径、pip freeze、驱动和 GPU/TensorRT 版本记录，不要在飞行链运行期间自动升级。

### 12.3 安装空间核验

源码/devel 可运行不代表独立 install 空间完整。需要单独的部署构建，例如：

```bash
catkin config --install
catkin build xd_uav_track xd_uav_detect sar_yolo_detector -j 2 -p 1
```

此命令会改变该工作区构建配置，建议在专用部署副本执行。随后在新终端只 source ROS 和 install/setup.bash：

```bash
source /opt/ros/noetic/setup.bash
source /path/to/deployment_workspace/install/setup.bash
rospack find xd_uav_track
rospack find sar_yolo_detector
rospack find xd_uav_detect
rosmsg show xd_uav_track/PerceptionTargetArray
rossrv show sar_yolo_detector/ListDetectorModels
```

确认 scripts、Python 模块、msg/srv、config、模型、launch、web 和 docs 均能从安装前缀解析。当前有相应安装规则，但本指南不声称已对你的另一台主机完成 install 实际验收。

2026-10-04 本机已重新安装并从 install 启动实际 Gazebo/共享 TensorRT/网页链路。
独立 ReID 节点与 Bash 包装器已加入安装规则；engine 验证器通过 rospkg 定位注册表。
跨包部署依赖由 `xd_uav_perception_bringup` 声明，避免 detector/localizer 依赖 tracker
消息后，再反向添加依赖而形成 catkin 环。另一台主机仍需准备兼容的 Python/CUDA/
TensorRT 环境与相机标定，不能直接把本机 engine 当成跨 GPU 通用产物。

DetectionArray、TrackState、TrackStateArray、人工身份服务等嵌套消息已有变更。发布、订阅、rosbridge 和其他 XD 包必须使用同一套生成类型并重启；遇到 MD5 不一致不能靠只拷脚本修复。旧 rosbag 需兼容转换或隔离旧版本回放。

### 12.4 多机与分布式网络

- 每机 UAV_NAME 唯一，图像源、sensor_id、body/world/camera frame 也要按设备独立；不同 UAV 的 ID 不应仅按整数比较。
- ROS 网络部署需各端互通 ROS_MASTER_URI、ROS_IP/ROS_HOSTNAME 和 TCPROS 动态端口；不是只开放 11311 就能远程收全部数据。
- 浏览器通过 rosbridge，不需要直接接 ROS TCPROS；视频通道和控制桥应分别限带宽和权限。
- 多机在一个 GPU 主机同时跑，会每机加载一个共享 YOLO；当前共享是每机两相机内共享，不是所有 UAV 全系统一个模型调度器。
- 每机 bridge 使用自己的端口/名称空间；不要重复启动相同节点名。硬件相机 topic 为绝对路径时，确认不会误把两机都接到同一设备。
- /tf、/tf_static、/diagnostics 是全局话题，应按 frame 或 diagnostic status.name 区分；frame 名本身不会自动因 ROS 命名空间而变化。

### 12.5 独立感知部署：不绑定运动学或飞行方案

现场模板：[perception_deployment.yaml](../config/perception_deployment.yaml)。
复制到现场配置目录再修改；升级代码不覆盖现场文件。`$(find ...)` 和
`$(arg UAV_NAME)` 由 launch 解析，在源码和 install 空间都适用，无本机
绝对路径。设备驱动和任务规划由外部 bringup 启动，每机只包含一次：

```bash
source /opt/ros/noetic/setup.bash
source /path/to/workspace/install/setup.bash
export SAR_YOLO_PYTHON=/path/to/verified_gpu_venv/bin/python
rosrun xd_uav_track validate_tracking_config.py \
  --perception-config /absolute/path/to/perception_deployment.yaml
roslaunch --skip-log-check xd_uav_perception_bringup perception.launch \
  UAV_NAME:=drone_a deployment_config:=/absolute/path/to/perception_deployment.yaml \
  enable_web:=true address:=127.0.0.1 http_port:=8080 websocket_port:=9090
```

网页 `http://localhost:8080/`；不需要网页时保持 `enable_web=false`。
跨主机使用 SSH 隧道或鉴权 HTTPS/WSS 代理，不开放公网裸 ROS/WS。
不能保证任意主机无需准备 CUDA/Python/驱动即可运行，环境检查见 12.2；
**感知部署边界完成不等于实景准确率、跨源身份与时延都已认证**。

| 配置块 | 适配内容 |
|---|---|
| `sar_yolo_multi_source_detection` | 模型/注册表、GPU 后端、各路原图/topic/source/sensor、帧率、ReID、ROI |
| `track` | 观测输入、允许来源、存储上限/TTL、FRD frame；保持 perception_only=true |
| `perception_mode_manager` | 初始模式、对应原图/来源、选择缓存、预测期限、轨迹缓存 |
| `fixed_perception_overlay` / `gimbal_perception_overlay` | 同采集帧的原图、原分辨率叠框、编码频率 |
| `perception` | 高频像素实验开关，模板保留已发布默认 false；改变后需同场景验收 |

默认两路订阅，未上线的相机不推理，不加载第二份 YOLO。仅一台相机时，
删除 detector/manager 第二项，同步修改 `track.tracker.sources.allowed`
和 maximum_count，删除第二个 overlay 块，传 `enable_gimbal_overlay:=false`。
可以用下视或前视相机；名称不限定安装朝向，但不同相机不能共用 source。
启动前校验重复键、模式名、来源/原图一致性、FRD 标记和缓存边界；
配置不合格时 required 节点退出，终止本次组合启动。

存储限制：最多配置 8 路，模板限制 2 路、闲置 TTL 30 s；沿用每源 128
条轨迹、512 全局实体、32 人工身份。表满时只回收无活动映射的非锁定
档案；全为活动身份时拒绝新 ID 分配，不冒用旧 ID。断流后定时清理来源、
时间戳、映射和历史，无需等下一张图像。清理扫描最多 2 Hz，TTL 清理延迟
还包含最多 0.5 s 扫描间隔及定时器调度时间。身份档案沿用现有 TTL，不是永久
跨进程身份库。选中目标轨迹默认 60 s/600 点，包含预测点和断测分段。

纯感知默认**仅像素定位**，两路直接发布 `detect/detections`，按 source
隔离轨迹。三维 `position_valid=false`，不虚构距离。需要可靠世界定位时：

1. 将每路 detector 的 `xd_detections_topic` 改成独立 2D 话题。
2. 在外部标定/定位 bringup 中分源定位，再转发到 tracker 的观测输入。
   保留原图 stamp、model_epoch、embedding 和来源元数据。
3. 提供采集时刻的位姿/TF，位置数值和 frame 保持声明的 FRD 合同。
4. 未校验外参、落点和时序时保持位置无效，不仅凭相似外观合并跨相机 ID。

空 engine_path 默认走本机指纹缓存，自动准备 train7 FP16。首次转换
可能耗时，应在任务前完成，等 `perception/state.model_ready=true`。
网页热切换仍只接受预先准备的注册 engine；新主机要离线准备三个模型并
更新现场注册表的路径、engine SHA-256 和源 PT SHA-256，不能沿用本机
旧 engine 哈希。PT/CUDA 可作为明确配置的部署后端，但不承诺同等吞吐。

公共合同不变，详见第 5～11 节：

- `perception/targets`：类别、稳定 ID/UID、原图左上角原点的中心像素、
  来源、实测/辅助/预测/失联、age、三维有效性。
- `perception/state`：模式、选中身份、视觉 TRACKING/LOST、模型摘要、
  epoch 和事务状态；LOST 不自动换另一辆车，也不是悬停命令。
- `perception/trajectory`：**选中目标**的有界轨迹，不是所有目标的永久
  世界轨迹数据库。任务系统自行保存跨任务/跨进程档案。
- `perception/video`：原分辨率、同帧目标快照，框上保留类别和 ID。
- mode/set、target/select、model/set：request/session/version 事务；
  点击和 ROI 必须绑定原画面 source/stamp。

新增只读 `/<uav>/track/runtime_diagnostics`，类型
`diagnostic_msgs/DiagnosticArray`，字段 sources、maximum_sources、
global_entities、maximum_global_entities、local_mappings、visual_identities、
pending_stitches。默认 2 Hz，维护端可检查断流回收；不开放给默认网页桥。
`track/status=perception_only` 只表示禁用制导，不替代视觉锁定状态。
节点禁止创建 flight command/velocity/reference 发布器，即使旧配置或
启动服务请求要求开启也无效，详见 13.4。外部飞行策略自行消费感知结果。

## 13. 配置适配清单

### 13.1 正式入口的主要 launch 参数

| 参数 | 当前默认值 | 部署作用 |
|---|---|---|
| UAV_NAME | 环境 UAV_NAME 或 uav1 | 全部相对 topic 的前缀 |
| vehicle_type | multirotor | 选择 tracker 启动分支；固定翼需配套 config |
| enable_tracking | true | 身份层/定位转发/三模式是否启动；不是 runtime mode |
| enable_perception_modes | false | 严格验收前保留发布门禁；联调显式 true |
| enable_high_frequency_pixels | false | 独立 latest-only 像素工作线程实验开关，需同时启用 tracking 与三模式；不是网页业务模式 |
| initial_perception_mode | detect | detect / track / fusion 启动模式 |
| enable_fixed_camera / enable_sar_fixed | true / true | 固定路定位转发/检测输入是否启用 |
| enable_gimbal_lrf / enable_sar_gimbal | true / true | 云台路是否启用；前者名字不证明 LRF 已接线 |
| fixed_image_topic / fixed_camera_info_topic | fixed_camera/image_raw、fixed_camera/camera_info | 图像与内参订阅 |
| gimbal_image_topic / gimbal_camera_info_topic | gimbal_camera/image_raw、gimbal_camera/camera_info | 云台原图与内参 |
| fixed_image_source / gimbal_image_source | fixed_rgb / gimbal_rgb | 业务来源标识，贯穿 detector、tracker、网页 |
| fixed_sensor_id / gimbal_sensor_id | fixed_camera / gimbal_camera | 设备元数据 |
| body_frame / body_frd_frame / world_frame | UAV/base_link、UAV/base_link_frd、UAV/local_origin | 实际 TF 名称；参数不创建 TF |
| fixed_detect_config / gimbal_detect_config | 各 detector YAML | 定位或纯视觉转发策略 |
| track_config | config/track.yaml | tracker、身份和 follower 配置 |
| detector_config | sar config/xd_vehicle_detection.yaml | YOLO 阈值、类别与 NMS |
| model_path / model_sha256 | train7 PT / 配套摘要 | 源权重身份 |
| inference_backend | tensorrt_fp16 | 推理后端；ultralytics 可做 GPU PT 对照 |
| engine_path / engine_sha256 / engine_source_model_sha256 | train7 engine / 两种配套摘要 | 平台产物及源权重一致性 |
| auto_export_engine | true | 仅启动时允许准备缓存，生产建议提前离线 false |
| detector_fps | 4.0 | 正常各源推理调度上限 |
| enable_reid | true | 可选外观编码；并不绕过运动/世界位置门控 |
| gimbal_roi_recovery_enabled | false | 云台 ROI 默认仍关闭，先验收旋转场景再启用 |
| enable_tracking_visualization | true | 叠框和合成图像；纯数值部署可 false |
| tracking_visualization_rate_hz | 3.0 | 合成图像发布上限，不改变算法频率 |
| enable_web_video | true | 新模式同帧 PNG 视频；无订阅者不编码 |
| web_video_rate_hz | 6.0 | 每源编码/发布上限，不降低算法频率 |
| web_video_max_pixels / web_video_max_encoded_bytes | 16777216 / 16777216 | 原图像素/PNG 字节上限，超限丢帧而不缩图 |
| perception_frame_cache_size | 15 | 原帧 ROI/点击缓存；最大 60，增大需评估内存 |

顶层目前没有转发 model_registry、reid_config、engine_cache_dir、use_gpu、initial_mode 等所有下层参数。不能把 YAML 私有参数当 launch 参数任意传递。

少量设备适配用上述参数即可；需要不同模型类别/自定义注册表/专属 Python 或高级参数时，在自己的部署包写一个 bringup，明确配置下层节点和命名空间，而不是复制并改乱算法代码。相机驱动和识别栈由同一 bringup 包含一次，不能让两路驱动各自启动整栈。

### 13.2 部署 bringup 的最小结构

```xml
<launch>
  <arg name="UAV_NAME" default="drone_a"/>
  <!-- 这里包含真实相机驱动；发布对应 image_raw/camera_info，不包含仿真设备。 -->
  <include file="$(find xd_uav_track)/launch/vision_tracking_stack.launch">
    <arg name="UAV_NAME" value="$(arg UAV_NAME)"/>
    <arg name="enable_perception_modes" value="true"/>
    <arg name="enable_gimbal_lrf" value="false"/>
    <arg name="enable_sar_gimbal" value="false"/>
    <arg name="fixed_image_topic" value="/camera_front/image_raw"/>
    <arg name="fixed_camera_info_topic" value="/camera_front/camera_info"/>
    <arg name="fixed_image_source" value="front_rgb"/>
    <arg name="fixed_sensor_id" value="front_camera"/>
  </include>
  <!-- 可选：包含 perception_web_app.launch，提供 HTTP/WS，不重复启动算法。 -->
</launch>
```

需要双路时按第 4.3 节配置云台纯视觉文件，或先补齐并验收外部包真正的 LRF 节点。现场机型配置独立于软件包版本保存，避免更新代码覆盖标定。

### 13.3 重要私有参数

manager 下：frame_cache_size=15、source_timeout_sec=1.0、prediction_horizon_sec=0.5、position_timeout_sec=1.0、visual_identity_ttl_sec=60、command_queue_size=16，以及第 9 节 trajectory/*。

高频实验线程的 manager 私有参数：pixel_max_tracks=8、pixel_anchor_timeout=2.2
秒。它们未作为顶层同名 launch 参数公开；需要修改时在自己的编排里
设置。轨迹上限与 YOLO 锚点超时是算力/漂移约束，不是长遮挡永不丢失承诺。

这些参数多数在初始化读取；运行后简单 rosparam set 不代表立即热更新。需要在启动前加载或自己的节点编排中设置；不要给未声明的顶层 launch 参数赋值。

detector 下：track_anchor_fps=1.0、track_standby_fps=0.5，model_switch/reserve_mib=256、candidate_mib=768，maximum_batch_size=1、batch_wait_ms=4、startup_warmup_frames=1。当前 engine 静态 batch=1；不能只把 maximum_batch_size 改 2 就宣称双帧 batch 兼容。

生产 detector 输出阈值 0.10，tracker 新轨迹阈值 0.50，低置信候选只维护已有轨迹。不要为了“干净显示”先把 detector 阈值提高到 0.60，否则低置信第二阶段会失去输入；显示应依 confirmed/证据状态过滤。

### 13.4 感知内核与运动/任务策略解耦

**部署要求：感知模式不决定无人机怎么飞。** `perception_mode_manager.py`
不生成巡航高度、航点、悬停点或飞行 setpoint。测试中的升高总览、等待
全部目标可见、局部重搜索、模拟云台转动来自
`moving_uav_search_driver.py`；它通过 Gazebo 模型状态服务移动模型，不是
真实飞控、固定翼动力学或大范围侦察规划器。迁移主机应优先使用第 12.5
节纯感知入口，不在设备 bringup 中包含该演示 driver 或仿真 launch。

| 层 | 负责的内容 | 不应做的内容 |
|---|---|---|
| 感知内核（本包三模式） | 检测、局部/多目标跟踪、身份、观测有效性、同帧视频与轨迹 | 为获得全部目标视野而强制升高/悬停、直接起飞或改变航线 |
| 外部任务/搜索层 | 区域分区、航线覆盖、目标优先级、目标记忆、失锁重搜索范围 | 把旧框或过期定位解释成当前实测 |
| 外部机型运动/控制层 | 四旋翼/固定翼约束、速度/高度/坡度、避障、禁飞区和执行控制 | 因机型变化而改写感知 ID 或 detect/track/fusion 语义 |
| 云台设备适配层 | 实际角度、时间戳、控制权、转速限制与设备命令 | 把网页视觉选择直接视为云台或飞行控制授权 |

三模式在不同飞行方案下保持同一合同：detect 发现当前视场内的目标；
track 处理指定 UID/ROI；fusion 继续发现目标并对选中实体加强跟踪。
它们不要求所有目标同时处于一张画面，也不承诺知道尚未搜索区域内的
目标数量。大地图应采用航线/分区覆盖，不能无限抬高到“全地图入镜”；
覆盖率、目标短边像素、机型可达性与风险约束由外部策略权衡。

外部任务模块接入现有公共接口即可，不需新增感知模式：

1. 订阅 `perception/state`、`perception/targets`，按 session/UID 和来源
   接收新鲜观测。通过 `perception/target/select` 指定身份，再按实际
   state 确认选择；使用 `perception/mode/set` 改感知策略。
2. 自己执行搜索、接近、绕飞、持续巡航或跟随；同一 fusion 模式下可以
   任意切换外部航线，不需要反复重建感知模式或目标身份。
3. 目标失联时保留原任务身份，结合最后可信惯性位置和协方差决定局部
   重搜索/扩大搜索/任务退出。LOST 是观测状态，不是“在原地悬停”命令；
   未经确认不能自动改选附近另一辆车。
4. 度量位置只在 `position_valid=true` 且 stamp/frame 新鲜时使用，并用
   **position_stamp 时刻**的机体到世界 TF 转到共同惯性坐标系。图像中心
   点、FRD 相对位置和像素轨迹不能直接当导航世界航点。没有可靠三维
   定位时，保持像素锁定/视线约束与搜索任务分离，不虚构目标距离。
5. 大地图历史目标目录、跨任务/跨节点重启的持久身份、覆盖地图和任务
   状态机需要外部任务系统实现；当前有界视觉身份/轨迹缓存不是永久
   GIS 目标数据库，不应直接延长所有视觉 TTL 代替任务记忆。

使用外部飞行方案时由纯感知入口强制设置 `runtime/perception_only=true`。
节点在启动时锁存此边界，即使旧配置写 `enabled_at_startup=true` 或
`publish_control_reference=true`，也不创建 `track/velocity_body`、
`track/command`、`control/reference/setpoint` 发布器，不运行制导定时器计算。
旧服务 `start_tracker(start=true)`、`set_profile`、
`select_track(start_tracking=true)` 返回失败；检测消息中的 `start_track`
不能越过边界。`manage_visual_identity` 和三模式视觉选择仍正常工作。
边界不能通过运行时 rosparam 修改解除；需要明确换入口并重启节点。

```bash
roslaunch xd_uav_perception_bringup perception.launch \
  UAV_NAME:=drone_a deployment_config:=/absolute/path/to/perception_deployment.yaml
```

兼容服务名称保留是为了返回明确拒绝，不表示开放飞控。网页默认桥也
未开放它们。外部规划器只消费 perception 结果，自己负责速度、高度、
航线、失联后的重搜索和安全退出。更换机型仍需匹配标定、位姿输入、
TF、观测时延和运动相机跟踪验收，不能把仿真高度参数照搬实机。

本次没有新增通用航线规划器、修改演示轨迹，也没有替换已有检测模型、
关联评分、SOT 算法或网页消息接口；来源/身份达到存储上限时安全拒绝新分配。

## 14. 内部话题、设备输入与控制边界

以下用于机器人集成和诊断，不建议网页直接依赖其内部实现。

| 相对 UAV 的名称 | 类型 | 用途 |
|---|---|---|
| fixed_camera/image_raw、gimbal_camera/image_raw | sensor_msgs/Image | 原始图像；非零、递增采集时间，正确宽高/encoding |
| fixed_camera/camera_info、gimbal_camera/camera_info | sensor_msgs/CameraInfo | 当前分辨率匹配的内参；zoom 改变时需与实际成像匹配 |
| detect/input/fixed/detections_2d、detect/input/gimbal/detections_2d | xd_uav_track/DetectionArray | 共享检测的分源候选，不是网页最终稳定目标 |
| detect/detections | xd_uav_track/DetectionArray | 默认 C++ tracker 观测输入，定位转发结果 |
| track/tracks_by_source | xd_uav_track/TrackStateArray | 各来源全局身份快照、共享 detector 的轨迹反馈 |
| track/tracks | xd_uav_track/TrackStateArray | 兼容轨迹输出，不能代替完整逐源网页快照 |
| track/target_pixels_by_source | xd_uav_track/TargetPixelArray | 旧中心点接口，新网页优先使用 perception/targets |
| track/visual_tracks_by_source | xd_uav_track/TrackStateArray | 人工身份视觉证据反馈 |
| track/internal/observations_by_source | xd_uav_track/DetectionArray | C++ 关联后的原始测量锚点，供同采集帧像素辅助校验；不是网页目标快照 |
| track/gimbal_state | xd_uav_track/GimbalState | 有时间戳的 yaw_rad/pitch_rad/roll_rad、valid |
| track/gimbal_status | xd_uav_track/GimbalStatus | 健康、锁定、控制权、zoom/FOV、测距状态补充合同 |
| sar_yolo_multi_source_detection/model_status | diagnostic_msgs/DiagnosticArray | 实际加载模型/产物/后端/设备/预热状态，锁存 |
| fixed_camera/tracking_overlay、gimbal_camera/tracking_overlay | sensor_msgs/Image | 每路同帧叠框，不新增识别 |
| tracking_visualization | sensor_msgs/Image | 单路或双路合成显示，默认 3 Hz |
| perception/internal/model/set | xd_uav_track/SetDetectorModelCommand | manager 到 detector 内部请求，不由网页直接发布 |
| perception/internal/model/result | xd_uav_track/DetectorModelCommit | 内部提交/失败结果 |
| perception/internal/pixel_observations | xd_uav_track/PerceptionTargetArray | 独立像素线程的当前帧视觉观测，manager → detector；不能标为新 YOLO 实测 |
| perception/internal/roi_request | std_msgs/String | manager → detector 的有界局部重检测请求，内部 JSON，不是网页框选接口 |
| perception/pixel_diagnostics | diagnostic_msgs/DiagnosticArray | 高频像素线程计数与图像回调到 targets 耗时；纯诊断，不作设备健康替代 |
| track/status | xd_uav_track/TrackStatus | 旧 follower/制导诊断，不等于新 PerceptionState |

云台角度约定：弧度，yaw 向右正、pitch 向下正、roll 顺时针正。不得直接把厂商度数或不一致 Euler 轴约定填入。GimbalStatus 字段包括 header、valid、healthy、tracking_active、range_valid、control_authority_available、zoom_ratio、horizontal_fov_rad、vertical_fov_rad、range_quality；状态与角度是独立时间戳，不能用一条新角度冒充健康、激光和控制权都新鲜。

当前纯视觉云台配置不读取真实激光；不要向一个名为 gimbal/range 的空 topic 发任意 Float32 就认为 LRF 生效。设备激光类型、质量、时间和投影合同需要外部定位包真正接线后明确。

旧 follower 服务仍保留 `/UAV/track/start_tracker`、set_profile、select_track、set_target_mode、emergency_stop。其输出 track/command、track/velocity_body、control/reference/setpoint 与新感知模式分离。速度可能为 FLU/ENU，而 target position 是 FRD；控制集成必须明确转换。本指南不下发这些飞行命令，也不为固定翼底层控制节点做操作说明。

兼容 track.yaml 中 `tracker/enabled_at_startup=false`，但 `runtime/publish_control_reference=true`。纯视觉应使用第 12.5 节入口，节点内部禁止飞行输出；仅在兼容入口修改两项 bool 不能阻止其他客户端调用旧 follower 服务。

### 14.1 tracker 兼容服务与机器人集成输出

下表以 tracker 节点名 `track`、UAV_NAME=uav1 说明私有服务的完整名字。
改节点名会改变私有服务路径；第 5 节公共 perception API 不依赖这个
节点名。机器人控制服务不在网页 rosbridge 默认允许列表中。

| 默认完整服务 | 服务类型 | 用途与边界 |
|---|---|---|
| `/uav1/track/manage_visual_identity` | `xd_uav_track/ManageVisualIdentity` | 内部 register/attach/detach/retire/update/bind/reset_bind；人工 ROI 及已有目标局部跟踪接入全局实体；不启用飞行，不供网页直接调用 |
| `/uav1/track/select_track` | `xd_uav_track/SelectTrack` | 旧轨迹选择合同；新网页使用 perception/target/select 的原帧/UID合同 |
| `/uav1/track/set_target_mode` | `xd_uav_track/SetTargetMode` | 旧 all/follow 选择策略，不是 detect/track/fusion |
| `/uav1/track/set_profile` | `xd_uav_track/SetProfile` | follower 配置切换，不是 YOLO 模型切换 |
| `/uav1/track/start_tracker` | `xd_uav_track/StartTracker` | 启停 follower，不能作为纯视觉模式按钮 |
| `/uav1/track/emergency_stop` | `std_srvs/SetBool` | 控制安全状态，不是取消视觉选中 |

| 默认相对 UAV 话题 | 类型 | 方向/说明 |
|---|---|---|
| `state_estimator/main/odom` | `nav_msgs/Odometry` | tracker 输入，采集时刻机体位姿/速度；不是目标真值 |
| `track/metric_target` | `xd_uav_track/MetricTarget` | 可选 tracker 度量观测输入，需真实 frame/stamp/covariance |
| `track/target_mode` | `std_msgs/String` | 旧 all/follow 状态输出，锁存；不与 perception/state.mode 混用 |
| `track/command` | `xd_uav_track/FollowerCommand` | follower 输出；视觉选择不等于飞行授权 |
| `track/velocity_body` | `geometry_msgs/TwistStamped` | 兼容控制输出，必须按 header/frame 解读，不用 FRD 目标位置约定代替 |
| `control/reference/setpoint` | `mavros_msgs/PositionTarget` | 仅启用 publish_control_reference 时创建，不属于视频结果 |
| `/tf`、`/tf_static` | `tf2_msgs/TFMessage` | 外部提供标定/位姿链；全局话题，按 frame 区分 UAV |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 共享检测器运行诊断，全局话题，按 status.name 区分实例 |

tracker 的 `interfaces/input/*`、`interfaces/output/*` 可覆盖这些相对名称；
直接 rosrun 的 detections 默认是 `track/detections`，正式编排接的是
`detect/detections`，不要把这两种默认混用。视频和 targets 都没有固定
TCP“监听端口”；ROS 1 使用 master/TCPROS，网页 HTTP/WS 才使用 8080/9090。

共享检测器每个 `sources` 项还可配置 `detections_topic` 并开启
`publish_vision_detections`，输出 `vision_msgs/Detection2DArray`。这是可选
兼容输出，不提供下游全局身份，不代替 `perception/targets`。生产 XD
分源输出由 `xd_detections_topic` 决定；来源 `name/image_source/sensor_id`
和 topic/frame 是不同标识，不能互相直接比较。

### 14.2 SAR 非正式主链服务定义

`sar_yolo_detector/srv` 还保留如下独立功能合同，**共享 YOLO 三模式入口
不默认启动它们**。仅在启用相应旧节点/独立任务系统时才有实际服务名，
部署方应通过 rosservice list/type 核对，不把推测路径接到网页：

| 功能 | 包内服务定义 |
|---|---|
| 旧 SmartTracker | `SelectSmartTrack`、`SwitchSmartTrackerModel`、`SwitchTrackingMode` |
| 区域/任务查询 | `GetFloodRegions`、`GetTaskCandidates`、`GetTaskStatuses`、`GetPerceptionSnapshot` |
| 任务提交 | `SubmitTaskAssignment`、`SubmitAreaTaskAssignment` |
| 证据裁剪 | `GetEvidenceCrop` |

当前模型列表主链使用 `ListDetectorModels`，切换使用 tracker 消息
`SetDetectorModelCommand`，**不是**旧 SwitchSmartTrackerModel 服务。
默认网页桥仅允许第 5 节 perception 话题与 models/list、state/get 两个
服务，不开放参数服务器、上述任务服务、内部绑定或飞行命令。

## 15. 性能诊断与低负载回归

### 15.1 就绪与负载分开看

```bash
rostopic echo -n 1 /uav1/sar_yolo_multi_source_detection/model_status
rostopic echo -n 1 /diagnostics
rostopic hz /uav1/fixed_camera/image_raw
rostopic hz /uav1/perception/targets
rostopic hz /uav1/fixed_camera/tracking_overlay
```

hz 是临时诊断订阅，不要长期留下多个终端重复接收大图。targets 的 hz 是多源混合发布频率，不是每个来源单独检测 FPS。

model_status 的 KeyValue 包含 state(READY/WARMING_UP/ERROR/UNAVAILABLE)、startup_ready/error、active_model_id、source_model_sha256、loaded_artifact_sha256、backend、effective_device、model_epoch 和 class_names(JSON)。DiagnosticArray 中的字符串布尔值为 true/false，需解析；不要根据文件扩展名或 launch 预期值判定运行后端。

`/diagnostics` 中 status.name 以 detector 节点名区分，关注 received_frames/processed_frames、pending_images、dropped_image_backlog、rate_limited_frames、inference_ms_ewma、pending_reid、reid_stale、reid_ms_ewma、ROI/像素辅助计数和 rss_peak_mib。latest-only 的丢帧是限制时延的策略；持续大量 backlog 丢帧说明吞吐不足，不等于所有漏框都来自 YOLO 本身。

2026-10-07 新增两项毫秒 EWMA：`gpu_wait_ms_ewma` 为完整画面推理等待
GPU 锁的时间，`batch_total_ms_ewma` 为整批图像解码开始至发布处理结束的
墙钟时间（包含 ROI、CPU 后处理及锁等待）。`inference_ms_ewma` 只统计
主画面 detect/detect_many 调用，不包括等待或额外 ROI 推理。
`roi_ms_ewma` 是 ROI 调用路径耗时，包含它自己的锁等待。不能把 batch
时间和其组成项相加，也不能将这些值直接称为 capture→浏览器端到端延迟。
无订阅者、空输入或推理失败时的旧 EWMA 不代表当前有新鲜输出，须同时
查看计数、state 和图像/目标时间戳。

降低负载优先关闭不需要的合成图像/桌面 GUI，按实际吞吐限制输入和检测频率，监测 age_ms/p95。不要通过去掉身份门控、盲目延长预测或把 stale 画实线来换取“画面上框不断”。

### 15.2 有界测试命令

这些测试应在**源码/devel 测试工作区**执行；当前 test/.test 未完整安装，不能在独立 install 中直接照跑。先关闭同 GPU 上本项目其他演示，在 source 当前构建环境后逐项执行，不要三条同时跑：

```bash
timeout --signal=INT --kill-after=10s 120s \
  rostest xd_uav_track perception_visual_identity.test

timeout --signal=INT --kill-after=5s 160s \
  rostest sar_yolo_detector model_switch_gpu.test

timeout --signal=INT --kill-after=10s 120s \
  rostest xd_uav_track perception_web_contract.test
```

第一项是合成图像/模拟检测的真实 ROS 原帧、人工→全局身份、跨源门控、失联/FRD 合同测试；后两项使用真实 GPU，验证双路 engine 切换和实际 rosbridge 重连事务。不是实拍识别召回、长遮挡 HOTA/IDF1 或飞行验收。

已有低负载验收记录见 [阶段 6/7/8 验收](perception_stage_6_7_8_acceptance.md)，包含 72 项 Python、22 项 C++ 回归及 ROS/GPU/WebSocket 测试。数值是已有记录，不代表另一台设备已达到相同延迟。

轻量视频入口 `video_tracking_replay.launch` 可用于旧主干的单路/双路回放；
回放不能只传 enable_perception_modes 给未声明该参数的 launch。如需新模式
视频验收，自己的回放 bringup 应明确组合正式接口和视频发布节点。
`moving_uav_vehicle_search_demo.launch` 当前已经默认启用三模式，
`perception_gazebo_web_test.launch` 还统一启动网页，操作见第 4.6 节；
不能再沿用“该移动演示没有新模式 manager”的旧说明。

### 15.3 停止与记录

正常用启动终端 Ctrl+C 结束本次 launch，或由部署服务管理器按整套进程生命周期停止。不要用全局 pkill python/ros/gazebo 影响其他 XD 功能包。网页 client.close() 只关闭前端，不停止机器人算法；关闭图像窗口也不停止 detector。

持久化记录可按需录制小体积数值话题：

```bash
rosbag record -O perception_numeric.bag \
  /uav1/perception/state /uav1/perception/command_result \
  /uav1/perception/targets /uav1/perception/trajectory
```

该命令不含原图，适合查事务和轨迹，但不足以评估识别召回。真实效果验收需要同步视频、原帧元数据和目标真值 ID；原图录制应另行设磁盘容量、分段和持续时间上限。退出后检查自己的 launch 子进程和 GPU 占用，不终止别人的桌面或系统 GPU 进程。

### 15.4 最新融合连续性与模型切换记录

2026-10-06 同地图复查保留了失败记录；不覆盖失败 JSON。修复后的
120 秒视频记录缺框 0（355 次视频更新），追加身份冲突保持修正后重新
启动的最终版本补测 90 秒：579 次目标更新、268 次视频更新均缺框 0、
同帧降级 0，最大有效观测更新间隔 0.50 秒，无人工重新选择。
88 项相关 Python 回归通过。完整条件与源码摘要见
[融合恢复记录](fusion_selected_lock_recovery.md)。

**旧首次快照统计不能单独证明不闪框。** 同一采集帧可能先有有效框，
后续修订变 LOST；本轮探针 schema_version=2 会分别统计
target_revisions / video_revisions 的每次更新、同帧降级和最终缺失。
视频编码/网络可能跳过更新，所以目标快照全通过也不等同实际浏览器
逐帧通过。目标真值 IoU、HOTA/IDF1 和真正身份误关联率还需带标注数据。

已有演示运行时只执行探针，**不要重起 Gazebo**：

```bash
cd /home/promise/mrs_test
source /opt/ros/noetic/setup.bash
source devel/setup.bash
timeout --signal=INT --kill-after=5s 150s \
  python3 src/xd_uav_track/test/measure_fusion_lock.py \
  --seconds 90 --output logs/fusion_selected_readonly.json
```

先在网页选中 fusion 目标；上述命令只读观察当前选择。测试时不要手动
换目标。需要自动选固定路第一个确认目标时加 `--select-first`（会改变
模式/选择）；可再加 `--track-id N`，N 不存在会有界失败，不自动改选。
测试文件路径属于源码工作区，不能假定独立安装空间包含它。

#### Fusion 云台局部框与新检测 ID 冲突（2026-10-07）

**宽框选已有车辆**：网页拖出的矩形是选择区域，不要求恰好等于车身框。
原帧中恰有一个明确识别目标被覆盖时，即使背景留白导致 IoU < 0.25，
也使用该目标已有 ID 和紧致目标框启动局部跟踪，不另建人工身份。
要求已有 confirmed 身份、有类别证据及当前实测/视觉观测、目标中心位于选择区内，
且覆盖目标面积至少 80%、目标面积至少占选择区 5%。若有多个不明确
身份，返回 `AMBIGUOUS_TARGET`；请缩小框选或按 ID 选择。不因框相近
强行合并。没有明确已识别目标的 ROI 仍支持任意人工目标初始化。
贴边 ROI 只容忍 float32 的微小舍入（1e-6），不放宽真实越界或原帧校验。

改选其他目标/新 ROI 时，原先**尚未确认类别的人工 ROI**停止后台局部
跟踪，通过内部 `retire` 标记失联并保留有界身份档案，不继续把背景矩形
作为蓝色实测框输出；已确认车辆的全局跟踪不变。重新进入 detect 时原有
人工身份保留机制仍可使用。再次选择已识别车辆时，旧的未分类背景框
不能凭更大 IoU 抢占其选择权。

普通目标被选中后，以 `attach` 接入 C++ 全局实体层，不重新分配公开 ID；
取消选择用 `detach` 释放局部观察器，实体档案不删除。人工框选仍用
`register`，两者不能混同。无需修改网页接口、YOLO 权重或检测类别合同。
网页选中的是中间光流帧时，使用同源、同 ID、最近 1.5 秒内仍在原图缓存
中的真实检测帧建立观察器；局部跟踪仍在用户选择的原帧初始化。
禁止把历史检测时间伪装成当前帧；同采集时刻的光流回调不覆盖缓存中的
真实检测证据。绑定失败可在诊断中查看，而不是静默丢失恢复能力。

检测器重建出新 ID 时，不能只因框重叠就替代旧选择。要求同源、同采集帧、
实际有效 SOT/LK、外观和唯一候选门控，以及连续三帧 C++ 确认。有世界状态
时沿用轨迹拼接的协方差、物理速度和位置创新约束；缺少共同世界状态时，
首个弱图像候选必须在最近可信检测的 1.5 秒内出现；一旦进入确认，
仅允许同一候选在最多 1.5 秒的确认窗口内连续完成三帧（帧间隔不得超过
`tracker/tracklet_stitching/maximum_confirmation_gap_sec`，默认 0.50 秒）。
它不是延长盲预测：候选变化、证据拒绝或连续性中断均清零；长失联仍要求
可靠世界状态。确认期间旧选择保持 `lost/stale`，
不是实测锁定框；新检测仍作为独立未选目标输出。两辆车同时实测可见时
拒绝合并。炮塔小框被另一个完整车身检测框包含时，也视为歧义，不能
把局部算法 `ok=True` 当成可靠实体身份。成功后同步像素、轨迹和选中快照
中的旧 ID 映射；迟到结果也必须经过同一映射。

CSRT/KCF 新增相对最后可信检测锚点的累计尺度检查，防止逐帧缓慢缩到
炮塔或膨胀到背景；已验证的完整车身检测可纠正尺度。健康状态不逐帧
重新创建 CSRT，不增加 YOLO 推理，也不因模型类别抖动更改选择身份。
失效 SOT 留下的旧框不能用于拼接确认。显示仍仅保留类别与 ID。

自动测试云台时，在上面的有界探针命令中增加
`--select-first --image-source search_gimbal`；固定相机为 `search_fixed`。
探针等待实际选择成功后才采样，新增 `overlap_diagnostics` 统计选中框与
其他 confirmed 框的重叠（IoU ≥ 0.15），**它不是带真值的误关联率**。
短段无冲突不能证明所有云台姿态、长遮挡或真实身份准确率已验收。

`/<uav>/perception/pixel_diagnostics` 的 `perception/selected_identity` 条目
提供 `global_id/image_source/observer_attached/manual_roi/fusion_conflict/
fusion_reason/identity_rejection/raw_sot_valid/sot_failure/pixel_lifecycle/
pixel_quality/pixel_capture_stamp`，用于区分算法失效、身份歧义
与身份服务拒绝。探针 JSON 的 `identity_diagnostics` 保存这些状态的采样计数；
`selection_unchanged=false` 表示用户中途改选，不能将该段当作单目标稳定性验收。

全局身份的世界位置及协方差变换复用采集时刻的姿态历史，不使用推理结束
时的最新无人机姿态。若 `TrackState.position_stamp` 是旧的保留定位，不以
当前图像时间再次变换或刷新世界位置；只有真实新定位可更新该证据。
世界创新拒绝时，节点输出限频日志，记录时间差、位置残差、创新值和速度差，
供标定/时间同步排查，不能直接放宽空间门控来掩盖拒绝。

同帧检测漏检/预测不覆盖已经独立推进成功的 LK 观测：只补位同一个
全局 ID、confirmed、质量 ≥ 0.60、最近检测时间严格早于当前帧的视觉
测量；真实当前检测仍保留优先级。CSRT 失败时，track/fusion 可采用该
当前视觉观测，不改变 ID、不刷新真实检测时间、不赋予三维位置有效性。
LK 晚于 YOLO 到达时再次裁决同帧证据，不新增确认次数；身份歧义仍阻止
恢复。预测、过期、低质量或其他采集时刻的像素结果不能用于补位。

身份 attach/bind 的检测证据使用关联后的**原始同帧 YOLO 框与置信度**，
不使用显示框的 Kalman 平滑位置或轨迹置信度 EMA。门限仍为 0.50、
三帧确认不变：原始低分检测不能借旧 EMA 高分通过，原始高分检测也
不会因为旧 EMA 低分而额外等待。网页显示及普通轨迹置信度平滑仍保留。
受控 `attach` 重试只在同 ID 的真实检测通过融合校验后执行，至少间隔
0.5 秒、最多查询四个缓存锚点；不在盲预测/其他 ID 上重试。

本次实测与失败样本说明见 `docs/fusion_selected_lock_recovery.md`。
2026-10-07 最新 60 秒云台段仍有 16/210 帧缺失，最长 5.01 秒，
不能宣称无闪框/无回退验收已通过；135 项 Python、31 项 C++ 及实际
ROS 合同测试通过，只证明相关逻辑与接口，不替代视觉真值验收。

模型切换记录 `logs/model_switch_dropdown_live.json`：train4/train/train7
依次 APPLIED，约 0.56/0.40/0.25 秒；另通过原生网页客户端经真实
WebSocket 切换 train4/train7，约 0.50/0.28 秒，实际模型与 epoch 同步。
这只是当前主机当时负载的操作耗时，不是其他 GPU 的性能保证，也不
说明 train/train4 在坦克场景精度高于 train7。

### 15.5 本轮联合效率优化的范围

保持 train7 权重、两阶段关联、全局匹配、ReID、歧义门控、模式及
模型切换合同不变，本轮调整：

- GPU 锁只围绕主画面/ROI 推理及已有候选预热；图像解码、光流、消息
  构造与发布不再占用该锁。主模型提交仍只在推理线程完整批次边界发生。
- 像素锚点先按原优先级排序截取，再深拷贝；保留对象仍独立于调用方，
  已关闭线程拒绝新增锚点。诊断的有界耗时窗口在短锁内取快照、锁外排序。
- Hungarian 的逐行 scratch buffer 和歧义排序数组在一次匹配内复用，
  仍每轮清零；评分、排序与拒绝条件不变，不跨帧缓存过期匹配结果。
- 检测像素辅助筛选未认领候选改为惰性迭代，不额外分配候选列表；
  保留逐目标防冲突与一对一认领规则。

当前主机局部微基准（不是整体 FPS 或识别精度）：最多保留 8 个锚点，
32 个候选的锚点准备中位数约 0.747→0.199 ms，128 个约 3.029→0.227 ms；
3 个候选约 0.077→0.077 ms，没有可证明的加速。输入选中顺序逐次比较
一致，原始记录为 `logs/algorithm_efficiency_seed_benchmark.json`。

111 项 Python 联合回归与 24 项多目标 C++ 测试通过，包含图像辅助的
检测新鲜度、帧/epoch 隔离、目标优先级、锁范围、跨源和模型切换合同。
记录为 `logs/algorithm_efficiency_python_tests.log` 与 C++ 构建日志。
这些证明的是测试覆盖条件，不替代实拍/同场景视频的精度和 IDF1/HOTA
非退化验收，不能宣称本轮已让所有场景更快或所有性能门限通过。

GPU 集成测试的首次冷启动在约 41 秒才 READY，超过旧测试等待上限
35 秒，记录 `logs/algorithm_efficiency_gpu_switch.log` 保留为失败。
将测试冷启动等待独立设为 60 秒，总测试上限 150 秒、外部 timeout
160 秒；**没有放宽在线候选准备 28 秒、manager 30 秒或推理时延门限**。
复测记录另存 `logs/algorithm_efficiency_gpu_switch_retry.log`，判定须以
实际 RESULT 为准，不能只因增大启动等待就称 GPU 测试通过。

该复测实际 RESULT: SUCCESS（1 个 GPU 集成测试、0 错误/失败），验证
train4 → train → train7 原子提交、未知模型失败保留旧模型、双源 epoch
不回退及既有输出时延/间隔断言。四组网页客户端/显示/切换回归也通过。
GPU 测试输入为合成空白图像，不是坦克视频精度、非空 ROI 推理效果或
真实交叉遮挡验收。本轮没有启动 Gazebo，不能沿用 10 月 6 日视频记录
直接认证新版本所有视觉指标；需要后续同场景 A/B 才能作该结论。

### 15.6 独立感知部署回归（2026-10-07）

新增 pure perception 入口、来源/身份存储边界和节点内飞行输出隔离。
验证记录见 [独立感知验收说明](perception_portable_acceptance_2026_10_07.md)：
91 项单元、12 项启动图、69 项 C++、两项真实 ROS 合同测试通过。
通用 unittest 发现的三个 ROS 集成项按设计跳过，必须单独用 rostest：

```bash
timeout 85s rostest xd_uav_track perception_visual_identity.test
timeout 45s rostest xd_uav_track perception_storage.test
```

测试使用合成图像/候选和真实 C++/Python 节点，不额外启动 YOLO/Gazebo。
install-only 环境已核验新入口、脚本预检、模型/配置路径；没有在另一台
主机上实机验证，也没有用这些合成输入计算 HOTA/IDF1 或检测召回。

## 16. 错误处理、已知缺口与上线检查

### 16.1 常见错误码

| 错误码 | 推荐处理 |
|---|---|
| INVALID_REQUEST / REQUEST_CONFLICT | 校正请求号；相同 request_id 必须完全相同内容 |
| STALE_SESSION | 获取新 state、清旧 UID，请操作者重新选择 |
| STALE_STATE | 获取新状态，重新确认操作后用新请求号提交，不擅自改原请求 |
| INVALID_TTL / INVALID_TIME / EXPIRED | 检查 ROS 时间、TTL 和客户端积压 |
| BUSY / UNAVAILABLE | 查队列、准备状态和节点/服务；不要无限密集重试 |
| INVALID_MODE / INVALID_SELECTION / UNKNOWN_SOURCE | 检查实际枚举和配置来源 |
| INVALID_TARGET_UID / TARGET_CONFLICT | 检查 session、别名和数值 ID 一致性 |
| STALE_FRAME | 原帧不存在/不一致或新 ROI 初始化失败；必须同时读 message |
| TARGET_LOST / INVALID_ROI | 等待有效目标或选择新鲜帧；检查归一化 ROI 范围 |
| ALGORITHM_UNAVAILABLE | 检查 manager 的 OpenCV 算法和初始化条件 |
| INVALID_MODEL / UNKNOWN_MODEL / NOT_PREPARED | 查询注册表，在目标机离线准备 engine |
| HASH_MISMATCH / CLASS_CONTRACT_MISMATCH | 停止使用不一致模型，核对产物和类别顺序，不关闭校验 |
| GPU_UNAVAILABLE / INSUFFICIENT_MEMORY | 检查运行库/设备与显存；降低竞争，不能假定无缝卸载旧模型 |
| LOAD_FAILED / TIMEOUT / INTERNAL_ERROR | 展示 message、查日志，核对旧模型/epoch 是否保持 |

特别是人工新 ROI 的某些失败统一返回 STALE_FRAME，实际原因可能是算法不可用、模板纹理不足、全局身份服务不可用；只弹出“视频过期”会误导用户。

### 16.2 当前仍需处理或独立验收的事项

- 默认总开关尚未统一开启，兼容演示/回放入口未全部转发新模式参数。
- `xd_uav_detect` 主节点未接入其云台/LRF 配置方法；投影工具文件存在不等于运行节点已消费真实测距。
- 跨包运行依赖和每台目标机的 install 干净环境需显式核验，不能仅凭源码编译成功。
- 网页视频、原帧元数据、ROI 映射、版本过滤和 watchdog 已实现并有代码回归；目标浏览器实际视觉操作、双路高分辨率长时带宽/资源仍需独立验收，不能用模拟 DOM 测试替代。
- 车辆域深度 ReID 不等于坦克域已标定；完全相似车辆、长遮挡、光照和移动云台需要有真值数据验收。
- 真正三维定位/跨相机同 ID 需要标定、时间同步、采集时刻 TF、合理协方差；不应仅凭两个画面目标外观相似强行统一。
- 高频辅助实验的部分单场景连续性通过不等于四项升级全部完成；移动相机补偿、融合质量裁决与 TensorRT 数据路径的完整非退化验收仍按各阶段记录推进。

排错时区分：下拉框选了模型但未点击是“待提交”；命令 APPLIED 后
顶部实际模型不更新才是“状态未同步”。更新网页后先刷新旧标签页；
不要以重复启动检测器来解决页面缓存。转弯缺框则查看 measurement_source、
生命周期、state.last_error 和逐次视频记录，不关闭身份门控或画旧框掩盖。

上线前至少核对：

- [ ] 当前主机从独立 install 环境能启动，无旧 MD5/源码路径借用。
- [ ] 每机一套 detector，所有来源/frame/sensor 独立且输入新鲜。
- [ ] state 报告预期实际模型/epoch/backend、model_ready，非仅文件名符合。
- [ ] detect→选择→track→fusion→detect 的 ID/版本/显示逻辑通过。
- [ ] 延迟点击/ROI 被正确拒绝或选中历史原帧，不命中错误最新目标。
- [ ] LOST 不画旧实线框；预测按期限过期；空目标快照能清除 UI。
- [ ] 模型失败保留旧模型；切换后旧 epoch 被清除，类别重新确认。
- [ ] 两路交接在可靠定位与时间条件下保持实体 ID，无证据时保守拒绝。
- [ ] 网页重连/节点重启/队列满/显存不足有可读结果和安全行为。
- [ ] 实拍召回、漏检间隔、ID switch/IDF1/HOTA、误关联率、时延及持续资源稳定性达到项目阈值。

以上全部验收后，再将正式入口默认 enable_perception_modes=true。开关变 true 只是部署默认值调整，不能替代效果验收。

## 17. 相关源码和进一步文档

- [正式启动入口](../launch/vision_tracking_stack.launch)、[模式管理器](../scripts/perception_mode_manager.py)、[统一叠框](../scripts/perception_overlay_node.py)。
- [Gazebo + 网页联调入口](../launch/perception_gazebo_web_test.launch)、[融合恢复记录](fusion_selected_lock_recovery.md)、[高频辅助验收](high_frequency_upgrade_acceptance.md)。
- [公共网页合同](perception_web_contract.md)、[模式与模型方案](perception_modes_and_model_switch_plan.md)、[阶段验收](perception_stage_6_7_8_acceptance.md)。
- [网页桥](../launch/perception_web_bridge.launch)、[客户端](../web/perception_client.js)、[示例页面](../web/index.html)。
- [ReID 配置](../config/reid.yaml)、[车辆 ReID 来源与限制](../models/reid/README.md)。
- [共享检测 launch](../../sar_yolo_detector/launch/xd_yolo_multi_source_detection_integration.launch)、[模型注册表](../../sar_yolo_detector/config/model_registry.yaml)、[离线 FP16 导出脚本](../../sar_yolo_detector/scripts/export_xd_vehicle_tensorrt.py)。

跨包源码链接在源码工作区可用；安装空间中脚本可能位于 lib/<package> 而不是 share/<package>/scripts。运行文件用 rospack/rosrun 或包实际安装规则定位，不依赖本机绝对源码路径。
