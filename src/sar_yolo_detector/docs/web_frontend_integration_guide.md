# 正式方案四模式检测/跟踪：新网页端接口与部署适配指南

校订日期：2026-10-09。接口依据本目录的实际 launch、Python 节点、C++ 桥接和 msg/srv 定义核对。

正式方案确认：用户于2026-10-09选定本工作区下的SAR和XD两包，后续开发、网页接入及部署以此为准。
目录名 `sar_fusion_experiment.wlpAs3` 保留以免破坏路径；下文“实验/探针/验证”描述实现及测试历史，
不再表示方案尚未选定，也不能反过来理解为已通过UE/实机上线验收。
旧 `/home/promise/mrs_test/src` 方案不再作为后续默认选型，本次没有覆盖、搬迁或删除旧包。

本文可作为单文件交接资料：接口合同、算法说明、环境准备、构建/install、启动、配置、验证、排错和完整绑定 JSON 均在正文内。
独立 JSON 仍保留给程序读取，与第18节全文保持一致；不是要求开发人员另找一套接口文档。

### 阅读导航

- 网页开发：第2、4～10节；完整可复制绑定表见第18节。
- 算法与产品了解：第1、6、8、13节。
- 新主机部署：第3、11、14节；配置与双相机示例见第15节。
- 现场演示/回归：第12、16节；异常定位和安全退出见第17节。
- 明确未实现的能力：第1、5、7～9、11节及第18节 `not_provided_by_this_runtime`。

## 1. 适用范围与交付边界

当前正式方案包含：

```text
/home/promise/mrs_test/sar_fusion_experiment.wlpAs3/src/sar_yolo_detector
/home/promise/mrs_test/sar_fusion_experiment.wlpAs3/src/xd_uav_track
```

本指南主要说明SAR的接口改动与其和XD的适配；同工作区的 `xd_uav_track` 保持导出基线不变。
正式采用的是SAR中的PixEagle/PT四模式融合链路，
不是旧工作区的“共享 TensorRT + perception/detect/track/fusion”方案。
确认正式方案没有修改 `Default_Tracker`：通用入口留空时仍遵循配置的SmartTracker；
需要融合请显式使用 `tracking_mode:=FusionTracker`，四模式的实际接口与功能边界不变。

本文是新网页开发的接入合同，**没有在本次文档工作中实现新网页、视频编码网关或新 ROS 接口**。
可复制的接口绑定表见 [web_frontend_interface_manifest.json](web_frontend_interface_manifest.json)，
全文同时嵌入第18节。它是文档附件，不是节点自动加载的配置文件，也不是服务器运行时发现接口。

| 项目 | 当前实际情况 |
|---|---|
| 推荐算法入口 | `sar_yolo_detector/launch/xd_smart_tracker_fusion_integration.launch` |
| 模型 | train7.pt，PyTorch CUDA；此实验不是 TensorRT 共享推理 |
| 四种模式 | `smart_tracker`、`feature_tracker`、`color_tracker`、`fusion_tracker` |
| 原有 ROS 接口 | 保留原 msg/srv；新增平行融合消息和原帧 ROI 服务 |
| 网页承载 | 原始 ROS Image 和服务均有；四模式新网页尚需开发 |
| 旧网页 | XD 的 `web/index.html` 面向旧 `perception/*`，不能直接控制本链路 |
| 飞行/云台控制 | 本指南只提供感知选择，不自动启动飞行、追车或云台电机 |
| 三维位置 | 另接现有定位链；SAR 的 2D 消息本身不提供可信世界坐标 |

当前真实 Gazebo 使用正式地图资源，但运行实验算法；不是 UE 验收记录。重复框专项验证见
[class_duplicate_fix_20261009.md](class_duplicate_fix_20261009.md)，算法和实验边界见
[fusion_experiment_guide.md](fusion_experiment_guide.md)。

## 2. 接入架构与命名规则

```text
相机/UE Image → SAR 四模式节点 → 同帧叠框 Image + 数值框 + 选中状态
                           └→ FusionDetection2DArray → SAR 桥接 → XD DetectionArray
网页 ← HTTP 静态资源 + WebSocket/应用网关 → 订阅输出 / 调用选择与切换服务
```

网页只展示后端结果，不再运行 YOLO、KCF 或第二套框修正。使用已叠框图像时，不要再画一层相同的框。

定义三项配置，不能把主机路径或 Gazebo 名字写死在网页源码里：

```text
uav_name       = uav1
instance_name  = smart
topic_root     = /uav1/sar_yolo_detector/smart
service_root   = /uav1/sar_yolo_smart_tracker_smart
```

其中 `topic_root=/{uav_name}/sar_yolo_detector/{instance_name}`，
`service_root=/{uav_name}/sar_yolo_smart_tracker_{instance_name}`。
这些规则针对默认顶层 launch；用户覆盖话题参数后以实际配置为准。

当前 Gazebo 的绑定为：

```text
uav_name       = moving_uav_search
instance_name  = fusion_fixed
topic_root     = /moving_uav_search/sar_yolo_detector/fusion_fixed
service_root   = /moving_uav_search/sar_yolo_smart_tracker_fusion_fixed
input_image    = /moving_uav_search/fixed_camera/image_raw
bridge_output  = /moving_uav_search/detect/input/fixed/detections_2d
```

网页应同时保存 `namespace/instance/session_uuid/track_id`，不要仅以整数 ID 为跨相机或跨重启主键。
SAR 公开锁定 ID、BoT-SORT 局部 ID 与 XD 全局 ID 是不同身份层，不可直接互换。

## 3. 三类启动：算法、传输、网页彼此独立

### 3.1 新主机启动算法

先部署 ROS Noetic 和两包所需编译依赖，使用独立工作区构建。确认 `rospack find` 指向本实验副本，
不要同时把两份同名包放入同一 src。以下用目标机示例路径，不要求沿用本机绝对路径：

```bash
source /opt/ros/noetic/setup.bash
source /opt/uav/fusion_ws/devel/setup.bash
rospack find sar_yolo_detector
rospack find xd_uav_track

roslaunch sar_yolo_detector xd_smart_tracker_fusion_integration.launch \
  UAV_NAME:=uav1 instance_name:=front tracking_mode:=FusionTracker \
  input_image_topic:=front_camera/image_raw \
  camera_info_topic:=front_camera/camera_info \
  image_width:=1280 image_height:=720 \
  image_source:=front_rgb sensor_id:=front_camera \
  output_detections_topic:=detect/input/front/detections_2d \
  python_executable:=/opt/uav/fusion_ws/.venv-fusion/bin/python
```

图像、内参、实际分辨率、frame、时钟必须与目标设备一致；前视/下视/云台没有绑定不同网页算法。
输入话题相对 `UAV_NAME` 命名空间，绝对话题以 `/` 开头。
该入口启动 SAR 和桥接，不创建相机、不启动 XD tracker、定位或飞行控制。

部署所需 Python 包参考 `requirements-smart-tracker.txt`；融合 KCF/CSRT 需要 OpenCV contrib。
本机采用 `opencv-contrib-python-headless==4.10.0.84`，不要与普通 OpenCV wheel 混装。
本机 `.venv-fusion` 的依赖链接不可直接复制到其他主机；需在目标机重建环境并验证 CUDA、KCF/CSRT。
新入口 CUDA 不可用会明确失败，不应把回退慢速 CPU 误认为正常 GPU 性能。

train7 权重摘要为：

```text
8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54
```

模型必须来自可信部署产物，摘要匹配，且不能 group/world writable；此入口不自动下载或训练模型。

### 3.2 传输和网页

ROS 1 没有为每个话题分配固定网页端口。网页一般连接一个 WebSocket 入口，再指定 ROS 话题/服务名称。
`ROS_MASTER_URI` 的默认 11311 不是浏览器的视频端口，不应直接暴露到公网。

现有旧 UI 预览：HTTP `http://127.0.0.1:8080/`、WebSocket `ws://127.0.0.1:9090`，
只用于看旧 UI。旧 `perception_web_bridge.launch` 的允许列表只包含 `perception/*`；
**即使 WebSocket 已连接，它也不允许本指南的 SAR 话题和服务**。不能只改网页按钮文字就接通。

新网页需单独部署静态资源和带正确允许列表的网关。可在外部部署 launch 中复用未修改 XD 包的
`bound_rosbridge_websocket.py`，或由网页后端转接 ROS。不要与已运行的 9090 冲突。
以下是外部部署文件的示例，**本包尚未增加这个 launch**：

```xml
<launch>
  <node pkg="xd_uav_track" type="bound_rosbridge_websocket.py"
        name="sar_web_rosbridge" output="screen">
    <param name="address" value="127.0.0.1"/>
    <param name="port" value="9091"/>
    <param name="topics_glob" value="[/uav1/sar_yolo_detector/smart/annotated,/uav1/sar_yolo_detector/smart/tracked_detections/fusion,/uav1/sar_yolo_detector/smart/selected_tracked_detections,/uav1/sar_yolo_detector/smart/state,/uav1/sar_yolo_detector/smart/select_pixel,/uav1/sar_yolo_detector/manual/initial_roi,/uav1/sar_yolo_smart_tracker_smart/fusion_evidence,/diagnostics]"/>
    <param name="services_glob" value="[/uav1/sar_yolo_smart_tracker_smart/switch_tracking_mode,/uav1/sar_yolo_smart_tracker_smart/select,/uav1/sar_yolo_smart_tracker_smart/select_fusion_roi]"/>
    <param name="params_glob" value="[]"/>
    <param name="max_message_size" value="33554432"/>
  </node>
</launch>
```

示例绑定默认 `uav1/smart`，不是上一节 `uav1/front`；实际使用须一起替换话题根和服务根。
模型服务默认不对网页开放；需要时按第 9 节加受信任后端代理。
`topics_glob` 不是逐用户/逐操作权限系统，生产部署仍需认证、只读/控制权限和命令审计。
远端浏览器建议经认证的 HTTPS/WSS 反向代理或 SSH 隧道；localhost 指浏览器所在主机，
不是自动指向无人机。不要默认开放 `0.0.0.0`、参数设置或任意 ROS 服务。

## 4. 完整话题清单

以下 `T` 为 `topic_root`，`S` 为 `service_root`；输入/桥接采用默认顶层配置。

| 方向 | 名称 | ROS 类型 | 用途与限制 |
|---|---|---|---|
| 输入 | `/{uav}/down_camera/image_raw` | `sensor_msgs/Image` | 默认相机输入，可覆盖；必须真实采集时间 |
| 输入 | `/{uav}/down_camera/camera_info` | `sensor_msgs/CameraInfo` | 默认桥接内参/尺寸输入，非网页框选坐标依据 |
| 输出 | `T/annotated` | `sensor_msgs/Image` | 原尺寸同帧叠框；有订阅者且开启发布才发布 |
| 输出 | `T/tracked_detections` | `sar_yolo_detector/TrackedDetection2DArray` | 所有最终观测框；兼容旧消费者，无来源字段 |
| 输出 | `T/tracked_detections/fusion` | `sar_yolo_detector/FusionDetection2DArray` | 新网页首选数值接口：原尺寸、来源、最近检测时间 |
| 输出 | `T/selected_tracked_detections` | `sar_yolo_detector/TrackedDetection2DArray` | smart/fusion 的单选观测；无可信选中框为空 |
| 输出 | `T/state` | `sar_yolo_detector/SmartTrackerState` | 选中状态、有效性、实际算法/设备/模型/耗时 |
| 输出 | `S/fusion_evidence` | `std_msgs/String` | 仅 fusion 分支逐帧输出 JSON 证据 |
| 输出 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 汇总诊断；按 hardware_id、节点名和 active_source 过滤 |
| 输出 | `/{uav}/detect/input/detections_2d` | `xd_uav_track/DetectionArray` | 原下游检测合同，必须另接定位才能得到 3D |
| 网页发布 | `T/select_pixel` | `geometry_msgs/PointStamped` | fusion 可原帧绑定点击；无服务式回执 |
| 网页发布 | `/{uav}/sar_yolo_detector/manual/initial_roi` | `sensor_msgs/RegionOfInterest` | 独立 feature/color 初始化；没有原帧时间戳 |

除诊断的实现行为外，不应假定这些输出锁存；重连后须等待新图像、新状态。
`selected_tracked_detections` 不应当作所有模式通用的单目标输出：独立 feature/color 当前会发布空的 selected 流，
其有效结果在全量 detections 和 manual state 中，云台消费端不能忽略此差异。

## 5. 数值框、类别、位置与身份

### 5.1 推荐消息结构

`FusionDetection2DArray` 的实际结构：

```text
header: std_msgs/Header
provenance: sar_yolo_detector/PerceptionIdentity
image_width: uint32
image_height: uint32
detections[]:
  target:
    track_id: int64
    track_id_is_stable: bool
    detection:
      header: std_msgs/Header
      bbox:
        center: {x, y, theta}     # Pose2D，不是 center.position.x
        size_x: float64
        size_y: float64
      results[]:
        id: int64                # 类别 ID，不是 track ID
        score: float64
        pose: PoseWithCovariance # 当前不能作为可信 3D 位置使用
      source_img: sensor_msgs/Image # 当前未填入目标图，不是视频入口
  measurement_source: string
  last_detector_stamp: time
```

原兼容 `TrackedDetection2DArray` 只有 header/provenance/detections[]；
其中每个 detection 是上述 `target`，不含 image_width/image_height、来源或最近 YOLO 时间。

坐标以原图左上角为原点，x 向右、y 向下，单位像素。框中心已提供，不需要再检测图像：

```text
cx = entry.target.detection.bbox.center.x
cy = entry.target.detection.bbox.center.y
w  = entry.target.detection.bbox.size_x
h  = entry.target.detection.bbox.size_y
xmin = cx - w/2; ymin = cy - h/2
xmax = cx + w/2; ymax = cy + h/2
normalized_center = [cx/image_width, cy/image_height]
```

框字段为浮点数，允许亚像素；绘制/点击按原始像素计算。桥接到 XD 的 bbox 为
`[xmin,ymin,xmax,ymax]` 整数，采用 floor/ceil，可能与浮点框边缘相差不到一个像素。

### 5.2 类别与来源的解释

train7 固定类别表：`0: car, 1: ar-car, 2: tank, 3: m142`。
数组没有全量类别名称字段；新网页应依据已校验模型摘要加载部署类别表，不能从文件名或旧 COCO 表猜测。
`state.selected_class_name` 只描述选中目标，不能当作全量类别表。

融合模式中未知人工 ROI 的类别为 `-1`，应显示“未分类”，而不是默认 tank。
与可信 YOLO 匹配后才补类别；已选车辆的子类别暂保持，以避免类别跳变打断锁定，
不是完整类别时域分类器。类别不一致本身不能作为重新分配实体 ID 的理由。

独立 feature/color 的 class_id 是配置 `ManualBoxTracker/class_id`（当前 2），
state 名称是 `manual_roi`：这是人工配置标签，不是 YOLO 识别证据。
这些模式统一写入 `measurement_source=visual_tracker`、`last_detector_stamp=0`，
`state.measurement_current=false`。合格视觉观测可以 `control_measurement_ready=true`，
但不能统计成新的 YOLO 检出或更新类别置信度。选中目标话题与全量数值结果一致，失败均发布空数组。

在 smart/fusion 中，新消息的 `detector` 表示该目标当前使用 YOLO 实测；`visual_tracker` 表示局部视觉辅助，
不能据此刷新 YOLO 最近检测时间。KCF/CSRT 的成功及其质量分数不等同于识别概率。
融合后端的 HSV 是弱外观证据，不是经过验证的坦克深度 ReID。

### 5.3 身份作用域

provenance 字段为 `mission_id/uav_id/session_uuid/sensor_id/profile/model_version/model_sha256/`
`calibration_version/coordinate_contract`。
网页实体键至少包含实例绑定、`session_uuid` 和 `track_id`。不稳定或负 ID 只用于临时展示，不开放可靠 ID 选择。
JSON/JavaScript 对 int64 的大整数精度有风险；当前手动 ID 保持 int32 范围，但外部网关不应随意截断未来 ID，
可用字符串承载应用层 ID。

融合选中目标的公开 ID 可保持不变，而证据中的 local_id 因 BoT-SORT 重建而改变。
不得把 local_id 替换到网页当前选中公共 ID 上。模式/模型切换、节点重启不承诺 ID 持续。
两个相机的局部 ID 恰好相同也不能证明同一车辆；本次修改未新增跨相机全局身份层。

### 5.4 三维位置

SAR 数值框的 `results[].pose` 默认值不是定位结果；`coordinate_contract=CAMERA_RAY` 也不是 3D 测量。
桥接初始输出 has_relative_position_body/has_relative_velocity_body/range_valid 均为 false。
只有现有定位节点补齐并明确置位后，网页才可显示有效三维位置；frame/stamp/协方差必须从该定位合同获取。
像素中心可以在未定位时显示，不得把像素值当米，或把机体系 FRD 数值标成相机/世界坐标。

## 6. 四种模式与切换服务

桥接特别约定：`T/tracked_detections/fusion` 是当前图像的展示/融合快照，可能在同一采集帧收到YOLO后再次修正。
原XD消费者拒绝重复/乱序时间戳，因此正式launch将桥接输入接到新增
`T/tracked_detections/observations`（同一FusionDetection2DArray类型）：只按原YOLO采集时间发布一次真实观测，
选中目标继承经过融合验证的公开ID；独立manual模式则按每帧发布视觉观测。
网页继续读 `/fusion`、`/annotated` 和选中流，不必改成低频桥接观测。XD输出话题、消息、定位接口均未改变。

| mode 值 | YOLO 行为 | 局部跟踪/初始化 | 网页应呈现 |
|---|---|---|---|
| `smart_tracker` | 全图 YOLO + BoT-SORT | 使用原 PixEagle 选择与状态管理 | 多目标检测/跟踪；不是检测无身份模式 |
| `feature_tracker` | 当前帧不运行 YOLO 分支；模型仍在内存 | 人工 ROI，KCF/CSRT 等由配置决定 | 单人工目标，等待 ROI 或有效/失联状态 |
| `color_tracker` | 当前帧不运行 YOLO 分支；模型仍在内存 | 空 color_name 学习 ROI；非空按预设色全图找一个区域 | 颜色视觉目标，不宣称已识别车辆 |
| `fusion_tracker` | 持续全图 YOLO + BoT-SORT | 对一个选中目标运行 KCF/CSRT 并门控融合 | 选中目标融合框 + 其他目标 YOLO 框 |

融合是单选目标局部跟踪，不是每个目标独立 KCF，也不是两个框取并集。
YOLO 与局部跟踪独立调度；同帧检测或经过严格光流对齐的迟到检测才参与位置裁决。
业务 mode 与 `ManualBoxTracker/feature_tracker/tracker_algorithm` 的 kcf/csrt 选择不同，
当前没有网页实时切换 KCF/CSRT 的服务。

服务：`S/switch_tracking_mode`，类型 `sar_yolo_detector/SwitchTrackingMode`。

```yaml
# request
mode: fusion_tracker
color_name: ''
# response 字段
success: bool
message: string
active_mode: string
color_source: string
color_name: string
waiting_for_initial_roi: bool
```

`color_name` 可取 `red/green/blue/white/black/custom` 或空；custom HSV 区间须已在部署配置定义。
服务注释仍只列原三模式，实际节点新增支持 fusion，原 srv 结构/MD5 未改。
旧归档入口没有启用融合时会明确拒绝 fusion；新网页应连接推荐新增入口。

```bash
rosservice call /moving_uav_search/sar_yolo_smart_tracker_fusion_fixed/switch_tracking_mode \
  "{mode: fusion_tracker, color_name: ''}"
```

每次成功调用（包括重选当前相同模式）都会重置局部跟踪、融合缓存及当前选择。
它不保证清空 BoT-SORT 所有历史或恢复旧公开身份；网页需清除旧框/拖拽状态，提示重新选目标。
以 response.success/active_mode 为切换结果，不要只把按钮变色当成功。

state 没有独立 mode 字段。首次连接/重连通过 `/diagnostics` 对应节点的 `active_source`
恢复实际模式（诊断约 1 Hz），并用服务回执更新；不要只依赖 fusion_evidence，它在其他模式会停止。
`state.tracker_type` 表示算法，不能稳定替代 mode。无新帧时状态可能迟到，需显示断流。

## 7. 选择、点击、框选和取消

### 7.1 当前帧按 ID 选择：有回执但不绑定旧画面

服务：`S/select`，类型 `sar_yolo_detector/SelectSmartTrack`。

```yaml
# request，所有坐标默认是原图像素，normalized 开关另行指定
clear_selection: false
use_target_id: true
target_id: 5
use_normalized_point: false
point_x: 0.0
point_y: 0.0
use_normalized_roi: false
normalized_roi: [0.0, 0.0, 0.0, 0.0]  # x,y,width,height，不是 cx,cy
# response
success: bool
message: string
tracking_active: bool
selected_target_id: int64
measurement_current: bool
```

支持当前检测 ID、点、归一化 ROI 及取消。此服务没有 header，不能证明选中的是网页延迟显示的那一帧。
旧 smart 模式 ROI 只能匹配已有检测，不能初始化任意背景/未知目标。
独立 feature/color 请用 7.4 的 ROI 接口，不要拿 select 服务替代其初始化。

```bash
rosservice call /moving_uav_search/sar_yolo_smart_tracker_fusion_fixed/select \
  "{use_target_id: true, target_id: 5}"
rosservice call /moving_uav_search/sar_yolo_smart_tracker_fusion_fixed/select \
  "{clear_selection: true}"
```

### 7.2 融合模式原帧 ROI：推荐的网页框选方式

服务：`S/select_fusion_roi`，类型 `sar_yolo_detector/SelectFusionROI`。

```yaml
# request，完整 header 复制自实际显示的 annotated 图像
header:
  seq: 123
  stamp: {secs: 100, nsecs: 250000000}
  frame_id: uav1/front_camera_optical_frame
roi: {x_offset: 120, y_offset: 80, width: 100, height: 60, do_rectify: false}
clear_selection: false
# response
success: bool
message: string
selected_target_id: int64
```

上面的 stamp/seq 是格式示例，不是可直接选择的真实帧。必须在 fusion 模式，ROI 至少 4×4、位于原图内。
服务器先匹配缓存原帧再寻找目标；包含唯一检测时吸附到紧框，包含多个目标拒绝，无检测时可初始化未知类别。

当前缓存最多 24 帧、24 MiB，最大年龄 0.75 秒，未来容忍 0.02 秒，三种上限同时生效。
高分辨率图像仍可能先达到字节上限；年龄合格不代表帧仍缓存。三项限制均有配置，不能无限放大缓存。
冻结浏览器不会冻结相机或服务器。长时间拖拽、网络延迟超出缓存时应提示刷新重试，
**不能把被拒绝的请求降级为无时间戳最新帧选择**。

不同 ROS 发布者会各自生成 header.seq；跨话题同帧配对主要看 stamp/frame_id，
严格选择回传 `S/fusion_evidence` 中的 `capture_header`（整数 secs/nsecs、原始 seq、frame_id）。
先按 stamp/frame_id 将证据与实际显示的 annotated 配对，不能直接假定 annotated 的发布序号就是原相机序号。
PNG/视频网关应连同画面传递该 capture_header；不能用网关或视频发布者新生成的 seq 替代原帧序号。

### 7.3 融合模式原帧点击：有时间绑定，无请求回执

发布至 `T/select_pixel`，类型 `geometry_msgs/PointStamped`：

```json
{
  "header": {"seq":123,"stamp":{"secs":100,"nsecs":250000000},"frame_id":"uav1/front_camera_optical_frame"},
  "point": {"x":170.0,"y":110.0,"z":0.0}
}
```

完整 header 同样来自显示帧，点必须命中唯一检测目标。
失败通过 ROS 日志告警，没有对应 command_result；网页需看后续 fusion state/selected ID 确认生效，
并区分“已发送”与“已生效”。需要可靠服务回执时，网页框选使用 7.2 的服务。

stamp=0 是旧的最新帧路径，不适合延迟网页；负 x/y 会取消选择。
smart 模式的像素话题仍按当前框距离选择，不执行相同原帧校验，且其单目标流选择与
SmartTracker 内核 state 可能不同步；不能把 fusion 的安全性推广到旧 smart 点击。

### 7.4 独立特征/颜色 ROI：保留旧合同，明确时序限制

先切 `feature_tracker` 或 ROI 颜色模式，然后发布 `sensor_msgs/RegionOfInterest` 到
`/{uav}/sar_yolo_detector/manual/initial_roi`：

```json
{"x_offset":120,"y_offset":80,"width":100,"height":60,"do_rectify":false}
```

该请求在下一张处理图像上初始化，**没有 header/原帧保证，也没有服务回执**。
大延迟网页不可宣称“准确锁住所框原帧目标”；可限制为低延迟本地调试，正式远端优先使用 fusion 原帧 ROI。
零宽/高会清除手动跟踪，指定颜色全图模式不接受 ROI 初始化。
manual ROI 的默认话题没有 instance 后缀，双相机必须在部署配置中分别覆盖
`ManualBoxTracker/initial_roi_topic`，否则一个 ROI 可能同时初始化两路相机。

### 7.5 浏览器尺寸与坐标换算

canvas 内部尺寸用原图 width/height，CSS 只缩放；不能把页面显示尺寸当原图像素。
对于 contain/等比缩放，设显示区域为 Wc×Hc，原图 W×H：

```text
s = min(Wc/W, Hc/H)
offset_x = (Wc - s*W)/2
offset_y = (Hc - s*H)/2
x = (mouse_x_inside_area - offset_x)/s
y = (mouse_y_inside_area - offset_y)/s
```

先减滚动/元素边框位置，点击黑边拒绝，再把框选起终点转换为原图 xywh，取整并裁剪到 [0,W]×[0,H]。
坐标与 header 必须来自同一冻结显示帧；显示下一帧后不能悄悄复用旧 ROI。

## 8. 有效性、丢失状态与重连

融合 state 字段建议解释如下：

| 字段 | 网页含义 |
|---|---|
| has_selection / selected_track_id | 仍保留该身份，不等于仍看到目标 |
| measurement_current | 当前画面有 YOLO 实测；特征/颜色及迟到检测经光流对齐后均为 false |
| control_measurement_ready | 当前门控允许的观测，不等于飞行/云台控制已开启 |
| prediction_only | 仅预测，不画作真实锁定 |
| tentative | 歧义/待确认，不能强制恢复控制 |
| data_is_stale | 旧或歧义数据，不应显示成新实测 |
| association_method / freshness_reason | 关联方式、拒绝或失联原因 |
| tracker_type / backend / device / model_path | 实际算法/后端/设备/模型，不从按钮推测 |
| frames_since_detection | 非全模式统一的秒数，不能直接当目标年龄 |
| frame_processing_ms | 状态/融合计算耗时，非端到端延迟；YOLO独立耗时另见 inference_ms 诊断 |

`S/fusion_evidence` 是 String 内 JSON，fusion 分支字段：

```text
stamp, frame_seq, frame_id, mode, selected_id, local_id,
source, reason, last_detection_stamp, age_ms,
observed, processing_ms, suppressed_class_duplicates,
capture_header, inference_ms, pipeline_epoch, capture_segment
```

source 为 `idle/fused/yolo/visual/prediction/lost/ambiguous`；没有最后真实检测时 age_ms 为 null。
顶层 stamp 是诊断浮点秒，严格框选使用新增 capture_header，不要从浮点秒反推纳秒。
`suppressed_class_duplicates` 只统计融合节点防线抑制数，不包含 YOLO 内部 NMS。
车辆父组去重不代表每一帧类别判定都正确。

fusion 中 `yolo/fused/visual` 才有可信观测框；prediction/lost/ambiguous 不输出选中有效框，
selected 数组为空。显示身份及失联原因即可，不要永久保留上一帧实线绿框制造假锁定。
独立手动模式失败时也不画旧实线框，只显示失联状态。不可从保留的身份状态判断观测仍有效。

融合默认视觉保持 1.5 秒、预测 0.35 秒、短时恢复窗口 8 秒；之后最多保留选中身份至最后可信观测后60秒，
继续检查新检测，但不继续推进旧KCF或绘制旧框。长时重获须更严格验证；不是无限遮挡恢复，详见第19节。
local ID 改变须由3次不同采集时刻的新检测连续确认，光流帧不会加速或打断正常确认。
待确认期间，独立可信且仍在保持期限内的 SOT 可继续输出原公开 ID；不更新最近 YOLO 时间。
候选冲突、SOT不可信或保持超期仍拒绝输出选中框。

网页断线/节点重启处理：

1. 停止发送命令，隐藏或标记冻结画面，清除未确认拖拽与旧目标高亮。
2. 重连后重新订阅；等新 provenance/session、诊断 active_source、图像和 state。
3. session 改变时丢弃上一会话 ID、帧缓存和未完成请求，不自动重发框选/模型切换。
4. WebSocket 在线不等于相机在线；本地按单调时间监测最新帧到达，超过部署设定期限显示断流。
5. ROS 时间回退/相机重计时当前会导致非递增帧被拒绝；没有自动新时间段重建，需协调重启感知实例。

## 9. 模型切换：保留服务，但网页需受信任代理

服务：`S/switch_model`，类型 `sar_yolo_detector/SwitchSmartTrackerModel`。

```yaml
# request
model_path: models/xd_vehicle_train7/xd_vehicle_train7.pt
device: cuda                  # auto / gpu / cuda / cpu
model_sha256: 8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54
# response
success: bool
message: string
effective_device: string
effective_model_path: string
class_count: uint32
```

这是同步服务，加载在状态锁外执行，与YOLO互斥；其他命令在加载期间返回带BUSY的失败消息。
内部pipeline_epoch在切换边界隔离旧任务、光流和结果；没有注册model_id与候选异步预热事务。
当前没有本实例的 models/list、对外model_epoch消息、NOT_PREPARED/INSUFFICIENT_MEMORY 标准错误码，
也没有 GPU 显存预检/网页命令去重队列；不要把其他共享检测节点的同名功能当成本节点能力。
本 SAR 实例同样不发布 trajectory 轨迹话题或 state/get 服务；需要轨迹时由明确接线的 XD 消费者
或网页后端另建有界历史，不能把旧 perception/trajectory 当成已有四模式输出。
底层加载失败会保留旧模型，服务返回实际仍在运行的路径/设备/类别数；进入切换就清除选择及缓存，避免旧身份跨模型残留。
模型服务返回成功不代表首帧推理已预热完成或识别精度已验收，仍需等新的运行状态/图像。

推荐新网页第一版固定 train7；后续模型按钮由受信任后端维护 `model_id → 服务器本地路径、摘要、类别表`，
浏览器不允许提交任意文件路径和自报可信摘要。
模型授权加载涉及可执行权重，摘要只证明内容一致，不能代替来源信任、认证及路径允许列表。
切换其他类别合同还须同步调整 allowed_class_ids、车辆父组、NMS 和网页类别表，
这些配置不会因切换权重自动重建，不能直接把 train/train4 当作已验证可互换模型。
业务 UI 应串行化模式/模型/选择命令，并在模型切换期间禁用冲突操作。

## 10. WebSocket 消息示例与视频承载

### 10.1 原生 rosbridge JSON

订阅数值结果（type 使用 ROS 包名/消息名，不带 .msg）：

```json
{"op":"subscribe","id":"frames-1","topic":"/uav1/sar_yolo_detector/smart/tracked_detections/fusion","type":"sar_yolo_detector/FusionDetection2DArray","queue_length":1}
```

服务切换示例：

```json
{"op":"call_service","id":"mode-1","service":"/uav1/sar_yolo_smart_tracker_smart/switch_tracking_mode","args":{"mode":"fusion_tracker","color_name":""}}
```

典型响应外壳与业务成功须分开检查：

```json
{"op":"service_response","id":"mode-1","service":"/uav1/sar_yolo_smart_tracker_smart/switch_tracking_mode","result":true,"values":{"success":true,"message":"...","active_mode":"fusion_tracker","color_source":"roi","color_name":"","waiting_for_initial_roi":false}}
```

外壳 result=true 不代替 values.success；服务 `id` 只用于传输相关性，不提供服务器幂等保证。
收到回执前不发依赖命令，超时显示“结果未知”并等待实际状态，不盲目重复切换。
发布点击/ROI 前用 `op:advertise` 声明对应类型，再 `op:publish` 携带第7节消息；
前端关闭时 unsubscribe/unadvertise，断线重连重新建立订阅，不累积消息处理器。

### 10.2 最小可运行视频方案：直接订阅 Image

当前已经有 `T/annotated` 的 `sensor_msgs/Image`，不自带 MJPEG/HLS/WebRTC URL，
也不自动生成 `/compressed`。旧 perception/video 消息不是本节点输出。
本机预览可经 rosbridge 订阅 annotated：

```json
{"op":"subscribe","id":"video-1","topic":"/uav1/sar_yolo_detector/smart/annotated","type":"sensor_msgs/Image","queue_length":1}
```

输出为 bgr8。浏览器需将 data 的 base64/字节数组解码，按 step 处理行跨度，BGR 转 RGBA 后绘制 canvas，
不能直接把裸像素当 JPEG/PNG 放入 img.src。保留 header/width/height/encoding/step，
设置单源 latest-only 解码，最多一个运行任务和一个待处理帧，禁止排队追赶旧视频。
这里“原像素”指保持原始分辨率和坐标、不缩放重采样；画框/文字必然改变标注区域像素，
annotated 不与 raw 图像逐字节相同。无损编码只保证叠框图传输保真，不会恢复被标注覆盖的原像素。

640×480、4 Hz、BGR 的裸像素约 3.69 MB/s，base64 约 4.92 MB/s，尚未计 JSON/TCP 开销。
双路1080p/高帧率不要默认使用这种直传方式。

新部署若需要更低带宽，应新增独立无损 PNG 或其他视频网关：保持原始尺寸，保留原 header、帧关联和快照，
编码与推理解耦；不允许通过缩小原图/重新修框改变坐标合同。
PNG 保真但耗 CPU；有损视频可作为另外明确标记的显示选项，不能声称原像素无损。
该网关目前未实现，本指南没有提供虚构的视频 HTTP 地址。

### 10.3 同帧显示策略

优先直接显示后端 annotated，数值面板按 capture stamp/frame_id 配对 fusion 数组；
只保留有界少量快照，等待超时则提示“坐标未同步”，不拿最新框套在旧图上。
ROS seq 跨发布者不保证相同，不能用数组 seq 替换显示图像 seq。
无数据时显示真实空状态；不要新增浏览器平滑框掩盖丢失或制造第二个框。

## 11. 双相机、云台和 UE 部署

两路相机分别配置 instance_name、输入图像/内参、sensor_id、image_source、检测输出与手动 ROI 话题。
网页显示两个独立来源卡片，每个按钮/点击/ROI 都绑定对应 service_root，不通过全局单一“选中ID”猜相机。

当前每个 SAR 实例各自加载一份 YOLO；两路意味着两份模型，不是旧正式栈的一模型共享调度。
部署前评估显存/CPU；无需视频窗口时关桌面查看器，不要再启动第三份模型供网页叠框。
本实验未新增两相机公共身份同步或未知 ROI 的 XD 全局注册，不应在网页自行按距离/同ID硬合并。

smart/fusion 的 selected 流可提供云台视觉伺服输入，但这里只设置检测话题参数，没有启动设备控制器；
云台角度、软限位、tracking-active、健康/控制权和实际转动由设备侧负责。
网页选中只代表感知选择，不代表云台已转向、无人机已追随。

UE 侧必须核对：

- ROS Image 的 BGR/RGB/编码、实际 width/height/step，原图不能提前拉伸后再错误标注内参。
- 图像、姿态、CameraInfo、云台、测距与 TF 使用同一 ROS 时钟；use_sim_time 时持续发布 /clock。
- 默认 maximum_capture_age_sec 来自配置（当前通用配置为2秒），future tolerance 为0.02秒；
  控制时延应另外量测，不能通过无限放宽旧帧门限解决同步问题。
- 画面目标像素大小、遮挡、模糊和训练域差异；Gazebo 指标不能认证 UE 模型精度。
- 世界定位需标定后的真实视觉测量，禁止用 Gazebo/UE 真值 ID 或真值位置冒充检测来证明关联有效。
- 保留类别冲突/误检记录；当前跨类 NMS + 融合去重解决重复输出，不证明 tank/m142 分类正确。

## 12. 网页交付与验收清单

最低可交付 UI：来源卡片和原尺寸视频；类别/公共ID/中心像素/框尺寸；四模式按钮；
fusion 原帧框选/点击/取消；有效/视觉辅助/歧义/失联提示；当前模型/设备/模式和诊断；重连恢复。
模型切换第一版固定 train7 或仅开放可信注册模型，不显示虚构可用选项。

| 验收项 | 判定要求 |
|---|---|
| 接口绑定 | 每个来源解析到本实验实例，旧 perception/* 不混用 |
| 原像素/坐标 | 图像不重采样；页面缩放/黑边下点击仍映射原图；中心与同帧数值一致 |
| 四模式 | 服务 success 和 active_mode 正确；手动模式提示等待 ROI；切换后不残留选中框 |
| 融合框选 | 正确原帧成功；错 seq/stamp/frame、过期、未来、越界、多目标均拒绝 |
| 锁定状态 | 辅助不冒充 YOLO；prediction/lost/ambiguous 不显示成有效实测 |
| 两路隔离 | A路点击不选择B路；手动ROI分源；公共 ID 不擅自跨相机合并 |
| 断网重连 | 不自动重发旧ROI/模型操作；新会话不沿用旧 ID；不积累订阅和解码队列 |
| 模型切换 | 仅可信模型；业务失败明确显示；等待新状态；不假称异步事务/标准错误码 |
| 安全 | localhost/TLS认证代理；精确允许列表；不开放任意模型路径、ROS参数或飞行服务 |
| 负载 | 单/双路分别记录视频FPS、ROS采集到显示延迟、处理P95、CPU/GPU/显存；无重复YOLO |
| 稳定性 | 带真值视频核对召回、ID switch、遮挡重获、误关联；UE另做实测验收 |

现场只读核对命令（以当前 Gazebo 为例）：

```bash
rosservice type /moving_uav_search/sar_yolo_smart_tracker_fusion_fixed/switch_tracking_mode
rosservice type /moving_uav_search/sar_yolo_smart_tracker_fusion_fixed/select_fusion_roi
rostopic type /moving_uav_search/sar_yolo_detector/fusion_fixed/tracked_detections/fusion
rostopic echo -n 1 /moving_uav_search/sar_yolo_detector/fusion_fixed/state
rostopic echo -n 1 /moving_uav_search/sar_yolo_detector/fusion_fixed/tracked_detections/provenance
rostopic hz /moving_uav_search/sar_yolo_detector/fusion_fixed/annotated
```

当前已有72项回归和Gazebo重复框专项记录；不等于新网页已经接通，更不等于UE或实机的精度验收。
本次文档修改不重启演示、不切换模式、不改消息、不修改 `xd_uav_track`。

## 13. 算法详细介绍与实现定位

### 13.1 每帧执行流程

四模式共用 `scripts/smart_tracker_node.py` 的输入检查和消息接口：

1. 检查采集时间不为0、不超龄、不在未来、不重复/乱序，解码成 BGR；无效帧拒绝，不推进跟踪确认。
2. 按实际业务模式选择 YOLO/BoT-SORT、人工特征、颜色或融合分支，不能只从显示框颜色推测算法。
3. 生成数值目标、选中有效性和诊断；融合分支先完成裁决和去重，再共用最终框输出与叠框。
4. 有视频订阅者才发布 annotated；网页断开不会关闭感知算法。

图像回调仅投递 latest-only 邮箱。一个图像线程推进局部跟踪，一个 YOLO 线程独占同一模型执行推理；
输入、推理任务和结果各只缓存一项，旧帧被替换而非排队积压。融合模式逐帧推进已确认目标的有界 LK tracklet 和选中目标 KCF。
迟到检测先用前后向光流校验对齐到当前画面，无法可靠对齐则不发布旧框；其来源标为 visual_tracker，保留真实检测时间。
绘图、编码和ROS发布在状态锁外，数值和视频使用同帧最终框。无视频订阅者不绘图/编码。
模型加载仍是同步服务，但在状态锁外与推理串行；同时发来的操作明确返回 BUSY。
这不是旧共享双相机调度器：每个相机实例仍加载自己的模型。

### 13.2 YOLO、类别门控与 BoT-SORT

YOLO 从当前图像输出 bbox、类别、置信度；模型本身不提供永久实体身份。
train7 的0～3类按车辆父组处理，当前检测输出门限为0.10。
车辆配置启用类别无关 NMS（IoU=0.60），在 BoT-SORT 分配轨迹前减少同车 tank/m142 双框。
此 NMS 是重复候选抑制，不证明保留类别正确；高度遮挡的两辆车也可能互相抑制。

BoT-SORT 的任务是跨帧关联：运动预测、框距离/重叠、分数参与关联，当前库配置带稀疏光流 GMC。
库支持 ReID 不等于本部署启用了深度 ReID；本机 Ultralytics 8.4.118 的默认 `botsort.yaml` 为 `with_reid: false`。
SAR 原 PixEagle 外观模块与新增融合 HSV 门控也不是经过训练验收的坦克深度身份网络。

必须区分三层门限：

| 层次 | 本机当前值 | 作用 |
|---|---:|---|
| SAR/YOLO 输出最低置信度 | 0.10 | 允许弱候选进入 BoT-SORT，不表示已确认锁定 |
| Ultralytics 默认 BoT-SORT high/low/new | 0.25 / 0.10 / 0.25 | 库内部关联和新局部轨迹；不是0.50 |
| 新增融合身份候选门限 | 0.50 | 新 ID 重获/身份证据；0.10～0.50只能有限维持已绑定身份 |

上述 BoT-SORT 值来自已安装库的 `ultralytics/cfg/trackers/botsort.yaml`，不是 SAR 的独立配置项，
升级库或换 tracker 配置后必须重新核对。默认 track_buffer=30 是处理帧数，不能等同于固定30秒。
新增融合层的秒制保持窗口不改变 BoT-SORT 或旧 SmartTracker 所有内部计时机制。
不要声称本次改动已把整个 PixEagle 栈统一改为真实时间模型。

GMC 依赖背景特征；纯色地面、运动模糊、大片目标占画面时可能没有足够匹配点。
有 GMC 不代表已实现按 UAV/云台采集姿态、内外参和深度的完整运动补偿；仍需移动相机/云台场景验收。

### 13.3 特征模式：KCF/CSRT 与失败保护

`ManualBoxTracker` 用人工 ROI 初始化 OpenCV 单目标跟踪器。
KCF 属于相关滤波跟踪，利用目标局部外观寻找下一帧位置；CSRT 也是相关滤波类跟踪器，
引入通道/空间可靠性约束。它们不重新识别类别，也不负责完整的多目标 ID 分配或长期重识别。

本实验配置明确选 `kcf`；如果需要 CSRT，应在部署 YAML 中设置 tracker_algorithm=csrt，
重新启动后检查 state.tracker_type，而不是仅修改网页文字。没有本项目数据不能声称 CSRT 必然更快或更准。
独立特征模式保留原模板兼容回退；融合默认禁止模板静默替代 KCF/CSRT，缺少 contrib 时会拒绝初始化。

安全检查包括中心突跳、面积比、边缘和连续失败次数。
这些是拒绝异常框的机制，不是“永不丢框”保证；完整遮挡、目标出画或背景相似时仍需报告失败。
KCF/CSRT 默认走 OpenCV CPU，与 YOLO 的 CUDA 推理分工不同；存在 OpenCV 代码不代表 YOLO 没有 GPU。

### 13.4 颜色模式：HSV、反向投影与 CamShift

ROI 颜色学习分支从初始目标构建 HSV 直方图，后续做反向投影和 CamShift 更新区域。
指定 red/green/blue/white/black/custom 时按配置色域生成掩膜，经过形态学及区域大小约束寻找目标区域。
该模式只维护一个区域，不是任意数量相同颜色物体的完整身份管理。

颜色是弱线索：阴影、曝光、迷彩变化或同色背景会导致漂移；绿色坦克和绿色地面尤其不能仅凭颜色认定身份。
手动模式的固定类别/ID 是部署标签，不能当成模型已经识别出一辆坦克的证明。

### 13.5 融合模式：候选验证、位置裁决与稳定公开 ID

实现位于 `python/sar_yolo_detector/box_fusion.py`，输入是原帧、采集时刻和 YOLO/BoT-SORT 候选。
选中目标后保留：公开 ID、当前局部 ID、类别、最后可信框、平滑速度、外观锚点和检测/观测时刻。

首先补偿相机运动，再按残余像素常速度预测中心，使用类别父组、位置、面积比、纹理质量、HSV相似度筛选候选。
补偿使用采集时刻的光学相机 TF 与匹配内参进行旋转预测，并用排除目标区域的低分辨率背景 LK/RANSAC 验证和补全平面运动；
无TF时使用背景估计，无背景且TF有效时仅补旋转。单一背景变换不等于所有深度的真实平移补偿。
TF/标定/内点/跨度/运动幅度不合格则跳过，不对 BoT-SORT 的内部 GMC 再叠加一次相同变换。
当前融合自己的运动状态是图像空间速度平滑，不是惯性世界坐标 EKF/IMM。
外观使用48×48 ROI的 L2 归一化 HSV 直方图，质量综合灰度标准差与目标短边尺寸；
它不是专门的模糊检测/过曝检测网络，不应把这一分数宣传成完整图像质量评估。

通过硬门控的候选评分按当前实现为：

```text
score = 0.45 * proximity + 0.25 * IoU + 0.30 * appearance_cosine
proximity = max(0, 1 - 中心距离 / max(8px, 预测框对角线))
```

这些系数是当前代码值，不是 YAML 中可直接调的三个参数。
当检测置信度达到 `detector_authority_confidence`（默认0.85）且局部框通过一致性门控时，
保留检测定位，不被KCF的小幅滞后拉偏；中等置信度仍按下式加权。强检测同样不能绕过身份歧义和位置门控。
最佳和次佳差值不足 ambiguity_margin 时进入 ambiguous；同时看到旧/新竞争轨迹时不强行拼接。
局部 ID 改变或从失联恢复时要求同一个候选连续确认，默认3帧；成功只更换局部 ID，不改公开锁定 ID。

YOLO 和视觉框 IoU>=0.30、尺寸/位置一致才做坐标加权，结果不是两框并集：

```text
alpha = max(0.5, detector_weight * YOLO_confidence /
            (detector_weight * YOLO_confidence + (1-detector_weight) * visual_quality))
fused_xywh = alpha * YOLO_xywh + (1-alpha) * visual_xywh
```

符合身份门控但与局部跟踪不一致的可信 YOLO 会单独校正位置并重新初始化局部跟踪，标记 yolo。
YOLO 暂缺、视觉检查合格且未超时，才使用 visual；两者均无可信位置时只保留 prediction/lost 状态，
不输出假装实测的旧框。已知身份类别暂保持；未知人工 ROI 后续由可信 YOLO 补类别。

外观锚点仅在足够质量且 YOLO 置信度达到身份门限时缓慢更新，避免弱检测污染身份。
相似车辆长遮挡、急转、纯色目标、相机快速转动仍可能无法可靠重获；拒绝歧义优先于错误接管。

### 13.6 重复框处理与下游接口

YOLO前端 NMS 之外，融合对同父组异子类候选/最终结果增加严格几何去重：IoU>=0.75，
中心偏差不超过小框宽高的20%。新鲜且已验证的选中观测优先，其他按置信度保留。
同类竞争轨迹留给身份歧义门控；不会因“两个框距离近”就合并身份档案。

最终框共用于数值、叠框和选中流，桥接继续生成 XD 原 DetectionArray；
新增 `measurement_source` 的平行消息在所有模式区分YOLO与视觉辅助，旧消息类型及MD5保持不变。
SAR 单选不自动写入 XD 的飞行跟随选择，部署时应由上层明确协调，不复用一个整数 ID 猜测。

### 13.7 实现文件索引

| 文件 | 责任 |
|---|---|
| scripts/smart_tracker_node.py | ROS输入/输出、四模式调度、选择/模型服务 |
| python/sar_yolo_detector/pixeagle/smart_tracker.py | 原PixEagle检测跟踪与选择状态管理 |
| python/sar_yolo_detector/pixeagle/backends/ultralytics_backend.py | 权重校验、设备选择、Ultralytics推理 |
| python/sar_yolo_detector/box_fusion.py | 新增秒制单选融合、门控、歧义、公开/局部ID分离、异类去重 |
| python/sar_yolo_detector/manual_box_tracker.py | OpenCV特征/颜色跟踪与安全检查 |
| src/vision_to_xd_bridge_node.cpp | 原vision/tracked及新增fusion桥接，不生成真实测距 |
| launch/xd_smart_tracker_fusion_integration.launch | 新实验部署入口，原三模式及新融合可切换 |
| launch/formal_gazebo_fusion_test.launch | 仿真联调，不作为实机飞行bringup |

## 14. 从零部署到新主机：环境、构建与 install

### 14.1 交付目录与依赖边界

向部署人员交付同版本的 SAR 和未修改 XD 源码、train7可信权重/摘要、配置、许可证和本文。
两包原消息必须与下游/网页网关生成类型一致，不能只复制一份 Python 节点绕过消息构建。
不复制本机 build/devel、虚拟环境 `.pth`、缓存或带令牌的 Git 配置作为部署产物。

两包可在干净 catkin 工作区编译，但运行功能所需依赖必须另行具备：

- 基础：ROS Noetic/catkin、C++编译工具、Eigen、OpenCV、OpenSSL、SQLite及消息生成。
- ROS类型/桥接：rospy/roscpp、geometry/sensor/std/nav/diagnostic/vision/mavros_msgs、tf2等，详见两包 package.xml。
- GPU运行：目标机驱动、与其兼容的PyTorch/torchvision CUDA环境、Ultralytics、lap、NumPy等。
- 网页传输：rosbridge_server及其Python依赖；不是安装YOLO就会自带。
- 仿真才需要：Gazebo/gazebo_ros、rqt_image_view、场景资源、地面定位所需xd_uav_detect。
- XD清单还包含其他xd系统运行包；若rosdep报告这些私有包无法解析，需提供对应包/已验证underlay，
  不能忽略后声称整套系统已独立部署。只运行SAR感知不等于运行了所有XD launch。

本指南不提供ROS2直接启动命令，也不承诺其他OS/Python/CUDA组合未经适配即可等效运行。
GPU是否满足要求以目标机实测为准，不把本机4GB RTX3050表现当成所有硬件指标。

### 14.2 工作区准备和低负载构建

以下命令由部署人员在目标机执行；`/opt/uav`须事先具有适当写权限，
`/srv/artifacts/sar_fusion_src`表示已交付且校验过的源码目录，不是本包自带下载器：

```bash
export FUSION_WS=/opt/uav/fusion_ws
mkdir -p "$FUSION_WS/src"
rsync -a /srv/artifacts/sar_fusion_src/sar_yolo_detector "$FUSION_WS/src/"
rsync -a /srv/artifacts/sar_fusion_src/xd_uav_track "$FUSION_WS/src/"

source /opt/ros/noetic/setup.bash
# 如使用已验证系统underlay，先source其setup.bash，再构建本工作区。
rosdep install --from-paths "$FUSION_WS/src" --ignore-src -r -y
# rosdep有未解决依赖时先处理，不以-r继续执行作为依赖已齐的证明。
catkin_make -C "$FUSION_WS" -j1 -l1 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
  -DSAR_YOLO_ENABLE_TENSORRT=OFF
source "$FUSION_WS/devel/setup.bash"
rospack find sar_yolo_detector
rospack find xd_uav_track
```

明确关闭的是此包可选C++ TensorRT后端，不会关闭PyTorch CUDA；实验要求继续使用train7.pt。
`-j1 -l1`限制构建并发，避免仿真/VSCode同时运行时占满机器；部署构建建议先停自己的仿真任务。
这里只给命令，不自动改正在运行的工作区。

### 14.3 创建独立 GPU Python 环境

本机实际核对：Python3.10.20、torch2.6.0+cu124、torchvision0.21.0+cu124、
OpenCV4.10.0、NumPy2.2.6、Ultralytics8.4.118，CUDA runtime12.4，KCF/CSRT均存在。
这是本机记录，不是目标机最低版本或兼容保证；不能只凭“CUDA版本号相同”认定驱动和wheel匹配。

目标机预置合适的Python3.10/venv后，在专用虚拟环境里安装；CUDA版torch/torchvision须由部署人员
按目标驱动准备匹配wheel，离线设备使用批准的wheelhouse。通用requirements不是完整锁版本部署清单。

```bash
source /opt/ros/noetic/setup.bash
source "$FUSION_WS/devel/setup.bash"
python3.10 -m venv --system-site-packages "$FUSION_WS/.venv-fusion"
"$FUSION_WS/.venv-fusion/bin/python" -m pip install \
  -r "$FUSION_WS/src/sar_yolo_detector/requirements-smart-tracker.txt"
# 上一步通用依赖包含普通OpenCV；只在此专用venv中移除冲突wheel，勿修改ROS系统Python。
"$FUSION_WS/.venv-fusion/bin/python" -m pip uninstall -y \
  opencv-python opencv-python-headless opencv-contrib-python
"$FUSION_WS/.venv-fusion/bin/python" -m pip install opencv-contrib-python-headless==4.10.0.84
"$FUSION_WS/.venv-fusion/bin/python" -m pip check
```

由于Ultralytics的通用wheel依赖可能声明普通opencv-python，而实际采用contrib-headless提供cv2，
`pip check`可能报告这一发行包名称的差异；应逐项核对并记录，不能因此重新混装多个cv2 wheel，
也不能把所有其他依赖报错都视为可忽略。最终运行导入和KCF/CSRT能力必须测试。
NumPy/系统ROS二进制扩展存在ABI兼容风险，不能把某个venv可导入torch等同于全部ROS图像组件可用。
SAR此节点自行转换Image以减少cv_bridge ABI耦合；其他节点仍需按自己的Python环境验收。

```bash
"$FUSION_WS/.venv-fusion/bin/python" - <<'PY'
import sys, cv2, torch, torchvision, ultralytics, rospy
print(sys.executable)
print('torch/vision:', torch.__version__, torchvision.__version__)
print('CUDA:', torch.version.cuda, torch.cuda.is_available())
legacy = getattr(cv2, 'legacy', object())
print('OpenCV:', cv2.__version__)
print('KCF:', hasattr(cv2, 'TrackerKCF_create') or hasattr(legacy, 'TrackerKCF_create'))
print('CSRT:', hasattr(cv2, 'TrackerCSRT_create') or hasattr(legacy, 'TrackerCSRT_create'))
print('Ultralytics:', ultralytics.__version__)
assert torch.cuda.is_available()
assert hasattr(cv2, 'TrackerKCF_create') or hasattr(legacy, 'TrackerKCF_create')
PY
sha256sum "$FUSION_WS/src/sar_yolo_detector/models/xd_vehicle_train7/xd_vehicle_train7.pt"
```

检查模型权限，若被拒绝需由部署人员修正可信模型文件的group/world写权限，而不是关闭摘要检查。
环境通过验收后应冻结wheel/版本/驱动记录；不要在任务启动时pip安装或自动升级。
使用`TRACKER_TYPE=custom_reid`或行人ReID时还有额外依赖/权重；本实验没有因此获得坦克深度ReID能力。

### 14.4 install 部署与启动

```bash
catkin_make -C "$FUSION_WS" install -j1 -l1
# 用新终端验证安装空间，先source所需ROS/系统underlay，再source install。
source /opt/ros/noetic/setup.bash
source "$FUSION_WS/install/setup.bash"
rospack find sar_yolo_detector
rospack find xd_uav_track

roslaunch sar_yolo_detector xd_smart_tracker_fusion_integration.launch \
  UAV_NAME:=uav1 instance_name:=smart tracking_mode:=FusionTracker \
  input_image_topic:=front_camera/image_raw camera_info_topic:=front_camera/camera_info \
  image_width:=1280 image_height:=720 image_source:=front_rgb sensor_id:=front_camera \
  python_executable:=/opt/uav/fusion_ws/.venv-fusion/bin/python
```

安装规则包含模型、配置、文档、包装器和旧archive launch依赖，但不包含虚拟环境、ROS/CUDA或全部外部系统包。
安装空间必须显式指定解释器，不能依赖源码路径自动发现`.venv-fusion`。
移动安装前缀到不同路径/主机后还需重建或校验catkin生成的路径，不假定复制install目录即可完全可移植。

先启动相机/UE输入，确认时间戳和CameraInfo，再启算法；随后启动新网页传输和UI。
冷启动的模型/torchvision懒初始化可能明显长于后续帧，已有记录约23秒；
以持续新鲜图像、实际state/device/model及诊断为准，不能只看ready日志。

### 14.5 服务化与远端网络

无人值守部署的服务启动脚本应source ROS和唯一install/underlay，固定python_executable、配置和模型摘要，
设置日志轮换及合理退出超时。由一层supervisor管理进程组，不同时用多层无条件respawn反复重启YOLO。
每个UAV/实例只运行一套感知入口；相机和网页重连不要再次拉起模型。
模型加载、网页断线不能触发自动飞行指令。

如果相机ROS进程与感知不在同一主机，需要分别设置正确ROS_MASTER_URI和可路由ROS_IP/ROS_HOSTNAME，
ROS1各节点TCPROS端口也必须在可信网络可达；这与浏览器WSS反向代理是两件事。
只让网页通过认证网关访问，不把整个ROS内部网络直接开放给浏览器或公网。

## 15. 配置文件、参数说明和双相机隔离

### 15.1 配置覆盖顺序

当前新增入口按以下顺序加载：

```text
config/smart_tracker.yaml
  → vehicle_config（默认config/smart_tracker_xd_vehicle.yaml）
  → fusion_config（默认config/fusion_tracker.yaml）
  → launch显式参数（模式/话题/模型/设备/来源等）
```

同名参数后加载者覆盖前者。启动后节点大部分参数已缓存，`rosparam set`不等于运行时重载；
模式/模型仅使用相应服务，其他修改通常需重启目标实例。
YAML同一层不能重复键；配置文件中的类名空间须严格区分SmartTracker、ManualBoxTracker、FusionTracker。

### 15.2 检测与融合参数表

| 参数路径 | 当前值 | 含义与风险 |
|---|---:|---|
| SmartTracker/SMART_TRACKER_ALLOWED_CLASS_IDS | [0,1,2,3] | 与train7类别合同一起配置，不随模型服务自动变化 |
| SmartTracker/SMART_TRACKER_CONFIDENCE_THRESHOLD | 0.10 | 输入候选下限，过高会截断低分关联输入，过低增加误检负担 |
| SmartTracker/SMART_TRACKER_MAX_DETECTIONS | 30 | 单帧候选上限，不等于30 FPS |
| SmartTracker/SMART_TRACKER_AGNOSTIC_NMS | true | 当前全部为车辆；加入其他父类后须重新评估 |
| SmartTracker/SMART_TRACKER_IOU_THRESHOLD | 0.60 | YOLO NMS门限，不是最终身份拼接门限 |
| SmartTracker/TRACKER_TYPE | BoT-SORT | 算法内部关联，和四业务mode不同 |
| FusionTracker/visual_hold_sec | 1.5秒 | 最近YOLO缺测时合格视觉辅助上限；越长漂移风险越大 |
| FusionTracker/prediction_hold_sec | 0.35秒 | 无可信观测的内部预测上限，不作为有效检测框 |
| FusionTracker/reacquire_hold_sec | 8秒 | 短时局部跟踪窗口；结束后仍检查可信新检测，不永久退出恢复 |
| FusionTracker/identity_memory_ttl_sec | 60秒 | 每实例最多保留1个选中身份；从最后可信观测计时，不被失败候选刷新；必须大于8秒且不超过300秒 |
| FusionTracker/long_reacquire_appearance_gate | 0.85 | 长时重获外观门限，不低于普通门限；当前HSV是弱线索而非车辆深度ReID |
| FusionTracker/long_reacquire_quality_gate | 0.65 | 长时重获ROI质量门限，不低于普通门限 |
| FusionTracker/long_reacquire_minimum_score | 0.65 | 长时联合关联的绝对门限；只有颜色相似、位置证据弱时不能恢复 |
| FusionTracker/long_reacquire_require_camera_motion | true | 长时重获默认要求自最后可信观测起相机运动补偿连续有效；缺失则保守拒绝 |
| FusionTracker/reacquire_motion_horizon_sec | 2秒 | 陈旧像素速度的最大外推时间，不随失联时间无限扩大位置门控 |
| FusionTracker/max_frame_gap_sec | 0.75秒 | 帧间隔过大重置局部跟踪，不跨长空窗直接推进KCF |
| FusionTracker/max_area_ratio | 2.5 | 面积比硬门控，过宽允许背景框膨胀 |
| FusionTracker/center_gate_sizes | 1.4 | 按框对角线归一化的位置门控，非米数 |
| FusionTracker/appearance_gate | 0.70 | HSV余弦相似度下限，非深度ReID概率 |
| FusionTracker/quality_gate | 0.35 | 纹理/尺寸质量下限，小框或纯色区域容易被拒绝 |
| FusionTracker/ambiguity_margin | 0.12 | 最佳/次佳评分间隔，减小会更激进接管 |
| FusionTracker/reacquire_confirm_frames | 3 | 同一新候选连续确认，不能为了框不断降为单帧 |
| FusionTracker/detector_weight | 0.70 | YOLO/SOT一致时的加权基值，不是所有阶段统一权重 |
| FusionTracker/detector_authority_confidence | 0.85 | 强检测与局部一致时保留检测坐标；不是放宽身份门控 |
| FusionTracker/minimum_detector_confidence | 0.50 | 身份恢复证据门限，不是BoT-SORT new_track_thresh |
| FusionTracker/minimum_maintenance_confidence | 0.10 | 已绑定身份低分维持下限，不允许低分新候选接管 |
| FusionTracker/class_groups | [[0,1,2,3]] | 车辆子类兼容，不能合并不同车辆实体 |
| FusionTracker/selection_max_age_sec | 0.75秒 | 原帧缓存 TTL 与选择年龄门限 |
| FusionTracker/selection_max_frames | 24 | 帧数上限，与TTL及字节上限同时生效 |
| FusionTracker/selection_max_bytes | 25165824 | 24MiB原图缓存字节上限，非可无限提升的网络缓冲 |
| FusionTracker/allow_template_fallback | false | 融合缺少真实KCF/CSRT时拒绝，不静默退化 |
| FusionTracker/async_max_result_age_sec | 0.75秒 | 异步检测对齐及非选中目标LK保持的最大年龄；不循环刷新 |
| FusionTracker/camera_motion/enabled | true | 相机运动补偿开关；无可靠变换时跳过 |
| FusionTracker/camera_motion/max_gap_sec | 0.75秒 | 背景运动估计允许的帧间隔 |
| FusionTracker/camera_motion/max_features | 160 | 背景特征数量上限，估计图宽最多320像素 |
| FusionTracker/camera_motion/minimum_inliers | 12 | 最少内点，同时要求内点比例及背景空间分布 |
| camera_motion_fixed_frame | world | launch参数；可配置为部署惯性系，空字符串禁用TF旋转先验 |
| camera_info_topic | 各相机独立配置 | 同时供桥接和本节点；内参分辨率/frame不匹配或存在未校正畸变时不用TF先验 |
| maximum_image_bytes | 33554432 | 单个原始图像消息上限；超限明确拒收，不缩放输出视频 |
| opencv_threads | 1 | OpenCV工作线程上限，允许1～4；避免WSL多核线程池抢占ROS/GPU |
| SmartTracker/SMART_TRACKER_TRACKER_CONFIG_PATH | config/vehicle_botsort.yaml | 固定BoT-SORT配置，避免不同主机库默认值漂移 |

最终异类去重IoU0.75和中心偏差0.20为当前函数默认代码值，不是新增的YAML可调开关。
`maximum_capture_age_sec`当前通用配置2秒、未来容忍0.02秒、reject_out_of_order=true；
相机帧超龄应先修时钟/积压，而不是无限放宽门限。
ROS仿真时钟回退、相机frame/分辨率改变会建立新时序段并清空局部状态；相机seq重启需两个新鲜递增帧确认。
普通重复/乱序帧仍拒绝。相机自己的开机秒数若未映射到ROS时间，仍拒绝，不能偷偷改成接收时间。
新时序段更换session_uuid，清除选中目标，前端应刷新并重新选择。pipeline_epoch隔离模式/模型切换前的在途结果。
重复请求当前模式不重置正在跟踪的目标；需要明确重置时使用 clear_selection 或切换到另一模式。

### 15.3 单/双相机部署文件示例

新部署可自行创建顶层launch，不必改算法源码。以下仅为文档示例，不是已经新增到包里的文件：

```xml
<launch>
  <arg name="gpu_python" default="/opt/uav/fusion_ws/.venv-fusion/bin/python"/>
  <include file="$(find sar_yolo_detector)/launch/xd_smart_tracker_fusion_integration.launch">
    <arg name="UAV_NAME" value="uav1"/>
    <arg name="instance_name" value="fixed"/>
    <arg name="tracking_mode" value="FusionTracker"/>
    <arg name="input_image_topic" value="fixed_camera/image_raw"/>
    <arg name="camera_info_topic" value="fixed_camera/camera_info"/>
    <arg name="image_width" value="1280"/>
    <arg name="image_height" value="720"/>
    <arg name="image_source" value="fixed_rgb"/>
    <arg name="sensor_id" value="fixed_camera"/>
    <arg name="output_detections_topic" value="detect/input/fixed/detections_2d"/>
    <arg name="python_executable" value="$(arg gpu_python)"/>
  </include>
  <include file="$(find sar_yolo_detector)/launch/xd_smart_tracker_fusion_integration.launch">
    <arg name="UAV_NAME" value="uav1"/>
    <arg name="instance_name" value="gimbal"/>
    <arg name="tracking_mode" value="FusionTracker"/>
    <arg name="input_image_topic" value="gimbal_camera/image_raw"/>
    <arg name="camera_info_topic" value="gimbal_camera/camera_info"/>
    <arg name="image_width" value="1280"/>
    <arg name="image_height" value="720"/>
    <arg name="image_source" value="gimbal_rgb"/>
    <arg name="sensor_id" value="gimbal_camera"/>
    <arg name="output_detections_topic" value="detect/input/gimbal/detections_2d"/>
    <arg name="python_executable" value="$(arg gpu_python)"/>
  </include>
  <!-- 同一份launch解析时在include之后覆盖；启动后才rosparam set不会重建订阅。 -->
  <param name="/uav1/sar_yolo_smart_tracker_fixed/ManualBoxTracker/initial_roi_topic"
         value="/uav1/sar_yolo_detector/fixed/manual/initial_roi"/>
  <param name="/uav1/sar_yolo_smart_tracker_gimbal/ManualBoxTracker/initial_roi_topic"
         value="/uav1/sar_yolo_detector/gimbal/manual/initial_roi"/>
</launch>
```

单相机只保留其中一路和它的ROI覆盖。双路完整输入/输出/服务根不同，还要修改网页绑定及网关允许列表。
默认顶层入口同时配置同一`gimbal_visual_servo/detection_topic`参数，两次include可能后者覆盖前者；
它不启动伺服，但实际云台控制部署须显式绑定gimbal实例的selected流，不能由include先后顺序决定控制权。
上述两路各加载模型，须先测显存，不因为示例可解析就同时在低显存机器上直接满帧率运行。

### 15.4 调优原则

先固定权重/类别/分辨率，记录场景和硬件基线；一次只改一类参数，用同视频逐帧回归。
优先修时间戳、目标有效像素、曝光/模糊、相机角速度和重复推理进程，再考虑放宽跟踪门限。
降低图像/检测目标FPS可减负载，但需测max_frame_gap和像素位移，不能把帧率减到跟踪无法连续还称性能不下降。
扩大画面不等于目标更清楚；飞得更高/更广FOV可能使目标短边太小、纹理质量不足。
不要通过长时间保留旧框、关闭歧义门控或把视觉辅助伪称YOLO来“提高锁定率”。

## 16. 本机演示、验证步骤与已知性能记录

### 16.1 正式地图 + 实验算法

本机使用正式工作区提供资源/已有定位依赖，实验overlay提供当前SAR和未修改XD：

```bash
source /opt/ros/noetic/setup.bash
source /home/promise/mrs_test/devel/setup.bash
source /home/promise/mrs_test/sar_fusion_experiment.wlpAs3/devel/setup.bash
roslaunch sar_yolo_detector formal_gazebo_fusion_test.launch \
  python_executable:=/home/promise/mrs_test/sar_fusion_experiment.wlpAs3/.venv-fusion/bin/python
```

默认打开Gazebo和rqt叠框，纯色地面、3辆坦克、固定相机640×480约4Hz。
运动由已有搜索驱动模拟，SAR选择目标不会使驱动自动追随；出画后的lost不能掩饰成仍锁定。
不需要桌面窗口时可`gui:=false show_image:=false`，网页图像仍须有订阅者且publish_annotated_image开启。
此launch默认formal_track_package是本机绝对路径；新主机必须覆盖为实际地图资源位置，
还需另提供xd_uav_detect等依赖，不能把“两包编译成功”当成Gazebo地图依赖全部齐全。
它不是PX4真实闭环/UE输入入口，也不绑定后续飞机的任务运动学。

旧网页仅UI预览（不连接四模式）：

```bash
roslaunch xd_uav_track perception_web_app.launch \
  UAV_NAME:=moving_uav_search address:=127.0.0.1 http_port:=8080 websocket_port:=9090
```

只在8080/9090未被自己的已有实例占用时运行；浏览器访问`http://127.0.0.1:8080/`。
视频/按钮无法联通的原因见第3节，不应另外启动旧三模式整套算法来“补视频”，否则会重复模型和身份管理。

### 16.2 分层验收，避免高负载反复冒烟

文档或静态配置核对不需要每次重启Gazebo。建议一阶段完成后做一次对应小测试，再做场景回归。
源码环境中的现有Python回归命令（会加载依赖，不启动额外YOLO/Gazebo）：

```bash
cd /home/promise/mrs_test/sar_fusion_experiment.wlpAs3/src/sar_yolo_detector/test
source /opt/ros/noetic/setup.bash
source /home/promise/mrs_test/sar_fusion_experiment.wlpAs3/devel/setup.bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 timeout 120s \
  /home/promise/mrs_test/sar_fusion_experiment.wlpAs3/.venv-fusion/bin/python -m unittest \
  test_box_fusion test_fusion_adapter test_manual_box_tracker \
  test_smart_tracker_image_codec test_pixeagle_portable -v
```

本机最终72项通过；缺少可选ReID依赖时有的测试可能跳过，不能将跳过也计作能力通过。
桥接专项可在准备好的测试环境执行`rostest sar_yolo_detector fusion_bridge.test`和
`rostest sar_yolo_detector vision_to_xd_bridge.test`；前者3项、后者2项曾通过，不能替代模型精度测试。
安装空间是否包含测试资源需单独检查，以上rostest主要面向源码工作区。

已经启动上述Gazebo后，可低负载只读采样（不要再启动第二份地图）：

```bash
rosrun sar_yolo_detector validate_formal_gazebo_fusion.py \
  --duration 45 --output /tmp/fusion_web_check.json
```

该脚本内置当前moving_uav_search/fusion_fixed命名，5～120秒；不是任意部署实例的通用验收器。
默认不选择目标，`--auto-select`会通过服务实际改变SAR选中目标，只有明确要测试选择时才加。
执行`validate_fusion_runtime.py`会发布测试图像、切换模式并选择目标，应在独立测试namespace运行，
不能误接实机输入；它是链路探针，不是真实视频准确率指标。

### 16.3 当前性能：可以证明什么

| 记录 | 实际结果 | 不能推出的结论 |
|---|---|---|
| 单元/适配回归 | 最终72项通过，含真实OpenCV KCF和错误类别重叠框测试 | 不等于真实车辆零误关联 |
| 静态真实模型ROS探针 | 5Hz，39帧融合，ID1保持，处理P95约71.97ms，593×469原尺寸 | 不等于30FPS、移动云台或长期遮挡通过 |
| Gazebo重复框专项45.02秒 | 176帧输出，异类高度重叠0帧、重复数值ID0帧、选中ID1保持 | 不等于每辆车类别真值正确或持续可见 |
| 同次Gazebo耗时/来源 | P95约84.85ms；fused87、yolo4、ambiguous5、lost78、idle2 | 不能将lost删除后宣称全程锁定；不能跨时段直接比较推理加速 |
| 接口只读校验 | 4个当前服务及完整字段、8个输出话题类型核对一致 | 不等于新四模式网页已实现 |

Gazebo该轮原始报告：实验工作区`logs/formal_gazebo_class_dedup_metrics_20261009.json`。
报告kind已明确是plumbing_not_identity_accuracy；passed只代表该脚本判据，不是上线证书。
仍缺UE/实拍带真值的召回、IDF1/HOTA、误关联、长遮挡、端到端网页时延与长期资源曲线。
这些历史日志不随docs必然打包，交接验收时应另附对应版本源码/配置/日志摘要，不能只转述数字。

## 17. 常见故障、定位顺序与安全停止

| 现象 | 优先检查 | 不应采取的掩盖方式 |
|---|---|---|
| 网页只有控件没视频 | 是否误用旧perception网页/允许列表；annotated订阅、图像编码解码、网关权限 | 再起一份YOLO或把裸BGR当PNG |
| WebSocket连通但按钮失败 | 服务根/实例、服务允许列表、业务success、fusion是否由新入口启用 | 仅看传输result=true |
| 框选总过期 | 回传同帧capture_header；24帧/24MiB/0.75秒淘汰；网络和拖拽耗时；ROS时钟 | 失败后偷偷选最新帧 |
| 点选了别的目标 | 原图缩放黑边换算、来源绑定、smart最新帧接口的限制 | 按同整数ID跨相机合并 |
| KCF找不到/融合初始化失败 | cv2来自哪个解释器、是否contrib、实际算法是否回退template | 启用模板后继续声称真实KCF |
| No module named rospy/cv2/torch | source正确ROS/overlay、显式python_executable、venv依赖与ABI | 反复全局pip安装污染系统Python |
| CUDA不可用/显存不足 | 驱动、wheel、torch.cuda.is_available、重复实例、是否两模型 | 默默CPU回退仍显示GPU模式 |
| capture stale/future | /clock、use_sim_time、采集时刻、积压/模型冷启动 | 无限放宽时间门限 |
| 相机重启后不再输出 | 时间戳是否回退，当前没有新时间段自动恢复 | 持续用旧框假装输出正常 |
| 同车两个异类框 | 实际加载车辆配置agnostic NMS、是否重复运行不同节点、融合去重/同帧 | 仅在网页隐藏另一个ID不修数值链 |
| 车辆转弯/云台转动丢框 | 目标像素、角速度、GMC背景匹配、尺度变化、真实时延 | 关闭所有门控或无限coast |
| 切换后选择消失 | 当前合同主动清选择；等待新帧再选 | 自动重发上一会话旧ROI |
| manual被显示为新YOLO | active_source与manual固定类别、兼容来源语义 | 用measurement_source单字段统计所有模式 |
| selected流为空但manual有框 | 独立feature/color选中流当前为空，读全量目标和manual state | 将空数组忽略并继续用旧伺服命令 |
| install能编译但无法启动 | 是否source正确install、解释器显式指定、外部私有包/地图依赖 | 认为只复制二进制就齐全 |
| 双相机互相收到ROI | 默认manual ROI话题无实例后缀，检查启动前覆盖 | 启动后只改rosparam不重建订阅 |

建议只读检查命令：

```bash
rosnode list
rosnode info /moving_uav_search/sar_yolo_smart_tracker_fusion_fixed
rostopic info /moving_uav_search/sar_yolo_detector/fusion_fixed/annotated
rosparam get /moving_uav_search/sar_yolo_smart_tracker_fusion_fixed/SmartTracker
ss -ltnp '( sport = :8080 or sport = :9090 or sport = :9091 )'
nvidia-smi
```

停止时在自己启动的roslaunch终端按Ctrl+C，等待子进程退出，再检查端口和GPU进程。
由服务管理器启动时停止对应服务；没有终端时先查明确切PID/命令，再向本次父roslaunch发送SIGINT。
不要使用`killall python`、广泛pkill或清理用户全部ROS/Gazebo进程；不要删除build/devel来代替关闭进程。
若这是无人机共用ROS master，不要关闭其他系统节点；网页关闭不应停止相机/感知或飞行系统。
日志保留失败和冷启动记录，避免下次只看到成功片段。

## 18. 完整接口绑定 JSON（与独立附件一致）

下面全文可直接复制为`web_frontend_interface_manifest.json`，内容与同目录独立附件一致。
模板中的`{uav_name}/{instance_name}/{topic_root}/{service_root}`需由网页部署配置展开，
ROS节点不会自动读取/替换它。独立附件保留供机器读取；变更接口时须同步更新此全文并做一致性检查。

<!-- BEGIN WEB FRONTEND INTERFACE MANIFEST -->
```json
{
  "document_schema": "sar-pixeagle-fusion-web-adaptation/1",
  "updated_at": "2026-10-09",
  "purpose": "Documentation-only interface binding template; not loaded by the ROS node and not a runtime discovery response.",
  "package": "sar_yolo_detector",
  "launch": "xd_smart_tracker_fusion_integration.launch",
  "solution_designation": {
    "status": "user_selected_official_baseline",
    "selected_on": "2026-10-09",
    "workspace": "/home/promise/mrs_test/sar_fusion_experiment.wlpAs3",
    "packages": ["sar_yolo_detector", "xd_uav_track"],
    "xd_uav_track_implementation_modified": false,
    "ue_real_world_deployment_certified": false,
    "selection_does_not_change_default_mode_or_move_packages": true
  },
  "bindings": {
    "uav_name": "uav1",
    "instance_name": "smart",
    "topic_root": "/{uav_name}/sar_yolo_detector/{instance_name}",
    "service_root": "/{uav_name}/sar_yolo_smart_tracker_{instance_name}",
    "manual_roi_topic": "/{uav_name}/sar_yolo_detector/manual/initial_roi",
    "manual_roi_requires_per_instance_override": true,
    "bridge_output_default": "/{uav_name}/detect/input/detections_2d"
  },
  "current_gazebo_example": {
    "uav_name": "moving_uav_search",
    "instance_name": "fusion_fixed",
    "topic_root": "/moving_uav_search/sar_yolo_detector/fusion_fixed",
    "service_root": "/moving_uav_search/sar_yolo_smart_tracker_fusion_fixed",
    "input_image": "/moving_uav_search/fixed_camera/image_raw",
    "bridge_output": "/moving_uav_search/detect/input/fixed/detections_2d"
  },
  "model": {
    "default_path": "models/xd_vehicle_train7/xd_vehicle_train7.pt",
    "sha256": "8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54",
    "default_execution": "PyTorch CUDA, one model per camera instance",
    "class_names": {"0": "car", "1": "ar-car", "2": "tank", "3": "m142"},
    "unknown_class_id": -1,
    "manual_class_is_configured_not_yolo_classification": true,
    "registry_service_available": false,
    "switch_requires_trusted_backend": true
  },
  "modes": ["smart_tracker", "feature_tracker", "color_tracker", "fusion_tracker"],
  "topics": {
    "annotated_image": {
      "name": "{topic_root}/annotated", "type": "sensor_msgs/Image", "direction": "subscribe",
      "encoding": "bgr8", "source_resolution_preserved": true, "overlay_modifies_pixels": true,
      "requires_subscriber": true, "requires_publish_annotated_image": true
    },
    "targets": {
      "name": "{topic_root}/tracked_detections/fusion", "type": "sar_yolo_detector/FusionDetection2DArray", "direction": "subscribe",
      "id_field": "detections[].target.track_id",
      "class_field": "detections[].target.detection.results[].id",
      "score_field": "detections[].target.detection.results[].score",
      "center_field": "detections[].target.detection.bbox.center",
      "source_field": "detections[].measurement_source",
      "detector_stamp_field": "detections[].last_detector_stamp",
      "manual_mode_source_semantics_require_override": false
    },
    "legacy_targets": {
      "name": "{topic_root}/tracked_detections", "type": "sar_yolo_detector/TrackedDetection2DArray", "direction": "subscribe",
      "contains_measurement_source": false
    },
    "authoritative_observations": {
      "name": "{topic_root}/tracked_detections/observations",
      "type": "sar_yolo_detector/FusionDetection2DArray", "direction": "subscribe",
      "intended_consumer": "existing XD bridge; ordered capture-domain measurements, not presentation corrections",
      "original_yolo_capture_stamp_preserved": true,
      "default_bridge_input": true
    },
    "selected_target": {
      "name": "{topic_root}/selected_tracked_detections", "type": "sar_yolo_detector/TrackedDetection2DArray", "direction": "subscribe",
      "supported_consumption_modes": ["smart_tracker", "feature_tracker", "color_tracker", "fusion_tracker"],
      "manual_modes_currently_publish_empty_selected_array": false
    },
    "state": {
      "name": "{topic_root}/state", "type": "sar_yolo_detector/SmartTrackerState", "direction": "subscribe",
      "latched": false, "contains_explicit_mode": false
    },
    "fusion_evidence": {
      "name": "{service_root}/fusion_evidence", "type": "std_msgs/String", "direction": "subscribe",
      "payload": "JSON", "emitted_in_modes": ["fusion_tracker"],
      "strict_selection_header_field": "capture_header",
      "pair_with_annotated_by": ["stamp.secs", "stamp.nsecs", "frame_id"]
    },
    "fusion_loss_history": {
      "name": "{service_root}/fusion_loss_history", "type": "std_msgs/String", "direction": "subscribe",
      "payload": "JSON", "latched": true, "maximum_rate_hz": 1,
      "emitted_in_modes": ["fusion_tracker"], "recent_completed_events": 2,
      "contains": ["loss_events", "recovered_events", "interrupted_events", "active", "recent_events", "finalized_through_stamp"]
    },
    "diagnostics": {
      "name": "/diagnostics", "type": "diagnostic_msgs/DiagnosticArray", "direction": "subscribe",
      "active_mode_key": "active_source", "filter_by_node_and_hardware_id": true
    },
    "click": {
      "name": "{topic_root}/select_pixel", "type": "geometry_msgs/PointStamped", "direction": "publish",
      "frame_bound_in_modes": ["fusion_tracker"], "has_command_ack": false
    },
    "manual_roi": {
      "name": "{manual_roi_topic}", "type": "sensor_msgs/RegionOfInterest", "direction": "publish",
      "frame_bound": false, "has_command_ack": false,
      "applies_to": ["feature_tracker", "color_tracker with empty color_name"]
    },
    "bridge_output": {
      "name": "{bridge_output_default}", "type": "xd_uav_track/DetectionArray", "direction": "subscribe",
      "initial_metric_position_valid": false
    }
  },
  "services": {
    "set_mode": {
      "name": "{service_root}/switch_tracking_mode", "type": "sar_yolo_detector/SwitchTrackingMode",
      "request_fields": ["mode", "color_name"],
      "response_fields": ["success", "message", "active_mode", "color_source", "color_name", "waiting_for_initial_roi"],
      "successful_call_resets_selection_even_if_mode_unchanged": true
    },
    "select_latest": {
      "name": "{service_root}/select", "type": "sar_yolo_detector/SelectSmartTrack",
      "request_fields": ["clear_selection", "use_target_id", "target_id", "use_normalized_point", "point_x", "point_y", "use_normalized_roi", "normalized_roi"],
      "response_fields": ["success", "message", "tracking_active", "selected_target_id", "measurement_current"],
      "frame_bound": false, "normalized_roi_format": ["x", "y", "width", "height"]
    },
    "select_original_frame_roi": {
      "name": "{service_root}/select_fusion_roi", "type": "sar_yolo_detector/SelectFusionROI",
      "request_fields": ["header", "roi", "clear_selection"],
      "response_fields": ["success", "message", "selected_target_id"],
      "required_mode": "fusion_tracker", "frame_bound": true,
      "header_origin": "exact displayed annotated Image header"
    },
    "switch_trusted_model": {
      "name": "{service_root}/switch_model", "type": "sar_yolo_detector/SwitchSmartTrackerModel",
      "request_fields": ["model_path", "device", "model_sha256"],
      "response_fields": ["success", "message", "effective_device", "effective_model_path", "class_count"],
      "execution": "synchronous service, backend-serialized, state lock released during load, concurrent commands return BUSY",
      "expose_to_browser_by_default": false,
      "has_model_epoch": true, "has_async_warmup_transaction": false
    }
  },
  "coordinates": {
    "origin": "top-left of original image", "x_direction": "right", "y_direction": "down", "unit": "pixel",
    "bbox": "center.x, center.y, size_x, size_y",
    "selection_header_match": ["seq", "stamp.secs", "stamp.nsecs", "frame_id"],
    "cross_topic_pairing": ["stamp.secs", "stamp.nsecs", "frame_id"],
    "cross_topic_seq_equality_guaranteed": false,
    "results_pose_is_valid_metric_measurement": false
  },
  "fusion_selection_defaults": {
    "cache_max_frames": 24, "cache_max_bytes": 25165824,
    "max_age_sec": 0.75, "future_tolerance_sec": 0.02,
    "minimum_roi_width_px": 4, "minimum_roi_height_px": 4,
    "never_fallback_to_latest_on_original_frame_rejection": true
  },
  "not_provided_by_this_runtime": [
    "four-mode web UI", "HTTP video stream endpoint", "automatic compressed Image output",
    "perception/* three-mode contract", "models/list registry", "state/get service",
    "unified request-id transaction queue", "trajectory topic", "cross-camera global identity merging",
    "frame-bound standalone feature/color ROI", "camera timestamp-reset recovery",
    "automatic flight or gimbal hardware actuation"
  ],
  "security": {
    "default_bind": "127.0.0.1", "remote_requires_authenticated_transport": true,
    "legacy_web_bridge_allowlist_matches_sar_runtime": false,
    "do_not_expose": ["arbitrary model paths/digests", "ROS parameter writes", "unrelated services", "flight control"],
    "rosbridge_topic_allowlist_is_not_per_user_per_operation_authorization": true
  }
}
```
<!-- END WEB FRONTEND INTERFACE MANIFEST -->

## 19. 失联、身份保留与重获验收（2026-10-09更新）

### 19.1 状态及恢复边界

本次仅修改本工作区SAR，XD源码和现有ROS消息/服务定义未改，train7权重未改。
每个相机实例最多保留**一个当前选中身份档案**；它不是多目标/跨相机全局身份库。
档案保留公开ID、最后局部ID、类别、外观锚点、位置/速度和最后可信观测时间。
默认TTL为60秒，从最后可信观测计时；失败候选、预测框、重复帧和待确认候选均不刷新TTL。
手动取消、重新选择、切换模式/模型或开启新采集时序段，仍按既有规则清理选择。

- 8秒内：保留既有视觉/预测期限及身份门控，不延长辅助框显示时间。
- 8～60秒：停止旧局部跟踪，但继续处理新YOLO证据。必须通过类别兼容、有限像素运动/尺寸门控、
  更严格外观与ROI质量门控、最佳/次佳候选差距以及同一候选连续3个不同检测采集时刻确认。
  连续候选的位置还必须合理，不能同一局部ID位置跳变却累计确认。
- 默认要求失联期间相机运动补偿链连续有效；缺少补偿、帧间隔过大或候选竞争时不强行恢复。
  当前ROS路径要求背景/姿态引导背景补偿；只有旋转TF而没有平移证据的`pose_rotation_only`可辅助短时跟踪，
  但不能单独证明长时身份位置连续性。
  `long_reacquire_require_camera_motion=false`只能用于明确静止相机的受控A/B试验，不能作为移动无人机的通用配置。
- 成功：保持公开ID，更新局部ID，重建KCF/CSRT和该目标的LK点，清零陈旧速度；不清除其他车辆的LK状态。
- TTL结束：`identity_memory_expired`，清除外观档案，仅保留不可用于控制的选择状态，需用户重新选择。

当前长时外观证据仍是HSV直方图，**不是坦克深度ReID**；位置约束仍是经相机补偿的像素约束，
不是世界坐标运动滤波。缺少距离时，纯旋转补偿也不能解释所有深度的平移。
因此不能把“重新出现且外观相似”认作绝对身份保证；相似目标缺乏区分证据时允许保持lost/ambiguous。

### 19.2 网页/日志诊断接口

原 `S/fusion_evidence` JSON保留原字段，增加：

| 字段 | 用途 |
|---|---|
| `selected_box` | 最终选中框 `[x,y,width,height]`，原图像素；失联时为空 |
| `diagnostics.no_detector_candidates` | 本次有新检测包，但没有可用新候选；不能直接推断目标出画 |
| `diagnostics.rejected` | 候选因置信度、类别、位置/尺寸、外观、质量、稳定ID、时间或运动链被拒绝的数量 |
| `diagnostics.local_tracker` | 局部跟踪有效、失败、外观/尺度拒绝、帧间隔重置等 |
| `diagnostics.lk` | LK有效轨迹数及前后向支持不足、形变、出画、TTL等拒绝计数 |
| `diagnostics.camera_motion_*` | 补偿是否可用、是否连续、当前来源 |
| `diagnostics.input_rejections` | 被拒输入的有界累计原因计数；拒绝帧不推进身份确认 |
| `diagnostics.inference_error` | 异步推理失败信息，区别于正常无候选 |
| `loss_event` | 独立失联事件数量、恢复数量、当前事件和首次原因摘要 |

新增只读 `S/fusion_loss_history`（`std_msgs/String`、JSON、锁存、最多1Hz）。
网页白名单可按需加入此话题，不必改动已有数据订阅。内容含累计事件数、当前事件、最近2个已结束事件，
每事件保存首次原因/门控、最后原因、时长及最多12帧前/12帧后上下文，不保存原图。
节点内部最多32个已结束事件；有界且限频，避免诊断日志反过来拖慢跟踪。

同一采集帧先有视觉更新、后有YOLO校正时，采用最后一次结果；上一帧在下一采集帧到达时才完成事件统计。
因此`finalized_through_stamp`可能落后当前画面一帧，这是避免临时状态被误计为失联事件的设计。
prediction/ambiguous/lost都表示缺少可信当前观测，会进入同一次失联事件；反复上报lost不新增事件。
手动取消或改选结束旧事件，但不算重获成功。末尾未恢复事件属于右删失事件，单独显示为active。

### 19.3 低负载验收与真值要求

`validate_formal_gazebo_fusion.py`保留原消息计数，另输出`capture_acceptance`：
采集帧去重后的DET/视觉/预测覆盖率、独立失联事件、每次时长/恢复耗时、恢复率、拒绝原因，
以及可选的视场内有效跟踪率、错误目标帧数、公开ID变化次数。
原始图像存在但未产出结果时，仍计入覆盖率分母；输入消息次数不能冒充有效相机FPS。
`raw_frames_without_selected_output`单列没有结果的采集帧，事件次数依据最终输出状态转换统计。
缓存最多4096个采集帧，覆盖默认120秒内30FPS；发生淘汰时严格验收不能通过。

无外部真值时，视场内有效率/误关联/真实ID切换为`null`，不能拿YOLO框充当真值。
报告分开提供`plumbing_passed`与`tracking_acceptance_passed`；缺少真值时总`passed=null`，不宣称跟踪验收通过。
普通命令退出码仍对应链路检查；加`--require-tracking`后退出码必须同时满足严格跟踪条件。

真值JSON示例（采集时间、frame_id必须与原图一致；目标出画需显式`visible:false`，不能省略当作出画）：

```json
[
  {"frame_id":"moving_uav_search/fixed_camera_optical_frame",
   "stamp":{"secs":19,"nsecs":100000000},
   "objects":[{"id":"tank_a","visible":true,"box":[60,60,40,40]},
              {"id":"tank_b","visible":false}]}
]
```

文件上限8MiB、10000帧、每帧100目标，拒绝重复采集时间/目标ID及非法框。
标注坐标同样为左上角原点、原图像素。不得将仿真目标ID直接等同于tracker ID；
`--selected-id`选tracker目标，`--truth-target-id`指定它对应的实体真值。
单次验收只测试一个明确选中实体，不能中途人工改选另一实体后仍计算同一个身份的正确率。
模型/时序切段后应分别验收，当前像素真值格式不认证跨时序段的身份连续性。

```bash
rosrun sar_yolo_detector validate_formal_gazebo_fusion.py \
  --duration 45 --selected-id 7 \
  --truth-annotations /absolute/path/labels.json --truth-target-id tank_a \
  --require-tracking --minimum-in-view-rate 0.95 --minimum-truth-coverage 0.95 \
  --minimum-recovered-events 1 --output /absolute/path/acceptance.json
```

工具默认门限不替代既有项目性能基线；实际验收应取项目基线与场景要求中的更严格值。
当前单元测试包含真实OpenCV KCF长失联后重获、转弯120帧基线、多候选拒绝、缓存上限和异步线程吞吐；
这些是受控算法回归，不能代替UE/实拍标注视频验收。
