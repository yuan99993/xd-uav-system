# sar_yolo_detector

面向多无人机应急搜救的 ROS1 感知与任务网关包。**主要入口是部署在侦察机上的
PixEagle SmartTracker**：相机图像经 YOLO 识别和稳定跟踪后，目标连同可验证的方位或
世界坐标被转换为规划器可消费的任务候选。规划器完成全局去重、定位融合、分配与航线规划后，再通过每架工作机
自己的任务网关下发任务。

包内按 `scout`（侦察感知上行）和 `worker`（规划任务下行）隔离职责。SmartTracker
及任务候选节点不发布飞控命令；工作机网关只把通过身份、地理参考、鉴权、能力和
状态检查的任务交给本机执行器。核心接口不依赖 Pod、Follower、MRS 或 PX4，因而可由
MRS 执行器在边界之外接入。`sar_yolo_detector` 是规划电脑和各飞机之间的稳定
ROS 消息合同。

## 支持范围

- 主输入：侦察机 EO `sensor_msgs/Image`
- 主输出：SmartTracker 发布带稳定 ID 的 `TrackedDetection2DArray`；飞机目标经时序
  确认后，以方位线索或有效三维定位上行发布 `PerceptionCandidateArray`
- 兼容输出：C++ YOLO profile 发布 `vision_msgs/Detection2DArray`；FloodNet profile
  发布 `sar_yolo_detector/FloodRegionArray`
- 后端：`opencv_dnn`、`tensorrt`、`test`；Noetic/OpenCV 4.2 另提供
  `opencv_darknet` 兼容后端，用于老系统的 COCO 冒烟验证
- 模型：未嵌入 NMS 的 YOLOv5、YOLOv8、YOLO11 detect ONNX，或 Darknet
  `.cfg + .weights`（兼容测试）
- 推理：PixEagle SmartTracker 使用 Ultralytics `.pt`；五个兼容 profile 为 C++，
  TensorRT 支持 FP32/FP16 输入输出。engine 后端在构建时检测到 CUDA
  runtime、`NvInfer.h` 和 `libnvinfer` 后才启用；缺少依赖时会明确报错。
- 测试：`test` 后端在没有模型/GPU 时生成确定性原始 YOLO 输出，用于接口回归
- 调度：单工作线程、latest-frame-wins，推理繁忙时覆盖旧帧
- 安全：采集时间戳、未来/乱序/重复/过期帧拒绝、NaN/输出形状检查、模型 SHA-256 启动校验
- 可观测性：`/diagnostics`、`VisionInfo` 和可选检测框图像
- 任务候选：独立 C++ 节点将稳定检测转为带定位状态、协方差、ID 和生命周期的
  `TaskCandidateArray`；决策桥通过 SQLite/WAL 持久化任务与状态，完成鉴权、恢复对账、
  执行器能力校验、任务接收和状态回传，它不直接控制飞行器

飞机识别入口附带 COCO YOLO11n 基线权重，COCO 类别 4 `airplane` 是唯一默认任务类别；
它用于打通接口，尚未针对侦察机视角、小目标、遮挡和任务空域完成部署验收，因此
`model_validated` 默认保持 `false`。maritime、thermal 和 FloodNet pilot 作为兼容
profile 保留；训练权重和数据集的适用许可证仍需部署方逐项确认。

## 构建

```bash
source /opt/ros/noetic/setup.bash
cd /path/to/catkin_ws
catkin build sar_yolo_detector
source devel/setup.bash
```

## 模型要求

导出原始 detect head 的 ONNX，不要把 NMS 合入模型。`class_names` 的数量和
顺序必须与 ONNX 输出完全一致：

- YOLOv5：每个候选为 `4 + objectness + class_count`
- YOLOv8/YOLO11：每个候选为 `4 + class_count`

较老的 OpenCV 可能不能解析新 ONNX 算子。当前 Noetic 环境的 OpenCV 4.2 已实测
不能加载常见的现代 YOLO ONNX（INT8、较新的 INT64 常量均会失败）。遇到加载错误
时，应在训练环境导出兼容 opset、升级 OpenCV，或在部署 GPU 上用本包的 TensorRT
engine 后端；不能在控制电脑上静默换模型。

## 主要飞机识别链与兼容 profile

飞机识别不通过下表的旧 `rescue_profile.launch` 切换，而由
[`scout_perception.launch`](launch/scout_perception.launch) 作为侦察机主要入口。
该入口加载 `aircraft_coco` profile：SmartTracker 可以跟踪和显示模型识别的所有
COCO 类别，但任务生成器只允许类别 4 `airplane` 进入规划接口。

以下五个 C++ 应急救援 profile 继续作为兼容模式。它们的 Tracker 输入、任务候选
消息和安全策略保持一致，只替换权重、传感器语义、类别合同和任务话题：

| profile | 传感器/数据源 | 默认任务目标 |
|---|---|---|
| `wildfire_ir` | FireMan Multiclass 热红外旧基线 | `fire_region`、`smoke_region`、`building`；仅用于复现，未达到生产精度 |
| `wildfire_ir_fire_smoke` | FireMan thermal v2（完成训练后） | `fire_region`、`smoke_region`；仅火焰可生成定位任务候选，烟雾仅作复查/告警线索 |
| `maritime_person` | SeaDronesSee v2 海上 EO | `swimmer/boat/jetski/life_saving_appliances/buoy`；仅 swimmer 和救生物品默认生成救援任务 |
| `thermal_uav` | HIT-UAV 高空热红外 | `person` |
| `floodnet` | FloodNet EO 语义分割 | `building_flooded/road_flooded/water` 世界多边形证据，可由决策层生成独立区域任务 |

切换 profile 时不需要修改 C++ 代码：

```bash
roslaunch sar_yolo_detector rescue_profile.launch \
  mission_id:=rescue_20260818 uav_id:=uav_01 profile:=thermal_uav \
  inference_backend:=tensorrt
```

`thermal_uav` 默认直接使用包内 `models/thermal_uav/` engine；`wildfire_ir`
默认使用 `models/wildfire_ir/fireman_ir_fp16.engine`。如需替换权重，传入
`engine_path:=/absolute/path/other.engine` 即可。两套稳定权重由训练脚本在完成
验证和 TensorRT 导出后原子刷新（同时保留 `.pt`、`.onnx` 和
`training_metadata.json`）。

可选值是 `wildfire_ir`、`wildfire_ir_fire_smoke`、`maritime_person`、`thermal_uav`
和 `floodnet`。对应的检测与任务话题
均为相对话题并由飞机命名空间隔离，例如 `/uav_01/sar_yolo_detector/thermal_uav/detections`、
`/uav_01/sar_yolo_detector/thermal_uav/task_candidates`。`VisionInfo.method` 与诊断中也会
记录 profile 名称，防止不同传感器和权重合同误接。

FloodNet 使用同一个搜救前端入口：

```bash
roslaunch sar_yolo_detector rescue_profile.launch profile:=floodnet
# 或使用简化入口
roslaunch sar_yolo_detector floodnet_profile.launch
```

该启动会运行完整的 `RGB -> SegFormer/TensorRT -> MONO8 -> 区域跟踪` 链路，
并发布 `/uav_01/sar_yolo_detector/floodnet/regions`。包内已经同时提供
`models/floodnet_segformer_b0/floodnet_segformer_b0_pilot_10e_best.pt`、对应
ONNX 和 TensorRT 10.1 FP16 engine；输入固定为 `1×3×1024×1024`，engine 输出
十类 `1×10×256×256` logits。C++ 推理节点完成 ImageNet 预处理、FP16/FP32
输出解码、argmax 和掩膜回缩，并保留采集时间戳。这样上层规划包只需依赖
`sar_yolo_detector`。本目录已经包含 FloodNet 的消息、推理节点、区域提取器、启动
文件、训练脚本、配置和模型文件；移植时不需要再复制另一个洪水感知 ROS 包。

Flood 区域提取器用采集时刻的 `CameraInfo`、相机/云台 TF 与 `TerrainGrid` DEM/DSM
把简化后的图像轮廓投影为世界坐标多边形，同时发布质心、实际面积、投影覆盖率和
保守定位协方差。缺少任一时空/标定条件时会保持 `localization_valid=false`。它还为
证据帧计算 SHA-256，并通过 `mission_interface/get_evidence` 按 UUID 提供有界缓存中的
JPEG ROI，而不是持续发送原图。多机重叠区域的合并与全局 `region UUID` 由上级决策层
在共同 `GeoReference` 下完成。

HIT-UAV `thermal_uav` 和 FireMan `wildfire_ir` 均已在 4 GB GPU 上完成训练。
FireMan 模型的 50 epoch 结果及类别限制记录在
[`models/wildfire_ir/README.md`](models/wildfire_ir/README.md)；它目前是
山火热红外的数据转换基线，不能替代包含烟雾和建筑验证样本的生产训练。

HIT-UAV smoke 训练使用 YOLO11n、640 输入、batch=1、1 epoch，原始训练产物示例位于：

```text
/data/sar_yolo/runs/hit_uav_smoke/weights/best.pt
/data/sar_yolo/runs/hit_uav_smoke/weights/best.onnx
/data/sar_yolo/runs/hit_uav_smoke/engine/hit_uav_yolo11n_fp16_sm86_trt10.engine
```

部署所需文件已同步收进包内的
[`models/thermal_uav/`](models/thermal_uav/)，协作者不需要访问上述 `runs/`
目录或重新训练。

后续正式训练完成后，可用以下参数将新 checkpoint、ONNX 和 TensorRT engine
原子刷新到包内稳定文件名；训练失败或中断时旧模型不会被替换：

```bash
python3 src/sar_yolo_detector/training/train_profile.py \
  --profile ir_hit_uav \
  --data /data/sar_yolo/hit_uav/dataset.yaml \
  --device 0 --epochs 100 --batch 1 --imgsz 640 \
  --export-engine --refresh-package-models \
  --min-map50 0.70 --min-recall 0.65
```

该短跑验证集整体 mAP50 约 0.29，person mAP50 约 0.60，仅用于确认数据、CUDA、
TensorRT 和 ROS 接口链路，不能作为最终部署精度。TensorRT 10.1 `trtexec` 冒烟测得
GPU 推理约 3.76 ms；启动时默认使用包内 `models/thermal_uav/thermal_uav_fp16.engine`。

## 启动

```bash
roslaunch sar_yolo_detector detector.launch \
  model_path:=/absolute/path/rescue_yolo.onnx \
  input_image_topic:=/camera/image_raw
```

查看输出：

```bash
rostopic echo /sar_yolo_detector/detections
rostopic echo /sar_yolo_detector/vision_info
rostopic echo /diagnostics
```

启用调试图：

```bash
roslaunch sar_yolo_detector detector.launch \
  model_path:=/absolute/path/rescue_yolo.onnx \
  publish_debug_image:=true
```

TensorRT engine（部署 GPU 构建后）示例：

```bash
roslaunch sar_yolo_detector detector.launch \
  inference_backend:=tensorrt engine_path:=/absolute/path/rescue.engine \
  input_image_topic:=/camera/image_raw
```

## PixEagle SmartTracker（侦察机主要节点）

`python/sar_yolo_detector/pixeagle/` 是整理后的独立移植层，包含检测结果归一化、
AABB/OBB 几何、ROI/点选校验、运动预测、Kalman 框跟踪、外观重识别、跟踪状态机、
坐标/云台变换、命令意图、偏航率平滑、Ultralytics 后端和完整 SmartTracker。
原 PixEagle 的 UI、Web API、配置管理器、视频输入、MAVSDK/PX4 和 Follower 没有耦合
进来；ROS 节点只提供图像、选择、状态和模型切换边界。识别、Kalman 跟踪、丢失预测、
重捕获和外观重识别保留在 SmartTracker 内，飞控职责仍在工作机执行器侧。

侦察机运行 SmartTracker 需要 Python 环境。推荐在工作区根目录使用独立环境；启动
脚本会自动发现 `/path/to/catkin_ws/.venv-sar-gpu`，也可显式传入 Python：

```bash
cd /path/to/catkin_ws
python3 -m venv --system-site-packages .venv-sar-gpu
.venv-sar-gpu/bin/pip install torch==2.4.1+cu121 \
  torchvision==0.19.1+cu121 \
  --index-url https://download.pytorch.org/whl/cu121
.venv-sar-gpu/bin/pip install -r \
  src/sar_yolo_detector/requirements-smart-tracker.txt
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sar_yolo_detector scout_perception.launch \
  mission_id:=rescue_01 scout_uav_id:=scout_01 \
  input_image_topic:=camera/eo/image_raw \
  python_executable:=$PWD/.venv-sar-gpu/bin/python
```

包装脚本只补充 `/opt/ros/$ROS_DISTRO/lib/python3/dist-packages`，不会把另一 Python
小版本的系统 NumPy/OpenCV 注入虚拟环境；`rospkg` 由上述可选 requirements 安装。

主要入口默认使用包内 `models/aircraft_coco/yolo11n.pt`，并在加载前校验 SHA-256、
文件类型、大小和写权限。通用兼容入口 `smart_tracker.launch` 仍默认使用 thermal
模型。配置还登记了 thermal、maritime 和 wildfire 权重的可信摘要。切换外部 `.pt` 时必须通过
`~switch_model` 服务同时提交预期 SHA-256；失败时旧模型保持活动。

ROS 接口：

- 输入 `camera/eo/image_raw`（可配置）：`sensor_msgs/Image`
- `sar_yolo_detector/aircraft/tracked_detections`：全部当前检测及稳定 ID 标志
- `sar_yolo_detector/aircraft/tracker_state`：选中目标、关联方式、预测/暂定/陈旧状态和时延
- `sar_yolo_detector/aircraft/annotated`：可选 HUD 图像
- `~select`：按目标 ID、像素/归一化点或归一化 ROI 选择，也可清除选择
- `~switch_model`：原子切换可信本地 YOLO `.pt` 模型

以上均位于侦察机命名空间下，例如首项完整话题为
`/scout_01/sar_yolo_detector/aircraft/tracked_detections`。

`SmartTrackerState.control_measurement_ready` 只有在选中目标由本帧真实检测确认，
且不是预测或暂定重捕获时才为 `true`。时间戳为零、过期、未来、重复或乱序的图像会
在推理前被拒绝。这个状态只表示感知测量新鲜，不代表飞行安全或允许控制。

PixEagle 移植代码及 Ultralytics 运行时/权重的第三方来源和许可证见
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) 与
[`LICENSES/`](LICENSES/)。随包 YOLO11n 基线受 AGPL-3.0 或适用的 Ultralytics
Enterprise 许可约束；发布闭源飞行系统前必须由部署方完成许可证审核。

## 检测到任务点：接入任务决策层

`sar_task_point_generator` 在检测器之后运行。它只产生**候选**任务点，绝不向
`/move_base_simple/goal`、MAVROS、MRS 或 XD 控制器发布指令。任务决策层应读取
候选的置信度、定位有效性和 `EXPIRED` 事件，再自行执行去重、地理围栏、能量、
航线可行性与人工授权检查。

```text
侦察机 scout_01
EO Image -> PixEagle SmartTracker -> TrackedDetection2DArray
          -> 定位/时序确认 -> TaskCandidateArray
          -> scout bridge -> /scout_01/mission_interface/perception_candidates
                                      |
                                      v
                         外部多机规划/任务分配包
                           |                    |
                           v                    v
 /worker_01/mission_interface/submit_task   /worker_02/.../submit_task
             |                                  |
        worker bridge                       worker bridge
             |                                  |
        本机执行器/MRS                      本机执行器/MRS
```

侦察机启动主要识别链：

```bash
roslaunch sar_yolo_detector scout_perception.launch \
  mission_id:=rescue_01 scout_uav_id:=scout_01 \
  input_image_topic:=camera/eo/image_raw \
  camera_info_topic:=camera/eo/camera_info
```

每架工作机只启动自己的任务网关（示例为 `worker_01`）：

```bash
roslaunch sar_yolo_detector worker_task_gateway.launch \
  mission_id:=rescue_01 worker_uav_id:=worker_01 \
  authorization_key_file:=/secure/decision_primary.key \
  geo_reference_validated:=true geo_map_uuid:=rescue_map_01
```

`scout_perception.launch` 中的 `model_validated` 默认是 `false`：SmartTracker 仍会发布
识别和跟踪结果，但任务候选为空。只有完成飞机模型、相机、飞行高度和现场数据验收后，
才应设为 `true`。默认 `bearing_only` 会把飞机识别结果作为带相机射线的任务线索发送给
规划器，但不会声称已有三维坐标。空中飞机的位置必须由测距、双机方位交会或多视角
估计补齐；不能把视线与地形的交点当作空中飞机位置。仅当目标已确认位于地面时，才可
启用 `terrain`，且必须获得同一采集时刻的 CameraInfo、相机/云台 TF、有效 TerrainGrid
和经过验证的 GeoReference。

旧 C++ 检测 profile 可继续联合启动检测器与任务候选节点：

```bash
roslaunch sar_yolo_detector detector_task_point.launch \
  inference_backend:=tensorrt \
  engine_path:=/absolute/path/rescue.engine \
  input_image_topic:=/camera/eo/image_raw \
  camera_info_topic:=/camera/camera_info \
  task_generation_enabled:=true model_validated:=true \
  localization_mode:=terrain task_frame:=map
```

感知包内部接口如下：

- `/sar_yolo_detector/task_candidates`：
  `sar_yolo_detector/TaskCandidateArray`，是任务决策层的**权威接口**。
  每个 `TaskCandidate` 含 `mission_id/uav_id/session_uuid`、全局语义、模型/标定版本、
  SHA-256、观测 UUID、事件序号、持久 `track_id`、平滑置信度、稳定度、优先级、
  观测次数、图像框、相机射线、定位协方差及 `PENDING/CONFIRMED/UPDATED/EXPIRED`
  生命周期。
- `/sar_yolo_detector/task_point`：`geometry_msgs/PoseStamped`，仅在候选已
  `CONFIRMED`、`localization_valid=true` 时发布，方便现有只接受 Pose 的决策适配器。
  `Header.seq` 是 ROS 发布序号，不可当作目标 ID；请以候选消息的 `track_id` 为准。

默认 `bearing_only` 不产生导航坐标。`terrain` 使用采集时刻的 CameraInfo、云台/机体 TF
和 `TerrainGrid` DEM/DSM 沿射线求交；兼容用的 `horizontal_plane` 才与固定高程面相交。
人员类默认取框底部而不是框中心。协方差传播像素、位姿、姿态、时间同步、标定和地形误差；没有 CameraInfo、
TF、射线近似水平，或交点超出距离范围时，候选仍可上报，但
`localization_valid=false`，不会生成 Pose 任务点。`bearing_only` 应保持为默认值，
直到坐标合同、地形、相机/云台位姿和时间同步经过现场验收。

默认策略要求全部 3 次观测落在同一个 1 秒窗口；优先消费稳定 Tracker ID，否则采用
速度预测、协方差/Mahalanobis 门控和全局一对一匹配，最后才退化为图像门控。候选 2 秒未更新会发布 `EXPIRED`，因此任务层能
撤销失效线索。全部阈值位于
[`config/task_point_generator.yaml`](config/task_point_generator.yaml)。
节点每秒发布 `full_snapshot=true` 的全量状态，也提供
`.../task_candidates/get_snapshot` 服务；FloodRegion 提供同样的快照/查询合同。

通用 profile 的任务生成默认关闭；主要侦察链将流程接通，但
`model_validated=false` 使输出保持为空。只有 `task_generation_enabled=true` 且
`model_validated=true`（或显式 `allow_experimental_model=true`）并配置非空任务类别时
才会产生候选；旧 FireMan profile 被硬性禁止生成任务。

### 跨电脑规划接口与角色隔离

本包提供候选上行、任务下行和状态回传的通信合同，**不包含全局任务分配或路径规划
算法**。规划功能包订阅所有侦察机的候选，完成合并、约束检查和工作机选择，然后调用
目标工作机命名空间下的提交服务。决策端开发者只需将整个 `sar_yolo_detector` 目录
复制到自己的 catkin 工作区 `src/` 即可；不再需要复制原有的接口包或洪水感知包。
运行时只要求目标电脑安装 ROS Noetic、catkin、OpenCV、
OpenSSL 和 SQLite；TensorRT、CUDA 与 Python SmartTracker 依赖按需安装。桥接接口
统一位于 UAV 命名空间下：

| 方向 | 默认端点 | 消息/服务 |
|---|---|---|
| 机载 -> 决策 | `mission_interface/perception_candidates` | `PerceptionCandidateArray` |
| 决策 -> 机载（权威入口） | `mission_interface/submit_task` | `SubmitTaskAssignment` |
| 决策 -> 机载（区域任务） | `mission_interface/submit_area_task` | `SubmitAreaTaskAssignment` |
| 机载 -> 决策 | `mission_interface/task_status` | `TaskExecutionStatus` |
| 机载 -> 决策 | `mission_interface/heartbeat` | `UavDecisionHeartbeat` |
| 重连/分页查询 | `mission_interface/get_perception_snapshot`、`get_task_statuses` | 查询服务 |
| 证据查询 | `mission_interface/get_evidence` | `GetEvidenceCrop` |
| 桥 -> 本机执行器 | `mission_interface/executor_assignments`、`executor_area_assignments` | 点/区域任务 |
| 本机执行器 -> 桥 | `mission_interface/executor_heartbeat`、`executor_status` | 能力、对账与状态 |

`bridge_role=scout` 只创建候选/证据/快照上行接口，明确不创建 `submit_task`、
`submit_area_task` 或执行器接口；`bridge_role=worker` 只创建任务提交、状态和本地执行器
接口，不订阅感知候选。`combined` 仅为旧单机部署保留。规划器必须向
`/worker_N/mission_interface/submit_task` 下发任务，不能向侦察机命名空间下发。
提交服务是默认且唯一的权威命令入口；topic 命令入口默认关闭，只能在可靠网关部署中
显式启用。

桥接器在任务写入 SQLite/WAL 后才投递给执行器。重启或执行器 session 变化后先进入
`RECOVERING`，用执行器 heartbeat 的 `active_tasks` 与本地记录对账；完成前拒绝新建和
更新任务。数据库同时保存分配序号、UUID/内容摘要、鉴权 nonce、感知来源序号、候选和
完整状态日志。`get_task_statuses` 使用 `since_status_sequence/limit` 分页，不再受 512 条
内存历史限制。候选全量快照按来源独立更新，旧 `array_sequence/event_sequence` 不会覆盖
新状态，也不会清空其他传感器缓存。

任务必须携带 heartbeat 一致的 `mission_id/uav_id/session_uuid`、协议版本、严格递增的
决策序号、截止时间以及结构化 `GeoReference`。后者包含坐标类型/EPSG、ENU 原点经纬高、
高程基准、map UUID、变换版本和有效期，避免不同无人机同名 `/map` 被误认为同一原点。
桥还会检查候选为 `CONFIRMED/UPDATED`、语义一致、位置/多边形可靠，任务类型/载荷/速度/
高度/容差/并发数在执行器能力范围内，以及 geofence ID 与版本完全一致。救援和物资投放
必须指定执行器已声明可用的载荷；`UPDATE/CANCEL` 只允许原 decision session 操作。

默认要求 HMAC-SHA256：密钥 ID 与 `decision_id` 固定绑定，nonce 持久去重，密钥文件必须
仅 owner 可读写。签名算法与可直接复用的发送脚本见
[`sar_yolo_detector/README.md`](../sar_yolo_detector/README.md) 和
[`sign_and_submit_task.py`](../sar_yolo_detector/examples/sign_and_submit_task.py)。
`operator_authorized` 只是被签名的策略声明，不再被当成身份认证。

机载部署必须显式提供统一地理参考和决策密钥；默认占位值会让 heartbeat 保持 fail-closed：

```bash
chmod 600 /secure/decision_primary.key
roslaunch sar_yolo_detector rescue_profile.launch \
  mission_id:=rescue_20260819 uav_id:=uav_01 profile:=floodnet \
  localization_mode:=terrain terrain_grid_topic:=terrain/grid \
  geo_reference_validated:=true geo_map_uuid:=rescue_map_20260819 \
  geo_origin_latitude_deg:=31.2304 geo_origin_longitude_deg:=121.4737 \
  geo_origin_altitude_m:=5.0 geo_vertical_datum:=WGS84_ELLIPSOID \
  geo_transform_version:=survey_v3 \
  authorization_key_id:=decision_primary \
  authorization_key_file:=/secure/decision_primary.key \
  authorized_decision_id:=decision_primary
```

本地执行器必须周期发布带自身 session、飞控/定位/电池状态、协议、geofence 版本、并发容量、
支持任务/载荷和权威 `active_tasks` 的 `ExecutorHeartbeat`；仅有 ROS subscriber 连接不会使桥
进入 `READY`。

`DISPATCHED_TO_EXECUTOR` 只表示网络输入已通过机载校验并投递给执行器，不表示飞机已经
接受任务。若执行器未在时限内回复 `ACCEPTED_BY_EXECUTOR`，任务进入
`EXECUTOR_TIMEOUT`。后续状态只能按 `ACCEPTED -> QUEUED -> EXECUTING -> 终态` 单调推进，
终态不可恢复，执行器状态序号必须递增。候选过期会发布非终态 `EVIDENCE_STALE` 事件，
由决策策略选择继续、复查、悬停或取消。具体 MRS/PX4 动作、避障、地理围栏、电量和载荷
联锁仍由本机执行器独立复核；ROS 连接数不再被当成执行器可用性依据。

同一 ROS master 的可信局域网/VPN 可直接跨电脑通信，但两端必须正确设置可互访的
`ROS_MASTER_URI` 与 `ROS_IP/ROS_HOSTNAME`。若使用独立 master 或不稳定无线链路，应把
这组冻结的 v1 消息接入带 mTLS 和设备认证的 DDS/MQTT/ROS 网关，而不是把 ROS master
暴露到公网。接口包包含 MD5 冻结测试；原两个接口包的消息/服务现已归属
`sar_yolo_detector/msg` 和 `sar_yolo_detector/srv`，下游规划器只需将类型导入改为
`sar_yolo_detector.*`。接口字段保持 v1 合同，但 ROS 类型名变化后应让通信双方同时升级；
不兼容修改必须新建 v2 类型，而不能直接改变 v1。

## 独立移植清单

```bash
source /opt/ros/noetic/setup.bash
cd /path/to/catkin_ws
cp -a /path/to/sar_yolo_detector src/
rosdep install --from-paths src/sar_yolo_detector --ignore-src -r -y
catkin build sar_yolo_detector
source devel/setup.bash
```

侦察机主要入口是 `launch/scout_perception.launch`；FloodNet 入口是
`launch/rescue_profile.launch profile:=floodnet`。接口、模型、配置、训练工具和示例
脚本都在本目录内。若没有 CUDA/TensorRT，可用
`catkin build sar_yolo_detector --cmake-args -DSAR_YOLO_ENABLE_TENSORRT=OFF` 完成
CPU/接口构建；GPU 部署再按目标机架构重新生成 TensorRT engine。

## 已验证的 COCO→Tracker 冒烟链

下列启动文件以 C++ 静态图像回放器、COCO YOLOv3-tiny、标准
`vision_msgs/Detection2DArray` 和现有 Tracker 验证完整链路。它不依赖 `pod`，
也不会向 Follower/飞控写入任何命令。为保持 Tracker 的帧容忍与演示输入一致，
回放默认 10 Hz。

```bash
roslaunch sar_yolo_detector coco_tracker_demo.launch \
  model_path:=/absolute/path/yolov3-tiny.weights \
  network_config_path:=/absolute/path/yolov3-tiny.cfg \
  image_path:=/absolute/path/person.jpg \
  config:=$(rospack find sar_yolo_detector)/config/coco_person_darknet.yaml
```

已验证输出的关键约束是：检测数组和每个检测框继承相机的采集时间戳；Tracker 对
推理延迟帧在 OOSM 历史窗口内以 `FUSED_OOSM` 进行回放融合，间帧为预测状态。

## 接入当前 Tracker

当前工作区的 Tracker 提供可选 `vision_msgs/Detection2DArray` 输入。联合启动：

```bash
roslaunch sar_yolo_detector tracker_integration.launch \
  model_path:=/absolute/path/rescue_yolo.onnx \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  frame_width:=640 frame_height:=480
```

普通 `Detection2DArray` 不携带稳定 ID；任务节点也可订阅
`TrackedDetection2DArray`，优先使用 Tracker 的稳定 ID。没有该适配输出时才由任务
节点的速度/协方差关联维持局部轨迹。

## 接入 XD 检测定位层

`sar_vision_to_xd_bridge` 将本包的标准 `vision_msgs/Detection2DArray` 或带稳定 ID 的
`TrackedDetection2DArray` 转为 `xd_uav_track/DetectionArray`，用于接入
`xd_uav_detect`，不需要修改后者的订阅或定位代码。桥接器保留图像采集时间戳和相机
frame，将中心点/宽高框转换为 `[x_min, y_min, x_max, y_max]`，并从 CameraInfo
补充原图尺寸。二维检测进入 `xd_uav_detect` 前保持 `range_valid=false`，由地面投影或
雷达相机融合成功后填写三维相对位置。

固定翼 `uav1` 使用包内真实 `YOLO11n COCO` 权重和 SmartTracker，同时启动消息桥
（原有 `xd_uav_detect` 继续独立运行）：

```bash
roslaunch sar_yolo_detector xd_smart_tracker_integration.launch \
  UAV_NAME:=uav1 \
  python_executable:=$PWD/.venv-sar-gpu/bin/python
```

该入口订阅 `/uav1/down_camera/image_raw`，用包内
`models/aircraft_coco/yolo11n.pt` 在 CUDA GPU 上推理。XD 专用覆盖配置
`config/smart_tracker_xd_vehicle.yaml` 只允许
COCO `car(2)`、`motorcycle(3)`、`bus(5)` 和 `truck(7)`，置信度门限为 0.45，
非车辆框在绘制和发布前都会被丢弃。随后节点把
`TrackedDetection2DArray`（包括稳定跟踪 ID）转换成
`/uav1/detect/input/detections_2d`。标注图发布到
`/uav1/sar_yolo_detector/coco/annotated`。这条链路不使用颜色或红色方块阈值。

旧的 `xd_detector_integration.launch` 保留给兼容 OpenCV/TensorRT 的 ONNX/engine
模型；当前 Ubuntu 20.04 自带 OpenCV 4.2 无法加载包内新式 Ultralytics ONNX，
因此本机默认使用上面的 `.pt` + CUDA SmartTracker 入口。启动时禁用静默 CPU
回退，CUDA 环境或显存不足会直接报错，避免误以为正在使用 GPU。

只启动消息桥、复用已经运行的 YOLO 时：

```bash
roslaunch sar_yolo_detector xd_detection_bridge.launch UAV_NAME:=uav1 \
  input_detections_topic:=sar_yolo_detector/detections \
  camera_info_topic:=down_camera/camera_info
```

普通桥接模式默认订阅 `/uav1/sar_yolo_detector/detections`，发布
`/uav1/detect/input/detections_2d`。`xd_uav_detect` 继续使用原有
`fixedwing_detect.yaml`，其输出仍为 `/uav1/track/detections`。使用 rescue profile
时，将 `input_detections_topic` 改为相应 profile 的相对检测话题，例如
`sar_yolo_detector/thermal_uav/detections`。

## ROI 复检与 EO/IR 后融合

`roi_recheck_enabled:=true` 时，Tracker 至多按 `roi_recheck_max_rate_hz`
发布已选目标框，检测器在同一采集时间附近做带上下文的裁剪复检。复检仅确认/恢复
当前全帧目标：全帧已有检测时，不会把不重叠 ROI 结果加入候选集合，因此不会生成
第二个竞争轨迹。默认关闭，且诊断中会报告请求数、实际复检数、复检时延。

EO/IR 使用检测后融合而非强行共享 Tracker 状态：

```bash
# 先在 config/eo_ir_late_fusion.yaml 写入实测的 3x3 IR->EO homography，
# 再人工确认 registration_verified。默认 false 会拒绝启动。
roslaunch sar_yolo_detector eo_ir_late_fusion.launch \
  registration_verified:=true calibration_version:=eo_ir_2026q3
```

融合节点用有界时间队列和全局一对一匹配融合框，发布逐框传感器来源与标定版本；
不同步时退回单传感器框并降低 IR-only 置信度。固定单应仅允许严格共注册/近似共面
场景；存在视差、变焦或云台运动时，应在统一世界坐标下融合。它不做
飞控决策；Tracker 接入时订阅融合输出即可。未经过共视标定时绝不能把 identity
homography 用于真实相机。

## 应急救援配置建议

- EO 和 IR 使用两个节点实例、两套权重和不同命名空间。
- COCO 演示配置默认只发布 `person`；自定义模型必须重新配置 `class_names`。
- `land_rescue_custom.yaml`、`maritime_rescue_custom.yaml` 和
  `ir_rescue.yaml` 的类别顺序已分别固定为 VisDrone、SeaDronesSee v2、HIT-UAV
  的训练合同；包内仍不附带权重。
- 离线转换、训练、验证与 ONNX/engine 导出流程在
  [`training/README.md`](training/README.md)。它不是 ROS 运行时依赖。
- 热红外 profile 原生接受 `MONO16/Y16`，提供百分位/固定温标归一化、坏点校正、
  可选 CLAHE，并在 `ThermalImageInfo` 保留原始编码、温标与 AGC 参数。
- 高空小目标可启用重叠切片和跨切片边缘框合并；应按飞行高度/GSD选择 tile 尺寸并
  重新做精度、吞吐和温升验收。
- 检测结果只能进入 Tracker/任务管理器，不能直接触发飞行、下降或物资投放。
- 端到端验收应记录人员召回率、误报/分钟、P95 推理时延、丢帧率和首次发现时间。
