# 检测、跟踪、融合模式与 YOLO 模型切换实施方案

日期：2026-10-01。状态：第 1～8 项已实现并逐项回归；真实 GPU 切换和网页重连验收通过。
最新结果见 [阶段验收记录](perception_stage_6_7_8_acceptance.md)，下文保留方案取舍说明。

### 本次三项整改（按顺序实施、逐项测试）

1. **算法调度已接线**：共享检测器订阅 `perception/state`。detect/fusion 维持原来
   的正常全图调度；track 的选中来源默认每秒一次检测锚定，其他来源默认 0.5 FPS。
   未选目标、目标 LOST 或状态消息超过 2.5 秒未更新时恢复原搜索调度。已有 ID 的
   选择同样初始化 SOT；fusion 用同帧运动几何、尺度、弱外观和竞争候选差距校验检测锚点。
   弱 LAB 直方图仅是辅助线索，不等同于坦克专用深度 ReID，也不能证明遮挡后的身份。
2. **选择先检查原帧**：点击和 ROI 必须带来源与 `frame_seq/capture_stamp`，先查原图，
   再查该帧的有界轨迹快照；不能命中最新框后跳过原帧检查。两者同时指定必须一致。
   内部 stamp 配对容差最多 1 ms；只有按 ID 选择允许不指定帧，并使用最新可用快照。
   无检测的有效原帧仍允许初始化人工 ROI。成功选择会从 detect 进入 track。
3. **人工 ROI 使用 C++ 全局身份**：内部 `track/manage_visual_identity` 服务统一分配 ID，
   不再使用保留数值区间或 manual UID。YOLO 合并需同采集时间、真实高置信检测、
   IoU≥0.5、候选差距≥0.15、弱外观一致以及同一候选连续三次确认；漏检、歧义或
   外观冲突打断确认，重复时间戳拒绝。绑定后迁移世界状态、深度特征库和各来源映射，
   继承现有 Tracklet Stitching；无可靠世界位置时仍不允许跨相机仅凭外观合并。

切回 detect/取消选择不会删除已注册实体：最多 8 路后台人工视觉跟踪继续输出证据，
失效时明确标 lost/stale。后台无有效证据的 TTL 默认 60 秒（可配置 1～120 秒）；
C++ 人工身份注册上限 32，观测/绑定历史各最多 32 帧，检测证据历史来源上限 16。
注册服务不可用或容量不足时明确失败并保留原选择，不创建旁路临时身份。
人工图像证据不伪造三维位置、不刷新世界位置时间戳、不自动开启飞行控制。

回归包括实际模式管理器/SOT/原帧选择/共享检测调度单元测试，以及 C++ tracker 与
Python 模式管理器的真实 ROS 集成测试（合成图像、模拟检测观测，无 Gazebo/PX4/GPU）。
这验证接口和身份逻辑，不替代实拍视频的 IDF1/HOTA、转动云台和长遮挡验收。
以上为第 1～3 项的历史实施范围；后续第 6～7 项已补齐轨迹和运行时模型切换。

### 第 4 项：失锁显示与预测期限

- SOT 失败返回 `bbox=None, valid=false`，不把内部保留的最后框当作本帧测量或预测。
- 统一输出的 lost/stale 为身份状态：框、中心点和归一化框清零，三维位置无效；
  内部最后图像框仍可用于恢复校验，不对外冒充当前位置。
- 已有 Kalman 图像预测只允许短时 occluded 输出，默认 `~prediction_horizon_sec=0.5`，
  可配置 0～2 秒；没有可信前序图像观测或超过期限即改为身份状态，不绘制旧框。
- overlay 对有效观测显示 `DET/SOT`；预测显示 `PRED` 虚线及空心中心点；失锁仅在
  状态栏显示 `LOST ID`，不在旧位置画框。相机停更超过 `~source_timeout_sec` 时，
  管理器主动发出一次清除快照，避免界面一直保留此前的实测框。
- 不新增低可信 SOT 预测，不延长失联控制权限，不修改检测模型和身份门控。

### 第 5 项：FRD 位置与真实观测时间

`TrackState` 增加 `last_observation_stamp / position_frame / position_stamp`：图像消息头
继续表示图像采集帧；机体系位置明确使用 FRD（前、右、下），不再冒用相机 optical frame。
四旋翼和固定翼入口均有 `body_frd_frame` 参数，默认 `$(arg UAV_NAME)/base_link_frd`，
分别接入 tracker 的 `frames/body_frd` 和模式管理器的 `~body_frd_frame`。
只改变坐标标签与元数据，不再次翻转已经由 detector 转换过的数值，不改变 FLU 控制输出。

- 图像观测时间只在真实图像匹配时更新；定位时间只在有限、协方差合格且未被跳点门控
  拒绝的真实定位时更新。仅图像检测、预测和失锁均不能刷新最后真实定位时间。
- `PerceptionTarget.position_valid` 要求已确认、非预测、可用于测量的轨迹，可靠测距，
  frame 与配置相符，定位时间与图像采集时间一致且未超过 `~position_timeout_sec`
  （默认 1 秒）。未来时间、缺失时间、非有限位置和非对称/非半正定协方差均拒绝。
- SOT 不创造三维定位：成功的纯图像跟踪输出位置无效且定位时间为零；失锁状态可保留
  最后真实定位的时间/标签以供诊断，但 `position_valid=false`，不得用于控制。
- `last_detection_stamp` 保留最后图像观测时间（来源字段区分 DET/SOT）；`age_ms`
  按当前 ROS 时间与当前帧时间的较大者计算到最后真实图像观测的年龄，包含传输延迟。
  未知或不一致的观测时间输出 `age_ms=-1`，不能伪装成 0。预测期限同样计入传输延迟。

**消息部署注意**：`TrackState` 和嵌套 `TrackStateArray/ManageVisualIdentity` 的 ROS MD5
发生变化，必须同步更新生成消息并重启所有相关发布/订阅端。最终 `PerceptionTarget` 字段
和飞行控制命令字段没有变；旧 TrackState rosbag 不能直接冒充新版本消息，需要兼容转换
或在旧版本隔离环境回放。

### 当前实施范围修订

当前已完成本方案中“优先补齐问题”的第一批实现：

- 正式共享 YOLO 链路新增独立的 `detect/track/fusion` 感知状态机；它不调用旧
  `all/follow` 服务，也不会启停飞行跟随。
- 新增带会话、版本、TTL 和幂等请求号的模式/选择命令、结果、状态和统一目标消息。
- ROI 若未唯一命中已有轨迹，可在命令指定的原始图像帧上初始化
  CSRT/KCF/MIL（不可用时 `auto` 才允许模板回退）；实际算法写入状态。
- 新 overlay 只绘制统一目标快照中的框，并严格配对同源 `frame_seq/stamp`，不再在
  显示端独立修正框。像素中心坐标仍以原图左上角为原点，x 向右、y 向下。
- 新增经 SHA-256 校验的 train/train4/train7 模型注册表和 `~list_models` 服务；
  `~model_status` 同时报告实际 `active_model_id`、源模型摘要、实际加载产物和 epoch。

接口、同帧视频与真实 GPU 事务回归已通过，但实际 Gazebo 跨源身份与时延验收未全通过。
因此 `enable_perception_modes` 暂保持 `false`；联调显式设 `true`，初始 `detect`。
`initial_perception_mode` 可配置启动模式。第 7 项已实现真实 GPU
事务式切换；train/train4/train7 engine 均已预构建，切换失败保留旧模型。

## 1. 结论与边界

在现有生产链上扩展：共享 YOLO 推理、统一目标身份、统一像素观测、统一模式管理。
网页通过类型明确的 ROS 命令话题控制，通过结果话题和状态快照确认实际状态。

三种业务模式为 `detect`、`track`、`fusion`；KCF、CSRT、LK 属于算法配置，不能与业务模式混为同一个枚举。
第一版以视觉目标跟踪和轨迹输出为范围。视觉选择状态与现有飞行跟随开关需要独立，模式命令只改变感知行为。
位置默认提供原图像素框、中心点和归一化坐标；有效三维位置复用现有定位链。

需要明确的工程限制：

- “所有目标”指当前模型支持的类别、可见且达到检测条件的目标，模型不能识别未训练的任意类别。
- 持续 ID 本身就需要跨帧关联，所以检测模式仍保留轻量身份管理。
- 完全遮挡、出画或没有有效图像证据时，不能保证框始终贴合物体。应输出预测/丢失状态，不能把预测标为真实检测。
- KCF/CSRT 输出几何跟踪结果，不输出语义类别。人工框选未识别目标先标 `unknown`，保留 ID，待 YOLO 有效确认后补充类别。
- 三维位置需要标定、时间同步、TF 和测距/有效地面约束；没有这些条件时输出位置无效标志，不能用零坐标表示成功定位。

## 2. 已核查的现状

| 位置 | 已有能力 / 本次核查结论 |
|---|---|
| `launch/vision_tracking_stack.launch` | 正式编排入口；固定相机与云台共享 YOLO；默认 train7 TensorRT FP16；`enable_tracking` 是启动配置，不能充当运行时模式接口 |
| `../sar_yolo_detector/scripts/multi_source_detection_node.py` | 共享推理、最新帧优先、异步 ReID、像素辅助；模型实态、注册表、候选后台加载/预热及批次边界切换，epoch 隔离 |
| `src/multi_track_manager.cpp` | 已有高低置信两阶段关联、全局分配、Kalman、遮挡与外观关联；优先复用 |
| `src/xd_uav_track_node.cpp` | 原服务仍只匹配已有轨迹；新 `perception_mode_manager.py` 在不改变旧接口的前提下补齐任意原帧 ROI 初始化 |
| `srv/SetTargetMode.srv` | 现有 `all/follow` 表示目标选择/跟随语义，不等于三种业务模式；`follow` 当前会激活跟踪控制状态 |
| `python/xd_uav_track/pixel_lock.py` | 已有 LK 前后向检查、局部颜色辅助、框校验及短时外推，无需重复实现基础光流 |
| `scripts/tracking_overlay_node.py` | 作为旧链路兼容保留；新模式使用 `perception_overlay_node.py`，只绘制统一快照，不再独立推进框 |
| `../sar_yolo_detector/python/sar_yolo_detector/manual_box_tracker.py` | 已有 CSRT/KCF/MIL 与模板回退的独立模块，可抽取复用；当前显式算法不可用也可能回退，正式接口应报告实际算法或明确失败 |
| `../sar_yolo_detector/srv/SwitchTrackingMode.srv` | 旧 `smart_tracker/feature_tracker/color_tracker` 语义，不能直接改成新模式而破坏兼容 |
| `../sar_yolo_detector/srv/SwitchSmartTrackerModel.srv` | 旧 SmartTracker 的 `.pt` 切换合同，可借鉴校验和失败处理；不是共享 TensorRT 检测入口的现成接口 |
| `../PixEagle/src/classes/trackers/kcf_kalman_tracker.py` | 已阅读本地 KCF+Kalman 实现和参考文档；核心可借鉴点为候选框校验、实际时间间隔、观测与预测分离 |

已对包内三份 `.pt` 执行 SHA-256 核查：

| 网页 model_id | 文件（相对 sar_yolo_detector） | SHA-256 |
|---|---|---|
| `train` | `models/xd_vehicle_latest/xd_vehicle_latest.pt` | `932c04d47e426143e9fa5084a4ca6a6ff49c1a3aff70fa2b44c643b165e5151f` |
| `train4` | `models/xd_vehicle_train4/xd_vehicle_train4.pt` | `12e26bd00dfe18bd5f2e3cbc579aaad911f12b05239bb3ada3656deacd0d90e5` |
| `train7` | `models/xd_vehicle_train7/xd_vehicle_train7.pt` | `8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54` |

当前 launch 默认 train7；train4 的“回退”历史说明已改为归档说明。
launch 默认值仍不能代表现场实际活动模型，网页必须读取运行时
`active_model_id/source_model_sha256/loaded_artifact_sha256/backend/model_epoch`。
类别表目前文档一致；实施时仍要从实际模型元数据验证类别数量与顺序。

### 第 6 项：有界目标轨迹已实现

新增 `TargetTrajectoryPoint/TargetTrajectoryArray`，发布锁存的 `perception/trajectory`。
track/fusion 只记录当前选中的实体，不对全场所有目标创建额外长期缓存。
默认 60 秒/600 点，10 Hz 采样、2 Hz 发布；参数为 `~trajectory/retention_sec`、
`maximum_points`、`sample_hz`、`publish_hz`、`gap_sec`。缓存不保存图像或 embedding。

每个点有原始采集时间、帧号、来源、像素/归一化中心以及实测/预测标志。三维字段只在
真实有效定位时填充，并明确是 FRD 相对位置而非世界轨迹；未实现世界坐标转换时不伪装
成惯性坐标。丢失不追加零坐标点，恢复、跨源、观测长间隔和取消选择后重新进入都会断段，
消费者按 `segment_id` 绘制，不能跨段连线。切换实体立即清空前一实体的路径；身份合并
仅重映射实体标签，不丢失路径。同帧重复只更新尾点，乱序帧拒绝；inactive 历史也受 TTL。

本机三分钟 30 FPS 合成输入测试：缓存最多 600 点，峰值约 0.44 MiB，追加点 p95
约 0.03 ms、600 点 ROS 二进制序列化 p95 约 2 ms，单次报文约 75 KB。默认 2 Hz
发布的开销足够低；这些是轨迹模块测试，不代表整条 YOLO 推理链的 FPS。

## 3. 三种模式的行为合同

| 行为 | detect 检测 | track 跟踪 | fusion AI 多算法融合 |
|---|---|---|---|
| 全图 YOLO | 周期性运行 | 初始化、类别确认、失联恢复时按需运行 | 周期性运行，持续发现新目标 |
| 目标显示 | 所有有效检测目标及 ID | 选中目标重点显示，其他目标默认不展示 | 所有目标 + 选中目标突出显示 |
| 身份关联 | 轻量跨帧关联 | 保持选中身份，恢复时校验 | 多目标关联 + 选中目标局部校验 |
| 图像帧间补偿 | 有质量门限的 LK 短时补偿 | CSRT/KCF 可选 | 选中目标 SOT + 检测校验，其余目标轻量补偿 |
| 手工 ROI | 可提交并进入 track | 支持已有目标或新 ROI 初始化 | 支持已有目标或新 ROI 初始化 |
| 轨迹输出 | 不默认积累长轨迹；基础身份历史有界 | 选中目标轨迹 | 选中目标默认输出，可配置其他目标 |
| 失联处理 | 标注过期/隐藏实测框，短期保留身份 | 有界预测 → 丢失 → 校验重捕获 | 同左，并保留全图发现能力 |

SOT 指单目标图像跟踪。三种模式都输出类别、ID、位置及观测状态。
默认进入 detect；track 没有选中目标时进入 `WAITING_SELECTION`，不自动随机选目标。
“移动目标”不是必须持续运动的门限：目标停下时仍保留跟踪身份。

切换规则：

1. detect → track/fusion：保留已建立的 ID；有有效选择时初始化单目标跟踪。
2. track ↔ fusion：保持目标 ID，初始化临时缓存并结束旧轨迹段，改变全图发现与校验策略（按后续自动初始化要求更新）。
3. track/fusion → detect：取消视觉选择、结束当前轨迹段，保留尚有效的全场目标身份。
4. 切换算法：用最近有效图像观测初始化新算法，失败保持旧算法；丢失期间拒绝无依据初始化。
5. 切换相机：原图 ROI 不跨相机复用；只有已有跨源关联证据时才继承公共 ID，否则新建源内身份。
6. 模式与模型命令串行提交。切换期间到达的选择若引用过期版本，应明确拒绝并返回最新状态。

## 4. 推荐架构与框贴合策略

```text
图像 / CameraInfo
       ├── 共享 YOLO 检测器 ─────────────┐
       └── 图像跟踪器（LK / CSRT / KCF）─┤
                                       ▼
                            统一观测校验与身份管理
                                       │
                         可选定位结果 ──┤
                                       ▼
                         目标快照 / 轨迹 / 标注图

网页 → 命令话题 → 模式管理器 → 算法调度 / 目标选择 / 模型事务
网页 ← 命令结果 + 状态快照 ←─────────────────────────┘
```

职责分配：

- `sar_yolo_detector` 管模型注册、推理后端、模型切换和纯 YOLO 观测。
- `xd_uav_track` 管模式、单目标图像跟踪、公共 ID、观测裁决和轨迹。
- `xd_uav_detect` 继续提供定位；若新高频视觉观测需要定位，增加明确的适配边界，不能只更新像素框却复用过期三维位置。
- 同一目标同一时间只推进一次身份滤波；检测与 SOT 同帧观测需要合并裁决，防止重复计数。

贴框具体策略：

1. YOLO 为周期性锚点，局部跟踪按可承受的图像频率推进；现有默认检测 4 Hz、标注 6 Hz、组合显示 3 Hz 应分别测量，不能仅靠提高绘制频率解决框滞后。
2. 统一观测保存 `measurement_source`：`detector / visual_tracker / prediction`，同时保存原图时间与最近检测时间。视觉质量和检测置信度使用独立字段。
3. LK 使用前后向一致性、有效特征数和运动门限；KCF/CSRT 检查中心位移、尺度、外观及竞争目标。
4. YOLO 和局部跟踪一致时更新；冲突时进入待确认，禁止无条件平均框或持续学习可疑模板。
5. 短时预测有明确期限；超过期限输出 lost，停止实测轨迹追加。不能用长期静止框伪装锁定。
6. 显示节点只绘制发布的快照；每个框与其对应原图使用相同 `frame_seq/stamp/source`。如需外推，必须由统一观测层生成并标明 prediction。
7. 当前像素辅助可产生合成候选，但 `DetectionCandidate` 没有完整来源字段。采用新观测消息与旧消息适配器，避免合成候选被当成 YOLO 证据反复强化身份。

## 5. ROS 接口草案

以下接口均已有实现；第 8 项补统一串行队列与重连验收。命名空间 `/<uav>/perception/`；首版
mode 对该无人机两路相机统一生效，选择操作必须带图像源。
模型切换作用于共享 YOLO 的所有输入源；独立相机用不同模型需未来增加独立实例。

| 相对接口 | 类型草案 | 用途 |
|---|---|---|
| `mode/set` | `SetPerceptionModeCommand` 话题 | 请求 detect/track/fusion |
| `target/select` | `SelectPerceptionTargetCommand` 话题 | ID、原图点、ROI 三选一；clear 清除选择 |
| `model/set` | `SetDetectorModelCommand` 话题 | 使用注册 model_id 切换模型 |
| `command_result` | `PerceptionCommandResult` 话题 | request_id、阶段、错误码、实际生效版本 |
| `state` | `PerceptionState` 锁存话题 | 当前模式、模型、选择、算法、切换状态及质量统计 |
| `targets` | `PerceptionTargetArray` 话题 | 各图像源完整目标快照；空数组也是有效更新 |
| `trajectory` | `TargetTrajectoryArray` 话题 | 有界轨迹片段与终止/失联标记 |
| `models/list` | `ListDetectorModels` 服务 | 可选模型、能力、类别、后端、预构建状态 |
| `state/get` | `GetPerceptionState` 服务 | 网页重连时获取一致快照 |

公共命令字段：

```text
request_id: string                 # 客户端生成；有界缓存实现幂等重试
session_id: string                 # 节点启动会话，防止重启后旧指令生效
expected_state_version: uint64     # 乐观并发检查
issued_at: time
ttl_sec: float32
```

模式命令附加 `mode`。模型命令附加 `model_id`、`preferred_backend`。
选择命令附加 `selection_type`、`target_uid`、`image_source`、`frame_seq`、`capture_stamp`、`point/roi`。
ROI 统一为原图归一化 `[x,y,width,height]`；输出框统一为像素 `[xmin,ymin,xmax,ymax)` 和归一化 `[cx,cy,w,h]`，字段名区分两种格式。
前端必须去除黑边、缩放、裁剪和拼接偏移；服务端在有限图像缓存中寻找指定帧。找不到时返回 `STALE_FRAME`，不直接用旧 ROI 初始化当前帧。

命令非锁存；入队返回 `QUEUED`，执行检查通过返回 `ACCEPTED`，仅完成一致性提交后返回 `APPLIED`；失败返回 `REJECTED/FAILED`。
客户端超时可用相同 request_id 重试，同 ID 不同参数返回冲突。重连通过状态快照恢复，不能只靠一次结果广播。
错误码包括 `INVALID_MODE`、`UNKNOWN_MODEL`、`STALE_STATE`、`STALE_FRAME`、`TARGET_LOST`、`ALGORITHM_UNAVAILABLE`、`BUSY`、`NOT_PREPARED`、`INSUFFICIENT_MEMORY`、`LOAD_FAILED`。

目标输出字段草案：

```text
header, frame_seq, image_source, image_width, image_height
session_id, state_version, model_epoch, model_id
target_uid, track_id, class_id, class_name, class_valid
bbox_xyxy, center_px, bbox_cxcywh_norm
detection_confidence, tracking_quality, selected
lifecycle_state, measurement_source, last_detection_stamp, age_ms
position_valid, position_frame, position_stamp, position_xyz, covariance
```

`target_uid` 使用会话与公共身份组合，网页按字符串处理；数值 track_id 保留兼容。
模型改变不直接改变 target_uid，但类别必须重新校验，不能沿用旧模型的类别索引解释新结果。
三维坐标必须声明单位和坐标系；现有 body 相对坐标与世界轨迹需要在采集时刻转换。
像素轨迹按源保存；世界轨迹只追加有效定位，断测处分段，不伪造直线连接。
轨迹默认保存选中目标最近 60 秒，最多 600 点，可配置；预测点单独标记，不计入实测点。

消息落点：通用感知/跟踪命令、状态、观测、轨迹放在 `xd_uav_track/msg`；模型目录和模型加载服务放在 `sar_yolo_detector`。
本次实际采用内部 DetectionArray/TrackStateArray 增补 epoch/类别合同字段，以及新增
感知消息/服务；这会改变内部消息 MD5，必须重编译并重启相关订阅节点。
现有飞控 reference / 任务分配输出接口未变，不能把内部消息增补称为旧二进制兼容。
现有模型/跟踪服务只作为兼容入口；新管理器不能直接调用会启用 follow 的旧服务实现纯视觉选择。

## 6. 模型切换实现

新增 `sar_yolo_detector/config/model_registry.yaml`，以 `train/train4/train7` 为稳定 ID。
注册项包含 `.pt` 路径/hash、类别映射、输入尺寸、预处理、检测阈值、engine 路径/hash/来源摘要、GPU/TRT 兼容信息、验证状态。
网页只传 model_id，状态显示活动模型的真实 hash 和后端；`current` 可作为 UI 标记，不作为独立模型文件名。

事务流程：

1. 校验注册项、文件摘要、类别/尺寸、后端兼容性和内存预算。
2. 有足够显存时独立加载候选后端并预热；旧模型继续推理。不能在同一个可变 backend 实例上覆盖加载。
3. 候选验证通过后，在推理批次边界停止旧任务提交，原子替换活动模型并递增 `model_epoch`。
4. 清理旧帧排队、旧异步 ReID 作业、ROI/类别缓存；下游拒绝旧 epoch 的迟到结果。双相机共同切换到新 epoch。
5. 类别合同一致时保留几何身份、将类别标为待确认；类别合同不兼容时结束旧轨迹会话并返回重置原因。
6. 发布完整 state 和 APPLIED，再释放旧后端资源；加载/预热失败则保留旧活动状态。

低显存首版策略：双驻留预检不足时直接拒绝热切换并报告内存原因。
若后续需要“有短暂停顿的切换”，增加独立能力声明：暂停、卸载、加载、失败时重载旧模型。
这种路径有可见中断且回滚也可能失败，不能宣称与双驻留一样无缝。
没有可用 engine 时返回 `NOT_PREPARED`；engine 构建作为独立离线准备步骤，避免网页切换请求触发长时间编译。

## 7. 算法选型

| 算法 / 方法 | 建议用途 | 优先级与限制 |
|---|---|---|
| 现有两阶段关联 + Kalman + ReID | 多目标身份主干 | P0 复用；先验证现状，避免再运行第二套公共 ID 分配器 |
| LK 稀疏光流 | 检测帧间补偿 | P0 复用；特征不足时明确降级 |
| CSRT | 人工 ROI 的首个对照候选 | P1；是否默认启用由目标视频上的准确率与 CPU 开销决定 |
| KCF | 资源紧张时的可选短时跟踪器 | P1；本身没有类别与长期重识别能力，不能单独满足全部需求 |
| 相机运动补偿 GMC | 相机移动下改善关联 | P2；评估背景光流 + RANSAC，失败回退，不能把目标运动当背景运动 |
| ReID 质量门控与类别时序确认 | 减少交叉换 ID、类别跳变 | P1；现有通用外观模型必须实测，不默认认为对当前数据有效 |
| 分割轮廓 | 将来确实需要物体轮廓时 | P3；当前需求是框定位，先不增加分割模型开销 |

融合采用证据门控与状态机：检测提供语义和周期锚点，局部跟踪提供图像运动，Kalman 提供搜索预测，ReID 提供身份辅助。
这些证据存在相关性，不能简单将置信度相乘或把多个框平均后声称更可靠。
先逐项做消融，再决定默认算法组合。

参考来源：

- [PixEagle 本地 KCF 文档](../../PixEagle/docs/trackers/02-reference/kcf-kalman-tracker.md)：短时跟踪、候选校验、预测分离。
- [KCF 原论文](https://arxiv.org/abs/1404.7584)：相关滤波方法依据，不作为本机 FPS 承诺。
- [ByteTrack 官方实现](https://github.com/FoundationVision/ByteTrack)：高低置信检测关联；当前主干已存在类似机制。
- [BoT-SORT 官方实现](https://github.com/NirAharon/BoT-SORT)：运动、外观和相机运动补偿的组合思路；其公开基准不能替代本项目视频验收。

## 8. 分阶段实施与完成条件

| 阶段 | 主要修改位置 | 可验收产物 |
|---|---|---|
| P0 基线与合同 | 两包 docs、模型 registry、现有录制/评估脚本 | 冻结模式与坐标定义；核对运行模型；记录单/双相机延迟、ID 与框贴合基线 |
| P1 模式与选择 | `xd_uav_track/msg`、新 `perception_mode_manager.py`、`xd_uav_track_node.cpp`、CMake/package | 三模式命令/结果/state 闭环；视觉选择独立；ID/已有 ROI 选择；旧入口兼容 |
| P2 统一观测与任意 ROI | 新图像跟踪适配器、`pixel_lock.py`、`multi_source_detection_node.py`、overlay | CSRT/KCF 能力检测；任意 ROI 初始化；统一来源和帧号；显示与数值框一致 |
| P3 生产模型切换 | registry、新模型加载管理模块、共享检测节点、后端适配 | train/train4/train7 切换，预热、epoch、异步隔离、失败保留旧模型 |
| P4 融合与轨迹 | `multi_track_manager.cpp`、新轨迹模块、定位适配 | 全图发现与选中目标并行；有界轨迹；遮挡、交叉、重捕获消融报告 |
| P5 网页合同与回归 | 新命令发布示例、launch、两包 tests/docs | ROS 演示客户端、重连/重复请求测试、完整回放报告及回退配置 |

先在原入口新增 `enable_perception_modes` 配置以便对照和回退；P5 通过后再调整默认入口。
模式系统启动时即建立所需节点，不能依赖运行时改变 launch 的 `if` 条件。
实施前逐文件保存基线：当前工作区与两个包的 git status 都未找到 Git 仓库，不能假定已有版本回退能力。
不预先承诺工期或实时性能；P0 给出硬件、视频、帧率、目标数、显存约束后再估算。

## 9. 验证方案和建议门限

首先建立含逐帧 bbox、身份及遮挡标签的独立验收集，覆盖：静止/移动目标、停走、交叉、尺度变化、部分/完全遮挡、出画重入、相机运动、低照度、多源断流。
三个模型使用同一数据、输入尺寸和评估协议；分别报告每类别召回、误检、IDF1/HOTA、ID 切换、框 IoU、错误重捕获、P50/P95 延迟与 CPU/GPU/显存。
旧训练报告只作背景，不能代替本次一致条件下的模型选择。

建议第一版门限（待 P0 校准；以下不是已测成绩）：

- 正常可见子集的框 IoU≥0.5 覆盖率≥95%，并报告漏检帧；小目标单独报告中心误差/框对角线，避免平均值掩盖问题。
- 相同源、相同帧的 overlay 框与 targets 输出坐标完全一致。
- 确定性模式切换测试中保留目标 ID；新模式 APPLIED 后不发布旧模式结果。
- 在固定验收集上 IDF1/HOTA 不劣于基线；严重交叉/遮挡场景逐条报告错误重捕获。
- 重复 request_id 只执行一次；无效模式、过期 ROI、失效 ID 有明确错误；断线重连能恢复活动状态。
- 三模型轮换 100 次：有效状态与真实后端一致；损坏模型/缺失 engine/预热异常不误报成功；显存经过预热后无持续增长。
- 切换时延分别记录预检、加载、预热、提交及首帧，不把 ACCEPTED 当完成。
- 超期预测不进入实测轨迹；相机重启/时钟回退触发新源时序段；断流时不得重发旧框为新测量。
- 新模式不改变旧飞行跟随启用状态；已有选择/停止/定位与录制测试保持通过。
- 资源允许时以选中目标观测≥15 Hz、端到端 P95≤200 ms 为优化目标；两路负载不足时报告实际值和降级状态，不能以预测帧充当检测 FPS。

复用已有 `test_multi_track_manager.cpp`、`test_pixel_lock_overlay.py`、`test_manual_box_tracker.py`、`test_shared_detector_outputs.py`、`test_multi_source_roi_class_bridge.py` 和视频评估工具。
新增测试集中覆盖状态迁移、ROI 原帧映射、跨 epoch 迟到结果、并发请求、模型失败恢复；最后使用真正 ROS 消息和实际模型做集成回放。

## 10. 本次交付与后续默认取值

当前已补齐三模式调度、原帧选择、人工全局身份、有界轨迹、真实 GPU 模型切换，
正式 launch 默认开关仍保留 A/B 回退。已进行真实 YOLO TensorRT 双源切换及
PT/engine 数值与速度抽检，未启动 Gazebo/PX4，不把接口或小样本测试当作实机精度认证。
后续实施采用：启动 detect、首版单个选中目标、双源共享模型、像素位置必备、有效三维位置可选、网页先接 ROS 合同。
浏览器最终使用 rosbridge 还是独立 WebSocket 网关，可在接入现有网页技术栈时决定，不影响本方案的 ROS 业务合同。
