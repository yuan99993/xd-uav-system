# 网页接入感知合同

正式栈显式启用，分别在两个已 source 工作区的终端执行：

```bash
roslaunch xd_uav_track vision_tracking_stack.launch UAV_NAME:=uav1 enable_perception_modes:=true enable_gimbal_lrf:=false enable_sar_gimbal:=false
roslaunch xd_uav_track perception_web_app.launch UAV_NAME:=uav1
```

相机与标定话题仍需按真实设备配置。第二条启动网页 HTTP 和 WebSocket 桥，不重复启动检测器或 tracker。
上面的首条以固定相机单路启动，避免当前外部定位节点不支持默认 gimbal_lrf 方法。
双路纯视觉配置、全部字段和跨主机部署见 [完整使用指南](perception_user_deployment_guide.md)。
浏览器打开 `http://127.0.0.1:8080`，连接后显示固定/云台原分辨率叠框视频、类别/ID/中心点、ROI/点击/UID选择、三模式和模型切换。详细字段、端口和网页文件见 [网页视频接口说明](perception_web_video_interface.md)及 [机器可读接口文件](../web/perception_api.json)。没有 CDN 依赖。
默认端口 9090，仅监听本机；远端应通过 SSH 隧道或经鉴权的 TLS 代理访问。
rosbridge 允许列表并不是认证机制，不能直接把裸 ws 暴露到公网。

## 统一接口

均位于 `/<UAV_NAME>/perception/`：

| 名称 | ROS 类型 | 行为 |
|---|---|---|
| `mode/set` | `xd_uav_track/SetPerceptionModeCommand` | detect / track / fusion 算法调度 |
| `target/select` | `xd_uav_track/SelectPerceptionTargetCommand` | clear / track_id / point / roi |
| `model/set` | `xd_uav_track/SetDetectorModelCommand` | 已注册且离线准备好 engine 的模型 |
| `command_result` | `xd_uav_track/PerceptionCommandResult` | QUEUED / ACCEPTED / APPLIED / REJECTED / FAILED |
| `state` | `xd_uav_track/PerceptionState`，锁存 | 实际模式/模型/epoch/公开选择 UID/事务状态 |
| `targets` | `xd_uav_track/PerceptionTargetArray` | 同原图帧的统一框/类别/ID/中心/证据 |
| `video` | `xd_uav_track/PerceptionVideoFrame` | 原尺寸无损 PNG 叠框图与同帧 targets，按 image_source 分流 |
| `trajectory` | `xd_uav_track/TargetTrajectoryArray`，锁存 | 选中实体的有界采集时刻轨迹 |
| `models/list` | `sar_yolo_detector/ListDetectorModels` | 注册文件摘要、类别、engine_prepared、validated |
| `state/get` | `xd_uav_track/GetPerceptionState` | 原子只读运行状态快照 |

`models/list` 由共享检测节点提供；旧 `~list_models` 仍兼容。`state/get` 由模式管理器提供。
models/list 的活动 ID 字段已修正；网页判断实际模式/模型生效仍等待命令 APPLIED 和对应新 state。
ROS schema 有增补字段，客户端和所有订阅 DetectionArray/TrackStateArray 的节点必须
重新生成消息并重启；只替换脚本不能兼容旧 MD5。

## 命令事务与选择

三类命令共用 `request_id/session_id/expected_state_version/issued_at/ttl_sec`。
先读取 state，再提交命令，等待终态及更新的 state 后再提交依赖它的命令。
`QUEUED` 仅表示已入队，`ACCEPTED` 也不代表已经生效。FIFO 默认最多 16 请求，
按 mode/model/select 三类共同串行；容量满返回 BUSY。异步模型准备期间旧模型继续运行。
队列不会把旧版本请求“升级”为新版本；前一个命令提交后，下一个旧版本命令将返回
STALE_STATE。可通过 `active_request_id/queued_commands/transaction_state` 查询进度。

客户端断线重连先恢复 state/list 和订阅。旧请求应原样重发，包含原 request_id、
版本、采集帧；服务器终态缓存最多 128 条，缓存内重发不会重复执行。已离开缓存的
请求不可无限保证去重，应查当前状态并让操作者重新决定。服务器 session 变化时，
不能把旧请求自动改成新 session。TTL 必须有限且位于 0～120 秒；0 表示普通命令不设
客户端期限，但模型准备仍受 28 秒墙钟限制，模式管理器等待上限 30 秒。

按 UID 选择：`selection_type=track_id`，`target_uid=当前 session:global_id`，
`track_id=-1`，指定 `image_source`。UID 可经身份合并别名解析；若额外指定正数 ID
且不一致则 TARGET_CONFLICT。点击/ROI 必须携带实际显示原帧的来源、frame_seq、
capture_stamp；两者同时提供必须一致。归一化 ROI 为 `[x,y,width,height]`，
点击为 `[x,y]`，以原始画面左上角为原点。不能以拼接窗口、缩放窗口的像素直接提交。
按 ID/UID 选择可以不指定帧，仍要求来源存在新鲜图像与对应轨迹快照。

三维为独立时间戳的 body FRD 相对位置，不是相机 frame 或世界轨迹。
position_valid=false、LOST/stale、预测期限届满的目标不能用于实测控制。

视频中 targets.frame_seq/header.stamp 是原始采集引用；最外层 header.seq 是发布序号，不能用于框选。网页 canvas 保持原图宽高，CSS 可缩放显示；拖拽从实际显示原帧冻结后转换回原图，超时返回 STALE_FRAME，不套到最新帧。PNG 编码按来源独立 latest-only，默认每源最多 6 Hz，无订阅者不编码。

## 模型与安全失败

仅支持注册 ID：train/train4/train7，不接受客户端文件路径。候选校验摘要、显存、
真实类别表、预热后在批次边界提交，双源 epoch 一起更新。NOT_PREPARED 不会触发
在线 engine 编译；INSUFFICIENT_MEMORY 不会先卸载旧模型。失败保留旧模型。
切换后保留几何/公开身份，旧类别暂变未知，重新确认后显示；不沿用旧类别合同。
所有模型 validated=false 仍表示没有实景验收，尤其旧 train 的误检风险仍存在。

## 有界验收

```bash
rostest xd_uav_track perception_visual_identity.test
timeout --signal=INT --kill-after=10s 120s rostest sar_yolo_detector model_switch_gpu.test
timeout --signal=INT --kill-after=10s 120s rostest xd_uav_track perception_web_contract.test
```

后两项显式使用本机 GPU，不加入常规自动冒烟测试。WebSocket 测试使用独立 ROS
master / 127.0.0.1:24177，验证实际连接、断线重连、原请求去重、原帧 ROI、UID、
轨迹、标准服务、模型切换与冲突串行化。全部有总体期限并自动关闭本次子进程。
不代表 Gazebo 飞行、实拍 IDF1/HOTA 或三维标定通过。
