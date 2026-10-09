# 网页视频、目标坐标和交互接口

校订日期：2026-10-04。本文配套当前 ROS 1 实现，供网页开发和跨主机部署直接接入。

## 1. 已提供的文件

| 文件 | 职责 |
|---|---|
| [web/index.html](../web/index.html) | 视频画面、来源选择、目标位置表、点击/拖拽框选、模式和模型开关 |
| [web/perception_client.js](../web/perception_client.js) | 原生 WebSocket/rosbridge、状态恢复、视频订阅和命令事务 |
| [web/perception_video.js](../web/perception_video.js) | PNG 同帧校验与原图/CSS 坐标转换 |
| [web/perception_ui.js](../web/perception_ui.js) | 可嵌入页面控制器、来源画面和原帧框选 |
| [web/perception_api.json](../web/perception_api.json) | 机器可读的接口名、消息类型、字段约定与端口模板 |
| [msg/PerceptionVideoFrame.msg](../msg/PerceptionVideoFrame.msg) | 原分辨率叠框图与同帧目标数据的原子视频包 |
| [launch/perception_web_app.launch](../launch/perception_web_app.launch) | 一次启动网页静态 HTTP 和 rosbridge，不重复启动算法 |
| [launch/perception_web_bridge.launch](../launch/perception_web_bridge.launch) | 仅启动 WebSocket 桥；视频允许列表可关闭 |
| [scripts/serve_perception_web.py](../scripts/serve_perception_web.py) | 可从源码/devel/install 定位网页资源，提供部署运行配置 |

网页不使用 YOLO 再识别一次来画框或填类别；图像、类别、公开 ID 和位置均来自同一份已裁决目标快照。原来的识别和多目标关联主干未被替换。

### 简易模式 / 模型切换测试页

网页入口启动后访问 `http://127.0.0.1:8080/switch_test.html`。该页面复用正式客户端，不订阅视频、不启动额外检测器；提供连接、三个模式按钮、注册模型下拉框、实际 state 和最多 60 条命令日志。

先连接，再依次测试 detect → track → fusion。模型切换选择已准备 engine 的注册 ID 后点击切换；只有收到 APPLIED 且实际 state 的模式 / active_model_id 与请求一致，才显示 PASS。失败显示错误码，不能把按钮点击当作切换成功。NOT_PREPARED 模型禁用，切换成功也不表示模型精度已验收。真实模型切换必须有共享检测节点和目标机兼容 engine；仅启动网页服务无法验证 GPU 模型切换。

源文件：[switch_test.html](../web/switch_test.html)、[switch_test.js](../web/switch_test.js)。

### 成功切换后的自动初始化

真正切换模式或成功提交新模型后，自动执行软初始化，不需要网页另发 reset：

- 模式管理器清理旧目标快照、历史框选证据、SOT 历史和本次临时轨迹；每来源只保留最新原图用于初始化。延迟旧轨迹受采集时间屏障约束。
- 选中目标的局部 SOT 从可信同帧框重新初始化；缺少可信原帧时进入 lost/stale 等待重捕获，不拿旧框冒充有效锁定。
- 共享检测器在推理批次边界清理等待帧、ROI/像素辅助缓存与短期 ReID 缓存；旧模式正在执行的推理/ReID 结果用内部 generation 拒绝。
- 保留公开实体 ID、长期身份档案和已加载的 GPU 模型。切换业务模式不重载权重、不重启 ROS，也不强制 empty_cache/GC，以免反而增加延迟。
- 模型切换仍先加载预热候选，成功提交后清理旧 epoch / 类别证据并卸载旧模型；失败保留旧模型和跟踪状态。
- 同模式重复点击或相同 request_id 重放不会重复初始化；心跳和普通状态刷新也不会清理正在运行的跟踪。

初始化会使旧 ROI 帧和临时轨迹失效，界面等待新状态/新帧后重新允许框选。这不是“完全清空所有身份再重新编号”。

## 2. 最小启动流程

先重新生成新增消息和启动脚本，在 `/home/promise/mrs_test` 工作区中执行：

```bash
cd /home/promise/mrs_test
source /opt/ros/noetic/setup.bash
catkin build xd_uav_track --no-deps -j 2 -p 1
source devel/setup.bash
export SAR_YOLO_PYTHON="$PWD/.venv-sar-gpu/bin/python"
```

其他主机首次部署需编译完整依赖，不能照搬 --no-deps。新增视频类型不会替代已有 DetectionArray/PerceptionTargetArray，但 rosbridge 与各端必须能加载最新生成的视频消息。

终端 A：已有真实相机图像输入时，先启动固定相机单路算法栈：

```bash
roslaunch xd_uav_track vision_tracking_stack.launch \
  UAV_NAME:=uav1 enable_perception_modes:=true \
  enable_gimbal_lrf:=false enable_sar_gimbal:=false \
  enable_web_video:=true web_video_rate_hz:=6.0
```

终端 B：启动网页入口，不再额外启动 perception_web_bridge：

```bash
roslaunch xd_uav_track perception_web_app.launch \
  UAV_NAME:=uav1 http_port:=8080 websocket_port:=9090
```

浏览器访问 `http://127.0.0.1:8080`，检查命名空间和 WS 地址后连接。网页从 HTTP 的 `/perception_runtime.json` 读取 UAV 和端口配置；普通 python http.server/file 打开方式没有该文件时使用手填/默认值。

这两条命令不创建相机、不会自动起飞、不会打开 Gazebo。如果没有相机或算法输入，页面会明确显示等待视频，而不是伪造画面。

双相机：沿用 [完整指南的纯视觉云台配置](perception_user_deployment_guide.md#43-固定相机--云台当前可用的纯视觉配置)，为 gimbal_detect_config 指定有效文件。当前外部定位主节点不接受默认 gimbal_lrf 方法，不应把该云台配置缺口忽略；纯视觉双路不等于真实 LRF 定位已经接通。

## 3. 必须订阅和发布的接口

`<uav>` 对应 UAV_NAME，例如 uav1。接口文件中的模板前缀为 `/<uav>/perception/`。

| 方向 | 完整名称 | ROS 类型 | 用途 |
|---|---|---|---|
| 订阅 | `/<uav>/perception/video` | `xd_uav_track/PerceptionVideoFrame` | 同帧叠框视频与坐标，默认非锁存 |
| 订阅 | `/<uav>/perception/targets` | `xd_uav_track/PerceptionTargetArray` | 不看视频时也能消费目标快照 |
| 订阅 | `/<uav>/perception/state` | `xd_uav_track/PerceptionState` | 模式、活动模型、选中身份、版本和心跳，锁存 |
| 订阅 | `/<uav>/perception/command_result` | `xd_uav_track/PerceptionCommandResult` | 命令 QUEUED/ACCEPTED/终态 |
| 订阅 | `/<uav>/perception/trajectory` | `xd_uav_track/TargetTrajectoryArray` | 选中身份有界轨迹，锁存 |
| 发布 | `/<uav>/perception/target/select` | `xd_uav_track/SelectPerceptionTargetCommand` | UID/点击/ROI/clear |
| 发布 | `/<uav>/perception/mode/set` | `xd_uav_track/SetPerceptionModeCommand` | detect / track / fusion |
| 发布 | `/<uav>/perception/model/set` | `xd_uav_track/SetDetectorModelCommand` | train / train4 / train7 等已注册 ID |
| 调用服务 | `/<uav>/perception/state/get` | `xd_uav_track/GetPerceptionState` | 连接/重连恢复一致状态 |
| 调用服务 | `/<uav>/perception/models/list` | `sar_yolo_detector/ListDetectorModels` | 可选模型、类别、预构建/验收状态 |

一条 video topic 复用固定和云台两个 source；前端必须按 `frame.targets.image_source` 分流，不能把两路图像和坐标混在一个 latest 对象里。

数值 targets 可用于目录更新，但**视频框和当帧位置表以 video 内嵌 targets 为准**；不要把最新独立 targets 画到较旧视频上。

## 4. 视频消息字段和原像素规则

```text
PerceptionVideoFrame
  header                 # stamp/frame_id 对应采集；顶层 seq 可能是 ROS 发布序号
  frame_revision         # 同来源编码提交的递增修订号
  targets                # 完整 PerceptionTargetArray
    frame_seq            # 原始采集帧号：框选必须使用它
    header.stamp         # 原始采集时间：secs/nsecs 均保留
    image_source
    image_width/height
    session_id/state_version/model_epoch/model_id/mode
    targets[]            # 类别、ID、bbox、center、状态及可选定位
  image                  # sensor_msgs/CompressedImage
    header.stamp         # 同一采集时间
    header.seq           # 保存 targets.frame_seq
    format               # bgr8; png compressed bgr8
    data                 # 无损 PNG；rosbridge JSON 通常编码为 base64 字符串
```

- **不缩放图像尺寸、不重新定义目标像素坐标、不用显示窗口尺寸替代原图。** PNG 中有后端叠绘框/文字，未叠绘区域无损保留，原图整体内容当然会因叠框而局部改变。
- 前端 canvas 的 backing width/height 必须与 PNG 和 targets.image_width/height 一致。允许 CSS 等比缩放以适应屏幕，但原始视频和目标坐标仍保持原分辨率。
- `frame_revision` 在一个 overlay 进程内单调，overlay 重启可以归零。排序先比较 capture_stamp，同一 capture 再比较 revision；新采集帧不能因 revision 变小而被永远拒绝。
- 相同 capture 可以先有 DET，后有类别补充或 LOST 清框。不能按时间戳相等就丢掉所有新版。
- 不以顶层 header.seq 选择目标，rospy 会改写该 seq；使用内嵌 targets.frame_seq 和 targets.header.stamp。
- 图片解码失败、PNG 尺寸与目标尺寸不符、旧会话/旧 epoch/旧模式的数据不显示为当前有效帧。

目标位置以原图左上角为原点，x 右、y 下；中心来自 center_px。position_valid=true 时另显示 position_xyz、position_frame 和真实 position_stamp；默认是相对机体 FRD 米制位置，不是地图坐标。

## 5. 框选、点击与模式/model开关

网页提供：

- detect/track/fusion 开关及实际生效模式，不把点击按钮当成已经成功提交。
- 模型下拉列表及切换按钮；标出未准备/未场景验收模型，活动模型取运行 state。
- 来源选择、原分辨率视频、类别与 ID、框中心像素和观测状态。
- 按 UID 选择、图像点击选择、拖拽 ROI、取消选择和恢复实时画面。
- 选中目标轨迹状态/点数以及命令结果和可读错误。

拖拽流程：

```text
pointerdown：冻结当时显示的 bundle，并保存 source/frame_seq/capture_stamp
    → CSS 坐标映射回原图 → pointerup 得到归一化 ROI
    → 发布 target/select → 检查终态及新 state
    → 恢复实时画面/等待同版本视频
```

命令字段：roi_norm=[left/W,top/H,width/W,height/H]，point_norm=[x/W,y/H]。点击必须落在该原帧已有目标中；任意人工 ROI 可注册 unknown 类别的全局视觉身份，后续经检测证据确认类别。业务模式切换不应重置 ID。

框选冻结只冻结网页显示，**不会冻结服务器相机或无人机运动**。原图缓存默认 15 帧，30 FPS 约 0.5 秒；手工拖动过久或网络延迟过大将返回 STALE_FRAME，而不会把原 ROI 偷套到新位置。可在正式入口显式配置 `perception_frame_cache_size:=30` 或最多 60，先评估内存；两路 1920×1080 BGR、60 帧仅原图缓存就约 712 MiB，不宜无条件全局放大。改善端到端延迟和合理源 FPS 更重要。

模式/模型变化、断线、会话/epoch变化时取消正在拖动的旧帧操作；目标失联不画旧实线假框。state 更新后等待同版本新视频，不能继续把旧选中框当作新状态。

模式、选择、模型三种命令共享 request_id/session_id/expected_state_version/issued_at/ttl_sec。网页按事务串行操作，等待 APPLIED + 对应 state 版本后再允许依赖命令。失败显示 error_code/message。模型切换仍要求目标主机提前准备对应 engine，不在线编译。

## 6. 带宽、负载与运行配置

| 参数位置 | 名称/默认值 | 作用 |
|---|---|---|
| vision_tracking_stack | enable_web_video=true | 新模式下提供视频包；关闭后不编码 |
| vision_tracking_stack | web_video_rate_hz=6 | 每来源视频编码/发布上限，不改变 YOLO/SOT频率 |
| vision_tracking_stack | web_video_max_pixels=16777216 | 单帧像素上限，超限丢帧而非缩小 |
| vision_tracking_stack | web_video_max_encoded_bytes=16777216 | 单帧 PNG 字节上限，超限提示而非伪造尺寸 |
| vision_tracking_stack | perception_frame_cache_size=15 | 原帧选择缓存，最大60，按实际内存配置 |
| perception_web_bridge | enable_video=true | 允许 video topic；数值-only端可 false |
| perception_web_bridge | max_message_size=33554432 | 32 MiB消息上限，容纳有界 PNG 的 base64开销 |
| perception_web_app | http_port=8080、websocket_port=9090 | HTTP与WebSocket端口，均可配置 |
| perception_web_app | address=127.0.0.1 | 默认仅本机；远端走SSH/TLS代理 |

每来源独立编码线程，只保存一个正在编码和一个等待的新帧；没有视频订阅者则不编码。PNG 无损比 JPEG耗带宽，6 Hz 是上限不是 FPS承诺；双路1080p实拍的带宽/CPU还需在目标机测量。可降低 web_video_rate_hz，**不通过降低图像分辨率、抹去ID或取消门控来换负载**。

原生网页客户端的 video/targets 订阅队列长度为 2，来源表默认最多 8（可配置 1～32），避免积累视频或未知来源。未来版本数据只有限缓存；图片解码使用每源单工作任务和最新等待帧，不建立长视频 FIFO。

网页视频工具也在 base64/数组解码前限制单帧 16 MiB，并在创建浏览器图片前限制 16,777,216 像素；这是安全上限，不是性能推荐值。若后台自行提高上限，还必须同步评估并修改前端与桥的限制，不能只改一个参数。

如果只需要网页，不需要桌面合成画面，可设 enable_tracking_visualization=false，仍保持 enable_web_video=true；各路同帧渲染节点继续为网页服务，不启动组合显示。关闭网页并断开 rosbridge订阅后，编码自然停止，但机器人检测/跟踪不因关网页而停止。

## 7. 跨主机网页部署

浏览器里的 localhost 指浏览器主机，不一定是无人机。推荐先用隧道：

```bash
ssh -N -L 8080:127.0.0.1:8080 -L 9090:127.0.0.1:9090 user@uav-host
```

访问本地 HTTP/WS。生产远程入口使用鉴权 HTTPS/WSS代理；允许列表不是身份认证，裸WS和HTTP不要直接暴露公网。感知桥不开放飞行服务、云台电机权限或ROS参数管理。

把 web 目录与 perception_api.json 提供给后续网页开发者即可复用；软件包安装规则包含 web/docs、HTTP脚本、新消息和Python编码模块。重建 install后，web app通过rospkg定位安装资源，无需写死原工作区。

## 8. 验证边界

新增低负载测试入口：

```bash
timeout --signal=INT --kill-after=10s 100s \
  rostest xd_uav_track perception_video_web.test
```

在源码/devel测试环境中执行，测试使用双源320×240合成图、真实C++身份层/manager/overlay和rosbridge，验证PNG原尺寸/原像素、同帧坐标、ROI/点击/UID、模式、LOST清框和模型请求安全拒绝；不启动GPU/Gazebo，不宣称验证真实模型精度或真实飞行。

浏览器客户端另有Node回归，覆盖像素映射、PNG元数据、状态版本、断线和事务；若测试环境无浏览器，应明确“未实际打开浏览器验证GUI”，不能以代码测试代替UI实机演示。

已完成的真实 ROS/HTTP/WebSocket 联调记录见 [网页视频验收记录](perception_web_video_acceptance.md)。它验证接口与传输，不代表真实车辆识别精度、标定或飞行验收。
