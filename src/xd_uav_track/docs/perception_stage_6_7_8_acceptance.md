# 感知接口阶段 6 / 7 / 8 验收记录

日期：2026-10-01。工作区仅 `/home/promise/mrs_test`。
每项实现与验收完成后才进入下一项；本记录不是实机飞行认证。

## 第 6 项：轨迹输出（完成）

`perception/trajectory` 为锁存的 `TargetTrajectoryArray`，选择对象唯一 UID，
像素坐标、原始采集时间、来源、实测/真实预测、断测与跨源分段明确。
默认 60 秒 / 600 点 / 10 Hz 采样 / 2 Hz 发布；TTL 与数量双限。
LOST 不把旧框当新点；预测点不伪造三维实测。

阶段验收：46 项 Python 回归及真实 ROS 身份/原帧/FRD 合同通过。
3 分钟 30 FPS 输入：append p95 0.028 ms，600 点序列化 p95 1.969 ms，
峰值跟踪内存 0.44 MiB，完整消息 74,474 字节。

## 第 7 项：注册模型异步切换（完成）

公开 `perception/model/set` 使用 `SetDetectorModelCommand`，与模式/选择共享
请求 ID、session、状态版本、TTL、结果缓存。准备中拒绝冲突命令 BUSY。
仅允许注册 ID，不接受网页传任意模型路径。

候选后台 SHA 重验、GPU 显存预检、加载、两次预热、实际类别合同核对；
在线不编译 engine。默认候选预算 768 MiB，额外保留 256 MiB，加载后再次
检查安全余量。不足返回 INSUFFICIENT_MEMORY；未准备返回 NOT_PREPARED。
加载失败旧模型/epoch/身份不变。准备有 28 秒上限，前端等待最多 30 秒。
候选只由共享推理线程在完整批次之间提交，清空旧图像、ReID、ROI/像素缓存。
运行中的旧 ReID 任务按 epoch 拒收；两个来源共同递增 epoch。
数值输出携带 epoch / class_names；tracker 保留几何轨迹与公开实体 ID，
清除旧类别与外观证据并重新确认类别。epoch 在同一 ROS master 上重启保持单调。

train、train4 engine 已在本机 RTX 3050 / TensorRT 10.1.0 / CUDA PyTorch 2.6.0
串行构建，原 PT 未修改。train4 真实类别有 m142，已纠正注册表中的遗漏。
三模型的 validated 仍为 false，不宣称已有实景精度认证。

真实双源 CUDA ROS 测试：train7→train4→train→train7，epoch 顺序、真实
类别合同、UNKNOWN_MODEL 失败保留旧模型均通过；切换分别 0.485 / 0.524 /
0.545 秒。fixed / gimbal 延迟 p95 189.01 / 139.43 ms，最大输出间隔
0.536 / 0.506 秒。测试为双源 640×480 空白帧，验证接口/负载，不验证识别召回。
日志：`/root/.ros/log/rostest-LAPTOP-30P34J5H-58791.log`（1 项实际用例通过）。

固定输入 PT vs FP16 数值/速度抽检：train7 的两张已有训练验证拼图，
置信度 ≥0.30 的框按同类别 IoU≥0.90 匹配。train/train4/train7 分别 4/1/11
个框全部匹配，最大置信差 0.00128/0.00256/0.00242；仅小样本数值一致性，
不是 mAP 或实际视频准确率证明。
中位耗时（PT→engine）：train 15.38→9.83 ms；train4 14.75→12.50 ms；
train7 14.72→10.03 ms。脚本 `sar_yolo_detector/scripts/verify_registered_engines.py`。

模型切换与关联：64 项 Python 回归通过，22 项 C++ MultiTrackManager 回归
通过，包含切换后 ID 保留且类别重新确认。

## 第 8 项：网页完整合同（完成）

第 7 项最终 64 项 Python、22 项 C++、真实 ROS 身份及最新 GPU 切换回归通过后，
再开始本项。现有实现：统一 16 请求串行 FIFO；三类命令共用请求 ID/结果缓存；
排队后执行时检查原状态版本，不自动重写版本；模型准备期间队列等待其终态。
网页 UID 为 `session:global_id`，沿用别名表，旧会话 UID 和冲突数值 ID 拒绝。
`perception/models/list` 和 `perception/state/get` 位于约定命名空间。
`web/index.html` 和无 CDN 依赖的 `web/perception_client.js` 支持断线自动重连，
原请求原样重发、终态去重，服务器重启则拒绝旧会话请求。
可选 `perception_web_bridge.launch` 默认仅绑定 127.0.0.1，话题/服务允许列表
不开放飞控和参数接口。网页测试只验证感知合同，不下发飞行指令。

验收通过：原生 JavaScript 客户端去重、重连原请求原样重发、服务端 session
改变后拒绝旧请求；Python 队列测试验证 FIFO、容量、异常恢复、三类事务串行化、
UID 别名与冲突门控。实际 ROS + rosbridge + TensorRT + C++ 全局身份测试
通过，包含断线重连、原帧人工 ROI、同实体 UID、轨迹、标准命名空间服务、
GPU 切换及排队命令版本冲突拒绝。
最新日志：`/root/.ros/log/rostest-LAPTOP-30P34J5H-74933.log`，1 项实际用例通过。
5 次实际只读 WebSocket state/get 往返 p95 6.089 ms（此前一次 16.123 ms）；
判定要求 <250 ms。不是互联网延迟，也不是图像端到端时延。
测试端口 24177 仅用于验收，测试结束自动关闭。

低负载队列回归：1000 次入队/容量拒绝循环必须 <200 ms，容量不超过 16；
已通过。最终三包以 `-j 2 -p 1` 构建成功，72 项 Python 回归、22 项 C++ 关联
回归及 JavaScript 客户端回归通过。真实 ROS 身份回归最终日志：
`/root/.ros/log/rostest-LAPTOP-30P34J5H-71162.log`。最新源码再次通过实际
WebSocket + GPU 联合测试，并检查无本次测试残留进程、24177 端口已释放。
本次未启动 Gazebo/PX4，不把这些结果称为实机认证。
