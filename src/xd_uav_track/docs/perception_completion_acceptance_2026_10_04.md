# 补齐项与实际联调验收

日期：2026-10-04；工作目录均为 `/home/promise/mrs_test`。

## 结论

观测来源、时间段恢复、TCP 安全绑定与部署依赖已实现并回归。实际共享 TensorRT、
双相机 Gazebo、网页视频、选中目标 CSRT、模型轮换与 install 启动已测试。
**完整身份/性能验收仍未通过，因此正式入口保持新模式默认关闭；联调入口显式开启。**
不声称真实标注数据的 HOTA/IDF1 或实机/LRF 标定已经完成。

## 代码与小回归

- 像素辅助携带 `visual_tracker`，不能创建/确认身份、刷新 YOLO 时间/类别/ReID、制造 metric。
- 分开 `last_observation_stamp` / `last_detection_stamp`、视觉质量 / YOLO 置信度。
- 来源大幅时钟回退清除对应帧、选择快照和模板；旧批次/ReID generation 作废。
  普通乱序、重复和回退后的未来旧帧拒绝；不自动标定设备 uptime 时钟。
- 修复本机 rosbridge listenWS 漏传 interface；实际 HTTP/WS 都只绑定 127.0.0.1。
- 新薄组合包 `xd_uav_perception_bringup` 管三个包的部署依赖，避免反向依赖环。
- 补独立 ReID 包装器/节点安装，engine 验证器用 rospkg 查注册表。
- 多包构建成功；23 项 C++、76 项 Python（最终门禁测试前）、8 项 detector、四组网页 Node 回归通过。
- 实际 ROS 视频/原帧 ROI/点击/UID/LOST/轨迹联调通过；真实 GPU 网页事务及重连通过。
- 新增的延迟定位回归：云台先输出无 metric 的已确认轨迹，定位可用后仍能继承固定相机 ID。
- ROS 离线验收脚本实测 15 秒生成失败报告，不再无限注册。

## 实际模型轮换（install 空间）

[完整记录](/home/promise/mrs_test/logs/perception_installed_gazebo/runtime_100_rotations.json)

- 100 次真实 train7/train4/train 模型轮换：全部 APPLIED，实际模型与 epoch 验证通过。
- 两来源无同帧错误，原图 640×480，无额外 YOLO 进程。
- GPU 总显存采样 876–883 MiB；预热后窗口增量 −1 MiB，低于本次 64 MiB 门限。
- 模型切换后的成功运行恢复 train7。这里只证明事务和显存稳定，旧模型仍可能误检。

## 最终 90 秒 Gazebo/网页联调

本次从 install 运行。GUI 与桌面组合窗口关闭，仅保留相机渲染、共享推理和网页。
修正了开启云台却加载不含云台模型地图的问题。

固定/云台投影改为统一框中心；根据场景实测的车辆参考点误差，将**仿真**位置标准差
设为 1 m，保持 4 m 跨源物理门控、协方差上限和三次确认。2 FPS 云台确认间隔
设为 1.25 s。拼接确认/歧义期间输出 tentative，不显示成已确认临时身份。
这些参数不能替代实机外参/测距/目标几何标定。

[网页与 CSRT 记录](/home/promise/mrs_test/logs/perception_final_pair/web_runtime.json)：

- 模式切换、真实目标选择、CSRT、选中 UID 保持通过。
- 固定 30 帧、云台 48 帧，同帧错误 0，均为 640×480。
- 采集到 WebSocket 接收 P95：固定 232 ms，云台 327 ms，**未达到 200 ms 优化目标**。
- 帧率是低负载配置下的实际结果，不证明 15 Hz 观测目标。

独立世界坐标真值监测（不是标准 bbox HOTA/IDF1）：

| 来源 | 真值匹配次数 | 单源 ID switch | 真值 ID → 输出 ID | 位置误差 P95 |
|---|---:|---:|---|---:|
| 固定 | 413 | 0 | 201→1、202→4、203→5 | 1.87 m |
| 云台 | 55 | 0 | 202→6、203→7 | 2.70 m |

[固定记录](/home/promise/mrs_test/logs/perception_final_pair/fixed_truth.json)
与 [云台记录](/home/promise/mrs_test/logs/perception_final_pair/gimbal_truth.json)。

**同一真值车跨相机 ID 仍不一致，严格身份验收未通过。** 之前一次运行出现过成功
拼接日志，但不同运行不能稳定复现，不能用成功片段掩盖本次失败。
仍需解决视角相关定位/参考点偏差、来源时序下的实体提交与重捕获，再验证唯一身份。
不得进一步放宽空间/歧义门控仅为通过数字。

## 剩余门禁

- 稳定复现跨相机相同实体 ID，并覆盖交接与重入；当前不能认证全量跨源锁定。
- 真实逐帧身份/bbox 数据的 IDF1/HOTA、误关联、长遮挡、交叉、低照度验收。
- 200 ms 时延目标、高帧率和长期资源回归；本次不是 30 分钟稳定性认证。
- 实机云台/LRF 时间同步、外参与协方差标定，不由 Gazebo ground-plane 场景代替。

正式 `enable_perception_modes=false` 保留门禁，显式 true 可继续联调。
本次 ROS/Gazebo/推理/HTTP/WS/监测进程已关闭；没有留下第二份模型进程。
实现与复现说明见 [更新指南](perception_observation_and_regression_update.md)。
