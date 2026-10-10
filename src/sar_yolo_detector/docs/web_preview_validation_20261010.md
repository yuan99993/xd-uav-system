# 本机融合网页交互验证 · 2026-10-10

范围：正式实验工作区的sar_yolo_detector；未修改xd_uav_track、train7权重或飞行逻辑。
场景：moving_uav_vehicle_search.world；train7.pt / CUDA / FusionTracker。

## 修复

- 新增原帧绑定的桌面交互窗口与轻量本机网页；当前演示只运行网页，不运行rqt/Qt预览。
- TCPROS序列化会重写话题及服务请求顶层header.seq。选择改用精确stamp/frame_id和缓存年龄门禁；
  保留原始seq作审计，不依赖传输序号。不是改成最新帧选择，也不放宽0.75秒时间限制。
- 未选中ID=-1与未确认目标ID=-1相等曾误画绿色；现在只有有效选中ID可以画绿色。
- 最终输出可去掉已验证选中目标的近乎重合同类框；输入候选、歧义检查及实体身份不受去重改变。
- 网页按原尺寸编码，5FPS JPEG预览，单模型复用；24帧header档案、32条元数据、8个HTTP线程、
  选择命令串行，跨进程随机frame_token前缀与时序段切换清理，拒绝旧token/过期帧/越界命令。

## 已完成检查

| 检查 | 结果 |
| --- | --- |
| fusion adapter回归，包括真实ROS序列化与负ID颜色 | 37测试通过 |
| BoxFusion核心回归 | 49测试通过 |
| 帧精确配对、缩放/黑边、缓存上限、选中重复输出 | 6测试通过 |
| 网页命令门禁、并发BUSY、旧token、非法坐标 | 7测试通过 |
| Qt原图缩放点击、冻结原帧拖框、黑边拒绝、右键取消 | 受控Qt事件检查通过 |
| 真实Gazebo → 网页HTTP点击 → 后端selected_id | 选中ID=1，证据流确认 |
| 真实网页HTTP ROI服务及后续状态 | 返回ID=10，后续实际ID=10，compatible_observations |
| 真实取消服务及后续状态 | selected_id=-1确认，测试已清除 |
| 未知frame_token、跨Origin命令 | 分别409、400拒绝 |
| Linux HTML/视频接口 | HTTP 200，持续新frame_token，640×480 |
| Windows IPv4访问 | http://127.0.0.1:8787/ HTTP 200 |
| 本机监听 | 127.0.0.1:8787，不开放0.0.0.0 |
| 单线程install构建、安装脚本与网页一致性 | 通过 |

此前点击及ROI失败已在同一真实链路修复后重新检查，不以早期直接调用函数的测试替代TCPROS联调。
网页浏览器已打开供人工操作；没有自动化浏览器DOM端到端验收。
本记录证明接口与本机交互链路，不证明UE/实拍身份准确率、HOTA/IDF1或完整性能基线。
网页JPEG有损但不缩放，坐标仍使用原图像素。点选202仅表示发送，应以实际选中ID确认。

## 当前运行入口

```bash
roslaunch sar_yolo_detector formal_gazebo_fusion_test.launch \
  gui:=true show_image:=false show_web:=true \
  python_executable:=/home/promise/mrs_test/sar_fusion_experiment.wlpAs3/.venv-fusion/bin/python
```

浏览器：http://127.0.0.1:8787/。左键点击，快速拖框，右键/Esc/取消按钮取消。
演示保持运行，所有会自动选择目标的临时测试已结束；无需额外启动第二个模型。
