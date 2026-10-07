# 网页视频与交互验收记录

校订日期：2026-10-04；实际 ROS 联调日志产生于 2026-10-01。

## 已验证

- tracker 消息与节点构建成功；新增视频类型包含原分辨率 PNG 与同帧目标快照。
- 双来源合成相机输入，通过实际 C++ 身份层、模式管理器、叠框节点和 rosbridge 传输。
- PNG 原始宽高、未叠绘区域像素、图像/目标采集时间及帧引用一致。
- ROI、点击和 UID 选择；错误原帧请求安全拒绝；模式切换保持公开身份。
- 同采集帧从有效观测更新至 LOST 时 revision 增长、旧框清除，三维位置无效。
- 无共享检测器时模型命令返回 UNAVAILABLE，不伪造切换成功。
- 实际 HTTP 提供页面、接口 JSON 和运行配置，命名空间/端口正确，响应不缓存。
- 网页 Node 回归覆盖原尺寸 canvas、CSS 黑边转换、冻结原帧、版本/epoch变化、串行事务、重连、超时和资源上限。

2026-10-04 收尾回归：69 项 Python 测试通过，三组 Node 测试通过；新增网页入口/无桌面合成器 launch 参数检查通过。

真实联调结果：1 项测试，0 errors、0 failures：

- [ROS 联调结果](/home/promise/mrs_test/logs/perception_video_web_retry_tests/xd_uav_track/rosunit-video_web_contract.xml)
- [rostest 总结果](/home/promise/mrs_test/logs/perception_video_web_retry_tests/xd_uav_track/rostest-test_perception_video_web.xml)

首轮测试将“最后采集帧必定出现在视频”作为断流前提，因 latest-only 合并而超时。修正为冻结实际已显示且仍为相机最新的帧后通过；未取消 LOST/同帧检查。另修复了叠框近邻时间配对：已知帧号冲突或采集时间不一致时等待正确原图，不将旧图重标为新帧。

## 验证边界

输入为双源 320×240 合成图与检测数据，无 YOLO/GPU、Gazebo/PX4 或飞行控制。页面测试使用模拟 DOM，未实际打开浏览器完成视觉操作验收。

仍需部署端验证：真实视频及高分辨率双路带宽/CPU、实际浏览器拖拽体验、注册 engine 的 GPU 模型切换、真实标定位置及跨相机身份准确率。不能把本记录作为实机性能认证。

复现入口和全部订阅话题见 [网页视频接口指南](perception_web_video_interface.md)。所有复现测试有超时上限，测试结束会停止本次启动的 ROS/HTTP/WebSocket 节点。
