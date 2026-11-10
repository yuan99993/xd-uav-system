# 独立感知部署整理与回归记录

日期：2026-10-07。范围：检测、身份、视觉锁定、选中目标轨迹和失联状态。
不包含任务航线、飞行器动力学、云台电机控制或实机安全认证。

## 本轮交付

- `xd_uav_perception_bringup/perception.launch` 包含一次
  `xd_uav_track/perception_core.launch`，初始 detect，完整三模式/网页合同。
- `config/perception_deployment.yaml` 集中配置原图来源、model/runtime、
  帧率、ReID/ROI、存储、同帧叠框。路径可从源码和 install 解析。
- 默认两路检测直接送身份层，仅输出可靠像素位置；不依赖尚未接通的
  gimbal_lrf 定位器，也不假称已获得三维目标位置。
- `runtime/perception_only` 是启动时锁存的代码边界。旧服务和检测中的
  start_track 不能启动 follower；没有 velocity/command/reference 发布器。
  保留位姿订阅用于已有世界坐标身份门控，不执行制导计算。
- 来源允许列表、数量上限、断流 TTL 清理；全局身份硬上限，优先回收
  无活动映射的非锁定档案，满载时拒绝分配而不是复用活动目标 ID。
- 只读 `track/runtime_diagnostics` 给出来源、身份、映射和拼接缓存数量。
  清理最多 2 Hz，不对每张图像扫描完整档案，映射查找不使用双重遍历。
- 部署预检复用严格 YAML 加载器，拒绝重复键和来源/原图/FRD 不一致。
- 指南第 12.5、13.4、15.6 节同步更新；兼容 flight/Gazebo 入口不替换。

保持 train7 权重、阈值、关联评分、局部跟踪器、ReID 和公开消息不变。
未增加航线或改变现有演示的高度、速度、轨迹。高频像素实验项保持
原正式默认 false，可显式启用，但不能视为严格非回退验收已完成。

## 当前版本验证

日志位于工作区 `logs/`，不是安装包运行时必需品。

| 验证 | 结果 | 记录 |
|---|---|---|
| 感知单元集合 | 91 passed；94 discovered，3 个 ROS 项正确跳过 | perception_portable_all_unit_final_2026_10_07.log |
| 启动图/参数 | 12 passed，包括新命名空间与无飞行/仿真节点 | perception_portable_launch_tests_2026_10_07.log |
| 多目标 C++ | 24 passed | perception_portable_cpp_tests_2026_10_07.log |
| 片段拼接 C++ | 14 passed | perception_portable_stitch_tests_2026_10_07.log |
| 兼容控制器 C++ | 31 passed，原控制算法未变 | perception_portable_controller_tests_2026_10_07.log |
| 真实 C++/Python ROS 身份合同 | passed | perception_portable_identity_ros_final_2026_10_07.log |
| 真实 ROS 存储上限/断流清理 | passed | perception_portable_storage_ros_final_2026_10_07.log |
| 单线程构建 | passed | perception_portable_build_final_2026_10_07.log |
| install-only 路径/预检/启动图 | passed | perception_portable_install_check_2026_10_07.log |

ROS 身份合同覆盖：冲突配置下仍禁止飞行输出；旧服务/消息不能越界；
人工 ROI 接入全局 ID；类别补充、模式切换不重新分配实体；经过连续帧/
世界坐标门控的跨源继承；无可靠定位的相似源不合并；失联后不画旧框，
三维位置失效，选中身份和 LOST 轨迹状态保留；来源容量/允许列表拒绝。

存储合同向真实节点送入 20 个独立候选，身份硬上限配置为 16，观察到
16 个不同公共 ID；观察到已填充存储后再停止输入，验证定时清理把来源、
实体、映射归零，另一允许来源随后可接入，公共 ID 不复用。

第一版存储测试曾错误使用初始零计数作为“已清理”证据，测试失败记录
保留于 `perception_portable_storage_ros_2026_10_07.log`。修正测试为先确认
填充、再等待归零，随后通过；最终代码加维护节流后又重新通过。
直接 unittest 误收 ROS 集成脚本的问题也已修正：未初始化 ROS 时跳过，
实际 rostest 中仍执行，不把跳过计入已通过的 ROS 项。

单元性能抽样：600 点轨迹、30 FPS 输入条件下追加 p95 0.113 ms、序列化
p95 6.286 ms、测试追踪内存峰值 0.44 MiB。只是当前主机单元测试的局部
数据，不是端到端延迟、完整进程 RSS、检测吞吐或准确率认证。

本轮没有再起一套 YOLO/Gazebo；测试只启动轻量且有超时的 ROS 子进程，
结束后已退出，原用户演示保持运行。安装核验只 source ROS+install，
模型和配置解析到 install/share，不偷用 devel 的消息/脚本路径。

## 仍须现场完成

1. 目标主机安装同一 ROS 消息版本和兼容 Python/CUDA/TensorRT 环境，
   显式指定 `SAR_YOLO_PYTHON`；完整 manifest 的其他 XD 依赖也要提供。
2. 提前准备本机 train7 缓存；若需要热切换，离线准备并校验三个注册
   模型的本机 engine/哈希。不能把本机 engine 当跨 GPU 通用文件。
3. 准备实际相机驱动；有三维需求时完成内外参、时间同步、TF、LRF
   落点和质量合同。纯视觉配置不保证跨相机必然获得同一个 ID。
4. 用实际标注视频验证 IDF1/HOTA、ID switch、重捕获、误关联、移动
   相机以及长期资源稳定性。本轮合成输入不能证明这些真实指标。
5. 由外部任务/飞控模块决定航线、速度、高度、重搜索和安全行为。
   session/global UID 属于当前感知会话，不是永久跨重启任务身份。

结论：独立感知入口、代码隔离、配置与安装路径已经整理并回归；具备
跨主机部署的清晰边界。**不据此宣布任意实机、任意遮挡/目标均已上线验收。**

## 关键文件 SHA-256

```text
xd_uav_track_node.cpp       d6bc77e6c50f0a0ced93639e04375caad61c6729f2411dcba4b9a5aaf7ce71c6
perception_core.launch     3740a80799d80ca83ac021dec9cb121ee2ba5931263539719ede0aee3881477c
perception_deployment.yaml bdf7fd9db1070af1602e6cc2127249c56d9de5ff201f5f3af0d659ddd5d47aa9
```
