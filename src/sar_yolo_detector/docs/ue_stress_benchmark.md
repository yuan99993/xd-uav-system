# UE 增补压力测试区

本工具在 `/Game/AirGround/AirGroundRange` 的另存副本
`/Game/AirGround/Benchmarks/AirGroundStressV1` 中添加测试区，保留原场景。
构建前后检查原关卡文件 SHA256；重复构建只替换 `SAR_BENCHMARK_V1` 标签的自有对象。
2026-11-15 在 UE5.5.4 的 pro1 工程实际生成 74 个对象，原文件哈希未改变。
材料、关卡生成脚本和场景参数都随 ROS 功能包安装；车辆使用工程已有 Vehicle Variety Pack。
草地、水面、树木和建筑为可控的程序化 RGB 视觉构件，仍有明显仿真外观差异。

## 构建

UE 启用 Python Editor Script Plugin、Editor Scripting Utilities 和 SarCameraStream。
先打开原关卡或已有测试副本，停止 Play 和图像采集，执行：

```bash
source /opt/ros/noetic/setup.bash
source /path/to/workspace/install/setup.bash
rosrun sar_yolo_detector augment_ue_benchmark.py --project /mnt/d/UE/Project/pro1
```

构建完成后编辑器会加载测试副本。结果在工程 `Scripts/AirGround/benchmark_build_result.json`。
原生产入口继续默认 PX4；本测试入口显式使用 kinematic 观察机隔离视觉性能，不能把其轨迹称为 PX4 动力学验证。

## 单场景网页测试

```bash
roslaunch sar_yolo_detector ue_stress_benchmark.launch \
  ue_project:=/mnt/d/UE/Project/pro1 \
  python_executable:=/path/to/model/venv/bin/python \
  case:=ground_6mps_h60 output:=/tmp/ue_ground6
```

网页 `http://127.0.0.1:8787`。先等待起飞完成、目标静止，选择模式并框选目标，再执行：

```bash
rostopic pub -1 /ue_test/benchmark/command std_msgs/String "data: 'start'"
rostopic pub -1 /ue_test/benchmark/command std_msgs/String "data: 'hold'"
```

`reset` 恢复场景运动时间为零；重复 `start` 不会重置。没有 benchmark 配置的旧场景运动公式保持不变。
`pose_source:=px4` 可联合飞行动力学验证，须按 xd_uav_track 的说明配置 PX4/Gazebo/MAVROS；
当前新增区的量化结果来自 kinematic 隔离测试，未声称这些场景通过了 PX4 联合验收。
视场角固定 90°，不通过动态缩放掩盖小目标问题。

## 自动审计和录像

关闭其他使用同一网页端口/命名空间的测试。下面的工具逐组启动完整原算法链路、正常网页 ROI 初始化、启动目标运动、保存实际 ROS 数据和评分，结束时清理自己启动的进程：

```bash
rosrun sar_yolo_detector run_ue_stress_suite.py \
  --project /mnt/d/UE/Project/pro1 --output /tmp/ue_stress_new_run \
  --python-executable /path/to/model/venv/bin/python
```

输出目录须为空。默认每组运动观测 20 秒、保存 28 秒 RGB bag。
`--cases` 可指定 `config/ue/benchmark_suite.json` 中的场景；
`--pose-source px4` 加入观察无人机动力学，`--master-uri` 可指定独立 ROS master。
模型 Python 需按 requirements-dfine-v3.txt 安装，含 OpenCV contrib；审计/rosbag 工具使用 ROS Noetic 的系统 Python3 和 numpy。

预设覆盖：车辆峰值 2/6/12/20 m/s、无人机 2/6/12 m/s、30/35/60/160m 观察高度、
三档顶棚遮挡和同类干扰目标。预设存在不代表已验证通过，实际执行范围见评估报告。
目标为确定性的往复正弦运动，所标速度是峰值，开始时在端点静止，并非恒速或目标飞行动力学。

评估器只在初始化时使用模拟器投影来模拟一次准确的人工框选。
随后检测器、BoT-SORT、选定目标特征、颜色、光流、融合仲裁和云台只使用真实 RGB/标定/姿态及各自算法证据。
评估文件位于 UE `Saved/AirGroundExchange/BenchmarkTruth/<session>.jsonl`，不进入检测、跟踪或云台控制话题。
真实算法输出、分支计数、gimbal 反馈和 RGB bag 一并保存，可以检查是否执行必要分支。

投影框是世界三维包围盒投影得到的 amodal AABB，遮挡是中心碰撞射线代理，
不是像素掩码、人工标注或标准检测 GT；细枝叶、局部遮挡和几何旋转会造成代理偏差。
结果用于给定初框后的 SOT/视线跟随审计，不能当作检测 mAP。
中心误差按图像半宽/半高归一化。可信当前测量与代理框 IoU≥0.25 且中心误差≤0.18 才计入正确状态；
通过要求为可见状态正确率≥90%、正确测量中心误差P95≤0.10、含首尾的最长正确测量间隔≤0.5s、ID切换0。
遮挡期间不能要求持续产生视觉实测；最长间隔须结合可见性序列解释。
射线未知或无配对状态时标记评估无效，不给通过结论；应先修复评估器并重新运行。
静止阶段已经丢锁时单独报告，不能据此推断目标速度上限。
状态发布频率包含 lost/stale，不能称为有效跟踪帧率；相机、检测、可信测量和网页刷新分别统计。

RGB bag 可沿用 [录像与回放工具](ue_rgb_recording.md)。本测试不产生真实红外能力，也不向空中目标套用地面投影。

录像清单的 validated 表示来源、时间顺序和每个记录帧的K/TF配对校验，不表示无掉帧。应把bag的捕获时间与同次capture的frame_timing.json逐帧对照，单独报告留存率；本次空中压力场景留存率约45.5%，需要继续定位记录通道瓶颈，不宜直接用于宣称完整帧率回放性能。
