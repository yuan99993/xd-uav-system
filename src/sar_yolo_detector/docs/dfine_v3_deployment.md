# D-FINE-S V3 完整四模式部署

当前四模式默认检测器为训练完成的 **D-FINE-S V3（16 epoch）**，类别为
`car=0, tank=1, drone=2`。权重不是旧 YOLO，也不是在线下载的 COCO 预训练模型。
`models/dfine_s_v3_16e/model.safetensors` 的 SHA256 为
`d4b408dd28f6e9d570304dce1bf342f8d1590193f05adca03f16dbc6ad1650bd`。
其配置、逐文件校验 manifest、D-FINE 固定版本源码和许可证全部包含在包内。
旧 YOLO 文件仅供历史配置显式选择，四模式主入口不加载它们。

## 环境和编译

已验证的环境为 Linux/WSL2、ROS Noetic、独立 Python 3.10、PyTorch 2.6.0、
torchvision 0.21.0、OpenCV contrib 4.10。ROS 的系统 Python 保持原样，网页和消息节点
继续使用系统 ROS Python；检测/特征/颜色/融合节点使用独立运行环境。CPU/GPU 的
数据流与算法相同，吞吐和延迟取决于显卡、CPU、分辨率、相机帧率和场景，不能保证与
开发机的数值相同。需要可用的 NVIDIA 驱动才能运行 CUDA wheel。

把 `sar_yolo_detector`、`xd_uav_track` 放在同一 catkin 工作区的 `src` 下。
ROS 依赖包括 `cv_bridge diagnostic_updater image_geometry image_transport tf2_ros
tf2_geometry_msgs vision_msgs mavros_msgs`，并安装 `libopencv-dev libssl-dev
libsqlite3-dev libeigen3-dev python3-nose`。例如在已配置 ROS Noetic apt 源的主机：

```bash
sudo apt-get install ros-noetic-cv-bridge ros-noetic-diagnostic-updater \
  ros-noetic-image-geometry ros-noetic-image-transport ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs ros-noetic-vision-msgs ros-noetic-mavros-msgs \
  libopencv-dev libssl-dev libsqlite3-dev libeigen3-dev python3-nose
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="xd_uav_track;sar_yolo_detector" \
  -DCMAKE_BUILD_TYPE=Release -DSAR_YOLO_ENABLE_TENSORRT=OFF
source devel/setup.bash
```

已安装 Python 3.10 后执行（`DFINE_PYTHON` 可指定它的绝对路径）：

```bash
DFINE_PYTHON=python3.10 bash "$(rospack find sar_yolo_detector)/scripts/setup_dfine_v3.sh"
```

安装脚本创建 `$HOME/.local/share/sar_yolo_detector/dfine-v3`，安装匹配的 CUDA 12.4
PyTorch wheel 和所需依赖，最后安装实际提供 KCF/CSRT 的 contrib OpenCV，并检查模型
哈希、原生源码、CUDA 和实际模型加载。它不会安装或自动下载任何新检测权重。
自定义位置时，安装和启动前都设置 `SAR_DFINE_ENV=/your/runtime/path`。
显式 `python_executable:=/your/runtime/bin/python` 优先于自动发现。

无 NVIDIA GPU 时可完整验证 CPU 路径，速度通常较低：

```bash
DFINE_TORCH_INDEX=https://download.pytorch.org/whl/cpu DFINE_DEVICE=cpu \
  DFINE_PYTHON=python3.10 bash "$(rospack find sar_yolo_detector)/scripts/setup_dfine_v3.sh"
# 启动时另加 use_gpu:=false，不会偷偷降级运行。
```

## 启动完整链路和网页

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch xd_uav_track dfine_v3_four_modes.launch UAV_NAME:=uav1
```

默认图像输入为 `/uav1/fixed_camera/image_raw`，CameraInfo 为
`/uav1/fixed_camera/camera_info`，网页为 `http://127.0.0.1:8787`。
输入必须是采集时刻的、单调递增且与 ROS 时钟一致的 `header.stamp`；
CameraInfo 需对应同一相机，TF 应提供 `world` 到图像 `frame_id` 的相机姿态。
相机运动估计同时使用图像背景；缺少/失效 TF 会失去姿态先验，不能当作姿态融合已验证。
图像支持 `bgr8/rgb8/bgra8/rgba8/mono8` 等节点支持的编码。设置实际相机输入，例如：

```bash
roslaunch xd_uav_track dfine_v3_four_modes.launch UAV_NAME:=uav1 \
  input_image_topic:=/camera/image_raw camera_info_topic:=/camera/camera_info \
  camera_motion_fixed_frame:=world image_width:=960 image_height:=540
```

已有 UE/PX4 相机流时使用同一个入口，将话题与固定坐标系改为相机桥实际发布的值。
兼容开发机的 UE 专用入口仍可使用：

```bash
roslaunch sar_yolo_detector ue_dfine_v3_legacy_four_modes.launch UAV_NAME:=ue_test
```

检测默认加载包内 V3，权重和源码均校验通过后才启动，没有包外 `experiments`、
`third_party`、本机用户名或训练工程依赖。UE 引擎、地图资源、相机驱动及 PX4
飞行动力学不属于这两个 ROS 功能包；本入口消费它们的真实相机/位姿输出，不产生
假的飞行控制或虚构目标。当前 XD 配置为图像感知模式，不发送飞行控制参考。

四种模式在同一网页切换。Smart 点击目标选择跟踪；Feature 在实时画面拖框初始化
KCF；Color 的 ROI 学习包含背景对比显著色块处理；Fusion 点击目标或拖框后同时
使用检测关联、特征、颜色和光流证据。拖框/点击携带显示帧的采集时间，过期帧会被拒绝，
需在实时画面重新选择。模式切换不会伪造新的检测或重复消费旧检测。

## 必要算法与可验证数据流

```text
Image + CameraInfo + TF
  → D-FINE-S V3 → 配置 NMS → BoT-SORT → 因果类别一致性
  → Smart 选定目标管理（Kalman/外观/关联）
  → Fusion：KCF/CSRT + ROI 颜色 + LK 光流 + 相机运动补偿
             → 同身份候选门控、融合仲裁、输出去重
  → SAR 原始/融合/选定观测 → vision_to_xd_bridge
  → XD 多目标关联、生命周期、全局身份 → ROS 跟踪输出
  → 网页显示及带时间戳的选定反馈
```

Feature/Color 单独模式按设计不调用检测器；Fusion 必须在选定目标之后才运行选定目标
分支。同一采集帧的异步检测修正不会重复推进 KCF。遮挡恢复、长期身份记忆、异常拒绝
仅在相应条件触发时执行。默认启用原来的必要分支；ROI 背景分类候选未通过收益验证，
它不是生产链路的必要算法，保持关闭。BoT-SORT 使用 Ultralytics 的跟踪实现，检测
网络使用 D-FINE；安装 Ultralytics 不意味着暗中运行 YOLO 检测。

原始检测分数和稳定类别分别保留；Smart/Fusion 与 XD 使用统一的类别契约。
主诊断话题为 `/uav1/sar_yolo_smart_tracker_dfine_v3/pipeline_trace`。
计数仅在实际调用返回后更新；`fusion_feature_update` 专门统计真实局部特征更新。
本机少一个必要阶段时，自检必须失败，不能只凭网页有框就判断链路完整。

## 包内目录职责与 UE/PX4 接入

| 功能 | 包内位置 |
| --- | --- |
| V3 权重、校验清单、解析配置 | `sar_yolo_detector/models/dfine_s_v3_16e/` |
| 固定版本 D-FINE 原生源码/许可证 | `sar_yolo_detector/third_party/D-FINE/` |
| 检测、选定目标、特征/颜色/光流融合 | `sar_yolo_detector/python/sar_yolo_detector/` |
| ROS 适配、网页、安装与验证入口 | `sar_yolo_detector/scripts/`、`web/` |
| UE5.5 原生相机桥、Editor server、帧完整性协议 | `sar_yolo_detector/tools/ue/` |
| UE 场景描述和完整启动组合 | `sar_yolo_detector/config/ue/`、`launch/ue_dfine_v3_live.launch` |
| XD 多目标跟踪、生命周期、全局身份 | `xd_uav_track/src/`、`include/` |
| NED 航线、PX4 ENU 姿态映射、反馈有效性 | `xd_uav_track/python/xd_uav_track/mission.py`、`px4_pose.py` |
| PX4 航线执行和 SITL 入口 | `xd_uav_track/scripts/px4_mission.py`、`launch/px4_ue_sitl.launch` |
| 测试航线 | `xd_uav_track/config/missions/ue_fourmode_route.json` |

在 Windows UE5.5 项目中启用 Python Editor Script Plugin 和 Editor Scripting Utilities。
从 WSL/Linux 指定可访问的项目路径，安装包内相机服务：

```bash
rosrun sar_yolo_detector install_ue_capture.py --project /your/UE/project --install-startup
```

它复制编辑器服务、帧协议、场景生成脚本及 `scene.json`，保留已有启动 Python 内容并备份。
重新打开编辑器后，`Saved/AirGroundExchange/ready.json` 应有新鲜心跳。
地图需要有场景配置中列出的 `AG_` 实体；可在专用测试地图用 UE Python 执行
`runpy.run_path(r"项目路径/Scripts/AirGround/ue_create_scene.py")` 生成。
场景引用的车辆资源需自行导入对应 UE 项目；这里只分发脚本、参数，不分发 UE 引擎或资源包。
缺资源会明确报错，不能把替代几何体的检测表现当作原精细资源的表现。

实际 PX4 模式分三步运行：

```bash
# 终端 1：指定本机 PX4 v1.13.2 源码/已构建 SITL 路径，启动 Gazebo/MAVROS
roslaunch xd_uav_track px4_ue_sitl.launch px4_autopilot_dir:=/your/PX4-Autopilot
# 终端 2：启动 UE 相机桥、包内 V3、四模式、XD 和网页
roslaunch sar_yolo_detector ue_dfine_v3_live.launch ue_project:=/your/UE/project seconds:=240
# 终端 3：在上述专用 SITL 中执行任务；不适用于真实飞行器
rosrun xd_uav_track px4_mission.py \
  --scene "$(rospack find xd_uav_track)/config/missions/ue_fourmode_route.json" \
  --out "$HOME/.ros/px4-route-$(date +%s)"
```

PX4 执行器保留本机仿真标记和 Gazebo 模型校验，实际 MAVROS 姿态反馈用于 UE 相机，
不把理想航线冒充飞行动力学。自定义航线时，同步修改 UE 场景中的 `mission` 和 XD
路线配置，以保证 home/坐标映射一致。没有 PX4 的纯图像调试可显式设置
`pose_source:=kinematic`；它只是运动学相机回放。训练与 UE 仿真分开执行，训练前停止
相机桥和 UE；`SAR_AIRGROUND_WORKLOAD_ROOT` 可配置共享工作负载锁目录。

## 其他机器上的验证

先用包内 UE 静帧做部署烟雾测试。它会发布带时间戳的图像、CameraInfo 和静态 TF，
通过真实网页命令切换四模式并选择目标，确认每个必要阶段有实际调用、Kalman/外观
有效、SAR→XD 存在候选和跟踪结果、两包可比较类别一致。请使用独立测试命名空间，
不要与真实相机同时发布同一个话题。

```bash
# 终端 1
roslaunch xd_uav_track dfine_v3_four_modes.launch UAV_NAME:=deploy_check web_port:=8788
# 终端 2（加载同一工作区环境）
rosrun sar_yolo_detector verify_dfine_pipeline.py --namespace deploy_check \
  --web-url http://127.0.0.1:8788 --output deployment-check.json
```

成功时 JSON 中 `passed=true`，同时保存逐阶段 trace 和网页采集帧证据。
本测试证明模型和算法链路能运行，不证明实际地图上的精度、长时遮挡恢复或 PX4 飞行
安全；这些需要实际任务视频、独立标注以及对应仿真航线验证。

V3 manifest 的部署路径改成包内 URI，故 manifest SHA 与训练目录版本不同；
**模型张量、解析配置及 89 个原生 Python 文件的 SHA 均与训练产物相同**。
历史配置可显式指定旧模型，不参与这个默认入口。
