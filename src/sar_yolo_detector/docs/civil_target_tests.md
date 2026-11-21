# 民用无人机与汽车测试入口

统一入口选择测试场景和运动状态，默认 `model_targets` 允许选择当前模型各类别的候选目标，由操作者点击或框选确认，仍运行 D-FINE-S V3、BoT-SORT、四模式、SAR→XD、网页及云台链路。模型类别为 `car` 和 `drone`；静止/运动是场景设置，不新增“违停车”“超速车”模型类别。人工特征/颜色框选本身不提供分类证据。

| scenario | 主目标 | 配置运动 |
| --- | --- | --- |
| `model_targets` | 当前模型全部支持类别 | 操作者点击或框选确认选定目标 |
| `drone_static` | 民用无人机 | 静止小目标锁定 |
| `car_static` | 路边汽车 | 静止，另有同类汽车及无人机干扰目标 |
| `car_fast` | 汽车 | 往返运动，峰值 20 m/s |
| `drone_fast` | 民用无人机 | 往返运动，峰值 12 m/s |

峰值是仿真轨迹配置，不是从图像测得的速度。入口仅负责识别、锁定、跟踪与视线跟随。

```bash
source /opt/ros/noetic/setup.bash
source /path/to/workspace/devel/setup.bash
export PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot
rosrun sar_yolo_detector run_civil_target_test.py --list
rosrun sar_yolo_detector run_civil_target_test.py --scenario car_fast --describe
rosrun sar_yolo_detector run_civil_target_test.py --scenario car_fast \
  --project /mnt/d/UE/Project/pro1 \
  --python-executable /path/to/workspace/.venv-dfine/bin/python
```

网页 `http://127.0.0.1:8787` 可切换四模式、点击检测框或按采集帧拖框选择目标。默认 PX4、1920×1080、固定 90°、30 FPS 采集请求、原生 JPEG 98 直接进入算法最新帧槽，网页以最多 10 FPS 独立压缩显示。采集 JPEG 在 ROS 传输端直接转发；算法端解码、跟踪计算和网页画框/编码使用相互独立的最新帧槽，不积压旧画面。原始 Image 订阅者仍获得解码后的有效 BGR 图像。汽车使用 Vehicle Variety Pack 的现有 SUV/小汽车资源。

场景初始暂停运动，框选并确认锁定后，可在网页点击“启动目标运动”；“暂停目标运动”用于调试。也可先启动，再手动选定运动目标。按钮只发送场景 start/hold，不提供交通行为判定。

锁定不再按类别白名单过滤，D-FINE 的全部已训练类别均可参与发现；原四模式的手工 ROI 视觉跟踪保留，分类证据仍由识别模型提供。类别变化不会单独阻断身份延续；位置、外观、当前像素和确认门控仍须通过。运动目标入口不启用“目标在世界中静止”的小目标参考约束，移动目标继续使用各分支的当前像素和检测证据。静止无人机入口保留独立的可信锁定验证。

复现完整链路评测时追加以下参数。评测器仅通过普通网页接口给定一次初始 ROI，UE 投影真值只用于独立评分。

```bash
rosrun sar_yolo_detector run_civil_target_test.py --scenario car_fast \
  --project /mnt/d/UE/Project/pro1 \
  --python-executable /path/to/workspace/.venv-dfine/bin/python \
  --benchmark --seconds 60 --output /path/to/new-results
```

默认不增加原始 ROS bag 录像负载，保留状态、分支证据、时序和初末帧；`--record` 单独验证带录像的负载。`--pose-source kinematic` 可隔离视觉性能。每个场景须看自己的实际验收结果，入口存在不表示快速运动已通过验收。

`--feature-tracker-algorithm kcf|csrt|nano` 可以在同一场景切换真实局部跟踪器，其他分支保留。该参数用于候选比较，不能将合成图像或入口预检通过当作实景验收。压缩传输、并行解码和网页工作线程均保留原始采集时间戳；超过 0.5 秒的选定结果不得作为当前控制观测。

候选参数 `experimental_foreground_flow` 的选定目标光流使用 ROI 与周围背景的 HSV 颜色分布差异来限定前景支持，缩小 LK 窗口及金字塔层级以减少路面等背景纹理的影响，使用相机运动与历史目标速度给出搜索初值；无可靠前景或前后向校验失败时返回无效。候选参数 `experimental_motion_hint` 的可靠光流只能调整 KCF 搜索位置，不能单独生成有效控制框，不能更新模型分类、检测时间或目标身份。KCF 当前响应、原始外观参考、分支几何一致性和控制时限仍须通过。此路径的诊断字段为 `selected_target_flow.foreground_only`、`foreground_fraction` 与 `target_motion_search_hint.used`。

候选参数 `experimental_moving_pixel_witness` 对初始边长 6～64 像素的小目标额外保留一次选定时的前景像素参考，在 KCF/CSRT 给出的当前图像位置复核原目标与邻域负样本。它需要 5 个不同的新采集帧确认；模板仅按图像候选位置及尺度重采样，不施加世界静止、距离或地面投影约束，不从空背景更新模板。光流角点不足时，该分支可提供当前像素证据；输出置信度仍不超过 0.65，分类与检测时间保持原证据。匹配器内的小尺度抗混叠过滤不改变输入检测图像或网页分辨率。

`FusionTracker/class_agnostic_lock: true` 同时作用于模型输出白名单和 Fusion 类别兼容门控。检测候选覆盖当前模型的全部标签，操作者确认后才建立选定锁定状态；BoT-SORT、外观比较、当前帧验证及误关联检查保留。测试场景中的 `class` 仅是场景注释；`model_targets --benchmark` 仍只对场景指定的主目标给定 ROI 评分。

后级 XD 同步使用 `tracker/association/class_agnostic: true`，不再因标签不在旧兼容类别表中而断开同一视觉身份。该开关同时覆盖多目标关联、恢复候选连续性、身份拼接及选定身份的类别冲突检查；几何与外观条件保留。

检测候选与操作者锁定分开：未选定时只显示候选框，不向云台输出选定目标控制观测；点击候选或按采集帧框选即表示操作者确认选定。算法之后复核当前像素是否仍可用，失锁时保持原选定身份等待合法恢复，不自动换成其他目标。自动评测工具的一次初始 ROI 是测试输入，等价于操作者的该次选定动作。

上述三个候选参数默认关闭，候选试验结果不用于宣称默认链路已通过快速目标验收。默认局部跟踪器仍为 KCF；NanoTrack 与 CSRT 尚未取代默认配置。
