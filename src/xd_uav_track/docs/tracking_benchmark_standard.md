# 双坦克固定相机回归标准

本标准只针对 `tracking_benchmark.world` 的两辆坦克、固定相机、train7
TensorRT FP16 检测和当前像素辅助跟踪组合；不能把仿真结果当作实机验收。
两座尖顶假山是背景干扰物，不属于目标。相机有效识别区域在
`tracking_benchmark_perception.launch` 中单独配置，不影响其他部署入口。

先在 `/home/promise/mrs_test` 启动地图和感知可视化，再执行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/xd_uav_track/scripts/check_tracking_benchmark_baseline.py \
  --strict --output logs/tracking_mountain_baseline.json
```

门槛由 `config/tracking_benchmark_baseline.json` 固定。脚本等两辆坦克首次
获得确认轨迹后才开始计时，持续至少 45 秒以覆盖完整的约 40.6 秒路线。
它使用 Gazebo 真值和跟踪器的世界位置建立身份配对，之后分别检查：

- 两个目标的确认轨迹覆盖率，以及短时 `occluded/PRED` 时公开 ID 是否仍在；
- 最长双目标更新空窗、ID switch 和额外确认轨迹；
- 地面投影误差上限；交会时的定位偏差不再误算成丢框或新增目标；
- 跟踪更新频率及圆周、转弯路段覆盖；
- 场景确为两辆坦克和两座假山，旧方块、旧树不能混入。

2026-09-29 的完整路线实测为 123/125 帧双目标同时确认、125/125 帧
同时保持确认或明确标记的短时预测、0 次 ID 互换、0 个额外确认轨迹，
最长确认更新空窗 0.992 秒，最大世界位置误差 3.645 米。因此回归下限
设为双目标确认率 98%、持续存活率 100%、确认更新空窗不超过 1 秒、
0 次 ID 互换、0 个额外确认轨迹，以及世界位置误差不超过 8 米。
这是当前低负载仿真基线，不代表实机或其他相机参数下的保证。
固定门槛后的复测报告 `logs/tracking_mountain_baseline.json` 为通过：
45.060 秒内 129/129 帧双目标同时确认、0 次 ID 互换、0 个额外确认轨迹，
最长确认更新空窗 0.507 秒，最大世界位置误差 4.015 米。

后续修改识别、ReID、像素辅助、关联或场景时，应在相同模型、相机位置、
目标路线及负载下重新运行此严格门槛。若失败，先查报告中的
`partial_events`、`id_switch_events`、`unmatched_events` 和
`detector_diagnostics`，不要通过放宽阈值掩盖退步。允许有明确标记的
极短 `PRED`，但不允许把它伪装成实时检测或让两个目标换 ID。
基线还固定了场景、相机/感知入口、路线、假山模型与 train7 TensorRT
engine 的 SHA-256；若有意改变其中任何一项，须重新建立可比场景并重测，
不能直接把旧指标移植到新场景。

三目标视觉观察使用独立入口，在两条原动态路线外于 `(12, 20)` m 增加一辆
固定坦克；它与两条路线的最近中心距至少 16 m。分别启动：

```bash
roslaunch xd_uav_track tracking_benchmark_three_tanks.launch
roslaunch xd_uav_track tracking_benchmark_perception.launch \
  UAV_NAME:=tracking_benchmark inference_backend:=tensorrt_fp16 \
  show_overlay:=true enable_reid:=true pixel_assist_enabled:=true
```

此入口用于观察三目标框和 ID，不替代上面的双目标定量验收。
