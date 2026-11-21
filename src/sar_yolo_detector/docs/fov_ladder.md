# RGB 固定视场分级验收

在已部署采集插件的 UE5.5 测试工程上使用 PX4 SITL。评测在每档固定视场运行同一组场景。车辆快速场景含同类干扰车，另有快速无人机、低速车辆和顶棚遮挡车辆。每档独立初始化，90°、75°、60°、45° 逐级执行。每档均需通过全部场景，只要一项失败便停止并保存已有结果。

```bash
source /opt/ros/noetic/setup.bash
source /path/to/workspace/install/setup.bash
export PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot
rosrun sar_yolo_detector run_ue_fov_ladder.py --project /path/to/ue/project \
  --python-executable /path/to/dfine/python --output /path/to/new-results
```

每轮在网页提交一次由独立投影日志提供的测试 ROI。运动开始前须验证连续的新鲜锁定。真实相机视场从已交付帧元数据读取。身份、目标框重合、框中心相对目标误差、目标中心相对画面中心偏差分别检查。云台时延从正确观测的采集时间算到 UE 应用对应命令的时间。投影包围框和中心射线是模拟器代理指标。

现行门槛包括可见正确锁定率至少 90%、允许误锁比例至多 1%、目标不可见时有效锁定数为零、同一身份、可见段最长空窗 0.5 秒、框与目标中心的归一化误差 P95 不超过 0.10。分级流程采用归一化画面居中误差 P95 不超过 0.25、正确观测到应用命令 P95 不超过 600 毫秒。遮挡场景须观察到遮挡结束并在 0.5 秒内恢复同一身份。结果见各档 `suite_results.json` 和总表 `ladder_results.json`。前三档均通过才会尝试 45°，范围由当前虚拟镜头限制决定。

## 自动检测和分类

用模型 Python 从包的 `training` 目录执行下列脚本。输入是从未叠加 HUD 或检测框的原始 RGB 视频抽取的、独立标注的 COCO 帧。标注类别使用 `car`、`tank`、`drone`，并保留已审核的背景帧。按源视频或场景划分训练和测试，避免相邻帧泄漏。工具运行 D-FINE-S V3 的单帧检测，输出 COCO AP、小目标 AP、分类和背景混淆以及推理耗时。混淆矩阵使用给定置信阈值和 IoU 0.5，类别无关匹配后记录真实类别与预测类别。

```bash
/path/to/dfine/python /path/to/sar_yolo_detector/training/evaluate_dfine_rgb_frames.py \
  --images /path/to/frames --annotations /path/to/test.json --output /path/to/rgb-score.json
```

红外需要独立的原始红外素材及标注，使用独立报告。RGB 结果和灰度或伪彩演示均不计入红外验收。
