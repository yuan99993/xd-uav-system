# 地面与空中目标的图像视线阶段

第 5 阶段基于真实 RGB 像素、原帧相机内参和采集时刻相机姿态生成世界视线。地面与空中目标都先做图像视线跟随，不由目标框稳定性推断距离或三维位置。

`/<UAV_NAME>/target_geometry`（std_msgs/String JSON）输出 `measurement_stamp`、光学坐标系、`world_frame=ue_render_ned`、单位视线 `ray_world_unit`、相机位置（可用时）、目标 ID/类别、几何策略和有效期。相机位置是视线起点，不是目标位置。过期、预测、未确认、不可信和不匹配原帧的观测不能产生新视线；清除选择会使证据失效。

| 观测类别 | 目标域 | 后续空间定位的必要条件 |
| --- | --- | --- |
| car / tank 等 | ground_candidate | 独立验证目标接地、已知地表、相机标定/姿态及投影不确定性后，才可使用地面约束 |
| drone / aircraft 等 | airborne | 独立测距或经验证的多视角三角化；禁止使用地面交点充当距离 |
| 未分类人工 ROI | unknown | 先确定目标域和测量来源；继续允许纯图像云台跟随 |

类别只是候选语义，不证明目标接地。框中心也不是接地点。当前阶段不启用地面投影，不输出目标空间位置；所有域明确 `range_valid=false`、`target_position_valid=false`、`ground_projection_applied=false`。SAR→XD 桥继续传递二维像素观测，XD 保持 perception_only，不发布目标跟随飞行指令。不能用场景对象真值、预设目标大小或相机高度补成测距。距离、空间位置和飞行跟随须各自完成独立验收后才启用。

## 默认 PX4 飞行

`ue_dfine_v3_live.launch` 默认 `pose_source=px4`，并启动所属的 PX4/Gazebo Classic iris 动力学、MAVROS 和场景任务航线。UE 使用实际估计姿态反馈渲染；视觉云台独立跟随目标。飞行器执行任务航线，不根据未经测距的目标视线执行空间追逐。

```bash
export PX4_AUTOPILOT_DIR=/your/PX4-Autopilot
roslaunch sar_yolo_detector ue_dfine_v3_live.launch \
  ue_project:=/mnt/d/UE/Project/pro1
```

需要构建好的 PX4 v1.13 SITL、Gazebo Classic 插件、ROS Noetic gazebo_ros/mavros/mavlink_sitl_gazebo，以及现有 D-FINE-S V3 环境。所属 SITL 固定 MAVROS `/uav1`；外部已启动的 PX4 使用 `start_px4:=false start_mission:=false`，自行提供匹配的任务状态/坐标原点。反馈缺失或过期会停止采集并报错，不自动切换为运动学。显式 `pose_source:=kinematic` 用于独立视觉调试，默认不启动 SITL 或任务节点。UE 场景地形碰撞并未共享给 Gazebo 的物理世界。

## 验证

`test_target_geometry.py` 检查空/地/未知目标不获得伪距离、向上空中视线、相同像素的控制一致性及原有观测拒绝门限。实际 UE 验证使用 `config/ue/ground_air_acceptance_scene.json`，含运动地面车辆与运动空中无人机：

```bash
rosrun sar_yolo_detector verify_ue_gimbal.py --class-name car --output /tmp/ground_follow
rosrun sar_yolo_detector verify_ue_gimbal.py --class-name drone --output /tmp/air_follow
```

验收脚本选择真实检测类别，不改标签；核验镜头真实转动/居中、持续锁定和制动，并保存本次类别对应的几何证据。类别误检不能因此记为检出准确率通过。

## 真实动力学下的局部视觉连续性

D-FINE 负责发现、分类和重识别；选定目标的图像跟随允许局部视觉独立持续工作。原有检测包光流传播保持不变；新增 `continuous_selected_flow` 对同一已锁定目标，在相邻实际原帧中估计其自身像素运动，不借用地面或远背景的平面运动充当目标运动。前后向 LK、至少 6 个有效点、有效比例≥70%、RANSAC 内点比例≥80%、每帧尺度和边界约束必须通过；当前特征分数还须≥0.85，与独立光流框 IoU≥0.60。

该路径只能延续连续有效的同一身份，不能恢复失锁、重绑定、跨越原有 0.75 秒帧间隔限制或绕过歧义门限。检测时间、类别证据年龄仍保留原值；有效像素观测不等同于新的分类或空间观测。任一证据失效，仍进入原有预测/失锁处理；预测不能驱动云台。

完整入口先等待 MAVROS 参数同步，并在实际 RGB 推理产生新鲜帧后才起飞；模型冷启动时发布坐标原点并停留等待，避免还没准备好就飞完整条航线。默认采集上限 600 秒，任务完成会提前正常结束。悬停时限单独计入阶段超时预算，不会把配置的正常长悬停判为失败。

迟到检测对同一个采集时刻作纠正时，如果检测未通过确认且没有明确的竞争身份证据，保留这个原帧已经通过的视觉观测；不重复运行局部跟踪、不增加观测时间、不续发同一帧的云台控制。弱检测不能把同帧已验证的视觉结果改写成失锁。地面 car/tank 的类别共识纠正仍允许，几何域必须始终为 ground_candidate；与空中目标的域不能互换。

2026-11-15 验收：地面/实际空中目标网页锁定采样均约 98.6%，同一公共 ID 和几何域保持，未输出有效距离。默认 PX4 四巡检点航线完整完成，位置跨度约 86×62 m，采集约 29 FPS。分类 car/tank 切换和 UAV 影子误检仍按实际问题保存。汇总见 [upgrade_acceptance_20261114.json](upgrade_acceptance_20261114.json) 的 stage5。
