# 真实录制数据的身份验收

`evaluate_tracking_recording.py` 是低负载离线工具：它不启动 ROS、PX4、Gazebo
或推理模型。先将 rosbag 中的检测真值、`TrackStateArray` 和可选光照/场景标签导出
为时间排序的 JSONL；每行是一条目标真值在该采集时刻的观测：

```json
{"t": 12.04, "ground_truth_id": "vehicle-03", "track_id": 1000000002,
 "visible": true, "image_source": "fixed_rgb", "scenario": "crossing",
 "illumination": "day"}
```

当人工标注确认目标在画面外或被遮挡时，写入 `visible:false`。重新出现后的第一条
可见记录将用于计算 ReID 重捕获率。不要把检测器漏检错误地当作不可见真值。

```bash
rosrun xd_uav_track evaluate_tracking_recording.py \
  --input annotated_tracking.jsonl --output tracking_acceptance.json --strict \
  --required-scenarios continuous,short_occlusion,long_occlusion,crossing,similar_targets,low_light,backlight
```

默认严格门限为 ID switch 率不高于 2%、误关联率不高于 1%、5 秒内 ReID 重捕获率
不低于 80%。这些是上线前起始门限，不是通用精度承诺；应按任务风险、目标数量和
标注质量调整。

验收数据至少分为 `continuous`、`short_occlusion`、`long_occlusion`、`crossing`、
`similar_targets`、`low_light` 和 `backlight` 场景。固定相机与云台相机的交接必须
额外记录两路 `image_source`、采集时间偏差、标定版本和可用的世界坐标测量质量。
没有可靠的世界位置和时间同步时，系统禁止仅根据相似外观合并跨相机身份。
