# 三项感知模式整改验收记录

日期：2026-10-01。范围：算法调度、原帧选择、人工 ROI 全局身份。
实施顺序：第 1 项通过小测试后开始第 2 项，第 2 项通过后开始第 3 项；最后做联合回归。

## 已实现

| 项目 | 结果 |
|---|---|
| detect/track/fusion 调度 | 共享检测器读取模式状态；track 稳定锁定时减少全图推理，失联恢复搜索；状态过期回退原调度 |
| 已有 ID 的局部跟踪 | 选择成功后初始化真实 SOT；显式算法不可用则失败，不默默替代 |
| fusion 裁决 | 同帧位置/尺度、弱外观、候选歧义门控；冲突标 lost，保留身份，不伪造锁定 |
| ROI/点击原帧合同 | 先查原图和对应历史快照，后匹配框；源、seq、stamp 不符或相机停更即拒绝 |
| 人工身份 | C++ 全局 ID 分配；没有临时 manual 数值 ID 或旁路身份池 |
| YOLO 合并 | 同一真实候选连续三次唯一确认，合并实体及来源映射；错误/重复时间戳不计数 |
| 合并后的迟到选择 | 历史快照同步重映射，旧 ID 别名有数量上限，避免复活退役 ID |
| detect 下身份持续 | 取消选择不删除已注册实体；后台视觉跟踪有数量与 TTL 上限；结果可再次框选/点击 |
| 跨相机 | 已绑定实体继承世界状态、深度特征库及既有跨源拼接；纯图像相似而无世界定位时拒绝跨源合并 |

本次不修改 YOLO 识别权重、不改变飞行跟随开关或控制输出接口。
新模式仍需 `vision_tracking_stack.launch enable_perception_modes:=true` 显式开启，保留现有链路回退。

## 测试结果

- 26 项 Python 单元回归全部通过：实际模式管理器回调、SOT、原帧选择和共享检测器调度。
- 14 项 TrackletStitcher、19 项 MultiTrackManager 回归全部通过。
- 重新编译 C++ tracker 成功。
- `rostest xd_uav_track perception_visual_identity.test` 通过：实际 C++ tracker 与 Python
  模式管理器通信，检查注册 ID、YOLO 合并、切回 detect 后持续显示及再次点击、公开 ID 去重、
  过期帧拒绝、错误会话/重复时间戳拒绝、固定相机→云台世界坐标交接，以及无世界位置的新来源
  不得仅凭相似外观继承身份。

ROS 测试使用合成纹理图像和显式模拟检测观测，不运行 YOLO/GPU、Gazebo 或 PX4。
该记录证明调度合同、身份映射和拒绝路径；不宣称已完成真实视频精度、长遮挡、坦克深度 ReID、
云台剧烈运动或实机控制验收。模型热切换、轨迹话题不在这次三项范围内。

测试设有 60 秒用例限时和 90 秒外层超时；完成后 ROS 测试节点和 master 已退出。

## 复现

```bash
cd /home/promise/mrs_test
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONPATH=/home/promise/mrs_test/src/xd_uav_track/test:/home/promise/mrs_test/src/sar_yolo_detector/test:$PYTHONPATH
python3 -m unittest test_perception_modes test_perception_frame_selection test_sot_tracker test_shared_detector_outputs
devel/lib/xd_uav_track/test_tracklet_stitcher
devel/lib/xd_uav_track/test_multi_track_manager
timeout --signal=INT --kill-after=10s 90s rostest xd_uav_track perception_visual_identity.test
```
