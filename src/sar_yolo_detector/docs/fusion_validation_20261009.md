# 最新 PixEagle 四模式融合验证记录

日期：2026-10-09。实验工作区 `/home/promise/mrs_test/sar_fusion_experiment.wlpAs3`。
上游提交 `3db09b001e903f70ca56815b325dba18d40fb756`。

## 修改与兼容性

只修改实验 SAR。两个正式包未修改，实验 XD 的全部上游文件逐字节一致；旧 SAR msg/srv 全部逐字节一致。
新增消息 FusionDetection2D、FusionDetection2DArray，新增 SelectFusionROI 服务，不改旧 ROS1 MD5。
旧 experiment 源码/build/devel 已备份至 `/home/promise/mrs_test/sar_fusion_backup_20261009.CpqpkW`。

模型 train7.pt，SHA-256 `8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54`。
上游检测阈值 0.10 / 四车辆类别保持不变；当前选中身份重获门限 0.50，弱框不能恢复新身份。

## 通过的检查

| 项目 | 结果 |
|---|---|
| 单线程 Release 构建 | 成功，-j1 -l1，C++ TensorRT 后端关闭 |
| install 构建 | 成功，包含新消息、脚本、配置、文档及 archive launch 依赖 |
| 核心、SAR 适配、手动/颜色、编解码、PixEagle 回归 | 60 项通过，GPU Python 环境无跳过 |
| 新融合 C++ 桥接 ROS 测试 | 3 项通过：原像素/来源/ID、非法来源与未来 stamp 拒绝、检测/空数组 |
| 原 vision/tracked 桥接 ROS 测试 | 2 项通过，原来源/身份合同保留 |
| XD / 原 SAR msg/srv 校验 | UNCHANGED |

真实 KCF 测试使用移动纹理目标、间歇无 YOLO 的序列验证 IoU > 0.70，不能等同于坦克实拍精度。
歧义相似目标、错误位置、类别父组、重复时间、失联过期、原帧 ROI/点击、未知 ROI、云台 selected 和缓存上限均有回归。

## 真实模型 ROS 链路探针

输入 `data/sar_yolo_detector/car/1/9.png`，原图 593×469，5 Hz 重复静态帧，选中后采样 8 秒。
启动 SAR 和桥接，不启动 Gazebo、UE、无人机控制或 XD 控制节点。

| 指标 | 记录 |
|---|---:|
| 四模式切换 | 全部成功 |
| 实际模型/设备/局部算法 | train7.pt / cuda / YOLO+kcf |
| 融合帧 | 39 |
| 公共选中 ID | 1，全程不变 |
| 同帧选中 ID 重复框 | 0 |
| 桥接消息 | 42 |
| selected 流有效 | 是 |
| 原宽高、叠框宽高 | 全部一致 |
| 选中后处理 P95 | 71.97 ms |
| 选中后最大处理 | 81.00 ms |

这不是 30 FPS 吞吐验收；实际验证的是 5 Hz 输入下的链路。处理时间不包括网页网络延迟，也不能与其他主机/后端的 TensorRT 指标直接比较。
静态图无法证明转弯、移动相机、遮挡和相似车辆的 IDF1/HOTA 或误关联率。visual_tracker 路径由核心与真实 ROS 桥接分别验证，本次静态真实模型探针的来源均为 detector。

原始记录在工作区 logs：

- fusion_unit_20261009.log
- fusion_bridge_20261009.log、legacy_bridge_20261009.log
- build_20261009.log、build_final_20261009.log、install_20261009.log
- fusion_runtime_20261009.json、fusion_ros_positive_20261009.log

## 保留的失败记录与原因

首轮 CUDA/torchvision 懒初始化约 23 秒，旧探针仅等待 20 秒；它未证明检测或融合成功。
再次使用 car/1/1.png 仍无稳定目标，历史同权重数据也将该帧记录为无检测。
改用有高置信目标的 car/1/9.png 后，首次选择遇到过期冷启动帧，被原帧年龄门控正确拒绝。
探针补充等待新鲜观测（不是放宽服务的原帧保护），预热后成功。
上述失败 JSON 分别保存在 fusion_runtime_cold_start、fusion_runtime_negative_sample、fusion_runtime_expired_cold_frame_20261009.json。

ROS 测试最初错误比较输入/输出 header.seq，ROS1 发布者独立计数导致超时。修正测试为比较采集 stamp/frame_id，未放松生产 ROI 的严格 seq 门控。
输入到叠框的 stamp/frame_id/宽高对应同帧；ROS1 的不同发布者 seq 不保证相同。框选须回传所显示叠框的 header，而不是猜测其他话题的序号。

## 尚未通过的上线条件

UE 同视频/rosbag 的原 YOLO、原特征与融合 A/B：IoU、空窗、误跟背景、ID switch、端到端 P95 与 CPU/GPU预算。
移动相机/云台、车辆旋转与遮挡，以及长期跨相机实体合并、坦克专用深度 ReID，未在本次认证。
当前局部跟踪与 YOLO 同步执行；不能宣称已实现独立高频像素线程或普遍提高识别召回。
测试结束后关闭本次启动的 ROS/YOLO/桥接进程；未启动网页、Gazebo或飞行控制。
