# xd_uav_track 验收矩阵

本矩阵区分单元、回放和真实闭环。前两类可在普通开发机上低负载运行；PX4/Gazebo 每次只运行
一个场景，完成后关闭进程再开始下一项。通过单元测试或 launch 解析不等同于飞行验收。

| 场景 | 配置入口 | 输入/故障注入 | 最低通过条件 | 证据 |
|---|---|---|---|---|
| 多旋翼 + 固定相机 | `tracking_multirotor_fixed.launch` | 5/15/30 FPS，掉帧，乱序帧 | 目标选择稳定；乱序不倒灌；停止时既有安全输出不变 | rosbag、`TrackStatus`、单元测试 |
| 多旋翼 + 云台/LRF | `tracking_multirotor_gimbal_lrf.launch` | 云台锁定/失锁、无回波、质量下降 | `GimbalStatus` 门控时不接受无效 metric range；bearing-only 可按配置退化 | 云台设备状态、检测状态、`TrackStatus` |
| 固定翼 + 固定相机持续可见 | `tracking_fixedwing_fixed.launch` | 静态目标、已标定固定相机 | metric 覆盖率、半径命中率、累计绕飞角达到任务阈值；course-rate 无跳变 | `verify_fixedwing_sitl.py` JSON、PX4/Gazebo 日志 |
| 固定翼 + 固定相机间歇失锁 | 同上 | 稳定锁定后遮挡，至少 90 度、目标最好完整一圈 | `target_visible` 不伪报；`center_hold` 期间世界圆心漂移在任务阈值内；重捕获不反转绕飞方向 | 强制遮挡 rosbag、中心/半径/角度记录 |
| 固定翼 + 云台/LRF | `tracking_fixedwing_dual_sensor.launch`，关闭固定相机 | 异步角度、范围延迟、无效回波 | 时间偏置和外参已启用；激光落点落入目标框才形成 metric；状态质量门控生效 | 标定记录、检测 debug、`GimbalStatus` |
| 固定相机到云台交接 | `tracking_fixedwing_dual_sensor.launch` | 固定相机边缘丢失、云台持续可见 | 不同 image source 图像坐标不互相关联；稳定 ID/米制门控后保留 public track；圆心不突跳 | `TrackStateArray.association_method`、目标中心轨迹 |
| 动态目标 | 任一 metric 配置 | 匀速、加速、转弯、延迟/乱序观测 | `cv` 或可选 `imm` 状态有界；马氏门控拒绝离群；受限 OOSM 不回退控制 epoch；coast 协方差随时间增长 | 回放曲线、世界位置/速度残差、诊断话题 |
| 相似目标交叉/遮挡 | 任一双目标配置 | 类别相同、embedding 相近、交叉遮挡 | 不跨相机混配；选择的 public ID 不无依据改变 | TrackStateArray、关联决策 JSONL |

## 执行顺序

1. `catkin build xd_uav_track --no-deps`，运行 `test_track_controller` 与
   `test_multi_track_manager`；先确认坐标、协方差、乱序、转弯半径、轨道切换和来源隔离。
2. 用 `roslaunch --nodes xd_uav_track <入口>.launch UAV_NAME:=uav_cfgtest` 只解析四个
   组合，确认每路 node 名称和 namespace 独立。
3. 用录制数据逐项启用 `optional_tracking_extensions.yaml` 中的单个功能，对比同一 rosbag
   的 identity switch、FOV loss、控制有效率和计算耗时；没有 A/B 证据时保持关闭。
   无 ROS/Gazebo 的快速回归可直接运行 `replay_tracking_faults.py`，固定随机种子后比较
   5/15/30 FPS、掉帧、延迟和乱序输出；CI 门禁使用 `--strict` 及可见率、半径、绕飞角、
   ID/source switch 和 `center_hold` 漂移阈值，避免没有失锁样本却误判通过。
4. 最后进行单实例 PX4/Gazebo 或实机闭环。固定翼记录 `target_visible` 覆盖率、
   `center_hold` 圆心世界坐标漂移、半径命中率、累计绕飞角、空速与 OFFBOARD 状态。

## 禁止伪通过

- 固定相机的 ground-plane 定位不得借用偏离激光光轴的距离。
- `target_predicted`、coast 和 center hold 不能计入视觉可见覆盖率。
- 只发布速度参考、未实际 OFFBOARD 或未拿到云台控制权均不得算闭环通过。
- 验收报告必须保存配置版本、相机内参与外参、时间偏置、目标真值和原始日志路径。
