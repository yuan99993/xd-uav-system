# target_geolocation

由**飞机 GPS + 云台姿态 + 测距仪斜距**解算目标 GPS 坐标的 ROS1 节点，面向
Jetson + PX4/MAVROS + core_sim 仿真链路（`/uav1` 命名空间）。

```
 GPS(lat,lon,alt) ──┐
 云台姿态 #285 ─────┼──> 目标 = 飞机位置 + 斜距 × 光轴单位向量(ENU) ──> NavSatFix
 测距仪 Range  ─────┘                                              └─> RViz Marker
```

## 1. 原理

设飞机位置 `P0`，光轴在 ENU 下的单位向量 `d`，测距仪给出沿光轴的斜距 `L`：

```
P_target = P0 + L · d            (在局部 ENU 切平面内相加，再投回 WGS84)

d = ( cos p·cos y ,  cos p·sin y ,  sin p )
    y = 世界系方位角(0=北,顺时针)   p = 仰角(负=俯视)
```

`d` 来自云台姿态四元数 `q`（机体系 FRD → 世界系 NED）：`d_ned = R(q)·(1,0,0)`，
再换成 ENU。俯角 -90° 时自然得到垂直下视，无需特判。

经纬度换算不是"米/111320"的近似，而是 WGS84 椭球曲率半径（`Δφ=north/M`、
`Δλ=east/(N·cosφ)`，中点纬度迭代），1 km 距离误差 < 1 mm。见
`src/target_geolocation/geodesy.py`。

## 2. 坐标系与符号约定（上机前必读）

| 项 | 约定 |
|---|---|
| `#285` 四元数 | 机体系 FRD(x前 y右 z下) → 世界系 NED(x北 y东 z下)，顺序 `[w,x,y,z]` |
| `YAW_LOCK`(bit 16) | 置位：yaw 已是世界系（相对正北），直接可用；未置位：yaw 相对机头，**必须**叠加 `compass_hdg` |
| 光轴 | 默认机体系 **+X**（镜头/激光朝外那一轴），可用 `~sensor_axis` 改 |
| pitch | 抬头为正。俯视时 pitch < 0，pitch=-90 为垂直下视 |
| yaw / 航向 | 0°=正北，90°=正东，顺时针为正（罗盘习惯） |
| 高度 | 与 mavros `NavSatFix` 一致（AMSL），不做基准换算 |

约定不一致时的现象很典型：方位整体偏 90°、俯仰反向（目标跑到反方向）、
离轴距离随飞机航向变化。**先用第 7 节的一步验证法确认，再谈精度。**

## 3. 目录结构

```
target_geolocation/
├── src/target_geolocation/
│   ├── geodesy.py           WGS84 <-> ENU 换算、ECEF、距离/方位（纯 Python）
│   └── solver.py            四元数 -> 光轴 -> 目标解算核心（与 ROS 解耦）
├── scripts/
│   ├── target_geolocation_node.py   主节点
│   ├── sim_target_sensor.py         仿真用：合成"共轴激光 + 云台指向目标"数据
│   └── check_accuracy.py            与真值比对，打印水平/垂直误差统计
├── launch/target_geolocation.launch
├── config/params.yaml       全部参数及说明
├── live/                    不装 ROS 也能跑的实测工具（纯标准库 TCPROS 客户端）
│   ├── ros1_client.py       极简 ROS1 客户端：发现话题、直连发布者、反序列化
│   ├── locate_gimbal.py     ★ 读 uav1 云台+测距+GPS 算目标 GPS（本仿真用这个）
│   ├── locate_live.py       读 AirSim 机体固连测距 + IMU 算目标 GPS
│   ├── check_against_uav2.py 以 uav2 为真值统计误差 + 拆成斜距/方位/俯仰三项
│   └── probe_gimbal_yaw.py  判决实验：云台 yaw 是世界系还是机体系
└── test/
    ├── test_solver.py       数学单元测试（12 项，不需要 ROS）
    ├── test_node_dryrun.py  节点干跑测试（23 项含欧拉角云台链路，不需要 ROS）
    ├── run_sim_demo.py      逐帧"测量"演示：打印真值/解算/误差
    └── offline_e2e.py       离线端到端精度评估 + 敏感度表
```

## 4. 依赖与编译

```bash
# Jetson (JetPack 5/6 + Ubuntu 20.04 + ROS Noetic)
sudo apt install ros-noetic-mavros ros-noetic-vision-msgs
pip3 install pymavlink                 # 解析 #285 云台姿态需要

mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
cp -r <本包> target_geolocation
cd ~/catkin_ws && catkin_make
source devel/setup.bash
chmod +x src/target_geolocation/scripts/*.py    # 从 Windows 拷过来必须做
```

纯 Python 包，不需要交叉编译；Jetson 上直接 `catkin_make` 即可（无 C++ 编译）。

**先在本机（无需 ROS）验证数学部分：**

```bash
python3 test/test_solver.py        # 12 项数学测试
python3 test/test_node_dryrun.py   # 6 项节点逻辑/闭环测试
python3 test/run_sim_demo.py       # 逐帧测量结果（真值/解算/误差表格）
python3 test/offline_e2e.py        # 精度评估，输出第 8 节那些数字
```

## 5. 在当前仿真系统里跑（推荐先这样验证链路）

本仿真里有云台姿态和飞机 GPS，但通常没有"激光正好打在被跟踪目标上"这种组合，
所以用 `sim_target_sensor.py` 合成传感器：它按几何真值构造云台姿态和斜距，
解算节点再用**反向模型**复原——两边符号约定独立，能真正暴露约定错误。

```bash
# 终端 1：仿真系统（core_sim + PX4 SITL + mavros）按你平常的方式启动
# 终端 2：合成传感器 + 解算节点 + 精度统计
roslaunch target_geolocation target_geolocation.launch sim:=true

# 或者分开跑，方便单独看日志
rosrun target_geolocation sim_target_sensor.py
rosrun target_geolocation target_geolocation_node.py \
    _range_topic:=/sim/range _gimbal_att_topic:=/sim/gimbal/attitude
rosrun target_geolocation check_accuracy.py
```

预期日志：

```
[GEO] 目标 lat=47.3961520 lon=8.5479357 alt=408.0 | 斜距=262.5m 平距=250.0m 方位=135.0° 仰角=-17.7° yaw_lock=True
[ACC] n=20  水平 mean=0.48 p95=1.06 max=1.80 m | 垂直 mean=0.40 max=1.43 m
```

## 5.2 直接对接 uav1 的真实云台（有云台，测距装在云台上）★实测可用

`uav1`（PX4 + mavros）带真实三轴云台，前置测距仪**固连在云台上**，
所以光轴 = 云台姿态，与机体姿态无关——**不需要 IMU**，只需要三样：

| 用途 | 话题 | 类型 | 实测值 |
|---|---|---|---|
| 飞机位置 | `/uav1/mavros/global_position/global` | `sensor_msgs/NavSatFix` | −37.71755, 144.83376, 9.95 m |
| 航向 | `/uav1/mavros/global_position/compass_hdg` | `std_msgs/Float64` | 279.9 ~ 282.5° |
| **云台姿态** | `/uav1/gimbal/attitude` | `std_msgs/Float64MultiArray` | `[yaw, pitch, roll]` deg |
| **测距** | `/Sim/SceneBasicDrone/robots/Drone1/sensors/FrontDistance/distance_sensor` | `sensor_msgs/Range` | 46.4 m |

```bash
roslaunch target_geolocation target_geolocation.launch gimbal:=true
```

`target_geolocation` accepts both `sar_yolo_detector/TrackedDetection2DArray`
and `xd_uav_track/DetectionArray`. The default input is the selected tracker
output consumed by the current gimbal controller:
`/uav1/sar_yolo_detector/smart/selected_tracked_detections`. It selects the
highest-confidence box (optionally filtered by `det_class_id`) and carries
its class and track metadata automatically. When the detection image is aligned
with the gimbal and range sensor, `use_img_offset:=true` also uses the box center
and image dimensions to correct the gimbal ray; otherwise leave that correction off. It
publishes `/uav1/target/global_position` (`sensor_msgs/NavSatFix`) plus
`/uav1/target/geolocated` (`target_geolocation/GeolocatedTarget`). The latter
pairs the same NavSatFix with the selected box's class, confidence, sensor and
track ID. For another gimbal detector, override the topic/type; add a class
filter only when the image can contain unrelated detections:

```bash
roslaunch target_geolocation target_geolocation.launch \
  gimbal:=true \
  det_topic:=/uav1/sar_yolo_detector/smart/selected_tracked_detections \
  det_msg_type:=TrackedDetection2DArray use_img_offset:=false
```

The current `session_one_multi_nvidia.yml` feeds the selected tracker from AirSim
`DownCamera`, so that session starts with `use_img_offset:=false`. For a detector
running on the gimbal camera itself, set it to `true` to apply the bbox-center
angular correction. The box class and tracker identity are forwarded either way.

The range sensor still measures along the gimbal optical axis. The detected
box center supplies the angular offset from that axis; the NavSatFix is the
resulting WGS84 target point. The task allocator uses the richer `GeolocatedTarget`
topic so the selected detection's class and track identity stay paired with it.

或者不装 ROS 直接读（本机就能跑）：

```bash
python3 live/locate_gimbal.py --uri http://192.168.2.99:11311/ --n 3
```

### `/uav1/gimbal/attitude` 的 yaw 是机体系（已用两条独立链路互证）

同一时刻有第二个数据源 `/uav1/mavlink/from` 里的 `#285 GIMBAL_DEVICE_ATTITUDE_STATUS`
（世界系四元数）。实测三帧：

```
航向 compass_hdg   云台yaw(话题)   #285 世界yaw    话题yaw + 航向
   282.470           167.306          89.616          89.776
   279.880           170.206          89.616          90.086
   281.350           168.286          89.616          89.636
```

- `#285` 的世界 yaw **恒定 89.616°**，即使航向在变 → 云台 yaw 是**惯性稳定**的
- `话题yaw` 跟着航向反向变动，但 `话题yaw + 航向` 恒定 → 话题 yaw 是**机体系**
- 两条链路的方位角只差 0.2~0.5°，互相印证

结论写进参数默认值：`att_msg_type:=Float64MultiArray`、`gimbal_yaw_is_body:=true`、
`gimbal_pitch_up_positive:=true`。pitch 两条链路同号为"上正"（−19.85° = 下俯 19.85°）。

> 注意：`#285` 的 `flags=0x7FC0` 并未置 YAW_LOCK(bit4)，按 MAVLink 语义应理解为
> 机体系，但数据表明它实际是世界系。**以数据为准**，本节点用参数显式声明，不依赖 flag。

### 实测解算结果（连续三帧，间隔 1.5 s）

```
  飞机 GPS   : -37.71755140, +144.83375750,  9.934 m AMSL
  云台姿态   : yaw=167.306  pitch=-19.848(上正)  roll=0.000
  距离       : 46.455 m      航向: 282.470°
  => 世界方位 167.306 + 282.470 = 89.776°

  >>> 目标 GPS = -37.71754986, 144.83425309,  -5.838 m (AMSL)
      水平 43.695 m, 垂直 -15.772 m

  第二帧 -37.71755309, 144.83425240, -5.838   (方位 90.086°)
  第三帧 -37.71755090, 144.83425058, -5.778   (方位 89.636°)
```

三帧目标点**重合在 0.4 m 以内**（经纬度极差 3e-6°，约 0.36 m），
而期间航向变了 2.6°、云台 yaw 变了 2.9°——两个量反向抵消，说明
"yaw + 航向"这层组合是对的。若把 yaw 当世界系直接用，目标会偏 **55 m**。

第二天仿真重启后再算（航向已变到 331°，云台 yaw 同步转到 117°）：

```
  航向 331.830  yaw 117.818  距离 47.343  =>  方位 89.648°
  >>> 目标 GPS = -37.71754663, 144.83425616,  -6.121 m
  航向 331.400  yaw 118.578  距离 46.923  =>  方位 89.978°
  >>> 目标 GPS = -37.71754774, 144.83425598,  -5.992 m
  航向 333.550  yaw 116.788  距离 47.370  =>  方位 90.338°
  >>> 目标 GPS = -37.71755516, 144.83426095,  -6.142 m
```

跨重启与前一天的结果只差 0.16 m（纬）/ 0.50 m（经）/ 0.02 m（高）。
航向从 282° 转到 333°，云台 yaw 相应从 167° 转到 117°，两者之和始终锁在 89.6~90.3°。

垂直分量 −15.77 m vs 相对高度 17.27 m：激光打在比起飞点高约 1.5 m 的地面上，
量级自洽（不是打到空中或障碍物）。

## 5.3 精度实测：以 uav2 为靶（uav2 落地静止，可当真值）

靶点就是 uav2，所以直接读 `/uav2/mavros/global_position/global` 当真值：

```bash
python3 live/check_against_uav2.py --uri http://192.168.2.99:11311/ --n 25
```

七轮独立测量（跨两次仿真启动，飞机一直在转，航向 282°→117°→331°）：

| 轮次 | 日期 | 航向 | 水平误差 | 垂直误差 | 三维误差 | Δ斜距 | Δ方位 | Δ俯仰 |
|---|---|---|---|---|---|---|---|---|
| 1 | 10-29 | 282.5° | 1.665 m | 1.275 m | 2.097 m | −1.974 m | −0.37° | +0.74° |
| 2 | 10-29 | 138.8° | 1.284 m | 1.238 m | 1.784 m | −1.630 m | +0.10° | +0.84° |
| 3 | 10-29 | 117.0° | 1.267 m | 1.372 m | 1.867 m | −1.652 m | −0.21° | +1.01° |
| 4 | 10-29 | — | 1.575 m | 1.325 m | 2.059 m | −1.911 m | −0.34° | +0.83° |
| 5 | 10-30 | 331.3° | 1.385 m | 1.496 m | 2.039 m | −1.788 m | −0.37° | +1.11° |
| 6 | 10-30 | — | 1.412 m | 1.505 m | 2.064 m | −1.842 m | +0.14° | +1.10° |
| 7 | 10-30 | — | 1.394 m | 1.724 m | 2.217 m | −1.882 m | −0.36° | +1.37° |
| **范围** | | | **1.27~1.58 m** | **1.24~1.72 m** | **1.78~2.22 m** | **−1.63~−1.97 m** | **−0.37~+0.14°** | **+0.74~+1.37°** |

单轮内 11 帧的 std 只有 **2~3 cm**，说明误差是**系统性偏差**而非噪声，完全可复现。

**跨仿真重启重复性**：10-29 与 10-30 两次重启后，同一靶点的解算结果只差
0.16 m（纬）/ 0.50 m（经）/ 0.02 m（高）。如果存在旋转、符号或坐标系的错误，
这种量级的重复性不可能出现。

### 判定：算法准不准 —— 把误差按视线方向拆开

单纯看"到 uav2 的直线距离约 2 m"会误判，因为它把两件不同的事混在一起。
把水平误差投影到**沿视线**和**垂直视线**两个方向：

| 轮次 | 实测斜距 | 径向（沿视线） | 横向（垂直视线） | 横向折合角度 |
|---|---|---|---|---|
| 1 | 47.84 m | **−1.634 m** | 0.304 m | 0.37° |
| 2 | 47.88 m | **−1.275 m** | 0.077 m | 0.10° |
| 3 | 47.45 m | **−1.295 m** | 0.170 m | 0.21° |
| 4 | 47.59 m | **−1.542 m** | 0.271 m | 0.34° |
| 5 | 47.07 m | **−1.352 m** | 0.288 m | 0.35° |
| 6 | 47.09 m | **−1.405 m** | 0.106 m | 0.13° |
| 7 | 47.02 m | **−1.365 m** | 0.272 m | 0.33° |

- **横向只有 0.08~0.30 m，折合 0.1~0.4°** → 指向（云台 yaw + 航向这个组合）是准的。
  若有任何旋转/符号/坐标系的错误，横向会先崩到几十米，不可能这么小。
- **径向 1.28~1.63 m，且七轮全部为负（都打近了）** → 误差不是"指错方向"，
  而是"打的距离偏短"。

### 那 1.3~1.6 m 的径向偏短是什么

是激光的物理行为：**测距值 = 沿视线到第一个表面的距离**，不是到靶标几何中心的距离。
uav2 是个有体积的机体，射线在距离 ~49 m 处本该穿过它的中心，实际在 ~47 m 就被
它的外壳挡住了 —— 差的这 1.5 m 就是机体半径。

垂直误差也能对上账：

```
垂直误差 1.24~1.72 m
  = 径向偏短 1.5 m × sin(20.7°)  ≈ 0.53 m   （打近了，落点自然偏高）
  + uav2 的 GPS 原点在机体中心下方 ≈ 0.75 m
```

后半项有独立佐证：uav2 落地时 `rel_alt = 0.006 m`，说明它的 GPS 原点正好落在
**地面/起落架**那一层，而机体中心必然在这之上。所以"对着 GPS 原点算"和"激光打在
机体表面"本来就该差这么多。

> 注：云台保持固定的世界姿态（俯角恒为 −19.85°、方位恒为 ~90°），并没有跟踪 uav2。
> 飞机慢慢漂移时，真值俯仰会随之变化（−20.6°→−21.2°），而云台不动，所以 Δ俯仰
> 从 +0.74° 飘到 +1.37°。这正说明残差取决于"固定指向的光束"与"有体积靶标"间的
> 几何关系，不是解算公式的问题。

### 独立交叉校验

除了和 uav2 比，还有两条不依赖 uav2 的校验：

| 校验 | 方法 | 结果 |
|---|---|---|
| 测地换算 | `test_solver.py` 里把 ENU→经纬 的结果与子午线弧长数值积分对比 | 误差 < 1 mm |
| 高度基准一致性 | uav1/uav2 的 `global_position/global` 高度差 vs 各自 `local_position/pose.z` 高度差 | 差 **0.105 m** |

第二条的意义：`local_position/pose` 是 PX4 自己从 home 起算的，与 GPS 是两条独立链路。
两者算出的 uav1–uav2 高差只差 10 cm，说明**两台车的 AMSL 基准是一致的**，
所以那 1.5 m 垂直残差不是"基准面不统一"造成的。

（水平方向没法用 pose 互校——PX4 的 local 系是每台车各自从自己的 home 起算的，
两台的 x/y 都在 0 附近，不是共享坐标系。）

### 结论

| 指标 | 数值 | 判断 |
|---|---|---|
| 指向误差（横向） | 0.08~0.30 m ≈ 0.1~0.4° | 准 |
| 沿视线偏短 | 1.28~1.63 m | 激光停在靶标表面，属物理事实 |
| 到 uav2 **GPS 原点**的三维距离 | 1.78~2.22 m | 主要是靶标体积，不是计算误差 |
| 到 uav2 **机体表面**的距离 | ≈ 0 m（射线确实命中机体） | 命中 |
| 单轮内重复性 | 2~3 cm | 无抖动 |
| 跨仿真重启的重复性 | 0.16 / 0.50 / 0.02 m | 无漂移、无帧错位 |

**算法本身没有系统性错误。** 如果目标是地面上的点（靶标、落点、车辆等），
不会有"命中最近表面"这一项，预期误差 ≈ 指向误差 0.1~0.4° × 距离 + 测距 ±3 cm，
在 50 m 处约 **0.1~0.3 m**。

需要按"打到靶标中心"而不是"打到表面"来报坐标时，可以用 `range_bias` 补上
机体半径（对 uav2 约 +1.5 m），但这是**靶标尺寸补偿，不是传感器标定** ——
换一个尺寸不同的靶标就要改，所以默认留 0。

## 5.1 直接对接 Project AirSim（机体固连测距，无云台时）

`live/` 下提供了纯标准库的 ROS1 客户端，不用在本机装 ROS 就能读远端话题并解算：

```bash
# 一次性读数（30 帧），打印原始测量 + 目标经纬度
python3 live/locate_live.py --uri http://<JETSON_IP>:11311/ --robot Drone1
```

在装了 ROS 的机器上，用主节点对接 AirSim 的四个话题（`FrontDistance` 是机体
固连的前视测距仪，没有云台，所以姿态源用 IMU，`~att_msg_type:=Imu`）：

```bash
rosrun target_geolocation target_geolocation_node.py \
  _uav_ns:=/Sim/SceneBasicDrone/robots/Drone1 \
  _gps_topic:=/Sim/SceneBasicDrone/robots/Drone1/sensors/GPS/gps \
  _range_topic:=/Sim/SceneBasicDrone/robots/Drone1/sensors/FrontDistance/distance_sensor \
  _gimbal_att_topic:=/Sim/SceneBasicDrone/robots/Drone1/sensors/IMU/imu_kinematics \
  _att_msg_type:=Imu \
  _local_pose_topic:=/Sim/SceneBasicDrone/robots/Drone1/actual_pose \
  _mavlink_topic:= \
  _output_prefix:=/drone1/target
```

Project AirSim 的 ROS 桥**不转换坐标系**：位置、姿态、速度全部是 NED/FRD
（X 北 / Y 东 / Z 下），与本包 `solver` 的默认约定一致，不需要额外换算。

## 6. 用真实数据（有测距仪 / 真实 #285）

```bash
roslaunch target_geolocation target_geolocation.launch            # /uav1
roslaunch target_geolocation target_geolocation.launch uav_ns:=/uav2
```

默认订阅（留空则按 `uav_ns` 自动拼接）：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/uav1/mavros/global_position/global` | NavSatFix | 飞机 GPS，基准点 |
| `/uav1/mavlink/from` | Mavlink | 原始帧，节点从中解析 `#285` |
| `/uav1/mavros/global_position/compass_hdg` | Float64 | 罗盘航向（#285 未锁 yaw 时必需） |
| `/uav1/mavros/distance_sensor/laser_1d` | Range | 测距仪斜距（可用 `~range_topic` 改） |
| `/uav1/mavros/local_position/pose` | PoseStamped | 仅用于把结果画到 RViz |

发布：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/uav1/target/global_position` | NavSatFix | 目标经纬高（主输出） |
| `/uav1/target/local_enu` | PointStamped | 相对飞机的 ENU 偏移 |
| `/uav1/target/marker` | Marker | RViz 红球 |
| `/uav1/target/solution` | Float64MultiArray | `[lat,lon,alt,e,n,u,斜距,平距,方位,仰角,向上标志]` |

如果仿真里没有测距仪话题（`rostopic hz /uav1/mavros/distance_sensor/laser_1d`
无输出），三种办法：给 core_sim 的机体加一个向下/随云台的测距传感器；把已有
测距话题用 `~range_topic` 指过去；或者先用第 5 节的合成传感器验证全链路。

## 7. 关键参数与标定

完整列表见 `config/params.yaml`，最常动的几个：

| 参数 | 默认 | 说明 |
|---|---|---|
| `~gimbal_att_topic` | `""` | 非空则用它当姿态源（`QuaternionStamped`，世界系），不再解析 `#285` |
| `~boresight_roll/pitch/yaw_deg` | 0 | 光轴相对云台基准的固定安装偏差 |
| `~sensor_axis` | `x` | 光轴对应的机体系轴 |
| `~max_skew` | 0.20 s | 三路输入允许的最大时间差；飞机 10 m/s 时 200 ms 就带来 ~2 m 误差 |
| `~use_img_offset` | true | 用检测框把目标偏出光轴的角度补回来（需 `~det_topic`） |
| `~smooth_alpha` | 0.4 | 输出 EMA，0=关闭 |
| `~publish_only_on_change` | true | 输入无新数据时跳过整个解算周期；静止悬停时 CPU 近似为 0 |
| `~enable_marker` | true | 关掉可省去每周期 Marker 组装与序列化（只要坐标时设 false） |
| `~range_bias` | 0.0 | 靶标尺寸补偿（激光停在表面导致的径向偏短），不是传感器标定 |

**CPU 占用实测**（`bench/cpu_profile.py`，Jetson Orin NX）：

| 环节 | 单次耗时 | 10 Hz 下单核占用 |
|---|---|---|
| `solve_target()` 完整解算 | 5.10 µs | 0.0056% |
| `enu_to_geodetic()` 测地换算 | 1.10 µs | — |
| 一次完整 tick | 5.62 µs | 0.0056% |
| 空转 tick（无新数据） | 0.03 µs | ~0% |
| `#285` MAVLink 解析（改后） | 0.04 µs/帧 | 比解析整帧快 19 倍 |

也就是说解算节点在任何合理频率下都不是 CPU 瓶颈；占用高要看视频链路（见
`bench/README` 里的线程级归因方法）。

**一步验证法（强烈建议先做）**：把飞机悬停在已知点正上方，云台垂直下视
（pitch≈-90），对地面一个 GPS 已知的点测距。

- 若方位对、距离对、高度差对 → 约定一致，可以继续；
- 若方位偏差固定值 → 填 `~boresight_yaw_deg`；
- 若俯仰反向（目标落在反方向）→ `~boresight_pitch_deg:=180`（或先确认云台
  是否真的下视）；
- 若方位偏 90° 的整数倍 → `~sensor_axis` 设错了，或 `YAW_LOCK` 判定有误
  （用 `rostopic echo /uav1/mavlink/from` 看 flags 是否含 16）。

## 8. 精度预算（`test/offline_e2e.py` 实测，非估算）

典型工况：平距 300 m、高差 120 m、姿态噪声 0.1°、测距噪声 5 cm
→ **水平 mean 0.48 m（p95 1.06 m）、垂直 mean 0.40 m**。

姿态误差是最主要的误差源（`横向误差 ≈ L·cosθ·δ角`）：

| 姿态噪声 | 水平 mean (L=323 m) |
|---|---|
| 0.05° | 0.23 m |
| 0.1° | 0.49 m |
| 0.5° | 2.37 m |
| 1.0° | 4.86 m |
| 2.0° | 9.90 m |

随距离线性放大（姿态 0.5°，高差 100 m）：50 m→0.8 m，200 m→1.8 m，400 m→2.9 m，
800 m→5.4 m，1500 m→10.9 m。

结论：**云台姿态反馈的精度决定一切**。用指令角度代替反馈、或 `#285` 有 1~2°
的抖动，误差就是 5~10 m 量级；这时应优先修姿态链路（稳像、反馈频率、安装偏差），
而不是去换更贵的测距仪。

## 9. 常见问题

- **一直"等待输入"**：`rostopic hz` 逐个确认三路话题；多机时检查 `uav_ns`。
- **"云台 yaw 未锁定，等待罗盘航向"**：`#285` 的 `YAW_LOCK` 位未置（未稳像），
  必须让 `compass_hdg` 有数据；真机上也说明云台没进入世界系稳像。
- **"传感器时间不同步"**：mavros 的 GPS 与 `#285` 时间差过大，检查 `/use_sim_time`
  与负载；飞机高速时这个门限是保护，不要随意放大。
- **解出来的点在动**：云台抖动 + 目标检测噪声，开 `~smooth_alpha` 或对下游做滤波；
  但平滑会引入滞后，跟踪移动目标时注意。
- **ROS2**：本包为 ROS1（`rospy`）。移植到 Humble 需改订阅/发布与参数 API，
  数学部分（`geodesy.py`、`solver.py`）可直接复用。

## 10. 设计取舍

- **不依赖 TF**：目标点算出来就在 WGS84 里，用 TF 反而要额外维护
  `world->base_link` 与 ENU/NED 两套约定，容易出错。这里只用三个标量话题。
- **拒绝而不是凑**：任一输入过期、时间不同步、yaw 未锁却没航向、测距越界、
  光轴朝上——一律不出解。宁可少输出，也不输出看似合理的错点。
- **合成传感器用正向模型**：如果用同一个函数正反各算一次，符号错了也会"自洽通过"。

## 11. 排查 CPU 占用（Jetson 上怎么定位到线程）

本节点 10 Hz 下单核占用约 0.006%，`top` 里看到的高占用基本都不是它。真要查，
按 `bench/README.md` 的方法做**线程级**归因，两个关键点：

1. `top -b -n 1` 的 %CPU 首次迭代口径不稳，用两次 `/proc/<pid>/task/*/stat`
   差分算（`bench/thread_cpu.py`）；
2. GStreamer 多实例线程同名，**不能按线程名采样**，要先从 `info threads` 的
   `(LWP <tid>)` 锁定 gdb 编号（`bench/sample_tid.py`）。

2026-10-30 实测：UE 像素流播放器 `ps_native_player.py` 占满一个核，拆到线程是
`rtpjitterbuffer` 独占 **70%**，而硬件解码 `nvv4l2decoder` 只有 10%。按 eth0
收包率 3309 包/s、平均包长 407 字节算，折合**每包 241 微秒**，而该元件正常只要
2~5 微秒——不是包太多，是这个元件在这种小包高包率的 WebRTC 流上低效。

改动是让 sink 按时间戳渲染（给它一个下游节流点）并放宽 basesink 的
`max-lateness`：

| 配置 | 平均单核 | fps | 丢帧 |
|---|---|---|---|
| `sync=False`（原始） | 94.9% | 60.0 | 无 |
| `sync=True`（只开同步） | 33.7% | 60 / 0.9 | **有** |
| `sync=True` + `max-lateness=-1` | **59.2%** | 60.0 | 无 |

只开 `sync=True` 会把 CPU 压到 33%，但 basesink 默认 20ms 的 `max-lateness`
会成批丢帧（日志刷 `gst_base_sink_is_too_late`，fps 周期性掉到 0.9）；
必须同时放宽才能保住画面。详细踩坑记录见 `bench/README.md`。

### 11.1 根治方向：把 UE 编码帧率降下来（包率减半）

11 节那套 `sync=True` 只是"少做无用的重采样"，包还是那么多。真正的杠杆在源头：
`rtpjitterbuffer` 的开销与**包率**成正比，而包率 ∝ 帧率。

Web 端「仿真配置 → 像素流帧率」可选 60/30/24/20（默认 30），随 manifest 的
`pixelStreamingFrameRate` 下发，经 orchestrator 透传到 `ue_controller`，最终拼成
UE 启动参数。60→30 帧，包率近似减半，播放器侧预期从 59% 降到 33% 附近
（`nvv4l2decoder` 本身只占 10%，省的不是解码，是收包与抖动缓冲）。

**踩坑（重要）**：UE 5.2 **没有** `-PixelStreamingFrameRate` 这个开关，写了会被
静默忽略、帧率仍是 60，白忙一场。真正生效的是 cvar `PixelStreaming.WebRTC.Fps`，
命令行形态为 `-PixelStreamingWebRTCFps=30`。

这个结论不是猜的，是在本机
`UE_5.2\Engine\Plugins\Media\PixelStreaming\Binaries\Win64\UnrealEditor-PixelStreaming.dll`
里把字符串全量扫出来确认的；同族的还有 `PixelStreamingWebRTCMaxBitrate=` /
`PixelStreamingWebRTCMinBitrate=` / `PixelStreamingWebRTCStartBitrate=` /
`PixelStreamingEncoderCodec=`，唯独没有 FrameRate 那个。

验证是否真的生效：查 UE 进程命令行里有没有 `-PixelStreamingWebRTCFps=30`，以及
`http://127.0.0.1:8090/status` 返回的 `pixelStreamingFrameRate` 字段。


