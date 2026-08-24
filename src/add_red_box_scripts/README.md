# Gazebo 搜索目标测试脚本

本目录提供 3 个正式测试脚本：

| 脚本 | 用途 |
|---|---|
| `spawn_red_boxes.py` | 在 Gazebo 中按指定坐标、矩形区域或任务搜索多边形生成红色方块 |
| `spawn_random_vehicles.py` | 在任务搜索多边形内随机生成 Gazebo 静态车辆 |
| `red_box_detector.py` | 从一架或多架无人机的相机图像中检测红色目标，并发布 `DetectionArray` |

`spawn_red_boxes copy.py` 是 `spawn_red_boxes.py` 的旧副本，缺少按
`SearchAreaArray` 生成目标的功能，不建议使用。以下命令均以正式脚本为准。

## 运行前准备

先启动 ROS、Gazebo 和所需的无人机仿真，然后在一个新终端加载工作空间：

```bash
cd /home/kzy/xd-uavsystem-test && source devel/setup.bash
```

如果尚未编译工作空间，先在项目根目录执行：

```bash
catkin_make && source devel/setup.bash
```

可以随时用 `--help` 查看脚本当前支持的完整参数：

```bash
python3 src/add_red_box_scripts/spawn_red_boxes.py --help
python3 src/add_red_box_scripts/spawn_random_vehicles.py --help
python3 src/add_red_box_scripts/red_box_detector.py --help
```

## 1. 生成红色方块：`spawn_red_boxes.py`

脚本通过 `/gazebo/spawn_sdf_model` 生成纯红色方块。方块默认是静态模型，
默认尺寸为 `1 m × 1 m × 1 m`，模型名依次为 `red_box_1`、
`red_box_2` 等。

### 按世界坐标生成

`--position` 可以重复使用。只给 `X,Y` 时，脚本把方块放到
`--ground-z` 所表示的地面上；给出 `X,Y,Z` 时，`Z` 是方块中心的世界坐标。

```bash
# 在 (5, 5) 和 (15, 15) 生成两个方块
python3 src/add_red_box_scripts/spawn_red_boxes.py --position 5,5 --position 15,15 --ground-z 0 --replace

# 指定方块中心的完整三维坐标；负数建议使用“参数=值”的写法
python3 src/add_red_box_scripts/spawn_red_boxes.py --position=-10,-10,0.5 --replace
```

### 在矩形区域内随机生成

每个 `--area` 的格式是 `XMIN,XMAX,YMIN,YMAX`，可重复指定多个区域。
未指定 `--area` 时使用内置的两个区域：`0,20,0,20` 和
`-30,0,-30,0`。

```bash
# 在一个矩形区域内随机生成 6 个方块
python3 src/add_red_box_scripts/spawn_red_boxes.py --count 6 --area=-25,-5,-25,-5 --ground-z 0 --seed 7 --replace

# 在两个矩形区域之间轮流分配方块
python3 src/add_red_box_scripts/spawn_red_boxes.py --count 8 --area=0,20,0,20 --area=-30,0,-30,0 --ground-z 0 --replace
```

### 在任务搜索区域内随机生成

脚本可等待 `/task_allocate/search_areas` 上的
`xd_uav_task_allocate/SearchAreaArray`，并把方块完整放在消息中的搜索多边形内。
因为任务区域通常用 `rostopic pub -1` 一次性发布，必须先启动本脚本等待消息，
再在另一个终端发布区域。

终端 1：

```bash
cd /home/kzy/xd-uavsystem-test && source devel/setup.bash && python3 src/add_red_box_scripts/spawn_red_boxes.py --count 6 --search-area-topic /task_allocate/search_areas --size 3 3 1 --ground-z 0 --replace
```

终端 2：执行任务系统原有的搜索区域发布命令。收到区域消息后，终端 1 才会
计算位置并调用 Gazebo 服务。搜索区域消息的 `header.frame_id` 必须和
`--reference-frame` 一致；默认均应为 `world`。

### 调整方块并预览结果

```bash
# 生成 2 m × 2 m × 1.5 m 的方块，方向为 30 度
python3 src/add_red_box_scripts/spawn_red_boxes.py --count 4 --area=0,20,0,20 --size 2 2 1.5 --yaw-deg 30 --ground-z 0 --replace

# 只计算和打印结果，不连接 Gazebo
python3 src/add_red_box_scripts/spawn_red_boxes.py --count 6 --area=0,20,0,20 --ground-z 0 --seed 7 --dry-run
```

使用 `--search-area-topic` 的 `--dry-run` 仍然需要 ROS 和一条搜索区域消息，
但不会调用 Gazebo 的生成、删除服务。

常用参数：

| 参数 | 说明 |
|---|---|
| `--position X,Y[,Z]` | 指定一个位置，可重复使用；不能和 `--count` 同时使用 |
| `--count N` | 随机生成数量；不能和 `--position` 同时使用 |
| `--area XMIN,XMAX,YMIN,YMAX` | 随机生成矩形，可重复使用 |
| `--search-area-topic TOPIC` | 从 `SearchAreaArray` 获取多边形，仅与 `--count` 配合，不能再指定 `--area` |
| `--search-area-timeout S` | 等待搜索区域消息的最长时间，默认 30 s |
| `--size X Y Z` | 方块尺寸，默认 `1 1 1` m |
| `--ground-z Z` | 地面世界坐标高度；当前脚本默认值为 2，通常建议按仿真地面显式设置 |
| `--margin M` | 方块与区域边界的额外安全距离，默认 0 m |
| `--min-spacing M` | 随机方块中心之间的最小 XY 距离，默认 2 m |
| `--seed N` | 固定随机数种子，便于重复测试 |
| `--yaw-deg DEG` | 所有方块的偏航角，默认 0° |
| `--prefix NAME` | 模型名前缀，默认 `red_box` |
| `--start-index N` | 第一个模型编号，默认 1 |
| `--replace` | 生成前删除 Gazebo 中所有同前缀的 `PREFIX_N` 模型 |
| `--dynamic` | 生成可运动模型；不指定时为静态模型 |
| `--reference-frame FRAME` | Gazebo 参考坐标系，默认 `world` |
| `--dry-run` | 只打印位置，不修改 Gazebo |

## 2. 在搜索区域内生成随机车辆：`spawn_random_vehicles.py`

脚本订阅 `/task_allocate/search_areas`，在所有搜索多边形内随机生成 Gazebo
静态车辆。默认可选模型为：

- `bus`
- `car_beetle`
- `car_golf`
- `car_lexus`
- `car_opel`
- `car_polo`
- `car_volvo`

当车辆数量不少于区域数量时，每个区域至少放置一辆车，其余车辆按区域面积
随机分配。脚本会保守检查车身与区域边界及其他车辆的距离，但不会检测区域中
已有的建筑物、树木等 Gazebo 障碍物。

与红色方块的搜索区域模式一样，要先启动车辆脚本，再发布一次性区域消息。

终端 1：

```bash
cd /home/kzy/xd-uavsystem-test && source devel/setup.bash && python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --ground-z 0 --replace
```

终端 2：执行任务系统原有的搜索区域发布命令。收到区域后，旧的
`search_vehicle_N` 模型会因 `--replace` 被删除，再生成一组新车辆。

其他常用方式：

```bash
# 只使用六种小汽车，不生成公交车
python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --replace --models car_beetle car_golf car_lexus car_opel car_polo car_volvo

# 固定随机结果，方便重复测试
python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --ground-z 0 --replace --seed 7

# 收到搜索区域后只计算并打印布局，不修改 Gazebo
python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --ground-z 0 --seed 7 --dry-run
```

车辆模型从 `--model-root`、`GAZEBO_MODEL_PATH`、`~/.gazebo/models` 和系统
Gazebo 模型目录中依次查找。若提示找不到 `MODEL/model.sdf`，请设置正确的
`GAZEBO_MODEL_PATH` 或显式指定模型根目录：

```bash
python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --model-root /path/to/gazebo/models --replace
```

常用参数：

| 参数 | 说明 |
|---|---|
| `--count N` | 车辆数量，默认 10 |
| `--models MODEL ...` | 限制随机选择的车型，默认使用全部 7 种 |
| `--search-area-topic TOPIC` | 搜索区域话题，默认 `/task_allocate/search_areas` |
| `--search-area-timeout S` | 等待区域消息的时间，默认 60 s |
| `--model-root DIRECTORY` | 指定 Gazebo 模型根目录 |
| `--ground-z Z` | Gazebo 地面世界坐标高度，默认 0 |
| `--margin M` | 车辆保守范围与区域边界的额外距离，默认 1 m |
| `--min-gap M` | 不同车辆保守范围之间的最小距离，默认 2 m |
| `--seed N` | 固定随机数种子 |
| `--prefix NAME` | 模型名前缀，默认 `search_vehicle` |
| `--start-index N` | 第一个模型编号，默认 1 |
| `--replace` | 删除所有同前缀的 `PREFIX_N` 车辆后重新生成 |
| `--reference-frame FRAME` | Gazebo 参考坐标系，默认 `world`，需与区域消息一致 |
| `--dry-run` | 收到区域后只打印车型、区域、位置和朝向，不修改 Gazebo |

如果区域过小或车辆过多，脚本会拒绝生成。此时可减小 `--count`、
`--margin` 或 `--min-gap`，或者发布更大的搜索区域。

## 3. 检测红色方块：`red_box_detector.py`

该脚本使用 OpenCV HSV 阈值从相机图像中提取红色区域，可在一个进程中同时
处理多架无人机。它只输出二维检测框，不负责三维定位或跟踪。

默认处理 `uav1` 和 `uav2`：

```bash
python3 src/add_red_box_scripts/red_box_detector.py
```

显式指定无人机：

```bash
python3 src/add_red_box_scripts/red_box_detector.py --uavs uav1 uav2 uav3
```

每架无人机的默认接口如下：

| 方向 | 默认话题 | 消息类型 |
|---|---|---|
| 订阅 | `/<uav>/camera/image_raw` | `sensor_msgs/Image` |
| 发布检测结果 | `/<uav>/detect/input/detections_2d` | `xd_uav_track/DetectionArray` |
| 发布标注图 | `/<uav>/track/red_detector/debug_image` | `sensor_msgs/Image` |

如果某架无人机使用下视相机，可单独覆盖输入话题；`--image-topic` 可重复使用：

```bash
python3 src/add_red_box_scripts/red_box_detector.py --uavs uav1 uav2 --image-topic uav1=/uav1/down_camera/image_raw --image-topic uav2=/uav2/down_camera/image_raw
```

也可以统一修改话题模板，模板中必须保留 `{uav}`。在 Bash 中建议用单引号：

```bash
python3 src/add_red_box_scripts/red_box_detector.py --uavs uav1 uav2 --image-template '/{uav}/down_camera/image_raw' --detections-template '/{uav}/detect/input/detections_2d' --debug-template '/{uav}/track/red_detector/debug_image'
```

不需要标注图时可关闭发布，以减少图像拷贝开销：

```bash
python3 src/add_red_box_scripts/red_box_detector.py --uavs uav1 uav2 --no-debug-image
```

命令行参数：

| 参数 | 说明 |
|---|---|
| `--uavs NAME ...` | 无人机命名空间；默认读取私有参数 `~uavs`，否则为 `uav1 uav2` |
| `--image-template TEMPLATE` | 所有无人机的输入图像话题模板 |
| `--image-topic NAME=TOPIC` | 覆盖单架无人机的输入话题，可重复使用 |
| `--detections-template TEMPLATE` | 检测结果话题模板 |
| `--debug-template TEMPLATE` | 标注图话题模板 |
| `--no-debug-image` | 禁止发布所有标注图 |

检测阈值使用 ROS 私有参数配置。例如，提高灵敏度并限制每帧最多输出 5 个目标：

```bash
python3 src/add_red_box_scripts/red_box_detector.py --uavs uav1 uav2 _minimum_area_px:=150 _minimum_width_px:=5 _minimum_height_px:=5 _maximum_targets:=5
```

主要 ROS 私有参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `~hue_low_1` / `~hue_high_1` | `0` / `12` | 第一段红色色相范围，OpenCV H 范围为 0～179 |
| `~hue_low_2` / `~hue_high_2` | `168` / `179` | 第二段红色色相范围 |
| `~saturation_min` | `100` | 最小饱和度 |
| `~value_min` | `60` | 最小亮度 |
| `~minimum_area_px` | `400` | 最小红色轮廓面积，单位 px² |
| `~minimum_width_px` / `~minimum_height_px` | `8` / `8` | 最小检测框宽度和高度 |
| `~maximum_targets` | `0` | 每帧最大目标数；0 表示不限制，面积大的目标优先 |
| `~class_id` | `0` | 写入 `DetectionCandidate.class_id` 的类别编号 |
| `~confidence` | `1.0` | 写入每个候选框的固定置信度 |
| `~morphology_kernel` | `5` | 形态学处理核大小；偶数会自动增大为奇数 |
| `~morphology_iterations` | `1` | 开、闭运算次数；0 表示关闭形态学处理 |
| `~publish_debug_image` | `true` | 是否发布标注图，也可由 `--no-debug-image` 强制关闭 |
| `~show_window` | `false` | 是否打开本机 OpenCV 窗口；仅在有图形界面时使用 |

可用以下命令检查数据是否正常：

```bash
# 检查相机是否有图像
rostopic hz /uav1/camera/image_raw

# 检查检测结果频率和内容
rostopic hz /uav1/detect/input/detections_2d
rostopic echo /uav1/detect/input/detections_2d

# 显示标注图
rqt_image_view /uav1/track/red_detector/debug_image
```

如果实际使用的是下视相机，第一条检查命令也应改为
`/uav1/down_camera/image_raw`，并确保启动检测器时使用了相同的输入话题。

## 常见问题

### 发布了搜索区域，但脚本仍然超时

`rostopic pub -1` 只发布一次。请先运行生成脚本，看到它开始等待区域消息后，
再发布搜索区域；或者让区域发布者使用 latched topic。
cd /home/kzy/xd-uavsystem-test && source devel/setup.bash && rostopic pub -1 /task_allocate/search_areas xd_uav_task_allocate/SearchAreaArray "{header: {frame_id: 'world'}, areas: [{area_id: 1, boundary: {points: [{x: 0.0, y: 0.0, z: 0.0}, {x: 20.0, y: 0.0, z: 0.0}, {x: 20.0, y: 20.0, z: 0.0}, {x: 0.0, y: 20.0, z: 0.0}]}, altitude: 5.0, lane_spacing: 5.0, priority: 1}, {area_id: 2, boundary: {points: [{x: -30.0, y: -30.0, z: 0.0}, {x: -10.0, y: -30.0, z: 0.0}, {x: -10.0, y: -10.0, z: 0.0}, {x: -30.0, y: -10.0, z: 0.0}]}, altitude: 5.0, lane_spacing: 5.0, priority: 1}]}"


cd /home/kzy/xd-uavsystem-test && source devel/setup.bash && rostopic pub -1 /task_allocate/search_areas xd_uav_task_allocate/SearchAreaArray "{header: {frame_id: 'world'}, areas: [{area_id: 1, boundary: {points: [{x: -75.0, y: -75.0, z: 0.0}, {x: 75.0, y: -75.0, z: 0.0}, {x: 75.0, y: 75.0, z: 0.0}, {x: -75.0, y: 75.0, z: 0.0}]}, altitude: 30.0, lane_spacing: 5.0, priority: 1}]}"
### Gazebo 提示模型名称已存在

再次生成时加 `--replace`，或者通过 `--prefix` 使用新的模型名前缀。

### 搜索区域坐标系不匹配

`SearchAreaArray.header.frame_id` 必须与 `--reference-frame` 一致。脚本不会自动
做 TF 变换，需要先把区域坐标转换到 Gazebo 使用的参考坐标系。

### 红色方块已出现，但没有检测结果

依次确认相机话题有图像、检测器订阅的话题正确、方块确实进入相机视野。若目标
很小，可适当降低 `~minimum_area_px`、`~minimum_width_px` 和
`~minimum_height_px`；若光照导致颜色偏暗，可降低 `~saturation_min` 或
`~value_min`。
