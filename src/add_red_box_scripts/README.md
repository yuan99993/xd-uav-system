# Gazebo 搜索目标测试脚本

## 在任务搜索区域内随机生成车辆

`spawn_random_vehicles.py` 订阅
`/task_allocate/search_areas`（`xd_uav_task_allocate/SearchAreaArray`），然后在消息里的所有搜索多边形内随机生成 Gazebo 静态车辆。

默认可选模型为：

- `bus`
- `car_beetle`
- `car_golf`
- `car_lexus`
- `car_opel`
- `car_polo`
- `car_volvo`

当生成数量不少于区域数量时，每个区域至少放置一辆车；其余车辆按区域面积随机分配。脚本会使用保守车身范围检查区域边界和车辆间距，但不会检测搜索区域中原有建筑物、树木等障碍物。

先启动等待区域消息的脚本：

```bash
cd /home/kzy/xd-uavsystem-test && source devel/setup.bash && python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --replace
```

再执行原来的区域发布命令。需要采用这个顺序，因为 `rostopic pub -1` 是一次性发布；脚本必须在消息发布时处于订阅等待状态。收到区域后，旧的 `search_vehicle_N` 模型会因为 `--replace` 被删除，再随机生成一组新车辆。

只使用六种小汽车、不生成公交车：

```bash
python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --replace --models car_beetle car_golf car_lexus car_opel car_polo car_volvo
```

固定随机结果以方便重复测试：

```bash
python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --replace --seed 7
```

只计算并打印位置，不向 Gazebo 添加模型：

```bash
python3 src/add_red_box_scripts/spawn_random_vehicles.py --count 10 --seed 7 --dry-run
```

常用参数：

| 参数 | 说明 |
|---|---|
| `--count N` | 车辆数量，默认 10 |
| `--replace` | 删除脚本上次生成的同前缀车辆后再生成 |
| `--models ...` | 限制随机选择的车型 |
| `--seed N` | 固定随机数种子 |
| `--ground-z Z` | Gazebo 地面世界坐标高度，默认 0 |
| `--margin M` | 车身与搜索区域边界的额外距离，默认 1 m |
| `--min-gap M` | 不同车辆保守范围之间的最小距离，默认 2 m |
| `--search-area-timeout S` | 等待区域消息的时间，默认 60 s |
| `--dry-run` | 不修改 Gazebo，只打印随机车型、区域、位置和朝向 |

如果区域较小而车辆数量过多，脚本会拒绝生成并提示减小 `--count`、`--margin` 或 `--min-gap`，避免车辆重叠或越出任务区域。
