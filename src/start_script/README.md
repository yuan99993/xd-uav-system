# UAV 启动管理器

`start_manager.sh` 在运行飞机代码的板子上启动 TCP 监听器。仿真端发送完整的 JSON 状态，板子按每架飞机的 `launches` 清单启动对应程序。管理器不会启动本地 `roscore`，进程使用环境中的 `ROS_MASTER_URI` 和 `ROS_IP`。

## 启动监听器

```bash
export ROS_MASTER_URI=http://<仿真主机地址>:11311
export ROS_IP=<飞机代码板子的地址>
export XD_UAV_START_TOKEN='<共享密钥>'
./start_manager.sh
```

默认监听 `0.0.0.0:8765`。端口和监听地址可在 `manager.yaml` 的 `network` 中修改。仿真端与板子端使用相同的 `XD_UAV_START_TOKEN`。

## JSON 状态

TCP 连接上每行发送一个 UTF-8 JSON 对象。开始时发送完整状态；飞机、启动清单或 profile 改变时用新的完整状态发送 `update`。每 30 秒内至少发送一次 heartbeat。连接断开、超时或收到 `stop` 后，管理器会停止这次会话启动的所有进程。

[example_state.json](example_state.json) 展示了两架飞机采用不同启动项的例子。关键字段如下：

示例里的 `/Sim/...` 相机和测距话题要按仿真实际发布的话题替换；管理器不会再假设所有仿真都使用同一组相机路径。

- `shared_launches`：放仿真端选择的整支机队共用程序。`world_tf` 会依据当前 `vehicles` 名称和数量生成临时配置。编队管理器由 start 自动启动一次，不需要在 JSON 里重复列出。
- `vehicles[].sensors`：沿用仿真端现有的传感器类型并声明实际话题。`front_gimbal_camera` 会自动启动云台姿态桥接、云台控制和目标定位；它仍然是一条前视相机记录。
- `vehicles[].launches`：选择该架飞机的识别和其他可选程序。启动项可以是字符串，也可以是带 `sensor` 和 `params` 的对象；前视相机对应的云台程序由 start 自动加入。

`xd_uav_formation/formation.launch` 使用包内固定的机队配置，由 start 在每个会话中只启动一个管理进程，供整支机队使用。每架飞机不需要单独启动一份。

相机只有前视和下视两路。保持仿真 JSON 现有的 `front_gimbal_camera`、`down_camera` 类型定义：`front_gimbal_camera` 表示前视这一路相机，它安装在云台上；云台不是额外的相机条目。云台相关启动项绑定到同一架飞机的前视相机 ID（示例中是 `front`）。

`sar_yolo_detector` 沿用仓库现有 `xd_smart_tracker_fusion_integration.launch` 的默认命名和话题。管理器只传入 `UAV_NAME`、图像话题和 CameraInfo 话题；模型、推理和输出话题采用该 launch 文件自身的默认值。不同飞机通过 `UAV_NAME` 和各自相机输入话题区分，不按传感器 ID 改写 `smart` 后缀。

目前可选传感器类型：

- `front_gimbal_camera`：现有 JSON 中的前视光电相机，安装在云台上；可提供 `range_topic`、`gimbal_att_topic`。
- `down_camera`：下视相机。
- `lidar`：点云或激光测距传感器。

机型对应的 `localization`、`control` 和 `task_execute`，以及视觉识别程序，都通过飞机自己的 `launches` 清单选择。识别程序必须明确选择 `detect` 或 `sar_yolo_detector` 并绑定相机；不需要识别时就省略它们。前视相机带来的三个云台/定位程序由 start 自动启动；若要覆盖默认参数，可以在 `launches` 里显式填写同名启动项。`task_allocate` 不在板端启动清单中。

自动启动的云台控制沿用 `tmux_start` 中的完整命令参数；只按飞机替换 UAV 名称和 `/uavN/` 话题前缀。仿真 JSON 中的 `front` 传感器 ID 或其启动参数不会改写云台控制参数。目标定位也固定连接同一个 `smart` 选择目标话题，前视传感器提供的测距和姿态话题仍按 JSON 配置。

云台姿态桥接沿用原启动方式，只传 `_uav_ns:=/uavN`；脚本按该参数发布 `/uavN/gimbal/attitude` 和 `/uavN/track/gimbal_state`。

`camera_stream_receiver` 是通用接收器，前视和下视都使用同一份启动定义；管理器按传感器类型给每个实例传入相机流名称和输出话题。默认接收 `DroneN/FrontCamera` 或 `DroneN/DownCamera`，分别发布到 `/uavN/front_camera/image_raw`、`/uavN/front_camera/camera_info` 和 `/uavN/down_camera/image_raw`、`/uavN/down_camera/camera_info`。仿真相机流名称不同时，可在对应 sensor 的 `params` 中设置 `camera_stream_name`。接收器会在视频服务暂未就绪时重试订阅。SAR YOLO 和 detect 使用对应飞机的本地相机话题；track 叠加沿用 `xd_uav_track/config/track.yaml` 的现有设置。

## 固定启动项与视频端口

每架飞机无论 JSON 如何配置，都会启动 `xd_uav_track/track.launch` 和 `src/bridge/h264_nal_multicast_sender.py`。发送器使用当前代码的跟踪输出话题 `/uavN/track/phase_annotated`；端口根据飞机编号自动分配：

| 飞机 | UDP 端口 | ROS 图像话题 |
| --- | ---: | --- |
| `uav1` | `32797` | `/uav1/track/phase_annotated` |
| `uav2` | `32798` | `/uav2/track/phase_annotated` |
| `uavN` | `32796 + N` | `/uavN/track/phase_annotated` |

端口之外的组播地址和网卡 IP 目前沿用 `manager.yaml` 中发送器配置。车辆名使用 `uav1`、`uav2` 这样的格式，以便稳定计算端口。

## 示例客户端

```bash
./client_example.py --host <飞机代码板子的地址> example_state.json
```

仿真端也可以自行实现 TCP 客户端。Heartbeat 示例：

```json
{"action":"heartbeat","session_id":"sim_run_001"}
```

每次 `update` 都要携带完整的 `vehicles` 和 `shared_launches`。从清单移除某项后，管理器会停止对应进程。

## 临时配置与进程清理

只有 `world_tf` 的运行配置写入 `~/.ros/xd_uav_start_manager/generated/`，不会改写 `src/` 下的源配置。配置会按照本次 JSON 中的飞机列表更新 `vehicles` 和 `vehicle_configs`。track 沿用 `track.launch` 默认加载的包内配置；编队包直接使用 `formation.launch` 中已设置的固定配置。

每个启动项独立运行并记录 PID、进程组、启动时间、命令和日志。同一启动项重启时继续写入同一个日志文件，并用启动时间分隔；追加前会修复已有的非法 UTF-8 字节。日志位于 `~/.ros/xd_uav_start_manager/logs/`，当前路径记录在进程状态中。停止会话、连接断开或更新移除启动项时，管理器会核对 PID 身份，结束启动进程组并清理它派生的 ROS 节点进程（ROS 节点可能各自在独立进程组）。
