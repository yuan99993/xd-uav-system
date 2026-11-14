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

- `shared_launches`：只放整支机队共用的程序。`world_tf` 会依据当前 `vehicles` 名称和数量生成临时配置；可选 `formation`。
- `vehicles[].sensors`：声明传感器类型和本仿真的实际话题。声明传感器本身不会启动识别。
- `vehicles[].launches`：只启动该架飞机列出的程序。启动项可以是字符串，也可以是带 `sensor` 和 `params` 的对象。
- `track_sensor`：选择 `track/phase_annotated` 使用的相机。若该传感器启用了 SAR YOLO，视频叠加使用 SAR 的 annotated 输出；否则直接使用传感器的 `image_topic`。

目前可选传感器类型：

- `front_gimbal_camera`：前视光电云台相机，可带 `range_topic`、`gimbal_att_topic`。
- `down_camera`：下视相机。
- `lidar`：点云或激光测距传感器。

飞机启动项包括机型对应的 `localization`、`control`，以及可选的 `detect`、`sar_yolo_detector`、`gimbal_attitude_bridge`、`gimbal_control`、`target_geolocation`、`task_execute`。识别程序必须明确写入飞机的 `launches`，并绑定传感器；不需要识别时就省略 `detect` 和 `sar_yolo_detector`。`task_allocate` 不在板端启动清单中。

`gimbal_control` 的 `mode` 必填。`mode: tracking` 时还要显式给出 `detection_input` 与 `detection_topic`。`target_geolocation` 需要在其启动参数中指定 `detection_topic`，并在所绑定的前视传感器中给出 `range_topic` 和 `gimbal_att_topic`。

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

`world_tf` 和 `formation` 的运行配置、SAR 的传感器 overlay、以及每架飞机的 track 配置都写入 `~/.ros/xd_uav_start_manager/generated/`，不会改写 `src/` 下的源配置。world_tf 临时配置会按照本次 JSON 中的飞机列表更新 `vehicles` 和 `vehicle_configs`。

每个启动项独立运行并记录 PID、进程组、启动时间、命令和日志。停止会话、连接断开或更新移除启动项时，管理器会先核对 PID 身份，再对目标进程组批量发送 `SIGKILL`，不逐个等待宽限期；日志位于 `~/.ros/xd_uav_start_manager/logs/`。
