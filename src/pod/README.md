# pod：光电吊舱任务软件 ROS1 包组

本目录承载 `OPTICAL_POD_SOFTWARE_PROJECT_PLAN.md` 第 6 节定义的包边界。

当前提交只建立可被 catkin 识别的包、公共消息和任务 Action；不包含真实吊舱协议、视频解码器、检测模型、Qt 界面或飞控写入逻辑。这样可在硬件协议和产品 ICD 冻结前保持职责、依赖与控制边界稳定。

产品级 Topic、Action、Service、唯一发布者、既有 ROS 接口映射和安全默认值统一配置在 [pod_bringup/config/interfaces.yaml](pod_bringup/config/interfaces.yaml)，由 `pod_system.launch` 在启动时加载。

| 包 | 当前职责 |
|---|---|
| `pod_msgs` | 产品级消息、任务 Action 与错误/状态语义 |
| `pod_camera_driver` / `pod_video_pipeline` | 相机接入与视频链路边界 |
| `pod_detector` / `pod_tracker_adapter` | 检测识别接入与现有 `tracker` 适配 |
| `pod_gimbal_driver` / `pod_gimbal_controller` | 硬件协议与视觉伺服边界 |
| `pod_command_arbiter` / `pod_mission_manager` | 控制权仲裁与任务状态机 |
| `pod_geolocator` / `pod_uav_adapter` | 目标定位与 UAV 平台数据适配 |
| `pod_gateway` / `pod_ui` | 地面任务端稳定接口与 Qt/QML 客户端边界 |
| `pod_recorder` / `pod_replay` | 同步记录与回放 |
| `pod_sim` / `pod_bringup` | 仿真与系统启动入口 |

运行时安全规则：飞机控制默认关闭；如果未来启用，`pod_uav_adapter` 必须只选择一个后端（`xd`、`mrs` 或 `mavros_direct`），并由对应控制管理器保持最终 PX4 Topic 的单写入。
