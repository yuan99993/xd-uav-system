# UE 网页演示

入口为 `roslaunch sar_yolo_detector ue_web_demo.launch`。此目录随功能包安装，包含演示网页、辅助框代码、启动配置、模型和 UE 演示资产。

1. 构建并载入两个 ROS Noetic 功能包。按 [D-FINE 环境说明](../../docs/dfine_v3_deployment.md) 准备模型 Python。
2. 使用原有的 UE 5.5 AirGround 项目。将本目录 `ue_project_overlay/Content` 合并到 UE 项目的 `Content`，保留目录层级。演示地图引用原项目的车辆、场景和网格，资产补丁不能单独用作空白 UE 工程。按 [UE 相机安装说明](../../docs/ue_low_latency_stream.md) 安装采集插件和编辑器启动脚本，启动 UE 编辑器。
3. 启动演示，传入实际项目路径与模型解释器路径。

```bash
roslaunch sar_yolo_detector ue_web_demo.launch \
  ue_project:=/path/to/UE/project \
  python_executable:=/path/to/dfine-env/bin/python
```

打开 http://127.0.0.1:8787/ 。如在 WSL 使用专用 ROS 地址，按原环境配置 ROS 网络。演示使用独立的默认输出目录。模型配置保留训练数据路径作为来源记录，演示推理无需这些数据文件。
