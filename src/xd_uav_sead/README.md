# xd_uav_sead

面向当前工作空间 `xd_uav` ROS 包的 SEAD 任务节点。包内负责通信协议、任务分配、路径规划、编队与打击时序；状态估计、PX4 模式管理和飞行控制仍由现有 `xd_uav` 功能包负责。

## 边界

SEAD 只写入以下 xd 接口：

- `/<uav>/control/reference/setpoint` (`mavros_msgs/PositionTarget`)
- `/<uav>/control_manager/takeoff`
- `/<uav>/control_manager/offboard`
- `/<uav>/control_manager/land`
- `/<uav>/control_manager/land_home`

它读取 `/<uav>/control_manager/state`，并只读 MAVROS 的连接/解锁状态和电池信息。包内不再包含直接 MAVROS 控制、SwiftWing 专用控制、GPS 原点换算、估计器参数或控制器增益，也不会启动其他功能包。

## 通信

三种入口最终都进入原有 `PacketProtocol` 解析与同一个任务状态机：

- `xbee`：DigiMesh/XBee 原始二进制包。
- `udp`：每个 UDP 数据报承载一个未经封装或改写的原始二进制包，可直接配合已有地面站。
- `ros`：`/<uav>/sead/command_raw` 与 `telemetry_raw` 使用 `std_msgs/UInt8MultiArray` 原样承载 PacketProtocol，已有地面站编解码可直接复用。`command`/`telemetry` 的 JSON 接口继续供兼容和调试使用，机间总线为 `/sead/u2u`。

通信地址、SEAD 任务、编队和日志参数集中在 `config/sead_defaults.yaml`。UDP 多机在同一主机运行时，每个节点必须配置不同的 `communication/udp/bind_port`。

## 启动

先按当前工作空间原有方式启动估计器、manager 和 controller，再单独启动 SEAD：

```bash
source devel/setup.bash
roslaunch xd_uav_sead sead_onboard.launch \
  UAV_NAME:=uav1 communication_mode:=ros sead_runtime_mode:=simple_strike
```

使用 UDP 或 XBee 时把 `communication_mode` 改为 `udp` 或 `xbee`。配置文件可以通过 `config:=/absolute/path/to/sead.yaml` 覆盖；它应只包含 SEAD 私有参数。

ROS 入口的快速测试示例：

```bash
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=takeoff _alt:=80
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=sead_mission \
  _targets_json:='[[100,0],[200,50],[300,100]]'
```

协议中的 `Arm` 和 `Origin_Correction` 消息编号继续保留，以兼容现有地面站；当前适配器会明确拒绝直接解锁，并忽略外部 GPS 原点修正，因为解锁和本地坐标系由 xd manager/estimator 管理。
