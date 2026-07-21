# xd_demo_cpp

一个用于验证开发流程的 ROS1 C++ 示例功能包。

## 功能

`heartbeat_node` 按配置频率发布 `std_msgs/String` 消息，默认话题为：

```text
/xd_demo_cpp/heartbeat
```

## 编译

在仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 运行

```bash
roslaunch xd_demo_cpp heartbeat.launch
```

另开终端查看消息：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo /xd_demo_cpp/heartbeat
```

## 参数

参数文件位于 `config/params.yaml`：

```yaml
rate_hz: 1.0
topic: /xd_demo_cpp/heartbeat
```
