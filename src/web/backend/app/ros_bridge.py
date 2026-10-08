"""ROS 桥接模块：通过 rosbridge_suite 把路径下发到机载 EGO 规划包。

为什么需要这个模块：
    Web 后端生成路径后，要把路径送给固定翼机载端。
    机载端跑的是 ROS，Web 不能直接发 ROS 消息（ROS 是 C++/Python 原生 socket）。
    标准方案是 rosbridge_suite：
        机载端启动 rosbridge 节点 → 暴露 WebSocket 端口（默认 9090）
        Web 后端通过 WebSocket 发 JSON → rosbridge 转成 ROS 消息 → 发到 topic

    前端也可以用 roslibjs 直接发，但我们放后端发更可控（失败重试、日志）。

对应 ROS 消息：
    planner_msgs/Path
        std_msgs/Header header
        planner_msgs/PathSegment[] segments
    PathSegment:
        bool reached
        bool periodic
        std_msgs/Float64 segment_length
        geometry_msgs/Vector3 segment_start
        geometry_msgs/Vector3 segment_tangent
        std_msgs/Float64 curvature

后续待和师兄确认：
    1. ROS1 还是 ROS2（影响 rosbridge 协议）
    2. EGO 包订阅的 topic 名字（暂用 /terrain_path）
    3. EGO 期望的消息结构是否和 planner_msgs/Path 一致（不一致就按 EGO 来）
"""
from __future__ import annotations

import logging
from typing import List

from .planner import PathSegment

logger = logging.getLogger(__name__)


class MockRosBridge:
    """假桥接：当没接机载 ROS 时，仅打印路径数据。

    作用：
        1. 框架阶段验证 Web 端能生成完整 Path 消息
        2. 接机载端时只改这个类，调用方不变
    """

    def send_path(self, topic: str, segments: List[PathSegment]) -> dict:
        logger.info("[MockRosBridge] 将向 topic=%s 下发 %d 段路径", topic, len(segments))
        return {"status": "mock_ok", "topic": topic, "n_segments": len(segments)}


class RosbridgeClient:
    """真桥接：通过 roslibpy 连 rosbridge_server。

    用法：
        client = RosbridgeClient(host="192.168.1.100", port=9090)
        client.send_path("/terrain_path", segments)

    启动 rosbridge（在机载端）：
        ROS1: sudo apt install ros-<distro>-rosbridge-server
              roslaunch rosbridge_server rosbridge_websocket.launch
        ROS2: 用 rosbridge_suite-ros2 版本
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 9090):
        self.host = host
        self.port = port
        self._client = None  # 懒加载，避免无 rosbridge 时初始化失败

    def _ensure_client(self):
        if self._client is None:
            try:
                import roslibpy
            except ImportError:
                raise RuntimeError(
                    "roslibpy 未安装，pip install roslibpy；"
                    "或先使用 MockRosBridge 跑框架阶段"
                )
            self._client = roslibpy.Ros(host=self.host, port=self.port)
            self._client.run()
        return self._client

    def send_path(self, topic: str, segments: List[PathSegment]) -> dict:
        ros = self._ensure_client()
        # 消息体对齐 planner_msgs/Path
        msg = {
            "header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "map"},
            "segments": [
                {
                    "reached": s.reached,
                    "periodic": s.is_periodic,
                    "segment_length": s.segment_length,
                    "segment_start": {"x": s.segment_start[0],
                                      "y": s.segment_start[1],
                                      "z": s.segment_start[2]},
                    "segment_tangent": {"x": s.segment_tangent[0],
                                         "y": s.segment_tangent[1],
                                         "z": s.segment_tangent[2]},
                    "curvature": s.curvature,
                }
                for s in segments
            ],
        }
        publisher = roslibpy.Topic(ros, topic, "planner_msgs/Path")
        publisher.advertise()
        publisher.publish(msg)
        return {"status": "sent", "topic": topic, "n_segments": len(segments)}


def get_bridge(use_mock: bool = True, host: str = "127.0.0.1", port: int = 9090):
    """工厂函数：根据配置选择真桥接还是假桥接。

    框架阶段用假桥接（默认），机载端联调时切到真桥接。
    """
    if use_mock:
        return MockRosBridge()
    return RosbridgeClient(host=host, port=port)
