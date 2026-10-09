"""Python support for the sar_yolo_detector ROS package."""

# GPU wrappers may put this source directory before catkin's generated messages.
# Preserve both locations instead of shadowing sar_yolo_detector.msg/srv.
from pkgutil import extend_path
__path__ = extend_path(__path__, __name__)

__all__ = ["pixeagle"]
