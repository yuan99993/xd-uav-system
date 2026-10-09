#!/usr/bin/env python3
"""Compose per-camera tracking overlays into one latest-only ROS image topic."""

from __future__ import annotations

import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CombinedTrackingView:
    def __init__(self) -> None:
        self._left_topic = str(rospy.get_param(
            "~fixed_overlay_topic", "fixed_camera/tracking_overlay"))
        self._right_topic = str(rospy.get_param(
            "~gimbal_overlay_topic", "gimbal_camera/tracking_overlay"))
        self._output_topic = str(rospy.get_param(
            "~output_image_topic", "tracking_visualization"))
        self._left_title = str(rospy.get_param("~fixed_title", "FIXED CAMERA"))
        self._right_title = str(rospy.get_param("~gimbal_title", "GIMBAL CAMERA"))
        self._enable_fixed = bool(rospy.get_param("~enable_fixed", True))
        self._enable_gimbal = bool(rospy.get_param("~enable_gimbal", True))
        if not (self._enable_fixed or self._enable_gimbal):
            raise ValueError("at least one camera view must be enabled")
        self._panel_height = max(120, min(1080, int(rospy.get_param(
            "~panel_height", 480))))
        self._stale_after_sec = max(0.2, min(10.0, float(rospy.get_param(
            "~stale_after_sec", 1.5))))
        self._rate_hz = max(0.5, min(10.0, float(rospy.get_param(
            "~publish_rate_hz", 3.0))))

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest = {"fixed": None, "gimbal": None}
        self._publisher = rospy.Publisher(
            self._output_topic, Image, queue_size=1, tcp_nodelay=True)
        self._subscribers = []
        for enabled, topic, source in (
                (self._enable_fixed, self._left_topic, "fixed"),
                (self._enable_gimbal, self._right_topic, "gimbal")):
            if enabled:
                self._subscribers.append(rospy.Subscriber(
                    topic, Image, self._image_callback, callback_args=source,
                    queue_size=1, buff_size=16 * 1024 * 1024,
                    tcp_nodelay=True))
        self._timer = rospy.Timer(rospy.Duration(1.0 / self._rate_hz),
                                  self._publish_latest)
        rospy.loginfo(
            "combined tracking view ready: fixed=%s gimbal=%s output=%s rate=%.1fHz",
            self._left_topic, self._right_topic, self._output_topic,
            self._rate_hz)

    def _image_callback(self, message: Image, source: str) -> None:
        with self._lock:
            previous = self._latest[source]
            if (previous is not None and not message.header.stamp.is_zero() and
                    not previous.header.stamp.is_zero() and
                    message.header.stamp <= previous.header.stamp):
                return
            # Decode only the latest frame at display rate, not every camera
            # frame. The per-camera overlay remains responsible for box drawing.
            self._latest[source] = message

    @staticmethod
    def _placeholder(height: int, width: int, title: str, note: str) -> np.ndarray:
        panel = np.full((height, width, 3), 34, dtype=np.uint8)
        cv2.putText(panel, title, (18, 34), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (230, 230, 230), 2, cv2.LINE_AA)
        cv2.putText(panel, note, (18, min(height - 16, 72)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (160, 180, 200),
                    1, cv2.LINE_AA)
        return panel

    def _panel(self, entry, title: str, now: rospy.Time):
        if entry is None:
            return self._placeholder(self._panel_height, 640, title,
                                     "waiting for camera overlay"), None
        message = entry
        stamp = message.header.stamp
        age = (now - stamp).to_sec() if not stamp.is_zero() else 0.0
        if age < -0.1 or age > self._stale_after_sec:
            panel = self._placeholder(
                max(1, message.height), max(1, message.width), title,
                "overlay stale ({:.1f}s)".format(max(0.0, age)))
            return panel, message

        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "combined view cannot decode %s: %s",
                                   title, error)
            return self._placeholder(max(1, message.height),
                                     max(1, message.width), title,
                                     "image decode failed"), message
        source_height, source_width = frame.shape[:2]
        if source_height <= 0 or source_width <= 0:
            return self._placeholder(self._panel_height, 640, title,
                                     "empty image"), message
        # Never resize a source panel: its pixel coordinates must remain the
        # native camera/DetectionArray coordinates even in a dual view.
        return frame, message

    def _publish_latest(self, _event) -> None:
        with self._lock:
            fixed = self._latest["fixed"]
            gimbal = self._latest["gimbal"]
        now = rospy.Time.now()
        panels = []
        if self._enable_fixed:
            panels.append(self._panel(fixed, self._left_title, now))
        if self._enable_gimbal:
            panels.append(self._panel(gimbal, self._right_title, now))
        if len(panels) == 1:
            composite = panels[0][0]
        else:
            height = max(panel.shape[0] for panel, _ in panels)
            width = sum(panel.shape[1] for panel, _ in panels)
            composite = np.full((height, width, 3), 34, dtype=np.uint8)
            x_offset = 0
            for panel, _ in panels:
                panel_height, panel_width = panel.shape[:2]
                composite[:panel_height, x_offset:x_offset + panel_width] = panel
                x_offset += panel_width

        output = self._bridge.cv2_to_imgmsg(composite, encoding="bgr8")
        output.header.stamp = now
        output.header.frame_id = "tracking_visualization"
        self._publisher.publish(output)


def main() -> None:
    rospy.init_node("combined_tracking_view")
    CombinedTrackingView()
    rospy.spin()


if __name__ == "__main__":
    main()
