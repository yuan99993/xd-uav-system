#!/usr/bin/env python3
"""One final overlay topic must work with either one or two cameras."""

import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from cv_bridge import CvBridge


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "combined_tracking_view_node.py"
SPEC = importlib.util.spec_from_file_location("combined_tracking_view_node", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class CombinedTrackingViewTest(unittest.TestCase):
    @staticmethod
    def view(dual):
        view = MODULE.CombinedTrackingView.__new__(MODULE.CombinedTrackingView)
        view._bridge = CvBridge()
        view._lock = threading.Lock()
        view._latest = {"fixed": None, "gimbal": None}
        view._publisher = _Publisher()
        view._enable_fixed = True
        view._enable_gimbal = dual
        view._panel_height = 120
        view._stale_after_sec = 1.5
        view._left_title = "FIXED"
        view._right_title = "GIMBAL"
        return view

    @staticmethod
    def frame(value, stamp, width=160, height=120):
        message = CvBridge().cv2_to_imgmsg(
            np.full((height, width, 3), value, np.uint8), encoding="bgr8")
        message.header.stamp = rospy.Time.from_sec(stamp)
        return message

    def test_single_and_dual_camera_publish_one_image_each(self):
        for dual, expected_width in ((False, 160), (True, 320)):
            view = self.view(dual)
            view._image_callback(self.frame(20, 10.0), "fixed")
            if dual:
                view._image_callback(self.frame(80, 10.0), "gimbal")
            with patch.object(MODULE.rospy.Time, "now",
                              return_value=rospy.Time.from_sec(10.1)):
                view._publish_latest(None)
            self.assertEqual(len(view._publisher.messages), 1)
            output = view._publisher.messages[0]
            self.assertEqual((output.width, output.height),
                             (expected_width, 120))
            self.assertEqual(output.header.frame_id, "tracking_visualization")

    def test_late_frame_cannot_roll_view_backward(self):
        view = self.view(False)
        newest = self.frame(80, 10.0)
        view._image_callback(newest, "fixed")
        view._image_callback(self.frame(20, 9.0), "fixed")
        self.assertIs(view._latest["fixed"], newest)

    def test_mixed_resolutions_are_padded_without_pixel_scaling(self):
        view = self.view(True)
        view._image_callback(self.frame(20, 10.0, 160, 120), "fixed")
        view._image_callback(self.frame(80, 10.0, 80, 90), "gimbal")
        with patch.object(MODULE.rospy.Time, "now",
                          return_value=rospy.Time.from_sec(10.1)):
            view._publish_latest(None)
        output = view._publisher.messages[0]
        canvas = CvBridge().imgmsg_to_cv2(output, desired_encoding="bgr8")
        self.assertEqual((output.width, output.height), (240, 120))
        self.assertEqual(tuple(canvas[20, 20]), (20, 20, 20))
        self.assertEqual(tuple(canvas[20, 180]), (80, 80, 80))
        self.assertEqual(tuple(canvas[110, 180]), (34, 34, 34))


if __name__ == "__main__":
    unittest.main()
