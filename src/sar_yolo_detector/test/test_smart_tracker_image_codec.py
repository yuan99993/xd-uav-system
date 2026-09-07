#!/usr/bin/env python3
"""Tests for the cv_bridge-free SmartTracker ROS image boundary."""

import importlib.util
import unittest
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs.msg import Image


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "smart_tracker_node.py"
SPEC = importlib.util.spec_from_file_location("smart_tracker_node", str(SCRIPT))
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


class SmartTrackerImageCodecTest(unittest.TestCase):
    def test_bgr8_padding_is_removed(self):
        message = Image()
        message.height = 2
        message.width = 2
        message.encoding = "bgr8"
        message.step = 8
        message.data = bytes([
            1, 2, 3, 4, 5, 6, 99, 99,
            7, 8, 9, 10, 11, 12, 99, 99,
        ])
        decoded = NODE._image_to_bgr(message)
        self.assertEqual(decoded.shape, (2, 2, 3))
        self.assertEqual(decoded.tolist(), [
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]],
        ])

    def test_rgb_and_mono16_are_converted_to_bgr(self):
        rgb = Image()
        rgb.height = 1
        rgb.width = 1
        rgb.encoding = "rgb8"
        rgb.step = 3
        rgb.data = bytes([10, 20, 30])
        self.assertEqual(NODE._image_to_bgr(rgb).tolist(), [[[30, 20, 10]]])

        mono = Image()
        mono.height = 2
        mono.width = 2
        mono.encoding = "mono16"
        mono.step = 4
        mono.is_bigendian = 0
        mono.data = np.array([[0, 100], [200, 300]], dtype="<u2").tobytes()
        decoded = NODE._image_to_bgr(mono)
        self.assertEqual(decoded.shape, (2, 2, 3))
        self.assertEqual(decoded.dtype, np.uint8)
        self.assertTrue(np.array_equal(decoded[:, :, 0], decoded[:, :, 1]))

    def test_malformed_image_fails_closed(self):
        message = Image()
        message.height = 2
        message.width = 2
        message.encoding = "bgr8"
        message.step = 6
        message.data = bytes(6)
        with self.assertRaisesRegex(ValueError, "shorter"):
            NODE._image_to_bgr(message)

    def test_annotated_output_is_tightly_packed(self):
        header = Image().header
        header.stamp = rospy.Time.from_sec(123.5)
        frame = np.zeros((3, 4, 3), dtype=np.uint8)
        message = NODE._bgr_to_image(frame, header)
        self.assertEqual(message.header.stamp, header.stamp)
        self.assertEqual(message.encoding, "bgr8")
        self.assertEqual(message.step, 12)
        self.assertEqual(len(message.data), 36)


if __name__ == "__main__":
    unittest.main()
