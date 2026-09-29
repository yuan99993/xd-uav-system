#!/usr/bin/env python3
"""The shared TensorRT detector also supports a true single-camera output."""

import importlib.util
from pathlib import Path
import unittest

import numpy as np
from sensor_msgs.msg import Image
from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "multi_source_detection_node.py"
SPEC = importlib.util.spec_from_file_location("multi_source_detection_node", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SharedDetectorOutputTest(unittest.TestCase):
    @staticmethod
    def node(xd_publisher, vision_publisher):
        node = MODULE.MultiSourceDetectionNode.__new__(MODULE.MultiSourceDetectionNode)
        node._sources = {"fixed": {
            "xd_publisher": xd_publisher,
            "vision_publisher": vision_publisher,
            "image_source": "fixed_rgb",
            "sensor_id": "fixed_camera",
            "detector_name": "sar_yolo_detector",
            "model_version": "test",
            "reid_enabled": False,
        }}
        node._filter_oversized = lambda detections, *_: detections
        node._recover_with_prediction_roi = lambda _, __, detections: detections
        node._filter_detection_area = lambda _, detections, *__: detections
        node._add_pixel_assist = lambda *args: (args[-1], ())
        return node

    @staticmethod
    def input():
        image = Image()
        image.width, image.height = 128, 96
        image.header.stamp.secs = 12
        detection = NormalizedDetection(
            track_id=-1, track_id_is_stable=False, class_id=2,
            confidence=0.82, aabb_xyxy=(10, 20, 60, 70), center_xy=(35, 45))
        return image, detection

    def test_vision_only_output_does_not_build_xd_identity(self):
        vision = Publisher()
        node = self.node(None, vision)
        image, detection = self.input()
        node._publish("fixed", image, np.zeros((96, 128, 3), np.uint8), [detection])
        self.assertEqual(1, len(vision.messages))
        self.assertEqual(1, len(vision.messages[0].detections))
        self.assertEqual(2, vision.messages[0].detections[0].results[0].id)

    def test_xd_output_leaves_identity_to_tracker(self):
        xd = Publisher()
        node = self.node(xd, None)
        image, detection = self.input()
        node._publish("fixed", image, np.zeros((96, 128, 3), np.uint8), [detection])
        candidate, = xd.messages[0].candidates
        self.assertEqual(-1, candidate.track_id)
        self.assertFalse(candidate.track_id_is_stable)
        self.assertEqual("fixed_rgb", xd.messages[0].image_source)


if __name__ == "__main__":
    unittest.main()
