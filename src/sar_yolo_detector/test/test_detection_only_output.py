#!/usr/bin/env python3
"""Fast contract checks for the direct YOLO -> XD conversion path."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
from sensor_msgs.msg import Image
from xd_uav_track.reid import AppearanceEncoder


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "detection_only_node.py"
SPEC = importlib.util.spec_from_file_location("detection_only_node", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DetectionOnlyOutputTest(unittest.TestCase):
    @staticmethod
    def detection(box=(10.2, 20.8, 80.1, 99.9), confidence=0.8):
        return SimpleNamespace(
            aabb_xyxy=box,
            confidence=confidence,
            class_id=2,
            rotation_deg=None,
        )

    def test_direct_candidate_has_no_upstream_identity(self):
        candidate = MODULE.DetectionOnlyNode._to_xd_candidate(
            self.detection(), 64, 96)
        self.assertEqual(candidate.track_id, -1)
        self.assertFalse(candidate.track_id_is_stable)
        self.assertEqual(candidate.class_id, 2)
        self.assertEqual(list(candidate.bbox), [10, 20, 64, 96])
        self.assertTrue(candidate.has_bbox)
        self.assertFalse(candidate.range_valid)

    def test_invalid_candidate_is_rejected(self):
        self.assertIsNone(MODULE.DetectionOnlyNode._to_xd_candidate(
            self.detection(box=(20, 20, 10, 30)), 64, 96))
        self.assertIsNone(MODULE.DetectionOnlyNode._to_xd_candidate(
            self.detection(confidence=float("nan")), 64, 96))

    def test_inline_reid_matches_tracker_owned_encoder(self):
        class Backend:
            def detect(self, *_args, **_kwargs):
                return "detect", [DetectionOnlyOutputTest.detection(
                    box=(16, 16, 112, 80))]

        class Publisher:
            def __init__(self):
                self.message = None

            def publish(self, message):
                self.message = message

        config = {
            "default_backend": "hybrid",
            "minimum_roi_width_px": 16,
            "minimum_roi_height_px": 16,
            "edge_margin_px": 2,
            "class_profiles": {
                "vehicle": {"class_ids": [2], "backend": "hybrid"},
            },
        }
        image = np.zeros((96, 128, 3), dtype=np.uint8)
        image[16:80, 16:112] = (30, 80, 210)
        message = Image()
        message.height, message.width = image.shape[:2]
        message.encoding = "bgr8"
        message.step = message.width * 3
        message.data = image.tobytes()

        node = MODULE.DetectionOnlyNode.__new__(MODULE.DetectionOnlyNode)
        node._backend = Backend()
        node._confidence = 0.6
        node._iou = 0.45
        node._maximum_detections = 30
        node._maximum_age_sec = 0.0
        node._publisher = None
        node._xd_publisher = Publisher()
        node._image_source = "fixed_rgb"
        node._sensor_id = "fixed_camera"
        node._detector_name = "sar_yolo_detector"
        node._model_version = "test"
        node._reid_encoder = AppearanceEncoder(config)
        node._reid_async = False
        node._maximum_reid_rois = 32
        node._encoded_embeddings = 0
        node._reid_processed = 0
        node._reid_ms_ewma = 0.0
        node._reid_backlog_drop = 0
        node._processed = 0
        node._received = 1
        node._dropped_backlog = 0
        node._skipped_without_subscriber = 0
        node._inference_ms_ewma = 0.0
        node._processing_ms_ewma = 0.0
        with mock.patch.object(MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time(0)), \
                mock.patch.object(MODULE.rospy, "loginfo_throttle"):
            node._process_image(message)

        candidate = node._xd_publisher.message.candidates[0]
        expected_encoder = AppearanceEncoder(config)
        expected = expected_encoder.encode(image, candidate.bbox, 2)
        np.testing.assert_array_equal(
            np.asarray(candidate.appearance_embedding, dtype=np.float32),
            expected)
        self.assertEqual(candidate.track_id, -1)
        self.assertFalse(candidate.track_id_is_stable)
        node._reid_encoder.close()
        expected_encoder.close()


if __name__ == "__main__":
    unittest.main()
