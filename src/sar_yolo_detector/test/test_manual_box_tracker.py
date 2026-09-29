#!/usr/bin/env python3
"""Deterministic tests for manually initialized image tracking."""

import unittest

import cv2
import numpy as np

from sar_yolo_detector.manual_box_tracker import ManualBoxTracker


class ManualBoxTrackerTest(unittest.TestCase):
    def test_template_tracker_follows_a_textured_roi(self):
        random = np.random.RandomState(7)
        texture = random.randint(0, 256, (20, 24, 3), dtype=np.uint8)
        first = np.zeros((120, 160, 3), dtype=np.uint8)
        second = np.zeros_like(first)
        first[30:50, 40:64] = texture
        second[36:56, 49:73] = texture

        tracker = ManualBoxTracker(
            {
                "tracker_type": "feature_tracker",
                "feature_tracker": {
                    "tracker_algorithm": "template",
                    "template_match_threshold": 0.4,
                    "template_search_expansion": 4.0,
                    "tracker_safety": {"edge_margin_px": 0.0},
                },
            }
        )
        self.assertTrue(tracker.initialize(first, (40, 30, 24, 20)).valid)
        result = tracker.update(second)

        self.assertTrue(result.valid, result.reason)
        self.assertAlmostEqual(result.bbox[0], 49.0, delta=1.0)
        self.assertAlmostEqual(result.bbox[1], 36.0, delta=1.0)

    def test_colour_tracker_learns_colour_from_initial_roi(self):
        first = np.zeros((120, 160, 3), dtype=np.uint8)
        second = np.zeros_like(first)
        cv2.rectangle(first, (40, 30), (65, 55), (0, 255, 0), -1)
        cv2.rectangle(second, (48, 35), (73, 60), (0, 255, 0), -1)

        tracker = ManualBoxTracker(
            {
                "tracker_type": "color_tracker",
                "feature_tracker": {
                    "tracker_safety": {"edge_margin_px": 0.0}
                },
                "color_tracker": {
                    "minimum_saturation": 40,
                    "minimum_value": 30,
                    "backprojection_threshold": 8,
                    "morph_kernel": 3,
                },
            }
        )
        self.assertTrue(tracker.initialize(first, (38, 28, 30, 30)).valid)
        result = tracker.update(second)

        self.assertTrue(result.valid, result.reason)
        self.assertAlmostEqual(result.bbox[0], 48.0, delta=5.0)
        self.assertAlmostEqual(result.bbox[1], 35.0, delta=5.0)

    def test_zero_information_colour_roi_is_rejected(self):
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        tracker = ManualBoxTracker({"tracker_type": "color_tracker"})
        result = tracker.initialize(frame, (10, 10, 20, 20))

        self.assertFalse(result.valid)
        self.assertFalse(tracker.active)
        self.assertEqual(result.reason, "initial_roi_has_no_matching_colour")

    def test_configured_colour_detects_selected_colour_without_roi(self):
        first = np.zeros((120, 180, 3), dtype=np.uint8)
        second = np.zeros_like(first)
        cv2.rectangle(first, (25, 30), (55, 60), (255, 0, 0), -1)
        cv2.rectangle(first, (110, 30), (140, 60), (0, 255, 0), -1)
        cv2.rectangle(second, (35, 38), (65, 68), (255, 0, 0), -1)
        cv2.rectangle(second, (100, 38), (130, 68), (0, 255, 0), -1)

        tracker = ManualBoxTracker(
            {
                "tracker_type": "color_tracker",
                "feature_tracker": {
                    "tracker_safety": {"edge_margin_px": 0.0}
                },
                "color_tracker": {
                    "color": "blue",
                    "backprojection_threshold": 8,
                    "morph_kernel": 3,
                },
            }
        )
        self.assertFalse(tracker.requires_initial_roi)
        self.assertTrue(tracker.update(first).valid)
        result = tracker.update(second)

        self.assertTrue(result.valid, result.reason)
        self.assertEqual(tracker.colour_source, "configured")
        self.assertEqual(tracker.colour_name, "blue")
        center_x = result.bbox[0] + result.bbox[2] * 0.5
        center_y = result.bbox[1] + result.bbox[3] * 0.5
        self.assertAlmostEqual(center_x, 50.0, delta=6.0)
        self.assertAlmostEqual(center_y, 53.0, delta=6.0)

    def test_colour_configuration_rejects_invalid_name_atomically(self):
        tracker = ManualBoxTracker({"tracker_type": "color_tracker"})
        with self.assertRaises(ValueError):
            tracker.configure_colour("orange")

        self.assertEqual(tracker.colour_source, "roi")
        self.assertEqual(tracker.colour_name, "")


if __name__ == "__main__":
    unittest.main()
