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

    def test_colour_tracker_uses_brightness_for_low_saturation_vehicle_roi(self):
        first = np.full((120, 160, 3), 24, dtype=np.uint8)
        second = first.copy()
        # A gray vehicle body has no useful hue; the brightness pattern still
        # differs from the nearby dark road and can provide bounded support.
        cv2.rectangle(first, (40, 30), (65, 55), (150, 150, 150), -1)
        cv2.rectangle(first, (46, 35), (59, 50), (205, 205, 205), -1)
        cv2.rectangle(second, (48, 35), (73, 60), (150, 150, 150), -1)
        cv2.rectangle(second, (54, 40), (67, 55), (205, 205, 205), -1)
        tracker = ManualBoxTracker({
            "tracker_type": "color_tracker",
            "feature_tracker": {"tracker_safety": {"edge_margin_px": 0.0}},
            "color_tracker": {"minimum_saturation": 40, "minimum_value": 20,
                              "backprojection_threshold": 8, "morph_kernel": 3},
        })
        self.assertTrue(tracker.initialize(first, (38, 28, 30, 30)).valid)
        self.assertEqual(tracker._colour_channel, 2)
        result = tracker.update(second)
        self.assertTrue(result.valid, result.reason)
        self.assertAlmostEqual(result.bbox[0], 48.0, delta=6.0)
        self.assertAlmostEqual(result.bbox[1], 35.0, delta=6.0)

    def test_zero_information_colour_roi_is_rejected(self):
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        tracker = ManualBoxTracker({"tracker_type": "color_tracker"})
        result = tracker.initialize(frame, (10, 10, 20, 20))

        self.assertFalse(result.valid)
        self.assertFalse(tracker.active)
        self.assertEqual(result.reason, "initial_roi_has_no_matching_colour")

    def test_black_roi_on_sand_remains_a_compact_foreground_box(self):
        first = np.full((120,180,3),(125,180,205),dtype=np.uint8)
        cv2.rectangle(first,(40,35),(70,60),(5,5,5),-1)
        tracker = ManualBoxTracker({'tracker_type':'color_tracker'})
        self.assertTrue(tracker.initialize(first,(40,35,31,26)).valid)
        for offset in range(1,8):
            frame = np.full_like(first,(125,180,205))
            cv2.rectangle(frame,(40+offset,35),(70+offset,60),(5,5,5),-1)
            result = tracker.update(frame)
            self.assertTrue(result.valid,result.reason)
            self.assertLess(result.bbox[2]*result.bbox[3],1.3*31*26)
            self.assertAlmostEqual(result.bbox[0],40+offset,delta=2.)

    def test_padded_gray_vehicle_roi_does_not_learn_the_grass_border(self):
        first = np.full((120,180,3),(25,145,25),dtype=np.uint8)
        cv2.rectangle(first,(40,35),(70,60),(80,80,80),-1)
        tracker = ManualBoxTracker({'tracker_type':'color_tracker'})
        self.assertTrue(tracker.initialize(first,(36,31,39,34)).valid)
        self.assertEqual(tracker._colour_channel,2)
        second = np.full_like(first,(25,145,25))
        cv2.rectangle(second,(45,37),(75,62),(80,80,80),-1)
        result = tracker.update(second)
        self.assertTrue(result.valid,result.reason)
        self.assertAlmostEqual(result.bbox[0],45.,delta=2.)
        self.assertLess(result.bbox[2]*result.bbox[3],1.3*31*26)

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

    def test_named_color_search_uses_shape_and_tracks_same_local_region(self):
        first = np.full((120, 200, 3), (25, 145, 25), dtype=np.uint8)
        second = first.copy()
        cv2.rectangle(first, (25, 35), (49, 53), (0, 0, 255), -1)
        cv2.rectangle(second, (29, 38), (53, 56), (0, 0, 255), -1)
        # A larger, equally red distractor outside the target's local search
        # window must not steal the active color lock.
        cv2.rectangle(second, (145, 35), (179, 67), (0, 0, 255), -1)
        tracker = ManualBoxTracker({"tracker_type": "color_tracker"})
        tracker.configure_colour("red")
        self.assertFalse(tracker.requires_initial_roi)
        first_result = tracker.update(first)
        self.assertTrue(first_result.valid, first_result.reason)
        second_result = tracker.update(second)
        self.assertTrue(second_result.valid, second_result.reason)
        self.assertAlmostEqual(second_result.bbox[0]+second_result.bbox[2]/2., 41.5, delta=3.)
        self.assertAlmostEqual(second_result.bbox[1]+second_result.bbox[3]/2., 47.5, delta=3.)
        self.assertEqual(tracker.tracker_name, "color_red_roi_camshift")

    def test_color_tracker_prefers_continuous_target_over_higher_area_distractor(self):
        first = np.full((120, 200, 3), (25, 145, 25), dtype=np.uint8)
        second = first.copy()
        cv2.rectangle(first, (25, 35), (49, 53), (0, 0, 255), -1)
        cv2.rectangle(second, (29, 38), (53, 56), (0, 0, 255), -1)
        cv2.rectangle(second, (64, 34), (94, 64), (0, 0, 255), -1)
        tracker = ManualBoxTracker({"tracker_type": "color_tracker",
            "color_tracker": {"color": "red", "search_expansion": 4.0}})
        self.assertTrue(tracker.update(first).valid)
        result = tracker.update(second)
        self.assertTrue(result.valid, result.reason)
        self.assertAlmostEqual(result.bbox[0]+result.bbox[2]/2., 41.5, delta=3.)

    def test_auto_color_search_finds_a_target_patch_over_dominant_grass(self):
        frame = np.full((120, 200, 3), (20, 145, 20), dtype=np.uint8)
        cv2.rectangle(frame, (45, 35), (75, 57), (0, 0, 255), -1)
        tracker = ManualBoxTracker({"tracker_type": "color_tracker",
                                    "color_tracker": {"color": "auto"}})
        self.assertEqual(tracker.colour_source, "auto")
        result = tracker.update(frame)
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(tracker._detected_colour_name, "red")
        self.assertTrue(tracker._roi_colour_tracking_active)
        self.assertIsNotNone(tracker._colour_histogram)
        self.assertAlmostEqual(result.bbox[0], 45., delta=1.)

    def test_auto_color_learns_roi_and_tracks_the_patch_between_frames(self):
        first = np.full((120, 200, 3), (20, 145, 20), dtype=np.uint8)
        second = first.copy()
        cv2.rectangle(first, (45, 35), (75, 57), (0, 0, 255), -1)
        cv2.rectangle(second, (51, 38), (81, 60), (0, 0, 255), -1)
        # A separate same-color distractor should not steal the learned ROI.
        cv2.rectangle(second, (145, 35), (180, 68), (0, 0, 255), -1)
        tracker = ManualBoxTracker({"tracker_type": "color_tracker",
                                    "color_tracker": {"color": "auto"}})
        acquired = tracker.update(first)
        self.assertTrue(acquired.valid, acquired.reason)
        self.assertTrue(tracker._roi_colour_tracking_active)
        tracked = tracker.update(second)
        self.assertTrue(tracked.valid, tracked.reason)
        self.assertIn("roi_camshift", tracked.tracker_name)
        self.assertAlmostEqual(tracked.bbox[0], 51., delta=8.)
        self.assertAlmostEqual(tracked.bbox[1], 38., delta=8.)

    def test_global_color_rejects_a_color_that_covers_the_background(self):
        frame = np.full((100, 140, 3), (0, 0, 255), dtype=np.uint8)
        tracker = ManualBoxTracker({"tracker_type": "color_tracker"})
        tracker.configure_colour("red")
        result = tracker.update(frame)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "selected_colour_dominates_background")

    def test_global_color_rejects_a_thin_fragment_with_vehicle_shape_gate(self):
        frame = np.full((120, 160, 3), (30, 140, 30), dtype=np.uint8)
        cv2.rectangle(frame, (60, 15), (65, 95), (0, 0, 255), -1)
        tracker = ManualBoxTracker({"tracker_type": "color_tracker"})
        tracker.configure_colour("red")
        result = tracker.update(frame)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "no_shape_like_contrasting_colour_region")

    def test_colour_configuration_rejects_invalid_name_atomically(self):
        tracker = ManualBoxTracker({"tracker_type": "color_tracker"})
        with self.assertRaises(ValueError):
            tracker.configure_colour("orange")

        self.assertEqual(tracker.colour_source, "roi")
        self.assertEqual(tracker.colour_name, "")


if __name__ == "__main__":
    unittest.main()
