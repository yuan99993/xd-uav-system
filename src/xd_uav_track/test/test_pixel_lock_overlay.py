#!/usr/bin/env python3
"""Multi-target pixel continuity must not promote a delayed detector jump."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import rospy
from xd_uav_track.msg import TrackState, TrackStateArray
from xd_uav_track.pixel_lock import (blend_boxes, coast_box, color_box, flow_box,
                                      compatible_box, turn_compatible_box)


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "tracking_overlay_node.py"
SPEC = importlib.util.spec_from_file_location("tracking_overlay_node", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def selected_tracks(stamp, box, track_id=7):
    output = TrackStateArray()
    output.header.stamp = rospy.Time.from_sec(stamp)
    output.image_source = "benchmark_fixed"
    track = TrackState()
    track.track_id = track_id
    track.selected = True
    track.detected = True
    track.lifecycle_state = "confirmed"
    track.bbox = box
    output.tracks = [track]
    return output


class PixelLockOverlayTest(unittest.TestCase):
    def test_detector_correction_is_bounded(self):
        self.assertTrue(compatible_box((10, 20, 50, 60), (13, 20, 53, 60)))
        self.assertFalse(compatible_box((10, 20, 50, 60), (35, 20, 75, 60)))
        self.assertTrue(turn_compatible_box(
            (10, 20, 50, 60), (15, 25, 80, 50)))
        self.assertFalse(turn_compatible_box(
            (10, 20, 50, 60), (70, 20, 110, 60)))
        self.assertEqual(blend_boxes((10, 20, 50, 60),
                                     (14, 20, 54, 60), 0.25),
                         (11, 20, 51, 60))
        self.assertEqual(coast_box((10, 20, 50, 60), (20, 0), 0.2,
                                   (80, 100)), (14, 20, 54, 60))
        self.assertIsNone(coast_box((10, 20, 50, 60), (20, 0), 0.7,
                                     (80, 100)))

    def test_color_fallback_follows_a_target_on_neutral_ground(self):
        previous = np.full((100, 160, 3), 180, dtype=np.uint8)
        current = previous.copy()
        previous[30:60, 30:80] = (40, 110, 80)
        current[30:60, 38:88] = (40, 110, 80)
        self.assertEqual(color_box(previous, current, (30, 30, 80, 60),
                                   0.2), (38, 30, 88, 60))
        self.assertIsNone(color_box(
            previous, np.full_like(previous, 180), (30, 30, 80, 60), 0.2))

    def test_velocity_lead_seeds_lk_search_but_keeps_forward_backward_gate(self):
        points = np.array([[[15, 25]], [[25, 25]], [[35, 25]],
                           [[15, 35]], [[25, 35]], [[35, 35]]], np.float32)
        calls = []

        def fake_flow(_previous, _current, old, guess=None, **options):
            calls.append(options.get("flags", 0))
            if len(calls) == 1:
                self.assertEqual(options["flags"], cv2.OPTFLOW_USE_INITIAL_FLOW)
                self.assertAlmostEqual(float((guess - old)[0, 0, 0]), 7.0)
                return guess, np.ones((len(old), 1), np.uint8), None
            return points.copy(), np.ones((len(old), 1), np.uint8), None

        gray = np.zeros((80, 100), np.uint8)
        with patch.object(cv2, "goodFeaturesToTrack", return_value=points), \
                patch.object(cv2, "calcOpticalFlowPyrLK", side_effect=fake_flow):
            moved = flow_box(gray, gray, (10, 20, 50, 60), 0.1,
                             min_features=4, image_velocity=(70.0, 0.0))
        self.assertEqual(moved, (17, 20, 57, 60))
        self.assertEqual(calls, [cv2.OPTFLOW_USE_INITIAL_FLOW, 0])

    def test_same_id_keeps_flow_box_across_a_bad_yolo_box(self):
        overlay = MODULE.TrackingOverlay.__new__(MODULE.TrackingOverlay)
        overlay._pixel_locks = {}
        overlay._pixel_lock_max_tracks = 8
        overlay._pixel_lock_max_gap_sec = 0.5
        overlay._pixel_lock_max_unanchored_sec = 2.0
        gray = np.zeros((80, 100), np.uint8)
        initial = selected_tracks(1.0, [10, 20, 50, 60])
        self.assertEqual(overlay._advance_pixel_locks(
            gray, 1.1, initial, 0.1)[7], (10, 20, 50, 60))
        with patch.object(MODULE, "flow_box", return_value=(12, 20, 52, 60)):
            boxes = overlay._advance_pixel_locks(
                gray, 1.2, initial, 0.2)
        self.assertEqual(boxes[7], (12, 20, 52, 60))
        jumped = selected_tracks(1.25, [35, 20, 75, 60])
        with patch.object(MODULE, "flow_box", return_value=(14, 20, 54, 60)):
            boxes = overlay._advance_pixel_locks(
                gray, 1.3, jumped, 0.05)
        self.assertEqual(boxes[7], (14, 20, 54, 60))
        self.assertEqual(overlay._pixel_locks[7]["anchor_stamp"], 1.0)

    def test_no_image_evidence_clears_box(self):
        overlay = MODULE.TrackingOverlay.__new__(MODULE.TrackingOverlay)
        overlay._pixel_locks = {}
        overlay._pixel_lock_max_tracks = 8
        overlay._pixel_lock_max_gap_sec = 0.5
        overlay._pixel_lock_max_unanchored_sec = 2.0
        gray = np.zeros((80, 100), np.uint8)
        initial = selected_tracks(1.0, [10, 20, 50, 60])
        overlay._advance_pixel_locks(gray, 1.1, initial, 0.1)
        with patch.object(MODULE, "flow_box", return_value=None):
            self.assertEqual(overlay._advance_pixel_locks(
                gray, 1.9, initial, None), {})

    def test_one_flow_miss_uses_short_bounded_predicted_box(self):
        overlay = MODULE.TrackingOverlay.__new__(MODULE.TrackingOverlay)
        overlay._pixel_locks = {}
        overlay._pixel_lock_max_tracks = 8
        overlay._pixel_lock_max_gap_sec = 0.5
        overlay._pixel_lock_max_unanchored_sec = 2.0
        gray = np.zeros((80, 100), np.uint8)
        initial = selected_tracks(1.0, [10, 20, 50, 60])
        initial.tracks[0].image_velocity = [20.0, 0.0]
        overlay._advance_pixel_locks(gray, 1.1, initial, 0.1)
        with patch.object(MODULE, "flow_box", return_value=None):
            boxes = overlay._advance_pixel_locks(gray, 1.3, initial, 0.3)
        self.assertEqual(boxes[7], (14, 20, 54, 60))
        self.assertEqual(overlay._pixel_predicted_ids, {7})

    def test_two_tracks_keep_separate_pixel_boxes(self):
        overlay = MODULE.TrackingOverlay.__new__(MODULE.TrackingOverlay)
        overlay._pixel_locks = {}
        overlay._pixel_lock_max_tracks = 8
        overlay._pixel_lock_max_gap_sec = 0.5
        overlay._pixel_lock_max_unanchored_sec = 2.0
        gray = np.zeros((80, 160), np.uint8)
        tracks = selected_tracks(1.0, [10, 20, 40, 50])
        second = TrackState()
        second.track_id = 8
        second.detected = True
        second.lifecycle_state = "tentative"
        second.bbox = [100, 20, 130, 50]
        tracks.tracks.append(second)
        boxes = overlay._advance_pixel_locks(gray, 1.1, tracks, 0.1)
        self.assertEqual(boxes, {7: (10, 20, 40, 50),
                                 8: (100, 20, 130, 50)})
        with patch.object(MODULE, "flow_box", side_effect=[
                (12, 20, 42, 50), (102, 20, 132, 50)]):
            boxes = overlay._advance_pixel_locks(gray, 1.2, tracks, 0.2)
        self.assertEqual(boxes[7], (12, 20, 42, 50))
        self.assertEqual(boxes[8], (102, 20, 132, 50))


if __name__ == "__main__":
    unittest.main()
