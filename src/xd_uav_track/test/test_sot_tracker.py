#!/usr/bin/env python3
"""Manual ROI tracking remains bounded and reports its actual algorithm."""

import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from xd_uav_track.sot_tracker import SotTracker


class SotTrackerTest(unittest.TestCase):
    def test_gradual_scale_collapse_is_rejected_against_trusted_anchor(self):
        image = np.random.RandomState(1).randint(0, 255, (100, 100, 3)).astype(np.uint8)
        tracker = SotTracker('template')
        self.assertTrue(tracker.initialize(image, (20, 20, 40, 40), 1.).valid)
        # Each step is legal relative to its predecessor, but not to the
        # trusted full-object scale. Do not accept an eventual turret-only box.
        for index, side in enumerate((36, 32, 28, 24), 1):
            tracker._tracker = SimpleNamespace(update=lambda frame, s=side: (True, (20, 20, s, s)))
            result = tracker.update(image, 1.+index*.1)
            if side >= 28:
                self.assertTrue(result.valid)
            else:
                self.assertFalse(result.valid)
                self.assertEqual('anchor_scale_gate', result.reason)
                self.assertIsNone(result.bbox)

    def test_verified_anchor_refresh_does_not_recreate_tracker(self):
        image = np.random.RandomState(1).randint(0, 255, (100, 100, 3)).astype(np.uint8)
        tracker = SotTracker('template')
        tracker.initialize(image, (20, 20, 40, 40), 1.)
        tracker._tracker = SimpleNamespace(update=lambda frame: (True, (20, 20, 32, 32)))
        self.assertTrue(tracker.update(image, 1.1).valid)
        backend = SimpleNamespace(update=lambda frame: (True, (20, 20, 24, 24)))
        tracker._tracker = backend
        self.assertTrue(tracker.accept_anchor((20, 20, 28, 28)))
        self.assertTrue(tracker.update(image, 1.2).valid)
        self.assertIs(backend, tracker._tracker)
        self.assertFalse(tracker.accept_anchor((20, 20, float('nan'), 28)))

    def test_template_fallback_tracks_a_small_translation(self):
        first = np.zeros((120, 180, 3), np.uint8)
        second = first.copy()
        pattern = np.zeros((24, 36, 3), np.uint8)
        cv2.line(pattern, (0, 0), (35, 23), (255, 255, 255), 2)
        cv2.circle(pattern, (10, 15), 5, (80, 180, 220), -1)
        first[40:64, 50:86] = pattern
        second[44:68, 57:93] = pattern
        tracker = SotTracker("template", template_threshold=0.7)
        initialized = tracker.initialize(first, (50, 40, 36, 24), 1.0)
        self.assertTrue(initialized.valid)
        self.assertEqual("template", initialized.algorithm)
        result = tracker.update(second, 1.1)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(57.0, result.bbox[0], delta=1.0)
        self.assertAlmostEqual(44.0, result.bbox[1], delta=1.0)

    def test_invalid_time_interval_is_rejected(self):
        frame = np.zeros((80, 100, 3), np.uint8)
        cv2.line(frame, (30, 20), (59, 39), (255, 255, 255), 2)
        cv2.circle(frame, (42, 30), 4, (120, 220, 80), -1)
        tracker = SotTracker("template", max_failures=2)
        self.assertTrue(tracker.initialize(frame, (30, 20, 30, 20), 2.0).valid)
        self.assertFalse(tracker.update(frame, 2.0).valid)
        self.assertFalse(tracker.update(frame, 2.0).valid)
        result = tracker.update(frame, 2.1)
        self.assertFalse(result.valid)
        self.assertEqual("not_initialized", result.reason)
        self.assertIsNone(result.bbox)

    def test_failed_update_does_not_return_last_observed_box(self):
        frame = np.zeros((80, 100, 3), np.uint8)
        cv2.line(frame, (30, 20), (59, 39), (255, 255, 255), 2)
        tracker = SotTracker("template")
        self.assertTrue(tracker.initialize(frame, (30, 20, 30, 20), 2.0).valid)
        result = tracker.update(frame, 2.0)
        self.assertFalse(result.valid)
        self.assertIsNone(result.bbox)

    def test_uniform_template_is_rejected_instead_of_locking_background(self):
        frame = np.zeros((80, 100, 3), np.uint8)
        frame[20:40, 30:60] = 180
        result = SotTracker("template").initialize(
            frame, (30, 20, 30, 20), 1.0)
        self.assertFalse(result.valid)
        self.assertEqual("template_low_texture", result.reason)


if __name__ == "__main__":
    unittest.main()
