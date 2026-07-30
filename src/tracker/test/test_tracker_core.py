"""Regression tests for tracker input and target-loss behavior."""

import unittest

from tracker.tracker_core import TrackerCore


class TrackerCoreTest(unittest.TestCase):
    def test_external_bbox_starts_tracking_and_expires_after_prediction_window(self):
        core = TrackerCore({
            'frame_width': 640,
            'frame_height': 480,
            'id_loss_tolerance_frames': 2,
            'enable_prediction_buffer': True,
        })
        core.process_external_bbox(
            bbox_pixel=(260, 180, 380, 300), confidence=0.9,
            class_id=1, command='start_track',
        )
        detected = core.update_with_detections(
            [[260, 180, 380, 300, 1, 0.9, 1, 1]], timestamp=1.0,
        )
        self.assertTrue(detected.tracking_active)
        self.assertFalse(detected.is_predicted)

        predicted = core.update_empty(timestamp=1.1)
        self.assertTrue(predicted.tracking_active)
        self.assertTrue(predicted.is_predicted)

        core.update_empty(timestamp=1.2)
        expired = core.update_empty(timestamp=1.3)
        self.assertFalse(expired.tracking_active)

    def test_stop_tracking_command_does_not_require_detection_payload(self):
        core = TrackerCore()
        core.process_external_bbox(
            normalized_bbox=(0.5, 0.5, 0.2, 0.2),
            confidence=0.9,
            command='start_track',
        )
        self.assertTrue(core.is_tracking())

        core.process_external_bbox(command='stop_track')
        stopped = core.update_empty(timestamp=1.0)
        self.assertFalse(stopped.tracking_active)

    def test_invalid_external_bbox_is_rejected(self):
        core = TrackerCore()
        with self.assertRaises(ValueError):
            core.process_external_bbox(bbox_pixel=(100, 100, 50, 150))


if __name__ == '__main__':
    unittest.main()
