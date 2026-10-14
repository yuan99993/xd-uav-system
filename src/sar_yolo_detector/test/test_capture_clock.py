import unittest
from sar_yolo_detector.capture_clock import CaptureClock


class CaptureClockTest(unittest.TestCase):
    def observe(self, clock, stamp, seq, now=None, frame="camera", shape=(480, 640)):
        return clock.observe(stamp, seq, frame, shape, stamp if now is None else now)

    def test_duplicate_and_ordinary_disorder_do_not_reset(self):
        clock = CaptureClock()
        self.assertTrue(self.observe(clock, 10., 100).accepted)
        self.assertFalse(self.observe(clock, 10., 100).accepted)
        self.assertFalse(self.observe(clock, 9.9, 99, now=10.).accepted)
        self.assertEqual(clock.segment, 0)
        self.assertTrue(self.observe(clock, 10.1, 101).accepted)

    def test_sim_clock_rollback_recovers_only_when_new_capture_is_fresh(self):
        clock = CaptureClock()
        self.observe(clock, 100., 1000)
        self.assertFalse(self.observe(clock, 100.1, 1001, now=1.).accepted)
        result = self.observe(clock, 1.1, 2, now=1.1)
        self.assertTrue(result.accepted)
        self.assertTrue(result.new_segment)
        self.assertTrue(self.observe(clock, 1.2, 3).accepted)

    def test_camera_sequence_restart_requires_two_new_frames(self):
        clock = CaptureClock()
        self.observe(clock, 10., 100)
        self.assertFalse(self.observe(clock, 9.95, 0, now=10.05).accepted)
        result = self.observe(clock, 10.05, 1, now=10.1)
        self.assertTrue(result.accepted)
        self.assertTrue(result.new_segment)

    def test_one_old_restart_like_frame_does_not_erase_tracking(self):
        clock = CaptureClock()
        self.observe(clock, 10., 100)
        self.assertFalse(self.observe(clock, 9.9, 1, now=10.).accepted)
        self.assertFalse(self.observe(clock, 9.9, 1, now=10.).accepted)
        result = self.observe(clock, 10.1, 101)
        self.assertTrue(result.accepted)
        self.assertFalse(result.new_segment)

    def test_unsynchronized_camera_clock_not_silently_restamped(self):
        clock = CaptureClock()
        self.observe(clock, 100., 1000)
        self.assertFalse(self.observe(clock, 1., 0, now=100.1).accepted)
        self.assertFalse(self.observe(clock, 1.1, 1, now=100.2).accepted)
        self.assertEqual(clock.segment, 0)

    def test_frame_or_resolution_change_is_a_new_segment(self):
        clock = CaptureClock()
        self.observe(clock, 10., 100)
        self.assertTrue(self.observe(clock, 10.1, 101, shape=(720, 1280)).new_segment)
        self.assertTrue(self.observe(clock, 10.2, 102, frame="new_camera", shape=(720, 1280)).new_segment)


if __name__ == "__main__":
    unittest.main()
