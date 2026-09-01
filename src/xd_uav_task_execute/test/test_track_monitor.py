#!/usr/bin/env python3

import unittest

from xd_uav_task_execute.core.track_monitor import (
    ERROR_ACQUISITION_TIMEOUT,
    ERROR_STATUS_TIMEOUT,
    ERROR_TARGET_LOST,
    STATE_FAILED,
    STATE_RUNNING,
    STATE_SUCCEEDED,
    TrackMonitor,
    TrackMonitorPolicy,
)


def observe(monitor, now, valid=True):
    return monitor.observe(
        now,
        tracker_active=True,
        target_visible=valid,
        target_predicted=False,
        command_valid=valid,
        state_valid=True,
        emergency_stop_active=False,
        tracking_state="tracking" if valid else "waiting_for_target",
        tracking_quality=0.9,
    )


class TrackMonitorTest(unittest.TestCase):
    def policy(self, **overrides):
        values = dict(
            required_tracking_sec=3.0,
            acquisition_timeout_sec=2.0,
            target_loss_timeout_sec=1.0,
            status_timeout_sec=0.5,
            maximum_duration_sec=10.0,
        )
        values.update(overrides)
        return TrackMonitorPolicy(**values)

    def test_continuous_valid_tracking_succeeds(self):
        monitor = TrackMonitor(0.0, self.policy())
        self.assertEqual(observe(monitor, 0.1), STATE_RUNNING)
        self.assertEqual(observe(monitor, 2.0), STATE_RUNNING)
        self.assertEqual(observe(monitor, 3.1), STATE_SUCCEEDED)
        self.assertEqual(monitor.progress, 1.0)

    def test_invalid_sample_resets_continuous_progress(self):
        monitor = TrackMonitor(0.0, self.policy())
        observe(monitor, 0.1)
        observe(monitor, 2.0)
        observe(monitor, 2.1, valid=False)
        observe(monitor, 2.2)
        self.assertEqual(observe(monitor, 4.0), STATE_RUNNING)
        self.assertLess(monitor.progress, 1.0)
        self.assertEqual(observe(monitor, 5.2), STATE_SUCCEEDED)

    def test_acquisition_timeout(self):
        monitor = TrackMonitor(0.0, self.policy())
        observe(monitor, 0.1, valid=False)
        observe(monitor, 1.9, valid=False)
        self.assertEqual(observe(monitor, 2.1, valid=False), STATE_FAILED)
        self.assertEqual(monitor.error, ERROR_ACQUISITION_TIMEOUT)

    def test_target_loss_timeout(self):
        monitor = TrackMonitor(0.0, self.policy())
        observe(monitor, 0.1)
        observe(monitor, 0.2, valid=False)
        observe(monitor, 1.0, valid=False)
        self.assertEqual(observe(monitor, 1.2, valid=False), STATE_FAILED)
        self.assertEqual(monitor.error, ERROR_TARGET_LOST)

    def test_status_timeout(self):
        monitor = TrackMonitor(0.0, self.policy())
        observe(monitor, 0.1)
        self.assertEqual(monitor.poll(0.7), STATE_FAILED)
        self.assertEqual(monitor.error, ERROR_STATUS_TIMEOUT)


if __name__ == "__main__":
    unittest.main()
