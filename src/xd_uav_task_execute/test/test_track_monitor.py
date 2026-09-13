#!/usr/bin/env python3

import unittest

from xd_uav_task_execute.core.track_monitor import (
    ERROR_ACQUISITION_TIMEOUT,
    ERROR_CONTROL_UNAVAILABLE,
    ERROR_GIMBAL_UNAVAILABLE,
    ERROR_PROFILE_MISMATCH,
    ERROR_STATUS_TIMEOUT,
    ERROR_TARGET_MISMATCH,
    ERROR_TARGET_LOST,
    ERROR_TRACKER_STOPPED,
    STATE_FAILED,
    STATE_RUNNING,
    STATE_SUCCEEDED,
    TrackMonitor,
    TrackMonitorPolicy,
)


def observe(monitor, now, valid=True, **overrides):
    values = dict(
        tracker_active=True,
        target_visible=valid,
        target_predicted=False,
        command_valid=valid,
        state_valid=True,
        emergency_stop_active=False,
        tracking_state="tracking" if valid else "waiting_for_target",
        tracking_quality=0.9,
        track_id=7,
        follower_profile="mc_velocity_ground",
        requested_profile="mc_velocity_ground",
        control_reference_published=valid,
        gimbal_state_valid=True,
        gimbal_fallback_active=False,
    )
    values.update(overrides)
    return monitor.observe(now, **values)


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

    def test_stale_stopped_status_is_tolerated_during_startup(self):
        monitor = TrackMonitor(0.0, self.policy())
        self.assertNotEqual(
            observe(monitor, 0.1, tracker_active=False), STATE_FAILED
        )
        self.assertEqual(observe(monitor, 0.2), STATE_RUNNING)

    def test_tracker_stopping_after_active_fails_immediately(self):
        monitor = TrackMonitor(0.0, self.policy())
        observe(monitor, 0.1)
        self.assertEqual(
            observe(monitor, 0.2, tracker_active=False), STATE_FAILED
        )
        self.assertEqual(monitor.error, ERROR_TRACKER_STOPPED)

    def test_tracker_never_starting_fails_at_acquisition_deadline(self):
        monitor = TrackMonitor(0.0, self.policy())
        observe(monitor, 0.1, tracker_active=False)
        self.assertEqual(
            observe(monitor, 2.1, tracker_active=False), STATE_FAILED
        )
        self.assertEqual(monitor.error, ERROR_TRACKER_STOPPED)

    def test_explicit_track_id_must_remain_selected(self):
        monitor = TrackMonitor(0.0, self.policy(required_track_id=7))
        observe(monitor, 0.1)
        observe(monitor, 0.2, track_id=8)
        self.assertEqual(observe(monitor, 1.2, track_id=8), STATE_FAILED)
        self.assertEqual(monitor.error, ERROR_TARGET_MISMATCH)

    def test_auto_selected_track_is_locked_for_the_action(self):
        monitor = TrackMonitor(0.0, self.policy())
        observe(monitor, 0.1, track_id=11)
        observe(monitor, 0.2, track_id=12)
        self.assertEqual(observe(monitor, 1.2, track_id=12), STATE_FAILED)
        self.assertEqual(monitor.error, ERROR_TARGET_MISMATCH)

    def test_profile_fallback_does_not_count_as_completion(self):
        monitor = TrackMonitor(
            0.0, self.policy(required_profile="gm_velocity_chase")
        )
        self.assertEqual(
            observe(
                monitor,
                2.1,
                follower_profile="mc_velocity_chase",
                requested_profile="gm_velocity_chase",
                gimbal_fallback_active=True,
            ),
            STATE_FAILED,
        )
        self.assertEqual(monitor.error, ERROR_PROFILE_MISMATCH)

    def test_missing_gimbal_state_is_reported_for_gm_profile(self):
        monitor = TrackMonitor(
            0.0, self.policy(required_profile="gm_velocity_vector")
        )
        self.assertEqual(
            observe(
                monitor,
                2.1,
                follower_profile="gm_velocity_vector",
                requested_profile="gm_velocity_vector",
                gimbal_state_valid=False,
            ),
            STATE_FAILED,
        )
        self.assertEqual(monitor.error, ERROR_GIMBAL_UNAVAILABLE)

    def test_missing_control_reference_does_not_count(self):
        monitor = TrackMonitor(0.0, self.policy())
        self.assertEqual(
            observe(monitor, 2.1, control_reference_published=False),
            STATE_FAILED,
        )
        self.assertEqual(monitor.error, ERROR_CONTROL_UNAVAILABLE)


if __name__ == "__main__":
    unittest.main()
