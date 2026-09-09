#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.join(os.path.dirname(__file__), "..", "src")
sys.path.insert(0, os.path.abspath(PACKAGE_SRC))

from xd_uav_planning.core import (  # noqa: E402
    CommandSample,
    StateSample,
    validate_command,
    validate_state,
)


class BridgeCoreTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.state = StateSample(
            stamp=9.95,
            frame_id="world",
            body_frame_id="uav1/base_link",
            position=(1.0, 2.0, 3.0),
            velocity_world=(4.0, 5.0, 6.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            body_rate=(0.1, 0.2, 0.3),
            state_valid=True,
            localization_valid=True,
            odometry_fresh=True,
        )
        self.command = CommandSample(
            stamp=9.95,
            frame_id="world",
            position=(1.0, 2.0, 3.0),
            velocity=(4.0, 5.0, 6.0),
            acceleration=(0.1, 0.2, 0.3),
            yaw=0.4,
            yaw_rate=0.5,
            trajectory_flag=1,
        )

    def test_valid_state_preserves_world_velocity(self):
        result = validate_state(
            self.state, self.now, "world", "uav1/base_link", 0.2, 0.02)
        self.assertTrue(result.valid)
        self.assertEqual((4.0, 5.0, 6.0), self.state.velocity_world)

    def test_state_flags_frames_and_time_fail_closed(self):
        changes = (
            {"state_valid": False},
            {"localization_valid": False},
            {"odometry_fresh": False},
            {"frame_id": "odom"},
            {"body_frame_id": "wrong"},
            {"stamp": 9.0},
            {"stamp": 10.1},
        )
        for change in changes:
            values = dict(self.state.__dict__)
            values.update(change)
            result = validate_state(
                StateSample(**values), self.now, "world", "uav1/base_link",
                0.2, 0.02)
            self.assertFalse(result.valid, change)

    def test_state_rejects_non_finite_and_bad_quaternion(self):
        for change in (
                {"velocity_world": (math.nan, 0.0, 0.0)},
                {"orientation": (0.0, 0.0, 0.0, 0.0)},
                {"orientation": (0.0, 0.0, 0.0, 2.0)}):
            values = dict(self.state.__dict__)
            values.update(change)
            result = validate_state(
                StateSample(**values), self.now, "world", "uav1/base_link",
                0.2, 0.02)
            self.assertFalse(result.valid, change)

    def test_valid_command(self):
        result = validate_command(
            self.command, self.now, "world", 1, 0.2, 0.02)
        self.assertTrue(result.valid)

    def test_nonfinite_yaw_rate_requires_explicit_policy(self):
        values = dict(self.command.__dict__)
        values["yaw_rate"] = float("nan")
        sample = CommandSample(**values)
        strict = validate_command(sample, 10.1, "world", 1, 0.2, 0.02)
        adapted = validate_command(
            sample, 10.1, "world", 1, 0.2, 0.02,
            allow_nonfinite_yaw_rate=True)
        self.assertFalse(strict.valid)
        self.assertTrue(adapted.valid)

    def test_100_hz_scale_samples_remain_valid(self):
        for index in range(1000):
            now = 10.0 + index * 0.01
            state_values = dict(self.state.__dict__, stamp=now)
            command_values = dict(self.command.__dict__, stamp=now)
            self.assertTrue(validate_state(
                StateSample(**state_values), now, "world", "uav1/base_link",
                0.2, 0.02).valid)
            self.assertTrue(validate_command(
                CommandSample(**command_values), now, "world", 1,
                0.2, 0.02).valid)

    def test_command_status_frame_values_and_time_fail_closed(self):
        changes = (
            {"trajectory_flag": 0},
            {"frame_id": "odom"},
            {"yaw": math.inf},
            {"acceleration": (0.0, math.nan, 0.0)},
            {"stamp": 9.0},
            {"stamp": 10.1},
        )
        for change in changes:
            values = dict(self.command.__dict__)
            values.update(change)
            result = validate_command(
                CommandSample(**values), self.now, "world", 1, 0.2, 0.02)
            self.assertFalse(result.valid, change)


if __name__ == "__main__":
    unittest.main()
