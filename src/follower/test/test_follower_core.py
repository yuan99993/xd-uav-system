"""Regression tests for the follower's fail-closed control behavior."""

import math
import unittest

from follower.follower_core import FollowerCore


class FollowerCoreTest(unittest.TestCase):
    def _core(self):
        return FollowerCore({
            'command_smoothing_enabled': False,
            'yaw_smoothing_enabled': False,
            'pid_yaw': {'kp': 1.0, 'ki': 0.0, 'kd': 0.0},
            'pid_right': {'kp': 1.0, 'ki': 0.0, 'kd': 0.0},
            'pid_down': {'kp': 1.0, 'ki': 0.0, 'kd': 0.0},
            'max_yaw_rate_deg_s': 90.0,
        })

    def test_visible_target_produces_corrective_yaw_command(self):
        result = self._core().compute(
            error_x=0.5, error_y=0.0, dt=0.1,
            error_valid=True, timestamp=1.0,
        )
        self.assertTrue(result.command_valid)
        self.assertTrue(result.target_visible)
        self.assertGreater(result.yaw_rate_deg_s, 0.0)

    def test_lost_target_fails_closed_without_nan_commands(self):
        core = self._core()
        core.compute(0.5, -0.3, 0.1, error_valid=True, timestamp=1.0)
        result = core.compute(
            error_x=float('nan'), error_y=float('nan'), dt=0.1,
            error_valid=False, timestamp=1.1,
        )
        self.assertFalse(result.command_valid)
        self.assertFalse(result.target_visible)
        self.assertEqual(result.velocity_right, 0.0)
        self.assertEqual(result.velocity_down, 0.0)
        self.assertEqual(result.yaw_rate_deg_s, 0.0)
        self.assertTrue(all(math.isfinite(value) for value in (
            result.velocity_forward, result.velocity_right,
            result.velocity_down, result.yaw_rate_deg_s,
        )))


if __name__ == '__main__':
    unittest.main()
