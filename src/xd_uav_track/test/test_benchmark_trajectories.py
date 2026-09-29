#!/usr/bin/env python3
"""Low-cost invariants for the analytic Gazebo benchmark paths."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import benchmark_trajectories as paths  # noqa: E402


class BenchmarkTrajectoryTest(unittest.TestCase):
    def test_speed_is_bounded_and_no_framewise_correction_is_needed(self):
        step = 0.002
        maximum = 0.0
        sample_count = int(paths.CYCLE_DURATION_SEC / step)
        for index in range(sample_count):
            t = index * step
            first = paths.alpha(t)
            second = paths.alpha(t + step)
            maximum = max(maximum, math.hypot(second[0] - first[0],
                                               second[1] - first[1]) / step)
        self.assertLessEqual(maximum, paths.MAX_SPEED_MPS + 0.03)

    def test_all_segment_boundaries_are_position_and_velocity_continuous(self):
        boundaries = [paths.CIRCLE_DURATION_SEC]
        boundaries.extend(segment[1] for segment in paths._ROUTE)
        epsilon = 0.01
        for boundary in boundaries:
            left = paths.alpha(boundary - epsilon)
            right = paths.alpha(boundary + epsilon)
            self.assertLess(math.hypot(right[0] - left[0],
                                       right[1] - left[1]), 0.08)
            left_v = paths.alpha(boundary - epsilon)
            left_v2 = paths.alpha(boundary - 2.0 * epsilon)
            right_v = paths.alpha(boundary + epsilon)
            right_v2 = paths.alpha(boundary + 2.0 * epsilon)
            vl = ((left[0] - left_v2[0]) / epsilon,
                  (left[1] - left_v2[1]) / epsilon)
            vr = ((right_v2[0] - right[0]) / epsilon,
                  (right_v2[1] - right[1]) / epsilon)
            self.assertLess(math.hypot(vl[0] - vr[0], vl[1] - vr[1]), 0.20)

    def test_single_target_route_has_no_occlusion_phases(self):
        self.assertFalse(any(segment[5] for segment in paths._ROUTE))
        samples = int(paths.CYCLE_DURATION_SEC / 0.1)
        self.assertTrue(all(not paths.alpha(index * 0.1)[3]
                            for index in range(samples)))

    def test_parallel_paths_preserve_minimum_spacing(self):
        minimum_tank_distance = float("inf")
        minimum_decoy_distance = float("inf")
        for index in range(2000):
            t = paths.CYCLE_DURATION_SEC * index / 2000.0
            alpha = paths.alpha(t)
            bravo = paths.bravo(t)
            decoy = paths.decoy(t)
            minimum_tank_distance = min(
                minimum_tank_distance,
                math.hypot(alpha[0] - bravo[0], alpha[1] - bravo[1]))
            minimum_decoy_distance = min(
                minimum_decoy_distance,
                math.hypot(alpha[0] - decoy[0], alpha[1] - decoy[1]),
                math.hypot(bravo[0] - decoy[0], bravo[1] - decoy[1]))
        self.assertGreaterEqual(minimum_tank_distance, 8.0)
        self.assertGreaterEqual(minimum_decoy_distance, 8.0)


if __name__ == "__main__":
    unittest.main()
