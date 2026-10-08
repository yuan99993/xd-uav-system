#!/usr/bin/env python3
"""Unit coverage for M4 terrain-swarm conflict geometry."""

import unittest

from xd_uav_planning.terrain_swarm import (
    detect_pair_conflict,
    path_length,
    release_conflict_counts,
)


class TerrainSwarmTest(unittest.TestCase):
    def test_simultaneous_crossing_is_spatiotemporal(self):
        result = detect_pair_conflict(
            [(-100.0, 0.0, 50.0), (100.0, 0.0, 50.0)], 10.0,
            [(0.0, -100.0, 50.0), (0.0, 100.0, 50.0)], 10.0,
            5.0)
        self.assertIsNotNone(result)
        self.assertEqual("spatiotemporal", result.conflict_type)
        self.assertAlmostEqual(0.0, result.separation_m, places=6)
        self.assertAlmostEqual(10.0, result.predicted_time_s, places=6)

    def test_different_arrival_times_are_spatial_only(self):
        result = detect_pair_conflict(
            [(-100.0, 0.0, 50.0), (100.0, 0.0, 50.0)], 10.0,
            [(0.0, -200.0, 50.0), (0.0, 100.0, 50.0)], 10.0,
            5.0)
        self.assertIsNotNone(result)
        self.assertEqual("spatial", result.conflict_type)
        self.assertAlmostEqual(0.0, result.separation_m, places=6)
        self.assertIsNone(result.predicted_time_s)

    def test_separated_parallel_paths_are_conflict_free(self):
        result = detect_pair_conflict(
            [(0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)], 15.0,
            [(0.0, 20.0, 0.0), (1000.0, 20.0, 0.0)], 15.0,
            10.0)
        self.assertIsNone(result)

    def test_vertical_separation_participates_in_distance(self):
        result = detect_pair_conflict(
            [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0)], 15.0,
            [(0.0, 0.0, 30.0), (100.0, 0.0, 30.0)], 15.0,
            20.0)
        self.assertIsNone(result)

    def test_duplicate_points_are_ignored_for_length(self):
        self.assertAlmostEqual(
            10.0,
            path_length([(0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                         (6.0, 8.0, 0.0)]))

    def test_invalid_nominal_speed_is_rejected(self):
        with self.assertRaises(ValueError):
            detect_pair_conflict(
                [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], 0.0,
                [(0.0, 1.0, 0.0), (1.0, 1.0, 0.0)], 1.0,
                2.0)

    def test_release_policy_can_demote_only_spatial_conflicts(self):
        types = ["spatial", "spatial", "spatiotemporal"]
        self.assertEqual((3, 2), release_conflict_counts(types, True))
        self.assertEqual((1, 2), release_conflict_counts(types, False))
        self.assertEqual(
            (0, 2), release_conflict_counts(["spatial", "spatial"], False))

    def test_release_policy_rejects_unknown_type(self):
        with self.assertRaises(ValueError):
            release_conflict_counts(["unknown"], False)


if __name__ == "__main__":
    unittest.main()
