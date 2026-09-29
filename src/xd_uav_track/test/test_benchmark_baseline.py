#!/usr/bin/env python3
"""The benchmark gate must reject scenery and duplicate target claims."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/check_tracking_benchmark_baseline.py"
SPEC = importlib.util.spec_from_file_location("benchmark_baseline", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def track(track_id, x, y):
    return SimpleNamespace(
        track_id=track_id, lifecycle_state="confirmed", range_valid=True,
        has_relative_position_body=True,
        relative_position_body=(x, -y, 0.0))


class BenchmarkBaselineTest(unittest.TestCase):
    def test_matching_is_one_to_one_and_flags_scenery(self):
        truth = {101: (-8.0, -11.0), 102: (-8.0, 5.0)}
        matches, unmatched = MODULE.assign_confirmed_tracks(
            [track(1, -8.1, -11.2), track(2, -7.8, 5.3),
             track(3, 34.0, 12.0)], truth, 7.0)
        self.assertEqual(matches, {101: 1, 102: 2})
        self.assertEqual(unmatched, 1)

    def test_reference_gate_rejects_a_single_missing_pair_frame(self):
        baseline = {
            "required_models": ["alpha", "bravo"],
            "forbidden_models": ["tree"],
            "minimum_duration_sec": 44.5,
            "minimum_tracker_hz": 2.5,
            "minimum_pair_confirmed_ratio": 0.99,
            "minimum_pair_alive_ratio": 1.0,
            "maximum_pair_gap_sec": 0.9,
            "maximum_alive_pair_gap_sec": 0.6,
            "maximum_id_switches": 0,
            "maximum_unmatched_confirmed_tracks": 0,
            "maximum_world_projection_error_m": 8.0,
            "required_phases": ["circle", "southbound_turn"],
        }
        report = {
            "duration_sec": 45.0, "tracker_frames": 130,
            "pair_confirmed_ratio": 0.98, "pair_alive_ratio": 1.0,
            "maximum_pair_gap_sec": 0.5,
            "maximum_alive_pair_gap_sec": 0.5, "id_switches": 0,
            "unmatched_confirmed_tracks": 0,
            "maximum_world_projection_error_m": 7.5,
            "invalid_world_measurements": 0,
            "phase_frames": {"circle": 40, "southbound_turn": 13},
        }
        checks = MODULE.checks_for(report, baseline, ["alpha", "bravo"])
        self.assertFalse(checks["simultaneous_confirmed"])
        report["pair_confirmed_ratio"] = 1.0
        self.assertTrue(all(MODULE.checks_for(
            report, baseline, ["alpha", "bravo"]).values()))

    def test_changed_scene_asset_invalidates_comparison(self):
        baseline = {"asset_sha256": {"scene.world": "expected"}}
        report = {"asset_sha256": {"scene.world": "different"}}
        self.assertFalse(MODULE.frozen_assets_match(report, baseline))


if __name__ == "__main__":
    unittest.main()
