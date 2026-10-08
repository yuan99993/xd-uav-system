#!/usr/bin/env python3
"""Unit coverage for the M6 fail-closed release decision."""

from types import SimpleNamespace as Item
import unittest

from xd_uav_planning.terrain_swarm_gate import GateNotReady, releasable_paths


def path(goal_id, frame="world"):
    header = Item(frame_id=frame, seq=0)
    poses = [Item(header=Item(frame_id=frame, seq=goal_id)) for _ in range(2)]
    return Item(header=header, poses=poses)


def status(conflict_free=True, second_goal=6202, conflicts=None):
    vehicles = [
        Item(uav_name="uav1", path_available=True,
             planner_status=Item(goal_id=6201)),
        Item(uav_name="uav2", path_available=True,
             planner_status=Item(goal_id=second_goal)),
        Item(uav_name="uav3", path_available=True,
             planner_status=Item(goal_id=6203)),
    ]
    return Item(
        analysis_ready=True,
        conflict_free=conflict_free,
        conflicts=([] if conflict_free else [Item(conflict_type=2)])
        if conflicts is None else conflicts,
        header=Item(frame_id="world"),
        vehicles=vehicles,
    )


class TerrainSwarmGateTest(unittest.TestCase):
    def test_complete_conflict_free_batch_is_releasable(self):
        output = releasable_paths(
            status(), {"uav1": path(6201), "uav2": path(6202),
                       "uav3": path(6203)},
            ["uav1", "uav2", "uav3"])
        self.assertEqual([("uav1", 6201), ("uav2", 6202),
                          ("uav3", 6203)],
                         [(name, goal) for name, goal, _ in output])

    def test_conflict_is_held(self):
        with self.assertRaisesRegex(GateNotReady, "conflict"):
            releasable_paths(
                status(False), {"uav1": path(6201), "uav2": path(6202),
                                "uav3": path(6203)},
                ["uav1", "uav2", "uav3"])

    def test_spatial_only_warning_is_releasable_when_monitor_clears_it(self):
        output = releasable_paths(
            status(conflicts=[Item(conflict_type=1)]),
            {"uav1": path(6201), "uav2": path(6202),
             "uav3": path(6203)},
            ["uav1", "uav2", "uav3"])
        self.assertEqual(3, len(output))

    def test_spatiotemporal_conflict_fails_closed_even_if_status_is_wrong(self):
        with self.assertRaisesRegex(GateNotReady, "blocking_conflict"):
            releasable_paths(
                status(conflicts=[Item(conflict_type=2)]),
                {"uav1": path(6201), "uav2": path(6202),
                 "uav3": path(6203)},
                ["uav1", "uav2", "uav3"])

    def test_goal_mismatch_is_held(self):
        with self.assertRaisesRegex(GateNotReady, "goal_id_mismatch"):
            releasable_paths(
                status(second_goal=9999),
                {"uav1": path(6201), "uav2": path(6202),
                 "uav3": path(6203)},
                ["uav1", "uav2", "uav3"])

    def test_frame_mismatch_is_held(self):
        with self.assertRaisesRegex(GateNotReady, "frame"):
            releasable_paths(
                status(), {"uav1": path(6201), "uav2": path(6202, "map"),
                           "uav3": path(6203)},
                ["uav1", "uav2", "uav3"])


if __name__ == "__main__":
    unittest.main()
