#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.join(os.path.dirname(__file__), "..", "src")
sys.path.insert(0, os.path.abspath(PACKAGE_SRC))

from xd_uav_planning.core import (  # noqa: E402
    PathSample,
    ReferenceSample,
    VehicleStateSample,
    arrival_reached,
    health_conjunction,
    switch_delta,
    validate_path,
    validate_reference,
    validate_vehicle_state,
)


class PlanningCoreTest(unittest.TestCase):
    @staticmethod
    def reference(**updates):
        values = dict(
            stamp=10.0, frame_id="world", coordinate_frame=1, type_mask=0,
            position=(1.0, 2.0, 3.0), velocity=(0.1, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0), yaw=0.0, yaw_rate=0.0)
        values.update(updates)
        return ReferenceSample(**values)

    @staticmethod
    def path(**updates):
        values = dict(
            stamp=10.0, frame_id="world",
            points=((0.0, 0.0, 10.0), (20.0, 0.0, 10.0)),
            pose_frames=("world", "world"))
        values.update(updates)
        return PathSample(**values)

    def test_reference_and_switch_contracts(self):
        self.assertTrue(validate_reference(
            self.reference(), 10.1, "world", 0.2, 0.02).valid)
        self.assertEqual("reference_frame_mismatch", validate_reference(
            self.reference(frame_id="odom"), 10.1, "world", 0.2, 0.02).reason)
        self.assertEqual("force_mode_unsupported", validate_reference(
            self.reference(type_mask=512), 10.1, "world", 0.2, 0.02).reason)
        self.assertTrue(switch_delta(
            self.reference(), self.reference(position=(1.1, 2.0, 3.0)),
            0.2, 0.3).valid)
        self.assertEqual("switch_position_jump", switch_delta(
            self.reference(), self.reference(position=(1.3, 2.0, 3.0)),
            0.2, 0.3).reason)
        self.assertTrue(health_conjunction([True, True], 2))
        self.assertFalse(health_conjunction([True, False], 2))

    def test_fixedwing_state_contract(self):
        valid = VehicleStateSample(10.0, 1, True, True, True)
        self.assertTrue(validate_vehicle_state(
            valid, 10.1, 1, 0.3, 0.02).valid)
        cases = [
            (VehicleStateSample(10.0, 0, True, True, True),
             "vehicle_type_mismatch"),
            (VehicleStateSample(10.0, 1, False, True, True), "state_invalid"),
            (VehicleStateSample(10.0, 1, True, False, True),
             "localization_invalid"),
            (VehicleStateSample(10.0, 1, True, True, False),
             "odometry_not_fresh"),
            (VehicleStateSample(9.0, 1, True, True, True), "input_stale"),
        ]
        for sample, reason in cases:
            self.assertEqual(reason, validate_vehicle_state(
                sample, 10.1, 1, 0.3, 0.02).reason)

    def test_path_contract(self):
        self.assertTrue(validate_path(
            self.path(), 10.1, "world", 0.5, 0.02).valid)
        cases = [
            (self.path(frame_id="odom"), "path_frame_mismatch"),
            (self.path(stamp=9.0), "path_input_stale"),
            (self.path(points=((0.0, 0.0, 10.0),),
                       pose_frames=("world",)), "path_too_short"),
            (self.path(pose_frames=("world", "odom")),
             "path_pose_frame_mismatch"),
            (self.path(points=((0.0, 0.0, 10.0),
                               (math.nan, 0.0, 10.0))), "path_not_finite"),
            (self.path(points=((0.0, 0.0, 10.0),
                               (0.001, 0.0, 10.0))),
             "path_segment_too_short"),
        ]
        for sample, reason in cases:
            result = validate_path(sample, 10.1, "world", 0.5, 0.02)
            self.assertFalse(result.valid)
            self.assertEqual(reason, result.reason)

    def test_arrival_contract(self):
        self.assertTrue(arrival_reached(
            (1.0, 2.0, 3.0), (0.1, 0.0, 0.0), (1.1, 2.0, 3.0),
            0.5, 0.35))
        self.assertFalse(arrival_reached(
            (1.0, 2.0, 3.0), (1.0, 0.0, 0.0), (1.1, 2.0, 3.0),
            0.5, 0.35))


if __name__ == "__main__":
    unittest.main()
