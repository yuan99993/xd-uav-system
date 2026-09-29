#!/usr/bin/env python3
"""Small dynamics checks for the fixed-camera search demonstration."""

import math
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from moving_uav_search_driver import MovingSearchDriver  # noqa: E402


class MovingSearchDynamicsTest(unittest.TestCase):
    @staticmethod
    def driver():
        vehicle = MovingSearchDriver.__new__(MovingSearchDriver)
        vehicle.drone_xy = [0.0, 0.0]
        vehicle.drone_velocity = [0.0, 0.0]
        vehicle.drone_yaw = math.pi / 2.0
        vehicle.altitude_m = 28.0
        vehicle.altitude_target_m = 28.0
        vehicle.minimum_altitude_m = 28.0
        vehicle.maximum_altitude_m = 60.0
        vehicle.horizontal_acceleration_mps2 = 1.2
        vehicle.search_cruise_speed_mps = 1.4
        vehicle.coverage_transit_speed_mps = 1.6
        vehicle.follow_loss_coast_sec = 5.0
        vehicle.local_reacquire_sec = 12.0
        vehicle.local_reacquire_altitude_m = 40.0
        vehicle.camera_horizontal_fov_rad = 1.22
        vehicle.camera_image_width = 640
        vehicle.camera_image_height = 480
        vehicle.follow_fov_margin = 0.72
        vehicle.follow_preserve_other_targets = True
        vehicle.follow_overview_altitude_m = 58.0
        vehicle.follow_overview_margin_m = 2.0
        vehicle.follow_overview_fraction = 0.35
        vehicle.search_area_bounds = (-34.0, 34.0, -18.0, 18.0)
        vehicle.expected_target_count = 3
        vehicle.reacquire_route_active = False
        vehicle.search_progress = 0.0
        vehicle.search_direction = 1.0
        vehicle.search_path, vehicle.search_cumulative, vehicle.search_length = (
            MovingSearchDriver._make_search_path())
        return vehicle

    def test_follow_keeps_wide_view_until_target_is_centered(self):
        vehicle = self.driver()
        vehicle.target_world = [30.0, 0.0]
        vehicle.target_velocity_world = [0.0, 0.0]
        self.assertEqual(60.0, vehicle._follow_altitude_target())
        vehicle.target_world = [2.0, 0.0]
        self.assertEqual(28.0, vehicle._follow_altitude_target())

    def test_search_route_join_has_no_position_or_velocity_jump(self):
        vehicle = self.driver()
        before = tuple(vehicle.drone_xy)
        vehicle._advance_search_route(0.05)
        displacement = math.hypot(vehicle.drone_xy[0] - before[0],
                                  vehicle.drone_xy[1] - before[1])
        speed = math.hypot(*vehicle.drone_velocity)
        self.assertLessEqual(speed, 1.2 * 0.05 + 1e-9)
        self.assertLessEqual(displacement, speed * 0.05 + 1e-9)
        self.assertGreater(displacement, 0.0)

    def test_local_reacquisition_then_wide_area_transit_is_continuous(self):
        vehicle = self.driver()
        vehicle.follow_preserve_other_targets = False
        vehicle.target_world = [12.0, 0.0]
        vehicle.target_velocity_world = [1.0, 0.0]
        vehicle._advance_follow_reacquire_search(0.05, None, 6.0)
        self.assertEqual("FOLLOW_LOCAL_REACQUIRE", vehicle.phase)
        self.assertEqual(40.0, vehicle.altitude_target_m)
        before = tuple(vehicle.drone_xy)
        vehicle._global_coverage_plan = lambda stamp: (
            0.0, 0.0, math.pi / 2.0, 52.0, 0, 0, True, 51.5)
        with mock.patch("moving_uav_search_driver.rospy.loginfo_throttle"):
            vehicle._advance_follow_reacquire_search(0.05, None, 18.0)
        self.assertEqual("FOLLOW_REACQUIRE_SEARCH", vehicle.phase)
        self.assertEqual(52.0, vehicle.altitude_target_m)
        displacement = math.hypot(vehicle.drone_xy[0] - before[0],
                                  vehicle.drone_xy[1] - before[1])
        self.assertLessEqual(displacement, 1.6 * 0.05 + 1e-9)

    def test_multitarget_follow_keeps_all_area_in_fixed_camera_fov(self):
        vehicle = self.driver()
        vehicle.altitude_m = 58.0
        vehicle._global_coverage_plan = lambda stamp: (
            0.0, 0.0, math.pi / 2.0, 51.5, 3, 3, True, 51.5)
        position, altitude = vehicle._overview_follow_plan([30.0, 0.0], None)
        self.assertEqual(58.0, altitude)
        self.assertLess(abs(position[0]), 5.0)
        self.assertAlmostEqual(0.0, position[1])
        vehicle.expected_target_count = 1
        self.assertIsNone(vehicle._overview_follow_plan([30.0, 0.0], None))

    def test_multitarget_local_reacquisition_does_not_descend(self):
        vehicle = self.driver()
        vehicle.altitude_m = 58.0
        vehicle.target_world = [30.0, 0.0]
        vehicle.target_velocity_world = [0.0, 0.0]
        vehicle._global_coverage_plan = lambda stamp: (
            0.0, 0.0, math.pi / 2.0, 51.5, 3, 2, True, 51.5)
        vehicle._advance_follow_reacquire_search(0.05, None, 6.0)
        self.assertEqual("FOLLOW_LOCAL_REACQUIRE", vehicle.phase)
        self.assertEqual(58.0, vehicle.altitude_target_m)
        self.assertLess(abs(vehicle.drone_xy[0]), 0.1)


if __name__ == "__main__":
    unittest.main()
