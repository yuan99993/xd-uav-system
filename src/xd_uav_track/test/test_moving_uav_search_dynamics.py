#!/usr/bin/env python3
"""Small dynamics checks for the fixed-camera search demonstration."""

import math
import os
import sys
import threading
import unittest
from unittest import mock
from collections import deque
import rospy
from xd_uav_track.msg import PerceptionState, PerceptionTarget, PerceptionTargetArray
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

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
        vehicle.altitude_rate_mps = 1.5
        vehicle.altitude_descent_rate_mps = 0.35
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
        vehicle.coverage_margin_m = 2.0
        vehicle.follow_fov_margin = 0.72
        vehicle.follow_preserve_other_targets = True
        vehicle.gimbal_camera_enabled = True
        vehicle.gimbal_min_look_down_rad = 0.10
        vehicle.gimbal_safe_standoff_margin_m = 2.0
        vehicle.gimbal_horizontal_fov_rad = 1.05
        vehicle.global_search_image_source = "search_fixed"
        vehicle.follow_tracking_sources = {"search_fixed", "search_gimbal"}
        vehicle.tracking_image_source = "search_fixed"
        vehicle.manual_selection_active = False
        vehicle.follow_track_id = None
        vehicle.follow_mode_started_at = rospy.Time()
        vehicle.last_visual_measurement = rospy.Time()
        vehicle.target_world = None
        vehicle.target_velocity_world = [0.0, 0.0]
        vehicle.previous_target_world = None
        vehicle.previous_target_stamp = rospy.Time()
        vehicle.current_gimbal_pan_rad = 0.0
        vehicle.current_gimbal_look_down_rad = 0.62
        vehicle.gimbal_look_down_rad = 0.62
        vehicle.gimbal_pan_amplitude_rad = math.radians(30.0)
        vehicle.gimbal_sweep_frequency_hz = 0.01
        vehicle.gimbal_pan_rate_limit_radps = math.radians(2.0)
        vehicle.gimbal_tilt_rate_limit_radps = math.radians(2.0)
        vehicle.last_gimbal_pan_stamp = rospy.Time()
        vehicle.perception_gimbal_lock = threading.Lock()
        vehicle.perception_gimbal_selection = None
        vehicle.perception_gimbal_selection_stamp = rospy.Time()
        vehicle.perception_gimbal_aim = None
        vehicle.gimbal_angle_history = deque(maxlen=200)
        vehicle.gimbal_camera_intrinsics = None
        vehicle.perception_gimbal_source = "search_gimbal"
        vehicle.gimbal_pan_limit_rad = math.radians(70)
        vehicle.gimbal_tracking_rate_radps = math.radians(6)
        vehicle.gimbal_visual_coast_sec = 3.0
        vehicle.target_state_lock = threading.Lock()
        vehicle.target_memory_sec = 15.0
        vehicle.known_targets = {}
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

    def test_altitude_descent_is_slow_and_clamped_at_safe_floor(self):
        vehicle = self.driver()
        vehicle.altitude_m = 58.0
        vehicle.minimum_altitude_m = 54.0
        vehicle.altitude_target_m = vehicle.minimum_altitude_m
        vehicle._approach_altitude_target(1.0)
        self.assertAlmostEqual(57.65, vehicle.altitude_m)
        vehicle._approach_altitude_target(20.0)
        self.assertAlmostEqual(54.0, vehicle.altitude_m)

    def test_global_mode_collects_but_never_follows_auto_selected_tracks(self):
        vehicle = self.driver()
        vehicle.selection_mode = "all"
        vehicle.manual_selection_active = True
        vehicle.follow_track_id = None
        vehicle._update_known_targets = mock.Mock()
        selected = mock.Mock(selected=True, track_id=7)
        message = mock.Mock(image_source="search_fixed", tracks=[selected])
        vehicle._track_callback(message)
        vehicle._update_known_targets.assert_called_once_with(message)
        self.assertFalse(vehicle.manual_selection_active)
        self.assertIsNone(vehicle.follow_track_id)

        vehicle._update_known_targets.reset_mock()
        gimbal_message = mock.Mock(image_source="search_gimbal", tracks=[selected])
        vehicle._track_callback(gimbal_message)
        vehicle._update_known_targets.assert_not_called()

    def test_follow_mode_accepts_both_sources_without_cross_source_id_switch(self):
        vehicle = self.driver()
        vehicle.selection_mode = "follow"
        vehicle.manual_selection_active = True
        vehicle._update_known_targets = mock.Mock()
        fixed_message = mock.Mock(image_source="search_fixed", tracks=[],
                                  header=mock.Mock(stamp=rospy.Time.from_sec(1.0)))
        vehicle._track_callback(fixed_message)
        vehicle._update_known_targets.assert_called_once_with(fixed_message)
        vehicle._update_known_targets.reset_mock()
        gimbal_message = mock.Mock(image_source="search_gimbal", tracks=[],
                                   header=mock.Mock(stamp=rospy.Time.from_sec(1.1)))
        vehicle._track_callback(gimbal_message)
        vehicle._update_known_targets.assert_not_called()

    def test_gimbal_follow_uses_safe_standoff_when_target_is_under_aircraft(self):
        vehicle = self.driver()
        vehicle.altitude_m = 58.0
        vehicle.drone_xy = [0.0, 0.0]
        vehicle.drone_yaw = math.pi / 2.0
        point = vehicle._gimbal_safe_follow_point([0.0, 0.0])
        self.assertAlmostEqual((58.0 - 0.12) * math.tan(0.10) + 2.0,
                               math.hypot(*point), places=6)
        self.assertGreater(point[1], 0.0)

    def test_gimbal_source_cannot_replace_selected_global_identity(self):
        vehicle = self.driver()
        vehicle.selection_mode = "follow"
        vehicle.follow_track_id = 4
        candidate = mock.Mock(selected=True, track_id=9)
        message = mock.Mock(image_source="search_gimbal", tracks=[candidate])
        vehicle._update_known_targets = mock.Mock()
        vehicle._track_callback(message)
        vehicle._update_known_targets.assert_called_once_with(message, {9})
        self.assertEqual(4, vehicle.follow_track_id)

    def test_overview_follow_preserves_global_fov_when_applying_standoff(self):
        vehicle = self.driver()
        vehicle.altitude_m = 58.0
        vehicle.drone_xy = [0.0, 0.0]
        vehicle.drone_yaw = math.pi / 2.0
        vehicle._global_coverage_plan = lambda stamp: (
            0.0, 0.0, math.pi / 2.0, 51.5, 3, 3, True, 51.5)
        position, altitude = vehicle._overview_follow_plan([0.0, 0.0], None)
        self.assertGreaterEqual(math.hypot(*position),
                                vehicle._gimbal_safe_standoff_m())
        self.assertEqual(58.0, altitude)

    def test_stale_cross_camera_target_measurement_cannot_rewind_memory(self):
        vehicle = self.driver()
        track = mock.Mock(lifecycle_state="confirmed", detected=True,
                          predicted=False, hit_count=4, track_id=12)
        fixed_message = mock.Mock(image_source="search_fixed", tracks=[track])
        vehicle._world_measurement = mock.Mock(return_value=(
            rospy.Time.from_sec(10.0), [5.0, 6.0]))
        with mock.patch("moving_uav_search_driver.rospy.Time.now",
                        return_value=rospy.Time.from_sec(10.1)):
            vehicle._update_known_targets(fixed_message)
            vehicle._world_measurement.return_value = (
                rospy.Time.from_sec(9.8), [50.0, 60.0])
            vehicle._update_known_targets(mock.Mock(
                image_source="search_gimbal", tracks=[track]))
        self.assertEqual([5.0, 6.0], vehicle.known_targets[12]["position"])
        self.assertEqual(10.0, vehicle.known_targets[12]["stamp"].to_sec())

    def test_gimbal_aims_at_follow_target_with_rate_limit(self):
        vehicle = self.driver()
        vehicle.altitude_m = 58.0
        vehicle.drone_yaw = 0.0
        vehicle.drone_xy = [0.0, 0.0]
        vehicle.gimbal_pan_amplitude_rad = math.radians(30.0)
        vehicle.gimbal_sweep_frequency_hz = 0.01
        vehicle.gimbal_pan_rate_limit_radps = math.radians(2.0)
        vehicle.gimbal_tilt_rate_limit_radps = math.radians(2.0)
        vehicle.current_gimbal_pan_rad = 0.0
        vehicle.gimbal_look_down_rad = 0.62
        vehicle.current_gimbal_look_down_rad = 0.62
        vehicle.last_gimbal_pan_stamp = rospy.Time.from_sec(1.0)
        vehicle.selection_mode = "follow"
        vehicle.target_world = [0.0, 30.0]
        vehicle.target_velocity_world = [0.0, 0.0]
        vehicle.last_visual_measurement = rospy.Time.from_sec(2.0)
        pan, look_down = vehicle._update_gimbal_angles(
            rospy.Time.from_sec(2.0), 25.0)
        self.assertGreater(pan, 0.0)
        self.assertLessEqual(pan, math.radians(2.0) * 0.2 + 1e-9)
        self.assertLess(look_down, 0.62)
        self.assertGreaterEqual(look_down, 0.62 - math.radians(2.0) * 0.2 - 1e-9)

    def test_web_gimbal_selection_uses_capture_ray_without_aircraft_follow(self):
        vehicle = self.driver()
        vehicle.selection_mode = "all"
        vehicle.drone_yaw = 0.0
        vehicle.last_gimbal_pan_stamp = rospy.Time(10)
        vehicle.gimbal_angle_history.append((rospy.Time(10), 0.0, .62, 0.0))
        info = CameraInfo(width=640, height=480, K=[500,0,320,0,500,240,0,0,1])
        vehicle._gimbal_camera_info(info)
        state = PerceptionState(session_id="s", model_epoch=1, mode="fusion",
            selection_active=True, selected_target_uid="s:4", selected_image_source="search_gimbal")
        target = PerceptionTarget(target_uid="s:4", track_id=4, selected=True,
            lifecycle_state="confirmed", measurement_source="visual_tracker", center_px=[420,290])
        message = PerceptionTargetArray(header=Header(stamp=rospy.Time(10)),
            session_id="s", model_epoch=1, image_source="search_gimbal",
            image_width=640, image_height=480, targets=[target])
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10.1)):
            vehicle._perception_gimbal_state(state)
            vehicle._perception_gimbal_targets(message)
        self.assertIsNotNone(vehicle.perception_gimbal_aim)
        pan, tilt = vehicle._update_gimbal_angles(rospy.Time.from_sec(10.2), 0)
        self.assertLess(pan, 0.0, "target right must rotate camera right")
        self.assertLess(tilt, .62, "target below must tilt toward nadir")
        self.assertLessEqual(abs(pan), math.radians(6)*.2+1e-9)
        self.assertEqual("all", vehicle.selection_mode)
        self.assertIsNone(vehicle.target_world)
        previous = vehicle.perception_gimbal_aim
        target.measurement_source = "prediction"
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10.3)):
            message.header.stamp = rospy.Time.from_sec(10.2)
            vehicle._perception_gimbal_targets(message)
        self.assertEqual(previous, vehicle.perception_gimbal_aim)
        state.selection_active = False
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10.3)):
            vehicle._perception_gimbal_state(state)
        self.assertIsNone(vehicle.perception_gimbal_aim)
        self.assertIsNone(vehicle.perception_gimbal_selection)

    def test_web_gimbal_coast_expires_and_cancel_does_not_jump_limit(self):
        vehicle = self.driver()
        vehicle.selection_mode = "all"
        vehicle.drone_yaw = 0.0
        vehicle.perception_gimbal_selection = ("s",1,"s:4")
        vehicle.perception_gimbal_aim = (rospy.Time(10), .5, .5)
        vehicle.last_gimbal_pan_stamp = rospy.Time.from_sec(11.8)
        pan, _ = vehicle._update_gimbal_angles(rospy.Time(12), 0)
        self.assertGreater(pan, 0, "short coast keeps last trusted ray")
        vehicle.current_gimbal_pan_rad = math.radians(55)
        vehicle.perception_gimbal_selection = None
        vehicle.last_gimbal_pan_stamp = rospy.Time.from_sec(15.8)
        pan, _ = vehicle._update_gimbal_angles(rospy.Time(16), 0)
        self.assertGreater(pan, math.radians(54), "cancel must not clip instantly to search amplitude")
        self.assertLessEqual(math.radians(55)-pan, math.radians(2)*.2+1e-9)
        vehicle.perception_gimbal_selection = ("s",1,"s:4")
        vehicle.current_gimbal_pan_rad = 0
        vehicle.last_gimbal_pan_stamp = rospy.Time.from_sec(16.8)
        pan, _ = vehicle._update_gimbal_angles(rospy.Time(17), 0)
        self.assertEqual(0, pan, "expired old ray must not continue steering")


if __name__ == "__main__":
    unittest.main()
