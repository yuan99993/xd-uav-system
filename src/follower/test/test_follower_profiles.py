#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import VFR_HUD
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from follower.msg import FollowerCommand, FollowerStatus
from follower.srv import SetMode
from tracker.msg import NormalizedError


class FollowerProfilesTest(unittest.TestCase):
    def setUp(self):
        self.velocity = None
        self.attitude = None
        self.velocity_status = None
        self.attitude_reference_count = 0
        self.velocity_reference_count = 0
        self.error_pub = rospy.Publisher('/profile_test/error', NormalizedError,
                                         queue_size=10)
        self.state_pub = rospy.Publisher('/profile_test/state', Odometry,
                                         queue_size=10)
        self.vfr_pub = rospy.Publisher('/mavros/vfr_hud', VFR_HUD,
                                       queue_size=10)
        self.test_airspeed = 18.0
        rospy.Subscriber('/profile_velocity_test/follower_command',
                         FollowerCommand, self._velocity_cb)
        rospy.Subscriber('/profile_attitude_test/follower_command',
                         FollowerCommand, self._attitude_cb)
        rospy.Subscriber('/profile_velocity_test/follower_status',
                         FollowerStatus,
                         lambda msg: setattr(self, 'velocity_status', msg))
        rospy.Subscriber('/profile_test/attitude_reference', Odometry,
                         lambda _: setattr(self, 'attitude_reference_count',
                                           self.attitude_reference_count + 1))
        rospy.Subscriber('/profile_test/velocity_reference', Odometry,
                         lambda _: setattr(self, 'velocity_reference_count',
                                           self.velocity_reference_count + 1))
        for service in ('/profile_velocity_test/start',
                        '/profile_attitude_test/start',
                        '/profile_velocity_test/set_profile',
                        '/profile_attitude_test/set_profile'):
            rospy.wait_for_service(service, timeout=10)
        rospy.ServiceProxy('/profile_velocity_test/start', SetBool)(True)
        rospy.ServiceProxy('/profile_attitude_test/start', SetBool)(True)

    def _velocity_cb(self, message):
        self.velocity = message

    def _attitude_cb(self, message):
        self.attitude = message

    def _drive(self, seconds=0.60, angular=True, confidence=0.95):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            error = NormalizedError()
            error.header.stamp = rospy.Time.now()
            error.error_x = 0.25
            error.error_y = -0.18
            error.error_size = -0.30
            error.error_valid = True
            error.target_visible = True
            error.confidence = confidence
            error.has_angular_error = angular
            error.yaw_error_rad = 0.12
            error.pitch_error_rad = -0.08
            self.error_pub.publish(error)
            state = Odometry()
            state.header.stamp = rospy.Time.now()
            state.header.frame_id = 'uav1/odom'
            state.pose.pose.orientation.w = 1.0
            self.state_pub.publish(state)
            hud = VFR_HUD()
            hud.header.stamp = rospy.Time.now()
            hud.airspeed = self.test_airspeed
            hud.groundspeed = self.test_airspeed
            self.vfr_pub.publish(hud)
            time.sleep(0.03)

    def _drive_lost(self, seconds=0.20):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            error = NormalizedError()
            error.header.stamp = rospy.Time.now()
            error.error_valid = False
            error.target_visible = False
            error.confidence = 0.0
            self.error_pub.publish(error)
            time.sleep(0.03)

    def test_all_profiles_and_capability_gate(self):
        group = rospy.get_param('~profile_group', 'all')
        velocity_service = rospy.ServiceProxy(
            '/profile_velocity_test/set_profile', SetMode)
        attitude_service = rospy.ServiceProxy(
            '/profile_attitude_test/set_profile', SetMode)
        lateral_service = rospy.ServiceProxy(
            '/profile_velocity_test/set_mode', SetMode)

        profiles = []
        if group in ('all', 'velocity'):
            profiles += ['mc_velocity_ground', 'mc_velocity_distance',
                         'mc_velocity_position', 'mc_velocity_chase']
        if group in ('all', 'gimbal'):
            profiles += ['gm_velocity_chase', 'gm_velocity_vector']
        for profile in profiles:
            self.assertTrue(velocity_service(profile).success, profile)
            self._drive()
            command = self.velocity
            self.assertIsNotNone(command)
            self.assertEqual(command.follower_profile, profile)
            self.assertEqual(command.control_mode, 'velocity_body')
            self.assertTrue(command.profile_supported)
            self.assertTrue(command.command_valid)
            if profile == 'mc_velocity_ground':
                self.assertGreater(command.velocity_forward, 0.0)
                self.assertNotEqual(command.velocity_right, 0.0)
                self.assertEqual(command.yaw_rate_deg_s, 0.0)
            elif profile == 'mc_velocity_distance':
                self.assertEqual(command.velocity_forward, 0.0)
                self.assertNotEqual(command.velocity_right, 0.0)
                self.assertNotEqual(command.yaw_rate_deg_s, 0.0)

            elif profile == 'mc_velocity_position':
                self.assertEqual(command.velocity_forward, 0.0)
                self.assertEqual(command.velocity_right, 0.0)
                self.assertNotEqual(command.yaw_rate_deg_s, 0.0)
            elif profile in ('mc_velocity_chase', 'gm_velocity_chase'):
                self.assertGreater(command.velocity_forward, 0.0)
                self.assertEqual(command.velocity_right, 0.0)
                self.assertNotEqual(command.yaw_rate_deg_s, 0.0)

        if group in ('all', 'velocity'):
            self.assertGreater(self.velocity_reference_count, 0)

        if group in ('all', 'gimbal'):
            self.assertTrue(lateral_service('sideslip').success)
            self.assertTrue(velocity_service('gm_velocity_vector').success)
            self._drive()
            self.assertNotEqual(self.velocity.velocity_right, 0.0)
            self.assertEqual(self.velocity.yaw_rate_deg_s, 0.0)
            # Direct-vector geometry must use the raw 0.12 rad LOS yaw. For a
            # horizontal mount, right/forward is tan(yaw), independent of speed.
            ratio = abs(self.velocity.velocity_right /
                        max(1e-6, self.velocity.velocity_forward))
            self.assertAlmostEqual(ratio, 0.1206, delta=0.08)

            # A temporary loss of tracker angular fields must not interrupt
            # command validity. The requested GM identity is retained while
            # the related normalized-error MC profile supplies the output.
            self._drive(0.20, angular=False)
            self.assertTrue(self.velocity.command_valid)
            self.assertEqual(self.velocity.follower_profile,
                             'gm_velocity_vector')
            self.assertIsNotNone(self.velocity_status)
            self.assertTrue(self.velocity_status.profile_fallback_active)
            self.assertEqual(self.velocity_status.effective_profile,
                             'mc_velocity_distance')
            self._drive(0.20, angular=True)
            self.assertFalse(self.velocity_status.profile_fallback_active)
            self.assertEqual(self.velocity_status.effective_profile,
                             'gm_velocity_vector')

        if group in ('all', 'velocity'):
            # Switching between velocity profiles is bumpless. Immediately
            # after a chase->position transition, the previous feasible
            # forward command is blended instead of being reset to zero.
            self.assertTrue(velocity_service('mc_velocity_chase').success)
            self._drive(0.35)
            before = self.velocity.velocity_forward
            self.assertGreater(before, 0.0)
            self.assertTrue(velocity_service('mc_velocity_position').success)
            self._drive(0.08)
            self.assertTrue(self.velocity_status.profile_transition_active)
            self.assertEqual(self.velocity_status.previous_profile,
                             'mc_velocity_chase')
            self.assertGreater(self.velocity.velocity_forward, 0.0)
            self.assertLessEqual(abs(self.velocity.velocity_forward - before),
                                 0.25)
            self._drive(0.60)
            self.assertFalse(self.velocity_status.profile_transition_active)

            # Confidence hysteresis retains an established lock at 0.46
            # (acquisition threshold is 0.5, release threshold is 0.42).
            self._drive(0.12, confidence=0.95)
            self._drive(0.12, confidence=0.46)
            self.assertTrue(self.velocity.command_valid)
            self._drive(0.12, confidence=0.35)
            self.assertTrue(self.velocity.target_lost)

            self.assertGreater(self.velocity_status.loop_actual_rate, 40.0)
            self.assertLess(self.velocity_status.compute_time_ms, 10.0)
            self.assertLess(self.velocity_status.end_to_end_latency_ms, 250.0)

            # Capture-time age is now part of the safety path.  A message can
            # arrive promptly while still carrying an old GPU measurement; it
            # must be limited first and rejected after the control timeout.
            delayed = NormalizedError()
            delayed.header.stamp = rospy.Time.now()
            delayed.capture_timestamp = delayed.header.stamp - rospy.Duration(0.08)
            delayed.error_x = 0.25
            delayed.error_y = -0.18
            delayed.error_size = -0.30
            delayed.error_valid = True
            delayed.target_visible = True
            delayed.confidence = 0.95
            self.error_pub.publish(delayed)
            rospy.sleep(0.12)
            self.assertIsNotNone(self.velocity_status)
            self.assertTrue(self.velocity_status.latency_limited)
            self.assertGreater(self.velocity_status.tracker_capture_age_ms, 120.0)
            delayed.capture_timestamp = rospy.Time.now() - rospy.Duration(0.35)
            delayed.header.stamp = rospy.Time.now()
            self.error_pub.publish(delayed)
            rospy.sleep(0.12)
            self.assertTrue(self.velocity_status.latency_abort)

        if group in ('all', 'attitude'):
            self.assertFalse(velocity_service('mc_attitude_rate').success)
            self.assertFalse(velocity_service('fw_attitude_rate').success)
            self.assertFalse(velocity_service('not_a_profile').success)

            for profile in ('mc_attitude_rate', 'fw_attitude_rate'):
                self.assertTrue(attitude_service(profile).success, profile)
                self._drive()
                command = self.attitude
                self.assertIsNotNone(command)
                self.assertEqual(command.follower_profile, profile)
                self.assertEqual(command.control_mode, 'attitude_rate')
                self.assertTrue(command.profile_supported)
                self.assertTrue(command.command_valid)
                self.assertGreater(command.thrust, 0.0)
                self.assertNotEqual(command.roll_rate_deg_s, 0.0)
                self.assertNotEqual(command.pitch_rate_deg_s, 0.0)
                if profile == 'mc_attitude_rate':
                    self._drive_lost()
                    command = self.attitude
                    self.assertTrue(command.target_lost)
                    self.assertTrue(command.command_valid)
                    self.assertEqual(command.roll_rate_deg_s, 0.0)
                    self.assertEqual(command.pitch_rate_deg_s, 0.0)
                    self.assertEqual(command.yaw_rate_deg_s, 0.0)
                    self.assertGreater(command.thrust, 0.0)
                if profile == 'fw_attitude_rate':
                    # Fresh low airspeed must preempt L1/TECS with the explicit
                    # nose-down/full-thrust stall recovery command.
                    self.test_airspeed = 8.0
                    self._drive(0.20)
                    command = self.attitude
                    self.assertEqual(command.roll_rate_deg_s, 0.0)
                    self.assertLess(command.pitch_rate_deg_s, 0.0)
                    self.assertEqual(command.yaw_rate_deg_s, 0.0)
                    self.assertAlmostEqual(command.thrust, 1.0, delta=0.01)
                    # On loss, wait just beyond the test-local timeout and
                    # verify a valid coordinated orbit replaces stale pixels.
                    self.test_airspeed = 18.0
                    self._drive(0.15)
                    self._drive_lost(0.60)
                    command = self.attitude
                    self.assertTrue(command.target_lost)
                    self.assertTrue(command.command_valid)
                    self.assertNotEqual(command.roll_rate_deg_s, 0.0)
                    self.assertNotEqual(command.yaw_rate_deg_s, 0.0)

            # The XD position-reference adapter must fail closed for
            # attitude-rate commands even when the core is enabled.
            self.assertEqual(self.attitude_reference_count, 0)


if __name__ == '__main__':
    rospy.init_node('test_follower_profiles')
    rostest.rosrun('follower', 'follower_profiles', FollowerProfilesTest)
