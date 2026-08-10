#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest

from mavros_msgs.msg import PositionTarget, State
from pod_msgs.msg import GimbalState
from std_srvs.srv import SetBool

from follower.msg import FollowerCommand, FollowerStatus
from follower.srv import ManageControlLease, ManageControlLeaseRequest
from tracker.msg import NormalizedError


class FollowerControlGuardInterfacesTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._command = None
        self._status = None
        self._body_command = None
        self._body_target = None
        self._error_pub = rospy.Publisher(
            '/follower_guard_test/error', NormalizedError, queue_size=1)
        self._gimbal_state_pub = rospy.Publisher(
            '/follower_guard_test/gimbal_state', GimbalState, queue_size=1,
            latch=True)
        rospy.Subscriber('/follower_guard_test/follower_command',
                         FollowerCommand, self._command_callback,
                         queue_size=1)
        rospy.Subscriber('/follower_guard_test/follower_status',
                         FollowerStatus, self._status_callback,
                         queue_size=1)
        self._body_error_pub = rospy.Publisher(
            '/body_test/error', NormalizedError, queue_size=1)
        self._body_state_pub = rospy.Publisher(
            '/body_test/mavros/state', State, queue_size=1, latch=True)
        rospy.Subscriber('/follower_mavros_body_test/follower_command',
                         FollowerCommand, self._body_command_callback,
                         queue_size=1)
        rospy.Subscriber('/body_test/setpoint', PositionTarget,
                         self._body_target_callback, queue_size=1)
        rospy.wait_for_service('/follower_guard_test/manage_control_lease', 5.0)
        rospy.wait_for_service('/follower_guard_test/start', 5.0)
        rospy.wait_for_service('/follower_mavros_body_test/start', 5.0)
        self._lease = rospy.ServiceProxy(
            '/follower_guard_test/manage_control_lease', ManageControlLease)
        self._start = rospy.ServiceProxy('/follower_guard_test/start', SetBool)
        self._body_start = rospy.ServiceProxy(
            '/follower_mavros_body_test/start', SetBool)
        deadline = time.monotonic() + 5.0
        while self._gimbal_state_pub.get_num_connections() == 0 and \
                time.monotonic() < deadline:
            rospy.sleep(0.01)
        self.assertGreater(self._gimbal_state_pub.get_num_connections(), 0)
        self._publish_gimbal_state()

    def _publish_gimbal_state(self):
        """Publish a fresh healthy direct state for the guarded follower."""
        gimbal = GimbalState()
        gimbal.header.stamp = rospy.Time.now()
        gimbal.connected = True
        gimbal.stabilized = True
        gimbal.attitude_valid = True
        gimbal.parent_frame = 'base_link'
        gimbal.gimbal_frame = 'pod_gimbal_link'
        gimbal.optical_frame = 'pod_camera_optical_frame'
        self._gimbal_state_pub.publish(gimbal)

    def _command_callback(self, message):
        with self._lock:
            self._command = message

    def _status_callback(self, message):
        with self._lock:
            self._status = message

    def _body_command_callback(self, message):
        with self._lock:
            self._body_command = message

    def _body_target_callback(self, message):
        with self._lock:
            self._body_target = message

    def _wait(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.sleep(0.01)
        return False

    def _publish_error(self, with_capture_timestamp):
        # The guarded node deliberately treats GimbalState as a short-lived
        # hardware-health signal, so refresh it along with each control input.
        self._publish_gimbal_state()
        message = NormalizedError()
        message.header.stamp = rospy.Time.now()
        if with_capture_timestamp:
            message.capture_timestamp = message.header.stamp
        message.error_x = 0.2
        message.error_y = -0.1
        message.error_size = -0.2
        message.error_valid = True
        message.target_visible = True
        message.confidence = 0.95
        self._error_pub.publish(message)

    def test_lease_timestamp_and_expiry_gates(self):
        self.assertTrue(self._wait(
            lambda: self._error_pub.get_num_connections() > 0))

        denied_start = self._start(True)
        self.assertFalse(denied_start.success)

        wrong_backend = ManageControlLeaseRequest()
        wrong_backend.acquire = True
        wrong_backend.requester = 'test_ui'
        wrong_backend.output_backend = 'mavros_body'
        wrong_backend.lease_duration_sec = 0.8
        self.assertFalse(self._lease(wrong_backend).success)

        acquire = ManageControlLeaseRequest()
        acquire.acquire = True
        acquire.requester = 'test_ui'
        acquire.output_backend = 'command_only'
        acquire.lease_duration_sec = 0.8
        self.assertTrue(self._lease(acquire).success)
        self.assertTrue(self._start(True).success)

        self._publish_error(False)
        self.assertTrue(self._wait(
            lambda: self._status is not None and
                    self._status.control_state == 'INPUT_REJECTED'))
        self.assertFalse(self._command.command_valid)
        self.assertEqual('valid tracker capture timestamp is required',
                         self._command.invalid_reason)

        end = time.monotonic() + 0.25
        while time.monotonic() < end:
            self._publish_error(True)
            rospy.sleep(0.02)
        self.assertTrue(self._wait(
            lambda: self._command is not None and
                    self._command.command_valid and
                    self._command.control_state == 'FOLLOWING'))
        self.assertTrue(self._command.control_authorized)
        self.assertTrue(self._command.platform_ready)

        self.assertTrue(self._wait(
            lambda: self._status is not None and
                    self._status.control_state == 'LEASE_EXPIRED',
            timeout=2.0))
        self.assertFalse(self._command.command_valid)
        self.assertFalse(self._status.following_active)

        # The non-legacy MAVROS path uses explicit BODY_NED/FRD semantics, so
        # forward/right/down pass through without ENU sign guesses.
        state = State()
        state.connected = True
        state.armed = True
        state.mode = 'OFFBOARD'
        self._body_state_pub.publish(state)
        self.assertTrue(self._body_start(True).success)
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            body_error = NormalizedError()
            body_error.header.stamp = rospy.Time.now()
            body_error.capture_timestamp = body_error.header.stamp
            body_error.error_x = 0.25
            body_error.error_y = -0.20
            body_error.error_size = -0.2
            body_error.error_valid = True
            body_error.target_visible = True
            body_error.confidence = 0.95
            self._body_error_pub.publish(body_error)
            self._body_state_pub.publish(state)
            rospy.sleep(0.02)
        self.assertTrue(self._wait(
            lambda: self._body_target is not None and
                    self._body_command is not None and
                    self._body_command.command_valid))
        self.assertEqual(PositionTarget.FRAME_BODY_NED,
                         self._body_target.coordinate_frame)
        self.assertAlmostEqual(self._body_command.velocity_forward,
                               self._body_target.velocity.x, delta=1e-4)
        self.assertAlmostEqual(self._body_command.velocity_right,
                               self._body_target.velocity.y, delta=1e-4)
        self.assertAlmostEqual(self._body_command.velocity_down,
                               self._body_target.velocity.z, delta=1e-4)


if __name__ == '__main__':
    rospy.init_node('follower_control_guard_interfaces_test')
    rostest.rosrun('follower', 'follower_control_guard_interfaces',
                   FollowerControlGuardInterfacesTest)
