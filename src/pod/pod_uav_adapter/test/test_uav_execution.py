#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest

from follower.msg import FollowerCommand
from mavros_msgs.msg import PositionTarget, State
from pod_msgs.msg import UavControlExecution, UavState


class UavExecutionTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._target = None
        self._execution = None
        self._state = None
        self._command_pub = rospy.Publisher('/uav_execution_test/follower_command',
                                            FollowerCommand, queue_size=1)
        self._mavros_state_pub = rospy.Publisher('/uav_execution_test/mavros/state',
                                                 State, queue_size=1, latch=True)
        rospy.Subscriber('/uav_execution_test/mavros/setpoint', PositionTarget,
                         self._target_callback, queue_size=1)
        rospy.Subscriber('/uav_execution_test/execution', UavControlExecution,
                         self._execution_callback, queue_size=1)
        rospy.Subscriber('/uav_execution_test/state', UavState,
                         self._state_callback, queue_size=1)

    def _target_callback(self, message):
        with self._lock:
            self._target = message

    def _execution_callback(self, message):
        with self._lock:
            self._execution = message

    def _state_callback(self, message):
        with self._lock:
            self._state = message

    def _wait(self, predicate, timeout=4.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not rospy.is_shutdown():
            with self._lock:
                if predicate():
                    return True
            rospy.sleep(0.01)
        return False

    @staticmethod
    def _command():
        command = FollowerCommand()
        command.header.stamp = rospy.Time.now()
        command.command_sequence = 42
        command.source_capture_timestamp = command.header.stamp
        command.command_valid = True
        command.profile_supported = True
        command.control_authorized = True
        command.platform_ready = True
        command.control_mode = 'velocity_body'
        command.velocity_forward = 1.5
        command.velocity_right = -0.4
        command.velocity_down = 0.2
        command.yaw_rate_deg_s = 12.0
        return command

    def test_single_writer_execution_ack_and_timeout_zero(self):
        state = State()
        state.connected = True
        state.armed = True
        state.mode = 'OFFBOARD'
        self._mavros_state_pub.publish(state)
        self.assertTrue(self._wait(lambda: self._command_pub.get_num_connections() > 0))
        self._command_pub.publish(self._command())
        self.assertTrue(self._wait(lambda: self._execution is not None and
                                   self._execution.accepted and
                                   self._execution.executing))
        self.assertEqual('mavros_direct', self._execution.backend)
        self.assertEqual(42, self._execution.source_command_sequence)
        self.assertEqual(PositionTarget.FRAME_BODY_NED, self._target.coordinate_frame)
        self.assertAlmostEqual(1.5, self._target.velocity.x, delta=1e-4)
        self.assertAlmostEqual(-0.4, self._target.velocity.y, delta=1e-4)
        self.assertAlmostEqual(0.2, self._target.velocity.z, delta=1e-4)
        self.assertTrue(self._state.connected)
        self.assertTrue(self._state.offboard_active)

        self.assertTrue(self._wait(lambda: self._execution is not None and
                                   self._execution.failsafe_active,
                                   timeout=2.0))
        self.assertAlmostEqual(0.0, self._target.velocity.x, delta=1e-4)
        self.assertAlmostEqual(0.0, self._target.velocity.y, delta=1e-4)
        self.assertAlmostEqual(0.0, self._target.velocity.z, delta=1e-4)


if __name__ == '__main__':
    rospy.init_node('uav_execution_test')
    rostest.rosrun('pod_uav_adapter', 'uav_execution', UavExecutionTest)
