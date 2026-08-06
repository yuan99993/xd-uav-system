#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest

from pod_msgs.msg import GimbalCommand, GimbalState
from pod_msgs.srv import ManageGimbalLease, ManageGimbalLeaseRequest
from std_srvs.srv import SetBool


class CommandArbiterTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._command = None
        self._state_pub = rospy.Publisher('/arbiter_test/state', GimbalState,
                                          queue_size=1)
        self._manual_pub = rospy.Publisher('/arbiter_test/manual', GimbalCommand,
                                           queue_size=1)
        self._safety_pub = rospy.Publisher('/arbiter_test/safety', GimbalCommand,
                                           queue_size=1)
        rospy.Subscriber('/arbiter_test/command', GimbalCommand,
                         self._command_callback, queue_size=1)
        rospy.wait_for_service('/pod/gimbal/manage_control_lease', 5.0)
        rospy.wait_for_service('/pod/gimbal/emergency_stop', 5.0)
        self._lease = rospy.ServiceProxy('/pod/gimbal/manage_control_lease',
                                        ManageGimbalLease)
        self._emergency = rospy.ServiceProxy('/pod/gimbal/emergency_stop', SetBool)
        deadline = time.monotonic() + 5.0
        while self._state_pub.get_num_connections() == 0 and time.monotonic() < deadline:
            rospy.sleep(0.01)
        self.assertGreater(self._state_pub.get_num_connections(), 0)

    def _command_callback(self, message):
        with self._lock:
            self._command = message

    def _wait(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.sleep(0.01)
        return False

    def _publish_healthy_state(self):
        state = GimbalState()
        state.header.stamp = rospy.Time.now()
        state.connected = True
        state.stabilized = True
        state.attitude_valid = True
        self._state_pub.publish(state)

    def _publish_manual(self):
        command = GimbalCommand()
        command.header.stamp = rospy.Time.now()
        command.source = 'manual_ui'
        command.mode = 'rate'
        command.target_rate_deg_s.z = 7.0
        self._manual_pub.publish(command)

    def _publish_safety(self):
        command = GimbalCommand()
        command.header.stamp = rospy.Time.now()
        command.source = 'safety_supervisor'
        command.mode = 'hold'
        self._safety_pub.publish(command)

    def test_manual_lease_safety_override_and_emergency_hold(self):
        request = ManageGimbalLeaseRequest()
        request.acquire = True
        request.requester = 'manual_ui'
        request.resource = 'gimbal'
        request.priority = 10
        request.ttl_sec = 1.0
        self.assertTrue(self._lease(request).success)

        end = time.monotonic() + 0.25
        while time.monotonic() < end:
            self._publish_healthy_state()
            self._publish_manual()
            rospy.sleep(0.02)
        self.assertTrue(self._wait(
            lambda: self._command is not None and
            self._command.source == 'manual_ui' and
            abs(self._command.target_rate_deg_s.z - 7.0) < 1e-4))

        end = time.monotonic() + 0.15
        while time.monotonic() < end:
            self._publish_safety()
            rospy.sleep(0.02)
        self.assertTrue(self._wait(
            lambda: self._command is not None and
            self._command.source == 'safety_supervisor' and
            self._command.mode == 'hold'))

        self.assertTrue(self._emergency(True).success)
        self.assertTrue(self._wait(
            lambda: self._command is not None and
            self._command.source == 'pod_command_arbiter' and
            self._command.emergency_stop))


if __name__ == '__main__':
    rospy.init_node('pod_command_arbiter_test')
    rostest.rosrun('pod_command_arbiter', 'command_arbiter', CommandArbiterTest)
