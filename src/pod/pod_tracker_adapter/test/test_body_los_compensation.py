#!/usr/bin/env python3
import math
import threading
import time
import unittest

import rospy
import rostest

from pod_msgs.msg import GimbalState
from tracker.msg import NormalizedError


class BodyLosCompensationTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._output = None
        self._error_pub = rospy.Publisher('/body_los_test/input', NormalizedError,
                                          queue_size=1)
        self._gimbal_pub = rospy.Publisher('/body_los_test/gimbal', GimbalState,
                                           queue_size=1)
        rospy.Subscriber('/body_los_test/output', NormalizedError,
                         self._callback, queue_size=1)

    def _callback(self, message):
        with self._lock:
            self._output = message

    def _wait(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if predicate():
                    return True
            rospy.sleep(0.01)
        return False

    @staticmethod
    def _error():
        message = NormalizedError()
        message.header.stamp = rospy.Time.now()
        message.capture_timestamp = message.header.stamp
        message.error_valid = True
        message.target_visible = True
        message.control_measurement_ready = True
        message.confidence = 0.9
        message.control_confidence = 0.9
        message.has_angular_error = True
        return message

    @staticmethod
    def _gimbal(connected=True):
        state = GimbalState()
        state.header.stamp = rospy.Time.now()
        state.connected = connected
        state.stabilized = connected
        state.attitude_valid = connected
        state.attitude_deg.z = 90.0
        return state

    def test_dynamic_gimbal_pose_transforms_camera_los_and_fails_closed(self):
        self.assertTrue(self._wait(lambda: self._error_pub.get_num_connections() > 0))
        self._gimbal_pub.publish(self._gimbal())
        rospy.sleep(0.05)
        self._error_pub.publish(self._error())
        self.assertTrue(self._wait(
            lambda: self._output is not None and
                    self._output.fusion_status == 'BODY_LOS_GIMBAL_COMPENSATED'))
        self.assertAlmostEqual(math.pi / 2.0, self._output.yaw_error_rad, delta=0.02)
        self.assertTrue(self._output.error_valid)
        self.assertTrue(self._output.control_measurement_ready)

        self._gimbal_pub.publish(self._gimbal(False))
        rospy.sleep(0.05)
        self._error_pub.publish(self._error())
        self.assertTrue(self._wait(
            lambda: self._output is not None and
                    self._output.fusion_status == 'BODY_LOS_GIMBAL_STATE_UNAVAILABLE'))
        self.assertFalse(self._output.error_valid)
        self.assertFalse(self._output.control_measurement_ready)


if __name__ == '__main__':
    rospy.init_node('body_los_compensation_test')
    rostest.rosrun('pod_tracker_adapter', 'body_los_compensation',
                   BodyLosCompensationTest)
