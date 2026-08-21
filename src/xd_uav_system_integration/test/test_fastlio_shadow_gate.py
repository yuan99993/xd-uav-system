#!/usr/bin/env python3

import threading
import unittest

from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
import rospy
import rostest
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from xd_uav_state_estimators.srv import (
    SwitchLocalizationSource, SwitchLocalizationSourceResponse)


class FastlioShadowGateTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._healthy = False
        self._requested = []
        self._gated_count = 0
        self._odom_pub = rospy.Publisher(
            "/uav1/fastlio/Odometry", Odometry, queue_size=20)
        self._state_pub = rospy.Publisher(
            "/uav1/mavros/state", State, queue_size=10)
        rospy.Subscriber("/uav1/fastlio_shadow/healthy", Bool,
                         self._health_callback)
        rospy.Subscriber("/uav1/fastlio_gated/Odometry", Odometry,
                         self._gated_callback)
        self._service = rospy.Service(
            "/uav1/state_estimator/switch_source",
            SwitchLocalizationSource, self._switch_callback)

    def _health_callback(self, message):
        with self._lock:
            self._healthy = message.data

    def _gated_callback(self, _message):
        with self._lock:
            self._gated_count += 1

    def _switch_callback(self, request):
        self._requested.append(request.source_name)
        return SwitchLocalizationSourceResponse(
            success=True, message="selected", active_source=request.source_name,
            requested_source=request.source_name, automatic=False)

    @staticmethod
    def _odom():
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "uav1/fastlio_origin"
        message.child_frame_id = "uav1/mid360_imu"
        message.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14):
            message.pose.covariance[index] = 0.1
            message.twist.covariance[index] = 0.2
        return message

    def _publish(self, armed, duration=0.6):
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._odom_pub.publish(self._odom())
            self._state_pub.publish(State(armed=armed, connected=True))
            rate.sleep()

    def test_shadow_health_and_disarmed_switch_gate(self):
        self._publish(armed=False)
        with self._lock:
            self.assertTrue(self._healthy)
            self.assertEqual(0, self._gated_count)
        rospy.wait_for_service(
            "/uav1/fastlio_shadow_gate/switch_to_fastlio", timeout=2.0)
        switch = rospy.ServiceProxy(
            "/uav1/fastlio_shadow_gate/switch_to_fastlio", Trigger)
        response = switch()
        self.assertTrue(response.success, response.message)
        self.assertEqual(["fastlio"], self._requested)
        self._publish(armed=False, duration=0.2)
        with self._lock:
            self.assertGreater(self._gated_count, 0)
            gated_before_armed = self._gated_count

        self._publish(armed=True, duration=0.2)
        response = switch()
        self.assertFalse(response.success)
        self.assertEqual("vehicle_armed", response.message)
        self.assertEqual(["fastlio"], self._requested)
        rospy.sleep(0.1)
        with self._lock:
            gated_after_armed = self._gated_count
        self._publish(armed=False, duration=0.2)
        with self._lock:
            self.assertGreaterEqual(gated_after_armed, gated_before_armed)
            self.assertEqual(gated_after_armed, self._gated_count)

        # A new ground authorization plus explicit flight authorization is
        # the only path which may keep the relay open across arming.
        response = switch()
        self.assertTrue(response.success, response.message)
        rospy.wait_for_service(
            "/uav1/fastlio_shadow_gate/authorize_flight", timeout=2.0)
        authorize_flight = rospy.ServiceProxy(
            "/uav1/fastlio_shadow_gate/authorize_flight", Trigger)
        response = authorize_flight()
        self.assertTrue(response.success, response.message)
        with self._lock:
            count_before_flight = self._gated_count
        self._publish(armed=True, duration=0.2)
        with self._lock:
            self.assertGreater(self._gated_count, count_before_flight)
        end_flight = rospy.ServiceProxy(
            "/uav1/fastlio_shadow_gate/end_flight", Trigger)
        self.assertFalse(end_flight().success)
        self._publish(armed=False, duration=0.2)
        self.assertTrue(end_flight().success)

        rospy.sleep(0.3)
        with self._lock:
            self.assertFalse(self._healthy)


if __name__ == "__main__":
    rospy.init_node("test_fastlio_shadow_gate")
    rostest.rosrun("xd_uav_system_integration", "fastlio_shadow_gate",
                   FastlioShadowGateTest)
