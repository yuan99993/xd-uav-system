#!/usr/bin/env python3

import unittest

import rospy
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from xd_uav_controller.msg import ControlState


class ReferenceIntegrationTest(unittest.TestCase):
    def setUp(self):
        self._canonical = []
        self._bridge_health = rospy.Publisher(
            "/uav1/ego/bridge/healthy", Bool, queue_size=1, latch=True)
        self._sensing_health = rospy.Publisher(
            "/uav1/ego/sensing/healthy", Bool, queue_size=1, latch=True)
        self._candidate = rospy.Publisher(
            "/uav1/ego/reference_candidate", PositionTarget, queue_size=5)
        self._state = rospy.Publisher(
            "/uav1/control_manager/state", ControlState, queue_size=5)
        self._canonical_sub = rospy.Subscriber(
            "/uav1/control/reference/setpoint", PositionTarget,
            self._canonical.append)

    @staticmethod
    def _wait(predicate, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return True
            rate.sleep()
        return False

    @staticmethod
    def _message():
        message = PositionTarget()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = PositionTarget.IGNORE_YAW_RATE
        message.position.x = 1.0
        message.position.y = 2.0
        message.position.z = 3.0
        return message

    @staticmethod
    def _state_message():
        message = ControlState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.body_frame_id = "uav1/base_link"
        message.position_odom.x = 1.0
        message.position_odom.y = 2.0
        message.position_odom.z = 3.0
        message.orientation_odom_body.w = 1.0
        message.state_valid = True
        message.localization_valid = True
        message.odometry_fresh = True
        return message

    def test_explicit_owner_and_fail_closed_health(self):
        self.assertTrue(self._wait(
            lambda: self._candidate.get_num_connections() > 0 and
                    self._state.get_num_connections() > 0 and
                    self._bridge_health.get_num_connections() > 0 and
                    self._sensing_health.get_num_connections() > 0))
        self._bridge_health.publish(Bool(data=True))
        self._sensing_health.publish(Bool(data=True))
        for _ in range(4):
            self._candidate.publish(self._message())
            rospy.sleep(0.03)
        self.assertEqual(len(self._canonical), 0)

        rospy.wait_for_service("/uav1/reference_mux/select_ego", timeout=3.0)
        select_ego = rospy.ServiceProxy(
            "/uav1/reference_mux/select_ego", SetBool)
        response = select_ego(True)
        self.assertFalse(response.success)
        self.assertIn("baseline", response.message)

        for _ in range(10):
            self._bridge_health.publish(Bool(data=True))
            self._sensing_health.publish(Bool(data=True))
            self._state.publish(self._state_message())
            self._candidate.publish(self._message())
            rospy.sleep(0.03)
        response = select_ego(True)
        self.assertTrue(response.success, response.message)
        for _ in range(3):
            self._candidate.publish(self._message())
            rospy.sleep(0.03)
        self.assertTrue(self._wait(lambda: len(self._canonical) > 0))

        count = len(self._canonical)
        invalid = self._message()
        invalid.type_mask = 4096
        for _ in range(3):
            self._candidate.publish(invalid)
            rospy.sleep(0.03)
        rospy.sleep(0.10)
        self.assertEqual(len(self._canonical), count)

        self._sensing_health.publish(Bool(data=False))
        rospy.sleep(0.30)
        count = len(self._canonical)
        for _ in range(3):
            self._candidate.publish(self._message())
            rospy.sleep(0.03)
        rospy.sleep(0.10)
        self.assertEqual(len(self._canonical), count)

        rospy.wait_for_service("/uav1/reference_mux/set_enabled", timeout=3.0)
        set_enabled = rospy.ServiceProxy(
            "/uav1/reference_mux/set_enabled", SetBool)
        self.assertTrue(set_enabled(False).success)
        self._bridge_health.publish(Bool(data=True))
        self._sensing_health.publish(Bool(data=True))
        rospy.sleep(0.05)
        count = len(self._canonical)
        for _ in range(4):
            self._candidate.publish(self._message())
            rospy.sleep(0.03)
        rospy.sleep(0.10)
        self.assertEqual(len(self._canonical), count)


if __name__ == "__main__":
    rospy.init_node("test_reference_integration")
    import rostest
    rostest.rosrun("xd_uav_planning", "test_reference_integration",
                   ReferenceIntegrationTest)
