#!/usr/bin/env python3

import threading
import unittest

import rosgraph
import rospy
import rostest
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool
from xd_uav_controller.msg import ControlState


class BridgeNodeTest(unittest.TestCase):
    def setUp(self):
        self.odometry = None
        self.candidate = None
        self.healthy = None
        self.lock = threading.Lock()
        self.state_pub = rospy.Publisher(
            "/uav1/control_manager/state", ControlState, queue_size=10)
        self.command_pub = rospy.Publisher(
            "/uav1/ego/position_command", PositionCommand, queue_size=10)
        rospy.Subscriber(
            "/uav1/ego/odometry", Odometry, self._set_odometry)
        rospy.Subscriber(
            "/uav1/ego/reference_candidate", PositionTarget,
            self._set_candidate)
        rospy.Subscriber(
            "/uav1/ego/bridge/healthy", Bool, self._set_healthy)

    def _set_odometry(self, message):
        with self.lock:
            self.odometry = message

    def _set_candidate(self, message):
        with self.lock:
            self.candidate = message

    def _set_healthy(self, message):
        with self.lock:
            self.healthy = message.data

    @staticmethod
    def _state(stamp):
        message = ControlState()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.body_frame_id = "uav1/base_link"
        message.position_odom.x = 1.0
        message.position_odom.y = 2.0
        message.position_odom.z = 3.0
        message.velocity_odom.x = 4.0
        message.velocity_odom.y = 5.0
        message.velocity_odom.z = 6.0
        message.orientation_odom_body.w = 1.0
        message.state_valid = True
        message.localization_valid = True
        message.odometry_fresh = True
        return message

    @staticmethod
    def _command(stamp):
        message = PositionCommand()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.position.x = 7.0
        message.position.y = 8.0
        message.position.z = 9.0
        message.velocity.x = 1.0
        message.velocity.y = 2.0
        message.velocity.z = 3.0
        message.acceleration.x = 0.1
        message.acceleration.y = 0.2
        message.acceleration.z = 0.3
        message.yaw = 0.4
        message.yaw_dot = 0.5
        message.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return message

    def test_mapping_health_timeout_and_no_canonical_connection(self):
        # A manager startup sample can have an empty frame.  It must be
        # rejected before tf2 lookup; subsequent valid input proves that the
        # bridge remains alive and recovers normally.
        empty_frame = self._state(rospy.Time.now())
        empty_frame.header.frame_id = ""
        self.state_pub.publish(empty_frame)
        rospy.sleep(0.1)
        with self.lock:
            self.assertFalse(self.healthy)

        deadline = rospy.Time.now() + rospy.Duration(3.0)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            now = rospy.Time.now()
            self.state_pub.publish(self._state(now))
            self.command_pub.publish(self._command(now))
            with self.lock:
                if self.odometry and self.candidate and self.healthy:
                    break
            rate.sleep()

        with self.lock:
            self.assertIsNotNone(self.odometry)
            self.assertIsNotNone(self.candidate)
            self.assertTrue(self.healthy)
            self.assertEqual("world", self.odometry.header.frame_id)
            self.assertEqual(4.0, self.odometry.twist.twist.linear.x)
            self.assertEqual(PositionTarget.FRAME_LOCAL_NED,
                             self.candidate.coordinate_frame)
            self.assertEqual(0, self.candidate.type_mask)
            self.assertEqual(7.0, self.candidate.position.x)
            self.assertEqual(0.3,
                             self.candidate.acceleration_or_force.z)
            self.assertEqual(0.5, self.candidate.yaw_rate)

        with self.lock:
            self.candidate = None
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            now = rospy.Time.now()
            command = self._command(now)
            command.yaw_dot = float("nan")
            self.state_pub.publish(self._state(now))
            self.command_pub.publish(command)
            with self.lock:
                if self.candidate is not None:
                    break
            rate.sleep()
        with self.lock:
            self.assertIsNotNone(self.candidate)
            self.assertEqual(PositionTarget.IGNORE_YAW_RATE,
                             self.candidate.type_mask)
            self.assertEqual(0.0, self.candidate.yaw_rate)

        publications, subscriptions, _services = rosgraph.Master(
            rospy.get_name()).getSystemState()
        canonical = "/uav1/control/reference/setpoint"
        bridge_name = "/uav1/ego_bridge"
        self.assertNotIn(
            bridge_name,
            dict(publications).get(canonical, []))
        self.assertNotIn(
            bridge_name,
            dict(subscriptions).get(canonical, []))
        rospy.sleep(0.35)
        with self.lock:
            self.assertFalse(self.healthy)


if __name__ == "__main__":
    rospy.init_node("ego_bridge_node_test")
    rostest.rosrun(
        "xd_uav_planning", "ego_bridge_node_test", BridgeNodeTest)
