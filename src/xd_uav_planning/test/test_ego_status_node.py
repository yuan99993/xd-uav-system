#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from xd_uav_task_allocate.msg import PlannerStatus
from xd_uav_task_allocate.srv import CancelPlanning


class EgoStatusNodeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.statuses = []
        self.forwarded = []
        self.goal_publisher = rospy.Publisher(
            "/uav1/planning/goal", PoseStamped, queue_size=1)
        rospy.Subscriber("/uav1/planning/status", PlannerStatus,
                         self._status_callback, queue_size=10)
        rospy.Subscriber("/uav1/ego/validated_goal", PoseStamped,
                         self._forwarded_callback, queue_size=10)

    def _status_callback(self, message):
        with self.lock:
            self.statuses.append(message)

    def _forwarded_callback(self, message):
        with self.lock:
            self.forwarded.append(message)

    @staticmethod
    def _wait(predicate, timeout, description):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        raise AssertionError("timeout waiting for " + description)

    def _counts(self):
        with self.lock:
            return len(self.statuses), len(self.forwarded)

    def _publish_goal(self, z):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = 3.0
        goal.pose.position.y = -1.0
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)

    def test_arbitrary_altitude_forwarding_and_cancel(self):
        self._wait(lambda: self.goal_publisher.get_num_connections() > 0,
                   5.0, "goal subscriber")
        self._publish_goal(2.0)
        self._wait(lambda: self._counts()[1] >= 1, 3.0,
                   "validated EGO goal")
        with self.lock:
            self.assertEqual(PlannerStatus.PLANNING, self.statuses[-1].state)
            self.assertAlmostEqual(2.0,
                                   self.forwarded[-1].pose.position.z)

        rospy.wait_for_service("/uav1/planning/cancel", timeout=3.0)
        response = rospy.ServiceProxy(
            "/uav1/planning/cancel", CancelPlanning)(0, "test handoff")
        self.assertTrue(response.success)
        self._wait(lambda: self.statuses[-1].state == PlannerStatus.IDLE,
                   3.0, "cancelled status")

        self._publish_goal(6.0)
        self._wait(lambda: len(self.forwarded) >= 2, 3.0,
                   "second arbitrary-altitude goal")
        with self.lock:
            self.assertEqual(PlannerStatus.PLANNING, self.statuses[-1].state)
            self.assertAlmostEqual(6.0,
                                   self.forwarded[-1].pose.position.z)
            self.assertEqual("world", self.forwarded[-1].header.frame_id)


if __name__ == "__main__":
    rospy.init_node("test_ego_status_node")
    rostest.rosrun("xd_uav_planning", "ego_status_node",
                   EgoStatusNodeTest)
