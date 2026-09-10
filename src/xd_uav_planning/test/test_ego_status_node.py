#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from std_srvs.srv import SetBool, SetBoolResponse
from xd_uav_task_allocate.msg import PlannerStatus


class EgoStatusNodeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.statuses = []
        self.forwarded = []
        self.gate_calls = []
        self.owner_calls = []
        self.gate_service = rospy.Service(
            "/uav1/test_mux/set_enabled", SetBool, self._set_enabled)
        self.owner_service = rospy.Service(
            "/uav1/test_mux/select_ego", SetBool, self._select_owner)
        self.goal_publisher = rospy.Publisher(
            "/uav1/planning/goal", PoseStamped, queue_size=1)
        self.candidate_publisher = rospy.Publisher(
            "/uav1/ego/reference_candidate", PositionTarget, queue_size=1)
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

    def _set_enabled(self, request):
        with self.lock:
            self.gate_calls.append(bool(request.data))
        return SetBoolResponse(True, "test gate")

    def _select_owner(self, request):
        with self.lock:
            self.owner_calls.append(bool(request.data))
        return SetBoolResponse(True, "test owner")

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

    def test_fresh_candidate_acquires_owner_and_releases_output(self):
        self._wait(
            lambda: self.goal_publisher.get_num_connections() > 0 and
                    self.candidate_publisher.get_num_connections() > 0,
            5.0, "goal and candidate subscribers")
        self._publish_goal(2.0)
        self._wait(lambda: self._counts()[1] >= 1, 3.0,
                   "validated EGO goal")
        with self.lock:
            self.assertEqual(PlannerStatus.PLANNING, self.statuses[-1].state)
            self.assertAlmostEqual(2.0, self.forwarded[-1].pose.position.z)
            self.assertEqual([False], self.gate_calls)
            self.assertEqual([], self.owner_calls)

        candidate = PositionTarget()
        candidate.header.stamp = rospy.Time.now()
        candidate.header.frame_id = "world"
        self.candidate_publisher.publish(candidate)
        self._wait(
            lambda: bool(self.owner_calls) and self.gate_calls == [False, True],
            3.0, "owner acquisition and output release")
        with self.lock:
            self.assertEqual([True], self.owner_calls)
            self.assertEqual("world", self.forwarded[-1].header.frame_id)


if __name__ == "__main__":
    rospy.init_node("test_ego_status_node")
    rostest.rosrun("xd_uav_planning", "ego_status_node",
                   EgoStatusNodeTest)
