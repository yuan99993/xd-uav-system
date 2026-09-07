#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from xd_uav_controller.msg import ControlState, PathStatus
from xd_uav_task_allocate.msg import PlannerStatus


class FixedwingPathBackendTest(unittest.TestCase):
    def setUp(self):
        self.forwarded = []
        self.statuses = []
        self.health = []
        self.state_pub = rospy.Publisher(
            "/uav1/control_manager/state", ControlState, queue_size=10)
        self.path_pub = rospy.Publisher(
            "/uav1/planning/mission_path", Path, queue_size=2)
        self.controller_status_pub = rospy.Publisher(
            "/uav1/controller/path_status", PathStatus, queue_size=10)
        self.forwarded_sub = rospy.Subscriber(
            "/uav1/control/reference/path", Path,
            self.forwarded.append, queue_size=2)
        self.status_sub = rospy.Subscriber(
            "/uav1/planning/status", PlannerStatus,
            self.statuses.append, queue_size=10)
        self.health_sub = rospy.Subscriber(
            "/uav1/planning/healthy", Bool,
            self.health.append, queue_size=10)
        self.running = True
        self.vehicle_type = ControlState.VEHICLE_FIXEDWING
        self.state_thread = threading.Thread(target=self._publish_state)
        self.state_thread.daemon = True
        self.state_thread.start()
        self._wait_for(lambda: self.state_pub.get_num_connections() > 0)
        self._wait_for(lambda: any(message.data for message in self.health))

    def tearDown(self):
        self.running = False
        self.state_thread.join(timeout=1.0)

    def _publish_state(self):
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            message = ControlState()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "uav1/odom"
            message.vehicle_type = self.vehicle_type
            message.state_valid = True
            message.localization_valid = True
            message.odometry_fresh = True
            self.state_pub.publish(message)
            rate.sleep()

    @staticmethod
    def _wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.02)
        raise AssertionError("condition timed out")

    @staticmethod
    def _path(path_id, frame="world"):
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = frame
        message.header.seq = path_id
        for x_value in (0.0, 20.0, 40.0):
            pose = PoseStamped()
            pose.header.seq = path_id
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = frame
            pose.pose.position.x = x_value
            pose.pose.position.z = 20.0
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def _publish_path_until(self, message, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            message.header.stamp = rospy.Time.now()
            for pose in message.poses:
                pose.header.stamp = message.header.stamp
            self.path_pub.publish(message)
            if predicate():
                return
            rospy.sleep(0.05)
        raise AssertionError("path result timed out")

    def _publish_controller_status_until(self, message, predicate,
                                         timeout=5.0):
        self._wait_for(
            lambda: self.controller_status_pub.get_num_connections() > 0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            message.header.stamp = rospy.Time.now()
            self.controller_status_pub.publish(message)
            if predicate():
                return
            rospy.sleep(0.05)
        observed = [(msg.goal_id, msg.state, msg.detail)
                    for msg in self.statuses[-10:]]
        raise AssertionError(
            "controller status result timed out; observed={!r}".format(
                observed))

    def test_valid_path_and_controller_status_mapping(self):
        self._wait_for(lambda: self.path_pub.get_num_connections() > 0)
        self._publish_path_until(
            self._path(42),
            lambda: bool(self.forwarded) and any(
                msg.goal_id == 42 and msg.state == PlannerStatus.PLANNING
                for msg in self.statuses))
        self.assertTrue(any(
            msg.goal_id == 42 and msg.state == PlannerStatus.PLANNING
            for msg in self.statuses))

        controller_path_id = self.forwarded[-1].header.seq
        accepted = PathStatus()
        accepted.path_id = controller_path_id
        accepted.state = PathStatus.ACCEPTED
        accepted.detail = "accepted"
        self._publish_controller_status_until(
            accepted,
            lambda: any(
                msg.goal_id == 42 and msg.detail.endswith("accepted")
                for msg in self.statuses))

        active = PathStatus()
        active.header.stamp = rospy.Time.now()
        active.path_id = controller_path_id
        active.state = PathStatus.ACTIVE
        active.progress = 0.5
        active.detail = "following"
        self._publish_controller_status_until(
            active,
            lambda: any(
                msg.goal_id == 42 and msg.state == PlannerStatus.ACTIVE
                for msg in self.statuses))

        complete = PathStatus()
        complete.header.stamp = rospy.Time.now()
        complete.path_id = controller_path_id
        complete.state = PathStatus.COMPLETED
        complete.progress = 1.0
        complete.detail = "done"
        self._publish_controller_status_until(
            complete,
            lambda: any(
                msg.goal_id == 42 and msg.state == PlannerStatus.REACHED
                for msg in self.statuses))

    def test_wrong_frame_and_wrong_vehicle_fail_closed(self):
        self._wait_for(lambda: self.path_pub.get_num_connections() > 0)
        before = len(self.forwarded)
        self._publish_path_until(
            self._path(50, "odom"),
            lambda: any(msg.goal_id == 50 and msg.state == PlannerStatus.FAILED
                        for msg in self.statuses))
        rospy.sleep(0.1)
        self.assertEqual(before, len(self.forwarded))

        self.vehicle_type = ControlState.VEHICLE_MULTIROTOR
        self._wait_for(lambda: self.health and not self.health[-1].data)
        self._publish_path_until(
            self._path(51),
            lambda: any(msg.goal_id == 51 and msg.state == PlannerStatus.FAILED
                        for msg in self.statuses))
        rospy.sleep(0.1)
        self.assertEqual(before, len(self.forwarded))


if __name__ == "__main__":
    rospy.init_node("fixedwing_path_backend_test")
    rostest.rosrun("xd_uav_planning", "fixedwing_path_backend_test",
                  FixedwingPathBackendTest)
