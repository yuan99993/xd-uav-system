#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from xd_uav_controller.msg import ControlState, PathStatus
from xd_uav_planning.msg import NoFlyZone
from xd_uav_task_allocate.msg import PlannerStatus


class FixedwingPathBackendTest(unittest.TestCase):
    def setUp(self):
        self.forwarded = []
        self.statuses = []
        self.health = []
        self.state_pub = rospy.Publisher(
            "/uav1/control_manager/state", ControlState, queue_size=10)
        self.path_pub = rospy.Publisher(
            "/uav1/planning/task_path", Path, queue_size=2)
        self.zone_pub = rospy.Publisher(
            "/uav1/planning/no_fly_zone", NoFlyZone,
            queue_size=2, latch=True)
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
        self.position_x = 0.0
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
            message.header.frame_id = "world"
            message.vehicle_type = self.vehicle_type
            message.position_odom.x = self.position_x
            message.position_odom.z = 20.0
            message.course = 0.0
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

    @staticmethod
    def _zone(operation=NoFlyZone.OP_UPSERT):
        message = NoFlyZone()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.schema_version = NoFlyZone.CURRENT_SCHEMA_VERSION
        message.operation = operation
        message.zone_id = 0 if operation == NoFlyZone.OP_CLEAR else 71
        message.enabled = True
        message.zone_type = NoFlyZone.TYPE_NO_FLY
        message.min_altitude = 0.0
        message.max_altitude = 100.0
        message.valid_until = rospy.Time(0)
        for x_value, y_value in (
                (15.0, -4.0), (25.0, -4.0),
                (25.0, 4.0), (15.0, 4.0)):
            message.polygon.points.append(
                Point32(x=x_value, y=y_value, z=0.0))
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

    def test_no_fly_zone_adjusts_path_before_controller(self):
        self._wait_for(lambda: self.zone_pub.get_num_connections() > 0)
        zone = self._zone()
        self.zone_pub.publish(zone)
        rospy.sleep(0.1)
        before = len(self.forwarded)
        try:
            self._publish_path_until(
                self._path(61),
                lambda: len(self.forwarded) > before and any(
                    msg.goal_id == 61 and msg.state == PlannerStatus.PLANNING
                    and "no-fly path adjusted" in msg.detail
                    for msg in self.statuses))
            adjusted = self.forwarded[-1]
            self.assertGreater(len(adjusted.poses), 3)
            self.assertTrue(any(abs(pose.pose.position.y) > 6.0
                                for pose in adjusted.poses))
        finally:
            clear = self._zone(NoFlyZone.OP_CLEAR)
            clear.header.stamp = rospy.Time.now()
            self.zone_pub.publish(clear)
            rospy.sleep(0.1)

    def test_active_path_is_replaced_on_zone_upsert_and_remove(self):
        self._wait_for(lambda: self.path_pub.get_num_connections() > 0)
        self._wait_for(lambda: self.zone_pub.get_num_connections() > 0)
        clear = self._zone(NoFlyZone.OP_CLEAR)
        self.zone_pub.publish(clear)
        rospy.sleep(0.1)

        self._publish_path_until(
            self._path(81), lambda: bool(self.forwarded))
        initial_count = len(self.forwarded)
        initial_controller_id = self.forwarded[-1].header.seq
        accepted = PathStatus(path_id=initial_controller_id,
                              state=PathStatus.ACCEPTED,
                              detail="initial accepted")
        self._publish_controller_status_until(
            accepted, lambda: any(
                msg.goal_id == 81 and msg.detail.endswith("initial accepted")
                for msg in self.statuses))
        active = PathStatus(path_id=initial_controller_id,
                            state=PathStatus.ACTIVE,
                            detail="initial active")
        self._publish_controller_status_until(
            active, lambda: any(
                msg.goal_id == 81 and msg.state == PlannerStatus.ACTIVE
                for msg in self.statuses))

        self.position_x = 5.0
        rospy.sleep(0.15)
        self.zone_pub.publish(self._zone())
        self._wait_for(lambda: len(self.forwarded) > initial_count)
        detour = self.forwarded[-1]
        self.assertGreater(len(detour.poses), 3)
        self.assertAlmostEqual(detour.poses[0].pose.position.x, 5.0,
                               delta=1.0)
        self.assertTrue(any(
            msg.goal_id == 81 and "dynamic no-fly upsert" in msg.detail
            for msg in self.statuses))

        detour_count = len(self.forwarded)
        detour_controller_id = detour.header.seq
        accepted.path_id = detour_controller_id
        accepted.detail = "detour accepted"
        self._publish_controller_status_until(
            accepted, lambda: any(
                msg.goal_id == 81 and msg.detail.endswith("detour accepted")
                for msg in self.statuses))
        remove = self._zone(NoFlyZone.OP_REMOVE)
        remove.header.stamp = rospy.Time.now()
        self.zone_pub.publish(remove)
        self._wait_for(lambda: len(self.forwarded) > detour_count)
        restored = self.forwarded[-1]
        self.assertLess(len(restored.poses), len(detour.poses))
        self.assertAlmostEqual(restored.poses[0].pose.position.x, 5.0,
                               delta=1.0)
        self.assertTrue(any(
            msg.goal_id == 81 and "dynamic no-fly remove" in msg.detail
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
