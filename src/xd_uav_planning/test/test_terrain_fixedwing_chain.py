#!/usr/bin/env python3
"""M3 no-flight acceptance for the terrain-to-fixed-wing control chain."""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from xd_uav_controller.msg import ControlCommand, ControlState, PathStatus
from xd_uav_task_allocate.msg import PlannerStatus


class TerrainFixedwingChainTest(unittest.TestCase):
    START = (-2012.4, -1188.0, 2340.246826)
    GOAL = (2012.4, 1188.0, 1676.894348)
    GOAL_ID = 4242
    REJECTED_GOAL_ID = 4241

    def setUp(self):
        self.terrain_paths = []
        self.controller_paths = []
        self.terrain_statuses = []
        self.planner_statuses = []
        self.path_statuses = []
        self.commands = []
        self._running = True

        self.state_publisher = rospy.Publisher(
            "/uav1/control_manager/state", ControlState, queue_size=20)
        self.task_publisher = rospy.Publisher(
            "/uav1/planning/task_path", Path, queue_size=2)
        self.backend_status_publisher = rospy.Publisher(
            "/uav1/planning/backend_status", PlannerStatus, queue_size=2)
        self.subscribers = [
            rospy.Subscriber(
                "/uav1/planning/terrain_path", Path,
                self.terrain_paths.append, queue_size=2),
            rospy.Subscriber(
                "/uav1/control/reference/path", Path,
                self.controller_paths.append, queue_size=2),
            rospy.Subscriber(
                "/uav1/planning/terrain_status", PlannerStatus,
                self.terrain_statuses.append, queue_size=20),
            rospy.Subscriber(
                "/uav1/planning/status", PlannerStatus,
                self.planner_statuses.append, queue_size=20),
            rospy.Subscriber(
                "/uav1/controller/path_status", PathStatus,
                self.path_statuses.append, queue_size=20),
            rospy.Subscriber(
                "/uav1/controller/command", ControlCommand,
                self.commands.append, queue_size=20),
        ]
        self.state_thread = threading.Thread(target=self._publish_state)
        self.state_thread.daemon = True
        self.state_thread.start()

    def tearDown(self):
        self._running = False
        self.state_thread.join(timeout=1.0)

    def _publish_state(self):
        course = math.atan2(
            self.GOAL[1] - self.START[1], self.GOAL[0] - self.START[0])
        rate = rospy.Rate(30)
        while self._running and not rospy.is_shutdown():
            message = ControlState()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "world"
            message.body_frame_id = "uav1/base_link"
            message.vehicle_type = ControlState.VEHICLE_FIXEDWING
            message.airframe_type = ControlState.AIRFRAME_FIXED_WING
            message.flight_regime = ControlState.REGIME_FORWARD_FLIGHT
            message.vehicle_action = ControlState.ACTION_NAVIGATE
            message.position_odom.x = self.START[0]
            message.position_odom.y = self.START[1]
            message.position_odom.z = self.START[2]
            message.velocity_odom.x = 15.0 * math.cos(course)
            message.velocity_odom.y = 15.0 * math.sin(course)
            message.orientation_odom_body.z = math.sin(0.5 * course)
            message.orientation_odom_body.w = math.cos(0.5 * course)
            message.airspeed = 15.0
            message.groundspeed = 15.0
            message.course = course
            message.state_valid = True
            message.localization_valid = True
            message.odometry_fresh = True
            message.imu_fresh = True
            message.acceleration_fresh = True
            message.airspeed_valid = True
            message.stable = True
            self.state_publisher.publish(message)
            rate.sleep()

    @staticmethod
    def _wait_for(predicate, timeout=15.0, detail="condition"):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.02)
        raise AssertionError("timed out waiting for {}".format(detail))

    @staticmethod
    def _path_goal_id(message):
        if message.poses:
            return int(message.poses[0].header.seq)
        return int(message.header.seq)

    @classmethod
    def _request(cls, goal_id, frame="world"):
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = frame
        for point in (cls.START, cls.GOAL):
            pose = PoseStamped()
            pose.header.seq = goal_id
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = frame
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def test_rejection_and_real_controller_handoff(self):
        self._wait_for(
            lambda: self.task_publisher.get_num_connections() == 1,
            detail="terrain request subscriber")
        self._wait_for(
            lambda: self.state_publisher.get_num_connections() >= 2,
            detail="backend and controller state subscribers")

        # A terrain rejection is terminal at the first stage: it must not
        # create either a terrain output or a controller input.
        self.task_publisher.publish(
            self._request(self.REJECTED_GOAL_ID, frame="wrong_frame"))
        self._wait_for(
            lambda: any(
                status.goal_id == self.REJECTED_GOAL_ID
                and status.state == PlannerStatus.FAILED
                and "request_frame_mismatch" in status.detail
                for status in self.terrain_statuses),
            detail="fail-closed terrain rejection")
        self._wait_for(
            lambda: any(
                status.goal_id == self.REJECTED_GOAL_ID
                and status.state == PlannerStatus.FAILED
                and "request_frame_mismatch" in status.detail
                for status in self.planner_statuses),
            detail="public rejected-goal status")
        rospy.sleep(0.2)
        self.assertFalse(any(
            self._path_goal_id(path) == self.REJECTED_GOAL_ID
            for path in self.terrain_paths))
        self.assertFalse(any(
            self._path_goal_id(path) == self.REJECTED_GOAL_ID
            for path in self.controller_paths))

        # A late status from an older controller goal cannot overwrite the
        # rejected active terrain request on the public status topic.
        self._wait_for(
            lambda: self.backend_status_publisher.get_num_connections() >= 1,
            detail="status coordinator backend subscriber")
        stale = PlannerStatus()
        stale.header.stamp = rospy.Time.now()
        stale.header.frame_id = "world"
        stale.goal_id = 999
        stale.state = PlannerStatus.ACTIVE
        stale.detail = "stale backend status"
        self.backend_status_publisher.publish(stale)
        rospy.sleep(0.2)
        self.assertFalse(any(
            status.goal_id == stale.goal_id
            for status in self.planner_statuses))

        # This is the fixed M1/M2 davosdorf case. The test uses the real
        # GeoTIFF, official planner core, existing fixed-wing backend and the
        # actual controller, while the aircraft state remains a no-flight
        # ROS fixture.
        self.task_publisher.publish(self._request(self.GOAL_ID))
        self._wait_for(
            lambda: any(
                status.goal_id == self.GOAL_ID
                and status.detail.startswith("terrain_path_ready:")
                for status in self.terrain_statuses),
            timeout=20.0,
            detail="validated terrain path")
        self._wait_for(
            lambda: any(
                self._path_goal_id(path) == self.GOAL_ID
                and len(path.poses) > 2
                for path in self.terrain_paths),
            detail="terrain path output")
        self._wait_for(
            lambda: any(
                self._path_goal_id(path) == self.GOAL_ID
                and len(path.poses) > 2
                for path in self.controller_paths),
            detail="fixed-wing backend handoff")
        self._wait_for(
            lambda: any(
                status.path_id == self.GOAL_ID
                and status.state in (PathStatus.ACCEPTED, PathStatus.ACTIVE)
                for status in self.path_statuses),
            detail="controller path acceptance")
        self._wait_for(
            lambda: any(
                command.valid
                and command.vehicle_type == ControlCommand.VEHICLE_FIXEDWING
                and command.active_backend == ControlCommand.BACKEND_FIXED_WING
                and command.reference_type == ControlCommand.REFERENCE_PATH
                and command.controller == "fixedwing_path_course_energy"
                for command in self.commands),
            detail="valid fixed-wing path control command")
        self._wait_for(
            lambda: any(
                status.goal_id == self.GOAL_ID
                and status.state == PlannerStatus.ACTIVE
                for status in self.planner_statuses),
            detail="end-to-end active planning status")


if __name__ == "__main__":
    rospy.init_node("terrain_fixedwing_chain_test")
    rostest.rosrun(
        "xd_uav_planning", "terrain_fixedwing_chain_test",
        TerrainFixedwingChainTest)
