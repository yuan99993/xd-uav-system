#!/usr/bin/env python3

import math
import time
import unittest

import rospy
from geometry_msgs.msg import (
    AccelStamped,
    PoseStamped,
    Transform,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import tf2_ros
from trajectory_msgs.msg import (
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)

from xd_uav_controller.msg import ControlCommand, ControlState
from xd_uav_controller.srv import (
    InternalCommand,
    InternalCommandRequest,
)


class ControllerInterfaceTest(unittest.TestCase):

    def setUp(self):
        self._position_x = 0.0
        self._position_y = 0.0
        self._position_z = 0.0
        self._local_to_odom_x = 10.0
        self._local_alignment_valid = True
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._state_publisher = rospy.Publisher(
            "state", ControlState, queue_size=10
        )
        self._reference_odometry_publisher = rospy.Publisher(
            "reference_odometry", Odometry, queue_size=10
        )
        self._reference_acceleration_publisher = rospy.Publisher(
            "reference_acceleration", AccelStamped, queue_size=10
        )
        self._reference_trajectory_publisher = rospy.Publisher(
            "reference_trajectory",
            MultiDOFJointTrajectory,
            queue_size=2,
        )
        self._simple_goal_publisher = rospy.Publisher(
            "simple_goal", PoseStamped, queue_size=2
        )
        self._local_alignment_valid_publisher = rospy.Publisher(
            "local_alignment_valid",
            Bool,
            queue_size=2,
            latch=True,
        )

    def _state(self):
        state = ControlState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "uav1/odom"
        state.body_frame_id = "uav1/base_link"
        state.vehicle_type = ControlState.VEHICLE_MULTIROTOR
        state.position_odom.x = self._position_x
        state.position_odom.y = self._position_y
        state.position_odom.z = self._position_z
        state.orientation_odom_body.w = 1.0
        state.state_valid = True
        state.localization_valid = True
        state.odometry_fresh = True
        state.imu_fresh = True
        state.acceleration_fresh = True
        state.stable = True
        return state

    @staticmethod
    def _reference():
        reference = Odometry()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = "uav1/odom"
        reference.child_frame_id = "uav1/base_link"
        reference.pose.pose.position.z = 1.0
        reference.pose.pose.orientation.w = 1.0
        return reference

    @staticmethod
    def _acceleration_reference(
        frame_id="uav1/odom"
    ):
        reference = AccelStamped()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = frame_id
        reference.accel.linear.z = 0.1
        return reference

    @staticmethod
    def _trajectory_reference(
        frame_id="uav1/odom"
    ):
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = frame_id
        trajectory.joint_names = ["uav1/base_link"]
        for time_from_start, x, z in (
            (0.0, 2.0, 1.0),
            (1.0, 3.0, 1.0),
        ):
            point = MultiDOFJointTrajectoryPoint()
            transform = Transform()
            transform.translation.x = x
            transform.translation.z = z
            transform.rotation.w = 1.0
            point.transforms = [transform]
            point.velocities = [Twist()]
            point.accelerations = [Twist()]
            point.time_from_start = rospy.Duration(
                time_from_start
            )
            trajectory.points.append(point)
        return trajectory

    @staticmethod
    def _simple_goal(
        frame_id="uav1/odom", position_x=1.0
    ):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = position_x
        goal.pose.position.z = -10.0
        goal.pose.orientation.w = 1.0
        return goal

    def _publish_reference_frames(self):
        self._local_alignment_valid_publisher.publish(
            Bool(data=self._local_alignment_valid)
        )
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "uav1/local_origin"
        transform.child_frame_id = "uav1/odom"
        transform.transform.translation.x = (
            self._local_to_odom_x
        )
        transform.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(transform)

    def _wait_for_command(self, predicate, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self._state_publisher.publish(self._state())
            self._publish_reference_frames()
            try:
                command = rospy.wait_for_message(
                    "command", ControlCommand, timeout=0.2
                )
                if predicate(command):
                    return command
            except rospy.ROSException:
                pass
        self.fail("没有在超时前收到预期控制输出")

    def test_reference_and_takeoff(self):
        idle_command = self._wait_for_command(lambda value: value.valid)
        self.assertTrue(idle_command.valid)

        goal = self._simple_goal()
        connection_deadline = time.time() + 2.0
        while (
            self._simple_goal_publisher.get_num_connections()
            == 0
            and time.time() < connection_deadline
        ):
            self._state_publisher.publish(self._state())
            rospy.sleep(0.02)
        self.assertGreater(
            self._simple_goal_publisher.get_num_connections(),
            0,
        )
        for _ in range(5):
            self._state_publisher.publish(self._state())
            self._simple_goal_publisher.publish(goal)
            rospy.sleep(0.02)
        adapted_goal = rospy.wait_for_message(
            "reference_odometry", Odometry, timeout=1.0
        )
        self.assertAlmostEqual(
            adapted_goal.pose.pose.position.x, 1.0
        )
        self.assertAlmostEqual(
            adapted_goal.pose.pose.position.z, 0.0
        )
        self.assertEqual(
            adapted_goal.child_frame_id, "uav1/base_link"
        )
        goal_command = self._wait_for_command(
            lambda value: (
                value.valid and abs(value.body_rate.y) > 0.01
            )
        )
        self.assertTrue(goal_command.valid)

        # A goal in local_origin is transformed through
        # local_origin -> odom. With T_local_odom.x = 10,
        # local x=9 corresponds to odom x=-1.
        local_goal = self._simple_goal(
            frame_id="uav1/local_origin",
            position_x=9.0,
        )
        for _ in range(5):
            self._state_publisher.publish(self._state())
            self._publish_reference_frames()
            self._simple_goal_publisher.publish(local_goal)
            rospy.sleep(0.02)
        adapted_local_goal = rospy.wait_for_message(
            "reference_odometry", Odometry, timeout=1.0
        )
        self.assertEqual(
            adapted_local_goal.header.frame_id,
            "uav1/local_origin",
        )
        self.assertAlmostEqual(
            adapted_local_goal.pose.pose.position.x, 9.0
        )
        local_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.y
                * goal_command.body_rate.y
                < -1e-4
            )
        )

        # Keep the same raw local goal and update only the
        # alignment. The normalized odom goal must move from
        # x=-1 to x=+1 without publishing a new goal.
        self._local_to_odom_x = 8.0
        updated_alignment_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.y
                * local_command.body_rate.y
                < -1e-4
            )
        )
        self.assertTrue(updated_alignment_command.valid)

        # local/global frames require a valid alignment. A new
        # local goal is rejected while alignment is invalid, and
        # the previous normalized target is held during grace.
        self._local_alignment_valid = False
        unavailable_alignment_goal = self._simple_goal(
            frame_id="uav1/local_origin",
            position_x=30.0,
        )
        for _ in range(5):
            self._state_publisher.publish(self._state())
            self._publish_reference_frames()
            self._simple_goal_publisher.publish(
                unavailable_alignment_goal
            )
            rospy.sleep(0.02)
        held_during_alignment_grace = self._wait_for_command(
            lambda value: value.valid,
            timeout=0.3,
        )
        self.assertTrue(held_during_alignment_grace.valid)
        self._local_alignment_valid = True
        self._wait_for_command(
            lambda value: value.valid,
            timeout=1.0,
        )

        # A cross-UAV frame is not allowed. Rejecting it must
        # keep the last valid local goal instead of invalidating
        # controller output.
        invalid_goal = self._simple_goal(
            frame_id="uav2/local_origin",
            position_x=20.0,
        )
        for _ in range(5):
            self._state_publisher.publish(self._state())
            self._publish_reference_frames()
            self._simple_goal_publisher.publish(invalid_goal)
            rospy.sleep(0.02)
        deadline = time.time() + 0.7
        while time.time() < deadline:
            held_after_rejection = self._wait_for_command(
                lambda value: value.valid,
                timeout=0.5,
            )
        self.assertTrue(held_after_rejection.valid)

        # A simple goal is a one-shot latched target, so it must
        # remain valid beyond the streaming Odometry timeout.
        deadline = time.time() + 0.7
        held_command = goal_command
        while time.time() < deadline:
            held_command = self._wait_for_command(
                lambda value: value.valid,
                timeout=0.5,
            )
        self.assertTrue(held_command.valid)

        deadline = time.time() + 2.0
        while time.time() < deadline:
            self._state_publisher.publish(self._state())
            self._publish_reference_frames()
            self._reference_odometry_publisher.publish(
                self._reference()
            )
            self._reference_acceleration_publisher.publish(
                self._acceleration_reference(
                    frame_id="uav1/local_origin"
                )
            )
            rospy.sleep(0.02)

        command = self._wait_for_command(lambda value: value.valid)
        self.assertEqual(command.vehicle_type, ControlCommand.VEHICLE_MULTIROTOR)
        self.assertTrue(math.isfinite(command.body_rate.x))
        self.assertTrue(math.isfinite(command.body_rate.y))
        self.assertTrue(math.isfinite(command.body_rate.z))
        self.assertGreaterEqual(command.thrust, 0.0)
        self.assertLessEqual(command.thrust, 1.0)

        trajectory = self._trajectory_reference(
            frame_id="uav1/local_origin"
        )
        self._reference_trajectory_publisher.publish(trajectory)
        visualized_trajectory = rospy.wait_for_message(
            "reference_trajectory_path", Path, timeout=1.0
        )
        self.assertEqual(
            visualized_trajectory.header.frame_id,
            "uav1/local_origin",
        )
        self.assertEqual(
            len(visualized_trajectory.poses),
            len(trajectory.points),
        )
        self.assertAlmostEqual(
            visualized_trajectory.poses[0].pose.position.x,
            trajectory.points[0].transforms[0].translation.x,
        )
        trajectory_command = self._wait_for_command(
            lambda value: (
                value.valid and abs(value.body_rate.y) > 0.01
            )
        )
        self.assertTrue(trajectory_command.valid)

        rospy.wait_for_service(
            "controller/internal/command", timeout=3.0
        )
        internal_command = rospy.ServiceProxy(
            "controller/internal/command", InternalCommand
        )
        response = internal_command(
            InternalCommandRequest.TAKEOFF, 1.5
        )
        self.assertTrue(response.success, response.message)
        takeoff_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.takeoff_active
                and value.thrust > 0.55
            )
        )
        self.assertTrue(takeoff_command.takeoff_active)
        self.assertGreater(takeoff_command.thrust, 0.55)

        # Sending a 2D simple goal while the aircraft is still
        # climbing must retain the requested takeoff altitude,
        # not capture the lower instantaneous measured altitude.
        takeoff_simple_goal = self._simple_goal(
            frame_id="uav1/odom",
            position_x=2.0,
        )
        deadline = time.time() + 2.0
        retained_altitude_goal = None
        while time.time() < deadline:
            self._state_publisher.publish(self._state())
            self._publish_reference_frames()
            self._simple_goal_publisher.publish(
                takeoff_simple_goal
            )
            try:
                candidate = rospy.wait_for_message(
                    "reference_odometry",
                    Odometry,
                    timeout=0.2,
                )
                if abs(
                    candidate.pose.pose.position.z - 1.5
                ) < 1e-3:
                    retained_altitude_goal = candidate
                    break
            except rospy.ROSException:
                pass
        self.assertIsNotNone(retained_altitude_goal)
        self.assertAlmostEqual(
            retained_altitude_goal.pose.pose.position.z,
            1.5,
            delta=1e-3,
        )
        self._wait_for_command(
            lambda value: (
                value.valid and not value.takeoff_active
            )
        )

        self._position_z = 1.5
        response = internal_command(
            InternalCommandRequest.LAND, 0.0
        )
        self.assertTrue(response.success, response.message)
        landing_command = self._wait_for_command(
            lambda value: value.valid and value.landing_active
        )
        self.assertTrue(landing_command.landing_active)

        self._position_z = 0.0
        touchdown_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.landing_touchdown
            )
        )
        self.assertTrue(touchdown_command.landing_touchdown)

        response = internal_command(
            InternalCommandRequest.RESET, 0.0
        )
        self.assertTrue(response.success, response.message)

        response = internal_command(
            InternalCommandRequest.TAKEOFF, 1.5
        )
        self.assertTrue(response.success, response.message)
        self._wait_for_command(
            lambda value: value.valid and value.takeoff_active
        )
        self._position_x = 1.0
        self._position_z = 1.5
        response = internal_command(
            InternalCommandRequest.LAND_HOME, 0.0
        )
        self.assertTrue(response.success, response.message)
        home_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and abs(value.body_rate.y) > 0.01
            )
        )
        self.assertTrue(home_command.landing_active)

        # The takeoff home was recorded in local_origin. Updating
        # local_origin -> odom without issuing another land_home
        # request must move the normalized odom target from x=0
        # to x=2. With the vehicle at x=1, the horizontal command
        # therefore changes direction. This verifies that home is
        # kept in its semantic frame and transformed using the
        # latest TF during control.
        self._local_to_odom_x = 6.0
        updated_home_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.body_rate.y
                * home_command.body_rate.y
                < -1e-4
            )
        )
        self.assertTrue(updated_home_command.landing_active)


if __name__ == "__main__":
    rospy.init_node("controller_interface_test")
    import rostest

    rostest.rosrun(
        "xd_uav_controller",
        "controller_interface_test",
        ControllerInterfaceTest,
    )
