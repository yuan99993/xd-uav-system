#!/usr/bin/env python3

import math
import time
import unittest

import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Transform,
    TransformStamped,
    Twist,
)
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Path
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
import tf2_ros
from trajectory_msgs.msg import (
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)

from xd_uav_controller.msg import ControlCommand, ControlState, PathStatus
from xd_uav_controller.srv import (
    InternalCommand,
    InternalCommandRequest,
)


class ControllerInterfaceTest(unittest.TestCase):

    def setUp(self):
        self._position_x = 0.0
        self._position_y = 0.0
        self._position_z = 0.0
        self._velocity_x = 0.0
        self._velocity_y = 0.0
        self._velocity_z = 0.0
        self._acceleration_x = 0.0
        self._acceleration_y = 0.0
        self._acceleration_z = 0.0
        self._acceleration_fresh = True
        self._local_to_odom_x = 10.0
        self._local_alignment_valid = True
        self._distance = 1.0
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._state_publisher = rospy.Publisher(
            "state", ControlState, queue_size=10
        )
        self._distance_sensor_publisher = rospy.Publisher(
            "distance_sensor", Range, queue_size=10
        )
        self._reference_position_target_publisher = rospy.Publisher(
            "reference_position_target",
            PositionTarget,
            queue_size=10,
        )
        self._reference_trajectory_publisher = rospy.Publisher(
            "reference_trajectory",
            MultiDOFJointTrajectory,
            queue_size=2,
        )
        self._reference_path_publisher = rospy.Publisher(
            "reference_path", Path, queue_size=2
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
        state.velocity_odom.x = self._velocity_x
        state.velocity_odom.y = self._velocity_y
        state.velocity_odom.z = self._velocity_z
        state.acceleration_odom.x = self._acceleration_x
        state.acceleration_odom.y = self._acceleration_y
        state.acceleration_odom.z = self._acceleration_z
        state.orientation_odom_body.w = 1.0
        state.state_valid = True
        state.localization_valid = True
        state.odometry_fresh = True
        state.imu_fresh = True
        state.acceleration_fresh = self._acceleration_fresh
        state.stable = True
        return state

    @staticmethod
    def _reference(frame_id="uav1/odom"):
        reference = PositionTarget()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = frame_id
        reference.coordinate_frame = (
            PositionTarget.FRAME_LOCAL_NED
        )
        reference.type_mask = PositionTarget.IGNORE_YAW_RATE
        reference.position.z = 1.0
        reference.acceleration_or_force.z = 0.1
        return reference

    @staticmethod
    def _velocity_reference(frame_id="uav1/odom"):
        reference = PositionTarget()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = frame_id
        reference.coordinate_frame = (
            PositionTarget.FRAME_LOCAL_NED
        )
        reference.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        reference.velocity.x = 1.0
        return reference

    @staticmethod
    def _single_axis_velocity_reference(
        frame_id="uav1/odom"
    ):
        reference = PositionTarget()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = frame_id
        reference.coordinate_frame = (
            PositionTarget.FRAME_LOCAL_NED
        )
        reference.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        reference.position.x = math.nan
        reference.position.y = math.nan
        reference.position.z = math.nan
        reference.velocity.x = 0.3
        reference.velocity.y = math.nan
        reference.velocity.z = math.nan
        reference.acceleration_or_force.x = math.nan
        reference.acceleration_or_force.y = math.nan
        reference.acceleration_or_force.z = math.nan
        reference.yaw = math.nan
        reference.yaw_rate = math.nan
        return reference

    @staticmethod
    def _horizontal_velocity_altitude_reference(
        frame_id="uav1/odom"
    ):
        reference = PositionTarget()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = frame_id
        reference.coordinate_frame = (
            PositionTarget.FRAME_LOCAL_NED
        )
        reference.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        reference.position.x = math.nan
        reference.position.y = math.nan
        reference.position.z = 1.0
        reference.velocity.x = 0.3
        reference.velocity.y = 0.0
        reference.velocity.z = math.nan
        return reference

    @staticmethod
    def _acceleration_reference(frame_id="uav1/odom"):
        reference = PositionTarget()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = frame_id
        reference.coordinate_frame = (
            PositionTarget.FRAME_LOCAL_NED
        )
        reference.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        reference.position.x = math.nan
        reference.position.y = math.nan
        reference.position.z = math.nan
        reference.velocity.x = math.nan
        reference.velocity.y = math.nan
        reference.velocity.z = math.nan
        reference.acceleration_or_force.x = math.nan
        reference.acceleration_or_force.y = 1.0
        reference.acceleration_or_force.z = math.nan
        reference.yaw = math.nan
        reference.yaw_rate = math.nan
        return reference

    @staticmethod
    def _acceleration_vector_reference(
        frame_id="uav1/odom"
    ):
        reference = PositionTarget()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = frame_id
        reference.coordinate_frame = (
            PositionTarget.FRAME_LOCAL_NED
        )
        reference.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
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

    def _publish_distance_sensor(self):
        message = Range()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "uav1/lidarlite_laser"
        message.radiation_type = Range.INFRARED
        message.field_of_view = 0.01
        message.min_range = 0.2
        message.max_range = 15.0
        message.range = self._distance
        self._distance_sensor_publisher.publish(message)

    def _wait_for_command(self, predicate, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self._state_publisher.publish(self._state())
            self._publish_distance_sensor()
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
            "reference_position_target",
            PositionTarget,
            timeout=1.0,
        )
        self.assertAlmostEqual(
            adapted_goal.position.x, 1.0
        )
        self.assertAlmostEqual(
            adapted_goal.position.z, 0.0
        )
        self.assertEqual(
            adapted_goal.coordinate_frame,
            PositionTarget.FRAME_LOCAL_NED,
        )
        self.assertEqual(
            adapted_goal.type_mask,
            (
                PositionTarget.IGNORE_VX
                | PositionTarget.IGNORE_VY
                | PositionTarget.IGNORE_VZ
                | PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
                | PositionTarget.IGNORE_YAW_RATE
            ),
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
            "reference_position_target",
            PositionTarget,
            timeout=1.0,
        )
        self.assertEqual(
            adapted_local_goal.header.frame_id,
            "uav1/local_origin",
        )
        self.assertAlmostEqual(
            adapted_local_goal.position.x, 9.0
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
        # remain valid beyond the streaming PositionTarget
        # timeout.
        deadline = time.time() + 0.7
        held_command = goal_command
        while time.time() < deadline:
            held_command = self._wait_for_command(
                lambda value: value.valid,
                timeout=0.5,
            )
        self.assertTrue(held_command.valid)

        # Per-axis masking combines horizontal velocity with a
        # vertical position hold. It must command both forward
        # tilt and additional thrust toward z=1.
        mixed_reference = (
            self._horizontal_velocity_altitude_reference()
        )
        self.assertEqual(mixed_reference.type_mask, 3555)
        for _ in range(10):
            mixed_reference.header.stamp = rospy.Time.now()
            self._state_publisher.publish(self._state())
            self._reference_position_target_publisher.publish(
                mixed_reference
            )
            rospy.sleep(0.02)
        mixed_command = self._wait_for_command(
            lambda value: (
                value.valid
                and abs(value.body_rate.y) > 0.01
                and value.thrust > 0.705
            ),
            timeout=0.4,
        )
        self.assertTrue(mixed_command.valid)

        deadline = time.time() + 2.0
        while time.time() < deadline:
            self._state_publisher.publish(self._state())
            self._publish_reference_frames()
            self._reference_position_target_publisher.publish(
                self._reference(
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

        # The same topic selects pure velocity and pure
        # acceleration control only by changing type_mask.
        single_axis_velocity_reference = (
            self._single_axis_velocity_reference()
        )
        self.assertEqual(
            single_axis_velocity_reference.type_mask,
            3575,
        )
        for _ in range(10):
            single_axis_velocity_reference.header.stamp = (
                rospy.Time.now()
            )
            self._state_publisher.publish(self._state())
            self._reference_position_target_publisher.publish(
                single_axis_velocity_reference
            )
            rospy.sleep(0.02)
        single_axis_velocity_command = self._wait_for_command(
            lambda value: (
                value.valid and abs(value.body_rate.y) > 0.01
            ),
            timeout=0.4,
        )
        self.assertTrue(single_axis_velocity_command.valid)

        velocity_reference = self._velocity_reference()
        for _ in range(10):
            velocity_reference.header.stamp = rospy.Time.now()
            self._state_publisher.publish(self._state())
            self._reference_position_target_publisher.publish(
                velocity_reference
            )
            rospy.sleep(0.02)
        velocity_command = self._wait_for_command(
            lambda value: (
                value.valid and abs(value.body_rate.y) > 0.01
            ),
            timeout=0.4,
        )
        self.assertTrue(velocity_command.valid)

        acceleration_reference = (
            self._acceleration_reference()
        )
        self.assertEqual(
            acceleration_reference.type_mask,
            3455,
        )
        for _ in range(10):
            acceleration_reference.header.stamp = (
                rospy.Time.now()
            )
            self._state_publisher.publish(self._state())
            self._reference_position_target_publisher.publish(
                acceleration_reference
            )
            rospy.sleep(0.02)
        acceleration_command = self._wait_for_command(
            lambda value: (
                value.valid and abs(value.body_rate.x) > 0.01
            ),
            timeout=0.4,
        )
        self.assertTrue(acceleration_command.valid)

        # Pure acceleration feedback must reject stale measured
        # acceleration, then increase thrust when measured AZ is
        # below a zero-acceleration setpoint.
        acceleration_vector = (
            self._acceleration_vector_reference()
        )
        self.assertEqual(
            acceleration_vector.type_mask,
            3135,
        )
        self._acceleration_fresh = False
        for _ in range(10):
            acceleration_vector.header.stamp = rospy.Time.now()
            self._state_publisher.publish(self._state())
            self._reference_position_target_publisher.publish(
                acceleration_vector
            )
            rospy.sleep(0.02)
        stale_acceleration_command = self._wait_for_command(
            lambda value: (
                not value.valid
                and "纯加速度控制要求" in value.rejection_reason
            ),
            timeout=0.4,
        )
        self.assertFalse(stale_acceleration_command.valid)

        self._acceleration_fresh = True
        self._acceleration_z = -1.0
        for _ in range(15):
            acceleration_vector.header.stamp = rospy.Time.now()
            self._state_publisher.publish(self._state())
            self._reference_position_target_publisher.publish(
                acceleration_vector
            )
            rospy.sleep(0.02)
        acceleration_feedback_command = self._wait_for_command(
            lambda value: (
                value.valid and value.thrust > 0.705
            ),
            timeout=0.4,
        )
        self.assertTrue(acceleration_feedback_command.valid)
        self._acceleration_z = 0.0

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

        geometric_path = Path()
        geometric_path.header.seq = 41
        geometric_path.header.stamp = rospy.Time.now()
        geometric_path.header.frame_id = "uav1/odom"
        for x, z in ((0.0, 0.0), (10.0, 2.0), (20.0, 2.0)):
            pose = PoseStamped()
            pose.header = geometric_path.header
            pose.pose.position.x = x
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            geometric_path.poses.append(pose)
        deadline = time.time() + 2.0
        path_status = None
        while time.time() < deadline:
            self._reference_path_publisher.publish(geometric_path)
            try:
                candidate = rospy.wait_for_message(
                    "path_status", PathStatus, timeout=0.2
                )
                if candidate.path_id == 41:
                    path_status = candidate
                    break
            except rospy.ROSException:
                pass
        self.assertIsNotNone(path_status)
        self.assertEqual(path_status.path_id, 41)
        self.assertIn(
            path_status.state,
            (PathStatus.ACCEPTED, PathStatus.ACTIVE),
        )
        path_command = self._wait_for_command(
            lambda value: value.valid and value.controller.startswith("path_")
        )
        self.assertTrue(path_command.valid)

        # Path order, rather than globally nearest geometry, determines
        # progress. The vehicle is closest to a later branch here but must
        # still start from segment zero.
        ordered_path = Path()
        ordered_path.header.seq = 42
        ordered_path.header.stamp = rospy.Time.now()
        ordered_path.header.frame_id = "uav1/odom"
        for x, y in ((50.0, 0.0), (60.0, 0.0), (0.0, 0.0), (0.0, 10.0)):
            pose = PoseStamped()
            pose.header = ordered_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            ordered_path.poses.append(pose)
        self._reference_path_publisher.publish(ordered_path)
        ordered_status = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            self._state_publisher.publish(self._state())
            self._publish_reference_frames()
            try:
                candidate = rospy.wait_for_message(
                    "path_status", PathStatus, timeout=0.2
                )
                if (
                    candidate.path_id == 42
                    and candidate.state == PathStatus.REACQUIRING
                ):
                    ordered_status = candidate
                    break
            except rospy.ROSException:
                pass
        self.assertIsNotNone(ordered_status)
        self.assertEqual(ordered_status.current_segment, 0)
        self.assertGreater(ordered_status.cross_track_error, 40.0)

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
                    "reference_position_target",
                    PositionTarget,
                    timeout=0.2,
                )
                if abs(
                    candidate.position.z - 1.5
                ) < 1e-3:
                    retained_altitude_goal = candidate
                    break
            except rospy.ROSException:
                pass
        self.assertIsNotNone(retained_altitude_goal)
        self.assertAlmostEqual(
            retained_altitude_goal.position.z,
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

        response = internal_command(
            InternalCommandRequest.CANCEL_LANDING, 0.0
        )
        self.assertTrue(response.success, response.message)
        cancelled_command = self._wait_for_command(
            lambda value: (
                value.valid
                and not value.landing_active
                and value.controller == "finite_horizon_mpc_so3"
            )
        )
        self.assertFalse(cancelled_command.landing_active)

        response = internal_command(
            InternalCommandRequest.LAND, 0.0
        )
        self.assertTrue(response.success, response.message)
        self._wait_for_command(
            lambda value: value.valid and value.landing_active
        )

        # The odometry altitude deliberately remains two metres above the
        # takeoff ground.  Touchdown must come from the downward rangefinder,
        # which represents landing on an elevated platform.  odom.vz is also
        # deliberately impossible: distance_sensor mode must use the filtered
        # AGL rate and must not inherit vertical velocity from the selected
        # localization source.
        self._position_z = 2.0
        self._velocity_z = 5.0
        self._distance = 0.2
        touchdown_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.landing_touchdown
            )
        )
        self.assertTrue(touchdown_command.landing_touchdown)
        self._velocity_z = 0.0

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
