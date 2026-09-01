#!/usr/bin/env python3

import math
import time
import unittest

import rospy
from geometry_msgs.msg import PoseStamped, Transform, Twist
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Path
from trajectory_msgs.msg import (
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)

from xd_uav_controller.msg import ControlCommand, ControlState, PathStatus
from xd_uav_controller.srv import (
    InternalCommand,
    InternalCommandRequest,
)


class FixedwingControllerInterfaceTest(unittest.TestCase):

    def setUp(self):
        self._state_publisher = rospy.Publisher(
            "state", ControlState, queue_size=10
        )
        self._reference_publisher = rospy.Publisher(
            "reference_position_target",
            PositionTarget,
            queue_size=10,
        )
        self._trajectory_publisher = rospy.Publisher(
            "reference_trajectory",
            MultiDOFJointTrajectory,
            queue_size=2,
        )
        self._path_publisher = rospy.Publisher(
            "reference_path", Path, queue_size=2
        )

    @staticmethod
    def _state():
        state = ControlState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "uav1/odom"
        state.body_frame_id = "uav1/base_link"
        state.vehicle_type = ControlState.VEHICLE_FIXEDWING
        state.position_odom.z = 100.0
        state.velocity_odom.x = 15.0
        state.orientation_odom_body.w = 1.0
        state.airspeed = 15.0
        state.groundspeed = 15.0
        state.course = 0.0
        state.state_valid = True
        state.localization_valid = True
        state.odometry_fresh = True
        state.imu_fresh = True
        state.acceleration_fresh = True
        state.airspeed_valid = True
        state.stable = True
        return state

    @staticmethod
    def _reference(
        position_x=200.0,
        position_y=0.0,
        position_z=110.0,
        velocity_x=15.0,
        velocity_y=0.0,
        velocity_z=1.0,
    ):
        reference = PositionTarget()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = "uav1/odom"
        reference.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        reference.type_mask = (
            PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        reference.position.x = position_x
        reference.position.y = position_y
        reference.position.z = position_z
        reference.velocity.x = velocity_x
        reference.velocity.y = velocity_y
        reference.velocity.z = velocity_z
        reference.yaw = 0.0
        return reference

    def _publish(
        self,
        include_reference=True,
        state=None,
        reference=None,
    ):
        self._state_publisher.publish(
            self._state() if state is None else state
        )
        if include_reference:
            self._reference_publisher.publish(
                self._reference() if reference is None else reference
            )

    def _wait_for_command(
        self,
        predicate,
        include_reference=True,
        timeout=5.0,
        state=None,
        reference=None,
    ):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self._publish(
                include_reference,
                state=state,
                reference=reference,
            )
            try:
                command = rospy.wait_for_message(
                    "command", ControlCommand, timeout=0.2
                )
                if predicate(command):
                    return command
            except rospy.ROSException:
                pass
        self.fail("没有在超时前收到固定翼控制输出")

    def _wait_for_trajectory_command(
        self,
        trajectory,
        predicate,
        timeout=5.0,
        state=None,
    ):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self._state_publisher.publish(
                self._state() if state is None else state
            )
            self._trajectory_publisher.publish(trajectory)
            try:
                command = rospy.wait_for_message(
                    "command", ControlCommand, timeout=0.2
                )
                if predicate(command):
                    return command
            except rospy.ROSException:
                pass
        self.fail("没有在超时前收到固定翼轨迹控制输出")

    @staticmethod
    def _turning_trajectory(
        yaw_rate=0.15,
        vertical_acceleration=0.0,
        duration=10.0,
    ):
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id = "uav1/odom"
        trajectory.joint_names = ["uav1/base_link"]
        for point_time in (0.0, duration):
            transform = Transform()
            transform.translation.z = 100.0
            transform.rotation.w = 1.0

            velocity = Twist()
            velocity.linear.x = 15.0
            velocity.angular.z = yaw_rate

            acceleration = Twist()
            acceleration.linear.y = 2.25
            acceleration.linear.z = vertical_acceleration

            point = MultiDOFJointTrajectoryPoint()
            point.transforms = [transform]
            point.velocities = [velocity]
            point.accelerations = [acceleration]
            point.time_from_start = rospy.Duration(point_time)
            trajectory.points.append(point)
        return trajectory

    def test_reference_and_takeoff(self):
        idle_command = self._wait_for_command(
            lambda value: value.valid,
            include_reference=False,
        )
        self.assertTrue(idle_command.valid)

        # Turn-energy feed-forward must raise throttle before measured
        # airspeed changes. Both cases use the same state and speed target;
        # only the requested course rate changes.
        straight_reference = self._reference(
            position_z=100.0,
            velocity_z=0.0,
        )
        straight_reference.type_mask &= ~PositionTarget.IGNORE_YAW_RATE
        straight_reference.yaw_rate = 0.0
        straight_command = self._wait_for_command(
            lambda value: value.valid and abs(value.body_rate.x) < 0.05,
            reference=straight_reference,
        )
        energetic_turn_reference = self._reference(
            position_z=100.0,
            velocity_z=0.0,
        )
        energetic_turn_reference.type_mask &= ~PositionTarget.IGNORE_YAW_RATE
        energetic_turn_reference.yaw_rate = 0.6
        energetic_turn_command = self._wait_for_command(
            lambda value: value.valid and value.body_rate.x < -0.5,
            reference=energetic_turn_reference,
        )
        self.assertGreater(
            energetic_turn_command.thrust,
            straight_command.thrust + 0.04,
            "转弯载荷必须在实测空速下降前产生油门前馈",
        )
        self.assertLess(
            energetic_turn_command.body_rate.y,
            straight_command.body_rate.y - 0.02,
            "转弯载荷必须在实际掉高前产生抬头前馈",
        )

        # Below minimum airspeed the normal altitude demand must yield to
        # recovery: full throttle, unloaded bank and a nose-down pitch rate
        # in this repository's ROS FLU convention.
        underspeed_state = self._state()
        underspeed_state.airspeed = 10.5
        underspeed_reference = self._reference(
            position_z=120.0,
            velocity_z=2.0,
        )
        underspeed_reference.type_mask &= ~PositionTarget.IGNORE_YAW_RATE
        underspeed_reference.yaw_rate = 0.6
        underspeed_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.thrust > 0.99
                and value.body_rate.y > 0.05
            ),
            state=underspeed_state,
            reference=underspeed_reference,
        )
        self.assertLessEqual(abs(underspeed_command.body_rate.x), 1.80)
        self.assertGreater(underspeed_command.body_rate.y, 0.05)
        self.assertGreater(underspeed_command.thrust, 0.99)

        # At a perfect circle tangent the position and course errors
        # are both zero. The trajectory yaw-rate feed-forward must
        # still establish the left bank needed for a CCW turn.
        trajectory_command = self._wait_for_trajectory_command(
            self._turning_trajectory(),
            lambda value: (
                value.valid
                and value.body_rate.x < -0.05
                and value.body_rate.z > 0.01
            ),
        )
        self.assertLess(trajectory_command.body_rate.x, -0.05)
        self.assertGreater(trajectory_command.body_rate.z, 0.01)

        # If angular.z is left at zero, the same turn feed-forward is
        # recovered from horizontal velocity and acceleration.
        curvature_command = self._wait_for_trajectory_command(
            self._turning_trajectory(yaw_rate=0.0),
            lambda value: (
                value.valid
                and value.body_rate.x < -0.05
                and value.body_rate.z > 0.01
            ),
        )
        self.assertLess(curvature_command.body_rate.x, -0.05)
        self.assertGreater(curvature_command.body_rate.z, 0.01)

        vertical_feedforward_command = self._wait_for_trajectory_command(
            self._turning_trajectory(vertical_acceleration=2.0),
            lambda value: (
                value.valid and value.body_rate.y < -0.05
            ),
        )
        self.assertLess(
            vertical_feedforward_command.body_rate.y,
            -0.05,
            "正向垂直加速度前馈必须提前产生抬头角速度",
        )

        # The same P/V/A reference must produce the same fixed-wing command
        # whether it arrives as a sampled trajectory or a masked
        # PositionTarget. This guards the canonical reference adapter: the
        # core controller must not implement a second set of equations for
        # the topic interface.
        unified_trajectory_command = self._wait_for_trajectory_command(
            self._turning_trajectory(
                yaw_rate=0.0,
                vertical_acceleration=2.0,
            ),
            lambda value: value.valid,
        )
        pva_reference = self._reference(
            position_x=0.0,
            position_y=0.0,
            position_z=100.0,
            velocity_x=15.0,
            velocity_y=0.0,
            velocity_z=0.0,
        )
        pva_reference.type_mask = PositionTarget.IGNORE_YAW_RATE
        pva_reference.acceleration_or_force.x = 0.0
        pva_reference.acceleration_or_force.y = 2.25
        pva_reference.acceleration_or_force.z = 2.0
        unified_setpoint_command = self._wait_for_command(
            lambda value: value.valid,
            reference=pva_reference,
        )
        self.assertAlmostEqual(
            unified_setpoint_command.body_rate.x,
            unified_trajectory_command.body_rate.x,
            delta=0.03,
        )
        self.assertAlmostEqual(
            unified_setpoint_command.body_rate.y,
            unified_trajectory_command.body_rate.y,
            delta=0.03,
        )
        self.assertAlmostEqual(
            unified_setpoint_command.body_rate.z,
            unified_trajectory_command.body_rate.z,
            delta=0.03,
        )
        self.assertAlmostEqual(
            unified_setpoint_command.thrust,
            unified_trajectory_command.thrust,
            delta=0.01,
        )

        # A completed fixed-wing trajectory must not keep chasing its
        # final static point. It must create a tangent waiting circle,
        # whose V/R feed-forward already commands bank at circle entry.
        short_trajectory_command = self._wait_for_trajectory_command(
            self._turning_trajectory(duration=0.15),
            lambda value: (
                value.valid
                and value.controller == "fixedwing_course_energy"
            ),
        )
        self.assertTrue(short_trajectory_command.valid)
        completed_trajectory_loiter = self._wait_for_command(
            lambda value: (
                value.valid
                and value.controller
                == "fixedwing_course_energy_loiter"
                and value.body_rate.x < -0.01
                and value.body_rate.z > 0.01
            ),
            include_reference=False,
            timeout=2.0,
        )
        self.assertLess(completed_trajectory_loiter.body_rate.x, 0.0)
        self.assertGreater(completed_trajectory_loiter.body_rate.z, 0.0)

        geometric_path = Path()
        # rospy rewrites this top-level sequence number.  The allocator keeps
        # its stable path ID in an independent nested pose Header instead.
        geometric_path.header.seq = 999
        geometric_path.header.stamp = rospy.Time.now()
        geometric_path.header.frame_id = "uav1/odom"
        for x, y in ((0.0, 0.0), (30.0, 0.0), (30.0, 30.0)):
            pose = PoseStamped()
            pose.header.seq = 52
            pose.header.stamp = geometric_path.header.stamp
            pose.header.frame_id = geometric_path.header.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 100.0
            pose.pose.orientation.w = 1.0
            geometric_path.poses.append(pose)
        deadline = time.time() + 2.0
        path_status = None
        while time.time() < deadline:
            self._path_publisher.publish(geometric_path)
            try:
                candidate = rospy.wait_for_message(
                    "path_status", PathStatus, timeout=0.2
                )
                if candidate.path_id == 52:
                    path_status = candidate
                    break
            except rospy.ROSException:
                pass
        self.assertIsNotNone(path_status)
        self.assertEqual(path_status.path_id, 52)
        self.assertIn(
            path_status.state,
            (PathStatus.ACCEPTED, PathStatus.ACTIVE),
        )
        straight_path_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.controller == "fixedwing_path_course_energy"
                and abs(value.body_rate.x) < 0.05
            ),
            include_reference=False,
        )
        self.assertTrue(straight_path_command.valid)
        self.assertAlmostEqual(
            straight_path_command.body_rate.x,
            0.0,
            delta=0.05,
            msg="远处弯道不能把固定翼提前拉离当前直线",
        )

        near_turn_state = self._state()
        near_turn_state.position_odom.x = 24.0
        turn_entry_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.controller == "fixedwing_path_course_energy"
                and value.body_rate.x < -0.05
            ),
            include_reference=False,
            state=near_turn_state,
        )
        self.assertLess(
            turn_entry_command.body_rate.x,
            -0.05,
            "接近弯道时曲率短预判必须提前建立滚转",
        )

        command = self._wait_for_command(
            lambda value: (
                value.valid and value.body_rate.y < -0.05
            )
        )
        self.assertEqual(command.vehicle_type, ControlCommand.VEHICLE_FIXEDWING)
        self.assertTrue(math.isfinite(command.body_rate.x))
        self.assertTrue(math.isfinite(command.body_rate.y))
        self.assertTrue(math.isfinite(command.body_rate.z))
        self.assertGreaterEqual(command.thrust, 0.0)
        self.assertLessEqual(command.thrust, 1.0)
        self.assertGreater(
            command.thrust,
            0.25,
            "爬升率目标必须在配平油门上增加前馈",
        )
        self.assertLess(
            command.body_rate.y,
            -0.05,
            "ROS FLU中正爬升必须产生负pitch rate",
        )

        # With no altitude error or vertical feed-forward, an actual upward
        # velocity still needs a nose-down correction. This exercises the
        # climb-rate feedback that damps altitude overshoot.
        climbing_state = self._state()
        climbing_state.velocity_odom.z = 2.0
        level_reference = self._reference(
            position_z=100.0,
            velocity_z=0.0,
        )
        climb_damping_command = self._wait_for_command(
            lambda value: value.valid and value.body_rate.y > 0.05,
            state=climbing_state,
            reference=level_reference,
        )
        self.assertGreater(climb_damping_command.body_rate.y, 0.05)

        left_turn_reference = self._reference(
            position_x=0.0,
            position_y=200.0,
            position_z=100.0,
            velocity_x=0.0,
            velocity_y=15.0,
            velocity_z=0.0,
        )
        left_turn_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.x < -0.05
                and value.body_rate.z > 0.01
            ),
            reference=left_turn_reference,
        )
        self.assertLess(left_turn_command.body_rate.x, 0.0)
        self.assertGreater(left_turn_command.body_rate.z, 0.0)

        # Fixed-wing uses the same per-axis PositionTarget mask as
        # multirotor. A VY-only command must not require dummy VX/VZ
        # fields and must command a left turn.
        velocity_y_only = self._reference(
            position_x=float("nan"),
            position_y=float("nan"),
            position_z=float("nan"),
            velocity_x=float("nan"),
            velocity_y=15.0,
            velocity_z=float("nan"),
        )
        velocity_y_only.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        velocity_y_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.x < -0.05
                and value.body_rate.z > 0.01
            ),
            reference=velocity_y_only,
        )
        self.assertLess(velocity_y_command.body_rate.x, 0.0)
        self.assertGreater(velocity_y_command.body_rate.z, 0.0)

        # A PositionTarget velocity stream does not carry acceleration,
        # but the changing horizontal direction still contains the path
        # turn rate. Keep measured course aligned with the vector so the
        # proportional course error is nearly zero: the bank below must
        # therefore come from the recovered stream feed-forward.
        turning_stream_command = None
        stream_start = time.time()
        stream_yaw_rate = 0.18
        while (
            time.time() - stream_start < 1.5
            and not rospy.is_shutdown()
        ):
            elapsed = time.time() - stream_start
            course = stream_yaw_rate * elapsed
            turning_state = self._state()
            turning_state.course = course
            turning_reference = self._reference(
                position_x=float("nan"),
                position_y=float("nan"),
                position_z=100.0,
                velocity_x=15.0 * math.cos(course),
                velocity_y=15.0 * math.sin(course),
                velocity_z=float("nan"),
            )
            turning_reference.type_mask = (
                PositionTarget.IGNORE_PX
                | PositionTarget.IGNORE_PY
                | PositionTarget.IGNORE_VZ
                | PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
                | PositionTarget.IGNORE_YAW
                | PositionTarget.IGNORE_YAW_RATE
            )
            self._publish(
                state=turning_state,
                reference=turning_reference,
            )
            try:
                candidate = rospy.wait_for_message(
                    "command", ControlCommand, timeout=0.12
                )
                if elapsed > 0.8 and candidate.valid:
                    turning_stream_command = candidate
            except rospy.ROSException:
                pass
            rospy.sleep(0.02)
        self.assertIsNotNone(
            turning_stream_command,
            "没有收到流式速度方向转弯控制输出",
        )
        self.assertLess(
            turning_stream_command.body_rate.x,
            -0.05,
            "流式速度方向变化应产生左转course-rate前馈",
        )
        self.assertGreater(turning_stream_command.body_rate.z, 0.01)

        # A vertical-only setpoint has no horizontal direction. It must
        # hold the course captured on mode entry; re-sampling the measured
        # course on every iteration would make this correction disappear.
        altitude_only_reference = self._reference(
            position_x=float("nan"),
            position_y=float("nan"),
            position_z=100.0,
            velocity_x=float("nan"),
            velocity_y=float("nan"),
            velocity_z=float("nan"),
        )
        altitude_only_reference.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        self._state_publisher.publish(self._state())
        rospy.sleep(0.10)
        self._wait_for_command(
            lambda value: value.valid,
            reference=altitude_only_reference,
        )
        drifted_course_state = self._state()
        drifted_course_state.course = 0.15
        held_course_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.x > 0.05
                and value.body_rate.z < -0.01
            ),
            state=drifted_course_state,
            reference=altitude_only_reference,
        )
        self.assertGreater(held_course_command.body_rate.x, 0.05)
        self.assertLess(held_course_command.body_rate.z, -0.01)

        # Pure PXY, pure VZ and explicit yaw-rate are separate valid fixed-
        # wing PositionTarget modes, not special cases requiring dummy axes.
        position_xy_only = self._reference(
            position_x=0.0,
            position_y=200.0,
            position_z=float("nan"),
            velocity_x=float("nan"),
            velocity_y=float("nan"),
            velocity_z=float("nan"),
        )
        position_xy_only.type_mask = (
            PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        position_only_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.x < -0.05
                and value.body_rate.z > 0.01
            ),
            reference=position_xy_only,
        )
        self.assertLess(position_only_command.body_rate.x, -0.05)

        velocity_z_only = self._reference(
            position_x=float("nan"),
            position_y=float("nan"),
            position_z=float("nan"),
            velocity_x=float("nan"),
            velocity_y=float("nan"),
            velocity_z=1.0,
        )
        velocity_z_only.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        vertical_velocity_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.y < -0.05
                and value.thrust > 0.25
            ),
            reference=velocity_z_only,
        )
        self.assertLess(vertical_velocity_command.body_rate.y, -0.05)
        self.assertGreater(vertical_velocity_command.thrust, 0.25)

        explicit_rate_reference = altitude_only_reference
        explicit_rate_reference.type_mask &= (
            ~PositionTarget.IGNORE_YAW_RATE
        )
        explicit_rate_reference.yaw_rate = 0.18
        explicit_rate_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.x < -0.05
                and value.body_rate.z > 0.01
            ),
            reference=explicit_rate_reference,
        )
        self.assertLess(explicit_rate_command.body_rate.x, -0.05)
        self.assertGreater(explicit_rate_command.body_rate.z, 0.01)

        # A trajectory tangent pointing east must still steer left
        # when the sampled path position lies north of the aircraft.
        # This verifies that velocity feed-forward no longer
        # overwrites horizontal position correction.
        offset_path_reference = self._reference(
            position_x=0.0,
            position_y=30.0,
            position_z=100.0,
            velocity_x=15.0,
            velocity_y=0.0,
            velocity_z=0.0,
        )
        offset_path_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.body_rate.x < -0.10
                and value.body_rate.z > 0.01
            ),
            reference=offset_path_reference,
        )
        self.assertLess(offset_path_command.body_rate.x, -0.10)
        # fixedwing.yaml currently permits up to 2.5 rad/s roll rate.
        self.assertGreaterEqual(offset_path_command.body_rate.x, -2.5)
        self.assertGreater(offset_path_command.body_rate.z, 0.0)

        rospy.wait_for_service(
            "controller/internal/command", timeout=3.0
        )
        internal_command = rospy.ServiceProxy(
            "controller/internal/command", InternalCommand
        )
        response = internal_command(
            InternalCommandRequest.TAKEOFF, 30.0
        )
        self.assertTrue(response.success, response.message)
        takeoff_command = self._wait_for_command(
            lambda value: value.valid and value.takeoff_active,
            include_reference=False,
        )
        self.assertGreater(takeoff_command.thrust, 0.8)
        self.assertLess(
            takeoff_command.body_rate.y,
            -0.05,
            "达到rotate空速后应给出抬头指令",
        )

        climbed_state = self._state()
        climbed_state.position_odom.z = 128.0
        loiter_command = self._wait_for_command(
            lambda value: (
                value.valid
                and not value.takeoff_active
                and value.controller
                == "fixedwing_course_energy_loiter"
            ),
            include_reference=False,
            state=climbed_state,
        )
        self.assertEqual(
            loiter_command.controller,
            "fixedwing_course_energy_loiter",
        )
        self.assertLess(
            loiter_command.body_rate.x,
            -0.01,
            "圆周切入点应由V/R前馈立即建立滚转",
        )
        self.assertGreater(loiter_command.body_rate.z, 0.01)

        # The configured CCW circle is tangent to course=0 at entry.
        # Moving outside that circle must ask for a left bank in ROS
        # FLU (negative roll rate) and a positive yaw rate.
        outside_loiter_state = self._state()
        outside_loiter_state.position_odom.x = 10.0
        outside_loiter_state.position_odom.z = 130.0
        corrected_loiter_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.controller
                == "fixedwing_course_energy_loiter"
                and value.body_rate.x < -0.01
                and value.body_rate.z > 0.01
            ),
            include_reference=False,
            state=outside_loiter_state,
        )
        self.assertLess(corrected_loiter_command.body_rate.x, 0.0)
        self.assertGreater(corrected_loiter_command.body_rate.z, 0.0)

        # A streamed external target takes over loiter. If the stream
        # disappears, fixed-wing control remains valid and creates a
        # new tangent waiting circle instead of dropping its output.
        external_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.controller == "fixedwing_course_energy"
            ),
            state=outside_loiter_state,
        )
        self.assertTrue(external_command.valid)
        timeout_loiter_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.controller
                == "fixedwing_course_energy_loiter"
            ),
            include_reference=False,
            state=outside_loiter_state,
            timeout=3.0,
        )
        self.assertTrue(timeout_loiter_command.valid)

        # land_home must accept the fixed Home supplied by the common
        # home configuration and switch to the fixed-wing landing
        # controller without dropping the body-rate/throttle output.
        land_home_response = internal_command(
            InternalCommandRequest.LAND_HOME, 0.0
        )
        self.assertTrue(
            land_home_response.success,
            land_home_response.message,
        )
        # Reproduce the old handoff transient: the aircraft enters the
        # enlarged capture radius while already banked in the direct-to-
        # point turn. Line capture must preserve that established bank and
        # hold approach altitude instead of immediately unloading the turn
        # as the glide-slope reference replaces the point reference.
        offtrack_approach_state = self._state()
        offtrack_approach_state.position_odom.x = 100.0
        offtrack_approach_state.position_odom.y = 0.0
        offtrack_approach_state.position_odom.z = 130.0
        offtrack_approach_state.course = 0.60
        established_roll = -0.35
        offtrack_approach_state.orientation_odom_body.x = math.sin(
            0.5 * established_roll
        )
        offtrack_approach_state.orientation_odom_body.w = math.cos(
            0.5 * established_roll
        )
        line_capture_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.controller
                == "fixedwing_course_energy_landing"
                and abs(value.body_rate.x) < 0.10
                and abs(value.body_rate.y) < 0.02
                and value.body_rate.z > 0.10
            ),
            include_reference=False,
            state=offtrack_approach_state,
        )
        self.assertTrue(line_capture_command.landing_active)
        self.assertAlmostEqual(
            line_capture_command.body_rate.x, 0.0, delta=0.10
        )
        self.assertAlmostEqual(
            line_capture_command.body_rate.y, 0.0, delta=0.02
        )
        self.assertGreater(line_capture_command.body_rate.z, 0.10)

        cancel_land_response = internal_command(
            InternalCommandRequest.CANCEL_LANDING, 0.0
        )
        self.assertTrue(
            cancel_land_response.success,
            cancel_land_response.message,
        )
        cancel_loiter_command = self._wait_for_command(
            lambda value: (
                value.valid
                and not value.landing_active
                and value.controller
                == "fixedwing_course_energy_loiter"
            ),
            include_reference=False,
            state=offtrack_approach_state,
        )
        self.assertFalse(cancel_loiter_command.landing_active)

        restart_land_response = internal_command(
            InternalCommandRequest.LAND_HOME, 0.0
        )
        self.assertTrue(
            restart_land_response.success,
            restart_land_response.message,
        )
        self._wait_for_command(
            lambda value: value.valid and value.landing_active,
            include_reference=False,
            state=offtrack_approach_state,
        )

        aligned_approach_state = self._state()
        aligned_approach_state.position_odom.x = 100.0
        aligned_approach_state.position_odom.y = 0.0
        aligned_approach_state.position_odom.z = 130.0
        aligned_approach_state.course = 0.55
        aligned_approach_state.orientation_odom_body.x = math.sin(
            0.5 * established_roll
        )
        aligned_approach_state.orientation_odom_body.w = math.cos(
            0.5 * established_roll
        )
        aligned_glide_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.body_rate.y > 0.02
                and abs(value.body_rate.x) < 0.20
                and value.body_rate.z > 0.10
            ),
            include_reference=False,
            state=aligned_approach_state,
        )
        self.assertLess(abs(aligned_glide_command.body_rate.x), 0.20)
        self.assertGreater(aligned_glide_command.body_rate.y, 0.02)

        reset_response = internal_command(
            InternalCommandRequest.RESET, 0.0
        )
        self.assertTrue(reset_response.success, reset_response.message)

        # Establish a fresh takeoff origin for the local land test.
        base_state = self._state()
        self._wait_for_command(
            lambda value: value.valid,
            include_reference=False,
            state=base_state,
        )
        second_takeoff_response = internal_command(
            InternalCommandRequest.TAKEOFF, 30.0
        )
        self.assertTrue(
            second_takeoff_response.success,
            second_takeoff_response.message,
        )
        self._wait_for_command(
            lambda value: value.valid and value.takeoff_active,
            include_reference=False,
            state=base_state,
        )
        self._wait_for_command(
            lambda value: (
                value.valid
                and not value.takeoff_active
                and value.controller
                == "fixedwing_course_energy_loiter"
            ),
            include_reference=False,
            state=climbed_state,
        )

        # At 30 m AGL, local land creates a touchdown point at least
        # 400 m ahead. With course=0 the aircraft is already at the
        # generated approach point, so it should enter the glideslope.
        land_response = internal_command(
            InternalCommandRequest.LAND, 0.0
        )
        self.assertTrue(land_response.success, land_response.message)
        glide_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.controller
                == "fixedwing_course_energy_landing"
                and value.body_rate.y > 0.02
            ),
            include_reference=False,
            state=outside_loiter_state,
        )
        self.assertGreater(
            glide_command.body_rate.y,
            0.02,
            "下滑阶段应给出低头指令",
        )

        near_ground_airborne_state = self._state()
        near_ground_airborne_state.position_odom.x = 370.0
        near_ground_airborne_state.position_odom.z = 102.5
        near_ground_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and not value.landing_touchdown
                and value.thrust > 0.05
            ),
            include_reference=False,
            state=near_ground_airborne_state,
        )
        self.assertFalse(near_ground_command.landing_touchdown)
        self.assertGreater(
            near_ground_command.thrust,
            0.05,
            "仍在空中时不能因固定距离而提前断油",
        )
        self.assertGreater(
            near_ground_command.body_rate.y,
            0.05,
            "低空阶段必须继续跟踪下滑率，不能配平为平飞",
        )

        low_airborne_state = self._state()
        low_airborne_state.position_odom.x = 399.0
        low_airborne_state.position_odom.z = 100.5
        low_airborne_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and not value.landing_touchdown
                and 0.01 < value.thrust
                < near_ground_command.thrust
            ),
            include_reference=False,
            state=low_airborne_state,
        )
        self.assertGreater(
            low_airborne_command.thrust,
            0.01,
            "最后一米内仍高速飞行时油门应连续衰减而非归零",
        )
        self.assertLess(
            low_airborne_command.thrust,
            near_ground_command.thrust,
        )
        self.assertGreater(
            low_airborne_command.body_rate.y,
            0.0,
            "接地前应保留轻微下沉而不是悬在跑道上方",
        )

        # Once rollout is active, the same lateral guidance error must
        # receive progressively less airborne body-rate control as ground
        # speed falls. The blend reuses approach airspeed, so halving the
        # groundspeed should approximately halve the roll/yaw commands.
        fast_rollout_state = self._state()
        fast_rollout_state.position_odom.x = 399.0
        fast_rollout_state.position_odom.y = 50.0
        fast_rollout_state.position_odom.z = 100.5
        fast_rollout_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and not value.landing_touchdown
                and abs(value.body_rate.x) > 0.20
                and abs(value.body_rate.z) > 0.02
            ),
            include_reference=False,
            state=fast_rollout_state,
        )

        slow_rollout_state = self._state()
        slow_rollout_state.position_odom.x = 399.0
        slow_rollout_state.position_odom.y = 50.0
        slow_rollout_state.position_odom.z = 100.5
        slow_rollout_state.velocity_odom.x = 7.5
        slow_rollout_state.groundspeed = 7.5
        slow_rollout_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and not value.landing_touchdown
                and abs(value.body_rate.x)
                < 0.75 * abs(fast_rollout_command.body_rate.x)
            ),
            include_reference=False,
            state=slow_rollout_state,
        )
        self.assertAlmostEqual(
            slow_rollout_command.body_rate.x,
            0.5 * fast_rollout_command.body_rate.x,
            # The final-line course target continues slewing while the
            # groundspeed changes, so compare the intended attenuation
            # without assuming an identical unscaled roll error.
            delta=0.06,
        )
        self.assertAlmostEqual(
            slow_rollout_command.body_rate.z,
            0.5 * fast_rollout_command.body_rate.z,
            delta=0.02,
        )

        rollout_state = self._state()
        rollout_state.position_odom.x = 409.0
        rollout_state.position_odom.z = 100.5
        rollout_state.velocity_odom.x = 1.0
        rollout_state.groundspeed = 1.0
        touchdown_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.landing_touchdown
                and value.thrust < 0.01
            ),
            include_reference=False,
            state=rollout_state,
        )
        self.assertTrue(touchdown_command.landing_touchdown)
        self.assertLess(touchdown_command.thrust, 0.01)
        self.assertAlmostEqual(touchdown_command.body_rate.x, 0.0)
        self.assertAlmostEqual(touchdown_command.body_rate.y, 0.0)
        self.assertAlmostEqual(touchdown_command.body_rate.z, 0.0)


if __name__ == "__main__":
    rospy.init_node("fixedwing_controller_interface_test")
    import rostest

    rostest.rosrun(
        "xd_uav_controller",
        "fixedwing_controller_interface_test",
        FixedwingControllerInterfaceTest,
    )
