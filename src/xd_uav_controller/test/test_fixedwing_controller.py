#!/usr/bin/env python3

import math
import time
import unittest

import rospy
from nav_msgs.msg import Odometry

from xd_uav_controller.msg import ControlCommand, ControlState
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
            "reference_odometry", Odometry, queue_size=10
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
        reference = Odometry()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = "uav1/odom"
        reference.child_frame_id = "uav1/odom"
        reference.pose.pose.position.x = position_x
        reference.pose.pose.position.y = position_y
        reference.pose.pose.position.z = position_z
        reference.pose.pose.orientation.w = 1.0
        reference.twist.twist.linear.x = velocity_x
        reference.twist.twist.linear.y = velocity_y
        reference.twist.twist.linear.z = velocity_z
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

    def test_reference_and_takeoff(self):
        idle_command = self._wait_for_command(
            lambda value: value.valid,
            include_reference=False,
        )
        self.assertTrue(idle_command.valid)

        command = self._wait_for_command(lambda value: value.valid)
        self.assertEqual(command.vehicle_type, ControlCommand.VEHICLE_FIXEDWING)
        self.assertTrue(math.isfinite(command.body_rate.x))
        self.assertTrue(math.isfinite(command.body_rate.y))
        self.assertTrue(math.isfinite(command.body_rate.z))
        self.assertGreaterEqual(command.thrust, 0.0)
        self.assertLessEqual(command.thrust, 1.0)
        self.assertLess(
            command.body_rate.y,
            -0.05,
            "ROS FLU中正爬升必须产生负pitch rate",
        )

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
                and -1.0 < value.body_rate.x < -0.10
                and value.body_rate.z > 0.01
            ),
            reference=offset_path_reference,
        )
        self.assertLess(offset_path_command.body_rate.x, -0.10)
        self.assertGreater(offset_path_command.body_rate.x, -1.0)
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
        # Reproduce an aircraft that cannot physically hit the
        # configured 50 m waypoint sphere because its turn radius is
        # larger. The dynamically enlarged capture radius must enter
        # the glideslope, and the landing-line guidance must command a
        # right correction from the left side of the final line.
        offtrack_approach_state = self._state()
        offtrack_approach_state.position_odom.x = 40.0
        offtrack_approach_state.position_odom.y = 200.0
        offtrack_approach_state.position_odom.z = 130.0
        land_home_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.controller
                == "fixedwing_course_energy_landing"
                and value.body_rate.x > 0.05
            ),
            include_reference=False,
            state=offtrack_approach_state,
        )
        self.assertTrue(land_home_command.landing_active)
        self.assertGreater(land_home_command.body_rate.x, 0.05)

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

        flare_state = self._state()
        flare_state.position_odom.x = 370.0
        flare_state.position_odom.z = 102.5
        flare_command = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.body_rate.y < -0.02
                and value.thrust < 0.01
            ),
            include_reference=False,
            state=flare_state,
        )
        self.assertLess(
            flare_command.body_rate.y,
            -0.02,
            "拉平阶段应给出抬头指令",
        )
        self.assertLess(flare_command.thrust, 0.01)

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


if __name__ == "__main__":
    rospy.init_node("fixedwing_controller_interface_test")
    import rostest

    rostest.rosrun(
        "xd_uav_controller",
        "fixedwing_controller_interface_test",
        FixedwingControllerInterfaceTest,
    )
