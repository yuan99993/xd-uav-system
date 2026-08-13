#!/usr/bin/env python3

import time
import unittest

import rospy

from xd_uav_controller.msg import ControlCommand, ControlState
from xd_uav_controller.srv import (
    InternalCommand,
    InternalCommandRequest,
)


class OdomLandingTest(unittest.TestCase):

    def setUp(self):
        self._x = 0.0
        self._z = 0.0
        self._vx = 0.0
        self._vz = 0.0
        self._state_publisher = rospy.Publisher(
            "state", ControlState, queue_size=10
        )

    def _publish_state(self):
        state = ControlState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "uav1/odom"
        state.body_frame_id = "uav1/base_link"
        state.vehicle_type = ControlState.VEHICLE_MULTIROTOR
        state.position_odom.x = self._x
        state.position_odom.z = self._z
        state.velocity_odom.x = self._vx
        state.velocity_odom.z = self._vz
        state.orientation_odom_body.w = 1.0
        state.state_valid = True
        state.localization_valid = True
        state.odometry_fresh = True
        state.imu_fresh = True
        state.acceleration_fresh = True
        state.stable = True
        self._state_publisher.publish(state)

    def _wait_for_command(self, predicate, timeout=4.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self._publish_state()
            try:
                command = rospy.wait_for_message(
                    "command", ControlCommand, timeout=0.2
                )
                if predicate(command):
                    return command
            except rospy.ROSException:
                pass
        self.fail("没有在超时前收到预期控制输出")

    def test_odom_landing_does_not_require_range(self):
        self._wait_for_command(lambda value: value.valid)
        rospy.wait_for_service(
            "controller/internal/command", timeout=3.0
        )
        internal_command = rospy.ServiceProxy(
            "controller/internal/command", InternalCommand
        )

        response = internal_command(
            InternalCommandRequest.TAKEOFF, 1.0
        )
        self.assertTrue(response.success, response.message)
        self._wait_for_command(
            lambda value: value.valid and value.takeoff_active
        )

        self._z = 1.0
        self._wait_for_command(
            lambda value: value.valid and not value.takeoff_active
        )
        self._x = 3.0
        response = internal_command(
            InternalCommandRequest.LAND_HOME, 0.0
        )
        self.assertTrue(response.success, response.message)

        toward_home = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and abs(value.body_rate.y) > 0.03
            )
        )

        # Close to home while still moving toward it at 1 m/s, the controller
        # must reverse horizontal acceleration to brake.  This catches the
        # old position+velocity objective conflict that kept pulling forward
        # until after crossing home.
        self._x = 0.5
        self._vx = -1.0
        braking = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.body_rate.y * toward_home.body_rate.y < -1e-3
            )
        )
        self.assertLess(
            braking.body_rate.y * toward_home.body_rate.y, 0.0
        )

        # No Range messages are published in this test.  The odom branch
        # must retain its original ground-relative Z and odom.vz touchdown
        # logic independently of distance_sensor_cfg.
        self._x = 0.0
        self._vx = 0.0
        self._z = 0.0
        self._vz = 0.0
        touchdown = self._wait_for_command(
            lambda value: (
                value.valid
                and value.landing_active
                and value.landing_touchdown
            )
        )
        self.assertTrue(touchdown.landing_touchdown)


if __name__ == "__main__":
    rospy.init_node("odom_landing_test")
    import rostest

    rostest.rosrun(
        "xd_uav_controller",
        "odom_landing_test",
        OdomLandingTest,
    )
