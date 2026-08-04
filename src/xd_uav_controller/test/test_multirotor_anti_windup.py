#!/usr/bin/env python3

import time
import unittest

import rospy
from mavros_msgs.msg import PositionTarget

from xd_uav_controller.msg import ControlCommand, ControlState
from xd_uav_controller.srv import (
    InternalCommand,
    InternalCommandRequest,
)


class MultirotorAntiWindupTest(unittest.TestCase):

    def setUp(self):
        self._state_publisher = rospy.Publisher(
            "state", ControlState, queue_size=10
        )
        self._reference_publisher = rospy.Publisher(
            "reference_position_target",
            PositionTarget,
            queue_size=10,
        )

    @staticmethod
    def _state():
        state = ControlState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "uav1/odom"
        state.body_frame_id = "uav1/base_link"
        state.vehicle_type = ControlState.VEHICLE_MULTIROTOR
        state.orientation_odom_body.w = 1.0
        state.state_valid = True
        state.localization_valid = True
        state.odometry_fresh = True
        state.imu_fresh = True
        state.acceleration_fresh = True
        state.stable = True
        return state

    @staticmethod
    def _position_reference(z):
        reference = PositionTarget()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = "uav1/odom"
        reference.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        reference.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        reference.position.z = z
        return reference

    def _drive(self, reference, duration):
        deadline = time.time() + duration
        latest = None
        while time.time() < deadline and not rospy.is_shutdown():
            reference.header.stamp = rospy.Time.now()
            self._state_publisher.publish(self._state())
            self._reference_publisher.publish(reference)
            try:
                candidate = rospy.wait_for_message(
                    "command", ControlCommand, timeout=0.05
                )
                if candidate.valid:
                    latest = candidate
            except rospy.ROSException:
                pass
        self.assertIsNotNone(latest)
        return latest

    def _wait_until_ready(self):
        deadline = time.time() + 2.0
        while time.time() < deadline and not rospy.is_shutdown():
            self._state_publisher.publish(self._state())
            try:
                command = rospy.wait_for_message(
                    "command", ControlCommand, timeout=0.1
                )
                if command.valid:
                    return
            except rospy.ROSException:
                pass
        self.fail("控制器没有进入有效的初始悬停状态")

    def test_position_disturbance_integral_and_saturation_recovery(self):
        self._wait_until_ready()

        # A small unsaturated position error must build a low-frequency
        # acceleration correction instead of retaining a permanent offset.
        small_error = self._position_reference(0.2)
        baseline = self._drive(small_error, 0.20)
        compensated = self._drive(small_error, 0.80)
        self.assertGreater(
            compensated.thrust,
            baseline.thrust + 0.005,
            "位置扰动积分没有增加稳态补偿",
        )

        rospy.wait_for_service(
            "controller/internal/command", timeout=2.0
        )
        reset = rospy.ServiceProxy(
            "controller/internal/command", InternalCommand
        )
        response = reset(InternalCommandRequest.RESET, 0.0)
        self.assertTrue(response.success, response.message)

        # A deliberately unreachable position saturates acceleration. The
        # integrator must stay neutral, so returning to zero error recovers
        # to hover thrust instead of retaining a large positive bias.
        self._drive(self._position_reference(100.0), 0.80)
        recovered = self._drive(self._position_reference(0.0), 0.25)
        self.assertAlmostEqual(recovered.thrust, 0.7, delta=0.012)


if __name__ == "__main__":
    rospy.init_node("multirotor_anti_windup_test")
    import rostest

    rostest.rosrun(
        "xd_uav_controller",
        "multirotor_anti_windup",
        MultirotorAntiWindupTest,
    )
