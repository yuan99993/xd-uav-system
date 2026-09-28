#!/usr/bin/env python3

import math
import time
import unittest

import rospy
from geometry_msgs.msg import AccelWithCovarianceStamped
from mavros_msgs.msg import AttitudeTarget, ExtendedState, State
from mavros_msgs.srv import (
    ParamGet,
    ParamGetResponse,
    MessageInterval,
    MessageIntervalResponse,
    SetMode,
    SetModeResponse,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from xd_uav_controller.msg import ControlCommand
from xd_uav_state_estimators.msg import EstimatorStatus


class ArduCopterControlManagerTest(unittest.TestCase):

    def setUp(self):
        self._mode = "STABILIZE"
        self._guid_options = 0
        self._hover_throttle = 0.39
        self._set_mode_requests = []
        self._message_interval_requests = []
        self._message_interval_success = False
        self._services = [
            rospy.Service("set_mode", SetMode, self._set_mode),
            rospy.Service("parameter_get", ParamGet, self._param_get),
            rospy.Service(
                "message_interval", MessageInterval, self._message_interval
            ),
        ]
        self._publishers = {
            "main_odometry": rospy.Publisher(
                "main_odometry", Odometry, queue_size=10
            ),
            "main_acceleration": rospy.Publisher(
                "main_acceleration",
                AccelWithCovarianceStamped,
                queue_size=10,
            ),
            "imu": rospy.Publisher("imu", Imu, queue_size=10),
            "estimator_status": rospy.Publisher(
                "estimator_status", EstimatorStatus, queue_size=10
            ),
            "mavros_state": rospy.Publisher(
                "mavros_state", State, queue_size=10
            ),
            "mavros_extended_state": rospy.Publisher(
                "mavros_extended_state", ExtendedState, queue_size=10
            ),
            "controller_command": rospy.Publisher(
                "controller_command", ControlCommand, queue_size=10
            ),
        }

    def _set_mode(self, request):
        self._set_mode_requests.append(request.custom_mode)
        self._mode = request.custom_mode
        return SetModeResponse(mode_sent=True)

    def _param_get(self, request):
        response = ParamGetResponse()
        if request.param_id == "GUID_OPTIONS":
            response.success = True
            response.value.integer = self._guid_options
        elif request.param_id == "MOT_THST_HOVER":
            response.success = True
            response.value.real = self._hover_throttle
        return response

    def _message_interval(self, request):
        self._message_interval_requests.append(
            (request.message_id, request.message_rate)
        )
        return MessageIntervalResponse(success=self._message_interval_success)

    def _publish_inputs(self):
        now = rospy.Time.now()
        odometry = Odometry()
        odometry.header.stamp = now
        odometry.header.frame_id = "uav1/odom"
        odometry.child_frame_id = "uav1/base_link"
        odometry.pose.pose.orientation.w = 1.0
        self._publishers["main_odometry"].publish(odometry)

        acceleration = AccelWithCovarianceStamped()
        acceleration.header.stamp = now
        acceleration.header.frame_id = "uav1/odom"
        self._publishers["main_acceleration"].publish(acceleration)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "uav1/base_link"
        imu.orientation.w = 1.0
        self._publishers["imu"].publish(imu)

        status = EstimatorStatus()
        status.header.stamp = now
        status.state_valid = True
        status.localization_valid = True
        self._publishers["estimator_status"].publish(status)

        state = State()
        state.header.stamp = now
        state.connected = True
        state.mode = self._mode
        self._publishers["mavros_state"].publish(state)

        extended = ExtendedState()
        extended.header.stamp = now
        extended.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self._publishers["mavros_extended_state"].publish(extended)

        command = ControlCommand()
        command.header.stamp = now
        command.header.frame_id = "uav1/base_link"
        command.vehicle_type = ControlCommand.VEHICLE_MULTIROTOR
        command.output_type = ControlCommand.OUTPUT_ATTITUDE
        command.attitude_valid = True
        command.attitude.z = math.sin(0.2)
        command.attitude.w = math.cos(0.2)
        command.thrust = 0.55
        command.hover_throttle = 0.39
        command.valid = True
        self._publishers["controller_command"].publish(command)

    def _spin_until(self, predicate, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self._publish_inputs()
            result = predicate()
            if result is not None:
                return result
            rospy.sleep(0.02)
        self.fail("ArduCopter manager未在超时前达到预期状态")

    def test_guid_options_gate_and_guided_attitude_target(self):
        rospy.wait_for_service("control_manager/offboard", timeout=3.0)
        activate = rospy.ServiceProxy("control_manager/offboard", Trigger)

        rejected = activate()
        self.assertFalse(rejected.success)
        self.assertIn("GUID_OPTIONS", rejected.message)
        self.assertEqual([], self._set_mode_requests)

        self._guid_options = 8
        interval_rejected = activate()
        self.assertFalse(interval_rejected.success)
        self.assertIn("245", interval_rejected.message)
        self.assertEqual([], self._set_mode_requests)

        self._message_interval_success = True
        accepted = activate()
        self.assertTrue(accepted.success, accepted.message)
        self.assertIn((245, 10.0), self._message_interval_requests)

        target = self._spin_until(
            lambda: (
                message
                if self._mode == "GUIDED"
                and (
                    message := rospy.wait_for_message(
                        "attitude_target", AttitudeTarget, timeout=0.2
                    )
                )
                else None
            )
        )
        expected_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )
        self.assertEqual(expected_mask, target.type_mask)
        self.assertAlmostEqual(math.sin(0.2), target.orientation.z, 1e-6)
        self.assertAlmostEqual(math.cos(0.2), target.orientation.w, 1e-6)
        self.assertAlmostEqual(0.55, target.thrust, delta=1e-6)
        self.assertIn("GUIDED", self._set_mode_requests)


if __name__ == "__main__":
    rospy.init_node("arducopter_control_manager_test")
    import rostest

    rostest.rosrun(
        "xd_uav_control_manager",
        "arducopter_control_manager_test",
        ArduCopterControlManagerTest,
    )
