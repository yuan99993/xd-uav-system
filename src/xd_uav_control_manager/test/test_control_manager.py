#!/usr/bin/env python3

import math
import time
import unittest

import rospy
from geometry_msgs.msg import AccelWithCovarianceStamped
from mavros_msgs.msg import (
    AttitudeTarget,
    ExtendedState,
    State,
    VFR_HUD,
)
from mavros_msgs.srv import (
    CommandBool,
    CommandBoolResponse,
    CommandLong,
    CommandLongResponse,
    SetMode,
    SetModeResponse,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

from xd_uav_controller.msg import ControlCommand, ControlState
from xd_uav_controller.srv import (
    InternalCommand,
    InternalCommandRequest,
    InternalCommandResponse,
    Takeoff,
)
from xd_uav_state_estimators.msg import EstimatorStatus


class ControlManagerInterfaceTest(unittest.TestCase):

    def setUp(self):
        self._vehicle_type = rospy.get_param(
            "~vehicle_type", "multirotor"
        )
        self._armed = False
        self._mode = "MANUAL"
        self._takeoff_active = False
        self._landing_active = False
        self._landing_mode = ""
        self._touchdown = False
        self._airspeed = -1.5
        self._body_speed = 2.0
        self._publish_estimator = True
        self._publish_mavros_state = True
        self._publish_extended_state = True
        self._landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self._arming_requests = []
        self._force_disarm_requests = []
        self._controller_reset = False

        self._arming_service = rospy.Service(
            "arming", CommandBool, self._handle_arming
        )
        self._set_mode_service = rospy.Service(
            "set_mode", SetMode, self._handle_set_mode
        )
        self._command_long_service = rospy.Service(
            "command_long", CommandLong, self._handle_command_long
        )
        self._internal_command_service = rospy.Service(
            "controller_internal_command",
            InternalCommand,
            self._handle_internal_command,
        )

        self._publishers = {
            "main_odometry": rospy.Publisher(
                "main_odometry", Odometry, queue_size=10
            ),
            "main_acceleration": rospy.Publisher(
                "main_acceleration",
                AccelWithCovarianceStamped,
                queue_size=10,
            ),
            "imu": rospy.Publisher("imu", Imu, queue_size=20),
            "estimator_status": rospy.Publisher(
                "estimator_status", EstimatorStatus, queue_size=10
            ),
            "mavros_state": rospy.Publisher(
                "mavros_state", State, queue_size=10
            ),
            "mavros_extended_state": rospy.Publisher(
                "mavros_extended_state",
                ExtendedState,
                queue_size=10,
            ),
            "airspeed": rospy.Publisher(
                "airspeed", VFR_HUD, queue_size=10
            ),
            "controller_command": rospy.Publisher(
                "controller_command", ControlCommand, queue_size=20
            ),
        }

    def _handle_arming(self, request):
        self._arming_requests.append(request.value)
        if not request.value:
            # Reproduce PX4's "Disarming denied, not landed":
            # the normal command is rejected and armed stays true.
            return CommandBoolResponse(success=False, result=4)
        self._armed = True
        return CommandBoolResponse(success=True, result=0)

    def _handle_command_long(self, request):
        self._force_disarm_requests.append(request)
        if (
            request.command == 400
            and request.param1 == 0.0
            and request.param2 == 21196.0
        ):
            self._armed = False
            return CommandLongResponse(success=True, result=0)
        return CommandLongResponse(success=False, result=3)

    def _handle_set_mode(self, request):
        self._mode = request.custom_mode
        return SetModeResponse(mode_sent=True)

    def _handle_internal_command(self, request):
        if request.command == InternalCommandRequest.TAKEOFF:
            self._takeoff_active = True
            self._landed_state = (
                ExtendedState.LANDED_STATE_IN_AIR
            )
            return InternalCommandResponse(
                success=True, message="accepted"
            )
        if request.command == InternalCommandRequest.LAND:
            self._takeoff_active = False
            self._landing_active = True
            self._landing_mode = "here"
            self._touchdown = False
            return InternalCommandResponse(
                success=True, message="accepted"
            )
        if request.command == InternalCommandRequest.LAND_HOME:
            self._takeoff_active = False
            self._landing_active = True
            self._landing_mode = "home"
            self._touchdown = False
            return InternalCommandResponse(
                success=True, message="accepted"
            )
        if request.command == InternalCommandRequest.CANCEL_LANDING:
            self._landing_active = False
            self._landing_mode = ""
            self._touchdown = False
            return InternalCommandResponse(
                success=True, message="landing cancelled"
            )
        if request.command == InternalCommandRequest.RESET:
            self._controller_reset = True
            self._landing_active = False
            self._touchdown = False
            return InternalCommandResponse(
                success=True, message="reset"
            )
        return InternalCommandResponse(
            success=False, message="unknown command"
        )

    def _publish_inputs(self):
        now = rospy.Time.now()

        odometry = Odometry()
        odometry.header.stamp = now
        odometry.header.frame_id = "uav1/odom"
        odometry.child_frame_id = "uav1/base_link"
        odometry.pose.pose.orientation.z = math.sin(math.pi / 4.0)
        odometry.pose.pose.orientation.w = math.cos(math.pi / 4.0)
        odometry.twist.twist.linear.x = self._body_speed
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
        status.state = EstimatorStatus.RUNNING
        status.state_name = "RUNNING"
        status.state_valid = True
        status.localization_valid = True
        status.active_source = "mavros"
        if self._publish_estimator:
            self._publishers["estimator_status"].publish(status)

        mavros_state = State()
        mavros_state.header.stamp = now
        mavros_state.connected = True
        mavros_state.armed = self._armed
        mavros_state.mode = self._mode
        if self._publish_mavros_state:
            self._publishers["mavros_state"].publish(mavros_state)

        extended_state = ExtendedState()
        extended_state.header.stamp = now
        extended_state.landed_state = self._landed_state
        if self._publish_extended_state:
            self._publishers["mavros_extended_state"].publish(
                extended_state
            )

        airspeed = VFR_HUD()
        airspeed.header.stamp = now
        airspeed.airspeed = self._airspeed
        self._publishers["airspeed"].publish(airspeed)

        command = ControlCommand()
        command.header.stamp = now
        command.header.frame_id = "uav1/base_link"
        command.vehicle_type = (
            ControlCommand.VEHICLE_FIXEDWING
            if self._vehicle_type == "fixedwing"
            else ControlCommand.VEHICLE_MULTIROTOR
        )
        command.thrust = 0.5
        command.valid = True
        command.takeoff_active = self._takeoff_active
        command.landing_active = self._landing_active
        command.landing_touchdown = self._touchdown
        self._publishers["controller_command"].publish(command)

    def _wait_for(self, predicate, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self._publish_inputs()
            value = predicate()
            if value is not None:
                return value
            rospy.sleep(0.02)
        self.fail("没有在超时前收到预期管理器状态")

    def test_explicit_offboard_takeoff_and_land(self):
        state = self._wait_for(
            lambda: (
                message
                if (
                    message := rospy.wait_for_message(
                        "control_manager/state",
                        ControlState,
                        timeout=0.2,
                    )
                ).stable
                and message.airspeed_valid
                else None
            )
        )
        self.assertTrue(state.state_valid)
        self.assertTrue(state.airspeed_valid)
        self.assertAlmostEqual(state.airspeed, 0.0)
        self.assertAlmostEqual(state.velocity_odom.x, 0.0, delta=0.05)
        self.assertAlmostEqual(state.velocity_odom.y, 2.0, delta=0.05)

        if self._vehicle_type == "fixedwing":
            # 缓存的“未解锁/在地面”不能在MAVROS状态超时后继续授权负空速钳制。
            self._publish_mavros_state = False
            self._publish_extended_state = False
            stale_state = self._wait_for(
                lambda: (
                    message
                    if not (
                        message := rospy.wait_for_message(
                            "control_manager/state",
                            ControlState,
                            timeout=0.2,
                        )
                    ).airspeed_valid
                    else None
                )
            )
            self.assertFalse(stale_state.airspeed_valid)
            self._publish_mavros_state = True
            self._publish_extended_state = True
            self._wait_for(
                lambda: (
                    message
                    if (
                        message := rospy.wait_for_message(
                            "control_manager/state",
                            ControlState,
                            timeout=0.2,
                        )
                    ).airspeed_valid
                    else None
                )
            )

        self._airspeed = -3.5
        invalid_airspeed_state = self._wait_for(
            lambda: (
                message
                if not (
                    message := rospy.wait_for_message(
                        "control_manager/state",
                        ControlState,
                        timeout=0.2,
                    )
                ).airspeed_valid
                else None
            )
        )
        self.assertFalse(invalid_airspeed_state.airspeed_valid)
        self._airspeed = -1.5
        self._wait_for(
            lambda: (
                message
                if (
                    message := rospy.wait_for_message(
                        "control_manager/state",
                        ControlState,
                        timeout=0.2,
                    )
                ).airspeed_valid
                else None
            )
        )

        with self.assertRaises(rospy.ROSException):
            rospy.wait_for_message(
                "attitude_target", AttitudeTarget, timeout=0.25
            )
        self.assertEqual(self._mode, "MANUAL")
        self.assertFalse(self._armed)

        rospy.wait_for_service("control_manager/offboard", timeout=3.0)
        offboard = rospy.ServiceProxy(
            "control_manager/offboard", Trigger
        )
        response = offboard()
        self.assertTrue(response.success, response.message)
        self._wait_for(
            lambda: (
                rospy.wait_for_message(
                    "attitude_target", AttitudeTarget, timeout=0.2
                )
                if self._mode == "OFFBOARD"
                else None
            )
        )
        self.assertFalse(self._armed)
        self.assertNotIn(True, self._arming_requests)

        self._mode = "POSCTL"
        status = self._wait_for(
            lambda: (
                message
                if (
                    message := rospy.wait_for_message(
                        "control_manager/status",
                        String,
                        timeout=0.2,
                    )
                ).data.startswith("STANDBY")
                else None
            )
        )
        self.assertIn("人工切出OFFBOARD", status.data)
        self.assertEqual(self._mode, "POSCTL")

        response = offboard()
        self.assertTrue(response.success, response.message)
        self._wait_for(
            lambda: (
                message
                if self._mode == "OFFBOARD"
                and (
                    message := rospy.wait_for_message(
                        "control_manager/status",
                        String,
                        timeout=0.2,
                    )
                ).data.startswith("ACTIVE")
                else None
            )
        )

        rospy.wait_for_service(
            "control_manager/cancel_offboard", timeout=3.0
        )
        cancel_offboard = rospy.ServiceProxy(
            "control_manager/cancel_offboard", Trigger
        )
        response = cancel_offboard()
        self.assertTrue(response.success, response.message)
        self.assertEqual(self._mode, "POSCTL")
        self.assertFalse(self._armed)

        rospy.wait_for_service("control_manager/takeoff", timeout=3.0)
        takeoff = rospy.ServiceProxy(
            "control_manager/takeoff", Takeoff
        )
        if self._vehicle_type == "fixedwing":
            # Negative VFR_HUD noise is tolerated only while the
            # aircraft is disarmed/on ground. Flight requires a real
            # non-negative airspeed measurement.
            self._airspeed = 15.0
        response = takeoff(1.5)
        self.assertTrue(response.success, response.message)
        self._wait_for(
            lambda: (
                message
                if self._armed
                and (
                    message := rospy.wait_for_message(
                        "control_manager/status",
                        String,
                        timeout=0.2,
                    )
                ).data.startswith("ACTIVE")
                else None
            )
        )
        self.assertTrue(self._takeoff_active)
        self.assertEqual(self._mode, "OFFBOARD")

        self._publish_estimator = False
        deadline = time.time() + 0.2
        while time.time() < deadline:
            self._publish_inputs()
            rospy.sleep(0.02)
        self._publish_estimator = True
        target = self._wait_for(
            lambda: rospy.wait_for_message(
                "attitude_target", AttitudeTarget, timeout=0.2
            )
        )
        self.assertAlmostEqual(target.thrust, 0.5, delta=1e-3)
        self.assertTrue(self._armed)
        self.assertEqual(self._mode, "OFFBOARD")

        rospy.wait_for_service(
            "control_manager/land_home", timeout=3.0
        )
        land_home = rospy.ServiceProxy(
            "control_manager/land_home", Trigger
        )
        response = land_home()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._landing_active)
        self.assertEqual(self._landing_mode, "home")

        rospy.wait_for_service(
            "control_manager/cancel_land", timeout=3.0
        )
        cancel_land = rospy.ServiceProxy(
            "control_manager/cancel_land", Trigger
        )
        response = cancel_land()
        self.assertTrue(response.success, response.message)
        self.assertFalse(self._landing_active)
        self.assertTrue(self._armed)
        self.assertEqual(self._mode, "OFFBOARD")
        self._wait_for(
            lambda: (
                message
                if (
                    message := rospy.wait_for_message(
                        "control_manager/status",
                        String,
                        timeout=0.2,
                    )
                ).data.startswith("ACTIVE")
                else None
            )
        )

        response = land_home()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._landing_active)
        self.assertEqual(self._landing_mode, "home")

        # PX4 may keep reporting IN_AIR while the controller still
        # supplies thrust on the ground. Verify the independent
        # controller touchdown path still disarms the vehicle.
        self._touchdown = True
        deadline = time.time() + 0.08
        while time.time() < deadline:
            self._publish_inputs()
            rospy.sleep(0.02)
        response = cancel_land()
        self.assertFalse(response.success)
        self.assertTrue(self._landing_active)
        self.assertTrue(self._armed)
        self.assertEqual(self._mode, "OFFBOARD")
        if self._vehicle_type == "fixedwing":
            # The fixed-wing manager must not treat a contact signal
            # at high ground speed as a safe point to stop publishing
            # control and disarm.
            high_speed_target = self._wait_for(
                lambda: rospy.wait_for_message(
                    "attitude_target",
                    AttitudeTarget,
                    timeout=0.2,
                ),
                timeout=0.5,
            )
            self.assertAlmostEqual(
                high_speed_target.thrust, 0.5, delta=1e-3
            )
            self.assertTrue(self._armed)
            self.assertFalse(self._force_disarm_requests)
            self._body_speed = 0.5
        zero_target = self._wait_for(
            lambda: (
                target
                if (
                    target := rospy.wait_for_message(
                        "attitude_target",
                        AttitudeTarget,
                        timeout=0.2,
                    )
                ).thrust == 0.0
                else None
            )
        )
        self.assertEqual(zero_target.thrust, 0.0)
        self._wait_for(
            lambda: (
                True
                if not self._armed and self._controller_reset
                else None
            )
        )
        self.assertIn(False, self._arming_requests)
        self.assertTrue(self._force_disarm_requests)
        self.assertEqual(
            self._force_disarm_requests[-1].param2, 21196.0
        )

        status = self._wait_for(
            lambda: (
                message
                if (
                    message := rospy.wait_for_message(
                        "control_manager/status",
                        String,
                        timeout=0.2,
                    )
                ).data.startswith("STANDBY")
                else None
            )
        )
        self.assertTrue(status.data.startswith("STANDBY"))


if __name__ == "__main__":
    rospy.init_node("control_manager_interface_test")
    import rostest

    rostest.rosrun(
        "xd_uav_control_manager",
        "control_manager_interface_test",
        ControlManagerInterfaceTest,
    )
