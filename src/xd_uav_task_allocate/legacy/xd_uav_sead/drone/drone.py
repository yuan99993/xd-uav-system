#!/usr/bin/env python3

"""SEAD adapter for the current xd_uav manager/controller stack.

This module deliberately has no direct MAVROS command publisher or service
client.  MAVROS state is read only because the current manager state message
does not expose armed/PX4 mode; all control requests go through
xd_uav_control_manager and xd_uav_controller.
"""

from math import asin, atan2, cos, isfinite, sin
from time import time

import rospy
from mavros_msgs.msg import PositionTarget, State
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger

from xd_uav_controller.msg import ControlState
from xd_uav_controller.srv import Takeoff as ManagerTakeoff
from xd_uav_sead.comms.communication_info import FrameType


class XdUavAdapter(object):
    def __init__(self, uav_name="uav1", message_rate=10):
        del message_rate
        self.uav_name = str(uav_name)
        self.uses_external_control_manager = True
        self.reference_frame = str(
            rospy.get_param("~control/reference_frame", f"{self.uav_name}/odom")
        )
        self.shared_frame_enabled = bool(
            rospy.get_param("~control/shared_frame/enabled", False)
        )
        self.shared_frame_offset = [
            float(rospy.get_param("~control/shared_frame/offset_x", 0.0)),
            float(rospy.get_param("~control/shared_frame/offset_y", 0.0)),
            float(rospy.get_param("~control/shared_frame/offset_z", 0.0)),
        ]

        vehicle_type = str(
            rospy.get_param("~control/vehicle_type", "multirotor")
        ).lower()
        self.frame_type = (
            FrameType.Fixed_wing if vehicle_type == "fixedwing" else FrameType.Quad
        )
        self.armed = False
        self.mode = "POSHOLD"
        self.state_valid = False
        self.localization_valid = False
        self.local_pose = [0.0, 0.0, 0.0]
        self.control_local_pose = [0.0, 0.0, 0.0]
        self.local_velo = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.battery_volt = 0.0
        self.battery_perc = 0.0
        self.v = 15.0
        self.Rmin = 50.0
        self.type = 2
        self.offboard_control_source = "xd_position_reference"
        self.last_setpoint_time = 0.0

        self.setpoint_pub = rospy.Publisher(
            f"/{self.uav_name}/control/reference/setpoint",
            PositionTarget,
            queue_size=1,
        )
        rospy.Subscriber(
            f"/{self.uav_name}/control_manager/state",
            ControlState,
            self.control_state_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            f"/{self.uav_name}/mavros/state",
            State,
            self.mavros_state_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            f"/{self.uav_name}/mavros/battery",
            BatteryState,
            self.battery_callback,
            queue_size=10,
        )
        rospy.loginfo(
            "[SEAD] xd_uav adapter: state=/%s/control_manager/state, "
            "reference=/%s/control/reference/setpoint, frame=%s",
            self.uav_name,
            self.uav_name,
            self.reference_frame,
        )

    def control_state_callback(self, msg):
        self.state_valid = bool(msg.state_valid)
        self.localization_valid = bool(msg.localization_valid)
        self.control_local_pose = [
            float(msg.position_odom.x),
            float(msg.position_odom.y),
            float(msg.position_odom.z),
        ]
        if self.shared_frame_enabled:
            self.local_pose = [
                self.control_local_pose[index] + self.shared_frame_offset[index]
                for index in range(3)
            ]
        else:
            self.local_pose = list(self.control_local_pose)
        self.local_velo = [
            float(msg.velocity_odom.x),
            float(msg.velocity_odom.y),
            float(msg.velocity_odom.z),
        ]
        q = msg.orientation_odom_body
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(
            q.x, q.y, q.z, q.w
        )
        self.frame_type = (
            FrameType.Fixed_wing
            if msg.vehicle_type == ControlState.VEHICLE_FIXEDWING
            else FrameType.Quad
        )

    def mavros_state_callback(self, msg):
        self.armed = bool(msg.armed)
        self.mode = {
            "OFFBOARD": "GUIDED",
            "AUTO.LAND": "LAND",
            "AUTO.LOITER": "LOITER",
            "AUTO.RTL": "RTL",
            "AUTO.TAKEOFF": "AUTO",
        }.get(msg.mode, "POSHOLD")

    def battery_callback(self, msg):
        self.battery_volt = float(msg.voltage) if isfinite(msg.voltage) else 0.0
        percentage = float(msg.percentage)
        self.battery_perc = percentage * 100.0 if isfinite(percentage) else 0.0

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = asin(pitch_term)
        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def shared_to_control_waypoint(self, waypoint):
        values = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
        if not self.shared_frame_enabled:
            return values
        return [
            values[index] - self.shared_frame_offset[index] for index in range(3)
        ]

    def _publish_reference(self, message):
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.reference_frame
        self.setpoint_pub.publish(message)
        self.last_setpoint_time = time()

    def _call_trigger(self, service):
        name = f"/{self.uav_name}/control_manager/{service}"
        try:
            rospy.wait_for_service(name, timeout=2.0)
            response = rospy.ServiceProxy(name, Trigger)()
            if not response.success:
                rospy.logwarn("[%s] %s rejected: %s", self.uav_name, service, response.message)
            return bool(response.success)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("[%s] %s failed: %s", self.uav_name, service, exc)
            return False

    def takeoff(self, altitude):
        name = f"/{self.uav_name}/control_manager/takeoff"
        try:
            rospy.wait_for_service(name, timeout=2.0)
            response = rospy.ServiceProxy(name, ManagerTakeoff)(altitude=float(altitude))
            if not response.success:
                rospy.logwarn("[%s] takeoff rejected: %s", self.uav_name, response.message)
            return bool(response.success)
        except (ValueError, rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("[%s] takeoff failed: %s", self.uav_name, exc)
            return False

    def set_mode(self, mode):
        requested = str(mode).upper()
        if requested in ("GUIDED", "OFFBOARD"):
            return self._call_trigger("offboard")
        if requested in ("LAND", "AUTO.LAND"):
            return self._call_trigger("land")
        if requested in ("RTL", "AUTO.RTL"):
            return self._call_trigger("land_home")
        if requested in ("LOITER", "POSHOLD"):
            return self.hold_position()
        rospy.logwarn("[%s] unsupported xd manager mode request: %s", self.uav_name, mode)
        return False

    def hold_position(self):
        return self.guide_to_waypoint(self.local_pose, yaw=self.yaw)

    def guide_to_waypoint(self, waypoint, yaw=None):
        east, north, altitude = self.shared_to_control_waypoint(waypoint)
        if altitude <= 0.0:
            altitude = self.control_local_pose[2]
        message = PositionTarget()
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.position.x = east
        message.position.y = north
        message.position.z = altitude
        message.yaw = self.yaw if yaw is None else float(yaw)
        self.offboard_control_source = "xd_position_reference"
        self._publish_reference(message)
        return True

    def velocity_control(self, velocity, yaw=None):
        message = PositionTarget()
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.velocity.x = float(velocity[0])
        message.velocity.y = float(velocity[1])
        message.velocity.z = float(velocity[2])
        message.yaw = self.yaw if yaw is None else float(yaw)
        self.offboard_control_source = "xd_velocity_reference"
        self._publish_reference(message)
        return True

    def guide_velocity(self, speed_mps, heading_rad, vertical_speed_mps=0.0):
        speed = float(speed_mps)
        heading = float(heading_rad)
        return self.velocity_control(
            [speed * cos(heading), speed * sin(heading), float(vertical_speed_mps)],
            yaw=heading,
        )

    def set_offboard_control_source(self, source):
        # Kept as a small compatibility hook for mission algorithms and logs.
        self.offboard_control_source = str(source)


# Preserve the historic import name while exposing the actual architecture.
Drone = XdUavAdapter
