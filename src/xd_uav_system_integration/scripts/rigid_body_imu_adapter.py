#!/usr/bin/env python3
"""Transform an offset sensor IMU into a calibrated vehicle body IMU."""

import copy
import math
import threading

import rospy
from sensor_msgs.msg import Imu

from xd_uav_system_integration.core import rigid_body_imu_to_body


class RigidBodyImuAdapter:
    def __init__(self):
        self._lock = threading.Lock()
        self._body_frame = rospy.get_param("~body_frame")
        self._sensor_frame = rospy.get_param("~sensor_frame")
        self._translation = tuple(float(value) for value in rospy.get_param(
            "~body_to_sensor_translation_m"))
        self._rotation = tuple(float(value) for value in rospy.get_param(
            "~sensor_to_body_rotation"))
        if len(self._translation) != 3 or len(self._rotation) != 9:
            raise rospy.ROSInitException("invalid rigid IMU extrinsic dimensions")
        self._last_stamp = None
        self._last_omega = None
        self._alpha = (0.0, 0.0, 0.0)
        self._alpha_tau = float(rospy.get_param(
            "~angular_acceleration_time_constant_s", 0.05))
        self._maximum_dt = float(rospy.get_param("~maximum_dt_s", 0.05))
        self._publisher = rospy.Publisher(
            rospy.get_param("~output_topic", "integration/imu_base_link"),
            Imu, queue_size=50)
        rospy.Subscriber(rospy.get_param("~input_topic"), Imu,
                         self._callback, queue_size=100)

    def _callback(self, message):
        if message.header.frame_id.strip("/") != self._sensor_frame.strip("/"):
            rospy.logerr_throttle(1.0, "rigid IMU input frame mismatch")
            return
        stamp = message.header.stamp.to_sec()
        omega_sensor = (message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z)
        acceleration_sensor = (message.linear_acceleration.x,
                               message.linear_acceleration.y,
                               message.linear_acceleration.z)
        if not all(math.isfinite(value) for value in
                   omega_sensor + acceleration_sensor + (stamp,)):
            return
        with self._lock:
            rotated_acceleration, omega_body = rigid_body_imu_to_body(
                acceleration_sensor, omega_sensor, (0.0, 0.0, 0.0),
                self._translation, self._rotation)
            if self._last_stamp is not None:
                dt = stamp - self._last_stamp
                if 1.0e-4 < dt <= self._maximum_dt:
                    raw = tuple((omega_body[index] - self._last_omega[index]) / dt
                                for index in range(3))
                    gain = dt / (max(0.0, self._alpha_tau) + dt)
                    self._alpha = tuple(self._alpha[index] + gain *
                                        (raw[index] - self._alpha[index])
                                        for index in range(3))
                elif dt <= 0.0 or dt > self._maximum_dt:
                    self._alpha = (0.0, 0.0, 0.0)
            acceleration_body, omega_body = rigid_body_imu_to_body(
                acceleration_sensor, omega_sensor, self._alpha,
                self._translation, self._rotation)
            self._last_stamp = stamp
            self._last_omega = omega_body
        output = copy.deepcopy(message)
        output.header.frame_id = self._body_frame
        output.linear_acceleration.x, output.linear_acceleration.y, \
            output.linear_acceleration.z = acceleration_body
        output.angular_velocity.x, output.angular_velocity.y, \
            output.angular_velocity.z = omega_body
        # The MRS installation has zero RPY.  Keep orientation and covariance;
        # non-identity profiles must add an explicit quaternion transform.
        if self._rotation != (1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 1.0):
            rospy.logerr_throttle(1.0, "non-identity IMU rotation unsupported")
            return
        self._publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("rigid_body_imu_adapter")
    RigidBodyImuAdapter()
    rospy.spin()
