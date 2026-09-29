#!/usr/bin/env python3
"""Fail-closed state and command adapter for EGO-Swarm."""

import math
import sys

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import (
    do_transform_pose, do_transform_vector3)
from xd_uav_controller.msg import ControlState

from xd_uav_planning.core import (
    CommandSample,
    StateSample,
    validate_command,
    validate_state,
)


class EgoBridge:
    def __init__(self, position_command_type):
        self._position_command_type = position_command_type
        self._common_frame = rospy.get_param("~common_frame", "world")
        self._body_frame = rospy.get_param("~body_frame", "")
        self._state_timeout = float(rospy.get_param("~state_timeout", 0.2))
        self._command_timeout = float(rospy.get_param("~command_timeout", 0.2))
        self._future_tolerance = float(
            rospy.get_param("~future_tolerance", 0.02))
        self._transform_state = bool(
            rospy.get_param("~transform_state_to_common", False))
        self._transform_timeout = float(
            rospy.get_param("~transform_timeout", 0.05))
        self._allow_nonfinite_yaw_rate = bool(
            rospy.get_param("~allow_nonfinite_yaw_rate", False))
        self._state_reason = "state_not_received"
        self._command_reason = "command_not_received"
        self._last_state_stamp = None
        self._last_command_stamp = None
        self._state_valid = False
        self._command_valid = False
        self._tf_buffer = None
        self._tf_listener = None
        if self._transform_state:
            self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        state_topic = rospy.get_param(
            "~state_topic", "control_manager/state")
        odometry_topic = rospy.get_param("~odometry_topic", "ego/odometry")
        command_topic = rospy.get_param(
            "~position_command_topic", "ego/position_command")
        candidate_topic = rospy.get_param(
            "~reference_candidate_topic", "ego/reference_candidate")
        healthy_topic = rospy.get_param(
            "~healthy_topic", "ego/bridge/healthy")
        diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "ego/bridge/diagnostics")

        self._odometry_pub = rospy.Publisher(
            odometry_topic, Odometry, queue_size=10)
        self._candidate_pub = rospy.Publisher(
            candidate_topic, PositionTarget, queue_size=10)
        self._healthy_pub = rospy.Publisher(
            healthy_topic, Bool, queue_size=1, latch=True)
        self._diagnostics_pub = rospy.Publisher(
            diagnostics_topic, DiagnosticArray, queue_size=1)
        self._state_sub = rospy.Subscriber(
            state_topic, ControlState, self._state_callback, queue_size=10)
        self._command_sub = rospy.Subscriber(
            command_topic, position_command_type, self._command_callback,
            queue_size=50)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)

    @staticmethod
    def _vector(vector):
        return (float(vector.x), float(vector.y), float(vector.z))

    def _state_sample(self, message):
        orientation = message.orientation_odom_body
        return StateSample(
            stamp=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id,
            body_frame_id=message.body_frame_id,
            position=self._vector(message.position_odom),
            velocity_world=self._vector(message.velocity_odom),
            orientation=(float(orientation.x), float(orientation.y),
                         float(orientation.z), float(orientation.w)),
            body_rate=self._vector(message.body_rate),
            state_valid=message.state_valid,
            localization_valid=message.localization_valid,
            odometry_fresh=message.odometry_fresh,
        )

    def _command_sample(self, message):
        return CommandSample(
            stamp=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id,
            position=self._vector(message.position),
            velocity=self._vector(message.velocity),
            acceleration=self._vector(message.acceleration),
            yaw=float(message.yaw),
            yaw_rate=float(message.yaw_dot),
            trajectory_flag=int(message.trajectory_flag),
        )

    def _state_callback(self, message):
        sample = self._state_sample(message)
        transformed_pose = None
        transformed_velocity = None
        # Never pass an empty frame into tf2.  Besides producing noisy
        # InvalidArgumentException logs, doing so bypassed the bridge's normal
        # fail-closed validation path during manager startup.
        if not sample.frame_id:
            self._state_valid = False
            self._state_reason = "state_frame_empty"
            self._last_state_stamp = sample.stamp
            return
        if self._transform_state and sample.frame_id != self._common_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._common_frame, message.header.frame_id,
                    message.header.stamp, rospy.Duration(self._transform_timeout))
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position = message.position_odom
                pose.pose.orientation = message.orientation_odom_body
                transformed_pose = do_transform_pose(pose, transform)
                velocity = Vector3Stamped()
                velocity.header = message.header
                velocity.vector = message.velocity_odom
                transformed_velocity = do_transform_vector3(velocity, transform)
                sample = StateSample(
                    stamp=sample.stamp,
                    frame_id=self._common_frame,
                    body_frame_id=sample.body_frame_id,
                    position=self._vector(transformed_pose.pose.position),
                    velocity_world=self._vector(transformed_velocity.vector),
                    orientation=(float(transformed_pose.pose.orientation.x),
                                 float(transformed_pose.pose.orientation.y),
                                 float(transformed_pose.pose.orientation.z),
                                 float(transformed_pose.pose.orientation.w)),
                    body_rate=sample.body_rate,
                    state_valid=sample.state_valid,
                    localization_valid=sample.localization_valid,
                    odometry_fresh=sample.odometry_fresh)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                self._state_valid = False
                self._state_reason = "state_transform_unavailable: {}".format(error)
                self._last_state_stamp = sample.stamp
                return
        result = validate_state(
            sample, rospy.Time.now().to_sec(), self._common_frame,
            self._body_frame, self._state_timeout, self._future_tolerance)
        self._state_valid = result.valid
        self._state_reason = result.reason
        self._last_state_stamp = sample.stamp
        if not result.valid:
            return

        odometry = Odometry()
        odometry.header = message.header
        odometry.header.frame_id = self._common_frame
        odometry.child_frame_id = message.body_frame_id
        odometry.pose.pose.position = (transformed_pose.pose.position
                                       if transformed_pose is not None
                                       else message.position_odom)
        odometry.pose.pose.orientation = (transformed_pose.pose.orientation
                                          if transformed_pose is not None
                                          else message.orientation_odom_body)
        # EGO reads twist.linear numerically as world-frame velocity. This is
        # intentionally an EGO-only Odometry contract, not generic ROS odom.
        odometry.twist.twist.linear = (transformed_velocity.vector
                                       if transformed_velocity is not None
                                       else message.velocity_odom)
        odometry.twist.twist.angular = message.body_rate
        self._odometry_pub.publish(odometry)

    def _command_callback(self, message):
        sample = self._command_sample(message)
        result = validate_command(
            sample, rospy.Time.now().to_sec(), self._common_frame,
            self._position_command_type.TRAJECTORY_STATUS_READY,
            self._command_timeout, self._future_tolerance,
            self._allow_nonfinite_yaw_rate)
        self._command_valid = result.valid
        self._command_reason = result.reason
        self._last_command_stamp = sample.stamp
        if not result.valid or not self._state_is_current():
            return

        candidate = PositionTarget()
        candidate.header = message.header
        candidate.header.frame_id = self._common_frame
        # The existing controller requires this enum while interpreting the
        # numeric fields as ROS ENU inertial values.
        candidate.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        yaw_rate_finite = math.isfinite(float(message.yaw_dot))
        candidate.type_mask = (0 if yaw_rate_finite
                               else PositionTarget.IGNORE_YAW_RATE)
        candidate.position = message.position
        candidate.velocity = message.velocity
        candidate.acceleration_or_force = message.acceleration
        candidate.yaw = message.yaw
        candidate.yaw_rate = message.yaw_dot if yaw_rate_finite else 0.0
        self._candidate_pub.publish(candidate)

    def _state_is_current(self):
        if not self._state_valid or self._last_state_stamp is None:
            return False
        age = rospy.Time.now().to_sec() - self._last_state_stamp
        return -self._future_tolerance <= age <= self._state_timeout

    def _command_is_current(self):
        if not self._command_valid or self._last_command_stamp is None:
            return False
        age = rospy.Time.now().to_sec() - self._last_command_stamp
        return -self._future_tolerance <= age <= self._command_timeout

    def _timer_callback(self, _event):
        state_current = self._state_is_current()
        command_current = self._command_is_current()
        if self._state_valid and not state_current:
            self._state_reason = "state_stream_stale"
        if self._command_valid and not command_current:
            self._command_reason = "command_stream_stale"
        healthy = state_current and command_current
        self._healthy_pub.publish(Bool(data=healthy))

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/health"
        status.hardware_id = "ego_bridge"
        status.level = (DiagnosticStatus.OK if healthy
                        else DiagnosticStatus.ERROR)
        status.message = "healthy" if healthy else "fail_closed"
        status.values = [
            KeyValue(key="state", value=self._state_reason),
            KeyValue(key="command", value=self._command_reason),
            KeyValue(key="common_frame", value=self._common_frame),
        ]
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)


def main():
    rospy.init_node("ego_bridge")
    try:
        from quadrotor_msgs.msg import PositionCommand
    except ImportError as error:
        rospy.logfatal("quadrotor_msgs is required to run ego_bridge: %s", error)
        return 2
    EgoBridge(PositionCommand)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
