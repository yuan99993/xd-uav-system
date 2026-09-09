#!/usr/bin/env python3
"""Explicit, fail-closed arbitration of SEAD and EGO reference candidates."""

import copy

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import (
    do_transform_pose, do_transform_vector3)

from xd_uav_planning.core import (
    ReferenceSample, switch_delta, validate_reference)


class ReferenceMux:
    def __init__(self):
        self._frame = rospy.get_param("~common_frame", "world").strip("/")
        self._output_frame = rospy.get_param(
            "~output_frame", self._frame).strip("/")
        self._transform_timeout = float(rospy.get_param(
            "~output_transform_timeout", 0.05))
        self._timeout = float(rospy.get_param("~candidate_timeout", 0.20))
        self._health_timeout = float(rospy.get_param("~health_timeout", 0.30))
        self._future_tolerance = float(rospy.get_param("~future_tolerance", 0.02))
        self._position_jump = float(rospy.get_param(
            "~switch_position_jump", 0.20))
        self._velocity_jump = float(rospy.get_param(
            "~switch_velocity_jump", 0.30))
        self._owner = rospy.get_param("~initial_owner", "none")
        self._enabled = True
        if self._owner not in ("none", "sead", "ego"):
            raise rospy.ROSInitException("initial_owner must be none, sead or ego")
        self._candidates = {"sead": None, "ego": None}
        self._healthy = {"sead": (False, None), "ego": (False, None)}
        self._last_output = None
        self._baseline = None
        self._reason = "owner_none" if self._owner == "none" else "waiting"
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._publisher = rospy.Publisher(
            rospy.get_param("~output_topic", "control/reference/setpoint"),
            PositionTarget, queue_size=10)
        self._owner_pub = rospy.Publisher(
            rospy.get_param("~owner_topic", "integration/reference_owner"),
            String, queue_size=1, latch=True)
        self._diag_pub = rospy.Publisher(
            rospy.get_param("~diagnostics_topic", "integration/mux_diagnostics"),
            DiagnosticArray, queue_size=1)
        self._subs = [
            rospy.Subscriber(rospy.get_param("~sead_candidate_topic",
                                             "sead/reference_candidate"),
                             PositionTarget, self._candidate_callback,
                             callback_args="sead", queue_size=10),
            rospy.Subscriber(rospy.get_param("~ego_candidate_topic",
                                             "ego/reference_candidate"),
                             PositionTarget, self._candidate_callback,
                             callback_args="ego", queue_size=20),
            rospy.Subscriber(rospy.get_param("~sead_healthy_topic",
                                             "sead/reference_healthy"),
                             Bool, self._health_callback,
                             callback_args="sead", queue_size=1),
            rospy.Subscriber(rospy.get_param("~ego_healthy_topic",
                                             "planning/healthy"),
                             Bool, self._health_callback,
                             callback_args="ego", queue_size=1),
            rospy.Subscriber(rospy.get_param("~switch_baseline_topic",
                                             "integration/switch_baseline"),
                             PositionTarget, self._baseline_callback,
                             queue_size=1),
        ]
        self._service = rospy.Service(
            "~select_ego", SetBool, self._select_ego)
        self._enable_service = rospy.Service(
            "~set_enabled", SetBool, self._set_enabled)
        self._timer = rospy.Timer(rospy.Duration(0.05), self._timer_callback)
        self._publish_status()

    @staticmethod
    def _vector(value):
        return (float(value.x), float(value.y), float(value.z))

    def _sample(self, message):
        return ReferenceSample(
            stamp=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id.strip("/"),
            coordinate_frame=int(message.coordinate_frame),
            type_mask=int(message.type_mask),
            position=self._vector(message.position),
            velocity=self._vector(message.velocity),
            acceleration=self._vector(message.acceleration_or_force),
            yaw=float(message.yaw), yaw_rate=float(message.yaw_rate))

    def _valid(self, message):
        return validate_reference(
            self._sample(message), rospy.Time.now().to_sec(), self._frame,
            self._timeout, self._future_tolerance,
            PositionTarget.FRAME_LOCAL_NED, PositionTarget.FORCE)

    def _candidate_callback(self, message, source):
        validation = self._valid(message)
        self._candidates[source] = copy.deepcopy(message) if validation.valid else None
        if source == self._owner and self._enabled:
            if not validation.valid:
                self._reason = source + ":" + validation.reason
            elif not self._source_healthy(source):
                self._reason = source + ":health_false"
            else:
                output = self._transform_output(message)
                if output is not None:
                    self._publisher.publish(output)
                    self._last_output = copy.deepcopy(message)
                    self._reason = "ok"

    def _health_callback(self, message, source):
        self._healthy[source] = (bool(message.data), rospy.Time.now())
        if source == self._owner and not message.data:
            self._reason = source + ":health_false"

    def _baseline_callback(self, message):
        validation = self._valid(message)
        self._baseline = copy.deepcopy(message) if validation.valid else None

    def _transform_output(self, message):
        if self._output_frame == self._frame:
            return message
        try:
            transform = self._tf_buffer.lookup_transform(
                self._output_frame, self._frame, rospy.Time(0),
                rospy.Duration(self._transform_timeout))
            pose = PoseStamped()
            pose.header = message.header
            pose.header.frame_id = self._frame
            pose.pose.position = message.position
            quaternion = quaternion_from_euler(0.0, 0.0, message.yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            transformed_pose = do_transform_pose(pose, transform)
            velocity = Vector3Stamped()
            velocity.header = pose.header
            velocity.vector = message.velocity
            transformed_velocity = do_transform_vector3(velocity, transform)
            acceleration = Vector3Stamped()
            acceleration.header = pose.header
            acceleration.vector = message.acceleration_or_force
            transformed_acceleration = do_transform_vector3(
                acceleration, transform)
            output = copy.deepcopy(message)
            output.header.frame_id = self._output_frame
            output.position = transformed_pose.pose.position
            output.velocity = transformed_velocity.vector
            output.acceleration_or_force = transformed_acceleration.vector
            orientation = transformed_pose.pose.orientation
            output.yaw = euler_from_quaternion((
                orientation.x, orientation.y,
                orientation.z, orientation.w))[2]
            return output
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            self._reason = "output_transform_unavailable:{}".format(error)
            return None

    def _select_ego(self, request):
        target = "ego" if request.data else "sead"
        candidate = self._candidates[target]
        if not self._source_healthy(target) or candidate is None:
            return SetBoolResponse(False, target + " is not ready")
        validation = self._valid(candidate)
        if not validation.valid:
            return SetBoolResponse(False, validation.reason)
        switch_from = (self._last_output if self._last_output is not None
                       else self._baseline)
        if switch_from is None:
            return SetBoolResponse(False, "switch baseline is not ready")
        baseline_validation = self._valid(switch_from)
        if not baseline_validation.valid:
            return SetBoolResponse(False, "switch baseline:" +
                                   baseline_validation.reason)
        delta = switch_delta(
            self._sample(switch_from), self._sample(candidate),
            self._position_jump, self._velocity_jump)
        if not delta.valid:
            return SetBoolResponse(False, delta.reason)
        self._owner = target
        self._reason = "owner_changed"
        self._owner_pub.publish(String(data=self._owner))
        return SetBoolResponse(True, "owner=" + self._owner)

    def _set_enabled(self, request):
        self._enabled = bool(request.data)
        if not self._enabled:
            # Do not allow a cached command from the cancelled navigation goal
            # to be emitted when planning is enabled for the next goal.
            self._candidates = {"sead": None, "ego": None}
            self._last_output = None
            self._reason = "output_disabled"
        else:
            self._reason = "waiting"
        self._publish_status()
        return SetBoolResponse(True, "enabled=" + str(self._enabled).lower())

    def _timer_callback(self, _event):
        if not self._enabled:
            self._reason = "output_disabled"
        elif self._owner != "none":
            candidate = self._candidates[self._owner]
            if not self._source_healthy(self._owner):
                self._reason = self._owner + ":health_false"
            elif candidate is None:
                self._reason = self._owner + ":candidate_missing"
            else:
                validation = self._valid(candidate)
                if not validation.valid:
                    self._reason = self._owner + ":" + validation.reason
        self._publish_status()

    def _source_healthy(self, source):
        value, stamp = self._healthy[source]
        return (value and stamp is not None and
                (rospy.Time.now() - stamp).to_sec() <= self._health_timeout)

    def _publish_status(self):
        self._owner_pub.publish(String(data=self._owner))
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/reference_mux"
        status.hardware_id = "xd_uav_planning"
        status.level = (DiagnosticStatus.OK if self._reason == "ok"
                        else DiagnosticStatus.ERROR)
        status.message = "forwarding" if self._reason == "ok" else "fail_closed"
        status.values = [KeyValue(key="owner", value=self._owner),
                         KeyValue(key="enabled",
                                  value=str(self._enabled).lower()),
                         KeyValue(key="reason", value=self._reason),
                         KeyValue(key="output_frame", value=self._output_frame)]
        array.status = [status]
        self._diag_pub.publish(array)


def main():
    rospy.init_node("reference_mux")
    ReferenceMux()
    rospy.spin()


if __name__ == "__main__":
    main()
