#!/usr/bin/env python3
"""Freeze a measured Gazebo-world to Fast-LIO odom alignment while disarmed."""

import math
import threading

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import tf2_ros
from xd_uav_controller.msg import ControlState


def pose_matrix(position, orientation):
    result = quaternion_matrix(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    result[0, 3], result[1, 3], result[2, 3] = (
        position.x, position.y, position.z)
    return result


class WorldAlignment:
    def __init__(self):
        self._model_name = rospy.get_param("~model_name", "uav1")
        self._world_frame = rospy.get_param("~world_frame", "world").strip("/")
        self._odom_frame = rospy.get_param("~odom_frame", "uav1/odom").strip("/")
        self._raw_odom_frame = rospy.get_param(
            "~raw_odom_frame", "uav1/fastlio_origin").strip("/")
        self._raw_body_frame = rospy.get_param(
            "~raw_body_frame", "uav1/mid360_imu_calibrated").strip("/")
        body_to_reference = rospy.get_param(
            "~body_to_raw_reference_translation_m", [0.011, 0.02329, 0.08588])
        if len(body_to_reference) != 3:
            raise ValueError("body_to_raw_reference_translation_m must have 3 values")
        self._body_to_raw_reference = np.identity(4)
        self._body_to_raw_reference[0:3, 3] = np.asarray(
            body_to_reference, dtype=float)
        self._timeout = float(rospy.get_param("~input_timeout", 0.20))
        self._max_residual = float(rospy.get_param(
            "~maximum_alignment_residual_m", 0.15))
        self._lock = threading.Lock()
        self._model_pose = None
        self._model_stamp = None
        self._state = None
        self._raw_odom = None
        self._armed = None
        self._armed_stamp = None
        self._matrix = None
        self._raw_matrix = None
        self._failure_latched = False
        self._reason = "not_authorized"
        self._healthy = False

        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._health_pub = rospy.Publisher("~healthy", Bool, queue_size=1, latch=True)
        self._diag_pub = rospy.Publisher(
            "~diagnostics", DiagnosticArray, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._models_callback, queue_size=1)
        rospy.Subscriber("control_manager/state", ControlState,
                         self._state_callback, queue_size=10)
        rospy.Subscriber("fastlio/Odometry", Odometry,
                         self._raw_odom_callback, queue_size=10)
        rospy.Subscriber("mavros/state", State, self._mavros_callback, queue_size=10)
        rospy.Service("~authorize", SetBool, self._authorize)
        rospy.Timer(rospy.Duration(0.05), self._timer)

    def _models_callback(self, message):
        try:
            index = message.name.index(self._model_name)
        except ValueError:
            with self._lock:
                self._model_pose = None
            return
        with self._lock:
            self._model_pose = message.pose[index]
            self._model_stamp = rospy.Time.now()

    def _state_callback(self, message):
        with self._lock:
            self._state = message

    def _raw_odom_callback(self, message):
        with self._lock:
            self._raw_odom = message

    def _mavros_callback(self, message):
        with self._lock:
            self._armed = bool(message.armed)
            self._armed_stamp = rospy.Time.now()

    def _inputs(self):
        now = rospy.Time.now()
        if self._armed is None or self._armed_stamp is None or (
                now - self._armed_stamp).to_sec() > self._timeout:
            return None, "mavros_state_stale"
        if self._armed:
            return None, "armed"
        if self._model_pose is None or self._model_stamp is None or (
                now - self._model_stamp).to_sec() > self._timeout:
            return None, "gazebo_pose_stale"
        state = self._state
        if state is None or (now - state.header.stamp).to_sec() > self._timeout:
            return None, "control_state_stale"
        if (not state.state_valid or not state.localization_valid or
                not state.odometry_fresh or
                state.header.frame_id.strip("/") != self._odom_frame):
            return None, "control_state_invalid"
        raw = self._raw_odom
        if raw is None or (now - raw.header.stamp).to_sec() > self._timeout:
            return None, "raw_fastlio_stale"
        if (raw.header.frame_id.strip("/") != self._raw_odom_frame or
                raw.child_frame_id.strip("/") != self._raw_body_frame):
            return None, "raw_fastlio_frame_invalid"
        return state, "ok"

    def _authorize(self, request):
        with self._lock:
            if not request.data:
                self._matrix = None
                self._raw_matrix = None
                self._healthy = False
                self._reason = ("failure_latched_restart_required"
                                if self._failure_latched else
                                "authorization_revoked")
                return SetBoolResponse(True, self._reason)
            if self._failure_latched:
                self._reason = "failure_latched_restart_required"
                return SetBoolResponse(False, self._reason)
            state, reason = self._inputs()
            if state is None:
                self._reason = reason
                return SetBoolResponse(False, reason)
            world_body = pose_matrix(
                self._model_pose.position, self._model_pose.orientation)
            odom_body = pose_matrix(
                state.position_odom, state.orientation_odom_body)
            self._matrix = np.dot(world_body, np.linalg.inv(odom_body))
            raw_reference = pose_matrix(
                self._raw_odom.pose.pose.position,
                self._raw_odom.pose.pose.orientation)
            raw_body = np.dot(raw_reference,
                              np.linalg.inv(self._body_to_raw_reference))
            self._raw_matrix = np.dot(world_body, np.linalg.inv(raw_body))
            self._healthy = True
            self._reason = "ok"
            return SetBoolResponse(True, "alignment_frozen")

    def _timer(self, _event):
        with self._lock:
            matrix = None if self._matrix is None else self._matrix.copy()
            raw_matrix = (None if self._raw_matrix is None
                          else self._raw_matrix.copy())
            state = self._state
            raw_odom = self._raw_odom
            model_pose = self._model_pose
            healthy = self._healthy
            reason = self._reason
            failure_latched = self._failure_latched
            if matrix is not None and state is not None and model_pose is not None:
                predicted = np.dot(matrix, pose_matrix(
                    state.position_odom, state.orientation_odom_body))
                truth = pose_matrix(model_pose.position, model_pose.orientation)
                residual = float(np.linalg.norm(predicted[0:3, 3] - truth[0:3, 3]))
                if not math.isfinite(residual) or residual > self._max_residual:
                    healthy = False
                    reason = "alignment_residual_exceeded"
                    self._healthy = False
                    self._reason = reason
                    self._failure_latched = True
            else:
                residual = float("nan")
            if (raw_matrix is not None and raw_odom is not None and
                    model_pose is not None):
                raw_reference = pose_matrix(
                    raw_odom.pose.pose.position,
                    raw_odom.pose.pose.orientation)
                raw_body = np.dot(
                    raw_reference, np.linalg.inv(self._body_to_raw_reference))
                raw_predicted = np.dot(raw_matrix, raw_body)
                raw_truth = pose_matrix(
                    model_pose.position, model_pose.orientation)
                raw_residual_vector = (
                    raw_predicted[0:3, 3] - raw_truth[0:3, 3])
                raw_residual = float(np.linalg.norm(raw_residual_vector))
            else:
                raw_residual_vector = np.full(3, float("nan"))
                raw_residual = float("nan")
            failure_latched = self._failure_latched
        if matrix is not None:
            q = quaternion_from_matrix(matrix)
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = self._world_frame
            transform.child_frame_id = self._odom_frame
            transform.transform.translation.x = matrix[0, 3]
            transform.transform.translation.y = matrix[1, 3]
            transform.transform.translation.z = matrix[2, 3]
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            self._broadcaster.sendTransform(transform)
        self._health_pub.publish(Bool(data=healthy))
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR,
            name=rospy.get_name() + "/alignment", hardware_id="simulation_alignment",
            message="healthy" if healthy else "fail_closed",
            values=[KeyValue("reason", reason),
                    KeyValue("residual_m", str(residual)),
                    KeyValue("raw_fastlio_residual_m", str(raw_residual)),
                    KeyValue("raw_fastlio_residual_xyz_m", ",".join(
                        str(float(value)) for value in raw_residual_vector)),
                    KeyValue("maximum_residual_m", str(self._max_residual)),
                    KeyValue("failure_latched", str(failure_latched).lower())])
        self._diag_pub.publish(DiagnosticArray(
            header=Header(stamp=rospy.Time.now()), status=[status]))


if __name__ == "__main__":
    rospy.init_node("gazebo_fastlio_world_alignment")
    WorldAlignment()
    rospy.spin()
