#!/usr/bin/env python3
"""Transform a physical sensor PointCloud2 into EGO's common world frame."""

import copy
import math

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
from xd_uav_controller.msg import ControlState

from xd_uav_planning.core import PointCloudSample, validate_pointcloud


class EgoPointCloudAdapter:
    def __init__(self):
        self._common_frame = rospy.get_param("~common_frame", "world").strip("/")
        self._expected_input_frame = rospy.get_param(
            "~expected_input_frame", "").strip("/")
        self._cloud_timeout = float(rospy.get_param("~cloud_timeout", 0.30))
        self._future_tolerance = float(rospy.get_param("~future_tolerance", 0.02))
        self._transform_timeout = float(rospy.get_param("~transform_timeout", 0.05))
        self._minimum_sensor_range = float(
            rospy.get_param("~minimum_sensor_range", 0.60))
        self._restamp_zero_timestamp = bool(
            rospy.get_param("~restamp_zero_timestamp", False))
        self._calibrated_body_transform = bool(
            rospy.get_param("~calibrated_body_transform", False))
        self._body_frame = rospy.get_param("~body_frame", "").strip("/")
        self._body_to_sensor = tuple(float(value) for value in rospy.get_param(
            "~body_to_sensor_translation_m", [0.0, 0.0, 0.0]))
        self._state_timeout = float(rospy.get_param("~state_timeout", 0.10))
        if self._calibrated_body_transform:
            if not self._body_frame or len(self._body_to_sensor) != 3:
                raise rospy.ROSInitException(
                    "calibrated transform requires body_frame and 3-D extrinsic")
        self._last_state = None
        self._reason = "cloud_not_received"
        self._last_valid_stamp = None

        input_topic = rospy.get_param("~input_topic", "sensing/points")
        output_topic = rospy.get_param("~output_topic", "ego/cloud_world")
        healthy_topic = rospy.get_param(
            "~healthy_topic", "ego/sensing/healthy")
        diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "ego/sensing/diagnostics")

        self._buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._cloud_pub = rospy.Publisher(output_topic, PointCloud2, queue_size=2)
        self._healthy_pub = rospy.Publisher(
            healthy_topic, Bool, queue_size=1, latch=True)
        self._diagnostics_pub = rospy.Publisher(
            diagnostics_topic, DiagnosticArray, queue_size=1)
        self._cloud_sub = rospy.Subscriber(
            input_topic, PointCloud2, self._cloud_callback, queue_size=2)
        self._state_sub = None
        if self._calibrated_body_transform:
            self._state_sub = rospy.Subscriber(
                rospy.get_param("~state_topic", "control_manager/state"),
                ControlState, self._state_callback, queue_size=20)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._timer_callback)
        self._publish_status(False)

    @staticmethod
    def _sample(message):
        return PointCloudSample(
            stamp=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id.strip("/"),
            width=int(message.width),
            height=int(message.height),
            point_step=int(message.point_step),
            data_size=len(message.data),
        )

    def _cloud_callback(self, message):
        now = rospy.Time.now()
        if message.header.stamp.is_zero() and self._restamp_zero_timestamp:
            message = copy.deepcopy(message)
            message.header.stamp = now
        validation = validate_pointcloud(
            self._sample(message), now.to_sec(), self._expected_input_frame,
            self._cloud_timeout, self._future_tolerance)
        if not validation.valid:
            self._reason = validation.reason
            self._last_valid_stamp = None
            self._publish_status(False)
            return
        try:
            transform = (self._calibrated_transform(message.header.stamp)
                         if self._calibrated_body_transform else
                         self._buffer.lookup_transform(
                             self._common_frame, message.header.frame_id,
                             message.header.stamp,
                             rospy.Duration(self._transform_timeout)))
            transformed = do_transform_cloud(message, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            self._reason = "cloud_transform_unavailable: {}".format(error)
            self._last_valid_stamp = None
            self._publish_status(False)
            return
        origin = transform.transform.translation
        finite_points = [
            point for point in point_cloud2.read_points(
                transformed, field_names=("x", "y", "z"), skip_nans=True)
            if math.sqrt((point[0] - origin.x) ** 2 +
                         (point[1] - origin.y) ** 2 +
                         (point[2] - origin.z) ** 2) >= self._minimum_sensor_range
        ]
        if not finite_points:
            self._reason = "cloud_no_finite_xyz"
            self._last_valid_stamp = None
            self._publish_status(False)
            return
        header = Header(stamp=message.header.stamp,
                        frame_id=self._common_frame)
        self._cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(header, finite_points))
        self._last_valid_stamp = message.header.stamp
        self._reason = "ok"
        self._publish_status(True)

    def _state_callback(self, message):
        if (message.state_valid and message.localization_valid and
                message.odometry_fresh and
                message.body_frame_id.strip("/") == self._body_frame):
            self._last_state = message
        else:
            self._last_state = None

    def _calibrated_transform(self, stamp):
        state = self._last_state
        if state is None:
            raise tf2_ros.LookupException("calibrated body state unavailable")
        age = abs((stamp - state.header.stamp).to_sec())
        if age > self._state_timeout:
            raise tf2_ros.ExtrapolationException(
                "cloud/body state separation {:.3f}s".format(age))
        pose = PoseStamped()
        pose.header = state.header
        pose.pose.position = state.position_odom
        pose.pose.orientation = state.orientation_odom_body
        if pose.header.frame_id.strip("/") != self._common_frame:
            world_from_state = self._buffer.lookup_transform(
                self._common_frame, pose.header.frame_id, state.header.stamp,
                rospy.Duration(self._transform_timeout))
            pose = do_transform_pose(pose, world_from_state)
        q = pose.pose.orientation
        matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[0, 3] = pose.pose.position.x
        matrix[1, 3] = pose.pose.position.y
        matrix[2, 3] = pose.pose.position.z
        sensor = np.identity(4)
        sensor[0:3, 3] = self._body_to_sensor
        result = np.dot(matrix, sensor)
        rotation = quaternion_from_matrix(result)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._common_frame
        transform.child_frame_id = self._expected_input_frame
        transform.transform.translation.x = result[0, 3]
        transform.transform.translation.y = result[1, 3]
        transform.transform.translation.z = result[2, 3]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        return transform

    def _timer_callback(self, _event):
        healthy = False
        if self._last_valid_stamp is not None:
            age = (rospy.Time.now() - self._last_valid_stamp).to_sec()
            healthy = -self._future_tolerance <= age <= self._cloud_timeout
            if not healthy:
                self._reason = "cloud_stream_stale"
        self._publish_status(healthy)

    def _publish_status(self, healthy):
        self._healthy_pub.publish(Bool(data=healthy))
        array = DiagnosticArray(header=Header(stamp=rospy.Time.now()))
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/pointcloud"
        status.hardware_id = "ego_sensing"
        status.level = (DiagnosticStatus.OK if healthy
                        else DiagnosticStatus.ERROR)
        status.message = "healthy" if healthy else "fail_closed"
        status.values = [
            KeyValue(key="reason", value=self._reason),
            KeyValue(key="common_frame", value=self._common_frame),
            KeyValue(key="expected_input_frame",
                     value=self._expected_input_frame or "any"),
            KeyValue(key="minimum_sensor_range",
                     value=str(self._minimum_sensor_range)),
            KeyValue(key="restamp_zero_timestamp",
                     value=str(self._restamp_zero_timestamp).lower()),
            KeyValue(key="transform_mode", value=(
                "calibrated_body_pose" if self._calibrated_body_transform
                else "tf2")),
        ]
        array.status = [status]
        self._diagnostics_pub.publish(array)


def main():
    rospy.init_node("ego_pointcloud_adapter")
    EgoPointCloudAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
