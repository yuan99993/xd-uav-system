#!/usr/bin/env python3
"""Transform a physical sensor PointCloud2 into EGO's common world frame."""

import copy

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
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
            rospy.get_param("~minimum_sensor_range", 0.35))
        self._restamp_zero_timestamp = bool(
            rospy.get_param("~restamp_zero_timestamp", False))
        self._calibrated_body_transform = bool(
            rospy.get_param("~calibrated_body_transform", False))
        self._body_frame = rospy.get_param("~body_frame", "").strip("/")
        self._body_to_sensor = tuple(float(value) for value in rospy.get_param(
            "~body_to_sensor_translation_m", [0.0, 0.0, 0.0]))
        self._self_filter_enabled = bool(
            rospy.get_param("~self_filter_enabled", True))
        self._self_filter_half_x = max(
            0.0, float(rospy.get_param("~self_filter_half_x", 0.32)))
        self._self_filter_half_y = max(
            0.0, float(rospy.get_param("~self_filter_half_y", 0.32)))
        self._self_filter_min_z = float(
            rospy.get_param("~self_filter_min_z", -0.20))
        self._self_filter_max_z = float(
            rospy.get_param("~self_filter_max_z", 0.25))
        if self._self_filter_max_z < self._self_filter_min_z:
            self._self_filter_min_z, self._self_filter_max_z = (
                self._self_filter_max_z, self._self_filter_min_z)
        self._state_timeout = float(rospy.get_param("~state_timeout", 0.10))
        if self._calibrated_body_transform:
            if not self._body_frame or len(self._body_to_sensor) != 3:
                raise rospy.ROSInitException(
                    "calibrated transform requires body_frame and 3-D extrinsic")
        self._last_state = None
        self._reason = "cloud_not_received"
        self._last_valid_stamp = None
        self._last_self_filtered_points = 0

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
            sensor_points = self._xyz_array(message)
            finite = np.all(np.isfinite(sensor_points), axis=1)
            sensor_points = sensor_points[finite]
            common_from_sensor = self._transform_matrix(transform)
            common_points = np.dot(
                sensor_points, common_from_sensor[0:3, 0:3].T)
            common_points += common_from_sensor[0:3, 3]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError) as error:
            self._reason = "cloud_transform_unavailable: {}".format(error)
            self._last_valid_stamp = None
            self._publish_status(False)
            return
        origin = common_from_sensor[0:3, 3]
        body_from_common = None
        if self._self_filter_enabled and self._body_frame:
            try:
                body_transform = self._buffer.lookup_transform(
                    self._body_frame, self._common_frame,
                    message.header.stamp,
                    rospy.Duration(self._transform_timeout))
                body_from_common = self._transform_matrix(body_transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                # Self filtering is supplemental.  Do not discard an
                # otherwise valid obstacle cloud just because the body TF is
                # temporarily unavailable; the normal sensor-range filter
                # still remains active and the diagnostic reports the issue.
                self._reason = "self_filter_transform_unavailable: {}".format(error)
                rospy.logwarn_throttle(
                    1.0, "EGO pointcloud self-filter disabled for this scan: %s",
                    error)

        if common_points.size:
            offset = common_points - origin
            keep = np.einsum("ij,ij->i", offset, offset) >= (
                self._minimum_sensor_range ** 2)
        else:
            keep = np.zeros((0,), dtype=bool)

        self_filtered = 0
        # Remove returns lying inside the configured vehicle envelope. This
        # remains a geometric self-return filter, but all points are handled
        # in one vectorized operation instead of one Python matrix multiply
        # per return.
        if body_from_common is not None and common_points.size:
            body_points = np.dot(
                common_points, body_from_common[0:3, 0:3].T)
            body_points += body_from_common[0:3, 3]
            inside_body = (
                (np.abs(body_points[:, 0]) <= self._self_filter_half_x) &
                (np.abs(body_points[:, 1]) <= self._self_filter_half_y) &
                (body_points[:, 2] >= self._self_filter_min_z) &
                (body_points[:, 2] <= self._self_filter_max_z))
            self_filtered = int(np.count_nonzero(keep & inside_body))
            keep &= ~inside_body
        finite_points = common_points[keep]

        self._last_self_filtered_points = self_filtered

        if finite_points.shape[0] == 0:
            self._reason = "cloud_no_finite_xyz"
            self._last_valid_stamp = None
            self._publish_status(False)
            return
        header = Header(stamp=message.header.stamp,
                        frame_id=self._common_frame)
        self._cloud_pub.publish(self._create_xyz_cloud(header, finite_points))
        self._last_valid_stamp = message.header.stamp
        self._reason = "ok"
        self._publish_status(True)

    @staticmethod
    def _transform_matrix(transform):
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([
            rotation.x, rotation.y, rotation.z, rotation.w])
        translation = transform.transform.translation
        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z
        return matrix

    @staticmethod
    def _xyz_array(message):
        """Expose PointCloud2 XYZ fields as an array without per-point loops."""
        fields = {field.name: field for field in message.fields}
        formats = {
            PointField.INT8: "i1", PointField.UINT8: "u1",
            PointField.INT16: "i2", PointField.UINT16: "u2",
            PointField.INT32: "i4", PointField.UINT32: "u4",
            PointField.FLOAT32: "f4", PointField.FLOAT64: "f8",
        }
        selected = []
        for name in ("x", "y", "z"):
            field = fields.get(name)
            if (field is None or field.count != 1 or
                    field.datatype not in formats):
                raise ValueError("PointCloud2 requires scalar numeric {} field".format(
                    name))
            byte_order = ">" if message.is_bigendian else "<"
            selected.append((name, byte_order + formats[field.datatype],
                             int(field.offset)))
        if message.point_step <= 0 or message.row_step <= 0:
            raise ValueError("PointCloud2 has invalid point/row step")
        dtype = np.dtype({
            "names": [entry[0] for entry in selected],
            "formats": [entry[1] for entry in selected],
            "offsets": [entry[2] for entry in selected],
            "itemsize": int(message.point_step),
        })
        try:
            view = np.ndarray(
                shape=(int(message.height), int(message.width)), dtype=dtype,
                buffer=message.data,
                strides=(int(message.row_step), int(message.point_step)))
        except (TypeError, ValueError) as error:
            raise ValueError("PointCloud2 data layout invalid: {}".format(error))
        return np.column_stack([
            view[name].reshape(-1) for name in ("x", "y", "z")
        ]).astype(np.float64, copy=False)

    @staticmethod
    def _create_xyz_cloud(header, points):
        xyz = np.ascontiguousarray(points, dtype=np.float32)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = int(xyz.shape[0])
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32,
                       count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = xyz.tobytes(order="C")
        return cloud

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
            KeyValue(key="self_filter_enabled",
                     value=str(self._self_filter_enabled).lower()),
            KeyValue(key="self_filtered_points",
                     value=str(self._last_self_filtered_points)),
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
