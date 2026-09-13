#!/usr/bin/env python3
import threading
import unittest
import math

import rospy
import rostest
from cv_bridge import CvBridge
from gazebo_msgs.msg import LinkStates, ModelState
from gazebo_msgs.srv import GetLinkState, SetModelState
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, Range
import tf2_ros
from tf.transformations import (concatenate_matrices, inverse_matrix,
                                quaternion_from_matrix, quaternion_matrix,
                                translation_from_matrix, translation_matrix)
from xd_uav_detect.msg import GimbalCommand, GimbalState, WorldDetectionArray
from xd_uav_track.msg import DetectionArray, DetectionCandidate


class GimbalRangeGazeboAcceptance(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._camera_info = None
        self._range = None
        self._seen_image = False
        self._metric = None
        self._world = None
        self._latest_metric = None
        self._latest_world = None
        self._gimbal_pose = None
        self._gimbal_state = None
        self._body_pose = None
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._publisher = rospy.Publisher(
            "/uav1/detect/input/detections_2d", DetectionArray, queue_size=2)
        self._gimbal_command_publisher = rospy.Publisher(
            "/uav1/gimbal/command", GimbalCommand, queue_size=2)
        rospy.Subscriber("/uav1/gimbal_camera/camera_info", CameraInfo,
                         self._camera_info_callback, queue_size=1)
        rospy.Subscriber("/uav1/gimbal/range", Range,
                         self._range_callback, queue_size=10)
        rospy.Subscriber("/uav1/gimbal_camera/image_raw", Image,
                         self._image_callback, queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates,
                         self._link_states_callback, queue_size=1)
        rospy.Subscriber("/uav1/detect/detections", DetectionArray,
                         self._metric_callback, queue_size=10)
        rospy.Subscriber("/uav1/detect/detections_world", WorldDetectionArray,
                         self._world_callback, queue_size=10)
        rospy.Subscriber("/uav1/gimbal/state", GimbalState,
                         self._gimbal_state_callback, queue_size=10)

    def _camera_info_callback(self, message):
        with self._lock:
            self._camera_info = message

    def _range_callback(self, message):
        with self._lock:
            self._range = message

    def _image_callback(self, message):
        with self._lock:
            ready = self._camera_info is not None and self._range is not None
            camera_info = self._camera_info
            measured_range = self._range.range if self._range else None
        if not ready:
            return
        self._publish_body_transform(message.header.stamp)
        image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if image.shape[:2] != (message.height, message.width):
            return
        fx = camera_info.P[0] if camera_info.P[0] > 0 else camera_info.K[0]
        fy = camera_info.P[5] if camera_info.P[5] > 0 else camera_info.K[4]
        cx = camera_info.P[2] if camera_info.P[0] > 0 else camera_info.K[2]
        cy = camera_info.P[6] if camera_info.P[5] > 0 else camera_info.K[5]
        half_width = fx / measured_range
        half_height = fy / measured_range
        detection = DetectionArray()
        detection.header = message.header
        detection.image_width = message.width
        detection.image_height = message.height
        detection.image_source = "gazebo_eo"
        detection.sensor_id = "gazebo_gimbal"
        detection.detector_name = "red_fixture"
        candidate = DetectionCandidate()
        candidate.track_id = 7
        candidate.class_id = 1
        candidate.confidence = 1.0
        candidate.has_bbox = True
        candidate.bbox = [int(cx - half_width), int(cy - half_height),
                          int(cx + half_width), int(cy + half_height)]
        detection.candidates = [candidate]
        with self._lock:
            self._seen_image = True
        self._publisher.publish(detection)

    def _link_states_callback(self, message):
        try:
            gimbal_index = message.name.index("x500_gimbal::gimbal_pitch")
            body_index = message.name.index("x500_gimbal::base_link")
        except ValueError:
            return
        with self._lock:
            self._gimbal_pose = message.pose[gimbal_index]
            self._body_pose = message.pose[body_index]

    def _metric_callback(self, message):
        with self._lock:
            self._latest_metric = message
        if message.candidates and message.candidates[0].range_valid:
            with self._lock:
                self._metric = message

    def _world_callback(self, message):
        with self._lock:
            self._latest_world = message
        if message.detections and message.detections[0].position_valid:
            with self._lock:
                self._world = message

    def _gimbal_state_callback(self, message):
        with self._lock:
            self._gimbal_state = message

    @staticmethod
    def _pose_matrix(pose):
        return concatenate_matrices(
            translation_matrix((pose.position.x, pose.position.y,
                                pose.position.z)),
            quaternion_matrix((pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w)))

    def _world_point_to_frd(self, body_pose, point):
        body_point = inverse_matrix(self._pose_matrix(body_pose)).dot(
            [point[0], point[1], point[2], 1.0])
        return (body_point[0], -body_point[1], -body_point[2])

    def _publish_body_transform(self, stamp):
        with self._lock:
            gimbal_pose = self._gimbal_pose
            body_pose = self._body_pose
        if gimbal_pose is None or body_pose is None:
            return
        world_from_body = TransformStamped()
        world_from_body.header.stamp = stamp
        world_from_body.header.frame_id = "map"
        world_from_body.child_frame_id = "uav1/base_link"
        world_from_body.transform.translation.x = body_pose.position.x
        world_from_body.transform.translation.y = body_pose.position.y
        world_from_body.transform.translation.z = body_pose.position.z
        world_from_body.transform.rotation = body_pose.orientation

        relative = concatenate_matrices(
            inverse_matrix(self._pose_matrix(body_pose)),
            self._pose_matrix(gimbal_pose))
        translation = translation_from_matrix(relative)
        rotation = quaternion_from_matrix(relative)
        body_from_laser = TransformStamped()
        body_from_laser.header.stamp = stamp
        body_from_laser.header.frame_id = "uav1/base_link"
        body_from_laser.child_frame_id = "uav1/gimbal_laser"
        body_from_laser.transform.translation.x = translation[0]
        body_from_laser.transform.translation.y = translation[1]
        body_from_laser.transform.translation.z = translation[2]
        body_from_laser.transform.rotation.x = rotation[0]
        body_from_laser.transform.rotation.y = rotation[1]
        body_from_laser.transform.rotation.z = rotation[2]
        body_from_laser.transform.rotation.w = rotation[3]
        self._tf_broadcaster.sendTransform(
            [world_from_body, body_from_laser])

    def _set_gimbal(self, yaw, pitch):
        command = GimbalCommand()
        command.header.stamp = rospy.Time.now()
        command.mode = GimbalCommand.POSITION
        command.yaw = yaw
        command.pitch = pitch
        command.yaw_rate = 1.0
        command.pitch_rate = 1.0
        self._gimbal_command_publisher.publish(command)
        self.assertTrue(self._wait_for(
            lambda: (self._gimbal_state is not None and
                     self._gimbal_state.joint_state_valid and
                     abs(self._gimbal_state.yaw - yaw) < 0.02 and
                     abs(self._gimbal_state.pitch - pitch) < 0.02),
            timeout=5.0), "gimbal command did not converge")
        rospy.wait_for_service("/gazebo/get_link_state", timeout=10.0)
        link_state = rospy.ServiceProxy(
            "/gazebo/get_link_state", GetLinkState)(
                "x500_gimbal::gimbal_pitch", "world")
        self.assertTrue(link_state.success, link_state.status_message)
        return link_state.link_state.pose

    def _set_target_on_axis(self, laser_pose, center_distance):
        q = laser_pose.orientation
        direction = (
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            2.0 * (q.x * q.y + q.z * q.w),
            2.0 * (q.x * q.z - q.y * q.w))
        state = ModelState()
        state.model_name = "red_target"
        state.reference_frame = "world"
        state.pose.position.x = (
            laser_pose.position.x + center_distance * direction[0])
        state.pose.position.y = (
            laser_pose.position.y + center_distance * direction[1])
        state.pose.position.z = (
            laser_pose.position.z + center_distance * direction[2])
        state.pose.orientation = q
        response = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)(state)
        self.assertTrue(response.success, response.status_message)
        return direction

    def _set_model(self, name, x, y, z, pitch=0.0, yaw=0.0):
        rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
        state = ModelState()
        state.model_name = name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        state.pose.orientation.x = -sp * sy
        state.pose.orientation.y = sp * cy
        state.pose.orientation.z = cp * sy
        state.pose.orientation.w = cp * cy
        response = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)(state)
        self.assertTrue(response.success, response.status_message)

    def _reset_results(self):
        with self._lock:
            self._metric = None
            self._world = None
            self._latest_metric = None
            self._latest_world = None

    def _wait_for(self, predicate, timeout=15.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not predicate():
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return predicate()

    def test_real_camera_ray_and_localization(self):
        self.assertTrue(self._wait_for(
            lambda: self._gimbal_pose is not None and
            self._body_pose is not None), "x500 gimbal links unavailable")
        laser_pose = self._set_gimbal(0.0, 0.0)
        direction = self._set_target_on_axis(laser_pose, 10.0)
        self._reset_results()
        deadline = rospy.Time.now() + rospy.Duration(30.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self._lock:
                complete = (self._seen_image and self._range is not None and
                            self._metric is not None and self._world is not None)
            if complete or rospy.Time.now() > deadline:
                break
            rate.sleep()
        with self._lock:
            measured_range = self._range
            metric = self._metric
            world = self._world
            seen_image = self._seen_image
            body_pose = self._body_pose
        self.assertTrue(seen_image, "Gazebo camera published no decodable image")
        self.assertIsNotNone(measured_range, "Gazebo ray published no Range")
        self.assertAlmostEqual(measured_range.range, 9.5, delta=0.10)
        self.assertIsNotNone(metric, "detect published no valid FRD position")
        endpoint = (laser_pose.position.x + measured_range.range * direction[0],
                    laser_pose.position.y + measured_range.range * direction[1],
                    laser_pose.position.z + measured_range.range * direction[2])
        expected_frd = self._world_point_to_frd(body_pose, endpoint)
        self.assertAlmostEqual(metric.candidates[0].relative_position_body[0],
                               expected_frd[0], delta=0.10)
        self.assertAlmostEqual(metric.candidates[0].relative_position_body[1],
                               expected_frd[1], delta=0.10)
        self.assertAlmostEqual(metric.candidates[0].relative_position_body[2],
                               expected_frd[2], delta=0.10)
        self.assertIsNotNone(world, "detect published no valid world position")
        self.assertEqual(world.header.frame_id, "map")
        self.assertEqual(world.detections[0].source_candidate_index, 0)
        self.assertEqual(world.detections[0].track_id, 7)
        self.assertEqual(world.detections[0].sensor_id, "gazebo_gimbal")
        self.assertAlmostEqual(world.detections[0].position_world.x, endpoint[0],
                               delta=0.10)
        self.assertAlmostEqual(world.detections[0].position_world.y, endpoint[1],
                               delta=0.10)
        self.assertAlmostEqual(world.detections[0].position_world.z, endpoint[2],
                               delta=0.10)

        # Rotate the physical Gazebo yaw/pitch joints. Capture-time transforms
        # come from the actual x500 base and gimbal link poses.
        yaw = 0.35
        pitch = -0.20
        with self._lock:
            self._pitch = pitch
            self._yaw = yaw
        laser_pose = self._set_gimbal(yaw, pitch)
        direction = self._set_target_on_axis(laser_pose, 10.0)
        self._reset_results()
        self.assertTrue(self._wait_for(
            lambda: self._metric is not None and self._world is not None),
            "no valid localization after yaw/pitch rotation")
        with self._lock:
            rotated_metric = self._metric
            rotated_world = self._world
            rotated_measured_range = self._range.range
            body_pose = self._body_pose
        rotated_position = (
            rotated_metric.candidates[0].relative_position_body)
        self.assertAlmostEqual(rotated_measured_range, 9.5, delta=0.10)
        rotated_endpoint = (
            laser_pose.position.x + rotated_measured_range * direction[0],
            laser_pose.position.y + rotated_measured_range * direction[1],
            laser_pose.position.z + rotated_measured_range * direction[2])
        expected_rotated_frd = self._world_point_to_frd(
            body_pose, rotated_endpoint)
        self.assertAlmostEqual(rotated_position[0],
                               expected_rotated_frd[0], delta=0.12)
        self.assertAlmostEqual(rotated_position[1],
                               expected_rotated_frd[1], delta=0.12)
        self.assertAlmostEqual(rotated_position[2],
                               expected_rotated_frd[2], delta=0.12)
        self.assertGreater(abs(rotated_position[1]), 1.0)
        self.assertGreater(abs(rotated_position[2]), 1.0)
        self.assertAlmostEqual(
            rotated_world.detections[0].position_world.x,
            rotated_endpoint[0], delta=0.12)
        self.assertAlmostEqual(
            rotated_world.detections[0].position_world.y,
            rotated_endpoint[1], delta=0.12)
        self.assertAlmostEqual(
            rotated_world.detections[0].position_world.z,
            rotated_endpoint[2], delta=0.12)

        # Move the target outside the ray. Gazebo reports the no-return
        # boundary and detect must forward the 2D candidate without metric
        # validity in either output.
        with self._lock:
            self._pitch = 0.0
            self._yaw = 0.0
        self._set_gimbal(0.0, 0.0)
        self._set_model("red_target", 10.0, 5.0, 1.0)
        self._reset_results()
        with self._lock:
            self._range = None
        self.assertTrue(self._wait_for(lambda: self._range is not None),
                        "Gazebo published no Range after target moved")
        with self._lock:
            no_return_range = self._range
        self.assertGreaterEqual(
            no_return_range.range, no_return_range.max_range - 0.002,
            "out-of-axis ray unexpectedly hit collision geometry")
        self.assertTrue(self._wait_for(
            lambda: (self._latest_metric is not None and
                     self._latest_metric.candidates and
                     not self._latest_metric.candidates[0].range_valid and
                     self._latest_world is not None)),
            "out-of-axis no-return did not produce fail-closed output: "
            "range={:.6f} max={:.6f}".format(
                no_return_range.range, no_return_range.max_range))
        with self._lock:
            invalid_metric = self._latest_metric
            invalid_world = self._latest_world
        self.assertFalse(
            invalid_metric.candidates[0].has_relative_position_body)
        self.assertFalse(invalid_world.detections[0].position_valid)


if __name__ == "__main__":
    rospy.init_node("gimbal_range_gazebo_acceptance")
    rostest.rosrun("xd_uav_detect", "gimbal_range_gazebo_acceptance",
                   GimbalRangeGazeboAcceptance)
