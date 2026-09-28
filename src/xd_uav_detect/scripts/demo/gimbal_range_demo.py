#!/usr/bin/env python3
"""Drive the Gazebo EO-payload demo without implementing product detection."""

import math
import threading

import rospy
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


class GimbalRangeDemo:
    def __init__(self):
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._camera_info = None
        self._laser_range = None
        self._gimbal_pose = None
        self._body_pose = None
        self._latest_metric = None
        self._latest_world = None
        self._gimbal_state = None
        self._model_name = rospy.get_param("~model_name", "x500_gimbal")
        self._gimbal_link = self._model_name + "::gimbal_pitch"
        self._body_link = self._model_name + "::base_link"
        self._airborne_min_z = float(
            rospy.get_param("~airborne_min_z", -1.0e9))
        self._ready_timeout = float(rospy.get_param("~ready_timeout", 20.0))
        self._loop = rospy.get_param("~loop", True)
        self._stage_duration = max(
            2.0, float(rospy.get_param("~stage_duration", 7.0)))

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._detections_publisher = rospy.Publisher(
            "/uav1/detect/input/detections_2d", DetectionArray,
            queue_size=2)
        self._gimbal_command_publisher = rospy.Publisher(
            "/uav1/gimbal/command", GimbalCommand, queue_size=2)
        rospy.Subscriber("/uav1/gimbal_camera/camera_info", CameraInfo,
                         self._camera_info_callback, queue_size=1)
        rospy.Subscriber("/uav1/gimbal_camera/image_raw", Image,
                         self._image_callback, queue_size=1)
        rospy.Subscriber("/uav1/gimbal/range", Range,
                         self._range_callback, queue_size=10)
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
            self._laser_range = message

    def _link_states_callback(self, message):
        try:
            gimbal_index = message.name.index(self._gimbal_link)
            body_index = message.name.index(self._body_link)
        except ValueError:
            return
        with self._lock:
            self._gimbal_pose = message.pose[gimbal_index]
            self._body_pose = message.pose[body_index]

    def _metric_callback(self, message):
        with self._lock:
            self._latest_metric = message
        if message.candidates:
            candidate = message.candidates[0]
            if candidate.range_valid:
                rospy.loginfo_throttle(
                    2.0, "[gimbal_demo] FRD = [%.2f, %.2f, %.2f] m",
                    *candidate.relative_position_body)
            else:
                rospy.loginfo_throttle(
                    2.0, "[gimbal_demo] FRD invalid (fail closed)")

    def _world_callback(self, message):
        with self._lock:
            self._latest_world = message
        if message.detections and message.detections[0].position_valid:
            point = message.detections[0].position_world
            rospy.loginfo_throttle(
                2.0, "[gimbal_demo] world = [%.2f, %.2f, %.2f] m",
                point.x, point.y, point.z)

    def _gimbal_state_callback(self, message):
        with self._lock:
            self._gimbal_state = message

    def _image_callback(self, message):
        if rospy.is_shutdown():
            return
        with self._lock:
            camera_info = self._camera_info
            laser_range = self._laser_range
            gimbal_pose = self._gimbal_pose
            body_pose = self._body_pose
        if (camera_info is None or laser_range is None or
                gimbal_pose is None or body_pose is None):
            return

        # Decode the real Gazebo image so the fixture fails visibly if the
        # camera transport is broken. Bounding boxes remain deterministic;
        # this demo is about localization, not product YOLO inference.
        image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if image.shape[:2] != (message.height, message.width):
            return
        try:
            self._publish_transforms(message.header.stamp, body_pose,
                                     gimbal_pose)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return
            raise

        fx = camera_info.P[0] if camera_info.P[0] > 0 else camera_info.K[0]
        fy = camera_info.P[5] if camera_info.P[5] > 0 else camera_info.K[4]
        cx = camera_info.P[2] if camera_info.P[0] > 0 else camera_info.K[2]
        cy = camera_info.P[6] if camera_info.P[5] > 0 else camera_info.K[5]
        box_range = laser_range.range
        if not math.isfinite(box_range) or box_range <= 0.2:
            box_range = 10.0
        box_range = min(box_range, 30.0)
        half_width = max(6.0, fx / box_range)
        half_height = max(6.0, fy / box_range)

        detections = DetectionArray()
        detections.header = message.header
        detections.image_width = message.width
        detections.image_height = message.height
        detections.image_source = "gazebo_eo"
        detections.sensor_id = "gazebo_gimbal"
        detections.detector_name = "deterministic_demo_fixture"
        detections.model_version = "demo-v1"
        candidate = DetectionCandidate()
        candidate.track_id = 7
        candidate.class_id = 1
        candidate.confidence = 1.0
        candidate.has_bbox = True
        candidate.bbox = [int(cx - half_width), int(cy - half_height),
                          int(cx + half_width), int(cy + half_height)]
        detections.candidates = [candidate]
        try:
            self._detections_publisher.publish(detections)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    @staticmethod
    def _pose_matrix(pose):
        return concatenate_matrices(
            translation_matrix((pose.position.x, pose.position.y,
                                pose.position.z)),
            quaternion_matrix((pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w)))

    def _publish_transforms(self, stamp, body_pose, gimbal_pose):
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
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while not rospy.is_shutdown():
            with self._lock:
                state = self._gimbal_state
            if (state is not None and state.joint_state_valid and
                    abs(state.yaw - yaw) < 0.02 and
                    abs(state.pitch - pitch) < 0.02):
                break
            if rospy.Time.now() > deadline:
                raise RuntimeError("gimbal command convergence timeout")
            rospy.sleep(0.02)
        response = rospy.ServiceProxy(
            "/gazebo/get_link_state", GetLinkState)(
                self._gimbal_link, "world")
        if not response.success:
            raise RuntimeError(response.status_message)
        return response.link_state.pose

    @staticmethod
    def _beam_direction(pose):
        q = pose.orientation
        return (1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                2.0 * (q.x * q.y + q.z * q.w),
                2.0 * (q.x * q.z - q.y * q.w))

    def _place_target_on_beam(self, laser_pose, distance=10.0):
        direction = self._beam_direction(laser_pose)
        state = ModelState()
        state.model_name = "red_target"
        state.reference_frame = "world"
        state.pose.position.x = laser_pose.position.x + distance * direction[0]
        state.pose.position.y = laser_pose.position.y + distance * direction[1]
        state.pose.position.z = laser_pose.position.z + distance * direction[2]
        state.pose.orientation = laser_pose.orientation
        self._set_model_state(state)

    def _place_target_off_beam(self):
        state = ModelState()
        state.model_name = "red_target"
        state.reference_frame = "world"
        state.pose.position.x = 10.0
        state.pose.position.y = 5.0
        state.pose.position.z = 1.0
        state.pose.orientation.w = 1.0
        self._set_model_state(state)

    @staticmethod
    def _set_model_state(state):
        response = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)(state)
        if not response.success:
            raise RuntimeError(response.status_message)

    def _wait_ready(self):
        for service in ("/gazebo/get_link_state",
                        "/gazebo/set_model_state"):
            rospy.wait_for_service(service, timeout=20.0)
        deadline = rospy.Time.now() + rospy.Duration(self._ready_timeout)
        while not rospy.is_shutdown():
            with self._lock:
                ready = (self._camera_info is not None and
                         self._laser_range is not None and
                         self._gimbal_pose is not None and
                         self._gimbal_state is not None and
                         self._gimbal_state.joint_state_valid and
                         self._body_pose is not None and
                         self._body_pose.position.z >= self._airborne_min_z)
            if ready:
                return
            if rospy.Time.now() > deadline:
                raise RuntimeError(
                    "Gazebo camera/range/gimbal/airborne state timeout")
            rospy.sleep(0.05)

    def _run_stage(self, name, yaw, pitch, target_on_beam):
        laser_pose = self._set_gimbal(yaw, pitch)
        if target_on_beam:
            self._place_target_on_beam(laser_pose)
        else:
            self._place_target_off_beam()
        rospy.loginfo("[gimbal_demo] stage=%s yaw=%.2f pitch=%.2f; hold %.1fs",
                      name, yaw, pitch, self._stage_duration)
        rospy.sleep(self._stage_duration)

    def run(self):
        self._wait_ready()
        rospy.loginfo("[gimbal_demo] ready; watch Gazebo, /uav1/gimbal/range, "
                      "/uav1/detect/detections and detections_world")
        while not rospy.is_shutdown():
            self._run_stage("forward_hit", 0.0, 0.0, True)
            self._run_stage("yaw_pitch_hit", 0.55, -0.22, True)
            self._run_stage("no_return_fail_closed", 0.0, 0.0, False)
            if not self._loop:
                rospy.loginfo("[gimbal_demo] sequence complete; holding")
                rospy.spin()
                return


if __name__ == "__main__":
    rospy.init_node("gimbal_range_demo")
    try:
        GimbalRangeDemo().run()
    except rospy.ROSInterruptException:
        pass
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        rospy.logfatal("[gimbal_demo] %s", error)
        raise
