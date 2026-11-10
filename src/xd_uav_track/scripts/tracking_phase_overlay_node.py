#!/usr/bin/env python3
"""Overlay tracking phase state on the SAR annotated image."""

from __future__ import annotations

import math
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from xd_uav_track.msg import DetectionArray, GimbalState, TrackStatus


GIMBAL_PROFILES = {
    "gm_velocity_chase",
    "gm_velocity_vector",
    "gmfw_velocity_vector",
}


class TrackingPhaseOverlay:
    def __init__(self) -> None:
        # track.launch loads track.yaml in the sibling /<uav>/track namespace.
        # Read phase-overlay settings from there so all runtime tuning lives
        # beside the tracker config instead of being repeated in launch args.
        config_ns = rospy.resolve_name("track/phase_overlay")

        def config_param(key, default):
            return rospy.get_param(config_ns + "/" + key, default)

        if not bool(config_param("enabled", True)):
            rospy.loginfo("tracking phase overlay disabled by track config")
            return

        self._input_image_topic = str(config_param(
            "input_image_topic", "sar_yolo_detector/smart/annotated"))
        self._input_detections_topic = str(config_param(
            "input_detections_topic", "track/selected_detections"))
        self._status_topic = str(config_param("status_topic", "track/status"))
        self._gimbal_state_topic = str(config_param(
            "gimbal_state_topic", "track/gimbal_state"))
        self._output_image_topic = str(config_param(
            "output_image_topic", "track/phase_annotated"))
        self._detection_timeout_sec = max(0.1, float(config_param(
            "detection_timeout_sec", 0.5)))
        self._status_timeout_sec = max(0.1, float(config_param(
            "status_timeout_sec", 0.5)))
        self._gimbal_state_timeout_sec = max(0.1, float(config_param(
            "gimbal_state_timeout_sec", 0.5)))
        self._minimum_tracking_duration_sec = max(0.0, float(config_param(
            "attacking_min_tracking_duration_sec", 1.0)))
        self._attack_enter_error = max(0.0, float(config_param(
            "attacking_error_threshold", 0.10)))
        self._attack_exit_error = max(self._attack_enter_error, float(
            config_param("attacking_error_exit_threshold",
                         self._attack_enter_error + 0.05)))
        self._gimbal_attack_enter_yaw = max(0.0, float(config_param(
            "attacking_gimbal_yaw_threshold_rad", 0.08)))
        self._gimbal_attack_exit_yaw = max(
            self._gimbal_attack_enter_yaw,
            float(config_param("attacking_gimbal_yaw_exit_threshold_rad",
                               self._gimbal_attack_enter_yaw + 0.10)))

        gimbal_config_ns = rospy.resolve_name("track/follower/gimbal")
        self._gimbal_yaw_inverted = bool(rospy.get_param(
            gimbal_config_ns + "/invert_yaw", False))
        self._gimbal_yaw_offset = float(rospy.get_param(
            gimbal_config_ns + "/yaw_offset", 0.0))
        self._gimbal_filter_previous_weight = min(0.99, max(0.0, float(
            rospy.get_param(gimbal_config_ns + "/angle_filter_previous_weight",
                            0.70))))
        self._gimbal_angle_deadzone = max(0.0, float(rospy.get_param(
            gimbal_config_ns + "/angle_deadzone", 0.035)))

        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._has_detection = False
        self._detection_received_at = 0.0
        self._tracker_active = False
        self._follower_profile = ""
        self._target_visible = False
        self._command_valid = False
        self._status_gimbal_valid = False
        self._error_x = 0.0
        self._error_y = 0.0
        self._gimbal_yaw = 0.0
        self._filtered_gimbal_yaw = 0.0
        self._gimbal_received_at = 0.0
        self._gimbal_valid = False
        self._gimbal_filter_initialized = False
        self._tracking_phase_started_at = 0.0
        self._status_received_at = 0.0
        self._attacking = False

        self._publisher = rospy.Publisher(
            self._output_image_topic, Image, queue_size=1, tcp_nodelay=True)
        self._image_subscriber = rospy.Subscriber(
            self._input_image_topic, Image, self._image_callback,
            queue_size=1, buff_size=16 * 1024 * 1024, tcp_nodelay=True)
        self._detection_subscriber = rospy.Subscriber(
            self._input_detections_topic, DetectionArray,
            self._detection_callback, queue_size=1, tcp_nodelay=True)
        self._status_subscriber = rospy.Subscriber(
            self._status_topic, TrackStatus, self._status_callback,
            queue_size=1, tcp_nodelay=True)
        self._gimbal_subscriber = rospy.Subscriber(
            self._gimbal_state_topic, GimbalState,
            self._gimbal_callback, queue_size=1, tcp_nodelay=True)
        rospy.loginfo(
            "tracking phase overlay ready: image=%s detections=%s status=%s "
            "gimbal=%s output=%s image_attack_error<=%.3f "
            "gimbal_attack_yaw<=%.3f rad (exit %.3f rad)",
            self._input_image_topic, self._input_detections_topic,
            self._status_topic, self._gimbal_state_topic,
            self._output_image_topic, self._attack_enter_error,
            self._gimbal_attack_enter_yaw, self._gimbal_attack_exit_yaw)

    @staticmethod
    def _has_valid_candidate(message: DetectionArray) -> bool:
        for candidate in message.candidates:
            if not candidate.has_bbox:
                continue
            x1, y1, x2, y2 = candidate.bbox
            if x2 > x1 and y2 > y1:
                return True
        return False

    def _detection_callback(self, message: DetectionArray) -> None:
        with self._lock:
            self._has_detection = self._has_valid_candidate(message)
            self._detection_received_at = time.monotonic()

    def _status_callback(self, message: TrackStatus) -> None:
        with self._lock:
            self._tracker_active = bool(message.tracker_active)
            profile = str(message.follower_profile)
            if profile != self._follower_profile:
                self._attacking = False
                self._tracking_phase_started_at = 0.0
            self._follower_profile = profile
            self._target_visible = bool(message.target_visible)
            self._command_valid = bool(message.command_valid)
            self._status_gimbal_valid = bool(message.gimbal_state_valid)
            self._error_x = float(message.error_x)
            self._error_y = float(message.error_y)
            self._status_received_at = time.monotonic()

    def _gimbal_callback(self, message: GimbalState) -> None:
        received_at = time.monotonic()
        with self._lock:
            self._gimbal_received_at = received_at
            self._gimbal_valid = bool(message.valid)
            if not self._gimbal_valid:
                self._gimbal_filter_initialized = False
                return

            yaw = float(message.yaw_rad)
            if not math.isfinite(yaw):
                self._gimbal_valid = False
                self._gimbal_filter_initialized = False
                return
            if self._gimbal_yaw_inverted:
                yaw = -yaw
            if not self._gimbal_filter_initialized:
                filtered_yaw = yaw
                self._gimbal_filter_initialized = True
            else:
                previous = self._gimbal_filter_previous_weight
                filtered_yaw = (previous * self._filtered_gimbal_yaw +
                                (1.0 - previous) * yaw)
            self._filtered_gimbal_yaw = filtered_yaw
            self._gimbal_yaw = filtered_yaw + self._gimbal_yaw_offset
            if abs(self._gimbal_yaw) < self._gimbal_angle_deadzone:
                self._gimbal_yaw = 0.0

    def _phase(self, now: float) -> str:
        with self._lock:
            has_detection = (
                self._has_detection and
                now - self._detection_received_at <= self._detection_timeout_sec)
            if not has_detection:
                self._attacking = False
                self._tracking_phase_started_at = 0.0
                return "UNIDENTIFICATION"

            status_fresh = (
                self._status_received_at > 0.0 and
                now - self._status_received_at <= self._status_timeout_sec)
            if not status_fresh:
                self._attacking = False
                self._tracking_phase_started_at = 0.0
                return "LOCKING"
            tracker_active = self._tracker_active

            if not tracker_active:
                self._attacking = False
                self._tracking_phase_started_at = 0.0
                return "LOCKING"

            if not self._target_visible or not self._command_valid:
                self._attacking = False
                self._tracking_phase_started_at = 0.0
                return "TRACKING"

            if self._tracking_phase_started_at <= 0.0:
                self._tracking_phase_started_at = now
            if now - self._tracking_phase_started_at < \
                    self._minimum_tracking_duration_sec:
                self._attacking = False
                return "TRACKING"

            if self._follower_profile in GIMBAL_PROFILES:
                gimbal_fresh = (
                    self._status_gimbal_valid and self._gimbal_valid and
                    self._gimbal_received_at > 0.0 and
                    now - self._gimbal_received_at <=
                    self._gimbal_state_timeout_sec)
                if not gimbal_fresh:
                    self._attacking = False
                    self._tracking_phase_started_at = 0.0
                    return "TRACKING"
                # GimbalState yaw is the camera's heading relative to the
                # aircraft body. In the GM profiles this is the aircraft to
                # target heading alignment cue; image-centre error is not used.
                error = abs(self._gimbal_yaw)
                enter_threshold = self._gimbal_attack_enter_yaw
                exit_threshold = self._gimbal_attack_exit_yaw
            else:
                error = max(abs(self._error_x), abs(self._error_y))
                enter_threshold = self._attack_enter_error
                exit_threshold = self._attack_exit_error
            if self._attacking:
                if error > exit_threshold:
                    self._attacking = False
            elif error <= enter_threshold:
                self._attacking = True
            return "ATTACKING" if self._attacking else "TRACKING"

    @staticmethod
    def _draw_phase(frame, phase: str) -> None:
        colors = {
            "UNIDENTIFICATION": (96, 96, 96),
            "LOCKING": (0, 165, 255),
            "TRACKING": (0, 210, 0),
            "ATTACKING": (0, 0, 255),
        }
        color = colors[phase]
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.9
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            phase, font, scale, thickness)
        padding = 12
        right = padding * 2 + text_width
        bottom = padding * 2 + text_height + baseline
        cv2.rectangle(frame, (8, 8), (8 + right, 8 + bottom), color, -1)
        cv2.putText(frame, phase, (8 + padding, 8 + padding + text_height),
                    font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

    def _image_callback(self, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(
                5.0, "tracking phase overlay cannot decode image: %s", error)
            return

        phase = self._phase(time.monotonic())
        self._draw_phase(frame, phase)
        try:
            output = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(
                5.0, "tracking phase overlay cannot encode image: %s", error)
            return
        output.header = message.header
        self._publisher.publish(output)


def main() -> None:
    rospy.init_node("tracking_phase_overlay")
    TrackingPhaseOverlay()
    rospy.spin()


if __name__ == "__main__":
    main()
