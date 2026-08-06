#!/usr/bin/env python3
"""
tracker_node.py — Tracker ROS Node

MRS-style ROS node that wraps TrackerCore for ROS-based target tracking.

Subscribes to:
  - /tracker/external_input (ExternalInput) — external bounding box / feature / ROI
  - /camera/image_raw (sensor_msgs/Image) — optional camera feed for visualization

Publishes:
  - /tracker/tracking_output (TrackingOutput) — unified tracker output
  - /tracker/normalized_error (NormalizedError) — normalized error for Follower

Services:
  - /tracker/start_tracking — start tracking a target
  - /tracker/stop_tracking — stop tracking

Parameters:
  - ~frame_width, ~frame_height
  - ~fov_horizontal, ~fov_vertical
  - ~enable_kalman_filter
  - ~tracking_strategy
  - ~target_size_ratio
  ...
"""

import rospy
import time
import threading
from typing import Optional, Tuple

from std_msgs.msg import Header
from tracker.msg import TrackingOutput, NormalizedError, ExternalInput

from tracker.tracker_core import TrackerCore, TrackerResult


class TrackerNode:
    """
    ROS node wrapping the TrackerCore tracking engine.

    Provides ROS interfaces for:
    - External input subscription
    - Tracking output publication
    - Normalized error publication for downstream Follower
    """

    def __init__(self):
        rospy.init_node('tracker_node', anonymous=False)
        rospy.logwarn(
            '[TrackerNode] Python reference implementation is deprecated for '
            'product use. Launch type="tracker_node" to run the C++ production '
            'implementation with multi-track, selection, CameraInfo/TF and diagnostics.')

        # ── Load parameters ──────────────────────────────────────────────
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        self.publish_rate = max(1.0, float(rospy.get_param('~publish_rate', 30.0)))
        self.tracker_name = rospy.get_param('~tracker_name', 'TrackerCore')
        self.direct_observation_hold_time = max(
            0.0, float(rospy.get_param('~direct_observation_hold_time', 0.15)))

        # Build config dict
        config = {
            'frame_width': self.frame_width,
            'frame_height': self.frame_height,
            'fov_horizontal': rospy.get_param('~fov_horizontal', 60.0),
            'fov_vertical': rospy.get_param('~fov_vertical', 45.0),
            'enable_kalman_filter': rospy.get_param('~enable_kalman_filter', True),
            'enable_motion_predictor': rospy.get_param('~enable_motion_predictor', True),
            'enable_prediction_buffer': rospy.get_param('~enable_prediction_buffer', True),
            'id_loss_tolerance_frames': rospy.get_param('~id_loss_tolerance_frames', 5),
            'spatial_iou_threshold': rospy.get_param('~spatial_iou_threshold', 0.35),
            'tracking_strategy': rospy.get_param('~tracking_strategy', 'hybrid'),
            'target_size_ratio': rospy.get_param('~target_size_ratio', 0.15),
            'kalman_process_noise': rospy.get_param('~kalman_process_noise', 1.0),
            'kalman_measurement_noise': rospy.get_param('~kalman_measurement_noise', 1.0),
            'velocity_alpha': rospy.get_param('~velocity_alpha', 0.7),
            'acceleration_alpha': rospy.get_param('~acceleration_alpha', 0.5),
            'mount_offset_yaw': rospy.get_param('~mount_offset_yaw', 0.0),
            'mount_offset_pitch': rospy.get_param('~mount_offset_pitch', 0.0),
            'mount_offset_roll': rospy.get_param('~mount_offset_roll', 0.0),
        }

        # ── Initialize TrackerCore ───────────────────────────────────────
        self.tracker = TrackerCore(config)
        self.auto_start_on_detection = rospy.get_param('~auto_start_on_detection', True)

        # One detection is consumed by one timer cycle. Reusing the final
        # box would incorrectly keep a missing target visible.
        self._pending_detection = None
        self._last_direct_detection_time = None
        self._last_direct_error = None
        self._input_lock = threading.Lock()

        # ── Publishers ───────────────────────────────────────────────────
        self.tracking_output_pub = rospy.Publisher(
            '~tracking_output', TrackingOutput, queue_size=10,
        )
        self.normalized_error_pub = rospy.Publisher(
            '~normalized_error', NormalizedError, queue_size=10,
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self.external_input_sub = rospy.Subscriber(
            '~external_input', ExternalInput,
            self._external_input_callback,
            queue_size=10,
        )

        # ── Timers ───────────────────────────────────────────────────────
        self._publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self._publish_callback,
        )

        rospy.loginfo(
            "[TrackerNode] Initialized: %dx%d, publish_rate=%.1f Hz, "
            "tracker=%s",
            self.frame_width, self.frame_height,
            self.publish_rate, self.tracker_name,
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _external_input_callback(self, msg: ExternalInput):
        """Normalize one external observation and queue one detection."""
        command = msg.command.strip().lower()
        source = msg.source.strip().lower()
        confidence = max(0.0, min(1.0, float(msg.confidence)))
        class_id = msg.class_id if msg.class_id >= 0 else None
        try:
            # stop_track and reset are control-only messages. They must not
            # require a bounding box, feature point, or ROI payload.
            if command in ('stop_track', 'reset'):
                self.tracker.process_external_bbox(command=command)
                with self._input_lock:
                    self._pending_detection = None
                    self._last_direct_detection_time = None
                    self._last_direct_error = None
                rospy.loginfo('[TrackerNode] External command accepted: %s', command)
                return

            effective_command = command
            if self.auto_start_on_detection and not command and not self.tracker.is_tracking():
                effective_command = 'start_track'
            if source in ('bounding_box', 'external_detector') and (msg.has_bbox or msg.has_normalized_bbox):
                inp = self.tracker.process_external_bbox(
                    bbox_pixel=tuple(msg.bbox) if msg.has_bbox else None,
                    normalized_bbox=tuple(msg.normalized_bbox) if msg.has_normalized_bbox else None,
                    confidence=confidence, class_id=class_id, command=effective_command)
            elif source == 'feature_point' and msg.has_feature_point:
                inp = self.tracker.process_external_feature(
                    point=tuple(msg.feature_point),
                    velocity=tuple(msg.feature_velocity) if msg.has_feature_velocity else None,
                    confidence=confidence, class_id=class_id, command=effective_command)
            elif source == 'roi' and msg.has_roi:
                inp = self.tracker.process_external_roi(
                    roi=tuple(msg.roi), confidence=confidence,
                    class_id=class_id, command=effective_command)
            else:
                rospy.logwarn_throttle(5.0, '[TrackerNode] Unsupported external input: %s', source)
                return
            if effective_command not in ('stop_track', 'reset') and inp.bbox_pixel is not None:
                with self._input_lock:
                    self._pending_detection = (inp.bbox_pixel, inp.confidence, inp.class_id)
                    self._last_direct_detection_time = time.monotonic()
        except (TypeError, ValueError) as exc:
            rospy.logwarn_throttle(2.0, '[TrackerNode] Rejected external input: %s', exc)
        except Exception as exc:
            rospy.logerr('[TrackerNode] Error processing external input: %s', exc)

    def _publish_callback(self, event):
        """Advance tracking and publish either a direct or predicted result."""
        try:
            with self._input_lock:
                pending = self._pending_detection
                self._pending_detection = None
            if pending is not None:
                bbox, confidence, class_id = pending
                x1, y1, x2, y2 = bbox
                track_id = class_id if class_id is not None else 0
                result = self.tracker.update_with_detections(
                    [[x1, y1, x2, y2, track_id, confidence, track_id, 1]], time.time())
            else:
                result = self.tracker.update_empty(time.time())
            self._publish_tracking_output(result)
            self._publish_normalized_error(result)
        except Exception as exc:
            rospy.logerr('[TrackerNode] Error in publish callback: %s', exc)

    def _publish_tracking_output(self, result: TrackerResult):
        """Convert TrackerResult to TrackingOutput ROS message and publish."""
        msg = TrackingOutput()
        msg.header = Header(stamp=rospy.Time.now())
        msg.tracking_active = result.tracking_active
        msg.target_id = result.target_id or -1
        msg.class_id = result.class_id or -1
        msg.confidence = result.confidence

        # Position 2D
        if result.center_normalized is not None:
            msg.position_2d = list(result.center_normalized)
            msg.has_position_2d = True

        # Bounding box
        if result.bbox is not None:
            msg.bbox = list(result.bbox)
            msg.has_bbox = True

        if result.bbox_normalized is not None:
            msg.normalized_bbox = list(result.bbox_normalized)
            msg.has_normalized_bbox = True

        # Geometry
        msg.geometry_type = result.geometry_type

        # Motion
        if result.velocity is not None:
            msg.velocity = list(result.velocity)
            msg.has_velocity = True

        if result.acceleration is not None:
            msg.acceleration = list(result.acceleration)
            msg.has_acceleration = True

        # Normalized error
        msg.error_x = result.error_x
        msg.error_y = result.error_y
        msg.error_size = result.error_size
        msg.has_normalized_error = True

        # Quality
        msg.tracking_quality = result.tracking_quality
        msg.frames_since_detection = result.frames_since_detection
        msg.is_predicted = result.is_predicted

        # Metadata
        msg.tracker_name = result.tracker_name
        msg.tracker_type = result.tracker_type

        self.tracking_output_pub.publish(msg)

    def _publish_normalized_error(self, result: TrackerResult):
        """Convert TrackerResult to NormalizedError ROS message and publish."""
        msg = NormalizedError()
        msg.header = Header(stamp=rospy.Time.now())

        direct_observation = result.tracking_active and not result.is_predicted
        with self._input_lock:
            last_detection_time = self._last_direct_detection_time
            if direct_observation:
                self._last_direct_error = (
                    result.error_x, result.error_y, result.error_size)
            last_direct_error = self._last_direct_error

        dt_since_detection = (
            0.0 if last_detection_time is None else
            min(float(time.monotonic() - last_detection_time), 1e6))
        error_is_fresh = (
            result.tracking_active and
            last_direct_error is not None and
            dt_since_detection <= self.direct_observation_hold_time)
        if error_is_fresh:
            msg.error_x, msg.error_y, msg.error_size = last_direct_error
        else:
            msg.error_x = result.error_x
            msg.error_y = result.error_y
            msg.error_size = result.error_size

        msg.error_valid = error_is_fresh
        msg.target_visible = error_is_fresh
        msg.is_estimated = not error_is_fresh and result.is_predicted
        msg.confidence = result.confidence
        msg.tracking_quality = result.tracking_quality
        msg.frames_since_detection = result.frames_since_detection
        msg.dt_since_detection = dt_since_detection

        self.normalized_error_pub.publish(msg)

    def run(self):
        """Main run loop."""
        rospy.loginfo("[TrackerNode] Running...")
        rospy.spin()


def main():
    """Entry point."""
    try:
        node = TrackerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[TrackerNode] Fatal error: %s", e)
        raise


if __name__ == '__main__':
    main()
