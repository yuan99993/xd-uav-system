#!/usr/bin/env python3
"""ROS1 adapter for the PixEagle SmartTracker transplant."""

from __future__ import annotations

import math
import threading
import time
import uuid
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Iterable, Optional, Tuple

import cv2
import diagnostic_updater
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from vision_msgs.msg import BoundingBox2D, Detection2D, ObjectHypothesisWithPose

from sar_yolo_detector.msg import (
    PerceptionIdentity,
    SmartTrackerState,
    TrackedDetection2D,
    TrackedDetection2DArray,
)
from sar_yolo_detector.manual_box_tracker import ManualBoxTracker, ManualTrackResult
from sar_yolo_detector.pixeagle.parameters import Parameters
from sar_yolo_detector.pixeagle.smart_tracker import SmartTracker
from sar_yolo_detector.pixeagle.tracking_roi import (
    TrackingROIError,
    tracking_point_to_pixels,
    tracking_roi_to_pixels,
)
from sar_yolo_detector.srv import (
    SelectSmartTrack,
    SelectSmartTrackResponse,
    SwitchSmartTrackerModel,
    SwitchSmartTrackerModelResponse,
    SwitchTrackingMode,
    SwitchTrackingModeResponse,
)


class _VideoDimensions:
    width = 1
    height = 1


class _SmartTrackerController:
    """Small compatibility boundary replacing PixEagle's application controller."""

    def __init__(self) -> None:
        self.video_handler = _VideoDimensions()
        self.current_frame = None
        self.tracker = None
        self.tracking_started = False


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return default
    return numeric if math.isfinite(numeric) else default


def _image_to_bgr(message: Image) -> np.ndarray:
    """Decode common ROS image encodings without a Python cv_bridge ABI."""
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")

    encoding = str(message.encoding or "").strip().lower()
    color_encodings = {
        "bgr8": (3, None),
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
        "8uc1": (1, cv2.COLOR_GRAY2BGR),
    }
    if encoding in color_encodings:
        channels, conversion = color_encodings[encoding]
        minimum_step = width * channels
        if step < minimum_step:
            raise ValueError(
                f"image step {step} is smaller than {minimum_step} for {encoding}"
            )
        expected_size = height * step
        if len(message.data) < expected_size:
            raise ValueError("image data is shorter than height * step")
        rows = np.frombuffer(message.data, dtype=np.uint8, count=expected_size)
        rows = rows.reshape(height, step)
        image = rows[:, :minimum_step]
        image = image.reshape(height, width, channels) if channels > 1 else image.reshape(height, width)
        if conversion is not None:
            image = cv2.cvtColor(image, conversion)
        return np.ascontiguousarray(image)

    if encoding in {"mono16", "16uc1", "y16"}:
        minimum_step = width * 2
        if step < minimum_step or step % 2:
            raise ValueError("16-bit image step is invalid")
        expected_size = height * step
        if len(message.data) < expected_size:
            raise ValueError("image data is shorter than height * step")
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        rows = np.frombuffer(
            message.data, dtype=dtype, count=expected_size // 2
        ).reshape(height, step // 2)
        gray16 = rows[:, :width].astype(np.float32)
        low, high = np.percentile(gray16, (1.0, 99.0))
        if high <= low:
            low, high = float(gray16.min()), float(gray16.max())
        if high <= low:
            gray8 = np.zeros((height, width), dtype=np.uint8)
        else:
            gray8 = np.clip((gray16 - low) * (255.0 / (high - low)), 0, 255).astype(np.uint8)
        return cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)

    raise ValueError(
        "unsupported image encoding; expected bgr8/rgb8/bgra8/rgba8/mono8/mono16"
    )


def _bgr_to_image(frame: np.ndarray, header) -> Image:
    """Create one tightly packed bgr8 ROS image without cv_bridge."""
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("annotated image must have HxWx3 shape")
    contiguous = np.ascontiguousarray(frame, dtype=np.uint8)
    message = Image()
    message.header = header
    message.height = int(contiguous.shape[0])
    message.width = int(contiguous.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = int(contiguous.shape[1] * 3)
    message.data = contiguous.tobytes()
    return message


class SmartTrackerNode:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._package_root = Path(
            rospkg.RosPack().get_path("sar_yolo_detector")
        ).resolve()
        self._controller = _SmartTrackerController()
        self._last_header = None
        self._last_capture_stamp = rospy.Time(0)
        self._last_frame_wall = 0.0
        self._last_error = ""
        self._processed_frames = 0
        self._rejected_frames = 0
        self._last_processing_ms = 0.0

        self._input_topic = rospy.get_param("~input_image_topic", "camera/image_raw")
        tracked_topic = rospy.get_param(
            "~tracked_detections_topic",
            "sar_yolo_detector/smart/tracked_detections",
        )
        state_topic = rospy.get_param(
            "~state_topic", "sar_yolo_detector/smart/state"
        )
        annotated_topic = rospy.get_param(
            "~annotated_image_topic", "sar_yolo_detector/smart/annotated"
        )
        self._publish_annotated = bool(
            rospy.get_param("~publish_annotated_image", True)
        )
        self._maximum_age = max(
            0.0, float(rospy.get_param("~maximum_capture_age_sec", 0.5))
        )
        self._future_tolerance = max(
            0.0, float(rospy.get_param("~future_timestamp_tolerance_sec", 0.02))
        )
        self._reject_out_of_order = bool(
            rospy.get_param("~reject_out_of_order", True)
        )

        default_tracker = str(
            rospy.get_param("~Default_Tracker", "SmartTracker")
        ).strip().lower()
        default_aliases = {
            "smarttracker": "smart_tracker",
            "smart_tracker": "smart_tracker",
            "smart": "smart_tracker",
            "yolo": "smart_tracker",
            "featuretracker": "feature_tracker",
            "feature_tracker": "feature_tracker",
            "manualboxtracker": "feature_tracker",
            "manual_box_tracker": "feature_tracker",
            "manual": "feature_tracker",
            "colortracker": "color_tracker",
            "color_tracker": "color_tracker",
            "colour_tracker": "color_tracker",
        }
        if default_tracker not in default_aliases:
            raise RuntimeError(
                "Default_Tracker must be SmartTracker, FeatureTracker, or ColorTracker"
            )
        self._tracking_mode = default_aliases[default_tracker]
        manual_config = dict(rospy.get_param("~ManualBoxTracker", {}))
        self._manual_enabled = bool(manual_config.get("enabled", True))
        self._manual_trackers = {}
        if self._manual_enabled:
            for tracker_type in ("feature_tracker", "color_tracker"):
                tracker_config = dict(manual_config)
                tracker_config["tracker_type"] = tracker_type
                self._manual_trackers[tracker_type] = ManualBoxTracker(
                    tracker_config
                )
        elif self._tracking_mode != "smart_tracker":
            raise RuntimeError("A manual Default_Tracker requires ManualBoxTracker/enabled")
        self._manual_active = False
        self._manual_pending_roi = None
        self._manual_class_id = int(manual_config.get("class_id", 2))
        self._manual_track_id = int(manual_config.get("track_id", 900000000))
        self._manual_min_confidence = max(
            0.0, min(1.0, float(manual_config.get("minimum_confidence", 0.0)))
        )
        self._manual_initial_roi_topic = str(
            manual_config.get(
                "initial_roi_topic", "sar_yolo_detector/manual/initial_roi"
            )
        )

        smart_config = dict(rospy.get_param("~SmartTracker", {}))
        if not smart_config.get("SMART_TRACKER_ENABLED", True):
            raise RuntimeError("SmartTracker is disabled by configuration")
        smart_config["SMART_TRACKER_MODELS_ROOT"] = str(self._package_root)
        for key in (
            "SMART_TRACKER_GPU_MODEL_PATH",
            "SMART_TRACKER_CPU_MODEL_PATH",
        ):
            raw_path = str(smart_config.get(key, "")).strip()
            if raw_path and not Path(raw_path).expanduser().is_absolute():
                smart_config[key] = str((self._package_root / raw_path).resolve())
        Parameters.configure_smart_tracker(smart_config)

        self._controller.video_handler.width = 1
        self._controller.video_handler.height = 1
        self._tracker = SmartTracker(self._controller)
        runtime = self._tracker.get_runtime_info()

        self._identity = PerceptionIdentity()
        self._identity.mission_id = rospy.get_param("~mission_id", "mission_unset")
        self._identity.uav_id = rospy.get_param("~uav_id", "uav_unset")
        configured_session = str(rospy.get_param("~session_uuid", "")).strip()
        self._identity.session_uuid = (
            configured_session
            if configured_session and configured_session != "smart_tracker_unset"
            else f"smart_{uuid.uuid4()}"
        )
        self._identity.sensor_id = rospy.get_param("~sensor_id", "camera_primary")
        self._identity.profile = rospy.get_param("~profile", "generic")
        self._identity.model_version = rospy.get_param(
            "~model_version", runtime.get("model_name", "unknown")
        )
        self._identity.model_sha256 = str(runtime.get("artifact_sha256") or "")
        self._identity.calibration_version = rospy.get_param(
            "~calibration_version", "unversioned"
        )
        self._identity.coordinate_contract = rospy.get_param(
            "~coordinate_contract", "CAMERA_RAY"
        )

        self._tracked_publisher = rospy.Publisher(
            tracked_topic, TrackedDetection2DArray, queue_size=3
        )
        self._state_publisher = rospy.Publisher(
            state_topic, SmartTrackerState, queue_size=3
        )
        self._annotated_publisher = rospy.Publisher(
            annotated_topic, Image, queue_size=1
        )

        self._select_service = rospy.Service(
            "~select", SelectSmartTrack, self._select
        )
        self._switch_service = rospy.Service(
            "~switch_model", SwitchSmartTrackerModel, self._switch_model
        )
        self._tracking_mode_service = rospy.Service(
            "~switch_tracking_mode", SwitchTrackingMode, self._switch_tracking_mode
        )
        self._subscriber = rospy.Subscriber(
            self._input_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
            tcp_nodelay=True,
        )
        self._manual_roi_subscriber = None
        if self._manual_enabled:
            self._manual_roi_subscriber = rospy.Subscriber(
                self._manual_initial_roi_topic,
                RegionOfInterest,
                self._manual_roi_callback,
                queue_size=1,
            )

        self._updater = diagnostic_updater.Updater()
        self._updater.setHardwareID("sar_yolo_pixeagle_smart_tracker")
        self._updater.add("smart_tracker", self._diagnostics)
        self._diagnostic_timer = rospy.Timer(
            rospy.Duration(1.0), lambda _event: self._updater.update()
        )
        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "SmartTracker ready: model=%s device=%s topic=%s default=%s manual_roi=%s",
            runtime.get("model_path"),
            runtime.get("effective_device"),
            self._input_topic,
            self._tracking_mode,
            self._manual_initial_roi_topic if self._manual_enabled else "disabled",
        )

    def close(self) -> None:
        with self._lock:
            tracker = getattr(self, "_tracker", None)
            if tracker is not None:
                tracker.close()
                self._tracker = None
            for manual_tracker in self._manual_trackers.values():
                manual_tracker.reset()

    def _current_manual_tracker(self) -> Optional[ManualBoxTracker]:
        return self._manual_trackers.get(self._tracking_mode)

    def _manual_roi_callback(self, message: RegionOfInterest) -> None:
        """Initialize the already-selected manual mode, or clear its target."""
        with self._lock:
            manual_tracker = self._current_manual_tracker()
            if manual_tracker is None:
                rospy.logwarn_throttle(
                    2.0,
                    "Initial ROI ignored while mode=%s; call ~switch_tracking_mode first",
                    self._tracking_mode,
                )
                return
            if not manual_tracker.requires_initial_roi:
                rospy.logwarn_throttle(
                    2.0,
                    "Initial ROI ignored: %s detects configured colour '%s' in the full image",
                    self._tracking_mode,
                    manual_tracker.colour_name,
                )
                return
            if int(message.width) == 0 or int(message.height) == 0:
                manual_tracker.reset()
                self._manual_pending_roi = None
                self._manual_active = False
                rospy.loginfo(
                    "%s target cleared; waiting for a new initial ROI",
                    self._tracking_mode,
                )
                return
            self._manual_pending_roi = (
                float(message.x_offset),
                float(message.y_offset),
                float(message.width),
                float(message.height),
            )
            rospy.loginfo(
                "%s ROI queued: x=%d y=%d width=%d height=%d",
                self._tracking_mode,
                message.x_offset,
                message.y_offset,
                message.width,
                message.height,
            )

    def _valid_timestamp(self, message: Image) -> Tuple[bool, str]:
        stamp = message.header.stamp
        if stamp.is_zero():
            return False, "capture timestamp is zero"
        now = rospy.Time.now()
        if not now.is_zero():
            age = (now - stamp).to_sec()
            if age < -self._future_tolerance:
                return False, f"capture timestamp is {abs(age):.3f}s in the future"
            if self._maximum_age > 0 and age > self._maximum_age:
                return False, f"capture is stale by {age:.3f}s"
        if (
            self._reject_out_of_order
            and not self._last_capture_stamp.is_zero()
            and stamp <= self._last_capture_stamp
        ):
            return False, "capture timestamp is duplicate or out of order"
        return True, ""

    def _image_callback(self, message: Image) -> None:
        valid, reason = self._valid_timestamp(message)
        if not valid:
            self._rejected_frames += 1
            self._last_error = reason
            rospy.logwarn_throttle(2.0, "SmartTracker rejected frame: %s", reason)
            return

        try:
            frame = _image_to_bgr(message)
        except Exception as exc:
            self._rejected_frames += 1
            self._last_error = f"image conversion failed: {exc}"
            rospy.logerr_throttle(2.0, self._last_error)
            return

        with self._lock:
            if self._tracker is None:
                return
            self._controller.current_frame = frame
            self._controller.video_handler.height = int(frame.shape[0])
            self._controller.video_handler.width = int(frame.shape[1])
            started = time.perf_counter()
            manual_handled = False
            manual_tracker = self._current_manual_tracker()
            if manual_tracker is not None:
                pending_roi = self._manual_pending_roi
                self._manual_pending_roi = None
                result = (
                    manual_tracker.initialize(frame, pending_roi)
                    if pending_roi is not None
                    else manual_tracker.update(frame)
                )
                self._manual_active = manual_tracker.active
                manual_handled = True
                annotated = self._annotate_manual(frame.copy(), result)
                self._publish_manual_result(message, result)
                self._last_error = (
                    ""
                    if result.valid or result.reason == "waiting_for_initial_roi"
                    else result.reason
                )

            if not manual_handled:
                annotated = self._tracker.track_and_draw(frame.copy())
                output = self._tracker.get_output()
                detections = tuple(self._tracker.last_detections)
                frame_error_rate = _finite(
                    output.quality_metrics.get("frame_error_rate")
                )
                self._last_error = (
                    "inference failure"
                    if frame_error_rate > 0 and not detections
                    else ""
                )
                self._publish_detections(message, detections)
                self._publish_state(message, output)

            self._last_processing_ms = (time.perf_counter() - started) * 1000.0
            self._last_header = message.header
            self._last_capture_stamp = message.header.stamp
            self._last_frame_wall = time.monotonic()
            self._processed_frames += 1
            if self._publish_annotated and self._annotated_publisher.get_num_connections() > 0:
                self._annotated_publisher.publish(
                    _bgr_to_image(annotated, message.header)
                )
        self._updater.update()

    def _publish_manual_result(
        self, image: Image, result: ManualTrackResult
    ) -> None:
        measurement_valid = bool(
            result.valid
            and result.bbox is not None
            and result.confidence >= self._manual_min_confidence
        )
        detections = ()
        if measurement_valid:
            x, y, width, height = result.bbox
            detections = (
                SimpleNamespace(
                    aabb_xyxy=(x, y, x + width, y + height),
                    class_id=self._manual_class_id,
                    confidence=result.confidence,
                    track_id=self._manual_track_id,
                    track_id_is_stable=True,
                    rotation_deg=0.0,
                ),
            )
        self._publish_detections(image, detections)
        self._publish_manual_state(image, result, measurement_valid)

    def _publish_manual_state(
        self,
        image: Image,
        result: ManualTrackResult,
        measurement_valid: bool,
    ) -> None:
        state = SmartTrackerState()
        state.header = image.header
        state.tracking_active = self._tracking_mode != "smart_tracker"
        state.has_selection = bool(self._manual_active and result.bbox is not None)
        state.selected_track_id = self._manual_track_id if state.has_selection else -1
        state.selected_class_id = self._manual_class_id
        state.selected_class_name = "manual_roi"
        state.confidence = _finite(result.confidence)
        self._fill_bbox(state.selected_bbox, result.bbox)
        state.geometry_type = "aabb"
        state.has_oriented_bbox = False
        state.measurement_current = measurement_valid
        state.prediction_only = False
        manual_tracker = self._current_manual_tracker()
        state.tentative = bool(
            not measurement_valid
            and manual_tracker is not None
            and manual_tracker.active
        )
        state.data_is_stale = False
        state.control_measurement_ready = measurement_valid
        state.frames_since_detection = max(0, result.consecutive_failures)
        state.association_method = "manual_%s" % result.tracker_name
        state.freshness_reason = result.reason
        state.tracker_type = result.tracker_name
        state.backend = "opencv"
        state.device = "cpu"
        state.model_path = ""
        state.detection_count = 1 if measurement_valid else 0
        state.frame_processing_ms = _finite(self._last_processing_ms)
        self._state_publisher.publish(state)

    @staticmethod
    def _annotate_manual(frame: np.ndarray, result: ManualTrackResult) -> np.ndarray:
        if result.bbox is not None:
            x, y, width, height = result.bbox
            start = (int(round(x)), int(round(y)))
            end = (int(round(x + width)), int(round(y + height)))
            colour = (0, 255, 100) if result.valid else (0, 165, 255)
            cv2.rectangle(frame, start, end, colour, 2)
            cv2.putText(
                frame,
                "manual %s %.2f" % (result.tracker_name, result.confidence),
                (start[0], max(18, start[1] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                colour,
                1,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                frame,
                "manual tracker: waiting for ROI",
                (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
        return frame

    @staticmethod
    def _detection_message(header, detection) -> Detection2D:
        x1, y1, x2, y2 = detection.aabb_xyxy
        message = Detection2D()
        message.header = header
        message.bbox.center.x = (x1 + x2) / 2.0
        message.bbox.center.y = (y1 + y2) / 2.0
        message.bbox.center.theta = _finite(detection.rotation_deg)
        message.bbox.size_x = max(0.0, float(x2 - x1))
        message.bbox.size_y = max(0.0, float(y2 - y1))
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = int(detection.class_id)
        hypothesis.score = _finite(detection.confidence)
        message.results.append(hypothesis)
        return message

    def _publish_detections(self, image: Image, detections: Iterable[Any]) -> None:
        output = TrackedDetection2DArray()
        output.header = image.header
        output.provenance = self._identity
        for detection in detections:
            tracked = TrackedDetection2D()
            tracked.detection = self._detection_message(image.header, detection)
            tracked.track_id = int(detection.track_id)
            tracked.track_id_is_stable = bool(detection.track_id_is_stable)
            output.detections.append(tracked)
        self._tracked_publisher.publish(output)

    @staticmethod
    def _fill_bbox(message: BoundingBox2D, bbox: Optional[Tuple[int, int, int, int]]) -> None:
        if not bbox:
            return
        x, y, width, height = bbox
        message.center.x = x + width / 2.0
        message.center.y = y + height / 2.0
        message.size_x = max(0.0, float(width))
        message.size_y = max(0.0, float(height))

    def _publish_state(self, image: Image, output) -> None:
        raw = output.raw_data or {}
        tracking_result = raw.get("tracking_state_result") or {}
        state = SmartTrackerState()
        state.header = image.header
        state.tracking_active = bool(output.tracking_active)
        state.has_selection = output.target_id is not None
        state.selected_track_id = int(output.target_id if output.target_id is not None else -1)
        selected_class_id = raw.get("selected_class_id")
        state.selected_class_id = int(
            selected_class_id if selected_class_id is not None else -1
        )
        state.selected_class_name = str(
            self._tracker.labels.get(state.selected_class_id, "")
        )
        state.confidence = _finite(output.confidence)
        self._fill_bbox(state.selected_bbox, output.bbox)
        state.geometry_type = str(output.geometry_type or "none")
        state.has_oriented_bbox = output.oriented_bbox is not None
        if output.oriented_bbox is not None:
            state.oriented_bbox = [_finite(value) for value in output.oriented_bbox]
        state.measurement_current = bool(raw.get("selected_detected_this_frame", False))
        state.prediction_only = bool(raw.get("prediction_only", False))
        state.tentative = bool(raw.get("tentative", False))
        state.data_is_stale = bool(raw.get("data_is_stale", False))
        state.control_measurement_ready = bool(raw.get("usable_for_following", False))
        manager_info = self._tracker.tracking_manager.get_tracking_info()
        state.frames_since_detection = max(
            0, int(manager_info.get("frames_since_detection", 0) or 0)
        )
        if tracking_result.get("appearance_match"):
            state.association_method = "appearance"
        elif tracking_result.get("distance_match"):
            state.association_method = "distance"
        elif tracking_result.get("iou_match"):
            state.association_method = "spatial"
        elif state.prediction_only:
            state.association_method = "prediction"
        elif state.measurement_current:
            state.association_method = "id_or_measurement"
        else:
            state.association_method = "none"
        state.freshness_reason = str(raw.get("freshness_reason", "unknown"))
        state.tracker_type = str(raw.get("tracker_type", "unknown"))
        state.backend = str(raw.get("backend", "unknown"))
        state.device = str(raw.get("device", "unknown"))
        state.model_path = str(raw.get("model_path") or "")
        state.detection_count = len(output.targets or [])
        state.frame_processing_ms = _finite(self._last_processing_ms)
        self._state_publisher.publish(state)

    @staticmethod
    def _iou(box_a, box_b) -> float:
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b
        width = max(0, min(ax2, bx2) - max(ax1, bx1))
        height = max(0, min(ay2, by2) - max(ay1, by1))
        intersection = width * height
        union = max(1, (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - intersection)
        return intersection / float(union)

    def _select(self, request) -> SelectSmartTrackResponse:
        response = SelectSmartTrackResponse()
        with self._lock:
            if self._tracker is None:
                response.message = "SmartTracker is shutting down"
                return response
            if request.clear_selection:
                manual_tracker = self._current_manual_tracker()
                if manual_tracker is not None:
                    manual_tracker.reset()
                    self._manual_pending_roi = None
                    self._manual_active = False
                self._tracker.clear_selection()
                response.success = True
                response.message = "Selection cleared"
                response.selected_target_id = -1
                return response
            detections = list(self._tracker.last_detections)
            width = int(self._controller.video_handler.width)
            height = int(self._controller.video_handler.height)
            if not detections or width <= 1 or height <= 1:
                response.message = "No current detections are available"
                response.selected_target_id = -1
                return response
            try:
                if request.use_target_id:
                    matches = [d for d in detections if int(d.track_id) == request.target_id]
                    if not matches:
                        response.message = "Requested target ID is not in the current frame"
                        response.selected_target_id = -1
                        return response
                    point = matches[0].center_xy
                elif request.use_normalized_roi:
                    x, y, roi_width, roi_height = request.normalized_roi
                    roi = tracking_roi_to_pixels(
                        x=x,
                        y=y,
                        width=roi_width,
                        height=roi_height,
                        coordinate_space="normalized",
                        frame_width=width,
                        frame_height=height,
                    )
                    roi_xyxy = (
                        roi["x"], roi["y"],
                        roi["x"] + roi["width"], roi["y"] + roi["height"],
                    )
                    ranked = sorted(
                        ((self._iou(d.aabb_xyxy, roi_xyxy), d) for d in detections),
                        key=lambda item: item[0], reverse=True,
                    )
                    if not ranked or ranked[0][0] <= 0:
                        response.message = "Normalized ROI does not overlap a current detection"
                        response.selected_target_id = -1
                        return response
                    point = ranked[0][1].center_xy
                else:
                    point = tracking_point_to_pixels(
                        x=request.point_x,
                        y=request.point_y,
                        coordinate_space=("normalized" if request.use_normalized_point else "pixels"),
                        frame_width=width,
                        frame_height=height,
                    )
            except (TrackingROIError, TypeError, ValueError) as exc:
                response.message = str(exc)
                response.selected_target_id = -1
                return response

            response.success = bool(self._tracker.select_object_by_click(*point))
            response.tracking_active = bool(self._tracker.selected_object_id is not None)
            response.selected_target_id = int(
                self._tracker.selected_object_id
                if self._tracker.selected_object_id is not None else -1
            )
            response.measurement_current = bool(self._tracker._last_measurement_current)
            response.message = "Target selected" if response.success else "Selection did not match uniquely"
            return response

    @staticmethod
    def _normalize_tracking_mode(value: str) -> Optional[str]:
        aliases = {
            "smarttracker": "smart_tracker",
            "smart_tracker": "smart_tracker",
            "smart": "smart_tracker",
            "yolo": "smart_tracker",
            "featuretracker": "feature_tracker",
            "feature_tracker": "feature_tracker",
            "feature": "feature_tracker",
            "colortracker": "color_tracker",
            "color_tracker": "color_tracker",
            "colour_tracker": "color_tracker",
            "color": "color_tracker",
            "colour": "color_tracker",
        }
        return aliases.get(str(value or "").strip().lower())

    def _switch_tracking_mode(self, request) -> SwitchTrackingModeResponse:
        response = SwitchTrackingModeResponse()
        requested_mode = self._normalize_tracking_mode(request.mode)
        with self._lock:
            if self._tracker is None:
                response.message = "SmartTracker is shutting down"
                return response
            if requested_mode is None:
                response.message = (
                    "mode must be smart_tracker, feature_tracker, or color_tracker"
                )
                response.active_mode = self._tracking_mode
                return response
            if requested_mode != "smart_tracker" and not self._manual_enabled:
                response.message = "ManualBoxTracker is disabled"
                response.active_mode = self._tracking_mode
                return response

            color_tracker = self._manual_trackers.get("color_tracker")
            if requested_mode == "color_tracker" and color_tracker is not None:
                try:
                    color_tracker.configure_colour(request.color_name)
                except ValueError as exc:
                    response.message = str(exc)
                    response.active_mode = self._tracking_mode
                    response.color_source = color_tracker.colour_source
                    response.color_name = color_tracker.colour_name
                    return response

            for tracker in self._manual_trackers.values():
                tracker.reset()
            self._manual_pending_roi = None
            self._manual_active = False
            self._tracking_mode = requested_mode
            self._last_error = ""
            self._tracker.clear_selection()

            response.success = True
            response.active_mode = self._tracking_mode
            selected_manual_tracker = self._manual_trackers.get(requested_mode)
            response.waiting_for_initial_roi = bool(
                selected_manual_tracker is not None
                and selected_manual_tracker.requires_initial_roi
            )
            if color_tracker is not None:
                response.color_source = color_tracker.colour_source
                response.color_name = color_tracker.colour_name
            if requested_mode == "smart_tracker":
                response.message = "Switched to SmartTracker/YOLO"
            elif response.waiting_for_initial_roi:
                response.message = "Switched to %s; waiting for initial ROI" % requested_mode
            else:
                response.message = "Switched to color_tracker; detecting '%s' in the full image" % response.color_name
            rospy.loginfo("%s", response.message)
            return response

    def _switch_model(self, request) -> SwitchSmartTrackerModelResponse:
        response = SwitchSmartTrackerModelResponse()
        raw_path = Path(str(request.model_path or "")).expanduser()
        model_path = raw_path if raw_path.is_absolute() else self._package_root / raw_path
        with self._lock:
            if self._tracker is None:
                response.message = "SmartTracker is shutting down"
                return response
            try:
                if request.model_sha256:
                    self._tracker.backend.authorize_model_digest(
                        str(model_path), request.model_sha256
                    )
                result = self._tracker.switch_model(
                    str(model_path), request.device or "auto"
                )
            except Exception as exc:
                response.message = f"Model switch failed: {type(exc).__name__}: {exc}"
                return response
            model_info = result.get("model_info") or {}
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", "Model switch completed"))
            response.effective_device = str(model_info.get("device", "unknown"))
            response.effective_model_path = str(model_info.get("path", model_path))
            response.class_count = int(model_info.get("num_classes", 0) or 0)
            if response.success:
                runtime = self._tracker.get_runtime_info()
                self._identity.model_version = str(runtime.get("model_name", model_path.name))
                self._identity.model_sha256 = str(
                    runtime.get("artifact_sha256") or request.model_sha256
                )
            return response

    def _diagnostics(self, status):
        with self._lock:
            tracker = self._tracker
            runtime = tracker.get_runtime_info() if tracker is not None else {}
            frame_age = (
                time.monotonic() - self._last_frame_wall
                if self._last_frame_wall else float("inf")
            )
            if tracker is None:
                status.summary(diagnostic_updater.ERROR, "SmartTracker unavailable")
            elif self._last_error:
                status.summary(diagnostic_updater.WARN, self._last_error)
            elif not math.isfinite(frame_age) or frame_age > 2.0:
                status.summary(diagnostic_updater.WARN, "Waiting for a fresh image")
            else:
                status.summary(diagnostic_updater.OK, "SmartTracker operational")
            status.add("input_topic", self._input_topic)
            status.add("processed_frames", self._processed_frames)
            status.add("rejected_frames", self._rejected_frames)
            status.add("last_processing_ms", self._last_processing_ms)
            status.add("frame_age_sec", frame_age if math.isfinite(frame_age) else -1.0)
            status.add("model_path", runtime.get("model_path", ""))
            status.add("effective_device", runtime.get("effective_device", ""))
            status.add("backend", runtime.get("backend", ""))
            status.add("artifact_sha256", runtime.get("artifact_sha256", ""))
            status.add("active_source", self._tracking_mode)
            status.add("manual_enabled", self._manual_enabled)
            status.add("manual_active", self._manual_active)
            status.add("manual_initial_roi_topic", self._manual_initial_roi_topic)
            manual_tracker = self._current_manual_tracker()
            status.add(
                "manual_tracker_type",
                manual_tracker.tracker_type if manual_tracker is not None else "inactive",
            )
            color_tracker = self._manual_trackers.get("color_tracker")
            status.add(
                "color_source",
                color_tracker.colour_source if color_tracker is not None else "disabled",
            )
            status.add(
                "color_name",
                color_tracker.colour_name if color_tracker is not None else "disabled",
            )
            status.add(
                "tracking_active",
                self._manual_active
                if manual_tracker is not None
                else bool(getattr(self._controller, "tracking_started", False)),
            )
            return status


def main() -> None:
    rospy.init_node("sar_yolo_smart_tracker")
    try:
        SmartTrackerNode()
    except Exception as exc:
        rospy.logfatal("SmartTracker startup failed: %s: %s", type(exc).__name__, exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
