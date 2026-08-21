#!/usr/bin/env python3
"""ROS1 adapter for the PixEagle SmartTracker transplant."""

from __future__ import annotations

import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import cv2
import diagnostic_updater
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, ObjectHypothesisWithPose

from sar_yolo_detector.msg import (
    PerceptionIdentity,
    SmartTrackerState,
    TrackedDetection2D,
    TrackedDetection2DArray,
)
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
        self._subscriber = rospy.Subscriber(
            self._input_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
            tcp_nodelay=True,
        )

        self._updater = diagnostic_updater.Updater()
        self._updater.setHardwareID("sar_yolo_pixeagle_smart_tracker")
        self._updater.add("smart_tracker", self._diagnostics)
        self._diagnostic_timer = rospy.Timer(
            rospy.Duration(1.0), lambda _event: self._updater.update()
        )
        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "SmartTracker ready: model=%s device=%s topic=%s",
            runtime.get("model_path"),
            runtime.get("effective_device"),
            self._input_topic,
        )

    def close(self) -> None:
        with self._lock:
            tracker = getattr(self, "_tracker", None)
            if tracker is not None:
                tracker.close()
                self._tracker = None

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
            annotated = self._tracker.track_and_draw(frame.copy())
            self._last_processing_ms = (time.perf_counter() - started) * 1000.0
            output = self._tracker.get_output()
            detections = tuple(self._tracker.last_detections)
            self._last_header = message.header
            self._last_capture_stamp = message.header.stamp
            self._last_frame_wall = time.monotonic()
            self._processed_frames += 1
            frame_error_rate = _finite(output.quality_metrics.get("frame_error_rate"))
            self._last_error = (
                "inference failure" if frame_error_rate > 0 and not detections else ""
            )

            self._publish_detections(message, detections)
            self._publish_state(message, output)
            if self._publish_annotated and self._annotated_publisher.get_num_connections() > 0:
                self._annotated_publisher.publish(
                    _bgr_to_image(annotated, message.header)
                )
        self._updater.update()

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
            status.add(
                "tracking_active",
                bool(getattr(self._controller, "tracking_started", False)),
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
