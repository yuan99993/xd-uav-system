#!/usr/bin/env python3
"""ROS1 adapter for the PixEagle SmartTracker transplant."""

from __future__ import annotations

import math
import threading
import time
import uuid
import json
import copy
import traceback
from collections import Counter, deque
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Iterable, Optional, Tuple

import cv2
import diagnostic_updater
import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D, Detection2D, ObjectHypothesisWithPose

from sar_yolo_detector.msg import (
    PerceptionIdentity,
    SmartTrackerState,
    TrackedDetection2D,
    TrackedDetection2DArray,
    FusionDetection2D,
    FusionDetection2DArray,
)
from sar_yolo_detector.manual_box_tracker import ManualBoxTracker, ManualTrackResult
from sar_yolo_detector.box_fusion import BoxFusion, Candidate, class_duplicate_indices, iou, valid_box
from sar_yolo_detector.async_pipeline import LatestMailbox, InferencePacket, FlowDetections
from sar_yolo_detector.camera_motion import CameraMotion, quaternion_rotation, rotation_homography
from sar_yolo_detector.capture_clock import CaptureClock
from sar_yolo_detector.loss_journal import LossJournal
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
    SelectFusionROI,
    SelectFusionROIResponse,
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
    message.header = copy.copy(header)
    message.height = int(contiguous.shape[0])
    message.width = int(contiguous.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = int(contiguous.shape[1] * 3)
    message.data = contiguous.tobytes()
    return message


class SmartTrackerNode:
    def __init__(self) -> None:
        # OpenCV otherwise may create a large CPU pool on WSL and contend with
        # CUDA/ROS/Gazebo. Keep the local-image budget explicit.
        cv2.setNumThreads(max(1, min(4, int(rospy.get_param("~opencv_threads", 1)))))
        self._lock = threading.RLock()
        self._backend_lock = threading.Lock()
        self._commit_lock = threading.RLock()
        self._epoch = 0
        self._model_busy = False
        self._image_mailbox = LatestMailbox()
        self._inference_mailbox = LatestMailbox()
        self._result_mailbox = LatestMailbox()
        self._publication_context = threading.local()
        self._pipeline_current = None
        self._last_input_stamp = rospy.Time(0)
        self._inference_ms = 0.
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
        self._selected_track_id: Optional[int] = None
        self._selected_track_id_is_stable = False
        self._selected_class_id: Optional[int] = None
        self._selected_last_bbox: Optional[Tuple[int, int, int, int]] = None

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
        selected_detections_topic = rospy.get_param(
            "~selected_detections_topic",
            "sar_yolo_detector/smart/selected_tracked_detections",
        )
        select_pixel_topic = rospy.get_param(
            "~select_pixel_topic", "sar_yolo_detector/smart/select_pixel"
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
        self._capture_clock = CaptureClock(self._maximum_age, self._future_tolerance)
        self._maximum_image_bytes = int(rospy.get_param("~maximum_image_bytes", 33554432))
        if not 1024 <= self._maximum_image_bytes <= 134217728:
            raise ValueError("maximum_image_bytes must be within 1 KiB..128 MiB")

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
            "fusiontracker": "fusion_tracker",
            "fusion_tracker": "fusion_tracker",
            "fusion": "fusion_tracker",
        }
        if default_tracker not in default_aliases:
            raise RuntimeError(
                "Default_Tracker must be SmartTracker, FeatureTracker, ColorTracker, or FusionTracker"
            )
        self._tracking_mode = default_aliases[default_tracker]
        # Archived launch files bridge the legacy array without provenance.
        # Only the additive fusion runtime may publish auxiliary observations
        # into its provenance-aware bridge; old mode contracts remain intact.
        self._fusion_mode_enabled = bool(rospy.get_param("~enable_fusion_mode", False))
        if self._tracking_mode == "fusion_tracker" and not self._fusion_mode_enabled:
            raise RuntimeError("Use xd_smart_tracker_fusion_integration.launch for provenance-safe FusionTracker")
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
        self._manual_color_candidates = ()
        self._manual_color_selected_bbox = None
        self._manual_color_misses = 0
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
        fusion_config = dict(rospy.get_param("~FusionTracker", {}))
        self._fusion = BoxFusion(fusion_config, manual_config)
        self._loss_journal = LossJournal()
        self._last_loss_revision = None
        self._input_rejection_counts = Counter()
        self._last_input_rejection = None
        self._fusion_next_id = max(900000001, self._manual_track_id + 1)
        self._fusion_snapshots = deque()
        self._fusion_snapshot_max_frames = int(fusion_config.get("selection_max_frames", 24))
        if not 2 <= self._fusion_snapshot_max_frames <= 256:
            raise ValueError("FusionTracker/selection_max_frames must be within 2..256")
        self._fusion_input_shape = None
        self._fusion_snapshot_max_age = max(.05, float(fusion_config.get("selection_max_age_sec", .75)))
        self._fusion_snapshot_max_bytes = max(1024, int(fusion_config.get("selection_max_bytes", 25165824)))
        self._flow = FlowDetections(float(fusion_config.get("async_max_result_age_sec", .75)))
        self._camera_motion = CameraMotion(fusion_config.get("camera_motion", {}))
        self._motion_header = None
        self._motion_frame = None
        self._camera_info = None
        self._motion_fixed_frame = str(rospy.get_param("~camera_motion_fixed_frame", "world"))
        self._tf_buffer = self._tf_listener = None
        if self._camera_motion.enabled and self._motion_fixed_frame:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(3.))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        smart_config = dict(rospy.get_param("~SmartTracker", {}))
        if not smart_config.get("SMART_TRACKER_ENABLED", True):
            raise RuntimeError("SmartTracker is disabled by configuration")
        smart_config["SMART_TRACKER_MODELS_ROOT"] = str(self._package_root)
        for key in (
            "SMART_TRACKER_GPU_MODEL_PATH",
            "SMART_TRACKER_CPU_MODEL_PATH",
            "DEEP_REID_MODEL_PATH",
            "SMART_TRACKER_TRACKER_CONFIG_PATH",
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
        self._fusion_array_publisher = rospy.Publisher(
            rospy.get_param("~fusion_detections_topic", tracked_topic + "/fusion"),
            FusionDetection2DArray, queue_size=1)
        self._observation_publisher = rospy.Publisher(
            rospy.get_param("~fusion_observations_topic", tracked_topic + "/observations"),
            FusionDetection2DArray, queue_size=1)
        self._last_observation_output_stamp = rospy.Time(0)
        self._fusion_evidence_publisher = rospy.Publisher("~fusion_evidence", String, queue_size=1)
        self._fusion_loss_publisher = rospy.Publisher("~fusion_loss_history", String, queue_size=1, latch=True)
        self._state_publisher = rospy.Publisher(
            state_topic, SmartTrackerState, queue_size=3
        )
        self._annotated_publisher = rospy.Publisher(
            annotated_topic, Image, queue_size=1
        )
        self._selected_detections_publisher = rospy.Publisher(
            selected_detections_topic, TrackedDetection2DArray, queue_size=3
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
        self._fusion_roi_service = rospy.Service("~select_fusion_roi", SelectFusionROI, self._select_fusion_roi)
        self._subscriber = rospy.Subscriber(
            self._input_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
            tcp_nodelay=True,
        )
        camera_info_topic = str(rospy.get_param("~camera_info_topic", ""))
        self._camera_info_subscriber = rospy.Subscriber(camera_info_topic, CameraInfo,
            self._camera_info_callback, queue_size=1) if camera_info_topic else None
        self._select_pixel_subscriber = rospy.Subscriber(
            select_pixel_topic,
            PointStamped,
            self._select_pixel_callback,
            queue_size=1,
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
        self._workers = [threading.Thread(target=self._frame_loop, name="sar-image", daemon=True),
                         threading.Thread(target=self._inference_loop, name="sar-yolo", daemon=True)]
        for worker in self._workers:
            worker.start()
        rospy.loginfo(
            "SmartTracker ready: model=%s device=%s topic=%s default=%s manual_roi=%s",
            runtime.get("model_path"),
            runtime.get("effective_device"),
            self._input_topic,
            self._tracking_mode,
            self._manual_initial_roi_topic if self._manual_enabled else "disabled",
        )
        rospy.loginfo(
            "Click selection: pixel_topic=%s (negative pixel cancels) selected_detections=%s",
            select_pixel_topic,
            selected_detections_topic,
        )

    def close(self) -> None:
        timer = getattr(self, "_diagnostic_timer", None)
        if timer is not None:
            timer.shutdown()
        for name in ("_subscriber", "_select_pixel_subscriber", "_manual_roi_subscriber",
                     "_camera_info_subscriber", "_tf_listener"):
            subscriber = getattr(self, name, None)
            if subscriber is not None:
                subscriber.unregister()
        # Stop submissions first; do not hold the state lock while joining.
        for name in ("_image_mailbox", "_inference_mailbox", "_result_mailbox"):
            mailbox = getattr(self, name, None)
            if mailbox is not None:
                mailbox.close()
        for worker in getattr(self, "_workers", ()):
            if worker is not threading.current_thread():
                worker.join(timeout=2.)
        if any(worker.is_alive() for worker in getattr(self, "_workers", ())):
            rospy.logwarn("Inference is still finishing; skipping unsafe concurrent model close")
            return
        with self._lock:
            tracker = getattr(self, "_tracker", None)
            if tracker is not None:
                if self._backend_lock.acquire(blocking=False):
                    try:
                        tracker.close()
                        self._tracker = None
                    finally:
                        self._backend_lock.release()
                else:
                    rospy.logwarn("Model switch still finishing; skipping unsafe concurrent model close")
            for manual_tracker in self._manual_trackers.values():
                manual_tracker.reset()
            self._fusion.reset()
            self._fusion_snapshots.clear()

    def _current_manual_tracker(self) -> Optional[ManualBoxTracker]:
        return self._manual_trackers.get(self._tracking_mode)

    def _camera_info_callback(self, message):
        # CameraInfo is immutable after receipt. No image/model lock required.
        self._camera_info = message

    def _pose_rotation(self, message):
        info, previous = self._camera_info, self._motion_header
        if (self._tf_buffer is None or info is None or previous is None
                or previous.frame_id != message.header.frame_id
                or info.header.frame_id != message.header.frame_id
                or (info.width, info.height) != (message.width, message.height)
                or any(abs(value) > 1e-8 for value in info.D)):
            return None
        if not info.header.stamp.is_zero() and abs((message.header.stamp-info.header.stamp).to_sec()) > .5:
            return None
        try:
            rotations = []
            for stamp in (previous.stamp, message.header.stamp):
                transform = self._tf_buffer.lookup_transform(self._motion_fixed_frame,
                    message.header.frame_id, stamp, rospy.Duration(0))
                q = transform.transform.rotation
                rotations.append(quaternion_rotation((q.x, q.y, q.z, q.w)))
            return rotation_homography(info.K, *rotations)
        except Exception:
            # Missing/extrapolated TF never blocks the image worker. Background
            # estimation remains available; neither path claims metric depth.
            return None

    def _manual_roi_callback(self, message: RegionOfInterest) -> None:
        """Initialize the already-selected manual mode, or clear its target."""
        with self._lock:
            if getattr(self, "_model_busy", False):
                rospy.logwarn_throttle(2., "Manual ROI ignored during model switch")
                return
            if self._tracking_mode == "fusion_tracker":
                rospy.logwarn_throttle(2., "Fusion ROI requires ~select_fusion_roi with the displayed image header")
                return
            manual_tracker = self._current_manual_tracker()
            if manual_tracker is None:
                rospy.logwarn_throttle(
                    2.0,
                    "Initial ROI ignored while mode=%s; call ~switch_tracking_mode first",
                    self._tracking_mode,
                )
                return
            configured_colour = (
                self._tracking_mode == "color_tracker"
                and manual_tracker.colour_source == "configured"
            )
            if not manual_tracker.requires_initial_roi and not configured_colour:
                rospy.logwarn_throttle(
                    2.0,
                    "Initial ROI ignored: %s does not accept a manual ROI in its current configuration",
                    self._tracking_mode,
                )
                return
            if int(message.width) == 0 or int(message.height) == 0:
                manual_tracker.reset()
                self._manual_pending_roi = None
                self._manual_active = False
                if configured_colour:
                    self._manual_color_selected_bbox = None
                    self._manual_color_misses = 0
                    self._publish_empty_selected_detections()
                rospy.loginfo(
                    "%s target cleared; waiting for a new initial ROI",
                    self._tracking_mode,
                )
                return
            if configured_colour:
                x, y, width, height = (float(message.x_offset), float(message.y_offset),
                                       float(message.width), float(message.height))
                roi_area = width * height
                matches = []
                for detection in self._manual_color_candidates:
                    bx1, by1, bx2, by2 = detection.aabb_xyxy
                    overlap = max(0., min(x+width, bx2)-max(x, bx1)) * max(0., min(y+height, by2)-max(y, by1))
                    if overlap > 0:
                        box_area = max(1., (bx2-bx1)*(by2-by1))
                        matches.append((overlap / max(1., min(roi_area, box_area)), overlap, detection))
                if not matches:
                    rospy.logwarn("Colour ROI did not overlap any detected colour box")
                    return
                _, _, selected = max(matches, key=lambda item: (item[0], item[1]))
                self._manual_color_selected_bbox = tuple(selected.aabb_xyxy)
                self._manual_color_misses = 0
                self._manual_active = True
                rospy.loginfo("UI selected colour box by ROI: track_id=%d bbox=%s",
                              selected.track_id, tuple(int(v) for v in selected.aabb_xyxy))
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
        # ROS transport never waits for CUDA, KCF, rendering, or a mode service.
        self._image_mailbox.put(message)

    def _infer(self, frame):
        tracker = self._tracker
        arguments = dict(conf=tracker.conf_threshold, iou=tracker.iou_threshold, max_det=tracker.max_det)
        if tracker._should_run_track_api():
            _, detections = tracker.backend.detect_and_track(
                frame, tracker_type=tracker.tracker_type_str, tracker_args=tracker.tracker_args, **arguments)
        else:
            _, detections = tracker.backend.detect(frame, **arguments)
        return tuple(detections)

    def _inference_loop(self):
        backend_epoch = None
        while not self._inference_mailbox.closed:
            job = self._inference_mailbox.get(.25)
            if job is None:
                continue
            epoch, message, frame = job
            started, error, detections = time.perf_counter(), "", ()
            with self._backend_lock:
                if epoch != self._epoch or self._model_busy:
                    continue
                try:
                    generation = (epoch, frame.shape[:2])
                    if backend_epoch != generation:
                        self._tracker.backend.reset_tracking()
                        backend_epoch = generation
                    detections = self._infer(frame)
                except Exception as exc:
                    error = "inference failed: %s: %s" % (type(exc).__name__, exc)
                    rospy.logerr_throttle(
                        2., "%s\n%s" % (error, traceback.format_exc()))
            if epoch != self._epoch:
                continue
            packet = InferencePacket(message, frame, detections, epoch,
                                     (time.perf_counter()-started)*1000., error)
            self._result_mailbox.put(packet)
            self._image_mailbox.wake()

    def _frame_loop(self):
        while not self._image_mailbox.closed:
            message = self._image_mailbox.get(.25)
            packet = self._result_mailbox.get()
            try:
                if message is not None:
                    if len(message.data) > self._maximum_image_bytes:
                        self._rejected_frames += 1
                        with self._lock:
                            self._note_frame_rejection("image_byte_limit")
                        self._last_error = "image exceeds configured byte limit"
                        continue
                    frame = _image_to_bgr(message)
                    with self._lock:
                        decision = self._capture_clock.observe(message.header.stamp.to_sec(),
                            message.header.seq, message.header.frame_id,
                            (message.height, message.width), rospy.Time.now().to_sec())
                        valid = decision.accepted
                        if decision.new_segment and valid:
                            self._reset_temporal_state()
                            rospy.logwarn("New capture segment: %s", decision.reason)
                        epoch, mode = self._epoch, self._tracking_mode
                    if not valid:
                        self._rejected_frames += 1
                        with self._lock:
                            self._note_frame_rejection(decision.reason)
                        self._last_error = decision.reason if not decision.accepted else "image exceeds configured byte limit"
                        if packet is not None:
                            self._result_mailbox.put(packet)
                            self._image_mailbox.wake()
                        continue
                    self._last_input_stamp = message.header.stamp
                    self._pipeline_current = (epoch, message, frame)
                    if mode in ("smart_tracker", "fusion_tracker") and not self._model_busy:
                        self._inference_mailbox.put((epoch, message, frame))
                current = self._pipeline_current
                if current is None or current[0] != self._epoch or self._model_busy:
                    continue
                if packet is not None and packet.epoch != self._epoch:
                    packet = None
                if self._tracking_mode == "smart_tracker":
                    if packet is not None:
                        self._process_image(packet.image, packet.frame, packet=packet, expected_epoch=packet.epoch)
                elif message is not None or packet is not None:
                    self._process_image(current[1], current[2], packet=packet,
                                        result_only=message is None, expected_epoch=current[0])
            except Exception as exc:
                self._publication_context.batch = None
                self._last_error = "image worker failed: %s" % type(exc).__name__
                rospy.logerr_throttle(2., "%s: %s", self._last_error, exc)

    def _emit(self, publisher, message):
        context = getattr(self, "_publication_context", None)
        batch = getattr(context, "batch", None)
        if batch is None:
            publisher.publish(message)
        else:
            batch.append((publisher, message))

    def _note_frame_rejection(self, reason):
        # Called under the state lock. Rejected captures are not observations
        # and must not advance target clocks or confirmation counters.
        if not hasattr(self, "_input_rejection_counts"):
            self._input_rejection_counts = Counter()
        key = str(reason)[:96]
        if key not in self._input_rejection_counts and len(self._input_rejection_counts) >= 16:
            key = "other"
        self._input_rejection_counts[key] += 1
        self._last_input_rejection = str(reason)[:160]

    def _process_image(self, message: Image, frame=None, packet=None, result_only=False, expected_epoch=None) -> None:
        valid, reason = self._valid_timestamp(message)
        if result_only and message.header.stamp == self._last_capture_stamp:
            age = (rospy.Time.now()-message.header.stamp).to_sec()
            valid = age >= -self._future_tolerance and (self._maximum_age == 0 or age <= self._maximum_age)
        if not valid:
            self._rejected_frames += 1
            with self._lock:
                self._note_frame_rejection(reason)
            self._last_error = reason
            rospy.logwarn_throttle(2.0, "SmartTracker rejected frame: %s", reason)
            return

        try:
            frame = _image_to_bgr(message) if frame is None else frame
        except Exception as exc:
            self._rejected_frames += 1
            self._last_error = f"image conversion failed: {exc}"
            rospy.logerr_throttle(2.0, self._last_error)
            return

        self._publication_context.batch = []
        overlay = None
        with self._lock:
            if self._tracker is None or self._model_busy or (expected_epoch is not None and expected_epoch != self._epoch):
                self._publication_context.batch = None
                return
            valid, reason = self._valid_timestamp(message)
            if result_only and message.header.stamp == self._last_capture_stamp:
                valid = True
            if packet is not None and packet.epoch != self._epoch:
                valid = False
            if not valid:
                self._rejected_frames += 1
                self._note_frame_rejection(reason)
                self._last_error = reason
                self._publication_context.batch = None
                return
            self._controller.current_frame = frame
            self._controller.video_handler.height = int(frame.shape[0])
            self._controller.video_handler.width = int(frame.shape[1])
            started = time.perf_counter()
            manual_handled = False
            manual_tracker = self._current_manual_tracker()
            previous_frame = getattr(self, "_motion_frame", None)
            motion_matrix = None
            if not result_only and self._tracking_mode != "smart_tracker":
                boxes = [(box[0], box[1], box[2]-box[0], box[3]-box[1]) for _, box, _ in self._flow.items]
                if self._fusion.box is not None:
                    boxes.append(self._fusion.box)
                if manual_tracker is not None and manual_tracker._last_bbox is not None:
                    boxes.append(manual_tracker._last_bbox)
                motion_matrix = self._camera_motion.update(frame, message.header.stamp.to_sec(),
                    boxes, self._pose_rotation(message) if boxes else None, estimate=bool(boxes))
                self._motion_header, self._motion_frame = message.header, frame
            if self._tracking_mode == "fusion_tracker":
                fresh = packet is not None and packet.image.header.stamp <= message.header.stamp
                detections = self._flow.update(frame, message.header.stamp.to_sec(),
                    packet.detections if fresh else None,
                    packet.frame if fresh else None,
                    packet.image.header.stamp.to_sec() if fresh else None, motion=self._camera_motion)
                if fresh:
                    self._inference_ms = packet.processing_ms
                annotated = self._process_fusion(message, frame, detections=detections,
                    detector_updated=fresh, result_only=result_only, defer_overlay=True,
                    detector_capture_stamp=packet.image.header.stamp.to_sec() if fresh else None,
                    inference_error=packet.error if fresh else "",
                    processing_started=started)
                if fresh:
                    self._inference_ms = packet.processing_ms
                    self._last_error = packet.error
                    self._publish_detector_observations(packet)
                overlay = self._overlay_payload
                manual_handled = True
            elif (manual_tracker is not None and self._tracking_mode == "color_tracker"
                  and manual_tracker.colour_source == "configured"):
                raw_candidates = manual_tracker.configured_colour_candidates(frame)
                # Keep output bounded while retaining the largest useful red regions.
                raw_candidates = sorted(raw_candidates, key=lambda item: item[2], reverse=True)[:30]
                raw_candidates.sort(key=lambda item: (item[0][0], item[0][1]))
                candidates = []
                for index, (bbox, confidence, _area) in enumerate(raw_candidates):
                    x, y, width, height = bbox
                    candidates.append(SimpleNamespace(
                        aabb_xyxy=(x, y, x+width, y+height),
                        class_id=self._manual_class_id,
                        confidence=confidence,
                        track_id=self._manual_track_id + index + 1,
                        track_id_is_stable=False,
                        rotation_deg=0.0,
                        measurement_source="visual_tracker",
                        last_detector_stamp=rospy.Time(0),
                    ))
                self._manual_color_candidates = tuple(candidates)
                selected_source = self._select_manual_color_candidate(candidates, frame.shape)
                selected = None
                detections = []
                for candidate in candidates:
                    is_selected = candidate is selected_source
                    item = SimpleNamespace(**vars(candidate))
                    if is_selected:
                        item.track_id = self._manual_track_id
                        item.track_id_is_stable = True
                        selected = item
                    detections.append(item)
                self._manual_color_candidates = tuple(detections)
                self._manual_active = selected is not None
                manual_handled = True
                annotated = frame
                if selected is not None:
                    x1, y1, x2, y2 = selected.aabb_xyxy
                    result = ManualTrackResult(
                        (x1, y1, x2-x1, y2-y1), selected.confidence, True,
                        "selected_colour_target", "color_%s_components" % manual_tracker.colour_name, 0)
                else:
                    reason = "waiting_for_ui_selection" if candidates and self._manual_color_selected_bbox is None else "selected_colour_not_found"
                    result = ManualTrackResult(None, 0.0, False, reason,
                        "color_%s_components" % manual_tracker.colour_name, self._manual_color_misses)
                self._publish_configured_colour_result(message, detections, selected, result)
                overlay = ("boxes", tuple(detections),
                    {self._manual_class_id: manual_tracker.colour_name},
                    self._manual_track_id if selected is not None else -1, "")
                self._last_processing_ms = (time.perf_counter()-started)*1000.
                self._last_error = "" if result.valid or result.reason == "waiting_for_ui_selection" else result.reason
            elif manual_tracker is not None:
                if motion_matrix is not None and previous_frame is not None:
                    manual_tracker.compensate_motion(previous_frame, motion_matrix)
                pending_roi = self._manual_pending_roi
                self._manual_pending_roi = None
                result = (
                    manual_tracker.initialize(frame, pending_roi)
                    if pending_roi is not None
                    else manual_tracker.update(frame)
                )
                self._manual_active = manual_tracker.active
                manual_handled = True
                annotated = frame
                overlay = ("manual", result)
                self._last_processing_ms = (time.perf_counter()-started)*1000.
                self._publish_manual_result(message, result)
                self._last_error = (
                    ""
                    if result.valid or result.reason == "waiting_for_initial_roi"
                    else result.reason
                )

            if not manual_handled:
                annotated = self._tracker.track_and_draw(frame, detections=packet.detections, draw=False)
                self._inference_ms = packet.processing_ms
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
                self._publish_detector_observations(packet)
                self._last_processing_ms = (time.perf_counter()-started)*1000.
                self._publish_state(message, output)
                overlay = ("boxes", tuple(detections), dict(self._tracker.labels), self._selected_track_id, "")
            else:
                detections = ()

            if not manual_handled:
                selected_detection = self._selected_detection_for_frame(detections)
                self._publish_selected_detections(message, selected_detection)

            self._last_processing_ms = (time.perf_counter() - started) * 1000.0
            self._last_header = message.header
            self._last_capture_stamp = message.header.stamp
            self._last_frame_wall = time.monotonic()
            self._processed_frames += 1
            epoch = self._epoch
        batch, self._publication_context.batch = self._publication_context.batch, None
        with self._commit_lock:
            if epoch != self._epoch:
                return
            for publisher, output_message in batch:
                publisher.publish(output_message)
        # Render/serialize only when requested, with immutable final boxes and
        # the exact capture header. Never draw by querying a newer live state.
        if self._publish_annotated and self._annotated_publisher.get_num_connections() > 0:
            annotated = self._render_overlay(frame.copy(), overlay)
            video = _bgr_to_image(annotated, message.header)
            with self._commit_lock:
                if epoch == self._epoch:
                    self._annotated_publisher.publish(video)
        self._updater.update()

    @classmethod
    def _render_overlay(cls, frame, payload):
        if payload[0] == "manual":
            return cls._annotate_manual(frame, payload[1])
        _, detections, labels, selected_id, status = payload
        for detection in detections:
            x1, y1, x2, y2 = detection.aabb_xyxy
            colour = (0, 255, 100) if selected_id >= 0 and detection.track_id == selected_id else (255, 190, 0)
            cv2.rectangle(frame, (round(x1), round(y1)), (round(x2), round(y2)), colour, 2)
            cv2.putText(frame, "%s ID %d" % (labels.get(detection.class_id, "unknown"), detection.track_id),
                        (round(x1), max(18, round(y1)-5)), cv2.FONT_HERSHEY_SIMPLEX, .5, colour, 1)
        if status:
            cv2.putText(frame, status, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 180, 255), 1)
        return frame

    @staticmethod
    def _fusion_candidate(detection):
        x1, y1, x2, y2 = detection.aabb_xyxy
        return Candidate((float(x1), float(y1), float(x2 - x1), float(y2 - y1)),
                         int(detection.class_id), float(detection.confidence),
                         int(detection.track_id), bool(detection.track_id_is_stable),
                         getattr(detection, "detector_stamp", None))


    def _process_fusion(self, image, frame, detections=None, detector_updated=True,
                        result_only=False, defer_overlay=False, detector_capture_stamp=None,
                        processing_started=None, inference_error=""):
        started = time.perf_counter() if processing_started is None else processing_started
        error = inference_error
        shape = frame.shape[:2]
        if getattr(self, "_fusion_input_shape", None) is not None and shape != self._fusion_input_shape:
            self._fusion.reset()
            self._fusion_snapshots.clear()
        self._fusion_input_shape = shape
        suppressed_class_duplicates = 0
        try:
            if detections is None:
                detections = self._infer(frame)
            allowed = self._tracker.allowed_class_ids
            detections = [d for d in detections if allowed is None or int(d.class_id) in allowed]
            candidates = [self._fusion_candidate(d) for d in detections]
            pairs = [(d, c) for d, c in zip(detections, candidates)
                     if valid_box(c.box, frame.shape) and math.isfinite(c.confidence) and 0 <= c.confidence <= 1]
            detections = [d for d, _ in pairs]
            candidates = [c for _, c in pairs]
            # Defensive fallback for adapters/backends without pre-track NMS.
            # Prevent duplicate leaf classes from making ROI selection ambiguous.
            selection_fresh = (self._fusion.result.observed and
                               0 <= image.header.stamp.to_sec() - self._fusion.last_observation_stamp <= self._fusion.max_gap)
            keep = class_duplicate_indices(candidates, self._fusion.class_groups,
                                           self._fusion.local_id if selection_fresh else -1)
            suppressed_class_duplicates += len(candidates) - len(keep)
            detections = [detections[index] for index in keep]
            candidates = [candidates[index] for index in keep]
        except Exception as exc:
            error = "fusion inference failed: %s" % type(exc).__name__
            rospy.logerr_throttle(2.0, error)
            detections, candidates = [], []
        self._tracker.last_detections = list(detections)
        stamp = image.header.stamp.to_sec()
        if self._fusion.public_id >= 0:
            # Cached LK targets are display observations, not repeated detector
            # confirmations. Only a NEW inference packet may rebind an ID.
            measured_candidates = [candidate for candidate in candidates
                if detector_capture_stamp is None or candidate.detector_stamp == detector_capture_stamp]
            # Feed the already validated same-identity LK observation to the
            # selected SOT arbiter as well. It remains visual evidence, not a
            # detector confirmation, including on same-capture corrections.
            flow_detection = self._flow.visual_observations.get(self._fusion.local_id) if self._flow.stamp == stamp else None
            if flow_detection is None:
                flow_detection = next((d for d in detections
                if int(d.track_id) == self._fusion.local_id
                and getattr(d, 'measurement_source', '') == 'visual_tracker'
                and getattr(d, 'flow_quality', 0.) >= .60), None)
            result = self._fusion.step(frame, stamp, measured_candidates if detector_updated else (),
                                      detector_updated=detector_updated,
                                      same_frame_correction=result_only,
                                      visual_observation=self._fusion_candidate(flow_detection) if flow_detection is not None else None,
                                      visual_quality=getattr(flow_detection, 'flow_quality', 0.),
                                      visual_scale_ratio=getattr(flow_detection, 'flow_scale_ratio', None),
                                      camera_motion_reliable=getattr(getattr(self, '_camera_motion', None),
                                          'source', '') in ('background', 'pose_seeded_background'),
                                      camera_transform=self._camera_motion.between(self._fusion.stamp, stamp)
                                      if hasattr(self, "_camera_motion") and not result_only else None)
            if self._fusion.diagnostics.get('identity_rebound'):
                self._fusion.diagnostics['lk_reseeded'] = self._flow.reseed_identity(
                    frame, stamp, self._fusion.diagnostics['previous_local_id'], result.local_id,
                    self._fusion.diagnostics['rebound_box'])
        else:
            result = self._fusion.result
        # A bounded original-image cache is used only by frame-bound selection.
        self._cache_fusion_snapshot(image, frame, candidates)
        # Raw BoT-SORT output must not bypass a rejected selected-entity gate.
        bound_ids = {identity for identity in (result.public_id, result.local_id) if identity >= 0}
        if result.observed and result.reason == "reacquisition_confirmation_visual_hold":
            # The new local ID is an uncommitted hypothesis for this selected
            # entity, not a second display target with the same pixels.
            bound_ids.add(self._fusion.pending_id)
        output = [d for d in detections if int(d.track_id) not in bound_ids]
        if result.observed:
            # Replace, never append a duplicate selected local YOLO track.
            output = [d for d in output if int(d.track_id) != result.local_id]
            x, y, w, h = result.box
            output.append(SimpleNamespace(aabb_xyxy=(x, y, x + w, y + h),
                                          class_id=result.class_id, confidence=result.confidence,
                                          track_id=result.public_id, track_id_is_stable=True, rotation_deg=0.,
                                          measurement_source="visual_tracker" if result.source == "visual" else "detector",
                                          last_detector_stamp=rospy.Time.from_sec(result.last_detection_stamp)))
        # SOT can overlap a new YOLO class even after upstream NMS. All numeric
        # topics and the overlay must consume this one final, deduplicated list.
        keep = class_duplicate_indices([self._fusion_candidate(d) for d in output],
                                       self._fusion.class_groups,
                                       result.public_id if result.observed else -1,
                                       suppress_selected_same_class=True)
        suppressed_class_duplicates += len(output) - len(keep)
        output = [output[index] for index in keep]
        self._last_processing_ms = (time.perf_counter() - started) * 1000.
        self._last_error = error
        evidence = dict(stamp=stamp, frame_seq=image.header.seq, frame_id=image.header.frame_id,
                        capture_header=dict(seq=image.header.seq, frame_id=image.header.frame_id,
                            stamp=dict(secs=image.header.stamp.secs, nsecs=image.header.stamp.nsecs)),
                        mode="fusion_tracker", selected_id=result.public_id, local_id=result.local_id,
                        source=result.source, reason=result.reason,
                        selected_box=result.box,
                        last_detection_stamp=result.last_detection_stamp,
                        age_ms=max(0., (stamp - result.last_detection_stamp) * 1000.) if result.last_detection_stamp else None,
                        observed=result.observed, processing_ms=self._last_processing_ms,
                        suppressed_class_duplicates=suppressed_class_duplicates,
                        inference_ms=getattr(self, "_inference_ms", 0.),
                        pipeline_epoch=getattr(self, "_epoch", 0),
                        capture_segment=getattr(getattr(self, "_capture_clock", None), "segment", 0))
        evidence['diagnostics'] = dict(self._fusion.diagnostics,
            inference_error=error,
            camera_motion_source=getattr(getattr(self, '_camera_motion', None), 'source', 'unavailable'),
            input_rejections=dict(getattr(self, '_input_rejection_counts', {})),
            last_input_rejection=getattr(self, '_last_input_rejection', None),
            lk_active_tracks=len(self._flow.items), lk=dict(self._flow.diagnostics))
        journal = getattr(self, '_loss_journal', None)
        if journal is None:
            self._loss_journal = journal = LossJournal()
        journal.observe(evidence)
        evidence['loss_event'] = journal.summary()
        revision = (journal.event_count, journal.recovered_count, journal.interrupted_count)
        history_publisher = getattr(self, '_fusion_loss_publisher', None)
        history_now = time.monotonic()
        if (history_publisher is not None
                and history_now-getattr(self, '_last_loss_publish_wall', 0.) >= 1.
                and (revision != getattr(self, '_last_loss_revision', None) or journal.active is not None)):
            self._emit(history_publisher, String(data=json.dumps(journal.snapshot(recent_limit=2), allow_nan=False)))
            self._last_loss_revision = revision
            self._last_loss_publish_wall = history_now
        self._last_processing_ms = (time.perf_counter()-started)*1000.
        evidence['processing_ms'] = self._last_processing_ms
        self._emit(self._fusion_evidence_publisher, String(data=json.dumps(evidence, allow_nan=False)))
        self._publish_detections(image, output)
        selected_detection = next((d for d in output if int(d.track_id) == result.public_id), None) if result.observed else None
        self._publish_selected_detections(image, selected_detection)
        state = SmartTrackerState()
        state.header = copy.copy(image.header)
        state.tracking_active = result.public_id >= 0
        state.has_selection = state.tracking_active
        state.selected_track_id = result.public_id
        state.selected_class_id = result.class_id
        state.selected_class_name = self._tracker.labels.get(result.class_id, "unknown")
        state.confidence = result.confidence
        self._fill_bbox(state.selected_bbox, result.box)
        state.geometry_type = "aabb" if result.box else "none"
        state.measurement_current = result.source in ("yolo", "fused")
        state.prediction_only = result.source == "prediction"
        state.tentative = result.source == "ambiguous"
        state.data_is_stale = result.source in ("lost", "ambiguous")
        state.control_measurement_ready = result.observed
        state.association_method = "fusion_" + result.source
        state.freshness_reason = result.reason
        state.tracker_type = "YOLO+" + self._fusion.tracker.tracker_name
        runtime = self._tracker.get_runtime_info()
        state.backend = str(runtime.get("backend", "ultralytics"))
        state.device = str(runtime.get("effective_device", runtime.get("device", "unknown")))
        state.model_path = str(runtime.get("model_path", ""))
        state.detection_count = len(output)
        state.frame_processing_ms = self._last_processing_ms
        self._emit(self._state_publisher, state)
        # Both image and numeric outputs use exactly the same final boxes.
        status = "%s ID %d" % (result.source.upper(), result.public_id) if result.source in ("lost", "ambiguous", "prediction") else ""
        self._overlay_payload = ("boxes", tuple(output), dict(self._tracker.labels), result.public_id, status)
        if defer_overlay:
            return frame
        return self._render_overlay(frame.copy(), self._overlay_payload) if self._publish_annotated else frame

    def _cache_fusion_snapshot(self, image, frame, candidates):
        if self._fusion_snapshots and self._fusion_snapshots[-1][0] == image.header:
            self._fusion_snapshots.pop()
        self._fusion_snapshots.append((copy.copy(image.header), frame, tuple(candidates)))
        while self._fusion_snapshots and (
                len(self._fusion_snapshots) > getattr(self, "_fusion_snapshot_max_frames", 2)
                or image.header.stamp.to_sec()-self._fusion_snapshots[0][0].stamp.to_sec() > self._fusion_snapshot_max_age
                or sum(snapshot[1].nbytes for snapshot in self._fusion_snapshots) > self._fusion_snapshot_max_bytes):
            self._fusion_snapshots.popleft()


    def _initialize_fusion_selection(self, snapshot, box):
        header, frame, candidates = snapshot
        if not valid_box(box, frame.shape):
            raise ValueError("ROI must lie inside the original image and be at least 4x4 pixels")
        # Do not initialize a large background ROI if it contains a unique YOLO box.
        matches = [c for c in candidates if iou(c.box, box) >= .15 or
                   (box[0] <= c.box[0] + c.box[2] / 2 <= box[0] + box[2] and
                    box[1] <= c.box[1] + c.box[3] / 2 <= box[1] + box[3])]
        if len(matches) > 1:
            raise ValueError("ROI contains multiple targets; select a tighter ROI")
        candidate = matches[0] if matches else None
        if candidate is not None:
            box = candidate.box
        if self._fusion_next_id > 2147483647:
            raise ValueError("Fusion identity range exhausted; restart with a new session")
        public_id = candidate.local_id if candidate is not None and candidate.stable and candidate.local_id >= 0 else self._fusion_next_id
        result = self._fusion.initialize(frame, box, header.stamp.to_sec(), public_id, candidate)
        if public_id == self._fusion_next_id:
            self._fusion_next_id += 1
        return result


    def _select_fusion_roi(self, request):
        response = SelectFusionROIResponse()
        with self._lock:
            if getattr(self, "_model_busy", False):
                response.message = "BUSY: model switch in progress"
                return response
            if self._tracking_mode != "fusion_tracker":
                response.message = "Switch to fusion_tracker before selecting an ROI"
                return response
            if request.clear_selection:
                self._fusion.reset()
                self._publish_empty_selected_detections()
                response.success = True
                response.selected_target_id = -1
                response.message = "Fusion selection cleared"
                return response
            snapshot = next((s for s in self._fusion_snapshots
                             if s[0].stamp == request.header.stamp
                             and s[0].frame_id == request.header.frame_id), None)
            if snapshot is None or request.header.stamp.is_zero():
                response.message = "Original image frame is not cached; refresh the displayed image"
                return response
            now = rospy.Time.now()
            age = (now - request.header.stamp).to_sec()
            if age > self._fusion_snapshot_max_age or age < -self._future_tolerance:
                response.message = "Original image is stale or in the future"
                return response
            try:
                roi = request.roi
                result = self._initialize_fusion_selection(snapshot, (roi.x_offset, roi.y_offset, roi.width, roi.height))
                response.success = True
                response.selected_target_id = result.public_id
                response.message = "Fusion initialized from the requested original frame"
            except ValueError as exc:
                response.message = str(exc)
            return response


    def _select_fusion_latest(self, request, response):
        if not self._fusion_snapshots:
            response.message = "No original image is available"
            return response
        snapshot = self._fusion_snapshots[-1]
        header, frame, candidates = snapshot
        age = (rospy.Time.now() - header.stamp).to_sec()
        if age > self._fusion_snapshot_max_age or age < -self._future_tolerance:
            response.message = "Latest selection image is stale"
            return response
        try:
            if request.use_target_id:
                matches = [c for c in candidates if c.local_id == request.target_id]
                if request.target_id == self._fusion.public_id and self._fusion.result.observed:
                    response.success = True
                    response.tracking_active = True
                    response.selected_target_id = self._fusion.public_id
                    response.measurement_current = self._fusion.result.source in ("yolo", "fused")
                    response.message = "Fusion entity already selected"
                    return response
                if len(matches) != 1:
                    raise ValueError("Target ID is not unique in the current detector frame")
                box = matches[0].box
            elif request.use_normalized_roi:
                x, y, w, h = request.normalized_roi
                roi = tracking_roi_to_pixels(x=x, y=y, width=w, height=h,
                                             coordinate_space="normalized",
                                             frame_width=frame.shape[1], frame_height=frame.shape[0])
                box = (roi["x"], roi["y"], roi["width"], roi["height"])
            else:
                point = tracking_point_to_pixels(x=request.point_x, y=request.point_y,
                                                coordinate_space="normalized" if request.use_normalized_point else "pixels",
                                                frame_width=frame.shape[1], frame_height=frame.shape[0])
                matches = [c for c in candidates if c.box[0] <= point[0] <= c.box[0] + c.box[2]
                           and c.box[1] <= point[1] <= c.box[1] + c.box[3]]
                if len(matches) != 1:
                    raise ValueError("Click does not identify exactly one detected target")
                box = matches[0].box
            result = self._initialize_fusion_selection(snapshot, box)
            response.success = True
            response.tracking_active = True
            response.selected_target_id = result.public_id
            response.measurement_current = result.source == "yolo"
            response.message = "Fusion initialized on latest frame; use select_fusion_roi for delayed UI frames"
        except (TrackingROIError, ValueError, TypeError) as exc:
            response.message = str(exc)
        return response


    def _select_manual_color_candidate(self, candidates, frame_shape):
        """Follow the UI-selected red component by its nearest current box."""
        anchor = self._manual_color_selected_bbox
        if anchor is None:
            return None
        if not candidates:
            self._manual_color_misses += 1
            return None
        ax1, ay1, ax2, ay2 = anchor
        acx, acy = (ax1+ax2)*.5, (ay1+ay2)*.5
        height, width = frame_shape[:2]
        frame_jump = .35 * math.hypot(width, height)
        target_jump = max(48.0, 4.0 * math.hypot(ax2-ax1, ay2-ay1))
        max_jump = min(frame_jump, target_jump)
        selected = min(candidates, key=lambda item: (
            (item.aabb_xyxy[0]+item.aabb_xyxy[2])*.5-acx
        )**2 + (
            (item.aabb_xyxy[1]+item.aabb_xyxy[3])*.5-acy
        )**2)
        cx, cy = (selected.aabb_xyxy[0]+selected.aabb_xyxy[2])*.5, (selected.aabb_xyxy[1]+selected.aabb_xyxy[3])*.5
        if math.hypot(cx-acx, cy-acy) > max_jump:
            self._manual_color_misses += 1
            return None
        self._manual_color_selected_bbox = tuple(selected.aabb_xyxy)
        self._manual_color_misses = 0
        return selected

    def _publish_configured_colour_result(self, image, detections, selected, result):
        self._publish_detections(image, detections)
        if hasattr(self, "_observation_publisher"):
            self._publish_observation_array(image, detections)
        self._publish_selected_detections(image, selected)
        self._publish_manual_state(image, result, selected is not None,
                                   detection_count=len(detections))
        tracker = self._current_manual_tracker()
        evidence = dict(
            capture_header=dict(seq=image.header.seq, frame_id=image.header.frame_id,
                stamp=dict(secs=image.header.stamp.secs, nsecs=image.header.stamp.nsecs)),
            mode=self._tracking_mode,
            selected_id=int(selected.track_id) if selected is not None else -1,
            source="configured_colour", reason=result.reason,
            color_source=tracker.colour_source, color_name=tracker.colour_name,
            detection_count=len(detections),
            pipeline_epoch=getattr(self, "_epoch", 0),
            capture_segment=getattr(getattr(self, "_capture_clock", None), "segment", 0),
        )
        if hasattr(self, "_fusion_evidence_publisher"):
            self._emit(self._fusion_evidence_publisher, String(data=json.dumps(evidence, allow_nan=False)))

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
                    measurement_source="visual_tracker",
                    last_detector_stamp=rospy.Time(0),
                ),
            )
        self._publish_detections(image, detections)
        # Manual modes have no concurrent YOLO correction for this capture.
        if hasattr(self, "_observation_publisher"):
            self._publish_observation_array(image, detections)
        # FeatureTracker is a single-target ROI tracker: its valid ROI result
        # is already the selected target and must not wait for a pixel click.
        self._publish_selected_detections(image, detections[0] if detections else None)
        self._publish_manual_state(image, result, measurement_valid)
        manual_tracker = self._current_manual_tracker()
        evidence = dict(
            capture_header=dict(seq=image.header.seq, frame_id=image.header.frame_id,
                stamp=dict(secs=image.header.stamp.secs, nsecs=image.header.stamp.nsecs)),
            mode=self._tracking_mode,
            selected_id=self._manual_track_id if measurement_valid else -1,
            source="manual", reason=result.reason,
            color_source=manual_tracker.colour_source if self._tracking_mode == "color_tracker" else "",
            color_name=manual_tracker.colour_name if self._tracking_mode == "color_tracker" else "",
            pipeline_epoch=getattr(self, "_epoch", 0),
            capture_segment=getattr(getattr(self, "_capture_clock", None), "segment", 0),
        )
        if hasattr(self, "_fusion_evidence_publisher"):
            self._emit(self._fusion_evidence_publisher, String(data=json.dumps(evidence, allow_nan=False)))
        return detections

    def _publish_manual_state(
        self,
        image: Image,
        result: ManualTrackResult,
        measurement_valid: bool,
        detection_count=None,
    ) -> None:
        state = SmartTrackerState()
        state.header = copy.copy(image.header)
        state.tracking_active = self._tracking_mode != "smart_tracker"
        state.has_selection = bool(self._manual_active and result.bbox is not None)
        state.selected_track_id = self._manual_track_id if state.has_selection else -1
        state.selected_class_id = self._manual_class_id
        state.selected_class_name = "manual_roi"
        state.confidence = _finite(result.confidence)
        self._fill_bbox(state.selected_bbox, result.bbox if measurement_valid else None)
        state.geometry_type = "aabb" if measurement_valid else "none"
        state.has_oriented_bbox = False
        # This flag denotes a current YOLO measurement, not SOT success.
        state.measurement_current = False
        state.prediction_only = False
        manual_tracker = self._current_manual_tracker()
        state.tentative = bool(
            not measurement_valid
            and manual_tracker is not None
            and manual_tracker.active
        )
        state.data_is_stale = not measurement_valid
        state.control_measurement_ready = measurement_valid
        state.frames_since_detection = max(0, result.consecutive_failures)
        state.association_method = "manual_%s" % result.tracker_name
        state.freshness_reason = result.reason
        state.tracker_type = result.tracker_name
        state.backend = "opencv"
        state.device = "cpu"
        state.model_path = ""
        state.detection_count = (1 if measurement_valid else 0) if detection_count is None else int(detection_count)
        state.frame_processing_ms = _finite(self._last_processing_ms)
        self._emit(self._state_publisher, state)

    @staticmethod
    def _annotate_manual(frame: np.ndarray, result: ManualTrackResult) -> np.ndarray:
        if result.valid and result.bbox is not None:
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
                "manual tracker: LOST" if result.bbox is not None else "manual tracker: waiting for ROI",
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
        message.header = copy.copy(header)
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
        output.header = copy.copy(image.header)
        output.provenance = copy.copy(self._identity)
        fusion_output = FusionDetection2DArray()
        fusion_output.header = copy.copy(image.header)
        fusion_output.provenance = copy.copy(self._identity)
        fusion_output.image_width = image.width
        fusion_output.image_height = image.height
        for detection in detections:
            tracked = TrackedDetection2D()
            tracked.detection = self._detection_message(image.header, detection)
            tracked.track_id = int(detection.track_id)
            tracked.track_id_is_stable = bool(detection.track_id_is_stable)
            output.detections.append(tracked)
            evidence = FusionDetection2D()
            evidence.target = tracked
            evidence.measurement_source = str(getattr(detection, "measurement_source", "detector"))
            evidence.last_detector_stamp = getattr(
                detection, "last_detector_stamp",
                rospy.Time.from_sec(detection.detector_stamp) if hasattr(detection, "detector_stamp")
                else image.header.stamp if evidence.measurement_source == "detector" else rospy.Time(0))
            fusion_output.detections.append(evidence)
        self._emit(self._fusion_array_publisher, fusion_output)
        self._emit(self._tracked_publisher, output)

    def _publish_observation_array(self, image, detections):
        output = FusionDetection2DArray()
        output.header = copy.copy(image.header)
        output.provenance = copy.copy(self._identity)
        output.image_width, output.image_height = image.width, image.height
        for detection in detections:
            entry = FusionDetection2D()
            entry.target.detection = self._detection_message(image.header, detection)
            entry.target.track_id = detection.track_id
            entry.target.track_id_is_stable = detection.track_id_is_stable
            entry.measurement_source = getattr(detection, "measurement_source", "detector")
            entry.last_detector_stamp = image.header.stamp if entry.measurement_source == "detector" else rospy.Time(0)
            output.detections.append(entry)
        self._emit(self._observation_publisher, output)

    def _publish_detector_observations(self, packet):
        """One ordered capture-domain observation per inference for legacy XD.

        A high-frequency visual update/correction pair has the same capture
        stamp, and XD deliberately rejects repeats. Its bridge therefore reads
        this authoritative stream instead of racing the presentation stream.
        Never restamp an old detector box or introduce a second identity pool.
        """
        stamp = packet.image.header.stamp
        if stamp <= self._last_observation_output_stamp or packet.epoch != self._epoch:
            return
        age = (rospy.Time.now()-stamp).to_sec()
        if age < -self._future_tolerance or age > self._flow.max_age:
            return
        allowed = self._tracker.allowed_class_ids
        detections = [item for item in packet.detections
            if (allowed is None or item.class_id in allowed)
            and valid_box(self._fusion_candidate(item).box, packet.frame.shape)
            and math.isfinite(item.confidence) and 0 <= item.confidence <= 1]
        if self._tracking_mode == "fusion_tracker":
            result = self._fusion.result
            bound = {value for value in (result.public_id, result.local_id, self._fusion.pending_id) if value is not None and value >= 0}
            output = [item for item in detections if item.track_id not in bound]
            if result.observed and result.last_detection_stamp == stamp.to_sec():
                selected = next((item for item in detections if item.track_id == result.local_id), None)
                if selected is not None:
                    selected = copy.copy(selected)
                    selected.track_id, selected.class_id = result.public_id, result.class_id
                    if stamp.to_sec() == self._fusion.stamp:
                        x, y, width, height = result.box
                        selected.aabb_xyxy = (x, y, x+width, y+height)
                    output.append(selected)
        else:
            output = detections
        keep = class_duplicate_indices([self._fusion_candidate(item) for item in output], self._fusion.class_groups,
                                       self._fusion.public_id if self._fusion.result.observed else -1)
        self._publish_observation_array(packet.image, [output[index] for index in keep])
        self._last_observation_output_stamp = stamp

    @staticmethod
    def _point_to_box_distance_squared(x: float, y: float, detection) -> float:
        x1, y1, x2, y2 = detection.aabb_xyxy
        dx = max(float(x1) - x, 0.0, x - float(x2))
        dy = max(float(y1) - y, 0.0, y - float(y2))
        return dx * dx + dy * dy

    def _select_pixel_callback(self, message: PointStamped) -> None:
        """Lock the detection box nearest to a UI click in source-image pixels."""
        x = _finite(message.point.x, float("nan"))
        y = _finite(message.point.y, float("nan"))
        with self._lock:
            if getattr(self, "_model_busy", False):
                rospy.logwarn_throttle(2., "Click ignored during model switch")
                return
            if not math.isfinite(x) or not math.isfinite(y):
                rospy.logwarn("Ignoring non-finite target pixel click")
                return
            if x < 0 or y < 0:
                self._clear_selected_target()
                self._publish_empty_selected_detections()
                rospy.loginfo("UI canceled selected YOLO target with negative pixel")
                return
            if self._tracking_mode == "fusion_tracker":
                # Match the legacy click behavior: pixel coordinates choose
                # the nearest detector box in the newest cached image. The UI
                # header is not required and may be empty or delayed.
                if not self._fusion_snapshots:
                    rospy.logwarn("Fusion pixel selection rejected: no current image")
                    return
                snapshot = self._fusion_snapshots[-1]
                header, frame, candidates = snapshot
                age = (rospy.Time.now() - header.stamp).to_sec()
                if age > self._fusion_snapshot_max_age or age < -self._future_tolerance:
                    rospy.logwarn("Fusion pixel selection rejected: latest image is stale")
                    return
                height, width = frame.shape[:2]
                if x >= width or y >= height:
                    rospy.logwarn("Ignoring Fusion target pixel outside current image: (%.1f, %.1f) in %dx%d",
                                  x, y, width, height)
                    return
                if not candidates:
                    rospy.logwarn("Fusion pixel selection rejected: no current detections")
                    return

                def rank(candidate):
                    x0, y0, box_width, box_height = candidate.box
                    dx = max(float(x0) - x, 0.0, x - float(x0 + box_width))
                    dy = max(float(y0) - y, 0.0, y - float(y0 + box_height))
                    center_distance = ((float(x0 + box_width / 2.0) - x) ** 2
                                       + (float(y0 + box_height / 2.0) - y) ** 2)
                    return (dx * dx + dy * dy, center_distance, -_finite(candidate.confidence))

                candidate = min(candidates, key=rank)
                try:
                    result = self._initialize_fusion_selection(snapshot, candidate.box)
                except ValueError as exc:
                    rospy.logwarn("Fusion pixel selection rejected: %s", exc)
                    return
                rospy.loginfo("UI selected nearest Fusion box: pixel=(%.1f, %.1f) local_id=%d selected_id=%d",
                              x, y, candidate.local_id, result.public_id)
                return
            manual_tracker = self._current_manual_tracker()
            if manual_tracker is not None:
                frame = getattr(self._controller, "current_frame", None)
                if frame is None or frame.ndim < 2:
                    rospy.logwarn("Ignoring manual target click: no current image")
                    return
                height, width = frame.shape[:2]
                if x >= width or y >= height:
                    rospy.logwarn("Ignoring manual target pixel outside current image: (%.1f, %.1f) in %dx%d",
                                  x, y, width, height)
                    return
                if (self._tracking_mode == "color_tracker"
                        and manual_tracker.colour_source == "configured"):
                    candidates = tuple(self._manual_color_candidates)
                    if not candidates:
                        self._manual_color_selected_bbox = None
                        self._manual_color_misses = 0
                        self._manual_active = False
                        self._publish_empty_selected_detections()
                        rospy.logwarn("Ignoring colour target click: no matching colour boxes")
                        return
                    def rank_colour(detection):
                        distance = self._point_to_box_distance_squared(x, y, detection)
                        cx = (detection.aabb_xyxy[0] + detection.aabb_xyxy[2]) * .5
                        cy = (detection.aabb_xyxy[1] + detection.aabb_xyxy[3]) * .5
                        center_distance = (cx-x)**2 + (cy-y)**2
                        return distance, center_distance, -_finite(detection.confidence)
                    selected = min(candidates, key=rank_colour)
                    self._manual_color_selected_bbox = tuple(selected.aabb_xyxy)
                    self._manual_color_misses = 0
                    self._manual_active = True
                    rospy.loginfo("UI selected colour box: pixel=(%.1f, %.1f) bbox=%s",
                                  x, y, tuple(int(v) for v in selected.aabb_xyxy))
                    return
                side = min(20, width, height)
                x0 = max(0, min(width-side, int(round(x))-side//2))
                y0 = max(0, min(height-side, int(round(y))-side//2))
                self._manual_pending_roi = (float(x0), float(y0), float(side), float(side))
                rospy.loginfo("UI selected manual colour region: pixel=(%.1f, %.1f) ROI=(%d,%d,%d,%d) colour=%s",
                              x, y, x0, y0, side, side,
                              getattr(manual_tracker, "colour_name", ""))
                return
            width = int(self._controller.video_handler.width)
            height = int(self._controller.video_handler.height)
            if x >= width or y >= height:
                rospy.logwarn(
                    "Ignoring target pixel click outside current image: (%.1f, %.1f) in %dx%d",
                    x, y, width, height,
                )
                return
            detections = tuple(self._tracker.last_detections or ())
            if not detections:
                self._clear_selected_target()
                rospy.logwarn("Ignoring target pixel click: no current YOLO detections")
                return

            def rank(detection):
                distance = self._point_to_box_distance_squared(x, y, detection)
                center_x, center_y = detection.center_xy
                center_distance = (float(center_x) - x) ** 2 + (float(center_y) - y) ** 2
                return (distance, center_distance, -_finite(detection.confidence))

            selected = min(detections, key=rank)
            self._remember_selected_detection(selected)
            rospy.loginfo(
                "UI selected YOLO box: pixel=(%.1f, %.1f) track_id=%d stable=%s class=%d",
                x,
                y,
                int(selected.track_id),
                bool(selected.track_id_is_stable),
                int(selected.class_id),
            )

    def _remember_selected_detection(self, detection) -> None:
        self._selected_track_id = int(detection.track_id)
        self._selected_track_id_is_stable = bool(detection.track_id_is_stable)
        self._selected_class_id = int(detection.class_id)
        self._selected_last_bbox = tuple(int(value) for value in detection.aabb_xyxy)

    def _clear_selected_target(self) -> None:
        self._fusion.reset()
        self._selected_track_id = None
        self._selected_track_id_is_stable = False
        self._selected_class_id = None
        self._selected_last_bbox = None
        if hasattr(self, "_manual_color_selected_bbox"):
            self._manual_color_selected_bbox = None
            self._manual_color_misses = 0

    def _publish_empty_selected_detections(self) -> None:
        output = TrackedDetection2DArray()
        if self._last_header is not None:
            output.header = copy.copy(self._last_header)
        else:
            output.header.stamp = rospy.Time.now()
        output.provenance = copy.copy(self._identity)
        self._emit(self._selected_detections_publisher, output)

    def _selected_detection_for_frame(self, detections):
        if self._selected_track_id is None:
            return None
        if self._selected_track_id_is_stable:
            for detection in detections:
                if int(detection.track_id) == self._selected_track_id:
                    self._selected_last_bbox = tuple(
                        int(value) for value in detection.aabb_xyxy
                    )
                    return detection
            return None

        # Some backends do not expose persistent IDs. In that case, associate
        # each new YOLO result with the selected class's nearest previous box.
        candidates = [
            detection for detection in detections
            if int(detection.class_id) == self._selected_class_id
        ]
        if not candidates:
            return None
        if self._selected_last_bbox is None:
            selected = candidates[0]
        else:
            x1, y1, x2, y2 = self._selected_last_bbox
            previous_center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
            selected = min(
                candidates,
                key=lambda detection: (
                    (float(detection.center_xy[0]) - previous_center[0]) ** 2
                    + (float(detection.center_xy[1]) - previous_center[1]) ** 2
                ),
            )
        self._remember_selected_detection(selected)
        return selected

    def _publish_selected_detections(self, image: Image, detection) -> None:
        output = TrackedDetection2DArray()
        output.header = copy.copy(image.header)
        output.provenance = copy.copy(self._identity)
        if detection is not None:
            tracked = TrackedDetection2D()
            tracked.detection = self._detection_message(image.header, detection)
            tracked.track_id = int(detection.track_id)
            tracked.track_id_is_stable = bool(detection.track_id_is_stable)
            output.detections.append(tracked)
        # Empty arrays keep the gimbal idle before selection and while YOLO
        # misses the selected target.
        self._emit(self._selected_detections_publisher, output)

    @staticmethod
    def _highlight_selected_detection(frame: np.ndarray, detection) -> None:
        """Highlight the selected box on the all-detections annotated image."""
        if detection is None:
            return
        x1, y1, x2, y2 = (int(value) for value in detection.aabb_xyxy)
        colour = (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 4, cv2.LINE_AA)
        label = "SELECTED %d %.2f" % (
            int(detection.track_id), _finite(detection.confidence)
        )
        cv2.putText(
            frame,
            label,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            colour,
            1,
            cv2.LINE_AA,
        )

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
        state.header = copy.copy(image.header)
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
        self._emit(self._state_publisher, state)

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
            if getattr(self, "_model_busy", False):
                response.message = "BUSY: model switch in progress"
                return response
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
                self._clear_selected_target()
                response.success = True
                response.message = "Selection cleared"
                response.selected_target_id = -1
                return response
            if self._tracking_mode == "fusion_tracker":
                return self._select_fusion_latest(request, response)
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
            if response.success:
                selected_id = int(self._tracker.selected_object_id)
                for detection in detections:
                    if int(detection.track_id) == selected_id:
                        self._remember_selected_detection(detection)
                        break
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
            "fusion": "fusion_tracker",
            "fusiontracker": "fusion_tracker",
            "fusion_tracker": "fusion_tracker",
        }
        return aliases.get(str(value or "").strip().lower())

    def _switch_tracking_mode(self, request) -> SwitchTrackingModeResponse:
        response = SwitchTrackingModeResponse()
        requested_mode = self._normalize_tracking_mode(request.mode)
        with self._lock:
            if getattr(self, "_model_busy", False):
                response.message = "BUSY: model switch in progress"
                response.active_mode = self._tracking_mode
                return response
            if self._tracker is None:
                response.message = "SmartTracker is shutting down"
                return response
            if requested_mode is None:
                response.message = (
                    "mode must be smart_tracker, feature_tracker, color_tracker, or fusion_tracker"
                )
                response.active_mode = self._tracking_mode
                return response
            if requested_mode == "fusion_tracker" and not self._fusion_mode_enabled:
                response.message = "Fusion requires the provenance-aware xd_smart_tracker_fusion_integration.launch"
                response.active_mode = self._tracking_mode
                return response
            if requested_mode != "smart_tracker" and not self._manual_enabled:
                response.message = "ManualBoxTracker is disabled"
                response.active_mode = self._tracking_mode
                return response

            color_tracker = self._manual_trackers.get("color_tracker")
            if requested_mode == self._tracking_mode and (
                    requested_mode != "color_tracker" or color_tracker is None
                    or str(request.color_name or "").strip().lower() == color_tracker.colour_name):
                response.success = True
                response.active_mode = requested_mode
                response.message = "Mode already active; existing target retained"
                response.waiting_for_initial_roi = bool(self._current_manual_tracker() is not None
                    and self._current_manual_tracker().requires_initial_roi and not self._manual_active)
                if color_tracker is not None:
                    response.color_source, response.color_name = color_tracker.colour_source, color_tracker.colour_name
                return response
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
            self._manual_color_candidates = ()
            self._manual_color_selected_bbox = None
            self._manual_color_misses = 0
            self._tracking_mode = requested_mode
            self._invalidate_pipeline()
            self._fusion.reset()
            self._fusion_snapshots.clear()
            self._last_error = ""
            self._tracker.clear_selection()
            self._clear_selected_target()

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
            elif requested_mode == "fusion_tracker":
                response.message = "Switched to YOLO/feature fusion; select by ID, point or frame-bound ROI"
            elif response.waiting_for_initial_roi:
                response.message = "Switched to %s; waiting for initial ROI" % requested_mode
            else:
                response.message = "Switched to color_tracker; detecting '%s' in the full image" % response.color_name
            rospy.loginfo("%s", response.message)
            return response

    def _invalidate_pipeline(self):
        """Called under state lock; old in-flight CUDA results cannot commit."""
        with getattr(self, "_commit_lock", self._lock):
            self._epoch = getattr(self, "_epoch", 0) + 1
        for name in ("_inference_mailbox", "_result_mailbox"):
            mailbox = getattr(self, name, None)
            if mailbox is not None:
                mailbox.clear()
        flow = getattr(self, "_flow", None)
        if flow is not None:
            flow.reset()
        motion = getattr(self, "_camera_motion", None)
        if motion is not None:
            motion.reset()
        self._motion_header = self._motion_frame = None
        self._pipeline_current = None
        self._last_observation_output_stamp = rospy.Time(0)
        self._tracker.last_detections = []

    def _reset_temporal_state(self):
        self._invalidate_pipeline()
        self._last_capture_stamp = rospy.Time(0)
        self._last_header = None
        self._last_input_stamp = rospy.Time(0)
        self._fusion_snapshots.clear()
        for tracker in self._manual_trackers.values():
            tracker.reset()
        self._manual_pending_roi = None
        self._manual_active = False
        self._tracker.clear_selection()
        self._clear_selected_target()
        self._identity.session_uuid = "smart_" + str(uuid.uuid4())
        self._publish_empty_selected_detections()

    def _switch_model(self, request) -> SwitchSmartTrackerModelResponse:
        response = SwitchSmartTrackerModelResponse()
        raw_path = Path(str(request.model_path or "")).expanduser()
        model_path = raw_path if raw_path.is_absolute() else self._package_root / raw_path
        with self._lock:
            if self._tracker is None:
                response.message = "SmartTracker is shutting down"
                return response
            if self._model_busy:
                response.message = "BUSY: model switch in progress"
                return response
            self._model_busy = True
            self._invalidate_pipeline()
            self._clear_selected_target()
            self._tracker.clear_selection()
            self._fusion_snapshots.clear()
            self._publish_empty_selected_detections()
        # GPU load/switch is serialized with inference, but never holds the
        # image/state lock. Other commands return BUSY rather than queueing.
        try:
            with self._backend_lock:
                if request.model_sha256:
                    self._tracker.backend.authorize_model_digest(
                        str(model_path), request.model_sha256
                    )
                result = self._tracker.switch_model(str(model_path), request.device or "auto")
        except Exception as exc:
            result = {"success": False, "message": "Model switch failed: %s: %s" % (type(exc).__name__, exc)}
        with self._lock:
            self._invalidate_pipeline()
            self._model_busy = False
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", "Model switch completed"))
            runtime = self._tracker.get_runtime_info()
            response.effective_device = str(runtime.get("effective_device", runtime.get("device", "unknown")))
            response.effective_model_path = str(runtime.get("model_path", ""))
            response.class_count = len(self._tracker.labels)
            if response.success:
                self._clear_selected_target()
                self._fusion_snapshots.clear()
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
            status.add("inference_ms", getattr(self, "_inference_ms", 0.))
            status.add("pipeline_epoch", getattr(self, "_epoch", 0))
            status.add("model_switch_busy", self._model_busy)
            status.add("capture_segment", self._capture_clock.segment)
            status.add("image_frames_replaced", self._image_mailbox.replaced)
            status.add("inference_frames_replaced", self._inference_mailbox.replaced)
            status.add("camera_motion_source", self._camera_motion.source)
            status.add("camera_motion_inlier_ratio", self._camera_motion.inlier_ratio)
            status.add("tracker_config_path", getattr(tracker.backend, "_tracker_config_path", "") if tracker else "")
            status.add("frame_age_sec", frame_age if math.isfinite(frame_age) else -1.0)
            status.add("model_path", runtime.get("model_path", ""))
            status.add("effective_device", runtime.get("effective_device", ""))
            status.add("backend", runtime.get("backend", ""))
            status.add("artifact_sha256", runtime.get("artifact_sha256", ""))
            reid_runtime = runtime.get("appearance_reid") or {}
            status.add("appearance_reid_model", reid_runtime.get("model_name", "disabled"))
            status.add(
                "appearance_reid_sha256",
                (reid_runtime.get("provenance") or {}).get("sha256", ""),
            )
            status.add("appearance_reid_device", reid_runtime.get("device", ""))
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
