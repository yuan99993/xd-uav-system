#!/usr/bin/env python3
"""One YOLO runtime serving several timestamped ROS camera sources.

The node is intentionally detection-only.  It shares CUDA context, model and
pre-processing worker while preserving one DetectionArray topic per camera, so
xd_uav_track remains the owner of all temporal identities.
"""

from __future__ import annotations

import math
import resource
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, List, Tuple

import rospy
import rospkg
import cv2
import numpy as np
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from xd_uav_track.msg import DetectionArray, DetectionCandidate, TrackStateArray
from xd_uav_track.reid import AppearanceEncoder
from xd_uav_track.pixel_lock import (blend_boxes, coast_box, color_box, compatible_box, flow_box,
                                      turn_compatible_box)

from sar_yolo_detector.pixeagle.backends import DevicePreference, create_backend
from sar_yolo_detector.pixeagle.backends.tensorrt_compat import (
    prepare_fp16_engine,
    sha256_file,
)
from sar_yolo_detector.scripts_compat import image_to_bgr
from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection


def _bounded(value: Any, default: float, low: float, high: float) -> float:
    try:
        return min(high, max(low, float(value)))
    except (TypeError, ValueError):
        return default


def _vision_detection(header, detection) -> Detection2D:
    x1, y1, x2, y2 = detection.aabb_xyxy
    output = Detection2D()
    output.header = header
    output.bbox.center.x = 0.5 * (float(x1) + float(x2))
    output.bbox.center.y = 0.5 * (float(y1) + float(y2))
    output.bbox.center.theta = float(detection.rotation_deg or 0.0)
    output.bbox.size_x = max(0.0, float(x2) - float(x1))
    output.bbox.size_y = max(0.0, float(y2) - float(y1))
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.id = int(detection.class_id)
    hypothesis.score = _bounded(detection.confidence, 0.0, 0.0, 1.0)
    output.results.append(hypothesis)
    return output


def _xd_candidate(detection, width: int, height: int):
    if detection.aabb_xyxy is None:
        return None
    x1, y1, x2, y2 = detection.aabb_xyxy
    if not all(math.isfinite(float(item)) for item in (x1, y1, x2, y2,
                                                        detection.confidence)):
        return None
    x1 = max(0, min(width, int(math.floor(x1))))
    y1 = max(0, min(height, int(math.floor(y1))))
    x2 = max(0, min(width, int(math.ceil(x2))))
    y2 = max(0, min(height, int(math.ceil(y2))))
    if x2 <= x1 or y2 <= y1:
        return None
    output = DetectionCandidate()
    output.track_id = -1
    output.track_id_is_stable = False
    output.class_id = int(detection.class_id)
    output.confidence = _bounded(detection.confidence, 0.0, 0.0, 1.0)
    output.bbox = [x1, y1, x2, y2]
    output.has_bbox = True
    return output


def _box_iou(first, second) -> float:
    """Small allocation-free IoU helper used by prediction-ROI recovery."""
    ax1, ay1, ax2, ay2 = [float(value) for value in first]
    bx1, by1, bx2, by2 = [float(value) for value in second]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if intersection <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    return intersection / max(1e-6, area_a + area_b - intersection)


def _association_class_compatible(observed: int, predicted: int,
                                  compatible_ids) -> bool:
    return (observed == predicted or
            (observed in compatible_ids and predicted in compatible_ids))


class MultiSourceDetectionNode:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._reid_condition = threading.Condition()
        self._closing = False
        self._pending: Dict[str, Image] = {}
        self._pending_reid: Dict[str, Tuple[Any, DetectionArray, List[int]]] = {}
        self._recent_reid = {}
        self._reid_reused = 0
        self._sources: Dict[str, Dict[str, Any]] = {}
        self._received = 0
        self._processed = 0
        self._dropped_backlog = 0
        self._reid_processed = 0
        self._reid_dropped_backlog = 0
        self._reid_stale = 0
        self._reid_ms_ewma = 0.0
        self._inference_ms_ewma = 0.0
        self._inference_batches = 0
        self._last_batch_size = 0
        self._last_success_wall = 0.0
        self._startup_ready = False
        self._startup_error = ""
        self._startup_started_wall = time.monotonic()
        self._startup_duration_sec = 0.0
        self._tracking_lock = threading.Lock()
        self._priority_source = str(rospy.get_param(
            "~priority_source", "") or "").strip()
        self._priority_source_wall = time.monotonic() if self._priority_source else 0.0
        self._priority_source_timeout_sec = max(0.1, float(rospy.get_param(
            "~priority_source_timeout_sec", 0.75)))
        self._discovery_max_fps = max(0.0, float(rospy.get_param(
            "~discovery_max_fps", 8.0)))
        self._priority_max_fps = max(0.0, float(rospy.get_param(
            "~priority_max_fps", 12.0)))
        self._standby_max_fps = max(0.0, float(rospy.get_param(
            "~standby_max_fps", 6.0)))
        # Full-frame resolution belongs to the immutable backend configuration.
        # Keep recovery crops cheaper by default; set 0 to reuse full-frame
        # resolution when the target is exceptionally small.
        self._roi_inference_image_size = int(rospy.get_param(
            "~roi_inference_image_size", 640))
        if self._roi_inference_image_size != 0 and (
                self._roi_inference_image_size < 320 or
                self._roi_inference_image_size > 1536 or
                self._roi_inference_image_size % 32 != 0):
            raise RuntimeError("~roi_inference_image_size must be 0 or a multiple of 32 within 320..1536")
        self._tracked_boxes: Dict[str, Tuple[Tuple[int, int, int, int], int, float]] = {}
        self._pixel_track_hints = {}
        self._other_confirmed_boxes = {}
        self._pixel_previous_frames = {}
        # Pixel flow is a permanent low-confidence companion for all eligible
        # tracks. Keep the old enable parameter readable for launch-file
        # compatibility, but do not let lifecycle/config state disable it.
        if not bool(rospy.get_param("~pixel_assist_enabled", True)):
            rospy.logwarn("pixel assist is mandatory for eligible tracks; ignoring pixel_assist_enabled=false")
        self._pixel_assist_hint_timeout_sec = max(0.2, min(3.0, float(
            rospy.get_param("~pixel_assist_hint_timeout_sec", 2.2))))
        self._pixel_assist_max_frame_gap_sec = max(0.05, min(1.0, float(
            rospy.get_param("~pixel_assist_max_frame_gap_sec", 0.8))))
        self._pixel_assist_confidence = _bounded(
            rospy.get_param("~pixel_assist_confidence", 0.45), 0.45, 0.11, 0.45)
        self._pixel_assist_min_features = max(4, min(30, int(rospy.get_param(
            "~pixel_assist_min_features", 7))))
        self._pixel_assist_fb_error_px = _bounded(
            rospy.get_param("~pixel_assist_fb_error_px", 1.5), 1.5, 0.5, 5.0)
        self._pixel_assist_max_velocity_px_sec = _bounded(
            rospy.get_param("~pixel_assist_max_velocity_px_sec", 300.0),
            300.0, 30.0, 1000.0)
        self._pixel_assist_max_tracks = max(1, min(16, int(rospy.get_param(
            "~pixel_assist_max_tracks", 8))))
        self._pixel_assist_frames = 0
        self._pixel_tracking_attempts = 0
        self._pixel_tracking_valid = 0
        self._pixel_assist_rejected = 0
        self._pixel_yolo_rejected = 0
        self._pixel_color_attempts = 0
        self._pixel_color_valid = 0
        self._pixel_assist_ms_ewma = 0.0
        self._tracking_tracks_topic = str(rospy.get_param(
            "~tracking_tracks_topic", "") or "").strip()
        self._rate_limited = 0
        self._roi_attempts = 0
        self._roi_hits = 0
        self._roi_ms_ewma = 0.0
        self._warmup_frames = max(0, min(4, int(rospy.get_param(
            "~startup_warmup_frames", 1))))
        self._warmup_width = max(32, int(rospy.get_param("~warmup_width", 640)))
        self._warmup_height = max(32, int(rospy.get_param("~warmup_height", 640)))
        self._batch_wait_sec = max(0.0, min(0.010, float(rospy.get_param(
            "~batch_wait_ms", 4.0)) / 1000.0))
        self._maximum_batch_size = max(1, min(8, int(rospy.get_param(
            "~maximum_batch_size", 2))))
        self._maximum_age_sec = max(0.0, float(rospy.get_param(
            "~maximum_capture_age_sec", 0.5)))
        self._require_stamp = bool(rospy.get_param("~require_capture_timestamp", True))
        self._reject_out_of_order = bool(rospy.get_param("~reject_out_of_order", True))
        self._pause_without_subscribers = bool(rospy.get_param(
            "~pause_without_subscribers", True))
        self._package_root = Path(rospkg.RosPack().get_path(
            "sar_yolo_detector")).resolve()
        config: Dict[str, Any] = dict(rospy.get_param("~SmartTracker", {}))
        self._allowed_class_ids = frozenset(
            int(value) for value in config.get(
                "SMART_TRACKER_ALLOWED_CLASS_IDS", [])
            if isinstance(value, int) and not isinstance(value, bool) and value >= 0)
        self._compatible_class_ids = frozenset(
            int(value) for value in config.get(
                "SMART_TRACKER_COMPATIBLE_CLASS_IDS", [])
            if isinstance(value, int) and not isinstance(value, bool) and value >= 0)
        config["SMART_TRACKER_MODELS_ROOT"] = str(self._package_root)
        for key in ("SMART_TRACKER_GPU_MODEL_PATH", "SMART_TRACKER_CPU_MODEL_PATH"):
            value = str(config.get(key, "") or "").strip()
            if value and not Path(value).expanduser().is_absolute():
                config[key] = str((self._package_root / value).resolve())
        config["TRACKER_TYPE"] = "detection_only"
        inference_backend = str(rospy.get_param(
            "~inference_backend", "tensorrt_fp16") or "tensorrt_fp16").strip().lower()
        model_path = str(rospy.get_param(
            "~model_path", config.get("SMART_TRACKER_GPU_MODEL_PATH", "")) or "").strip()
        engine_path = str(rospy.get_param("~engine_path", "") or "").strip()
        auto_export_engine = False
        auto_engine_options = None
        selected_explicit_engine = False
        if inference_backend in {"tensorrt", "tensorrt_fp16", "engine"}:
            engine_sha256 = str(rospy.get_param("~engine_sha256", "") or "").strip()
            expected_model_sha256 = str(config.get(
                "SMART_TRACKER_MODEL_SHA256", "") or "").strip()
            engine_source_sha256 = str(rospy.get_param(
                "~engine_source_model_sha256", "") or "").strip().lower()
            auto_export_engine = bool(rospy.get_param("~auto_export_engine", True))
            auto_engine_options = {
                "cache_dir": Path(str(rospy.get_param(
                    "~engine_cache_dir", "~/.cache/sar_yolo_detector/tensorrt"))),
                "image_size": int(rospy.get_param(
                    "~engine_image_size", config.get("SMART_TRACKER_INFERENCE_IMAGE_SIZE", 640))),
                "workspace_gib": float(rospy.get_param("~engine_workspace_gib", 0.5)),
                "optimization_level": int(rospy.get_param(
                    "~engine_builder_optimization_level", 1)),
                "expected_model_sha256": expected_model_sha256,
            }
            if model_path.lower().endswith(".engine") and not engine_path:
                requested_model = model_path
                engine_sha256 = engine_sha256 or expected_model_sha256
                if not engine_sha256:
                    raise RuntimeError("~engine_sha256 or model SHA-256 is required for an engine")
                observed_engine_sha256 = sha256_file(Path(requested_model).expanduser())
                if observed_engine_sha256 != engine_sha256.lower():
                    raise RuntimeError("TensorRT engine SHA-256 does not match its configured digest")
                engine_sha256 = observed_engine_sha256
            else:
                source_model_sha256 = sha256_file(Path(model_path).expanduser())
                if (expected_model_sha256 and
                        source_model_sha256 != expected_model_sha256.lower()):
                    raise RuntimeError(
                        "source model SHA-256 does not match SMART_TRACKER_MODEL_SHA256"
                    )
                if engine_path and Path(engine_path).expanduser().is_file():
                    if not engine_sha256:
                        raise RuntimeError(
                            "~engine_sha256 is required when an explicit engine_path exists"
                        )
                    observed_engine_sha256 = sha256_file(Path(engine_path).expanduser())
                    if observed_engine_sha256 != engine_sha256.lower():
                        raise RuntimeError("TensorRT engine SHA-256 does not match its configured digest")
                    if engine_source_sha256 == source_model_sha256:
                        requested_model = engine_path
                        engine_sha256 = observed_engine_sha256
                        selected_explicit_engine = True
                    else:
                        rospy.logwarn(
                            "configured TensorRT engine is not tagged for the selected .pt model; "
                            "will build/reuse a matching engine cache entry"
                        )
                        requested_model = ""
                else:
                    requested_model = ""
                if not requested_model:
                    if not auto_export_engine:
                        raise RuntimeError(
                            "matching TensorRT engine is unavailable and ~auto_export_engine is false"
                        )
                    rospy.loginfo(
                        "preparing cached TensorRT FP16 engine for %s (imgsz=%d)",
                        model_path, auto_engine_options["image_size"])
                    requested_path, engine_sha256 = prepare_fp16_engine(
                        Path(model_path), auto_engine_options["cache_dir"],
                        image_size=auto_engine_options["image_size"],
                        workspace_gib=auto_engine_options["workspace_gib"],
                        optimization_level=auto_engine_options["optimization_level"],
                        expected_model_sha256=auto_engine_options["expected_model_sha256"],
                    )
                    requested_model = str(requested_path)
                    rospy.loginfo("using TensorRT FP16 engine %s", requested_model)
            config["SMART_TRACKER_MODEL_SHA256_BY_NAME"] = {
                Path(requested_model).name: engine_sha256.lower()
            }
        else:
            requested_model = model_path
        if not requested_model:
            raise RuntimeError("~model_path or SmartTracker GPU model path is required")
        self._backend = create_backend("ultralytics", config=config)
        if not self._backend.is_available:
            raise RuntimeError("Ultralytics backend is unavailable on this host")
        device_preference = (
            DevicePreference.CUDA if bool(rospy.get_param("~use_gpu", True))
            else DevicePreference.CPU
        )
        allow_cpu_fallback = bool(rospy.get_param("~fallback_to_cpu", False))
        try:
            self._runtime = self._backend.load_model(
                requested_model,
                device_preference,
                fallback_enabled=allow_cpu_fallback,
                context="multi_source_detection_startup",
            )
        except RuntimeError as engine_load_error:
            can_rebuild = (
                selected_explicit_engine and auto_export_engine and
                model_path.lower().endswith(".pt") and auto_engine_options is not None
            )
            if not can_rebuild:
                raise
            rospy.logwarn(
                "prebuilt TensorRT engine could not load on this host (%s); "
                "building/reusing a host-matched engine from the verified .pt model",
                engine_load_error)
            requested_path, engine_sha256 = prepare_fp16_engine(
                Path(model_path), auto_engine_options["cache_dir"],
                image_size=auto_engine_options["image_size"],
                workspace_gib=auto_engine_options["workspace_gib"],
                optimization_level=auto_engine_options["optimization_level"],
                expected_model_sha256=auto_engine_options["expected_model_sha256"],
            )
            requested_model = str(requested_path)
            config["SMART_TRACKER_MODEL_SHA256_BY_NAME"] = {
                requested_path.name: engine_sha256
            }
            self._backend = create_backend("ultralytics", config=config)
            self._runtime = self._backend.load_model(
                requested_model,
                device_preference,
                fallback_enabled=allow_cpu_fallback,
                context="multi_source_detection_host_matched_engine",
            )
        self._confidence = _bounded(config.get("SMART_TRACKER_CONFIDENCE_THRESHOLD", 0.25),
                                    0.25, 0.0, 1.0)
        self._iou = _bounded(config.get("SMART_TRACKER_IOU_THRESHOLD", 0.45),
                             0.45, 0.0, 1.0)
        self._maximum_detections = max(1, int(config.get("SMART_TRACKER_MAX_DETECTIONS", 100)))
        self._maximum_bbox_area_fraction = _bounded(
            rospy.get_param("~maximum_bbox_area_fraction", 1.0), 1.0, 0.01, 1.0)
        self._oversized_detections = 0
        self._masked_detections = 0

        raw_sources = rospy.get_param("~sources", [])
        if not isinstance(raw_sources, list) or len(raw_sources) < 2:
            raise RuntimeError("~sources must contain at least two camera mappings")
        reid_config = dict(rospy.get_param("~reid", {}))
        # Read once: this runs in the per-frame ReID worker, where parameter
        # server RPCs would add avoidable jitter.
        self._minimum_embedding_quality = max(0.0, min(1.0, float(
            reid_config.get("minimum_embedding_quality", 0.04))))
        self._reid_cache_age_sec = max(0.1, min(1.0, float(
            reid_config.get("maximum_async_feature_age_sec", 0.75))))
        self._encoders: Dict[str, AppearanceEncoder] = {}
        self._source_name_by_image_source: Dict[str, str] = {}
        for raw in raw_sources:
            if not isinstance(raw, dict):
                raise RuntimeError("every ~sources entry must be a mapping")
            name = str(raw.get("name", "")).strip()
            input_topic = str(raw.get("input_image_topic", "")).strip()
            xd_topic = str(raw.get("xd_detections_topic", "")).strip()
            if not name or not input_topic or not xd_topic or name in self._sources:
                raise RuntimeError("each source needs unique name, input_image_topic and xd_detections_topic")
            image_source = str(raw.get("image_source", name) or name).strip()
            if image_source in self._source_name_by_image_source:
                raise RuntimeError("each source needs a unique image_source")
            self._source_name_by_image_source[image_source] = name
            profile = str(raw.get("reid_model_profile", "") or "").strip()
            enabled = bool(raw.get("reid_enabled", False))
            if enabled and profile not in self._encoders:
                profile_config = dict(reid_config)
                if profile:
                    profile_config["active_model_profile"] = profile
                self._encoders[profile] = AppearanceEncoder(profile_config)
            source = dict(raw)
            source["last_stamp"] = rospy.Time(0)
            source["xd_publisher"] = rospy.Publisher(xd_topic, DetectionArray, queue_size=1)
            vision_topic = str(raw.get("detections_topic", "") or "").strip()
            source["vision_publisher"] = (rospy.Publisher(
                vision_topic, Detection2DArray, queue_size=1) if vision_topic else None)
            source["reid_enabled"] = enabled
            source["reid_profile"] = profile
            source["maximum_reid_rois"] = max(1, int(raw.get("maximum_reid_rois_per_frame", 32)))
            source["last_accept_wall"] = 0.0
            source["roi_recovery_enabled"] = bool(raw.get(
                "roi_recovery_enabled", name == "fixed"))
            source["roi_padding_ratio"] = max(0.25, min(3.0, float(raw.get(
                "roi_padding_ratio", 1.0))))
            source["roi_target_short_side"] = max(16, int(raw.get(
                "roi_target_short_side", 56)))
            source["roi_min_padding_ratio"] = max(0.20, min(1.0, float(raw.get(
                "roi_min_padding_ratio", 0.35))))
            source["roi_max_padding_ratio"] = max(
                source["roi_min_padding_ratio"], min(2.0, float(raw.get(
                    "roi_max_padding_ratio", source["roi_padding_ratio"]))))
            source["roi_min_short_side"] = max(8, int(raw.get(
                "roi_min_short_side", 24)))
            area = raw.get("detection_area", [0.0, 0.0, 1.0, 1.0])
            if (not isinstance(area, (list, tuple)) or len(area) != 4 or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                        not math.isfinite(v) for v in area) or
                    not (0.0 <= area[0] < area[2] <= 1.0 and
                         0.0 <= area[1] < area[3] <= 1.0)):
                raise RuntimeError("source %s detection_area must be [x_min,y_min,x_max,y_max] in [0,1]" % name)
            source["detection_area"] = tuple(float(v) for v in area)
            source["subscriber"] = rospy.Subscriber(
                input_topic, Image, lambda message, key=name: self._image_callback(key, message),
                queue_size=1, buff_size=16 * 1024 * 1024, tcp_nodelay=True)
            self._sources[name] = source
        self._priority_source = self._source_name_by_image_source.get(
            self._priority_source, self._priority_source)
        self._tracking_subscriber = None
        if self._tracking_tracks_topic:
            self._tracking_subscriber = rospy.Subscriber(
                self._tracking_tracks_topic, TrackStateArray,
                self._tracking_callback, queue_size=1, tcp_nodelay=True)
        self._worker = threading.Thread(target=self._worker_loop,
                                        name="multi_source_yolo", daemon=True)
        self._reid_worker = None
        if self._encoders:
            self._reid_worker = threading.Thread(
                target=self._reid_worker_loop,
                name="multi_source_reid", daemon=True)
            self._reid_worker.start()
        self._worker.start()
        self._diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=2)
        self._diagnostics_timer = rospy.Timer(
            rospy.Duration(1.0), self._publish_diagnostics)
        rospy.on_shutdown(self.close)
        rospy.loginfo("shared YOLO ready: model=%s backend=%s device=%s sources=%s batch_wait_ms=%.1f discovery_fps=%.1f priority_fps=%.1f standby_fps=%.1f roi=%s roi_imgsz=%s",
                      self._runtime.get("model_path", requested_model),
                      self._runtime.get("backend", inference_backend),
                      self._runtime.get("effective_device", "unknown"),
                      ",".join(sorted(self._sources)), self._batch_wait_sec * 1000.0,
                      self._discovery_max_fps, self._priority_max_fps,
                      self._standby_max_fps,
                      "on" if any(source["roi_recovery_enabled"]
                                  for source in self._sources.values()) else "off",
                      self._roi_inference_image_size or "default")

    def close(self) -> None:
        with self._condition:
            if self._closing:
                return
            self._closing = True
            self._condition.notify_all()
        with self._reid_condition:
            self._reid_condition.notify_all()
        if self._worker is not threading.current_thread():
            self._worker.join(timeout=3.0)
        if (self._reid_worker is not None and
                self._reid_worker is not threading.current_thread()):
            self._reid_worker.join(timeout=3.0)
        if (not self._worker.is_alive() and
                (self._reid_worker is None or not self._reid_worker.is_alive())):
            self._backend.unload_model()
            for encoder in self._encoders.values():
                encoder.close()

    def _image_callback(self, source_name: str, message: Image) -> None:
        source = self._sources[source_name]
        self._received += 1
        if self._pause_without_subscribers:
            vision = source["vision_publisher"]
            if (source["xd_publisher"].get_num_connections() +
                    (vision.get_num_connections() if vision is not None else 0) == 0):
                return
        if not self._source_rate_available(source_name, source):
            self._rate_limited += 1
            return
        stamp = message.header.stamp
        if self._require_stamp and stamp.is_zero():
            rospy.logwarn_throttle(2.0, "shared YOLO rejected zero image timestamp")
            return
        now = rospy.Time.now()
        if (not stamp.is_zero() and not now.is_zero() and self._maximum_age_sec > 0.0 and
                (now - stamp).to_sec() > self._maximum_age_sec):
            return
        with self._condition:
            previous = source["last_stamp"]
            if self._reject_out_of_order and not stamp.is_zero() and not previous.is_zero() and stamp <= previous:
                rospy.logwarn_throttle(2.0, "shared YOLO rejected out-of-order image from %s", source_name)
                return
            if source_name in self._pending:
                self._dropped_backlog += 1
            self._pending[source_name] = message
            if not stamp.is_zero():
                source["last_stamp"] = stamp
            self._condition.notify()

    def _tracking_callback(self, message: TrackStateArray) -> None:
        """Feed only source priority and prediction ROI hints from the tracker.

        This is a one-way scheduling hint.  Detection remains authoritative for
        observations and the tracker continues to own identity and lifecycle.
        """
        image_source = str(message.image_source or "").strip()
        source = self._source_name_by_image_source.get(image_source)
        if source is None:
            return
        selected = None
        hints = []
        now_wall = time.monotonic()
        for track in message.tracks:
            if self._allowed_class_ids and int(track.class_id) not in self._allowed_class_ids:
                continue
            x1, y1, x2, y2 = [int(value) for value in track.bbox]
            if x2 <= x1 or y2 <= y1:
                continue
            if track.selected:
                selected = ((x1, y1, x2, y2), int(track.class_id), now_wall)
            if (track.selected or
                    (track.lifecycle_state in (
                        "tentative", "confirmed", "occluded", "lost")
                     and track.frames_since_detection <= 6)) and \
                    track.association_method not in (
                        "stitch_pending", "stitch_ambiguous", "ambiguous",
                        "reacquiring"):
                hints.append((track.selected, float(track.tracking_quality),
                              ((x1, y1, x2, y2), int(track.class_id),
                               int(track.track_id),
                               float(message.header.stamp.to_sec()),
                               now_wall,
                               tuple(float(v) for v in track.image_velocity),
                               track.lifecycle_state)))
        hints.sort(key=lambda item: (not item[0], -item[1]))
        pixel_hints = [item[2] for item in hints[:self._pixel_assist_max_tracks]]
        with self._tracking_lock:
            if selected is not None:
                self._priority_source = source
                self._priority_source_wall = time.monotonic()
                self._tracked_boxes[source] = selected
            if not pixel_hints:
                self._pixel_track_hints.pop(source, None)
            else:
                self._pixel_track_hints[source] = pixel_hints
            self._other_confirmed_boxes[source] = [
                (int(track.track_id), tuple(int(value) for value in track.bbox))
                for track in message.tracks
                if track.lifecycle_state == "confirmed"]

    def _current_priority_source(self) -> str:
        with self._tracking_lock:
            if (self._priority_source and
                    time.monotonic() - self._priority_source_wall <=
                    self._priority_source_timeout_sec):
                return self._priority_source
            self._priority_source = ""
            return ""

    def _source_rate_available(self, source_name: str, source: Dict[str, Any]) -> bool:
        priority = self._current_priority_source()
        if priority:
            limit = self._priority_max_fps if source_name == priority else self._standby_max_fps
        else:
            limit = self._discovery_max_fps
        if limit <= 0.0:
            return True
        now = time.monotonic()
        last = float(source.get("last_accept_wall", 0.0))
        if last > 0.0 and now - last < 1.0 / limit:
            return False
        source["last_accept_wall"] = now
        return True

    def _worker_loop(self) -> None:
        if not self._run_startup_warmup():
            return
        while not rospy.is_shutdown():
            with self._condition:
                while not self._pending and not self._closing and not rospy.is_shutdown():
                    self._condition.wait(timeout=0.25)
                if self._closing or rospy.is_shutdown():
                    return
                # A short collection window batches simultaneously-arriving cameras;
                # latest-only semantics keep latency bounded under overload.
                deadline = time.monotonic() + self._batch_wait_sec
                while (len(self._pending) < self._maximum_batch_size and
                       time.monotonic() < deadline and not self._closing):
                    self._condition.wait(timeout=max(0.0, deadline - time.monotonic()))
                priority = self._current_priority_source()
                items = sorted(
                    self._pending.items(),
                    key=lambda item: (0 if priority and item[0] == priority else 1)
                )[:self._maximum_batch_size]
                for name, _ in items:
                    self._pending.pop(name, None)
            try:
                self._process_batch(items)
            except Exception as error:
                rospy.logerr_throttle(1.0, "shared YOLO processing failed: %s", error)

    def _process_batch(self, items: List[Tuple[str, Image]]) -> None:
        decoded = [(name, message, image_to_bgr(message)) for name, message in items]
        frames = [item[2] for item in decoded]
        same_shape = len({tuple(frame.shape) for frame in frames}) == 1
        start = time.perf_counter()
        if len(frames) > 1 and same_shape:
            results = self._backend.detect_many(frames, self._confidence, self._iou,
                                                self._maximum_detections)
        else:
            results = [self._backend.detect(frame, self._confidence, self._iou,
                                            self._maximum_detections) for frame in frames]
        inference_ms = 1000.0 * (time.perf_counter() - start)
        self._inference_batches += 1
        self._last_batch_size = len(items)
        self._inference_ms_ewma = (
            inference_ms if self._inference_batches == 1 else
            self._inference_ms_ewma + 0.10 *
            (inference_ms - self._inference_ms_ewma))
        for (name, message, frame), (_, detections) in zip(decoded, results):
            now = rospy.Time.now()
            if (not message.header.stamp.is_zero() and not now.is_zero() and
                    self._maximum_age_sec > 0.0 and
                    (now - message.header.stamp).to_sec() > self._maximum_age_sec):
                continue
            self._publish(name, message, frame, detections)
            self._processed += 1
            self._last_success_wall = time.monotonic()
        rospy.loginfo_throttle(10.0, "shared YOLO received=%d processed=%d backlog_drop=%d batch=%d inference_ms=%.2f",
                               self._received, self._processed, self._dropped_backlog,
                               len(items), inference_ms)

    def _publish(self, name: str, message: Image, frame, detections) -> None:
        source = self._sources[name]
        detections = self._filter_oversized(detections, message.width, message.height)
        detections = self._recover_with_prediction_roi(name, frame, detections)
        detections = self._filter_oversized(detections, message.width, message.height)
        detections = self._filter_detection_area(
            source, detections, message.width, message.height)
        detections, pixel_assists = self._add_pixel_assist(
            name, message, frame, detections)
        detections = self._filter_detection_area(
            source, detections, message.width, message.height)
        xd_output = DetectionArray()
        xd_output.header = message.header
        xd_output.image_width = message.width
        xd_output.image_height = message.height
        xd_output.image_source = str(source.get("image_source", name))
        xd_output.sensor_id = str(source.get("sensor_id", name))
        xd_output.detector_name = str(source.get("detector_name", "sar_yolo_detector"))
        xd_output.model_version = str(source.get("model_version", "unknown"))
        vision = Detection2DArray() if source["vision_publisher"] is not None else None
        if vision is not None:
            vision.header = message.header
        pixel_assist_ids = {id(item) for item in pixel_assists}
        pixel_candidate_indices = set()
        for detection in detections:
            if detection.aabb_xyxy is None:
                continue
            is_pixel_assist = id(detection) in pixel_assist_ids
            if vision is not None and not is_pixel_assist:
                vision.detections.append(_vision_detection(message.header, detection))
            candidate = _xd_candidate(detection, int(message.width), int(message.height))
            if candidate is not None:
                xd_output.candidates.append(candidate)
                if is_pixel_assist:
                    pixel_candidate_indices.add(len(xd_output.candidates) - 1)
        if vision is not None:
            source["vision_publisher"].publish(vision)
        reid_indices = [index for index in range(len(xd_output.candidates))
                        if index not in pixel_candidate_indices]
        if source["reid_enabled"] and reid_indices:
            self._attach_recent_reid(name, xd_output, reid_indices)
        # Tracking observations must never wait for a slow appearance model.
        # The worker caches descriptors for a later geometrically matched
        # frame, instead of dropping this DetectionArray on ReID timeout.
        source["xd_publisher"].publish(xd_output)
        if not source["reid_enabled"] or not reid_indices:
            return
        # ReID is intentionally decoupled from YOLO. One newest job is retained
        # per source so a crowded frame cannot stall the next detector batch.
        with self._reid_condition:
            if name in self._pending_reid:
                self._reid_dropped_backlog += 1
            self._pending_reid[name] = (frame, xd_output, reid_indices)
            self._reid_condition.notify()

    def _filter_detection_area(self, source, detections, width, height):
        """Optional per-camera valid image region; benchmark masks are not global."""
        x_min, y_min, x_max, y_max = source.get(
            "detection_area", (0.0, 0.0, 1.0, 1.0))
        if (x_min, y_min, x_max, y_max) == (0.0, 0.0, 1.0, 1.0):
            return detections
        kept = []
        for item in detections:
            if item.aabb_xyxy is None:
                continue
            x1, y1, x2, y2 = item.aabb_xyxy
            cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
            if (x_min * width <= cx <= x_max * width and
                    y_min * height <= cy <= y_max * height):
                kept.append(item)
            else:
                self._masked_detections += 1
        return kept

    def _attach_recent_reid(self, name, output, candidate_indices):
        with self._reid_condition:
            cached = self._recent_reid.get(name)
        if cached is None:
            return
        age = output.header.stamp.to_sec() - cached["stamp"]
        if not 0.0 < age <= self._reid_cache_age_sec:
            return
        proposals = []
        for index in candidate_indices:
            candidate = output.candidates[index]
            scores = sorted((
                (_box_iou(candidate.bbox, entry["bbox"]), position)
                for position, entry in enumerate(cached["entries"])
                if _association_class_compatible(
                    int(candidate.class_id), entry["class_id"],
                    self._compatible_class_ids)), reverse=True)
            if not scores or scores[0][0] < 0.45:
                continue
            if len(scores) > 1 and scores[0][0] - scores[1][0] < 0.15:
                continue
            proposals.append((scores[0][0], index, scores[0][1]))
        used_candidates, used_entries = set(), set()
        for _, index, entry_index in sorted(proposals, reverse=True):
            if index in used_candidates or entry_index in used_entries:
                continue
            output.candidates[index].appearance_embedding = \
                cached["entries"][entry_index]["embedding"]
            used_candidates.add(index)
            used_entries.add(entry_index)
            self._reid_reused += 1

    @staticmethod
    def _box_center_distance(first, second) -> float:
        first_x = 0.5 * (float(first[0]) + float(first[2]))
        first_y = 0.5 * (float(first[1]) + float(first[3]))
        second_x = 0.5 * (float(second[0]) + float(second[2]))
        second_y = 0.5 * (float(second[1]) + float(second[3]))
        return math.hypot(first_x - second_x, first_y - second_y)

    def _matching_detection(self, detections, box, class_id):
        box_width = max(1.0, float(box[2] - box[0]))
        box_height = max(1.0, float(box[3] - box[1]))
        center_gate = max(8.0, 0.45 * math.hypot(box_width, box_height))
        best = None
        best_score = -1.0
        for item in detections:
            if item.aabb_xyxy is None or not _association_class_compatible(
                    int(item.class_id), int(class_id),
                    self._compatible_class_ids):
                continue
            overlap = _box_iou(box, item.aabb_xyxy)
            distance = self._box_center_distance(box, item.aabb_xyxy)
            if overlap >= 0.15 or distance <= center_gate:
                score = max(overlap, 1.0 - min(1.0, distance / center_gate))
                if score > best_score:
                    best, best_score = item, score
        return best

    def _flow_bbox(self, previous_gray, current_gray, box, dt_sec,
                   image_velocity=None):
        return flow_box(previous_gray, current_gray, box, dt_sec,
                        self._pixel_assist_min_features,
                        self._pixel_assist_fb_error_px,
                        self._pixel_assist_max_velocity_px_sec,
                        image_velocity)

    def _add_pixel_assist(self, name, message, frame, detections):
        with self._tracking_lock:
            hints = tuple(self._pixel_track_hints.get(name, ()))
            confirmed_boxes = tuple(self._other_confirmed_boxes.get(name, ()))
        if not hints:
            self._pixel_previous_frames.pop(name, None)
            return detections, ()
        current_stamp = float(message.header.stamp.to_sec())
        current_wall = time.monotonic()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        previous = self._pixel_previous_frames.get(name)
        previous_boxes = previous.get("boxes", {}) if previous is not None else {}
        dt_sec = current_stamp - previous["stamp"] if previous is not None else 0.0
        within_gap = 0.0 < dt_sec <= self._pixel_assist_max_frame_gap_sec
        next_boxes = {}
        assists = []
        claimed = set()
        for box, class_id, track_id, hint_stamp, hint_wall, velocity, lifecycle in hints:
            elapsed = current_stamp - hint_stamp if current_stamp > 0.0 else 0.0
            if (current_wall - hint_wall > self._pixel_assist_hint_timeout_sec or
                    not -1e-3 <= elapsed <= self._pixel_assist_hint_timeout_sec):
                continue
            next_box = box
            moved = None
            if within_gap and track_id in previous_boxes:
                self._pixel_tracking_attempts += 1
                start = time.perf_counter()
                moved = self._flow_bbox(previous["gray"], gray,
                                        previous_boxes[track_id], dt_sec,
                                        velocity)
                if moved is None and previous.get("frame") is not None:
                    self._pixel_color_attempts += 1
                    moved = color_box(previous["frame"], frame,
                                      previous_boxes[track_id], dt_sec)
                    if moved is not None:
                        self._pixel_color_valid += 1
                elapsed_ms = 1000.0 * (time.perf_counter() - start)
                self._pixel_assist_ms_ewma = (
                    elapsed_ms if self._pixel_tracking_attempts == 1 else
                    self._pixel_assist_ms_ewma + 0.10 *
                    (elapsed_ms - self._pixel_assist_ms_ewma))
            if moved is not None:
                self._pixel_tracking_valid += 1
                available = [item for item in detections if id(item) not in claimed]
                matched = self._matching_detection(available, moved, class_id)
                belongs_to_other = (matched is not None and any(
                    other_id != track_id and
                    _box_iou(matched.aabb_xyxy, other_box) >= 0.15
                    for other_id, other_box in confirmed_boxes))
                strict_match = (matched is not None and
                                compatible_box(moved, matched.aabb_xyxy))
                turn_match = (matched is not None and not belongs_to_other and
                              turn_compatible_box(moved, matched.aabb_xyxy))
                agrees = strict_match or turn_match
                next_box = moved
                if agrees:
                    next_box = blend_boxes(moved, matched.aabb_xyxy,
                                           detector_weight=0.35 if strict_match
                                           else 0.55)
                    corrected = replace(
                        matched, aabb_xyxy=next_box,
                        center_xy=(int(0.5 * (next_box[0] + next_box[2])),
                                   int(0.5 * (next_box[1] + next_box[3]))))
                    detections = [corrected if item is matched else item
                                  for item in detections]
                    claimed.add(id(corrected))
                else:
                    if matched is not None and lifecycle in (
                            "confirmed", "occluded", "lost"):
                        if not belongs_to_other:
                            detections = [item for item in detections
                                          if item is not matched]
                            self._pixel_yolo_rejected += 1
                    collides = any(item.aabb_xyxy is not None and
                                   _box_iou(moved, item.aabb_xyxy) >= 0.15
                                   for item in detections)
                    collides = collides or any(
                        other_id != track_id and
                        _box_iou(moved, other_box) >= 0.35
                        for other_id, other_box in confirmed_boxes)
                    if not collides:
                        center = (int(0.5 * (moved[0] + moved[2])),
                                  int(0.5 * (moved[1] + moved[3])))
                        synthetic = NormalizedDetection(
                            track_id=-1, track_id_is_stable=False,
                            class_id=int(class_id),
                            confidence=self._pixel_assist_confidence,
                            aabb_xyxy=moved, center_xy=center)
                        detections = list(detections) + [synthetic]
                        assists.append(synthetic)
                        claimed.add(id(synthetic))
                        self._pixel_assist_frames += 1
                    else:
                        self._pixel_assist_rejected += 1
            else:
                if within_gap and track_id in previous_boxes:
                    self._pixel_assist_rejected += 1
                available = [item for item in detections if id(item) not in claimed]
                # Lead is a bounded association hint only: it never creates a
                # box or a new ID without current image evidence.
                lead = coast_box(box, velocity, min(0.25, max(0.0, elapsed)),
                                 frame.shape)
                match_box = lead if lead is not None else box
                matched = self._matching_detection(available, match_box, class_id)
                belongs_to_other = (matched is not None and any(
                    other_id != track_id and
                    _box_iou(matched.aabb_xyxy, other_box) >= 0.15
                    for other_id, other_box in confirmed_boxes))
                if matched is not None and (compatible_box(
                        match_box, matched.aabb_xyxy) or
                        (not belongs_to_other and
                         turn_compatible_box(match_box, matched.aabb_xyxy))):
                    next_box = tuple(matched.aabb_xyxy)
                    claimed.add(id(matched))
            next_boxes[track_id] = next_box
        self._pixel_previous_frames[name] = {
            "gray": gray, "frame": frame, "stamp": current_stamp,
            "boxes": next_boxes,
        }
        return detections, tuple(assists)

    def _filter_oversized(self, detections, width: int, height: int):
        if self._maximum_bbox_area_fraction >= 1.0:
            return detections
        maximum_area = self._maximum_bbox_area_fraction * max(1, width * height)
        accepted = []
        for item in detections:
            if item.aabb_xyxy is None:
                continue
            x1, y1, x2, y2 = item.aabb_xyxy
            area = max(0.0, float(x2 - x1)) * max(0.0, float(y2 - y1))
            if area > maximum_area:
                self._oversized_detections += 1
            else:
                accepted.append(item)
        return accepted

    def _recover_with_prediction_roi(self, name: str, frame, detections):
        source = self._sources[name]
        if not source.get("roi_recovery_enabled", False):
            return detections
        with self._tracking_lock:
            hint = self._tracked_boxes.get(name)
        if hint is None:
            return detections
        predicted_box, predicted_class, hint_wall = hint
        if time.monotonic() - hint_wall > self._priority_source_timeout_sec:
            return detections
        if any(_association_class_compatible(
                   int(item.class_id), predicted_class,
                   self._compatible_class_ids) and
               _box_iou(predicted_box, item.aabb_xyxy) >= 0.20
               for item in detections if item.aabb_xyxy is not None):
            return detections
        height, width = int(frame.shape[0]), int(frame.shape[1])
        x1, y1, x2, y2 = predicted_box
        box_width, box_height = max(1, x2 - x1), max(1, y2 - y1)
        nominal_padding = float(source.get("roi_padding_ratio", 1.0))
        minimum_padding = float(source.get("roi_min_padding_ratio", 0.35))
        maximum_padding = float(source.get("roi_max_padding_ratio", nominal_padding))
        target_short_side = float(source.get("roi_target_short_side", 56))
        # A small target benefits from a tighter crop: the detector still gets
        # the complete frame on every scheduled full-frame pass, while this
        # recovery pass spends pixels only around the predicted target.
        adaptive_padding = 0.5 * (
            min(box_width, box_height) / max(1.0, target_short_side) - 1.0)
        padding = max(minimum_padding, min(maximum_padding, adaptive_padding))
        left = max(0, int(round(x1 - padding * box_width)))
        top = max(0, int(round(y1 - padding * box_height)))
        right = min(width, int(round(x2 + padding * box_width)))
        bottom = min(height, int(round(y2 + padding * box_height)))
        if right - left < source.get("roi_min_short_side", 24) or \
                bottom - top < source.get("roi_min_short_side", 24):
            return detections
        self._roi_attempts += 1
        start = time.perf_counter()
        try:
            _, roi_detections = self._backend.detect(
                frame[top:bottom, left:right], self._confidence, self._iou,
                self._maximum_detections,
                self._roi_inference_image_size or None)
        except Exception as error:
            rospy.logwarn_throttle(2.0, "prediction ROI recovery failed for %s: %s",
                                   name, error)
            return detections
        elapsed_ms = 1000.0 * (time.perf_counter() - start)
        self._roi_ms_ewma = elapsed_ms if self._roi_attempts == 1 else \
            self._roi_ms_ewma + 0.10 * (elapsed_ms - self._roi_ms_ewma)
        recovered = []
        for item in roi_detections:
            if item.aabb_xyxy is None:
                continue
            if not _association_class_compatible(
                    int(item.class_id), predicted_class,
                    self._compatible_class_ids):
                continue
            rx1, ry1, rx2, ry2 = item.aabb_xyxy
            shifted = (max(0, min(width, int(rx1 + left))),
                       max(0, min(height, int(ry1 + top))),
                       max(0, min(width, int(rx2 + left))),
                       max(0, min(height, int(ry2 + top))))
            if shifted[2] <= shifted[0] or shifted[3] <= shifted[1]:
                continue
            predicted_center = ((predicted_box[0] + predicted_box[2]) * 0.5,
                                (predicted_box[1] + predicted_box[3]) * 0.5)
            recovered_center = ((shifted[0] + shifted[2]) * 0.5,
                                (shifted[1] + shifted[3]) * 0.5)
            gate_x = max(12.0, (predicted_box[2] - predicted_box[0]) * (1.0 + padding))
            gate_y = max(12.0, (predicted_box[3] - predicted_box[1]) * (1.0 + padding))
            if (abs(recovered_center[0] - predicted_center[0]) > gate_x or
                    abs(recovered_center[1] - predicted_center[1]) > gate_y):
                continue
            recovered.append(replace(
                item, aabb_xyxy=shifted,
                center_xy=(int((shifted[0] + shifted[2]) * 0.5),
                           int((shifted[1] + shifted[3]) * 0.5)),
                track_id=-1, track_id_is_stable=False))
        if not recovered:
            return detections
        merged = list(detections)
        for item in recovered:
            overlaps = [index for index, existing in enumerate(merged)
                        if existing.aabb_xyxy is not None and
                        _box_iou(item.aabb_xyxy, existing.aabb_xyxy) >= 0.20]
            if not overlaps:
                merged.append(item)
                continue
            best = max(overlaps, key=lambda index: merged[index].confidence)
            if item.confidence > merged[best].confidence:
                merged[best] = item
        if len(merged) > self._maximum_detections:
            merged.sort(key=lambda item: float(item.confidence), reverse=True)
            merged = merged[:self._maximum_detections]
        self._roi_hits += 1
        return merged

    def _reid_worker_loop(self) -> None:
        while not rospy.is_shutdown():
            with self._reid_condition:
                while (not self._pending_reid and not self._closing and
                       not rospy.is_shutdown()):
                    self._reid_condition.wait(timeout=0.25)
                if self._closing or rospy.is_shutdown():
                    return
                name = next(iter(self._pending_reid))
                frame, output, candidate_indices = self._pending_reid.pop(name)
            try:
                self._encode_reid_cache(
                    name, frame, output, candidate_indices)
            except Exception as error:
                rospy.logerr_throttle(1.0, "shared ReID processing failed: %s", error)

    def _encode_reid_cache(
            self, name: str, frame, output: DetectionArray,
            candidate_indices: List[int]) -> None:
        source = self._sources[name]
        start = time.perf_counter()
        encoder = self._encoders[source["reid_profile"]]
        candidates = [output.candidates[index] for index in
                      candidate_indices[:source["maximum_reid_rois"]]]
        features, qualities = encoder.encode_many_with_quality(
            frame, [(item.bbox, item.class_id) for item in candidates])
        entries = []
        for candidate, feature, quality in zip(candidates, features, qualities):
            if feature is not None and quality >= self._minimum_embedding_quality:
                entries.append({
                    "bbox": tuple(candidate.bbox),
                    "class_id": int(candidate.class_id),
                    "embedding": feature.astype("float32", copy=False).tolist(),
                })
        now = rospy.Time.now()
        if (not output.header.stamp.is_zero() and not now.is_zero() and
                (now - output.header.stamp).to_sec() >
                2.0 * self._reid_cache_age_sec):
            self._reid_stale += 1
        elif entries:
            with self._reid_condition:
                previous = self._recent_reid.get(name)
                stamp = output.header.stamp.to_sec()
                if previous is None or stamp > previous["stamp"]:
                    self._recent_reid[name] = {"stamp": stamp,
                                               "entries": entries}
        elapsed_ms = 1000.0 * (time.perf_counter() - start)
        self._reid_processed += 1
        self._reid_ms_ewma = (elapsed_ms if self._reid_processed == 1 else
                              self._reid_ms_ewma + 0.10 *
                              (elapsed_ms - self._reid_ms_ewma))

    def _run_startup_warmup(self) -> bool:
        try:
            if self._warmup_frames > 0:
                import numpy as np
                frame = np.zeros(
                    (self._warmup_height, self._warmup_width, 3), dtype=np.uint8)
                for _ in range(self._warmup_frames):
                    self._backend.detect(frame, self._confidence, self._iou,
                                         self._maximum_detections)
            self._startup_ready = True
            self._startup_duration_sec = time.monotonic() - self._startup_started_wall
            rospy.loginfo("shared YOLO warmup complete in %.3fs",
                          self._startup_duration_sec)
            return True
        except Exception as error:
            self._startup_error = "%s: %s" % (type(error).__name__, error)
            self._startup_duration_sec = time.monotonic() - self._startup_started_wall
            rospy.logerr("shared YOLO startup warmup failed: %s", self._startup_error)
            return False

    @staticmethod
    def _diagnostic_value(key: str, value: Any) -> KeyValue:
        return KeyValue(key=key, value=str(value))

    def _publish_diagnostics(self, _event) -> None:
        status = DiagnosticStatus()
        status.name = rospy.get_name() + ": shared_yolo"
        status.hardware_id = str(self._runtime.get("effective_device", "unknown"))
        if self._startup_error:
            status.level = DiagnosticStatus.ERROR
            status.message = "startup warmup failed"
        elif not self._startup_ready:
            status.level = DiagnosticStatus.WARN
            status.message = "model warmup in progress"
        else:
            drop_ratio = float(self._dropped_backlog) / max(1, self._received)
            status.level = (DiagnosticStatus.WARN if drop_ratio > 0.50
                            else DiagnosticStatus.OK)
            status.message = ("detector overloaded" if status.level == DiagnosticStatus.WARN
                              else "shared detector ready")
        with self._condition:
            pending_images = len(self._pending)
        with self._reid_condition:
            pending_reid = len(self._pending_reid)
        values = {
            "startup_ready": self._startup_ready,
            "startup_error": self._startup_error,
            "startup_seconds": round(self._startup_duration_sec, 3),
            "device": self._runtime.get("effective_device", "unknown"),
            "backend": self._runtime.get("backend", "unknown"),
            "model_format": self._runtime.get("model_format", "unknown"),
            "sources": len(self._sources),
            "received_frames": self._received,
            "processed_frames": self._processed,
            "pending_images": pending_images,
            "dropped_image_backlog": self._dropped_backlog,
            "rate_limited_frames": self._rate_limited,
            "priority_source": self._current_priority_source(),
            "last_batch_size": self._last_batch_size,
            "inference_ms_ewma": round(self._inference_ms_ewma, 3),
            "pending_reid": pending_reid,
            "reid_processed": self._reid_processed,
            "reid_dropped_backlog": self._reid_dropped_backlog,
            "reid_stale": self._reid_stale,
            "reid_reused": self._reid_reused,
            "reid_ms_ewma": round(self._reid_ms_ewma, 3),
            "minimum_embedding_quality": self._minimum_embedding_quality,
            "roi_attempts": self._roi_attempts,
            "roi_hits": self._roi_hits,
            "roi_ms_ewma": round(self._roi_ms_ewma, 3),
            "pixel_assist_enabled": True,
            "pixel_tracking_attempts": self._pixel_tracking_attempts,
            "pixel_tracking_valid": self._pixel_tracking_valid,
            "pixel_assist_frames": self._pixel_assist_frames,
            "pixel_assist_rejected": self._pixel_assist_rejected,
            "masked_detections": self._masked_detections,
            "pixel_yolo_rejected": self._pixel_yolo_rejected,
            "pixel_color_attempts": self._pixel_color_attempts,
            "pixel_color_valid": self._pixel_color_valid,
            "pixel_assist_ms_ewma": round(self._pixel_assist_ms_ewma, 3),
            "pixel_assist_confidence": self._pixel_assist_confidence,
            "roi_inference_image_size": self._roi_inference_image_size or "default",
            "oversized_detections_rejected": self._oversized_detections,
            "rss_peak_mib": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, 2),
        }
        # ReID runs on its own latest-only worker. Report the actual backend
        # selected by the model session, rather than echoing the requested
        # device from YAML (which may have fallen back to CPU).
        for profile_name, encoder in self._encoders.items():
            runtime_status = getattr(encoder, "runtime_status", None)
            if callable(runtime_status):
                for model_name, runtime in runtime_status().items():
                    values["reid_runtime_%s_%s" % (profile_name, model_name)] = runtime
        torch_module = sys.modules.get("torch")
        if torch_module is not None and torch_module.cuda.is_available():
            values["cuda_allocated_mib"] = round(
                torch_module.cuda.memory_allocated() / 1048576.0, 2)
            values["cuda_reserved_mib"] = round(
                torch_module.cuda.memory_reserved() / 1048576.0, 2)
        status.values = [self._diagnostic_value(key, value)
                         for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self._diagnostics_publisher.publish(message)


def main() -> None:
    rospy.init_node("sar_yolo_multi_source_detection")
    MultiSourceDetectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
