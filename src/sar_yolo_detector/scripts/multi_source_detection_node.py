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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from xd_uav_track.msg import DetectionArray, DetectionCandidate, TrackStateArray
from xd_uav_track.reid import AppearanceEncoder

from sar_yolo_detector.pixeagle.backends import DevicePreference, create_backend
from sar_yolo_detector.scripts_compat import image_to_bgr


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


class MultiSourceDetectionNode:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._reid_condition = threading.Condition()
        self._closing = False
        self._pending: Dict[str, Image] = {}
        self._pending_reid: Dict[str, Tuple[Any, DetectionArray]] = {}
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
        self._priority_lost_max_frames = max(0, int(rospy.get_param(
            "~priority_lost_max_frames", 30)))
        self._discovery_max_fps = max(0.0, float(rospy.get_param(
            "~discovery_max_fps", 8.0)))
        self._priority_max_fps = max(0.0, float(rospy.get_param(
            "~priority_max_fps", 12.0)))
        self._standby_max_fps = max(0.0, float(rospy.get_param(
            "~standby_max_fps", 6.0)))
        self._tracked_boxes: Dict[str, Tuple[Tuple[int, int, int, int], int, float]] = {}
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
        config["SMART_TRACKER_MODELS_ROOT"] = str(self._package_root)
        for key in ("SMART_TRACKER_GPU_MODEL_PATH", "SMART_TRACKER_CPU_MODEL_PATH"):
            value = str(config.get(key, "") or "").strip()
            if value and not Path(value).expanduser().is_absolute():
                config[key] = str((self._package_root / value).resolve())
        config["TRACKER_TYPE"] = "detection_only"
        self._backend = create_backend("ultralytics", config=config)
        if not self._backend.is_available:
            raise RuntimeError("Ultralytics backend is unavailable on this host")
        inference_backend = str(rospy.get_param(
            "~inference_backend", "ultralytics") or "ultralytics").strip().lower()
        model_path = str(rospy.get_param(
            "~model_path", config.get("SMART_TRACKER_GPU_MODEL_PATH", "")) or "").strip()
        engine_path = str(rospy.get_param("~engine_path", "") or "").strip()
        if inference_backend in {"tensorrt", "tensorrt_fp16", "engine"}:
            if not engine_path:
                raise RuntimeError(
                    "~engine_path is required when ~inference_backend=tensorrt_fp16"
                )
            requested_model = engine_path
            engine_sha256 = str(rospy.get_param("~engine_sha256", "") or "").strip()
            if not engine_sha256:
                raise RuntimeError(
                    "~engine_sha256 is required for a TensorRT engine artifact"
                )
            config["SMART_TRACKER_MODEL_SHA256_BY_NAME"] = {
                Path(engine_path).name: engine_sha256
            }
        else:
            requested_model = model_path
        if not requested_model:
            raise RuntimeError("~model_path or SmartTracker GPU model path is required")
        self._runtime = self._backend.load_model(
            requested_model,
            DevicePreference.CUDA if bool(rospy.get_param("~use_gpu", True))
            else DevicePreference.CPU,
            fallback_enabled=bool(rospy.get_param("~fallback_to_cpu", False)),
            context="multi_source_detection_startup",
        )
        self._confidence = _bounded(config.get("SMART_TRACKER_CONFIDENCE_THRESHOLD", 0.25),
                                    0.25, 0.0, 1.0)
        self._iou = _bounded(config.get("SMART_TRACKER_IOU_THRESHOLD", 0.45),
                             0.45, 0.0, 1.0)
        self._maximum_detections = max(1, int(config.get("SMART_TRACKER_MAX_DETECTIONS", 100)))

        raw_sources = rospy.get_param("~sources", [])
        if not isinstance(raw_sources, list) or len(raw_sources) < 2:
            raise RuntimeError("~sources must contain at least two camera mappings")
        reid_config = dict(rospy.get_param("~reid", {}))
        self._encoders: Dict[str, AppearanceEncoder] = {}
        for raw in raw_sources:
            if not isinstance(raw, dict):
                raise RuntimeError("every ~sources entry must be a mapping")
            name = str(raw.get("name", "")).strip()
            input_topic = str(raw.get("input_image_topic", "")).strip()
            xd_topic = str(raw.get("xd_detections_topic", "")).strip()
            if not name or not input_topic or not xd_topic or name in self._sources:
                raise RuntimeError("each source needs unique name, input_image_topic and xd_detections_topic")
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
            source["subscriber"] = rospy.Subscriber(
                input_topic, Image, lambda message, key=name: self._image_callback(key, message),
                queue_size=1, buff_size=16 * 1024 * 1024, tcp_nodelay=True)
            self._sources[name] = source
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
        rospy.loginfo("shared YOLO ready: model=%s backend=%s device=%s sources=%s batch_wait_ms=%.1f discovery_fps=%.1f priority_fps=%.1f standby_fps=%.1f roi=%s",
                      self._runtime.get("model_path", requested_model),
                      self._runtime.get("backend", inference_backend),
                      self._runtime.get("effective_device", "unknown"),
                      ",".join(sorted(self._sources)), self._batch_wait_sec * 1000.0,
                      self._discovery_max_fps, self._priority_max_fps,
                      self._standby_max_fps,
                      "on" if any(source["roi_recovery_enabled"]
                                  for source in self._sources.values()) else "off")

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
        source = str(message.image_source or "").strip()
        if not source:
            return
        selected = None
        for track in message.tracks:
            if not track.selected:
                continue
            if track.lifecycle_state not in ("confirmed", "occluded", "lost"):
                continue
            if (track.lifecycle_state == "lost" and
                    int(track.frames_since_detection) > self._priority_lost_max_frames):
                continue
            x1, y1, x2, y2 = [int(value) for value in track.bbox]
            if x2 <= x1 or y2 <= y1:
                continue
            selected = ((x1, y1, x2, y2), int(track.class_id),
                        time.monotonic())
            break
        with self._tracking_lock:
            if selected is not None:
                self._priority_source = source
                self._priority_source_wall = time.monotonic()
                self._tracked_boxes[source] = selected

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
        detections = self._recover_with_prediction_roi(name, frame, detections)
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
        for detection in detections:
            if detection.aabb_xyxy is None:
                continue
            if vision is not None:
                vision.detections.append(_vision_detection(message.header, detection))
            candidate = _xd_candidate(detection, int(message.width), int(message.height))
            if candidate is not None:
                xd_output.candidates.append(candidate)
        if vision is not None:
            source["vision_publisher"].publish(vision)
        if not source["reid_enabled"] or not xd_output.candidates:
            source["xd_publisher"].publish(xd_output)
            return
        # ReID is intentionally decoupled from YOLO. One newest job is retained
        # per source so a crowded frame cannot stall the next detector batch.
        with self._reid_condition:
            if name in self._pending_reid:
                self._reid_dropped_backlog += 1
            self._pending_reid[name] = (frame, xd_output)
            self._reid_condition.notify()

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
        if any(_box_iou(predicted_box, item.aabb_xyxy) >= 0.20
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
                self._maximum_detections)
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
            rx1, ry1, rx2, ry2 = item.aabb_xyxy
            shifted = (max(0, min(width, int(rx1 + left))),
                       max(0, min(height, int(ry1 + top))),
                       max(0, min(width, int(rx2 + left))),
                       max(0, min(height, int(ry2 + top))))
            if shifted[2] <= shifted[0] or shifted[3] <= shifted[1]:
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
                frame, output = self._pending_reid.pop(name)
            try:
                self._encode_and_publish_reid(name, frame, output)
            except Exception as error:
                rospy.logerr_throttle(1.0, "shared ReID processing failed: %s", error)

    def _encode_and_publish_reid(
            self, name: str, frame, output: DetectionArray) -> None:
        source = self._sources[name]
        start = time.perf_counter()
        encoder = self._encoders[source["reid_profile"]]
        candidates = output.candidates[:source["maximum_reid_rois"]]
        features, qualities = encoder.encode_many_with_quality(
            frame, [(item.bbox, item.class_id) for item in candidates])
        for candidate, feature, quality in zip(candidates, features, qualities):
            if feature is not None:
                candidate.appearance_embedding = feature.astype(
                    "float32", copy=False).tolist()
            candidate.appearance_quality = 0.0 if feature is None else \
                max(0.001, float(quality))
        now = rospy.Time.now()
        if (not output.header.stamp.is_zero() and not now.is_zero() and
                self._maximum_age_sec > 0.0 and
                (now - output.header.stamp).to_sec() > self._maximum_age_sec):
            self._reid_stale += 1
            return
        source["xd_publisher"].publish(output)
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
            "reid_ms_ewma": round(self._reid_ms_ewma, 3),
            "roi_attempts": self._roi_attempts,
            "roi_hits": self._roi_hits,
            "roi_ms_ewma": round(self._roi_ms_ewma, 3),
            "rss_peak_mib": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, 2),
        }
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
