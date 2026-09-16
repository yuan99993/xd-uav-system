#!/usr/bin/env python3
"""ROS adapter for Ultralytics YOLO detection without any temporal tracker.

This node is intentionally narrower than ``smart_tracker_node.py``: one input
image produces one ``vision_msgs/Detection2DArray`` and it never allocates a
track id, owns a gallery, or invokes Ultralytics' ``model.track()`` API.  The
XD tracking package is therefore the sole owner of temporal identity.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any, Dict

import rospy
import rospkg
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from xd_uav_track.msg import DetectionArray, DetectionCandidate
from xd_uav_track.reid import AppearanceEncoder

from sar_yolo_detector.pixeagle.backends import DevicePreference, create_backend
from sar_yolo_detector.scripts_compat import image_to_bgr


def _bounded(value: Any, default: float, low: float, high: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return min(high, max(low, result))


class DetectionOnlyNode:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._closing = False
        self._pending_image = None
        self._dropped_backlog = 0
        self._skipped_without_subscriber = 0
        self._received = 0
        self._processed = 0
        self._inference_ms_ewma = 0.0
        self._processing_ms_ewma = 0.0
        self._reid_ms_ewma = 0.0
        self._reid_processed = 0
        self._reid_backlog_drop = 0
        self._reid_condition = threading.Condition()
        self._pending_reid = None
        self._package_root = Path(
            rospkg.RosPack().get_path("sar_yolo_detector")
        ).resolve()
        self._input_topic = rospy.get_param("~input_image_topic", "camera/image_raw")
        self._maximum_age_sec = max(
            0.0, float(rospy.get_param("~maximum_capture_age_sec", 0.5))
        )
        self._require_capture_stamp = bool(
            rospy.get_param("~require_capture_timestamp", True)
        )
        self._reject_out_of_order = bool(
            rospy.get_param("~reject_out_of_order", True)
        )
        self._pause_without_subscribers = bool(
            rospy.get_param("~pause_without_subscribers", False)
        )
        self._last_stamp = rospy.Time(0)

        config: Dict[str, Any] = dict(rospy.get_param("~SmartTracker", {}))
        config["SMART_TRACKER_MODELS_ROOT"] = str(self._package_root)
        for key in ("SMART_TRACKER_GPU_MODEL_PATH", "SMART_TRACKER_CPU_MODEL_PATH"):
            value = str(config.get(key, "") or "").strip()
            if value and not Path(value).expanduser().is_absolute():
                config[key] = str((self._package_root / value).resolve())
        # This is a detector endpoint.  Rejecting the tracking-only selector is
        # deliberate: it prevents a future YAML overlay from silently reviving
        # a second identity allocator.
        config["TRACKER_TYPE"] = "detection_only"
        self._backend = create_backend("ultralytics", config=config)
        if not self._backend.is_available:
            raise RuntimeError("Ultralytics backend is unavailable on this host")

        requested_model = str(
            rospy.get_param(
                "~model_path", config.get("SMART_TRACKER_GPU_MODEL_PATH", "")
            )
            or ""
        ).strip()
        if not requested_model:
            raise RuntimeError("~model_path or SmartTracker GPU model path is required")
        use_gpu = bool(rospy.get_param("~use_gpu", True))
        fallback_to_cpu = bool(rospy.get_param("~fallback_to_cpu", True))
        self._runtime = self._backend.load_model(
            requested_model,
            DevicePreference.CUDA if use_gpu else DevicePreference.CPU,
            fallback_enabled=fallback_to_cpu,
            context="detection_only_startup",
        )
        self._confidence = _bounded(
            config.get("SMART_TRACKER_CONFIDENCE_THRESHOLD", 0.25), 0.25, 0.0, 1.0
        )
        self._iou = _bounded(
            config.get("SMART_TRACKER_IOU_THRESHOLD", 0.45), 0.45, 0.0, 1.0
        )
        self._maximum_detections = max(
            1, int(config.get("SMART_TRACKER_MAX_DETECTIONS", 100))
        )
        self._publish_vision = bool(rospy.get_param("~publish_vision_detections", True))
        vision_topic = rospy.get_param(
            "~detections_topic", "sar_yolo_detector/detections")
        self._publisher = rospy.Publisher(
            vision_topic, Detection2DArray, queue_size=1
        ) if self._publish_vision else None
        xd_topic = str(rospy.get_param("~xd_detections_topic", "") or "").strip()
        self._xd_publisher = rospy.Publisher(
            xd_topic, DetectionArray, queue_size=1
        ) if xd_topic else None
        if self._publisher is None and self._xd_publisher is None:
            raise RuntimeError(
                "publish_vision_detections is false and xd_detections_topic is empty")
        self._image_source = str(rospy.get_param("~image_source", "eo"))
        self._sensor_id = str(rospy.get_param("~sensor_id", "camera_primary"))
        self._detector_name = str(rospy.get_param(
            "~detector_name", "sar_yolo_detector"))
        self._model_version = str(rospy.get_param(
            "~model_version", Path(requested_model).name))
        self._reid_enabled = bool(rospy.get_param("~reid_enabled", False))
        reid_config = dict(rospy.get_param("~reid", {}))
        if rospy.has_param("~reid_model_profile"):
            reid_config["active_model_profile"] = rospy.get_param(
                "~reid_model_profile")
        reid_runtime = dict(reid_config.get("runtime", {}))
        self._maximum_reid_rois = max(1, int(rospy.get_param(
            "~maximum_reid_rois_per_frame",
            reid_runtime.get("maximum_rois_per_frame", 32))))
        self._reid_encoder = (
            AppearanceEncoder(reid_config) if self._reid_enabled else None)
        if self._reid_enabled and self._xd_publisher is None:
            raise RuntimeError("inline ReID requires xd_detections_topic")
        self._encoded_embeddings = 0
        self._reid_async = bool(rospy.get_param("~reid_async", True))
        self._subscriber = rospy.Subscriber(
            self._input_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
            tcp_nodelay=True,
        )
        self._reid_worker = None
        if self._reid_encoder is not None and self._reid_async:
            self._reid_worker = threading.Thread(
                target=self._reid_worker_loop, name="inline_reid", daemon=True)
            self._reid_worker.start()
        self._worker = threading.Thread(
            target=self._worker_loop, name="yolo_detection", daemon=True)
        self._worker.start()
        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "sar_yolo_detection_only ready: model=%s device=%s input=%s",
            self._runtime.get("model_path", requested_model),
            self._runtime.get("effective_device", "unknown"),
            self._input_topic,
        )

    def close(self) -> None:
        with self._condition:
            if self._closing:
                return
            self._closing = True
            self._condition.notify_all()
        with self._reid_condition:
            self._reid_condition.notify_all()
        worker = getattr(self, "_worker", None)
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=3.0)
        if worker is not None and worker.is_alive():
            rospy.logwarn("YOLO inference did not stop within 3s; leaving backend cleanup to process exit")
            return
        reid_worker = getattr(self, "_reid_worker", None)
        if reid_worker is not None and reid_worker is not threading.current_thread():
            reid_worker.join(timeout=3.0)
        if reid_worker is not None and reid_worker.is_alive():
            rospy.logwarn("inline ReID did not stop within 3s; leaving cleanup to process exit")
            return
        backend = getattr(self, "_backend", None)
        if backend is not None:
            backend.unload_model()
            self._backend = None
        reid_encoder = getattr(self, "_reid_encoder", None)
        if reid_encoder is not None:
            reid_encoder.close()
            self._reid_encoder = None

    @staticmethod
    def _to_message(header, detection) -> Detection2D:
        x1, y1, x2, y2 = detection.aabb_xyxy
        message = Detection2D()
        message.header = header
        message.bbox.center.x = 0.5 * (float(x1) + float(x2))
        message.bbox.center.y = 0.5 * (float(y1) + float(y2))
        message.bbox.center.theta = float(detection.rotation_deg or 0.0)
        message.bbox.size_x = max(0.0, float(x2) - float(x1))
        message.bbox.size_y = max(0.0, float(y2) - float(y1))
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = int(detection.class_id)
        hypothesis.score = _bounded(detection.confidence, 0.0, 0.0, 1.0)
        message.results.append(hypothesis)
        return message

    @staticmethod
    def _to_xd_candidate(detection, width: int, height: int):
        if detection.aabb_xyxy is None:
            return None
        x1, y1, x2, y2 = detection.aabb_xyxy
        values = (x1, y1, x2, y2, detection.confidence)
        if not all(math.isfinite(float(value)) for value in values):
            return None
        x1 = max(0, min(width, int(math.floor(x1))))
        y1 = max(0, min(height, int(math.floor(y1))))
        x2 = max(0, min(width, int(math.ceil(x2))))
        y2 = max(0, min(height, int(math.ceil(y2))))
        if x2 <= x1 or y2 <= y1:
            return None
        candidate = DetectionCandidate()
        # Temporal identity belongs exclusively to xd_uav_track.
        candidate.track_id = -1
        candidate.track_id_is_stable = False
        candidate.class_id = int(detection.class_id)
        candidate.confidence = _bounded(
            detection.confidence, 0.0, 0.0, 1.0)
        candidate.bbox = [x1, y1, x2, y2]
        candidate.has_bbox = True
        return candidate

    def _image_callback(self, message: Image) -> None:
        self._received += 1
        if self._pause_without_subscribers:
            vision_connections = (
                self._publisher.get_num_connections()
                if self._publisher is not None else 0)
            xd_connections = (
                self._xd_publisher.get_num_connections()
                if self._xd_publisher is not None else 0)
            if vision_connections + xd_connections == 0:
                self._skipped_without_subscriber += 1
                return
        stamp = message.header.stamp
        if self._require_capture_stamp and stamp.is_zero():
            rospy.logwarn_throttle(2.0, "YOLO detection rejected: image stamp is zero")
            return
        with self._condition:
            if (self._reject_out_of_order and not stamp.is_zero() and
                    not self._last_stamp.is_zero() and stamp <= self._last_stamp):
                rospy.logwarn_throttle(
                    2.0, "YOLO detection rejected: image stamp did not advance")
                return
            now = rospy.Time.now()
            if (not stamp.is_zero() and not now.is_zero() and
                    self._maximum_age_sec > 0.0):
                age = (now - stamp).to_sec()
                if age > self._maximum_age_sec:
                    rospy.logwarn_throttle(
                        2.0, "YOLO detection rejected: stale image %.3fs", age)
                    return
            if self._pending_image is not None:
                self._dropped_backlog += 1
            self._pending_image = message
            if not stamp.is_zero():
                self._last_stamp = stamp
            self._condition.notify()

    def _worker_loop(self) -> None:
        while not rospy.is_shutdown():
            with self._condition:
                while (self._pending_image is None and not self._closing and
                       not rospy.is_shutdown()):
                    self._condition.wait(timeout=0.25)
                if self._closing or rospy.is_shutdown():
                    return
                message = self._pending_image
                self._pending_image = None
            try:
                self._process_image(message)
            except Exception as error:
                # A malformed third-party backend result must not kill the
                # only inference worker for the rest of the flight.
                rospy.logerr_throttle(
                    1.0, "YOLO result conversion failed: %s", error)

    def _process_image(self, message: Image) -> None:
        processing_start = time.perf_counter()
        try:
            frame = image_to_bgr(message)
            inference_start = time.perf_counter()
            _, detections = self._backend.detect(
                frame,
                conf=self._confidence,
                iou=self._iou,
                max_det=self._maximum_detections,
            )
            inference_ms = 1000.0 * (time.perf_counter() - inference_start)
        except Exception as error:
            rospy.logerr_throttle(1.0, "YOLO detection failed: %s", error)
            return

        now = rospy.Time.now()
        stamp = message.header.stamp
        if (not stamp.is_zero() and not now.is_zero() and
                self._maximum_age_sec > 0.0 and
                (now - stamp).to_sec() > self._maximum_age_sec):
            rospy.logwarn_throttle(
                2.0, "YOLO detection dropped: stale after inference")
            return

        vision_output = Detection2DArray() if self._publisher is not None else None
        if vision_output is not None:
            vision_output.header = message.header
        xd_output = DetectionArray() if self._xd_publisher is not None else None
        if xd_output is not None:
            xd_output.header = message.header
            xd_output.image_width = message.width
            xd_output.image_height = message.height
            xd_output.image_source = self._image_source
            xd_output.sensor_id = self._sensor_id
            xd_output.detector_name = self._detector_name
            xd_output.model_version = self._model_version
        for detection in detections:
            if detection.aabb_xyxy is None:
                continue
            if vision_output is not None:
                vision_output.detections.append(
                    self._to_message(message.header, detection))
            if xd_output is not None:
                candidate = self._to_xd_candidate(
                    detection, int(message.width), int(message.height))
                if candidate is not None:
                    xd_output.candidates.append(candidate)
        if vision_output is not None:
            self._publisher.publish(vision_output)
        if xd_output is not None:
            if self._reid_encoder is None:
                self._xd_publisher.publish(xd_output)
            elif self._reid_async:
                with self._reid_condition:
                    if self._pending_reid is not None:
                        self._reid_backlog_drop += 1
                    self._pending_reid = (frame, xd_output)
                    self._reid_condition.notify()
            else:
                self._encode_and_publish_reid(frame, xd_output)
        self._processed += 1
        processing_ms = 1000.0 * (time.perf_counter() - processing_start)
        alpha = 0.10
        if self._processed == 1:
            self._inference_ms_ewma = inference_ms
            self._processing_ms_ewma = processing_ms
        else:
            self._inference_ms_ewma += alpha * (
                inference_ms - self._inference_ms_ewma)
            self._processing_ms_ewma += alpha * (
                processing_ms - self._processing_ms_ewma)
        rospy.loginfo_throttle(
            10.0,
            "YOLO live stats received=%d processed=%d backlog_drop=%d "
            "no_subscriber_skip=%d inference_ms=%.2f processing_ms=%.2f "
            "reid_ms=%.2f reid_backlog_drop=%d reid_embeddings=%d",
            self._received, self._processed, self._dropped_backlog,
            self._skipped_without_subscriber, self._inference_ms_ewma,
            self._processing_ms_ewma, self._reid_ms_ewma,
            self._reid_backlog_drop, self._encoded_embeddings)

    def _reid_worker_loop(self) -> None:
        while not rospy.is_shutdown():
            with self._reid_condition:
                while (self._pending_reid is None and not self._closing and
                       not rospy.is_shutdown()):
                    self._reid_condition.wait(timeout=0.25)
                if self._closing or rospy.is_shutdown():
                    return
                frame, output = self._pending_reid
                self._pending_reid = None
            try:
                self._encode_and_publish_reid(frame, output)
            except Exception as error:
                rospy.logerr_throttle(1.0, "inline ReID failed: %s", error)

    def _encode_and_publish_reid(self, frame, output: DetectionArray) -> None:
        start = time.perf_counter()
        candidates = output.candidates[:self._maximum_reid_rois]
        features = self._reid_encoder.encode_many(
            frame, [(candidate.bbox, candidate.class_id)
                    for candidate in candidates])
        for candidate, feature in zip(candidates, features):
            if feature is not None:
                candidate.appearance_embedding = feature.astype(
                    "float32", copy=False).tolist()
                self._encoded_embeddings += 1
        stamp = output.header.stamp
        now = rospy.Time.now()
        if (not stamp.is_zero() and not now.is_zero() and
                self._maximum_age_sec > 0.0 and
                (now - stamp).to_sec() > self._maximum_age_sec):
            rospy.logwarn_throttle(2.0, "inline ReID dropped stale output")
            return
        elapsed_ms = 1000.0 * (time.perf_counter() - start)
        self._reid_processed += 1
        if self._reid_processed == 1:
            self._reid_ms_ewma = elapsed_ms
        else:
            self._reid_ms_ewma += 0.10 * (elapsed_ms - self._reid_ms_ewma)
        self._xd_publisher.publish(output)


def main() -> None:
    rospy.init_node("sar_yolo_detection_only")
    DetectionOnlyNode()
    rospy.spin()


if __name__ == "__main__":
    main()
