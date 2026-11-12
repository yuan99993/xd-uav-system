#!/usr/bin/env python3
"""Elect one shared YOLO model process for identical aircraft launches."""

from __future__ import annotations

import fcntl
import json
import os
import re
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rospy
import rospkg
import yaml
from sar_yolo_detector.msg import NormalizedDetection
from sar_yolo_detector.pixeagle.backends import DevicePreference, create_backend
from sar_yolo_detector.pixeagle.backends.tensorrt_compat import (
    prepare_fp16_engine,
    sha256_file,
)
from sar_yolo_detector.scripts_compat import image_to_bgr
from sar_yolo_detector.srv import (
    GetSharedModelInfo,
    GetSharedModelInfoResponse,
    RunSharedInference,
    RunSharedInferenceResponse,
)


def _merge(base, overlay):
    result = dict(base or {})
    for key, value in (overlay or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


def _read_yaml(path):
    if not path:
        return {}
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        result = yaml.safe_load(stream) or {}
    if not isinstance(result, dict):
        raise RuntimeError("shared inference config must contain a YAML mapping")
    return result


class SharedInferenceServer:
    def __init__(self):
        self._backend_lock = threading.Lock()
        self._queue_condition = threading.Condition()
        self._pending = deque()
        self._pending_by_source = {}
        self._worker = None
        self._closing = False
        self._backend = None
        self._runtime = {}
        self._labels = {}
        self._model_task = "detect"
        self._package_root = Path(rospkg.RosPack().get_path("sar_yolo_detector")).resolve()

        raw_model = str(rospy.get_param("~model_path", "") or "").strip()
        if not raw_model:
            raise RuntimeError("~model_path is required for the shared YOLO provider")
        model_path = Path(raw_model).expanduser()
        self._model_path = (model_path if model_path.is_absolute()
                            else self._package_root / model_path).resolve()
        if not self._model_path.is_file():
            raise RuntimeError("shared YOLO model does not exist: %s" % self._model_path)
        expected = str(rospy.get_param("~model_sha256", "") or "").strip().lower()
        if expected and not re.fullmatch(r"[0-9a-f]{64}", expected):
            raise RuntimeError("~model_sha256 must be a 64-character hexadecimal digest")
        actual = sha256_file(self._model_path)
        if expected and expected != actual:
            raise RuntimeError("shared YOLO model SHA-256 does not match the launch parameter")
        self._model_key = expected or actual
        self._service_base = "/sar_yolo/shared_inference/m_" + self._model_key
        self._provider_model_path = self._model_path
        self._provider_model_sha256 = self._model_key
        self._lock_path = str(Path(rospy.get_param("~lock_dir", "/tmp")) /
                              ("sar_yolo_shared_inference_%s.lock" % self._model_key))
        self._lock_fd = os.open(self._lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        self._lock_owned = False
        self._info_service = None
        self._detect_service = None
        self._maximum_age_sec = max(0.0, float(rospy.get_param(
            "~maximum_capture_age_sec", 0.5)))
        self._request_count = 0
        self._failure_count = 0
        self._batch_wait_sec = max(0.0, min(0.010, float(rospy.get_param(
            "~batch_wait_ms", 4.0)) / 1000.0))
        requested_batch = max(1, min(8, int(rospy.get_param("~maximum_batch_size", 2))))
        self._requested_batch_size = requested_batch
        self._maximum_batch_size = self._resolve_batch_size(requested_batch)

        base_config = _read_yaml(rospy.get_param("~config", ""))
        overlay_config = _read_yaml(rospy.get_param("~overlay_config", ""))
        merged = _merge(base_config, overlay_config)
        self._smart_config = dict(merged.get("SmartTracker", {}))
        self._smart_config["SMART_TRACKER_MODELS_ROOT"] = str(self._package_root)
        self._smart_config["SMART_TRACKER_GPU_MODEL_PATH"] = str(self._model_path)
        self._smart_config["SMART_TRACKER_CPU_MODEL_PATH"] = str(self._model_path)
        self._smart_config["SMART_TRACKER_MODEL_SHA256"] = self._model_key
        self._smart_config["SMART_TRACKER_REQUIRE_MODEL_SHA256"] = True
        digests = dict(self._smart_config.get("SMART_TRACKER_MODEL_SHA256_BY_NAME", {}) or {})
        digests[self._model_path.name] = self._model_key
        self._smart_config["SMART_TRACKER_MODEL_SHA256_BY_NAME"] = digests

    def _resolve_batch_size(self, requested):
        if self._model_path.suffix.lower() != ".engine":
            return requested
        manifest_path = Path(str(self._model_path) + ".json")
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            rospy.logwarn("TensorRT batch mode disabled: no engine manifest at %s", manifest_path)
            return 1
        if not bool(manifest.get("dynamic", False)):
            source = Path(str(manifest.get("source_model", ""))).expanduser()
            if source.is_file() and manifest.get("source_sha256"):
                return requested
            rospy.logwarn("Static TensorRT engine has no verified source model; batch size is 1")
            return 1
        return requested

    def _prepare_dynamic_batch_engine(self):
        if (self._requested_batch_size <= 1 or
                self._model_path.suffix.lower() != ".engine"):
            return
        manifest_path = Path(str(self._model_path) + ".json")
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if bool(manifest.get("dynamic", False)):
                self._provider_model_path = self._model_path
                self._provider_model_sha256 = self._model_key
                return
            source = Path(str(manifest.get("source_model", ""))).expanduser().resolve()
            source_sha = str(manifest.get("source_sha256", "") or "").strip().lower()
            if not source.is_file() or not source_sha:
                self._maximum_batch_size = 1
                return
            rospy.loginfo(
                "Building a cached batch-2 TensorRT engine from the verified source %s",
                source,
            )
            cache_dir = Path(rospy.get_param(
                "~engine_cache_dir", "~/.cache/sar_yolo_detector/tensorrt"
            )).expanduser()
            self._provider_model_path, self._provider_model_sha256 = prepare_fp16_engine(
                source,
                cache_dir,
                image_size=int(manifest.get("image_size", 512)),
                workspace_gib=float(manifest.get("workspace_gib", 0.5)),
                optimization_level=int(manifest.get("builder_optimization_level", 1)),
                expected_model_sha256=source_sha,
                dynamic_batch=True,
            )
            self._maximum_batch_size = self._requested_batch_size
            rospy.loginfo("Using dynamic-batch shared engine %s", self._provider_model_path)
        except Exception as exc:
            self._provider_model_path = self._model_path
            self._provider_model_sha256 = self._model_key
            self._maximum_batch_size = 1
            rospy.logwarn(
                "Could not build dynamic-batch TensorRT engine; falling back to the "
                "specified static engine (multi-aircraft inference will be serialized): %s: %s",
                type(exc).__name__, exc,
            )

    def _load_backend(self):
        self._prepare_dynamic_batch_engine()
        backend_name = str(self._smart_config.get("DETECTION_BACKEND", "ultralytics"))
        self._smart_config["SMART_TRACKER_GPU_MODEL_PATH"] = str(self._provider_model_path)
        self._smart_config["SMART_TRACKER_CPU_MODEL_PATH"] = str(self._provider_model_path)
        self._smart_config["SMART_TRACKER_MODEL_SHA256"] = self._provider_model_sha256
        digests = dict(self._smart_config.get("SMART_TRACKER_MODEL_SHA256_BY_NAME", {}) or {})
        digests[self._provider_model_path.name] = self._provider_model_sha256
        self._smart_config["SMART_TRACKER_MODEL_SHA256_BY_NAME"] = digests
        self._backend = create_backend(backend_name, config=self._smart_config)
        if not self._backend.is_available:
            raise RuntimeError("shared YOLO backend is unavailable: %s" % backend_name)
        use_gpu = bool(rospy.get_param("~use_gpu", True))
        fallback = bool(rospy.get_param("~fallback_to_cpu", False))
        preference = DevicePreference.CUDA if use_gpu else DevicePreference.CPU
        self._runtime = self._backend.load_model(
            str(self._provider_model_path), preference, fallback_enabled=fallback,
            context="shared_inference_startup",
        )
        # TensorRT creates its execution context lazily on the first predict().
        # Warm it before advertising the service so that the first live camera
        # request does not consume the short shared-inference queue deadline.
        if (self._provider_model_path.suffix.lower() == ".engine" and
                bool(rospy.get_param("~startup_warmup", True))):
            image_size = int(self._smart_config.get(
                "SMART_TRACKER_INFERENCE_IMAGE_SIZE", 512))
            started = time.perf_counter()
            self._backend.detect(
                np.zeros((image_size, image_size, 3), dtype=np.uint8),
                conf=1.0, iou=0.3, max_det=1, image_size=image_size,
            )
            warmup_ms = (time.perf_counter() - started) * 1000.0
            self._runtime["startup_warmup_ms"] = warmup_ms
            rospy.loginfo("shared YOLO TensorRT warmup completed in %.1f ms", warmup_ms)
        self._labels = self._backend.get_model_labels()
        self._model_task = self._backend.get_model_task()
        rospy.loginfo(
            "shared YOLO provider ready: key=%s backend=%s device=%s batch=%d model=%s",
            self._model_key[:12], self._runtime.get("backend"),
            self._runtime.get("effective_device"), self._maximum_batch_size,
            self._provider_model_path,
        )

    def _unload_backend(self):
        backend, self._backend = self._backend, None
        if backend is not None:
            try:
                backend.unload_model()
            except Exception as exc:
                rospy.logerr("failed to release shared YOLO model: %s", exc)

    def _info(self, request):
        response = GetSharedModelInfoResponse()
        response.success = request.model_key == self._model_key and self._backend is not None
        response.message = "" if response.success else "shared model key mismatch or provider not ready"
        if response.success:
            response.runtime_json = json.dumps(self._runtime, default=str)
            response.labels_json = json.dumps(self._labels, default=str)
            response.model_task = str(self._model_task)
        return response

    @staticmethod
    def _to_message(detection):
        output = NormalizedDetection()
        output.track_id = int(detection.track_id)
        output.class_id = int(detection.class_id)
        output.confidence = float(detection.confidence)
        output.aabb_xyxy = [float(value) for value in detection.aabb_xyxy]
        output.center_xy = [int(value) for value in detection.center_xy]
        output.geometry_type = str(detection.geometry_type or "aabb")
        if detection.obb_xywhr is not None:
            output.obb_xywhr = [float(value) for value in detection.obb_xywhr]
        else:
            output.obb_xywhr = [0.0] * 5
        if detection.polygon_xy:
            output.polygon_xy = [float(value) for point in detection.polygon_xy for value in point]
        output.has_rotation = detection.rotation_deg is not None
        output.rotation_deg = float(detection.rotation_deg or 0.0)
        output.track_id_is_stable = bool(detection.track_id_is_stable)
        return output

    def _detect(self, request):
        response = RunSharedInferenceResponse()
        if request.model_key != self._model_key or self._backend is None:
            response.success = False
            response.message = "shared model key mismatch or provider not ready"
            return response
        stamp = request.image.header.stamp
        now = rospy.Time.now()
        if (self._maximum_age_sec > 0 and not stamp.is_zero() and
                not now.is_zero() and (now - stamp).to_sec() > self._maximum_age_sec):
            age = (now - stamp).to_sec()
            response.success = False
            response.message = "stale image discarded by shared inference (age %.3fs)" % age
            self._failure_count += 1
            return response
        try:
            frame = image_to_bgr(request.image)
            source_id = str(request.source_id or "unknown")
            pending = {
                "request": request,
                "frame": frame,
                "source_id": source_id,
                "event": threading.Event(),
                "response": None,
            }
            with self._queue_condition:
                previous = self._pending_by_source.get(source_id)
                if previous is not None and not previous["event"].is_set():
                    try:
                        self._pending.remove(previous)
                    except ValueError:
                        pass
                    previous["response"] = RunSharedInferenceResponse(
                        success=False, message="superseded by a newer frame"
                    )
                    previous["event"].set()
                    self._failure_count += 1
                self._pending_by_source[source_id] = pending
                self._pending.append(pending)
                self._queue_condition.notify_all()
            if not pending["event"].wait(max(2.0, self._maximum_age_sec + 2.0)):
                response.success = False
                response.message = "shared inference queue timed out"
                self._failure_count += 1
                return response
            return pending["response"] or response
        except Exception as exc:
            self._failure_count += 1
            response.success = False
            response.message = "%s: %s" % (type(exc).__name__, exc)
            rospy.logerr_throttle(2.0, "shared YOLO inference failed: %s", response.message)
        return response

    @staticmethod
    def _compatible(first, second):
        left, right = first["request"], second["request"]
        return (
            first["frame"].shape == second["frame"].shape and
            float(left.confidence) == float(right.confidence) and
            float(left.iou) == float(right.iou) and
            int(left.max_detections) == int(right.max_detections) and
            int(left.image_size) == int(right.image_size)
        )

    def _take_batch(self):
        with self._queue_condition:
            while not self._closing and not self._pending:
                self._queue_condition.wait(0.25)
            if self._closing:
                return []
            first = self._pending.popleft()
            if self._pending_by_source.get(first["source_id"]) is first:
                self._pending_by_source.pop(first["source_id"], None)
            batch = [first]
            deadline = time.monotonic() + self._batch_wait_sec
            while (len(batch) < self._maximum_batch_size and not self._closing):
                selected = None
                for candidate in tuple(self._pending):
                    if candidate["event"].is_set():
                        self._pending.remove(candidate)
                        continue
                    if self._compatible(first, candidate):
                        selected = candidate
                        break
                if selected is not None:
                    self._pending.remove(selected)
                    if self._pending_by_source.get(selected["source_id"]) is selected:
                        self._pending_by_source.pop(selected["source_id"], None)
                    batch.append(selected)
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._queue_condition.wait(remaining)
            return batch

    def _complete(self, pending, response):
        pending["response"] = response
        pending["event"].set()

    def _worker_loop(self):
        while not rospy.is_shutdown():
            batch = self._take_batch()
            if not batch:
                if self._closing:
                    return
                continue
            fresh = []
            now = rospy.Time.now()
            for pending in batch:
                stamp = pending["request"].image.header.stamp
                if (self._maximum_age_sec > 0 and not stamp.is_zero() and
                        not now.is_zero() and (now - stamp).to_sec() > self._maximum_age_sec):
                    age = (now - stamp).to_sec()
                    self._complete(pending, RunSharedInferenceResponse(
                        success=False,
                        message="stale image discarded by shared inference (age %.3fs)" % age,
                    ))
                    self._failure_count += 1
                else:
                    fresh.append(pending)
            if not fresh:
                continue
            request = fresh[0]["request"]
            try:
                started = time.perf_counter()
                with self._backend_lock:
                    if len(fresh) > 1 and callable(getattr(self._backend, "detect_many", None)):
                        try:
                            results = self._backend.detect_many(
                                [item["frame"] for item in fresh],
                                conf=float(request.confidence),
                                iou=float(request.iou),
                                max_det=max(1, int(request.max_detections)),
                                image_size=(int(request.image_size) if request.image_size > 0 else None),
                            )
                        except Exception as batch_exc:
                            # Some TensorRT builds report dynamic metadata but
                            # still reject multi-image predict(). Disable
                            # batching for this provider and preserve detection
                            # with the already loaded shared engine.
                            self._maximum_batch_size = 1
                            rospy.logwarn(
                                "Shared batch inference unavailable; switching to batch=1: %s: %s",
                                type(batch_exc).__name__, batch_exc,
                            )
                            results = [self._backend.detect(
                                item["frame"],
                                conf=float(item["request"].confidence),
                                iou=float(item["request"].iou),
                                max_det=max(1, int(item["request"].max_detections)),
                                image_size=(int(item["request"].image_size)
                                            if item["request"].image_size > 0 else None),
                            ) for item in fresh]
                    else:
                        results = [self._backend.detect(
                            item["frame"],
                            conf=float(item["request"].confidence),
                            iou=float(item["request"].iou),
                            max_det=max(1, int(item["request"].max_detections)),
                            image_size=(int(item["request"].image_size)
                                        if item["request"].image_size > 0 else None),
                        ) for item in fresh]
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                if len(results) != len(fresh):
                    raise RuntimeError("shared backend returned an incomplete batch")
                for pending, (_mode, detections) in zip(fresh, results):
                    response = RunSharedInferenceResponse()
                    response.success = True
                    response.inference_ms = elapsed_ms
                    response.detections = [self._to_message(item) for item in detections]
                    self._complete(pending, response)
                    self._request_count += 1
            except Exception as exc:
                message = "%s: %s" % (type(exc).__name__, exc)
                rospy.logerr_throttle(2.0, "shared YOLO inference failed: %s", message)
                for pending in fresh:
                    self._complete(pending, RunSharedInferenceResponse(
                        success=False, message=message
                    ))
                    self._failure_count += 1

    def _stop_worker(self):
        self._closing = True
        with self._queue_condition:
            self._queue_condition.notify_all()
            for pending in self._pending:
                self._complete(pending, RunSharedInferenceResponse(
                    success=False, message="shared inference provider is stopping"
                ))
            self._pending.clear()
            self._pending_by_source.clear()
        if self._worker is not None and self._worker is not threading.current_thread():
            self._worker.join(timeout=10.0)
        self._worker = None

    def _serve_as_provider(self):
        self._load_backend()
        self._closing = False
        self._worker = threading.Thread(
            target=self._worker_loop, name="shared-yolo-inference", daemon=True
        )
        self._worker.start()
        self._info_service = rospy.Service(
            self._service_base + "/info", GetSharedModelInfo, self._info
        )
        self._detect_service = rospy.Service(
            self._service_base + "/detect", RunSharedInference, self._detect
        )
        rospy.loginfo("shared YOLO service available at %s", self._service_base)
        while not rospy.is_shutdown():
            rospy.sleep(0.5)

    def run(self):
        while not rospy.is_shutdown():
            try:
                fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self._lock_owned = True
            except BlockingIOError:
                rospy.sleep(0.25)
                continue
            try:
                self._serve_as_provider()
            except Exception as exc:
                rospy.logerr("shared YOLO provider startup failed: %s: %s",
                             type(exc).__name__, exc)
                if not rospy.is_shutdown():
                    rospy.sleep(5.0)
            finally:
                if self._info_service is not None:
                    self._info_service.shutdown("shared inference provider stopping")
                    self._info_service = None
                if self._detect_service is not None:
                    self._detect_service.shutdown("shared inference provider stopping")
                    self._detect_service = None
                self._stop_worker()
                self._unload_backend()
                if self._lock_owned:
                    fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
                    self._lock_owned = False
        os.close(self._lock_fd)


def main():
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    rospy.init_node("sar_yolo_shared_inference_candidate")
    server = SharedInferenceServer()
    server.run()


if __name__ == "__main__":
    main()
