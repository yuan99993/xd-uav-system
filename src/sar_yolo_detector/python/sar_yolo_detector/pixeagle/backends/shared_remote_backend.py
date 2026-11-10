"""ROS client backend for a process-shared YOLO inference runtime.

Each SmartTracker keeps its own tracking state, while the model and CUDA
context live in one elected shared-inference server process.
"""

from __future__ import annotations

import hashlib
import json
import logging
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rospy
from sensor_msgs.msg import Image
from sar_yolo_detector.msg import NormalizedDetection as NormalizedDetectionMsg
from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection
from sar_yolo_detector.srv import (
    GetSharedModelInfo,
    GetSharedModelInfoRequest,
    RunSharedInference,
    RunSharedInferenceRequest,
)

from .detection_backend import DetectionBackend, DevicePreference

logger = logging.getLogger(__name__)


def _model_key(path: str, config: dict) -> str:
    configured = str(config.get("SMART_TRACKER_MODEL_SHA256", "") or "").strip().lower()
    if re.fullmatch(r"[0-9a-f]{64}", configured):
        return configured
    model_path = Path(path).expanduser().resolve()
    if model_path.is_file():
        digest = hashlib.sha256()
        with model_path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()
    return hashlib.sha256(str(model_path).encode("utf-8")).hexdigest()


def _service_base(config: dict, key: str) -> str:
    configured = str(config.get("SMART_TRACKER_SHARED_SERVICE_BASE", "") or "").strip()
    if configured:
        return "/" + configured.strip("/")
    return "/sar_yolo/shared_inference/m_" + key


class SharedRemoteBackend(DetectionBackend):
    """DetectionBackend proxy; all model calls go through one ROS service."""

    def __init__(self, config: Optional[dict] = None):
        self._config = dict(config or {})
        self._model_path = ""
        self._key = ""
        self._base = ""
        self._runtime: Dict[str, Any] = {}
        self._labels: Dict[int, str] = {}
        self._task = "detect"
        self._stale_frames_dropped = 0
        self._loaded = False
        self._info_proxy = None
        self._detect_proxy = None
        self.tracker_type_str = str(self._config.get("TRACKER_TYPE", "botsort")).lower()
        self.use_custom_reid = False
        self.tracker_args = {"persist": False, "verbose": False}

    @property
    def is_available(self) -> bool:
        return True

    @property
    def backend_name(self) -> str:
        return "shared_remote"

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def stale_frames_dropped(self) -> int:
        return self._stale_frames_dropped

    def load_model(
        self,
        model_path: str,
        device: DevicePreference = DevicePreference.AUTO,
        fallback_enabled: bool = False,
        context: str = "startup",
    ) -> Dict[str, Any]:
        del device, fallback_enabled, context
        self._model_path = str(Path(model_path).expanduser().resolve())
        self._key = _model_key(self._model_path, self._config)
        self._base = _service_base(self._config, self._key)
        self._info_proxy = rospy.ServiceProxy(
            self._base + "/info", GetSharedModelInfo, persistent=False
        )
        self._detect_proxy = rospy.ServiceProxy(
            self._base + "/detect", RunSharedInference, persistent=False
        )

        timeout_sec = max(10.0, float(self._config.get(
            "SMART_TRACKER_SHARED_STARTUP_TIMEOUT_SEC", 600.0)))
        deadline = time.monotonic() + timeout_sec
        last_error = "shared inference provider has not registered yet"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                rospy.wait_for_service(self._base + "/info", timeout=1.0)
                response = self._info_proxy(GetSharedModelInfoRequest(model_key=self._key))
                if not response.success:
                    raise RuntimeError(response.message or "shared model provider rejected the request")
                self._runtime = json.loads(response.runtime_json or "{}")
                raw_labels = json.loads(response.labels_json or "{}")
                self._labels = {int(class_id): str(name) for class_id, name in raw_labels.items()}
                self._task = str(response.model_task or "detect")
                if (bool(self._config.get("SMART_TRACKER_USE_GPU", True)) and
                        str(self._runtime.get("effective_device", "")).lower() not in
                        ("cuda", "gpu")):
                    raise RuntimeError(
                        "shared YOLO provider is not using CUDA: %s" %
                        self._runtime.get("effective_device", "unknown")
                    )
                self._runtime["backend"] = self._runtime.get("backend", "shared_remote")
                self._runtime["requested_model_path"] = self._model_path
                self._runtime["shared_model_key"] = self._key
                self._loaded = True
                logger.info("Using shared YOLO model %s through %s", self._key[:12], self._base)
                return dict(self._runtime)
            except Exception as exc:
                if isinstance(exc, RuntimeError) and str(exc).startswith(
                        "shared YOLO provider is not using CUDA:"):
                    raise
                last_error = "%s: %s" % (type(exc).__name__, exc)
                time.sleep(0.25)
        raise RuntimeError("Timed out waiting for shared YOLO model: " + last_error)

    def unload_model(self) -> None:
        # The shared provider owns the model lifetime. A tracker leaving must
        # never unload the model still serving other vehicles.
        self._loaded = False
        self._info_proxy = None
        self._detect_proxy = None

    def reset_tracking(self) -> None:
        # Temporal identity state belongs to the vehicle-local tracker.
        return None

    def authorize_model_digest(self, model_path: str, expected_sha256: str) -> None:
        requested = str(Path(model_path).expanduser().resolve())
        digest = str(expected_sha256 or "").strip().lower()
        if requested != self._model_path or (digest and digest != self._key):
            raise RuntimeError("shared inference model is immutable while any vehicle is using it")

    def switch_model(
        self,
        new_model_path: str,
        device: DevicePreference = DevicePreference.AUTO,
        fallback_enabled: bool = False,
    ) -> Dict[str, Any]:
        del device, fallback_enabled
        requested = str(Path(new_model_path).expanduser().resolve())
        if requested != self._model_path:
            raise RuntimeError(
                "A shared model is fixed for this launch; change model_path for all vehicles together"
            )
        return dict(self._runtime)

    def detect(
        self,
        frame: np.ndarray,
        conf: float = 0.3,
        iou: float = 0.3,
        max_det: int = 20,
        image_size: Optional[int] = None,
    ) -> Tuple[str, List[NormalizedDetection]]:
        return "detect", self.detect_frame(
            frame, None, "unknown", conf, iou, max_det, image_size
        )

    def detect_frame(
        self,
        frame: np.ndarray,
        header,
        source_id: str,
        conf: float = 0.3,
        iou: float = 0.3,
        max_det: int = 20,
        image_size: Optional[int] = None,
    ) -> List[NormalizedDetection]:
        if not self._loaded or self._detect_proxy is None:
            raise RuntimeError("shared YOLO model is not connected")
        if header is None:
            from std_msgs.msg import Header
            header = Header(stamp=rospy.Time.now())
        contiguous = np.ascontiguousarray(frame, dtype=np.uint8)
        if contiguous.ndim != 3 or contiguous.shape[2] != 3:
            raise ValueError("shared YOLO expects a BGR HxWx3 image")
        message = Image()
        message.header = header
        message.height, message.width = map(int, contiguous.shape[:2])
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = contiguous.tobytes()
        return self.detect_image(
            message, source_id, conf, iou, max_det, image_size
        )

    def detect_image(
        self,
        message: Image,
        source_id: str,
        conf: float = 0.3,
        iou: float = 0.3,
        max_det: int = 20,
        image_size: Optional[int] = None,
    ) -> List[NormalizedDetection]:
        if not self._loaded or self._detect_proxy is None:
            raise RuntimeError("shared YOLO model is not connected")
        request = RunSharedInferenceRequest(
            model_key=self._key,
            source_id=str(source_id),
            image=message,
            confidence=float(conf),
            iou=float(iou),
            max_detections=max(1, int(max_det)),
            image_size=int(image_size or 0),
        )
        response = self._detect_proxy(request)
        if not response.success:
            message = str(response.message or "")
            if (message.startswith("stale image discarded by shared inference") or
                    message == "superseded by a newer frame"):
                self._stale_frames_dropped += 1
                logger.debug("Dropped an outdated shared-inference frame: %s", message)
                return []
            raise RuntimeError(response.message or "shared YOLO inference failed")
        return [self._from_message(item) for item in response.detections]

    @staticmethod
    def _from_message(item: NormalizedDetectionMsg) -> NormalizedDetection:
        polygon = list(zip(item.polygon_xy[0::2], item.polygon_xy[1::2]))
        obb = tuple(float(value) for value in item.obb_xywhr)
        return NormalizedDetection(
            track_id=int(item.track_id),
            class_id=int(item.class_id),
            confidence=float(item.confidence),
            aabb_xyxy=tuple(int(round(value)) for value in item.aabb_xyxy),
            center_xy=tuple(int(round(value)) for value in item.center_xy),
            geometry_type=str(item.geometry_type or "aabb"),
            obb_xywhr=obb if item.geometry_type == "obb" else None,
            polygon_xy=polygon or None,
            rotation_deg=float(item.rotation_deg) if item.has_rotation else None,
            track_id_is_stable=bool(item.track_id_is_stable),
        )

    def detect_and_track(
        self,
        frame: np.ndarray,
        conf: float = 0.3,
        iou: float = 0.3,
        max_det: int = 20,
        tracker_type: str = "bytetrack",
        tracker_args: Optional[Dict] = None,
    ) -> Tuple[str, List[NormalizedDetection]]:
        del tracker_type, tracker_args
        # The remote model is detection-only. Temporal identities remain local
        # to each SmartTracker, so two aircraft never share a BoT-SORT state.
        return "detect", self.detect(frame, conf, iou, max_det)[1]

    def get_model_labels(self) -> Dict[int, str]:
        return dict(self._labels)

    def get_model_task(self) -> str:
        return self._task

    def supports_tracking(self) -> bool:
        return False

    def supports_obb(self) -> bool:
        return self._task.lower() == "obb"

    def get_device_info(self) -> Dict[str, Any]:
        return dict(self._runtime)
