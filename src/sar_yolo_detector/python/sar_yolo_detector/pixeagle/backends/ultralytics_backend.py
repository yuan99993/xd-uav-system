# Derived from PixEagle under Apache-2.0; see THIRD_PARTY_NOTICES.md.
"""Ultralytics backend adapted to sar_yolo_detector's model-integrity contract."""

from __future__ import annotations

import hashlib
import logging
import math
import os
import stat
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .detection_backend import DetectionBackend, DevicePreference
from ..detection_adapter import NormalizedDetection
from ..geometry_utils import obb_xywhr_to_aabb, validate_obb_xywhr

# Runtime dependency installation is inappropriate on an aircraft or other
# managed ROS host. Missing packages must cause an explicit startup/inference
# error and be provisioned from requirements-smart-tracker.txt instead.
os.environ.setdefault("YOLO_AUTOINSTALL", "false")

try:
    from ultralytics import YOLO

    ULTRALYTICS_AVAILABLE = True
    ULTRALYTICS_IMPORT_ERROR = ""
except Exception as exc:  # Optional runtime dependency.
    YOLO = None
    ULTRALYTICS_AVAILABLE = False
    ULTRALYTICS_IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


logger = logging.getLogger(__name__)
DEFAULT_TRACKER_TYPE = "botsort"
DEFAULT_MAX_MODEL_BYTES = 512 * 1024 * 1024


class UltralyticsBackend(DetectionBackend):
    """YOLO detect/OBB backend with CPU/CUDA fallback and SHA-256 verification."""

    def __init__(self, config: dict, *, models_root: Optional[Path] = None):
        self._config = dict(config or {})
        configured_root = self._config.get("SMART_TRACKER_MODELS_ROOT")
        self._models_root = Path(
            models_root or configured_root or Path.cwd()
        ).expanduser().resolve()
        self._max_model_bytes = int(
            self._config.get("SMART_TRACKER_MODEL_MAX_BYTES", DEFAULT_MAX_MODEL_BYTES)
        )
        if not 0 < self._max_model_bytes <= DEFAULT_MAX_MODEL_BYTES:
            raise ValueError("SMART_TRACKER_MODEL_MAX_BYTES is outside 1..512 MiB")
        self._require_sha256 = bool(
            self._config.get("SMART_TRACKER_REQUIRE_MODEL_SHA256", True)
        )
        self._model = None
        self._runtime_info: Dict[str, Any] = {}
        raw_allowed_class_ids = self._config.get("SMART_TRACKER_ALLOWED_CLASS_IDS")
        self._allowed_class_ids = (
            sorted({int(class_id) for class_id in raw_allowed_class_ids})
            if raw_allowed_class_ids is not None
            else None
        )
        self.tracker_type_str, self.use_custom_reid = self._select_tracker_type()
        self.tracker_args = {"persist": True, "verbose": False}

    @property
    def is_available(self) -> bool:
        return ULTRALYTICS_AVAILABLE

    @property
    def backend_name(self) -> str:
        return "ultralytics"

    @property
    def is_loaded(self) -> bool:
        return self._model is not None

    def _select_tracker_type(self) -> Tuple[str, bool]:
        requested = str(
            self._config.get("TRACKER_TYPE", DEFAULT_TRACKER_TYPE)
        ).strip().lower()
        if requested == "bytetrack":
            return "bytetrack", False
        if requested == "custom_reid":
            return "bytetrack", True
        if requested != "botsort":
            logger.warning("Unknown tracker type %s; using botsort", requested)
        return "botsort", False

    @staticmethod
    def _device_name(device: Any) -> str:
        raw = device.value if isinstance(device, DevicePreference) else str(device)
        normalized = str(raw or "auto").strip().lower()
        if normalized in {"gpu", "cuda"}:
            return "cuda"
        if normalized == "cpu":
            return "cpu"
        return "auto"

    @staticmethod
    def _cuda_available() -> bool:
        try:
            import torch

            return bool(torch.cuda.is_available())
        except Exception:
            return False

    @staticmethod
    def _clear_cuda_cache() -> None:
        try:
            import torch

            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:
            pass

    def _resolve_model_path(self, model_path: str) -> Path:
        candidate = Path(str(model_path or "").replace("\\", "/")).expanduser()
        if not candidate.is_absolute():
            direct = self._models_root / candidate
            candidate = direct if direct.exists() else self._models_root / candidate.name
        if candidate.is_symlink():
            raise ValueError("SmartTracker model must not be a symbolic link")
        resolved = candidate.resolve(strict=True)
        if not (resolved.is_file() or resolved.is_dir()):
            raise ValueError(f"Unsupported model artifact: {resolved}")
        return resolved

    def _expected_sha256(self, path: Path) -> Optional[str]:
        mapping = self._config.get("SMART_TRACKER_MODEL_SHA256_BY_NAME", {})
        value = mapping.get(path.name) if isinstance(mapping, dict) else None
        if not value:
            configured_path = str(
                self._config.get("SMART_TRACKER_GPU_MODEL_PATH", "")
            )
            if Path(configured_path).name == path.name:
                value = self._config.get("SMART_TRACKER_MODEL_SHA256")
        normalized = str(value or "").strip().lower()
        if not normalized:
            if self._require_sha256 and path.is_file():
                raise ValueError(
                    f"No trusted SHA-256 configured for executable model {path.name}"
                )
            return None
        if len(normalized) != 64 or any(c not in "0123456789abcdef" for c in normalized):
            raise ValueError("Configured model SHA-256 must contain 64 hexadecimal characters")
        return normalized

    def authorize_model_digest(self, model_path: str, sha256: str) -> None:
        """Bind a publisher/operator supplied digest to one model filename."""
        normalized = str(sha256 or "").strip().lower()
        if len(normalized) != 64 or any(c not in "0123456789abcdef" for c in normalized):
            raise ValueError("Model SHA-256 must contain 64 hexadecimal characters")
        mapping = self._config.setdefault("SMART_TRACKER_MODEL_SHA256_BY_NAME", {})
        if not isinstance(mapping, dict):
            raise ValueError("SMART_TRACKER_MODEL_SHA256_BY_NAME must be a mapping")
        mapping[Path(str(model_path)).name] = normalized

    def _verify_file(self, path: Path) -> Dict[str, Any]:
        expected = self._expected_sha256(path)
        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(str(path), flags)
        try:
            before = os.fstat(descriptor)
            if not stat.S_ISREG(before.st_mode):
                raise ValueError("SmartTracker .pt artifact must be a regular file")
            if before.st_size <= 0 or before.st_size > self._max_model_bytes:
                raise ValueError("SmartTracker model size is outside the configured limit")
            if stat.S_IMODE(before.st_mode) & 0o022:
                raise ValueError("SmartTracker model must not be group/world writable")
            digest = hashlib.sha256()
            while True:
                block = os.read(descriptor, 1024 * 1024)
                if not block:
                    break
                digest.update(block)
            after = os.fstat(descriptor)
            identity_before = (
                before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns
            )
            identity_after = (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns
            )
            if identity_before != identity_after:
                raise ValueError("SmartTracker model changed while it was being verified")
            observed = digest.hexdigest()
            if expected and observed != expected:
                raise ValueError(
                    f"SmartTracker model SHA-256 mismatch for {path.name}: {observed}"
                )
            return {
                "verified": bool(expected),
                "sha256": observed,
                "expected_sha256": expected,
                "size_bytes": before.st_size,
            }
        finally:
            os.close(descriptor)

    def _load_candidate(self, path: Path, target_device: str):
        if not ULTRALYTICS_AVAILABLE:
            raise RuntimeError(
                "Ultralytics is unavailable: " + ULTRALYTICS_IMPORT_ERROR
            )
        if path.is_file() and path.suffix.lower() != ".pt":
            raise ValueError("Direct SmartTracker backend accepts .pt or NCNN directories")
        provenance = self._verify_file(path) if path.is_file() else {
            "verified": False,
            "sha256": None,
            "size_bytes": None,
        }
        model = YOLO(str(path))
        if target_device == "cuda":
            if not self._cuda_available():
                raise RuntimeError("CUDA requested but torch.cuda.is_available() is false")
            model.to("cuda")
        return model, provenance

    def load_model(
        self,
        model_path: str,
        device: DevicePreference = DevicePreference.AUTO,
        fallback_enabled: bool = True,
        context: str = "startup",
    ) -> Dict[str, Any]:
        requested = self._device_name(device)
        primary_device = (
            "cuda" if requested == "cuda" or (requested == "auto" and self._cuda_available())
            else "cpu"
        )
        attempts: List[Dict[str, Any]] = []
        candidates: List[Tuple[str, str]] = [(model_path, primary_device)]
        if primary_device == "cuda" and fallback_enabled:
            cpu_path = str(
                self._config.get("SMART_TRACKER_CPU_MODEL_PATH", model_path)
            )
            candidates.append((cpu_path, "cpu"))

        loaded = None
        selected_path: Optional[Path] = None
        selected_device = primary_device
        provenance: Dict[str, Any] = {}
        for candidate_name, candidate_device in candidates:
            try:
                path = self._resolve_model_path(candidate_name)
                loaded, provenance = self._load_candidate(path, candidate_device)
                selected_path = path
                selected_device = candidate_device
                attempts.append({"path": str(path), "device": candidate_device, "success": True})
                break
            except Exception as exc:
                attempts.append({
                    "path": str(candidate_name),
                    "device": candidate_device,
                    "success": False,
                    "error": f"{type(exc).__name__}: {exc}",
                })
        if loaded is None or selected_path is None:
            errors = "; ".join(a.get("error", "unknown") for a in attempts)
            raise RuntimeError(f"No SmartTracker model candidate could be loaded: {errors}")

        previous = self._model
        self._model = loaded
        previous = None
        self._clear_cuda_cache()
        self._runtime_info = {
            "requested_device": requested,
            "effective_device": selected_device,
            "backend": "cuda_torch" if selected_device == "cuda" else "cpu_torch",
            "model_path": str(selected_path),
            "model_name": selected_path.name,
            "fallback_enabled": bool(fallback_enabled),
            "fallback_occurred": selected_device != primary_device,
            "fallback_reason": attempts[0].get("error") if len(attempts) > 1 else None,
            "model_provenance": provenance,
            "artifact_sha256": provenance.get("sha256"),
            "context": context,
            "attempts": attempts,
        }
        return dict(self._runtime_info)

    def unload_model(self) -> None:
        self._model = None
        self._runtime_info = {}
        self._clear_cuda_cache()

    close = unload_model

    def switch_model(
        self,
        new_model_path: str,
        device: DevicePreference = DevicePreference.AUTO,
        fallback_enabled: bool = True,
    ) -> Dict[str, Any]:
        previous_model = self._model
        previous_info = dict(self._runtime_info)
        try:
            return self.load_model(
                new_model_path, device, fallback_enabled, context="switch"
            )
        except Exception:
            self._model = previous_model
            self._runtime_info = previous_info
            raise

    def detect(
        self, frame, conf: float = 0.3, iou: float = 0.3, max_det: int = 20
    ) -> Tuple[str, List[NormalizedDetection]]:
        if self._model is None:
            raise RuntimeError("SmartTracker model is not loaded")
        inference_args = {}
        if self._allowed_class_ids is not None:
            inference_args["classes"] = self._allowed_class_ids
        results = self._model.predict(
            frame,
            conf=conf,
            iou=iou,
            max_det=max_det,
            verbose=False,
            **inference_args,
        )
        return self._normalize_results(results)

    def detect_and_track(
        self,
        frame,
        conf: float = 0.3,
        iou: float = 0.3,
        max_det: int = 20,
        tracker_type: str = "bytetrack",
        tracker_args: Optional[Dict] = None,
    ) -> Tuple[str, List[NormalizedDetection]]:
        if self._model is None:
            raise RuntimeError("SmartTracker model is not loaded")
        args = dict(tracker_args or self.tracker_args)
        if self._allowed_class_ids is not None:
            args["classes"] = self._allowed_class_ids
        results = self._model.track(
            frame,
            conf=conf,
            iou=iou,
            max_det=max_det,
            tracker=f"{tracker_type}.yaml",
            **args,
        )
        return self._normalize_results(results)

    def get_model_labels(self) -> Dict[int, str]:
        return dict(getattr(self._model, "names", {}) or {}) if self._model else {}

    def get_model_task(self) -> str:
        return str(getattr(self._model, "task", "detect")) if self._model else "detect"

    def supports_tracking(self) -> bool:
        return True

    def supports_obb(self) -> bool:
        return True

    def get_device_info(self) -> Dict[str, Any]:
        return dict(self._runtime_info)

    @staticmethod
    def _to_list(value: Any) -> List[Any]:
        if value is None:
            return []
        try:
            if hasattr(value, "detach"):
                value = value.detach()
            if hasattr(value, "cpu"):
                value = value.cpu()
            if hasattr(value, "numpy"):
                value = value.numpy()
            if hasattr(value, "tolist"):
                return value.tolist()
        except Exception:
            pass
        return value if isinstance(value, list) else []

    @classmethod
    def _parse_boxes(cls, result: Any) -> List[NormalizedDetection]:
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []
        xyxy = cls._to_list(getattr(boxes, "xyxy", None))
        confs = cls._to_list(getattr(boxes, "conf", None))
        classes = cls._to_list(getattr(boxes, "cls", None))
        ids = cls._to_list(getattr(boxes, "id", None))
        output = []
        for index in range(min(len(xyxy), len(confs), len(classes))):
            values = tuple(float(v) for v in xyxy[index][:4])
            if len(values) != 4 or not all(math.isfinite(v) for v in values):
                continue
            x1, y1, x2, y2 = values
            aabb = (int(x1), int(y1), int(x2), int(y2))
            stable = index < len(ids) and ids[index] is not None
            output.append(NormalizedDetection(
                track_id=int(ids[index]) if stable else -(index + 1),
                class_id=int(classes[index]),
                confidence=float(confs[index]),
                aabb_xyxy=aabb,
                center_xy=((aabb[0] + aabb[2]) // 2, (aabb[1] + aabb[3]) // 2),
                geometry_type="aabb",
                track_id_is_stable=stable,
            ))
        return output

    @classmethod
    def _parse_obb(cls, result: Any) -> List[NormalizedDetection]:
        obb = getattr(result, "obb", None)
        if obb is None:
            return []
        xywhr = cls._to_list(getattr(obb, "xywhr", None))
        confs = cls._to_list(getattr(obb, "conf", None))
        classes = cls._to_list(getattr(obb, "cls", None))
        ids = cls._to_list(getattr(obb, "id", None))
        polygons = cls._to_list(getattr(obb, "xyxyxyxy", None))
        output = []
        for index in range(min(len(xywhr), len(confs), len(classes))):
            values = tuple(float(v) for v in xywhr[index][:5])
            if not validate_obb_xywhr(values):
                continue
            try:
                aabb = obb_xywhr_to_aabb(values)
            except Exception:
                continue
            polygon = None
            if index < len(polygons) and len(polygons[index]) == 4:
                polygon = [(float(p[0]), float(p[1])) for p in polygons[index]]
            stable = index < len(ids) and ids[index] is not None
            output.append(NormalizedDetection(
                track_id=int(ids[index]) if stable else -(index + 1),
                class_id=int(classes[index]),
                confidence=float(confs[index]),
                aabb_xyxy=aabb,
                center_xy=((aabb[0] + aabb[2]) // 2, (aabb[1] + aabb[3]) // 2),
                geometry_type="obb",
                obb_xywhr=values,
                polygon_xy=polygon,
                rotation_deg=math.degrees(values[4]),
                track_id_is_stable=stable,
            ))
        return output

    @classmethod
    def _normalize_results(cls, results: Any) -> Tuple[str, List[NormalizedDetection]]:
        if not results:
            return "none", []
        result = results[0]
        obb = getattr(result, "obb", None)
        boxes = getattr(result, "boxes", None)
        has_obb = obb is not None and len(getattr(obb, "data", [])) > 0
        has_boxes = boxes is not None and len(getattr(boxes, "data", [])) > 0
        if has_obb:
            return "obb", cls._parse_obb(result)
        if has_boxes:
            return "detect", cls._parse_boxes(result)
        return "none", []


__all__ = ["UltralyticsBackend", "ULTRALYTICS_AVAILABLE"]
