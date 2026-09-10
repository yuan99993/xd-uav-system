# Deep person re-identification backend for sar_yolo_detector.
#
# The model wrapper deliberately keeps the ReID dependency optional.  A
# deployment that enables deep ReID must provision torchreid and a verified
# ReID checkpoint; detector-only deployments do not import torchreid.
"""OSNet-based deep person ReID feature extraction.

This module uses the OSNet implementation and checkpoint format from
``torchreid``.  It extracts the embedding before the classifier, L2 normalizes
it, and exposes a small NumPy API compatible with the existing AppearanceModel.
No model or dependency is downloaded implicitly at runtime.
"""

from __future__ import annotations

import hashlib
import logging
import os
import stat
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

try:  # Keep detector-only installations independent of torchreid.
    import torch
    import torch.nn.functional as torch_functional
except Exception as exc:  # pragma: no cover - exercised on hosts without torch
    torch = None
    torch_functional = None
    _TORCH_IMPORT_ERROR = "%s: %s" % (type(exc).__name__, exc)
else:
    _TORCH_IMPORT_ERROR = ""


logger = logging.getLogger(__name__)

DEFAULT_MODEL_NAME = "osnet_x0_25"
DEFAULT_INPUT_SIZE = (256, 128)  # height, width used by Torchreid models
DEFAULT_MAX_MODEL_BYTES = 512 * 1024 * 1024


class DeepReIDModel:
    """Verified OSNet embedding extractor for person re-identification."""

    def __init__(self, config: Dict[str, Any]):
        self.config = dict(config or {})
        self.model_name = str(
            self.config.get("DEEP_REID_MODEL_NAME", DEFAULT_MODEL_NAME)
        ).strip().lower()
        self.model_path = self._resolve_model_path(
            self.config.get("DEEP_REID_MODEL_PATH", "")
        )
        self.max_model_bytes = int(
            self.config.get("DEEP_REID_MODEL_MAX_BYTES", DEFAULT_MAX_MODEL_BYTES)
        )
        if not 0 < self.max_model_bytes <= DEFAULT_MAX_MODEL_BYTES:
            raise ValueError("DEEP_REID_MODEL_MAX_BYTES is outside 1..512 MiB")

        self.require_sha256 = bool(
            self.config.get("DEEP_REID_REQUIRE_SHA256", True)
        )
        self.expected_sha256 = self._normalize_digest(
            self.config.get("DEEP_REID_MODEL_SHA256", "")
        )
        self.provenance = self._verify_checkpoint()

        self.input_height = max(
            16, int(self.config.get("DEEP_REID_INPUT_HEIGHT", DEFAULT_INPUT_SIZE[0]))
        )
        self.input_width = max(
            16, int(self.config.get("DEEP_REID_INPUT_WIDTH", DEFAULT_INPUT_SIZE[1]))
        )
        self.pixel_mean = np.asarray(
            self.config.get("DEEP_REID_PIXEL_MEAN", [0.485, 0.456, 0.406]),
            dtype=np.float32,
        )
        self.pixel_std = np.asarray(
            self.config.get("DEEP_REID_PIXEL_STD", [0.229, 0.224, 0.225]),
            dtype=np.float32,
        )
        if self.pixel_mean.shape != (3,) or self.pixel_std.shape != (3,):
            raise ValueError("DEEP_REID_PIXEL_MEAN/STD must contain 3 values")
        if np.any(~np.isfinite(self.pixel_mean)) or np.any(
            ~np.isfinite(self.pixel_std)
        ) or np.any(self.pixel_std <= 0.0):
            raise ValueError("DEEP_REID_PIXEL_MEAN/STD must be finite and std > 0")

        self.device = self._select_device()
        self.model = self._load_model()
        self.embedding_dimension = self._infer_embedding_dimension()
        logger.info(
            "[DeepReIDModel] Loaded %s (%d-D) on %s from %s",
            self.model_name,
            self.embedding_dimension,
            self.device,
            self.model_path,
        )

    @staticmethod
    def _normalize_digest(value: Any) -> Optional[str]:
        normalized = str(value or "").strip().lower()
        if not normalized:
            return None
        if len(normalized) != 64 or any(
            character not in "0123456789abcdef" for character in normalized
        ):
            raise ValueError("DEEP_REID_MODEL_SHA256 must contain 64 hexadecimal characters")
        return normalized

    @staticmethod
    def _resolve_model_path(value: Any) -> Path:
        raw = str(value or "").strip()
        if not raw:
            raise ValueError(
                "DEEP_REID_MODEL_PATH is required when APPEARANCE_FEATURE_TYPE=deep"
            )
        candidate = Path(raw).expanduser()
        # Reject links before resolving so a trusted path cannot silently
        # redirect to an unreviewed checkpoint.
        if candidate.is_symlink():
            raise ValueError("DEEP_REID_MODEL_PATH must not be a symbolic link")
        path = candidate.resolve(strict=True)
        if not path.is_file():
            raise ValueError("DEEP_REID_MODEL_PATH must be a regular, non-symlink file")
        return path

    def _verify_checkpoint(self) -> Dict[str, Any]:
        descriptor = os.open(
            str(self.model_path),
            os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
        )
        try:
            before = os.fstat(descriptor)
            if not stat.S_ISREG(before.st_mode):
                raise ValueError("Deep ReID checkpoint must be a regular file")
            if before.st_size <= 0 or before.st_size > self.max_model_bytes:
                raise ValueError("Deep ReID checkpoint size is outside the configured limit")
            if stat.S_IMODE(before.st_mode) & 0o022:
                raise ValueError("Deep ReID checkpoint must not be group/world writable")

            digest = hashlib.sha256()
            while True:
                block = os.read(descriptor, 1024 * 1024)
                if not block:
                    break
                digest.update(block)
            after = os.fstat(descriptor)
            identity_before = (
                before.st_dev,
                before.st_ino,
                before.st_size,
                before.st_mtime_ns,
            )
            identity_after = (
                after.st_dev,
                after.st_ino,
                after.st_size,
                after.st_mtime_ns,
            )
            if identity_before != identity_after:
                raise ValueError("Deep ReID checkpoint changed while being verified")
            observed = digest.hexdigest()
        finally:
            os.close(descriptor)

        if self.require_sha256 and not self.expected_sha256:
            raise ValueError(
                "DEEP_REID_MODEL_SHA256 is required for the executable ReID checkpoint"
            )
        if self.expected_sha256 and observed != self.expected_sha256:
            raise ValueError(
                "Deep ReID checkpoint SHA-256 mismatch: %s" % observed
            )
        return {
            "verified": bool(self.expected_sha256),
            "sha256": observed,
            "expected_sha256": self.expected_sha256,
            "size_bytes": before.st_size,
        }

    def _select_device(self) -> str:
        requested = str(self.config.get("DEEP_REID_DEVICE", "auto") or "auto")
        requested = requested.strip().lower()
        if requested in ("gpu", "cuda"):
            requested = "cuda"
        elif requested not in ("auto", "cpu"):
            raise ValueError("DEEP_REID_DEVICE must be auto, cpu, or cuda")
        if requested == "auto":
            use_gpu = bool(
                self.config.get(
                    "DEEP_REID_USE_GPU",
                    self.config.get("SMART_TRACKER_USE_GPU", True),
                )
            )
            requested = "cuda" if use_gpu else "cpu"
        if requested == "cuda":
            if torch is None or not bool(torch.cuda.is_available()):
                if bool(self.config.get("DEEP_REID_FALLBACK_TO_CPU", True)):
                    logger.warning("[DeepReIDModel] CUDA unavailable; falling back to CPU")
                    return "cpu"
                raise RuntimeError("Deep ReID requested CUDA but CUDA is unavailable")
        return requested

    @staticmethod
    def _load_checkpoint(path: Path) -> Dict[str, Any]:
        try:
            return torch.load(str(path), map_location="cpu", weights_only=False)
        except TypeError:  # torch < 2.6
            return torch.load(str(path), map_location="cpu")

    @staticmethod
    def _state_dict_from_checkpoint(checkpoint: Any) -> Dict[str, Any]:
        if not isinstance(checkpoint, dict):
            raise ValueError("Deep ReID checkpoint must contain a state dictionary")
        state = checkpoint
        for key in ("state_dict", "model_state_dict", "model"):
            candidate = checkpoint.get(key)
            if isinstance(candidate, dict) and candidate:
                state = candidate
                break
        if not state:
            raise ValueError("Deep ReID checkpoint state dictionary is empty")
        return state

    def _load_model(self):
        if torch is None:
            raise RuntimeError("PyTorch is unavailable: %s" % _TORCH_IMPORT_ERROR)
        try:
            from torchreid import models as reid_models
        except Exception as exc:
            raise RuntimeError(
                "torchreid is required for deep ReID; install requirements-smart-tracker.txt"
            ) from exc

        supported = {
            "osnet_x0_25",
            "osnet_x0_5",
            "osnet_x0_75",
            "osnet_x1_0",
            "osnet_ibn_x1_0",
        }
        if self.model_name not in supported:
            raise ValueError(
                "Unsupported DEEP_REID_MODEL_NAME %r (supported: %s)"
                % (self.model_name, ", ".join(sorted(supported)))
            )

        model = reid_models.build_model(
            name=self.model_name,
            num_classes=1,
            loss="softmax",
            pretrained=False,
            use_gpu=False,
        )
        checkpoint = self._load_checkpoint(self.model_path)
        state = self._state_dict_from_checkpoint(checkpoint)
        model_state = model.state_dict()
        compatible = {}
        for key, value in state.items():
            normalized_key = str(key)
            for prefix in ("module.", "model."):
                if normalized_key.startswith(prefix):
                    normalized_key = normalized_key[len(prefix) :]
            if (
                normalized_key in model_state
                and hasattr(value, "shape")
                and tuple(value.shape) == tuple(model_state[normalized_key].shape)
            ):
                compatible[normalized_key] = value
        minimum_compatible = max(10, int(len(model_state) * 0.90))
        if len(compatible) < minimum_compatible:
            raise ValueError(
                "Deep ReID checkpoint has too few compatible OSNet parameters "
                "(%d/%d, minimum %d)"
                % (len(compatible), len(model_state), minimum_compatible)
            )
        model_state.update(compatible)
        model.load_state_dict(model_state, strict=True)
        model.eval()
        model.to(torch.device(self.device))
        self.loaded_parameter_count = len(compatible)
        return model

    def _infer_embedding_dimension(self) -> int:
        # OSNet returns the embedding in eval mode.  A tiny zero tensor avoids
        # relying on private model attributes and verifies the forward contract.
        sample = torch.zeros(
            (1, 3, self.input_height, self.input_width),
            dtype=torch.float32,
            device=torch.device(self.device),
        )
        with torch.inference_mode():
            output = self.model(sample)
        if isinstance(output, (tuple, list)):
            output = output[-1]
        if not hasattr(output, "shape") or output.ndim != 2 or output.shape[0] != 1:
            raise ValueError("Deep ReID model must return a [batch, embedding] tensor")
        return int(output.shape[1])

    @staticmethod
    def _crop(frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> Optional[np.ndarray]:
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            return None
        if frame.ndim != 3 or frame.shape[2] != 3:
            return None
        try:
            x1, y1, x2, y2 = (int(round(float(value))) for value in bbox)
        except (TypeError, ValueError):
            return None
        height, width = frame.shape[:2]
        x1 = max(0, min(x1, width - 1))
        y1 = max(0, min(y1, height - 1))
        x2 = max(x1 + 1, min(x2, width))
        y2 = max(y1 + 1, min(y2, height))
        if x2 <= x1 or y2 <= y1:
            return None
        return frame[y1:y2, x1:x2]

    def extract_features(
        self, frame: np.ndarray, bbox: Tuple[int, int, int, int]
    ) -> Optional[np.ndarray]:
        """Return an L2-normalized OSNet embedding for one BGR ROI."""
        roi = self._crop(frame, bbox)
        if roi is None or roi.shape[0] < 2 or roi.shape[1] < 2:
            return None
        try:
            rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(
                rgb, (self.input_width, self.input_height), interpolation=cv2.INTER_LINEAR
            )
            image = resized.astype(np.float32) / 255.0
            image = (image - self.pixel_mean) / self.pixel_std
            tensor = torch.from_numpy(np.ascontiguousarray(image)).permute(2, 0, 1)
            tensor = tensor.unsqueeze(0).to(torch.device(self.device))
            with torch.inference_mode():
                embedding = self.model(tensor)
            if isinstance(embedding, (tuple, list)):
                embedding = embedding[-1]
            embedding = embedding.reshape(embedding.shape[0], -1)
            embedding = torch_functional.normalize(embedding, p=2, dim=1, eps=1e-12)
            result = embedding[0].detach().cpu().numpy().astype(np.float32, copy=False)
            if result.ndim != 1 or result.size != self.embedding_dimension:
                return None
            return result
        except Exception:
            logger.exception("[DeepReIDModel] Feature extraction failed")
            return None

    def get_status(self) -> Dict[str, Any]:
        return {
            "model_name": self.model_name,
            "model_path": str(self.model_path),
            "device": self.device,
            "embedding_dimension": self.embedding_dimension,
            "loaded_parameter_count": self.loaded_parameter_count,
            "provenance": dict(self.provenance),
        }

    def close(self) -> None:
        """Release the model and any CUDA allocator pages deterministically."""
        self.model = None
        if torch is not None and torch.cuda.is_available():
            torch.cuda.empty_cache()


__all__ = ["DeepReIDModel"]
