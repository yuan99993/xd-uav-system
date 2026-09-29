"""Optional verified OSNet feature extractor.

The module intentionally has no temporal state: identity galleries and
re-acquisition decisions remain in the C++ multi-target tracker.
"""

from __future__ import annotations

import hashlib
import os
import stat
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np


def _resolve_model_path(raw_path: str) -> Path:
    """Resolve a package URI without weakening the regular-file trust gate."""
    if raw_path.startswith("package://"):
        package_and_path = raw_path[len("package://"):]
        package, separator, relative = package_and_path.partition("/")
        if not package or not separator or not relative:
            raise ValueError("invalid deep ReID package URI")
        # Avoid rospack's global crawl when the standard source path is
        # already available (especially important on field computers/NFS).
        for root in os.environ.get("ROS_PACKAGE_PATH", "").split(":"):
            candidate = Path(root) / package
            if candidate.is_dir():
                return candidate / relative
        try:
            import rospkg
            return Path(rospkg.RosPack().get_path(package)) / relative
        except Exception as error:
            raise ValueError("cannot resolve deep ReID package URI") from error
    return Path(raw_path).expanduser()


class DeepReIDModel:
    """Load one trusted Torchreid OSNet checkpoint and emit L2 features."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = dict(config or {})
        self.model_name = str(self.config.get("model_name", "osnet_x0_25")).strip()
        raw_path = str(self.config.get("model_path", "") or "").strip()
        if not raw_path:
            raise ValueError("deep ReID requires model_path")
        candidate = _resolve_model_path(raw_path)
        if candidate.is_symlink():
            raise ValueError("deep ReID model_path must not be a symbolic link")
        self.model_path = candidate.resolve(strict=True)
        if not self.model_path.is_file():
            raise ValueError("deep ReID model_path must be a regular file")
        self._verify_checkpoint()
        try:
            import torch
            import torch.nn.functional as torch_functional
            from torchreid import models as reid_models
        except Exception as error:
            raise RuntimeError("deep ReID requires torch and torchreid") from error
        self._torch = torch
        self._functional = torch_functional
        requested = str(self.config.get("device", "auto") or "auto").lower()
        use_cuda = requested in ("cuda", "gpu") or (
            requested == "auto" and bool(self.config.get("use_gpu", True))
        )
        if use_cuda and torch.cuda.is_available():
            self.device = "cuda"
        elif use_cuda and not bool(self.config.get("fallback_to_cpu", True)):
            raise RuntimeError("deep ReID requested CUDA but CUDA is unavailable")
        else:
            self.device = "cpu"
        self.input_height = max(16, int(self.config.get("input_height", 256)))
        self.input_width = max(16, int(self.config.get("input_width", 128)))
        self.mean = np.asarray(self.config.get("pixel_mean", [0.485, 0.456, 0.406]), dtype=np.float32)
        self.std = np.asarray(self.config.get("pixel_std", [0.229, 0.224, 0.225]), dtype=np.float32)
        if self.mean.shape != (3,) or self.std.shape != (3,) or np.any(self.std <= 0):
            raise ValueError("deep ReID normalization parameters are invalid")
        supported = {"osnet_x0_25", "osnet_x0_5", "osnet_x0_75", "osnet_x1_0", "osnet_ibn_x1_0"}
        if self.model_name not in supported:
            raise ValueError("unsupported deep ReID model: %s" % self.model_name)
        self.model = reid_models.build_model(
            name=self.model_name, num_classes=1, loss="softmax", pretrained=False, use_gpu=False
        )
        try:
            checkpoint = torch.load(str(self.model_path), map_location="cpu", weights_only=False)
        except TypeError:
            checkpoint = torch.load(str(self.model_path), map_location="cpu")
        state = checkpoint.get("state_dict", checkpoint) if isinstance(checkpoint, dict) else None
        if not isinstance(state, dict) or not state:
            raise ValueError("deep ReID checkpoint has no state dictionary")
        expected = self.model.state_dict()
        compatible = {}
        for key, value in state.items():
            normalized = str(key)
            for prefix in ("module.", "model."):
                if normalized.startswith(prefix):
                    normalized = normalized[len(prefix):]
            if normalized in expected and hasattr(value, "shape") and tuple(value.shape) == tuple(expected[normalized].shape):
                compatible[normalized] = value
        if len(compatible) < max(10, int(0.9 * len(expected))):
            raise ValueError("deep ReID checkpoint is incompatible with %s" % self.model_name)
        expected.update(compatible)
        self.model.load_state_dict(expected, strict=True)
        self.model.eval().to(torch.device(self.device))
        with torch.inference_mode():
            probe = self.model(torch.zeros((1, 3, self.input_height, self.input_width), device=self.device))
        if isinstance(probe, (tuple, list)):
            probe = probe[-1]
        self.dimension = int(probe.reshape(1, -1).shape[1])

    def _verify_checkpoint(self) -> None:
        maximum = int(self.config.get("maximum_model_bytes", 512 * 1024 * 1024))
        descriptor = os.open(str(self.model_path), os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0))
        try:
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_size <= 0 or metadata.st_size > maximum:
                raise ValueError("deep ReID checkpoint size/type is invalid")
            if stat.S_IMODE(metadata.st_mode) & 0o022:
                raise ValueError("deep ReID checkpoint must not be group/world writable")
            digest = hashlib.sha256()
            while True:
                block = os.read(descriptor, 1024 * 1024)
                if not block:
                    break
                digest.update(block)
            expected = str(self.config.get("sha256", "") or "").lower()
            if self.config.get("require_sha256", True) and len(expected) != 64:
                raise ValueError("deep ReID sha256 is required")
            if expected and digest.hexdigest() != expected:
                raise ValueError("deep ReID checkpoint SHA-256 mismatch")
            self.sha256 = digest.hexdigest()
        finally:
            os.close(descriptor)

    def encode_many(self, bgr_rois):
        """Encode one inference batch while preserving input order.

        The model is in eval/inference mode, so batching changes neither the
        network nor its normalization. Invalid ROIs remain ``None`` instead of
        shifting subsequent outputs.
        """
        outputs = [None] * len(bgr_rois)
        valid_indices = []
        prepared = []
        try:
            for index, bgr_roi in enumerate(bgr_rois):
                if (bgr_roi is None or bgr_roi.ndim != 3 or
                        bgr_roi.shape[2] != 3 or min(bgr_roi.shape[:2]) < 2):
                    continue
                rgb = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2RGB)
                image = cv2.resize(
                    rgb, (self.input_width, self.input_height)).astype(
                        np.float32) / 255.0
                prepared.append((image - self.mean) / self.std)
                valid_indices.append(index)
            if not prepared:
                return outputs
            batch = np.ascontiguousarray(np.stack(prepared, axis=0))
            tensor = self._torch.from_numpy(batch).permute(0, 3, 1, 2).to(
                self.device)
            with self._torch.inference_mode():
                feature = self.model(tensor)
            if isinstance(feature, (tuple, list)):
                feature = feature[-1]
            feature = self._functional.normalize(
                feature.reshape(len(prepared), -1), p=2, dim=1, eps=1e-12)
            host = feature.detach().cpu().numpy().astype(np.float32, copy=False)
            for row, output_index in enumerate(valid_indices):
                value = host[row]
                if value.size == self.dimension and np.isfinite(value).all():
                    outputs[output_index] = value
        except Exception:
            return [None] * len(bgr_rois)
        return outputs

    def encode(self, bgr_roi: np.ndarray) -> Optional[np.ndarray]:
        return self.encode_many([bgr_roi])[0]

    def close(self) -> None:
        self.model = None
        if self._torch.cuda.is_available():
            self._torch.cuda.empty_cache()
