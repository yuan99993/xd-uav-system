"""Verified ONNX ReID encoder with selectable OpenCV or ONNX Runtime EPs."""

from __future__ import annotations

import hashlib
import importlib
import logging
import os
import stat
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from .deep_reid_model import _resolve_model_path

LOGGER = logging.getLogger(__name__)


class OnnxReIDModel:
    """Load a trusted ONNX embedding model without requiring PyTorch."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = dict(config or {})
        raw_path = str(self.config.get("model_path", "") or "").strip()
        if not raw_path:
            raise ValueError("ONNX ReID requires model_path")
        candidate = _resolve_model_path(raw_path)
        if candidate.is_symlink():
            raise ValueError("ONNX ReID model_path must not be a symbolic link")
        self.model_path = candidate.resolve(strict=True)
        self._verify_model()
        self.input_height = max(16, int(self.config.get("input_height", 256)))
        self.input_width = max(16, int(self.config.get("input_width", 128)))
        self.mean = tuple(float(v) for v in self.config.get(
            "pixel_mean_bgr", [0.406, 0.456, 0.485]))
        self.scale = float(self.config.get("input_scale", 1.0 / 255.0))
        self.swap_rb = bool(self.config.get("swap_rb", True))
        self.runtime = str(self.config.get("runtime", "opencv_dnn") or
                           "opencv_dnn").strip().lower()
        target = str(self.config.get("device", "cpu") or "cpu").lower()
        self.device_requested = "cuda" if target in {"cuda", "gpu"} else "cpu"
        self.effective_device = "cpu"
        self.net = None
        self._ort = None
        self._session = None
        self._input_name = ""
        self._output_name = ""
        if self.runtime in {"onnxruntime", "ort"}:
            self._init_onnxruntime()
        elif self.runtime in {"opencv", "opencv_dnn", "cv_dnn"}:
            self.runtime = "opencv_dnn"
            self._init_opencv_dnn()
        else:
            raise ValueError("unsupported ONNX ReID runtime: %s" % self.runtime)
        self.dimension = int(self.config.get("embedding_dimension", 0))

    def _init_opencv_dnn(self) -> None:
        self.net = cv2.dnn.readNetFromONNX(str(self.model_path))
        if self.device_requested == "cuda":
            try:
                if cv2.cuda.getCudaEnabledDeviceCount() <= 0:
                    raise RuntimeError("OpenCV has no CUDA device support")
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
                self.effective_device = "cuda"
            except Exception:
                if not bool(self.config.get("fallback_to_cpu", True)):
                    raise
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                LOGGER.warning("OpenCV-DNN CUDA unavailable; ReID explicitly fell back to CPU")
        else:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def _init_onnxruntime(self) -> None:
        try:
            ort = importlib.import_module("onnxruntime")
        except ImportError as error:
            raise RuntimeError("ONNX Runtime is required by the selected ReID profile") from error
        # OSNet's exported graph has many unused counters. Its CUDA kernels
        # may also print one warning per convolution on some hosts; our
        # measured runtime/device remains available through diagnostics.
        ort.set_default_logger_severity(3)
        # The SAR virtual environment ships CUDA/cuDNN libraries through the
        # NVIDIA Python wheels. ORT's helper loads those before the CUDA EP is
        # created, without changing LD_LIBRARY_PATH for unrelated ROS nodes.
        preload = getattr(ort, "preload_dlls", None)
        if self.device_requested == "cuda" and callable(preload):
            try:
                preload()
            except Exception as error:
                LOGGER.warning("ONNX Runtime CUDA library preload failed: %s", error)

        available = list(ort.get_available_providers())
        fallback = bool(self.config.get("fallback_to_cpu", True))
        options = ort.SessionOptions()
        # Exported OSNet contains many unused BatchNorm counters; suppress
        # per-initializer warnings so first model load does not flood roslaunch.
        options.log_severity_level = 3
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        options.intra_op_num_threads = max(1, int(self.config.get("intra_op_num_threads", 1)))
        options.inter_op_num_threads = max(1, int(self.config.get("inter_op_num_threads", 1)))
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        if self.device_requested == "cuda":
            if "CUDAExecutionProvider" not in available:
                if not fallback:
                    raise RuntimeError("ONNX Runtime CUDAExecutionProvider is unavailable")
                providers = ["CPUExecutionProvider"]
                LOGGER.warning("ONNX Runtime CUDA EP unavailable; ReID explicitly fell back to CPU")
            else:
                providers = [
                    ("CUDAExecutionProvider", {
                        "device_id": 0,
                        "cudnn_conv_algo_search": "DEFAULT",
                        "do_copy_in_default_stream": 1,
                    }),
                    "CPUExecutionProvider",
                ]
        else:
            providers = ["CPUExecutionProvider"]

        try:
            session = ort.InferenceSession(
                str(self.model_path), sess_options=options, providers=providers)
        except Exception as error:
            if self.device_requested != "cuda" or not fallback:
                raise
            LOGGER.warning("ONNX Runtime CUDA session failed; retrying CPU: %s", error)
            session = ort.InferenceSession(
                str(self.model_path), sess_options=options,
                providers=["CPUExecutionProvider"])
        session_providers = list(session.get_providers())
        if self.device_requested == "cuda" and "CUDAExecutionProvider" not in session_providers:
            if not fallback:
                raise RuntimeError("ONNX Runtime session did not activate CUDAExecutionProvider")
            LOGGER.warning("ONNX Runtime session is CPU-only; ReID is running on CPU")
        self.effective_device = (
            "cuda" if "CUDAExecutionProvider" in session_providers else "cpu")
        inputs = session.get_inputs()
        outputs = session.get_outputs()
        if not inputs or not outputs:
            raise RuntimeError("ONNX ReID model has no input or output")
        self._ort = ort
        self._session = session
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name
        LOGGER.info("ONNX ReID initialized: runtime=onnxruntime device=%s providers=%s",
                    self.effective_device, session_providers)

    def _verify_model(self) -> None:
        maximum = int(self.config.get("maximum_model_bytes", 512 * 1024 * 1024))
        descriptor = os.open(str(self.model_path), os.O_RDONLY |
                             getattr(os, "O_CLOEXEC", 0) |
                             getattr(os, "O_NOFOLLOW", 0))
        try:
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode) or not 0 < metadata.st_size <= maximum:
                raise ValueError("ONNX ReID model size/type is invalid")
            if stat.S_IMODE(metadata.st_mode) & 0o022:
                raise ValueError("ONNX ReID model must not be group/world writable")
            digest = hashlib.sha256()
            while True:
                block = os.read(descriptor, 1024 * 1024)
                if not block:
                    break
                digest.update(block)
            expected = str(self.config.get("sha256", "") or "").strip().lower()
            if self.config.get("require_sha256", True) and len(expected) != 64:
                raise ValueError("ONNX ReID sha256 is required")
            if expected and digest.hexdigest() != expected:
                raise ValueError("ONNX ReID model SHA-256 mismatch")
            self.sha256 = digest.hexdigest()
        finally:
            os.close(descriptor)

    def encode_many(self, bgr_rois) -> List[Optional[np.ndarray]]:
        """Run compatible ROIs in one OpenCV-DNN call.

        Invalid inputs retain their original positions.  Batching changes only
        scheduling and memory transfer; preprocessing and L2 normalization are
        identical to the legacy one-ROI path.
        """
        outputs: List[Optional[np.ndarray]] = [None] * len(bgr_rois)
        valid_indices = []
        valid_rois = []
        for index, roi in enumerate(bgr_rois):
            if (roi is None or roi.ndim != 3 or roi.shape[2] != 3 or
                    min(roi.shape[:2]) < 2):
                continue
            valid_indices.append(index)
            valid_rois.append(roi)
        if not valid_rois:
            return outputs
        blob = cv2.dnn.blobFromImages(
            valid_rois, scalefactor=self.scale,
            size=(self.input_width, self.input_height), mean=self.mean,
            swapRB=self.swap_rb, crop=False)
        if self._session is not None:
            raw = np.asarray(self._session.run(
                [self._output_name], {self._input_name: blob})[0], dtype=np.float32)
        else:
            self.net.setInput(blob)
            raw = np.asarray(self.net.forward(), dtype=np.float32)
        if raw.size == 0 or raw.size % len(valid_rois) != 0:
            return outputs
        features = raw.reshape(len(valid_rois), -1)
        inferred_dimension = int(features.shape[1])
        if self.dimension > 0 and inferred_dimension != self.dimension:
            return outputs
        if self.dimension == 0:
            self.dimension = inferred_dimension
        norms = np.linalg.norm(features, axis=1)
        for row, output_index in enumerate(valid_indices):
            if np.isfinite(features[row]).all() and float(norms[row]) > 1e-12:
                outputs[output_index] = (
                    features[row] / norms[row]).astype(np.float32, copy=False)
        return outputs

    def encode(self, bgr_roi: np.ndarray) -> Optional[np.ndarray]:
        return self.encode_many([bgr_roi])[0]

    def close(self) -> None:
        self.net = None
        self._session = None
        self._ort = None

    def runtime_status(self) -> str:
        return "%s/%s" % (self.runtime, self.effective_device)
