"""RT-DETRv2 (Transformers), detection only; identities belong to xd_uav_track.

The model_path is an integrity-pinned manifest, not a pickle checkpoint.
Loading is local-only and verifies every model/processor artifact first.
"""
import hashlib
import importlib.util
import json
import math

import numpy as np

from .ultralytics_backend import UltralyticsBackend
from ..detection_adapter import NormalizedDetection


class RTDetrBackend(UltralyticsBackend):
    @property
    def backend_name(self):
        return "rtdetr_v2"

    @property
    def is_available(self):
        return all(importlib.util.find_spec(m) is not None
                   for m in ("torch", "transformers", "safetensors"))

    def _load_candidate(self, path, target_device):
        provenance = self._verify_file(path)
        manifest = json.loads(path.read_text())
        if manifest.get("architecture") != "rtdetr_v2":
            raise ValueError("Expected an rtdetr_v2 manifest")
        required = {"config.json", "preprocessor_config.json", "model.safetensors"}
        if set(manifest.get("files", {})) != required:
            raise ValueError("Manifest must pin config, processor and safetensors")
        for name, expected in manifest["files"].items():
            artifact = path.parent / name
            if artifact.is_symlink() or not artifact.is_file():
                raise ValueError("Missing or symbolic-link artifact: " + name)
            if not 0 < artifact.stat().st_size <= self._max_model_bytes:
                raise ValueError("Artifact size limit: " + name)
            with artifact.open("rb") as stream:
                digest = hashlib.file_digest(stream, "sha256").hexdigest() if hasattr(hashlib, "file_digest") else self._digest(stream)
            if digest != expected:
                raise ValueError("Artifact SHA-256 mismatch: " + name)
        from transformers import RTDetrImageProcessor, RTDetrV2ForObjectDetection
        processor = RTDetrImageProcessor.from_pretrained(path.parent, local_files_only=True)
        model = RTDetrV2ForObjectDetection.from_pretrained(
            path.parent, local_files_only=True, use_safetensors=True).to(target_device).eval()
        names = {int(k): v for k, v in model.config.id2label.items()}
        if names != {i: n for i, n in enumerate(manifest["class_names"])}:
            raise ValueError("Manifest and model class contracts differ")
        model.names = names
        model.task = "detect"
        # Processor is attached to the candidate, so a failed model swap cannot
        # replace the active model's preprocessing configuration.
        model.airground_processor = processor
        return model, provenance

    @staticmethod
    def _digest(stream):
        digest = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
        return digest.hexdigest()

    def load_model(self, *args, **kwargs):
        result = super().load_model(*args, **kwargs)
        self._runtime_info.update(backend="rtdetr_v2_" + result["effective_device"],
                                  model_format="safetensors_manifest")
        return dict(self._runtime_info)

    def detect(self, frame, conf=0.3, iou=0.3, max_det=20, image_size=None):
        if self._model is None:
            raise RuntimeError("RT-DETRv2 model is not loaded")
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError("Expected a uint8 BGR image")
        import torch
        size = self._resolve_image_size(image_size)
        h, w = frame.shape[:2]
        processor = self._model.airground_processor
        inputs = processor(images=np.ascontiguousarray(frame[:, :, ::-1]),
                           size={"height": size, "width": size}, return_tensors="pt")
        inputs = inputs.to(self._model.device)
        with torch.inference_mode():
            output = self._model(**inputs)
        result = processor.post_process_object_detection(
            output, threshold=conf, target_sizes=torch.tensor([[h, w]], device=self._model.device))[0]
        records = []
        for box, score, label in zip(result["boxes"].tolist(), result["scores"].tolist(), result["labels"].tolist()):
            if self._allowed_class_ids is not None and label not in self._allowed_class_ids:
                continue
            if not all(math.isfinite(v) for v in (*box, score)):
                continue
            x1, y1, x2, y2 = box
            bbox = (max(0, min(w, math.floor(x1))), max(0, min(h, math.floor(y1))),
                    max(0, min(w, math.ceil(x2))), max(0, min(h, math.ceil(y2))))
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue
            records.append(NormalizedDetection(
                track_id=-1, class_id=label, confidence=float(score), aabb_xyxy=bbox,
                center_xy=((bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2),
                track_id_is_stable=False))
        # RT-DETR is end-to-end: no class-agnostic vehicle NMS is applied to
        # a mixed air/ground label space. max_det remains an explicit bound.
        return "detect", sorted(records, key=lambda d: -d.confidence)[:max_det]

    def detect_many(self, frames, conf=0.3, iou=0.3, max_det=20, image_size=None):
        # Serial inference bounds VRAM on the 4 GB development GPU.
        return [self.detect(f, conf, iou, max_det, image_size) for f in frames]

    def detect_and_track(self, frame, conf=0.3, iou=0.3, max_det=20,
                         tracker_type="bytetrack", tracker_args=None):
        return self.detect(frame, conf, iou, max_det)

    def supports_tracking(self):
        return False

    def supports_obb(self):
        return False

    def reset_tracking(self):
        pass
