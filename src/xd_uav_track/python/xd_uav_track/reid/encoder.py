"""Per-class stateless appearance descriptors for detector observations."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple
import logging

import cv2
import numpy as np

from .deep_reid_model import DeepReIDModel
from .onnx_reid_model import OnnxReIDModel

LOGGER = logging.getLogger(__name__)


class AppearanceEncoder:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = dict(config or {})
        self.minimum_width = max(2, int(self.config.get("minimum_roi_width_px", 16)))
        self.minimum_height = max(2, int(self.config.get("minimum_roi_height_px", 32)))
        self.edge_margin = max(0, int(self.config.get("edge_margin_px", 2)))
        self.histogram_bins = max(4, min(64, int(self.config.get("histogram_bins", 16))))
        self.hybrid_hue_bins = max(4, min(32, int(self.config.get("hybrid_hue_bins", 8))))
        self.hybrid_saturation_bins = max(2, min(16, int(self.config.get("hybrid_saturation_bins", 4))))
        self.hybrid_gradient_bins = max(4, min(18, int(self.config.get("hybrid_gradient_bins", 9))))
        self.default_backend = str(self.config.get("default_backend", "histogram")).lower()
        self._profiles = self._build_profiles(self.config)
        self._class_profiles = self._index_class_profiles(self._profiles)
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        self._deep_models: Dict[str, Any] = {}
        self._failed_deep_profiles = set()

    @staticmethod
    def _build_profiles(config: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        profile_name = str(config.get("active_model_profile", "default"))
        model_profiles = config.get("model_profiles", {})
        selected = model_profiles.get(profile_name, {}) if isinstance(model_profiles, dict) else {}
        profiles = selected.get("class_profiles", config.get("class_profiles", {}))
        return dict(profiles) if isinstance(profiles, dict) else {}

    @staticmethod
    def _index_class_profiles(profiles: Dict[str, Dict[str, Any]]):
        indexed = {}
        for name, profile in profiles.items():
            if not isinstance(profile, dict):
                continue
            for value in profile.get("class_ids", []):
                try:
                    indexed[int(value)] = (str(name), profile)
                except (TypeError, ValueError):
                    continue
        return indexed

    def _profile_for_class(self, class_id: int) -> Tuple[str, Dict[str, Any]]:
        return self._class_profiles.get(
            int(class_id), ("default", {"backend": self.default_backend}))

    @staticmethod
    def _normalise(feature: np.ndarray) -> Optional[np.ndarray]:
        if feature is None or feature.ndim != 1 or feature.size == 0 or not np.isfinite(feature).all():
            return None
        norm = float(np.linalg.norm(feature))
        return (feature / norm).astype(np.float32, copy=False) if norm > 1e-12 else None

    def _histogram(self, roi: np.ndarray) -> Optional[np.ndarray]:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        feature = cv2.calcHist([hsv], [0, 1], None, [self.histogram_bins, self.histogram_bins], [0, 180, 0, 256]).reshape(-1)
        return self._normalise(feature.astype(np.float32, copy=False))

    def _hybrid(self, roi: np.ndarray) -> Optional[np.ndarray]:
        """Lighting-tolerant colour, texture and spatial descriptor.

        It is intentionally deterministic and inexpensive. It is not called a
        deep ReID model, but is substantially less ambiguous than one global
        colour histogram when two vehicles cross or illumination changes.
        """
        sample = cv2.resize(roi, (64, 64), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(sample, cv2.COLOR_BGR2LAB)
        lab[:, :, 0] = self._clahe.apply(lab[:, :, 0])
        # Equalized Lab luminance is already the signal required for the
        # gradient descriptor; converting Lab->BGR->gray was redundant.
        gray = lab[:, :, 0]
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        magnitude, angle = cv2.cartToPolar(gx, gy, angleInDegrees=False)
        parts = []
        for row in range(2):
            for column in range(2):
                ys = slice(row * 32, (row + 1) * 32)
                xs = slice(column * 32, (column + 1) * 32)
                colour = cv2.calcHist(
                    [hsv[ys, xs]], [0, 1], None,
                    [self.hybrid_hue_bins, self.hybrid_saturation_bins],
                    [0, 180, 0, 256]).reshape(-1).astype(np.float32)
                colour /= max(1.0, float(colour.sum()))
                gradient, _ = np.histogram(
                    angle[ys, xs], bins=self.hybrid_gradient_bins,
                    range=(0.0, 2.0 * np.pi), weights=magnitude[ys, xs])
                gradient = gradient.astype(np.float32)
                gradient /= max(1e-6, float(np.linalg.norm(gradient)))
                parts.extend((colour, gradient))
        # Per-channel moments carry coarse chromaticity but are far less
        # sensitive to exposure than raw RGB values.
        lab_float = lab.astype(np.float32) / 255.0
        moments = np.concatenate((lab_float.mean(axis=(0, 1)),
                                  lab_float.std(axis=(0, 1)))).astype(np.float32)
        parts.append(moments)
        return self._normalise(np.concatenate(parts).astype(np.float32, copy=False))

    def _fallback_feature(
            self, profile: Dict[str, Any], roi: np.ndarray) -> Optional[np.ndarray]:
        """Return the configured low-cost fallback for an unavailable model.

        A fallback is selected for a whole profile only after model
        construction fails.  We intentionally do not mix a 512-D deep vector
        and a 170-D hybrid vector during normal operation.
        """
        backend = str(profile.get("fallback_backend", "none") or "none").lower()
        if backend == "hybrid":
            return self._hybrid(roi)
        if backend == "histogram":
            return self._histogram(roi)
        return None

    @staticmethod
    def quality(roi: np.ndarray) -> float:
        """Return a conservative visual reliability score without a model call.

        Blur, clipping and tiny patches are poor ReID evidence even when an
        embedding network returns a finite vector.  This score is metadata,
        not an identity decision; the C++ tracker lowers appearance influence
        rather than discarding valid geometric observations.
        """
        if roi is None or roi.ndim != 3 or roi.shape[2] != 3:
            return 0.0
        height, width = roi.shape[:2]
        if height < 2 or width < 2:
            return 0.0
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        sharpness = min(1.0, max(0.0, cv2.Laplacian(
            gray, cv2.CV_32F).var() / 120.0))
        # Penalize severe under/over exposure but do not assume a particular
        # lighting domain. Mid-range luminance remains neutral.
        low = float(np.mean(gray <= 8))
        high = float(np.mean(gray >= 247))
        exposure = max(0.0, 1.0 - 1.5 * (low + high))
        area = min(1.0, math.sqrt(float(width * height)) / 64.0)
        return float(min(1.0, max(0.0, area * (0.35 + 0.65 * sharpness) * exposure)))

    def encode(self, image: np.ndarray, bbox, class_id: int) -> Optional[np.ndarray]:
        return self.encode_many(image, [(bbox, class_id)])[0]

    def encode_many(self, image: np.ndarray, observations):
        """Encode multiple ROIs, batching only compatible deep profiles."""
        outputs = [None] * len(observations)
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            return outputs
        height, width = image.shape[:2]
        deep_groups = {}
        onnx_groups = {}
        for index, (bbox, class_id) in enumerate(observations):
            try:
                x1, y1, x2, y2 = [int(round(float(value))) for value in bbox]
            except (TypeError, ValueError):
                continue
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            if x2 - x1 < self.minimum_width or y2 - y1 < self.minimum_height:
                continue
            if self.edge_margin and (
                    x1 < self.edge_margin or y1 < self.edge_margin or
                    x2 > width - self.edge_margin or
                    y2 > height - self.edge_margin):
                continue
            name, profile = self._profile_for_class(class_id)
            backend = str(profile.get("backend", self.default_backend)).lower()
            roi = image[y1:y2, x1:x2]
            if backend == "histogram":
                outputs[index] = self._histogram(roi)
            elif backend == "hybrid":
                outputs[index] = self._hybrid(roi)
            elif backend == "onnx":
                if name in self._failed_deep_profiles:
                    outputs[index] = self._fallback_feature(profile, roi)
                    continue
                group = onnx_groups.setdefault(name, {
                    "profile": profile, "indices": [], "rois": []})
                group["indices"].append(index)
                group["rois"].append(roi)
            elif backend == "deep":
                if name in self._failed_deep_profiles:
                    outputs[index] = self._fallback_feature(profile, roi)
                    continue
                group = deep_groups.setdefault(name, {
                    "profile": profile, "indices": [], "rois": []})
                group["indices"].append(index)
                group["rois"].append(roi)
        for name, group in deep_groups.items():
            try:
                model = self._deep_models.get(name)
                if model is None:
                    model = DeepReIDModel(group["profile"])
                    self._deep_models[name] = model
                features = model.encode_many(group["rois"])
                for output_index, feature in zip(group["indices"], features):
                    outputs[output_index] = feature
            except Exception as error:
                self._disable_failed_profile(name, error)
                for output_index, roi in zip(group["indices"], group["rois"]):
                    outputs[output_index] = self._fallback_feature(
                        group["profile"], roi)
        for name, group in onnx_groups.items():
            try:
                model = self._deep_models.get(name)
                if model is None:
                    model = OnnxReIDModel(group["profile"])
                    self._deep_models[name] = model
                features = model.encode_many(group["rois"])
                for output_index, feature in zip(group["indices"], features):
                    outputs[output_index] = feature
            except Exception as error:
                self._disable_failed_profile(name, error)
                for output_index, roi in zip(group["indices"], group["rois"]):
                    outputs[output_index] = self._fallback_feature(
                        group["profile"], roi)
        return outputs

    def encode_many_with_quality(self, image: np.ndarray, observations):
        """Encode ROIs and return a same-order quality score for each one."""
        features = self.encode_many(image, observations)
        qualities = [0.0] * len(observations)
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            return features, qualities
        height, width = image.shape[:2]
        for index, ((bbox, class_id), feature) in enumerate(zip(observations, features)):
            if feature is None:
                continue
            try:
                x1, y1, x2, y2 = [int(round(float(value))) for value in bbox]
            except (TypeError, ValueError):
                continue
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            if x2 > x1 and y2 > y1:
                _, profile = self._profile_for_class(class_id)
                evidence_weight = min(1.0, max(0.0, float(
                    profile.get("association_weight", 1.0))))
                qualities[index] = evidence_weight * self.quality(
                    image[y1:y2, x1:x2])
        return features, qualities

    def _disable_failed_profile(self, name, error) -> None:
        if name not in self._failed_deep_profiles:
            self._failed_deep_profiles.add(name)
            LOGGER.warning(
                "deep ReID profile '%s' disabled; using its configured "
                "fallback before motion/spatial association: %s", name, error)

    def close(self) -> None:
        for model in self._deep_models.values():
            model.close()
        self._deep_models.clear()
