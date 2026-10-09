"""Bounded single-object tracker used for user supplied image ROIs."""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


Box = Tuple[float, float, float, float]  # x, y, width, height


@dataclass(frozen=True)
class SotResult:
    bbox: Optional[Box]
    valid: bool
    confidence: float
    algorithm: str
    reason: str


def _factory(name):
    attribute = "Tracker{}_create".format(name.upper())
    for namespace in (cv2, getattr(cv2, "legacy", None)):
        candidate = getattr(namespace, attribute, None) if namespace else None
        if callable(candidate):
            return candidate()
    return None


def _clamp(box, width, height):
    x, y, box_width, box_height = [float(value) for value in box]
    x = max(0.0, min(x, width - 1.0))
    y = max(0.0, min(y, height - 1.0))
    return (x, y, max(1.0, min(box_width, width - x)),
            max(1.0, min(box_height, height - y)))


class SotTracker:
    """CSRT/KCF/MIL with a template fallback and geometric validation."""

    def __init__(self, algorithm="auto", max_failures=5,
                 maximum_center_speed_px_sec=800.0,
                 maximum_scale_ratio=2.5, template_threshold=0.45,
                 minimum_template_stddev=5.0):
        self.requested_algorithm = str(algorithm or "auto").lower()
        if self.requested_algorithm not in {
                "auto", "csrt", "kcf", "mil", "template"}:
            raise ValueError("unsupported SOT algorithm")
        self.max_failures = max(1, int(max_failures))
        self.maximum_center_speed = max(10.0, float(maximum_center_speed_px_sec))
        self.maximum_scale_ratio = max(1.1, float(maximum_scale_ratio))
        self.template_threshold = max(0.0, min(1.0, float(template_threshold)))
        self.minimum_template_stddev = max(0.0, float(minimum_template_stddev))
        self.reset()

    def reset(self):
        self._tracker = None
        self._template = None
        self._bbox = None
        self._anchor_bbox = None
        self._stamp = None
        self._failures = 0
        self.algorithm = "none"

    def initialize(self, frame, bbox, stamp_sec):
        if frame is None or frame.ndim != 3:
            return SotResult(None, False, 0.0, "none", "invalid_frame")
        height, width = frame.shape[:2]
        box = _clamp(bbox, width, height)
        if min(box[2], box[3]) < 4.0:
            return SotResult(box, False, 0.0, "none", "roi_too_small")
        self.reset()
        choices = ("csrt", "kcf", "mil") if self.requested_algorithm == "auto" \
            else (self.requested_algorithm,)
        for name in choices:
            if name == "template":
                break
            tracker = _factory(name)
            if tracker is None:
                continue
            try:
                initialized = tracker.init(
                    frame, tuple(int(round(value)) for value in box))
                if initialized is not False:
                    self._tracker, self.algorithm = tracker, name
                    break
            except (cv2.error, TypeError):
                continue
        if (self._tracker is None and
                self.requested_algorithm not in ("auto", "template")):
            return SotResult(box, False, 0.0, "none",
                             "requested_algorithm_unavailable")
        if self._tracker is None:
            x, y, w, h = [int(round(value)) for value in box]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            crop = gray[y:y + h, x:x + w]
            if crop.size == 0:
                return SotResult(box, False, 0.0, "none",
                                 "template_initialization_failed")
            if float(np.std(crop)) < self.minimum_template_stddev:
                return SotResult(box, False, 0.0, "none",
                                 "template_low_texture")
            self._template = crop.copy()
            self.algorithm = "template"
        self._bbox = box
        self._anchor_bbox = box
        self._stamp = float(stamp_sec)
        return SotResult(box, True, 1.0, self.algorithm, "initialized")

    def accept_anchor(self, bbox):
        """Refresh scale only after the caller validates real detector evidence.

        This does not retrain/recreate CSRT on every detection. Frame-to-frame
        validation alone cannot detect a gradual collapse onto a target part.
        """
        values = np.asarray(bbox, dtype=float)
        if values.shape != (4,) or not np.isfinite(values).all() or min(values[2:]) < 4:
            return False
        self._anchor_bbox = tuple(values)
        return True

    def update(self, frame, stamp_sec):
        if self._bbox is None or self._stamp is None:
            return SotResult(None, False, 0.0, self.algorithm, "not_initialized")
        dt = float(stamp_sec) - self._stamp
        if not 0.0 < dt <= 1.0:
            return self._failure(self._bbox, "invalid_frame_interval")
        if self._tracker is not None:
            try:
                ok, candidate = self._tracker.update(frame)
            except cv2.error:
                ok, candidate = False, self._bbox
            confidence = 1.0 if ok else 0.0
        else:
            candidate, confidence, ok = self._template_update(frame)
        if not ok or candidate is None:
            return self._failure(self._bbox, "tracker_update_failed")
        candidate = _clamp(candidate, frame.shape[1], frame.shape[0])
        valid, reason = self._validate(candidate, dt)
        if not valid:
            return self._failure(candidate, reason)
        self._bbox, self._stamp, self._failures = candidate, float(stamp_sec), 0
        return SotResult(candidate, True, confidence, self.algorithm, "ok")

    def _template_update(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        x, y, w, h = self._bbox
        tw, th = self._template.shape[1], self._template.shape[0]
        radius_x, radius_y = 1.5 * w, 1.5 * h
        left = max(0, int(round(x + 0.5 * w - radius_x)))
        top = max(0, int(round(y + 0.5 * h - radius_y)))
        right = min(gray.shape[1], int(round(x + 0.5 * w + radius_x)))
        bottom = min(gray.shape[0], int(round(y + 0.5 * h + radius_y)))
        search = gray[top:bottom, left:right]
        if search.shape[0] < th or search.shape[1] < tw:
            return self._bbox, 0.0, False
        response = cv2.matchTemplate(search, self._template,
                                     cv2.TM_CCOEFF_NORMED)
        _, score, _, location = cv2.minMaxLoc(response)
        return (float(left + location[0]), float(top + location[1]), w, h), \
            float(score), bool(np.isfinite(score) and
                               score >= self.template_threshold)

    def _validate(self, candidate, dt):
        old = self._bbox
        old_center = (old[0] + 0.5 * old[2], old[1] + 0.5 * old[3])
        new_center = (candidate[0] + 0.5 * candidate[2],
                      candidate[1] + 0.5 * candidate[3])
        speed = np.hypot(new_center[0] - old_center[0],
                         new_center[1] - old_center[1]) / dt
        if speed > self.maximum_center_speed:
            return False, "center_speed_gate"
        area_ratio = (candidate[2] * candidate[3]) / max(1.0, old[2] * old[3])
        if not 1.0 / self.maximum_scale_ratio <= area_ratio <= self.maximum_scale_ratio:
            return False, "scale_gate"
        anchor = self._anchor_bbox
        if anchor is not None:
            anchor_ratio = candidate[2] * candidate[3] / max(1.0, anchor[2] * anchor[3])
            if not 1.0 / self.maximum_scale_ratio <= anchor_ratio <= self.maximum_scale_ratio:
                return False, "anchor_scale_gate"
        return True, "ok"

    def _failure(self, bbox, reason):
        self._failures += 1
        if self._failures >= self.max_failures:
            self._tracker = None
            self._template = None
            self._bbox = None
            self._stamp = None
        # Keep the last box internally for recovery, never return it as new evidence.
        return SotResult(None, False, 0.0, self.algorithm, reason)
