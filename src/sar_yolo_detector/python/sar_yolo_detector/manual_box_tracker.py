"""Single-object image tracking initialized from a user supplied pixel ROI.

This module deliberately has no ROS dependency.  The ROS adapter owns topic
selection and message conversion, while this class owns OpenCV tracker state,
validation, and the learned colour model.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np


PixelBox = Tuple[float, float, float, float]


@dataclass(frozen=True)
class ManualTrackResult:
    bbox: Optional[PixelBox]
    confidence: float
    valid: bool
    reason: str
    tracker_name: str
    consecutive_failures: int


def _nested(config: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = config.get(key, {})
    return dict(value) if isinstance(value, dict) else {}


def _create_opencv_tracker(name: str):
    requested = str(name or "auto").strip().lower()
    namespaces = (cv2, getattr(cv2, "legacy", None))

    def factory(algorithm: str):
        attribute = "Tracker%s_create" % algorithm.upper()
        for namespace in namespaces:
            candidate = getattr(namespace, attribute, None) if namespace else None
            if callable(candidate):
                return candidate
        return None

    algorithms = ("csrt", "kcf", "mil") if requested == "auto" else (requested,)
    for algorithm in algorithms:
        candidate = factory(algorithm)
        if candidate is not None:
            return candidate(), algorithm
    return None, "template"


def _clamp_box(box: PixelBox, width: int, height: int) -> PixelBox:
    x, y, box_width, box_height = box
    x = max(0.0, min(float(x), max(0.0, float(width - 1))))
    y = max(0.0, min(float(y), max(0.0, float(height - 1))))
    box_width = max(1.0, min(float(box_width), float(width) - x))
    box_height = max(1.0, min(float(box_height), float(height) - y))
    return x, y, box_width, box_height


class ManualBoxTracker:
    """Feature/template or learned-colour tracker with bounded failure state."""

    VALID_TYPES = {"feature_tracker", "color_tracker"}
    PRESET_HSV_RANGES = {
        "red": (((0, 80, 50), (12, 255, 255)), ((168, 80, 50), (180, 255, 255))),
        "green": (((35, 60, 40), (90, 255, 255)),),
        "blue": (((90, 60, 40), (130, 255, 255)),),
        "white": (((0, 0, 180), (180, 60, 255)),),
        "black": (((0, 0, 0), (180, 255, 60)),),
    }

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = dict(config or {})
        self.tracker_type = str(
            self.config.get("tracker_type", "feature_tracker")
        ).strip().lower()
        if self.tracker_type not in self.VALID_TYPES:
            raise ValueError(
                "ManualBoxTracker/tracker_type must be feature_tracker or color_tracker"
            )

        feature = _nested(self.config, "feature_tracker")
        colour = _nested(self.config, "color_tracker")
        safety = _nested(feature, "tracker_safety")

        self.algorithm = str(feature.get("tracker_algorithm", "auto"))
        self.template_threshold = float(
            feature.get("template_match_threshold", 0.45)
        )
        self.template_search_expansion = max(
            1.0, float(feature.get("template_search_expansion", 2.5))
        )

        self.histogram_bins = max(8, int(colour.get("histogram_bins", 32)))
        self.minimum_saturation = max(
            0, min(255, int(colour.get("minimum_saturation", 40)))
        )
        self.minimum_value = max(0, min(255, int(colour.get("minimum_value", 32))))
        self.backprojection_threshold = max(
            0, min(255, int(colour.get("backprojection_threshold", 16)))
        )
        self.morph_kernel = max(1, int(colour.get("morph_kernel", 5)))
        self.term_iterations = max(1, int(colour.get("term_iterations", 10)))
        self.term_epsilon = max(0.1, float(colour.get("term_epsilon", 1.0)))
        configured_colour = str(colour.get("color", "") or "").strip()
        self.colour_source = "configured" if configured_colour else "roi"
        self.colour_name = (
            self._normalize_colour_name(configured_colour)
            if configured_colour
            else ""
        )
        self._custom_hsv_ranges = self._parse_hsv_ranges(
            colour.get("custom_hsv_ranges", [])
        )
        self.minimum_colour_area = max(
            1.0, float(colour.get("min_area_px", 80.0))
        )
        self.minimum_colour_width = max(
            1.0, float(colour.get("min_width_px", 4.0))
        )
        self.minimum_colour_height = max(
            1.0, float(colour.get("min_height_px", 4.0))
        )
        self._validate_configured_colour(self.colour_name)

        self.safety_enabled = bool(safety.get("enabled", True))
        self.max_center_jump_norm = max(
            0.0, float(safety.get("max_center_jump_norm", 0.35))
        )
        self.min_area_ratio = max(0.0, float(safety.get("min_area_ratio", 0.25)))
        self.max_area_ratio = max(
            self.min_area_ratio, float(safety.get("max_area_ratio", 4.0))
        )
        self.edge_margin_px = max(0.0, float(safety.get("edge_margin_px", 0.0)))
        self.max_fail_frames = max(1, int(safety.get("max_fail_frames", 3)))

        self._opencv_tracker = None
        self._tracker_name = self.tracker_type
        self._template = None
        self._colour_histogram = None
        self._last_bbox: Optional[PixelBox] = None
        self._active = False
        self._failures = 0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def tracker_name(self) -> str:
        return self._tracker_name

    @property
    def consecutive_failures(self) -> int:
        return self._failures

    @property
    def requires_initial_roi(self) -> bool:
        return not (
            self.tracker_type == "color_tracker"
            and self.colour_source == "configured"
        )

    def reset(self) -> None:
        self._opencv_tracker = None
        self._template = None
        self._colour_histogram = None
        self._last_bbox = None
        self._active = False
        self._failures = 0

    def configure_colour(self, colour_name: str = "") -> None:
        """Select global named-colour detection or ROI-learned colour tracking.

        A non-empty name enables global configured-colour detection. An empty
        name selects ROI learning and waits for one initial pixel box.
        """
        if self.tracker_type != "color_tracker":
            raise ValueError("colour configuration requires color_tracker")
        raw_name = str(colour_name or "").strip()
        next_source = "configured" if raw_name else "roi"
        next_name = self._normalize_colour_name(raw_name) if raw_name else ""
        self._validate_configured_colour(next_name)
        self.colour_source = next_source
        self.colour_name = next_name
        self.reset()

    def _validate_configured_colour(self, name: str) -> None:
        if name == "custom" and not self._custom_hsv_ranges:
            raise ValueError(
                "color_tracker/color=custom requires custom_hsv_ranges"
            )

    @classmethod
    def _normalize_colour_name(cls, value: Any) -> str:
        name = str(value or "red").strip().lower()
        if name not in cls.PRESET_HSV_RANGES and name != "custom":
            raise ValueError(
                "color_tracker/color must be red, green, blue, white, black, or custom"
            )
        return name

    @staticmethod
    def _parse_hsv_ranges(raw_ranges: Any):
        parsed = []
        for raw in raw_ranges if isinstance(raw_ranges, (list, tuple)) else ():
            if not isinstance(raw, (list, tuple)) or len(raw) != 6:
                raise ValueError(
                    "color_tracker/custom_hsv_ranges entries must contain 6 integers"
                )
            values = [int(value) for value in raw]
            lower = (
                max(0, min(180, values[0])),
                max(0, min(255, values[1])),
                max(0, min(255, values[2])),
            )
            upper = (
                max(0, min(180, values[3])),
                max(0, min(255, values[4])),
                max(0, min(255, values[5])),
            )
            if any(low > high for low, high in zip(lower, upper)):
                raise ValueError("color_tracker/custom_hsv_ranges lower bound exceeds upper bound")
            parsed.append((lower, upper))
        return tuple(parsed)

    def initialize(self, frame: np.ndarray, roi: PixelBox) -> ManualTrackResult:
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            self.reset()
            return self._result(None, 0.0, False, "invalid_bgr_frame")
        height, width = frame.shape[:2]
        if width <= 1 or height <= 1:
            self.reset()
            return self._result(None, 0.0, False, "invalid_image_size")

        bbox = _clamp_box(roi, width, height)
        if bbox[2] < 2.0 or bbox[3] < 2.0:
            self.reset()
            return self._result(bbox, 0.0, False, "initial_roi_too_small")

        self.reset()
        self._last_bbox = bbox
        if self.tracker_type == "color_tracker":
            if self.colour_source == "configured":
                return self._update_configured_colour(frame)
            if not self._initialize_colour(frame, bbox):
                self.reset()
                return self._result(
                    bbox,
                    0.0,
                    False,
                    "initial_roi_has_no_matching_colour",
                )
            self._tracker_name = "color_roi_camshift"
        else:
            tracker, name = _create_opencv_tracker(self.algorithm)
            self._tracker_name = name
            if tracker is not None:
                try:
                    tracker_bbox = tuple(int(round(value)) for value in bbox)
                    initialized = tracker.init(frame, tracker_bbox)
                    # OpenCV 4 Python bindings return either True or None on success.
                    if initialized is False:
                        tracker = None
                except Exception:
                    tracker = None
            if tracker is None:
                self._tracker_name = "template"
                self._template = self._extract_gray(frame, bbox)
                if self._template is None:
                    self.reset()
                    return self._result(bbox, 0.0, False, "template_initialization_failed")
            else:
                self._opencv_tracker = tracker

        self._active = True
        return self._result(bbox, 1.0, True, "initialized")

    def update(self, frame: np.ndarray) -> ManualTrackResult:
        if (
            self.tracker_type == "color_tracker"
            and self.colour_source == "configured"
        ):
            if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
                return self._configured_colour_failure("invalid_bgr_frame")
            return self._update_configured_colour(frame)
        if not self._active or self._last_bbox is None:
            return self._result(None, 0.0, False, "waiting_for_initial_roi")
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return self._record_failure(self._last_bbox, "invalid_bgr_frame")

        if self.tracker_type == "color_tracker":
            bbox, confidence, valid, reason = self._update_colour(frame)
        elif self._tracker_name == "template":
            bbox, confidence, valid, reason = self._update_template(frame)
        else:
            try:
                ok, candidate = self._opencv_tracker.update(frame)
            except Exception:
                ok, candidate = False, self._last_bbox
            bbox = tuple(float(value) for value in candidate) if candidate is not None else None
            confidence = 1.0 if ok else 0.0
            valid = bool(ok and bbox is not None)
            reason = "ok" if valid else "opencv_tracker_update_failed"

        if valid and bbox is not None:
            height, width = frame.shape[:2]
            bbox = _clamp_box(bbox, width, height)
            valid, reason = self._validate_bbox(bbox, self._last_bbox, width, height)
        if not valid or bbox is None:
            return self._record_failure(bbox or self._last_bbox, reason)

        self._last_bbox = bbox
        self._failures = 0
        return self._result(bbox, confidence, True, reason)

    def _result(
        self, bbox: Optional[PixelBox], confidence: float, valid: bool, reason: str
    ) -> ManualTrackResult:
        return ManualTrackResult(
            bbox=bbox,
            confidence=max(0.0, min(1.0, float(confidence))),
            valid=bool(valid),
            reason=str(reason),
            tracker_name=self._tracker_name,
            consecutive_failures=self._failures,
        )

    def _record_failure(self, bbox: Optional[PixelBox], reason: str) -> ManualTrackResult:
        self._failures += 1
        result = self._result(bbox, 0.0, False, reason)
        if self._failures >= self.max_fail_frames:
            self._active = False
            self._opencv_tracker = None
            self._template = None
            self._colour_histogram = None
        return result

    @staticmethod
    def _extract_gray(frame: np.ndarray, bbox: PixelBox) -> Optional[np.ndarray]:
        x, y, width, height = (int(round(value)) for value in bbox)
        crop = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)[y : y + height, x : x + width]
        return crop.copy() if crop.size else None

    def _update_template(self, frame: np.ndarray):
        if self._template is None or self._last_bbox is None:
            return self._last_bbox, 0.0, False, "template_unavailable"
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        template_height, template_width = self._template.shape[:2]
        x, y, width, height = self._last_bbox
        expansion = self.template_search_expansion
        search_width = max(width, template_width) * expansion
        search_height = max(height, template_height) * expansion
        x1 = max(0, int(round(x + width * 0.5 - search_width * 0.5)))
        y1 = max(0, int(round(y + height * 0.5 - search_height * 0.5)))
        x2 = min(gray.shape[1], int(round(x1 + search_width)))
        y2 = min(gray.shape[0], int(round(y1 + search_height)))
        search = gray[y1:y2, x1:x2]
        if search.shape[0] < template_height or search.shape[1] < template_width:
            return self._last_bbox, 0.0, False, "template_search_window_too_small"
        response = cv2.matchTemplate(search, self._template, cv2.TM_CCOEFF_NORMED)
        _, maximum, _, location = cv2.minMaxLoc(response)
        bbox = (
            float(x1 + location[0]),
            float(y1 + location[1]),
            float(template_width),
            float(template_height),
        )
        valid = bool(np.isfinite(maximum) and maximum >= self.template_threshold)
        return bbox, float(maximum) if np.isfinite(maximum) else 0.0, valid, (
            "ok" if valid else "template_confidence_too_low"
        )

    def _initialize_colour(self, frame: np.ndarray, bbox: PixelBox) -> bool:
        x, y, width, height = (int(round(value)) for value in bbox)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        crop = hsv[y : y + height, x : x + width]
        if crop.size == 0:
            return False
        mask = cv2.inRange(
            crop,
            (0, self.minimum_saturation, self.minimum_value),
            (180, 255, 255),
        )
        if cv2.countNonZero(mask) < 4:
            return False
        histogram = cv2.calcHist(
            [crop], [0], mask, [self.histogram_bins], [0, 180]
        )
        if histogram is None or float(histogram.max()) <= 0.0:
            return False
        self._colour_histogram = cv2.normalize(
            histogram, None, 0, 255, cv2.NORM_MINMAX
        )
        return True

    def _update_colour(self, frame: np.ndarray):
        if self._last_bbox is None:
            return self._last_bbox, 0.0, False, "colour_model_unavailable"
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if self._colour_histogram is None:
            return self._last_bbox, 0.0, False, "colour_model_unavailable"
        backprojection = cv2.calcBackProject(
            [hsv], [0], self._colour_histogram, [0, 180], 1
        )
        valid_pixels = cv2.inRange(
            hsv,
            (0, self.minimum_saturation, self.minimum_value),
            (180, 255, 255),
        )
        backprojection = cv2.bitwise_and(backprojection, valid_pixels)
        if self.backprojection_threshold > 0:
            _, backprojection = cv2.threshold(
                backprojection,
                self.backprojection_threshold,
                255,
                cv2.THRESH_TOZERO,
            )
        if self.morph_kernel > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel)
            )
            backprojection = cv2.morphologyEx(
                backprojection, cv2.MORPH_OPEN, kernel
            )
            backprojection = cv2.morphologyEx(
                backprojection, cv2.MORPH_CLOSE, kernel
            )

        window = (
            max(0, int(round(self._last_bbox[0]))),
            max(0, int(round(self._last_bbox[1]))),
            max(1, int(round(self._last_bbox[2]))),
            max(1, int(round(self._last_bbox[3]))),
        )
        criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            self.term_iterations,
            self.term_epsilon,
        )
        try:
            rotated, updated_window = cv2.CamShift(backprojection, window, criteria)
        except cv2.error:
            return self._last_bbox, 0.0, False, "camshift_update_failed"
        points = cv2.boxPoints(rotated)
        x, y, width, height = cv2.boundingRect(np.intp(points))
        bbox = (float(x), float(y), float(width), float(height))
        roi = backprojection[y : y + height, x : x + width]
        confidence = float(np.mean(roi) / 255.0) if roi.size else 0.0
        valid = width >= 2 and height >= 2 and updated_window[2] >= 2 and updated_window[3] >= 2
        return bbox, confidence, valid, "ok" if valid else "colour_target_not_found"

    def _update_configured_colour(self, frame: np.ndarray) -> ManualTrackResult:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._configured_colour_mask(hsv)
        if self.morph_kernel > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours_info = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
        best_bbox = None
        best_area = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.minimum_colour_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if (
                width < self.minimum_colour_width
                or height < self.minimum_colour_height
            ):
                continue
            if area > best_area:
                best_area = area
                best_bbox = (float(x), float(y), float(width), float(height))

        self._tracker_name = "color_%s_detector" % self.colour_name
        if best_bbox is None:
            return self._configured_colour_failure("configured_colour_not_found")
        image_area = float(frame.shape[0] * frame.shape[1])
        confidence = min(1.0, best_area / max(1.0, image_area * 0.001))
        self._last_bbox = best_bbox
        self._active = True
        self._failures = 0
        return self._result(best_bbox, confidence, True, "ok")

    def _configured_colour_failure(self, reason: str) -> ManualTrackResult:
        self._failures += 1
        self._last_bbox = None
        self._active = False
        return self._result(None, 0.0, False, reason)

    def _configured_colour_mask(self, hsv: np.ndarray) -> np.ndarray:
        ranges = (
            self._custom_hsv_ranges
            if self.colour_name == "custom"
            else self.PRESET_HSV_RANGES[self.colour_name]
        )
        if not ranges:
            raise ValueError(
                "color_tracker/color=custom requires custom_hsv_ranges"
            )
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        return mask

    def _validate_bbox(
        self,
        bbox: PixelBox,
        previous: PixelBox,
        image_width: int,
        image_height: int,
    ) -> Tuple[bool, str]:
        if not self.safety_enabled:
            return True, "ok"
        x, y, width, height = bbox
        previous_x, previous_y, previous_width, previous_height = previous
        if width < 2.0 or height < 2.0:
            return False, "bbox_too_small"
        margin = self.edge_margin_px
        if margin > 0.0 and (
            x <= margin
            or y <= margin
            or x + width >= image_width - margin
            or y + height >= image_height - margin
        ):
            return False, "bbox_too_close_to_image_edge"
        previous_area = max(1.0, previous_width * previous_height)
        area_ratio = width * height / previous_area
        if area_ratio < self.min_area_ratio:
            return False, "bbox_area_too_small"
        if area_ratio > self.max_area_ratio:
            return False, "bbox_area_too_large"
        center_x = x + width * 0.5
        center_y = y + height * 0.5
        previous_center_x = previous_x + previous_width * 0.5
        previous_center_y = previous_y + previous_height * 0.5
        delta_x = (center_x - previous_center_x) / max(1.0, float(image_width))
        delta_y = (center_y - previous_center_y) / max(1.0, float(image_height))
        if (delta_x * delta_x + delta_y * delta_y) ** 0.5 > self.max_center_jump_norm:
            return False, "bbox_center_jump_too_large"
        return True, "ok"
