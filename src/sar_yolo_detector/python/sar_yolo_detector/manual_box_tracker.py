"""Single-object image tracking initialized from a user supplied pixel ROI.

This module deliberately has no ROS dependency.  The ROS adapter owns topic
selection and message conversion, while this class owns OpenCV tracker state,
validation, and the learned colour model.
"""

from __future__ import annotations

import math
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
        self.require_opencv_tracker = bool(feature.get("require_opencv_tracker", False))
        if self.tracker_type == 'feature_tracker' and self.require_opencv_tracker:
            probe, actual = _create_opencv_tracker(self.algorithm)
            if probe is None or actual == 'template':
                raise ValueError('Configured OpenCV tracker is unavailable: '+self.algorithm)
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
        self.colour_search_expansion = max(1., float(colour.get('search_expansion', 2.5)))
        self.colour_max_reference_area_ratio = max(1., float(colour.get('max_reference_area_ratio', 3.)))
        configured_colour = str(colour.get("color", "") or "").strip()
        if configured_colour.lower() == "auto":
            self.colour_source, self.colour_name = "auto", "auto"
        elif configured_colour.lower() == "roi" or not configured_colour:
            self.colour_source, self.colour_name = "roi", ""
        else:
            self.colour_source = "configured"
            self.colour_name = self._normalize_colour_name(configured_colour)
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
        self.target_shape = str(colour.get("target_shape", "vehicle")).strip().lower()
        if self.target_shape not in {"vehicle", "drone", "compact", "any"}:
            raise ValueError("color_tracker/target_shape must be vehicle, drone, compact, or any")
        self.minimum_shape_solidity = max(0., min(1., float(colour.get("minimum_shape_solidity", .45))))
        self.minimum_shape_extent = max(0., min(1., float(colour.get("minimum_shape_extent", .16))))
        self.minimum_local_colour_contrast = max(0., min(1., float(colour.get("minimum_local_colour_contrast", .12))))
        self.maximum_background_colour_ratio = max(.01, min(1., float(colour.get("maximum_background_colour_ratio", .35))))
        self.maximum_target_frame_ratio = max(.01, min(1., float(colour.get("maximum_target_frame_ratio", .25))))
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
        self._colour_channel = 0
        self._colour_reference_area = None
        self._colour_acquired = False
        self._roi_colour_tracking_active = False
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
            and self.colour_source in ("configured", "auto")
        )

    def reset(self) -> None:
        self._opencv_tracker = None
        self._template = None
        self._colour_histogram = None
        self._colour_channel = 0
        self._colour_reference_area = None
        self._colour_acquired = False
        self._roi_colour_tracking_active = False
        self._last_bbox = None
        self._active = False
        self._failures = 0

    def compensate_motion(self, previous_frame, matrix):
        """Move the previous template into the current camera plane, not onto
        the current target. The subsequent update remains an independent test.
        """
        if not self.active or self._last_bbox is None:
            return False
        from .camera_motion import CameraMotion, transform_box
        if not CameraMotion.validate(matrix, previous_frame.shape):
            return False
        box = transform_box(self._last_bbox, matrix)
        if box is None:
            return False
        x, y, width, height = box
        if x < 0 or y < 0 or x+width > previous_frame.shape[1] or y+height > previous_frame.shape[0]:
            return False
        if np.max(np.abs(np.asarray(box)-self._last_bbox)) < 3.:
            return True
        if self.tracker_type == "color_tracker":
            if self._colour_reference_area is not None:
                self._colour_reference_area *= box[2]*box[3]/max(1.,self._last_bbox[2]*self._last_bbox[3])
            self._last_bbox = box
            return True
        warped = cv2.warpPerspective(previous_frame, matrix,
            (previous_frame.shape[1], previous_frame.shape[0]))
        return self.initialize(warped, box).valid

    def configure_colour(self, colour_name: str = "") -> None:
        """Select global contrast/color tracking or ROI-learned tracking.

        ``auto`` searches for a shape-like color patch that contrasts with its
        local and global background. A named color searches for that color.
        Empty/``roi`` preserves the manually learned ROI tracker.
        """
        if self.tracker_type != "color_tracker":
            raise ValueError("colour configuration requires color_tracker")
        raw_name = str(colour_name or "").strip()
        if raw_name.lower() == "auto":
            next_source, next_name = "auto", "auto"
        elif raw_name.lower() == "roi" or not raw_name:
            next_source, next_name = "roi", ""
        else:
            next_source, next_name = "configured", self._normalize_colour_name(raw_name)
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
            if self.colour_source in ("configured", "auto"):
                # Global color modes do not use this supplied box; the normal
                # UI path never requests ROI initialization for them.
                return self._update_global_colour(frame)
            if not self._initialize_colour(frame, bbox):
                self.reset()
                return self._result(
                    bbox,
                    0.0,
                    False,
                    "initial_roi_has_no_matching_colour",
                )
            self._tracker_name = "color_roi_camshift"
            self._colour_reference_area = bbox[2]*bbox[3]
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
            and self.colour_source in ("configured", "auto")
        ):
            if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
                return self._configured_colour_failure("invalid_bgr_frame")
            if self._roi_colour_tracking_active and self._last_bbox is not None:
                bbox, confidence, valid, reason = self._update_colour(frame)
                if valid and bbox is not None:
                    height, width = frame.shape[:2]
                    bbox = _clamp_box(bbox, width, height)
                    valid, reason = self._validate_bbox(bbox, self._last_bbox, width, height)
                    if valid:
                        self._last_bbox = bbox
                        self._failures = 0
                        return self._result(bbox, confidence, True, reason)
                # CamShift can drift or lose a patch. Re-run the scene-contrast
                # detector in the local search window and relearn the ROI from
                # a newly validated candidate before declaring a miss.
                reacquired = self._update_global_colour(frame)
                if reacquired.valid:
                    return reacquired
                return reacquired
            return self._update_global_colour(frame)
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
        # Select the channel by foreground/background separation. A padded
        # gray/black vehicle ROI can contain plenty of saturated grass: that
        # must not make grass hue the learned target model.
        margin_x, margin_y = max(4, width//2), max(4, height//2)
        left, top = max(0, x-margin_x), max(0, y-margin_y)
        right = min(hsv.shape[1], x+width+margin_x)
        bottom = min(hsv.shape[0], y+height+margin_y)
        surrounding = hsv[top:bottom, left:right]
        ring = np.full(surrounding.shape[:2], 255, dtype=np.uint8)
        ring[y-top:y-top+height, x-left:x-left+width] = 0
        ring_pixels = cv2.countNonZero(ring)
        candidates = []
        for channel, channel_range in ((0, [0, 180]), (2, [0, 256])):
            if channel == 0:
                mask = cv2.inRange(crop,
                    (0, self.minimum_saturation, self.minimum_value), (180, 255, 255))
                background_mask = cv2.bitwise_and(ring, cv2.inRange(surrounding,
                    (0, self.minimum_saturation, self.minimum_value), (180, 255, 255)))
            else:
                mask = np.full(crop.shape[:2], 255, dtype=np.uint8)
                background_mask = ring
            if cv2.countNonZero(mask) < 4:
                continue
            histogram = cv2.calcHist([crop], [channel], mask,
                                    [self.histogram_bins], channel_range)
            background = cv2.calcHist([surrounding], [channel], background_mask,
                                     [self.histogram_bins], channel_range)
            # Normalize by all ROI/ring pixels, so a small grassy border
            # cannot outweigh a large achromatic vehicle body.
            histogram = histogram / float(crop.shape[0]*crop.shape[1])
            if ring_pixels and background is not None:
                histogram = np.maximum(0., histogram-background/float(ring_pixels))
            separation = float(histogram.sum())
            if separation > 1e-6:
                candidates.append((separation, channel, histogram))
        if not candidates:
            return False
        _, self._colour_channel, histogram = max(candidates, key=lambda item: item[0])
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
            [hsv], [self._colour_channel], self._colour_histogram,
            [0, 180] if self._colour_channel == 0 else [0, 256], 1
        )
        valid_pixels = (cv2.inRange(hsv,
            (0, self.minimum_saturation, self.minimum_value), (180, 255, 255))
            if self._colour_channel == 0 else np.full(hsv.shape[:2],255,dtype=np.uint8))
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

        x,y,width,height=self._last_bbox
        search_width,search_height=width*self.colour_search_expansion,height*self.colour_search_expansion
        left=max(0,int(x+width/2.-search_width/2.))
        top=max(0,int(y+height/2.-search_height/2.))
        right=min(frame.shape[1],int(x+width/2.+search_width/2.))
        bottom=min(frame.shape[0],int(y+height/2.+search_height/2.))
        bounded=np.zeros_like(backprojection)
        bounded[top:bottom,left:right]=backprojection[top:bottom,left:right]
        backprojection=bounded
        if cv2.countNonZero(backprojection)<4:
            return self._last_bbox,0.,False,'colour_target_not_found'

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
        # CamShift's covariance ellipse includes a growing margin on every
        # update. Refine it to an actual connected foreground component so
        # surrounding grass/road cannot become a large "successful" lock.
        binary = np.uint8(backprojection > 0)*255
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        previous = self._last_bbox
        old_center = (previous[0]+previous[2]/2., previous[1]+previous[3]/2.)
        diagonal = max(1., math.hypot(previous[2], previous[3]))
        candidates = []
        for contour in contours:
            x,y,width,height = cv2.boundingRect(contour)
            if cv2.contourArea(contour) < max(4., .03*previous[2]*previous[3]):
                continue
            bbox = (float(x),float(y),float(width),float(height))
            valid,_ = self._validate_bbox(bbox,previous,frame.shape[1],frame.shape[0])
            if not valid or (self._colour_reference_area is not None and
                    width*height > self._colour_reference_area*self.colour_max_reference_area_ratio):
                continue
            _,contrast = self._local_color_contrast(binary,(x,y,width,height))
            if contrast < self.minimum_local_colour_contrast:
                continue
            center = (x+width/2.,y+height/2.)
            distance = math.hypot(center[0]-old_center[0],center[1]-old_center[1])/diagonal
            area_ratio = width*height/max(1.,previous[2]*previous[3])
            continuity = math.exp(-distance-abs(math.log(max(.01,area_ratio))))
            quality = float(np.mean(backprojection[y:y+height,x:x+width])/255.)
            camshift_distance = math.hypot(center[0]-rotated[0][0],center[1]-rotated[0][1])/diagonal
            candidates.append((.55*continuity+.30*quality+.15*math.exp(-camshift_distance),bbox,quality))
        if not candidates:
            return self._last_bbox,0.,False,'no_continuous_roi_colour_component'
        _,bbox,confidence = max(candidates,key=lambda item:item[0])
        return bbox,confidence,True,'ok'

    def _color_mask(self, hsv: np.ndarray, colour_name: str) -> np.ndarray:
        if colour_name == "custom":
            ranges = self._custom_hsv_ranges
        else:
            ranges = self.PRESET_HSV_RANGES[colour_name]
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        return mask

    def _clean_colour_mask(self, mask: np.ndarray) -> np.ndarray:
        if self.morph_kernel > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _shape_quality(self, contour, bbox):
        x, y, width, height = bbox
        area = float(cv2.contourArea(contour))
        extent = area / max(1., float(width*height))
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = area / max(1., hull_area)
        aspect = width / max(1., float(height))
        minimum_aspect, maximum_aspect = {
            "vehicle": (.35, 3.2),
            "drone": (.55, 1.8),
            "compact": (.65, 1.55),
            "any": (.20, 5.0),
        }[self.target_shape]
        if (extent < self.minimum_shape_extent or solidity < self.minimum_shape_solidity
                or not minimum_aspect <= aspect <= maximum_aspect):
            return None
        aspect_center = math.sqrt(minimum_aspect*maximum_aspect)
        aspect_quality = math.exp(-abs(math.log(max(.01, aspect/aspect_center))))
        return .45*min(1., solidity) + .35*min(1., extent/.65) + .20*aspect_quality

    @staticmethod
    def _local_color_contrast(mask, bbox):
        x, y, width, height = bbox
        target = mask[y:y+height, x:x+width]
        if target.size == 0:
            return 0., 0.
        target_ratio = cv2.countNonZero(target) / float(target.size)
        margin_x, margin_y = max(4, width//2), max(4, height//2)
        left, top = max(0, x-margin_x), max(0, y-margin_y)
        right, bottom = min(mask.shape[1], x+width+margin_x), min(mask.shape[0], y+height+margin_y)
        ring = mask[top:bottom, left:right].copy()
        ring[y-top:y-top+height, x-left:x-left+width] = 0
        ring_ratio = cv2.countNonZero(ring) / float(max(1, ring.size-target.size))
        return target_ratio, max(0., target_ratio-ring_ratio)

    def _candidate_components(self, mask, frame_shape, *, local_search=False):
        frame_area = float(frame_shape[0]*frame_shape[1])
        full_ratio = cv2.countNonZero(mask)/max(1., frame_area)
        if not local_search and full_ratio > self.maximum_background_colour_ratio:
            return [], "selected_colour_dominates_background"
        components, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in components:
            area = float(cv2.contourArea(contour))
            if area < self.minimum_colour_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width < self.minimum_colour_width or height < self.minimum_colour_height:
                continue
            area_ratio = width*height/frame_area
            if area_ratio > self.maximum_target_frame_ratio:
                continue
            shape_quality = self._shape_quality(contour, (x, y, width, height))
            if shape_quality is None:
                continue
            purity, contrast = self._local_color_contrast(mask, (x, y, width, height))
            if contrast < self.minimum_local_colour_contrast:
                continue
            score = .48*shape_quality + .32*min(1., contrast*2.) + .20*min(1., purity)
            candidates.append((score, area, (float(x), float(y), float(width), float(height))))
        candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
        return candidates, "no_shape_like_contrasting_colour_region"

    def _update_global_colour(self, frame: np.ndarray) -> ManualTrackResult:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        names = tuple(self.PRESET_HSV_RANGES) if self.colour_source == "auto" else (self.colour_name,)
        if self.colour_source == 'auto' and self._colour_acquired and getattr(self,'_detected_colour_name',None):
            names = (self._detected_colour_name,)
        if self._colour_acquired and self._last_bbox is not None:
            x, y, width, height = self._last_bbox
            search_width = max(width, 8.)*self.colour_search_expansion
            search_height = max(height, 8.)*self.colour_search_expansion
            left = max(0, int(x+width/2.-search_width/2.))
            top = max(0, int(y+height/2.-search_height/2.))
            right = min(frame.shape[1], int(x+width/2.+search_width/2.))
            bottom = min(frame.shape[0], int(y+height/2.+search_height/2.))
            local_search = True
        else:
            left, top, right, bottom = 0, 0, frame.shape[1], frame.shape[0]
            local_search = False

        all_candidates = []
        reasons = []
        for name in names:
            whole_mask = self._clean_colour_mask(self._color_mask(hsv, name))
            bounded = np.zeros_like(whole_mask)
            bounded[top:bottom, left:right] = whole_mask[top:bottom, left:right]
            found, reason = self._candidate_components(bounded, frame.shape[:2], local_search=local_search)
            reasons.append(reason)
            all_candidates.extend((score, area, box, name) for score, area, box in found)

        if not all_candidates:
            reason = "selected_colour_dominates_background" if reasons and all(
                item == "selected_colour_dominates_background" for item in reasons) else "no_shape_like_contrasting_colour_region"
            self._tracker_name = ("color_auto_contrast" if self.colour_source == "auto"
                                  else "color_%s_contrast" % self.colour_name)
            if self._colour_acquired:
                return self._record_failure(self._last_bbox, reason)
            return self._configured_colour_failure(reason)

        if self._colour_acquired and self._last_bbox is not None:
            continuous = []
            rejected_reasons = []
            previous = self._last_bbox
            previous_center = (previous[0]+previous[2]/2., previous[1]+previous[3]/2.)
            diagonal = max(1., math.hypot(previous[2], previous[3]))
            for candidate in all_candidates:
                score, area, bbox, detected_colour = candidate
                valid, reason = self._validate_bbox(bbox, previous, frame.shape[1], frame.shape[0])
                if not valid:
                    rejected_reasons.append(reason)
                    continue
                current_center = (bbox[0]+bbox[2]/2., bbox[1]+bbox[3]/2.)
                distance = math.hypot(current_center[0]-previous_center[0],
                                      current_center[1]-previous_center[1])/diagonal
                area_ratio = (bbox[2]*bbox[3])/max(1., previous[2]*previous[3])
                continuity = math.exp(-distance) * math.exp(-abs(math.log(max(.01, area_ratio))))
                continuous.append((score+.20*continuity, area, bbox, detected_colour))
            if not continuous:
                reason = rejected_reasons[0] if rejected_reasons else "no_continuous_colour_candidate"
                return self._record_failure(self._last_bbox, "colour_"+reason)
            score, area, bbox, detected_colour = max(continuous, key=lambda item: (item[0], item[1]))
        else:
            score, area, bbox, detected_colour = max(all_candidates, key=lambda item: (item[0], item[1]))
        self._last_bbox = bbox
        self._active = True
        self._colour_acquired = True
        self._failures = 0
        self._detected_colour_name = detected_colour
        # Global contrast acquisition seeds the same ROI histogram used by
        # manual ColorTracker. Subsequent frames follow that learned patch;
        # if CamShift loses it, this method provides a bounded reacquisition.
        self._colour_histogram = None
        self._colour_reference_area = bbox[2]*bbox[3]
        self._roi_colour_tracking_active = self._initialize_colour(frame, bbox)
        if self._roi_colour_tracking_active:
            self._tracker_name = ("color_auto_roi_camshift" if self.colour_source == "auto"
                                  else "color_%s_roi_camshift" % self.colour_name)
        else:
            self._tracker_name = ("color_auto_contrast" if self.colour_source == "auto"
                                  else "color_%s_contrast" % self.colour_name)
        confidence = max(0., min(1., float(score)))
        return self._result(bbox, confidence, True, "ok")

    def _configured_colour_failure(self, reason: str) -> ManualTrackResult:
        self._failures += 1
        self._last_bbox = None
        self._active = False
        return self._result(None, 0.0, False, reason)

    def _configured_colour_mask(self, hsv: np.ndarray) -> np.ndarray:
        return self._color_mask(hsv, self.colour_name)

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
