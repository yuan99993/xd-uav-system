"""
External Input Handler — 外部输入处理器。

Handles incoming external detection/feature inputs from various sources:
- Bounding box inputs (from external detectors)
- Feature point inputs (from feature matchers)
- ROI inputs (from user selection or external systems)

Converts all input formats to a unified internal representation
consumed by the TrackerCore.
"""

import logging
import math
from dataclasses import dataclass
from typing import Optional, Tuple, List
from enum import Enum

logger = logging.getLogger(__name__)


class InputSource(Enum):
    """External input source types."""
    BOUNDING_BOX = "bounding_box"
    FEATURE_POINT = "feature_point"
    ROI = "roi"
    EXTERNAL_DETECTOR = "external_detector"
    NONE = "none"


@dataclass
class UnifiedInput:
    """
    Unified internal representation of external inputs.

    All coordinates are normalized [0, 1] unless specified otherwise.
    """
    source: InputSource = InputSource.NONE
    timestamp: float = 0.0

    # Bounding box (normalized)
    normalized_bbox: Optional[Tuple[float, float, float, float]] = None  # (cx, cy, w, h)
    has_bbox: bool = False

    # Bounding box (pixel)
    bbox_pixel: Optional[Tuple[int, int, int, int]] = None  # (x1, y1, x2, y2)

    # Feature point (normalized)
    feature_point: Optional[Tuple[float, float]] = None  # (x, y)
    has_feature: bool = False

    # Feature velocity (normalized)
    feature_velocity: Optional[Tuple[float, float]] = None  # (vx, vy)

    # ROI (normalized)
    roi: Optional[Tuple[float, float, float, float]] = None  # (x, y, w, h)
    has_roi: bool = False

    # Metadata
    confidence: float = 1.0
    class_id: Optional[int] = None
    command: str = ""  # "start_track", "stop_track", "reset", ""


class ExternalInputHandler:
    """
    Handles and normalizes external inputs from various sources.

    Converts bounding boxes, feature points, and ROIs into a unified format
    that the TrackerCore can consume directly.
    """

    def __init__(self, frame_width: int = 640, frame_height: int = 480):
        """
        Initialize the external input handler.

        Args:
            frame_width: Default frame width for pixel conversions
            frame_height: Default frame height for pixel conversions
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self._last_input: Optional[UnifiedInput] = None
        logger.info(
            "[ExternalInputHandler] Initialized: frame=%dx%d",
            frame_width, frame_height,
        )

    def set_frame_size(self, width: int, height: int):
        """Update frame dimensions."""
        self.frame_width = width
        self.frame_height = height

    def process_bounding_box(
        self, bbox_pixel: Optional[Tuple[int, int, int, int]] = None,
        normalized_bbox: Optional[Tuple[float, float, float, float]] = None,
        confidence: float = 1.0,
        class_id: Optional[int] = None,
        command: str = "",
        timestamp: float = 0.0,
    ) -> UnifiedInput:
        """
        Process a bounding box input.

        Args:
            bbox_pixel: (x1, y1, x2, y2) in pixels
            normalized_bbox: (cx, cy, w, h) in [0, 1]
            confidence: Detection confidence
            class_id: Optional class ID
            command: Optional command string
            timestamp: Timestamp

        Returns:
            UnifiedInput with normalized bounding box
        """
        inp = UnifiedInput(
            source=InputSource.BOUNDING_BOX,
            timestamp=timestamp,
            confidence=confidence,
            class_id=class_id,
            command=command,
        )

        if bbox_pixel is not None:
            inp.bbox_pixel = self._sanitize_pixel_bbox(bbox_pixel)
            # Convert to normalized (cx, cy, w, h)
            x1, y1, x2, y2 = inp.bbox_pixel
            w_px = x2 - x1
            h_px = y2 - y1
            inp.normalized_bbox = (
                (x1 + w_px / 2.0) / self.frame_width,
                (y1 + h_px / 2.0) / self.frame_height,
                w_px / self.frame_width,
                h_px / self.frame_height,
            )
            inp.has_bbox = True
        elif normalized_bbox is not None:
            inp.normalized_bbox = self._sanitize_normalized_bbox(normalized_bbox)
            # Convert to pixel
            cx, cy, w, h = inp.normalized_bbox
            w_px = int(w * self.frame_width)
            h_px = int(h * self.frame_height)
            x1 = int((cx - w / 2.0) * self.frame_width)
            y1 = int((cy - h / 2.0) * self.frame_height)
            inp.bbox_pixel = (x1, y1, x1 + w_px, y1 + h_px)
            inp.has_bbox = True

        self._last_input = inp
        return inp

    def process_feature_point(
        self,
        point: Tuple[float, float],
        velocity: Optional[Tuple[float, float]] = None,
        confidence: float = 1.0,
        class_id: Optional[int] = None,
        command: str = "",
        timestamp: float = 0.0,
    ) -> UnifiedInput:
        """
        Process a feature point input.

        Args:
            point: (x, y) normalized coordinates [0, 1]
            velocity: (vx, vy) normalized velocity
            confidence: Detection confidence
            class_id: Optional class ID
            command: Optional command string
            timestamp: Timestamp

        Returns:
            UnifiedInput with feature point
        """
        point = self._sanitize_normalized_point(point)
        inp = UnifiedInput(
            source=InputSource.FEATURE_POINT,
            timestamp=timestamp,
            confidence=confidence,
            class_id=class_id,
            command=command,
            feature_point=point,
            feature_velocity=velocity,
            has_feature=True,
        )

        # Create a synthetic bounding box around the feature point
        box_size = 0.05  # 5% of frame
        cx, cy = point
        inp.normalized_bbox = (cx, cy, box_size, box_size)
        inp.has_bbox = True

        w_px = int(box_size * self.frame_width)
        h_px = int(box_size * self.frame_height)
        px = int(cx * self.frame_width)
        py = int(cy * self.frame_height)
        inp.bbox_pixel = (
            px - w_px // 2,
            py - h_px // 2,
            px + w_px // 2,
            py + h_px // 2,
        )

        self._last_input = inp
        return inp

    def process_roi(
        self,
        roi: Tuple[float, float, float, float],
        confidence: float = 1.0,
        class_id: Optional[int] = None,
        command: str = "",
        timestamp: float = 0.0,
    ) -> UnifiedInput:
        """
        Process an ROI input.

        Args:
            roi: (x, y, w, h) normalized [0, 1]
            confidence: Detection confidence
            class_id: Optional class ID
            command: Optional command string
            timestamp: Timestamp

        Returns:
            UnifiedInput with ROI
        """
        x, y, w, h = self._sanitize_normalized_roi(roi)
        inp = UnifiedInput(
            source=InputSource.ROI,
            timestamp=timestamp,
            confidence=confidence,
            class_id=class_id,
            command=command,
            roi=roi,
            has_roi=True,
            normalized_bbox=(x + w / 2.0, y + h / 2.0, w, h),
            has_bbox=True,
        )

        w_px = int(w * self.frame_width)
        h_px = int(h * self.frame_height)
        x1 = int(x * self.frame_width)
        y1 = int(y * self.frame_height)
        inp.bbox_pixel = (x1, y1, x1 + w_px, y1 + h_px)

        self._last_input = inp
        return inp

    def get_last_input(self) -> Optional[UnifiedInput]:
        """Get the last processed input."""
        return self._last_input

    def reset(self):
        """Reset the handler state."""
        self._last_input = None

    def _sanitize_pixel_bbox(
        self, bbox: Tuple[int, int, int, int],
    ) -> Tuple[int, int, int, int]:
        if len(bbox) != 4:
            raise ValueError("bounding box must contain four coordinates")
        x1, y1, x2, y2 = (int(value) for value in bbox)
        x1 = max(0, min(self.frame_width, x1))
        x2 = max(0, min(self.frame_width, x2))
        y1 = max(0, min(self.frame_height, y1))
        y2 = max(0, min(self.frame_height, y2))
        if x2 <= x1 or y2 <= y1:
            raise ValueError("bounding box must have positive width and height")
        return x1, y1, x2, y2

    @staticmethod
    def _sanitize_normalized_point(
        point: Tuple[float, float],
    ) -> Tuple[float, float]:
        if len(point) != 2 or not all(math.isfinite(value) for value in point):
            raise ValueError("feature point must contain two finite values")
        return tuple(max(0.0, min(1.0, float(value))) for value in point)

    def _sanitize_normalized_bbox(
        self, bbox: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
            raise ValueError("normalized bounding box must contain four finite values")
        cx, cy, width, height = (float(value) for value in bbox)
        if width <= 0.0 or height <= 0.0:
            raise ValueError("normalized bounding box must have positive size")
        width = min(width, 1.0)
        height = min(height, 1.0)
        cx = max(width / 2.0, min(1.0 - width / 2.0, cx))
        cy = max(height / 2.0, min(1.0 - height / 2.0, cy))
        return cx, cy, width, height

    def _sanitize_normalized_roi(
        self, roi: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        if len(roi) != 4 or not all(math.isfinite(value) for value in roi):
            raise ValueError("ROI must contain four finite values")
        x, y, width, height = (float(value) for value in roi)
        if width <= 0.0 or height <= 0.0:
            raise ValueError("ROI must have positive size")
        x = max(0.0, min(1.0, x))
        y = max(0.0, min(1.0, y))
        width = min(width, 1.0 - x)
        height = min(height, 1.0 - y)
        if width <= 0.0 or height <= 0.0:
            raise ValueError("ROI must overlap the image")
        return x, y, width, height
