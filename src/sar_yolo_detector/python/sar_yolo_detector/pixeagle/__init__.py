"""Portable PixEagle perception, tracking, geometry, and command primitives."""

from .command_intent import CommandIntent
from .detection_adapter import NormalizedDetection, to_tracking_state_rows
from .geometry_utils import (
    clip_aabb_to_frame,
    obb_xywhr_to_aabb,
    obb_xywhr_to_polygon,
    point_in_polygon,
    polygon_to_aabb,
    validate_obb_xywhr,
)
from .kalman_box_tracker import KalmanBoxTracker
from .motion_predictor import MotionPredictor
from .smart_tracker import SmartTracker
from .tracker_output import TrackerDataType, TrackerOutput
from .tracking_roi import (
    TrackingROIError,
    tracking_point_to_pixels,
    tracking_roi_to_pixels,
    tracking_xyxy_to_pixels,
)
from .yaw_rate_smoother import YawRateSmoother

__all__ = [
    "CommandIntent",
    "KalmanBoxTracker",
    "MotionPredictor",
    "NormalizedDetection",
    "SmartTracker",
    "TrackerDataType",
    "TrackerOutput",
    "TrackingROIError",
    "YawRateSmoother",
    "clip_aabb_to_frame",
    "obb_xywhr_to_aabb",
    "obb_xywhr_to_polygon",
    "point_in_polygon",
    "polygon_to_aabb",
    "to_tracking_state_rows",
    "tracking_point_to_pixels",
    "tracking_roi_to_pixels",
    "tracking_xyxy_to_pixels",
    "validate_obb_xywhr",
]
