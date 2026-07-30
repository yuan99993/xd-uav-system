"""
TrackerCore — 跟踪器核心模块。

Central tracking coordinator that integrates:
- External input handling (bounding box, feature point, ROI)
- Kalman filter state estimation
- Motion prediction (EMA short-term + B-spline long-term)
- Tracking state management (hybrid ID/spatial/distance matching)
- Coordinate transformations
- Normalized error computation for downstream Follower
- BSpline trajectory prediction (inspired by ego-planner-swarm)

This is the main algorithm module extracted from PixEagle's SmartTracker,
adapted for ROS-based operation with PX4-Autopilot and Gazebo.

Algorithm optimizations from ego-planner-swarm:
- BSplinePredictor: B-spline trajectory parameterization with feasibility constraints
"""

import time
import logging
from typing import Optional, Tuple, Dict, Any, List
from dataclasses import dataclass, field

from tracker.detection import NormalizedDetection
from tracker.kalman_box_tracker import KalmanBoxTracker
from tracker.motion_predictor import MotionPredictor
from tracker.bspline_predictor import BSplinePredictor
from tracker.tracking_state_manager import TrackingStateManager
from tracker.coordinate_transformer import CoordinateTransformer, CameraParameters
from tracker.external_input_handler import ExternalInputHandler, UnifiedInput, InputSource
from tracker.geometry_utils import compute_iou

logger = logging.getLogger(__name__)


@dataclass
class TrackerResult:
    """
    Unified tracker output result.

    Contains all information needed by downstream consumers (Follower, logging, visualization).
    """
    timestamp: float = 0.0
    tracking_active: bool = False
    target_id: Optional[int] = None
    class_id: Optional[int] = None
    confidence: float = 0.0

    # Bounding box (pixel)
    bbox: Optional[Tuple[int, int, int, int]] = None
    bbox_normalized: Optional[Tuple[float, float, float, float]] = None

    # Center position
    center_pixel: Optional[Tuple[float, float]] = None
    center_normalized: Optional[Tuple[float, float]] = None

    # Normalized error (for Follower) [-1, 1]
    error_x: float = 0.0
    error_y: float = 0.0
    error_size: float = 0.0

    # Motion
    velocity: Optional[Tuple[float, float]] = None
    acceleration: Optional[Tuple[float, float]] = None

    # Angular error (for gimbal/PX4)
    yaw_error_deg: float = 0.0
    pitch_error_deg: float = 0.0

    # State flags
    is_predicted: bool = False
    frames_since_detection: int = 0
    tracking_quality: float = 0.0

    # BSpline prediction (ego-planner inspired)
    bspline_trajectory: Optional[List[Tuple[float, float]]] = None  # 预测轨迹采样点
    predicted_velocity: Optional[Tuple[float, float]] = None        # B-spline 预测速度

    # Metadata
    geometry_type: str = "aabb"
    tracker_name: str = "TrackerCore"
    tracker_type: str = "hybrid_kalman_bspline"


class TrackerCore:
    """
    Main tracker coordinator — extracts and refines PixEagle's tracking algorithms.

    Features:
    - Accepts external inputs (bounding box, feature point, ROI)
    - Runs Kalman filter for state estimation
    - Motion prediction with EMA smoothing during occlusion
    - Hybrid ID/spatial/distance matching
    - Outputs normalized errors for Follower
    - Coordinate transformations (pixel ↔ normalized ↔ angular)
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize TrackerCore.

        Args:
            config: Configuration dictionary with these keys:
                - frame_width, frame_height: Camera frame dimensions
                - fov_horizontal, fov_vertical: Camera FOV in degrees
                - enable_kalman_filter: Enable Kalman filter (default True)
                - enable_motion_predictor: Enable motion prediction (default True)
                - enable_prediction_buffer: Enable prediction during occlusion (default True)
                - id_loss_tolerance_frames: Max frames without detection (default 5)
                - spatial_iou_threshold: IoU threshold for spatial matching (default 0.35)
                - tracking_strategy: "hybrid" (default)
                - target_size_ratio: Desired target/frame ratio (default 0.15)
                - kalman_process_noise: Kalman process noise scale (default 1.0)
                - kalman_measurement_noise: Kalman measurement noise scale (default 1.0)
        """
        self.config = config or {}

        # Frame parameters
        self.frame_width = self.config.get('frame_width', 640)
        self.frame_height = self.config.get('frame_height', 480)

        # Camera parameters
        camera_params = CameraParameters(
            fov_horizontal=self.config.get('fov_horizontal', 60.0),
            fov_vertical=self.config.get('fov_vertical', 45.0),
            mount_offset_yaw=self.config.get('mount_offset_yaw', 0.0),
            mount_offset_pitch=self.config.get('mount_offset_pitch', 0.0),
            mount_offset_roll=self.config.get('mount_offset_roll', 0.0),
        )

        # Subsystems
        self.coord_transformer = CoordinateTransformer(camera_params)
        self.input_handler = ExternalInputHandler(self.frame_width, self.frame_height)

        # Motion predictor (EMA, short-term)
        self.enable_motion_predictor = self.config.get('enable_motion_predictor', True)
        self.motion_predictor = MotionPredictor(
            history_size=self.config.get('id_loss_tolerance_frames', 5),
            velocity_alpha=self.config.get('velocity_alpha', 0.7),
            acceleration_alpha=self.config.get('acceleration_alpha', 0.5),
        ) if self.enable_motion_predictor else None

        # BSpline predictor (ego-planner inspired, long-term trajectory)
        self.enable_bspline_predictor = self.config.get('enable_bspline_predictor', True)
        self.bspline_predictor = BSplinePredictor(
            order=self.config.get('bspline_order', 3),
            num_control_points=self.config.get('bspline_num_ctrl_pts', 8),
            history_size=self.config.get('bspline_history_size', 15),
            max_vel=self.config.get('bspline_max_vel', 500.0),
            max_acc=self.config.get('bspline_max_acc', 200.0),
            dt_estimate=1.0 / max(self.config.get('publish_rate', 30.0), 1.0),
            prediction_horizon=self.config.get('bspline_horizon', 1.0),
        ) if self.enable_bspline_predictor else None

        # Tracking state manager
        self.tracking_manager = TrackingStateManager(
            config=self.config,
            motion_predictor=self.motion_predictor,
        )

        # Target size ratio for size error computation
        self.target_size_ratio = self.config.get('target_size_ratio', 0.15)

        # Statistics
        self._frame_count = 0
        self._tracking_start_time: Optional[float] = None

        logger.info(
            "[TrackerCore] Initialized: frame=%dx%d, fov=%.1f°x%.1f°, "
            "kalman=%s, predictor=%s, strategy=%s",
            self.frame_width, self.frame_height,
            camera_params.fov_horizontal, camera_params.fov_vertical,
            self.config.get('enable_kalman_filter', True),
            self.enable_motion_predictor,
            self.config.get('tracking_strategy', 'hybrid'),
        )

    def set_frame_size(self, width: int, height: int):
        """Update frame dimensions."""
        self.frame_width = width
        self.frame_height = height
        self.input_handler.set_frame_size(width, height)

    # ── External Input Interface ─────────────────────────────────────────

    def process_external_bbox(
        self,
        bbox_pixel: Optional[Tuple[int, int, int, int]] = None,
        normalized_bbox: Optional[Tuple[float, float, float, float]] = None,
        confidence: float = 1.0,
        class_id: Optional[int] = None,
        command: str = "",
    ) -> UnifiedInput:
        """
        Process an external bounding box input.

        This is the primary interface for external detectors or manual selection.

        Args:
            bbox_pixel: (x1, y1, x2, y2) in pixels
            normalized_bbox: (cx, cy, w, h) in [0, 1]
            confidence: Detection confidence [0, 1]
            class_id: Object class ID
            command: "start_track", "stop_track", "reset", or ""

        Returns:
            UnifiedInput
        """
        inp = self.input_handler.process_bounding_box(
            bbox_pixel=bbox_pixel,
            normalized_bbox=normalized_bbox,
            confidence=confidence,
            class_id=class_id,
            command=command,
            timestamp=time.time(),
        )

        # Handle commands
        if command == "start_track" and inp.bbox_pixel is not None:
            track_id = class_id if class_id is not None else 0
            self.tracking_manager.start_tracking(
                bbox=inp.bbox_pixel,
                track_id=track_id,
                class_id=class_id or 0,
                confidence=confidence,
            )
            self._tracking_start_time = time.time()

        elif command == "stop_track":
            self.tracking_manager.stop_tracking()
            self._tracking_start_time = None

        elif command == "reset":
            self.reset()

        return inp

    def process_external_feature(
        self,
        point: Tuple[float, float],
        velocity: Optional[Tuple[float, float]] = None,
        confidence: float = 1.0,
        class_id: Optional[int] = None,
        command: str = "",
    ) -> UnifiedInput:
        """
        Process an external feature point input.

        Args:
            point: (x, y) normalized [0, 1]
            velocity: (vx, vy) normalized velocity
            confidence: Detection confidence
            class_id: Object class ID
            command: "start_track", "stop_track", "reset", or ""

        Returns:
            UnifiedInput
        """
        inp = self.input_handler.process_feature_point(
            point=point,
            velocity=velocity,
            confidence=confidence,
            class_id=class_id,
            command=command,
            timestamp=time.time(),
        )

        if command == "start_track" and inp.bbox_pixel is not None:
            track_id = class_id if class_id is not None else 0
            self.tracking_manager.start_tracking(
                bbox=inp.bbox_pixel,
                track_id=track_id,
                class_id=class_id or 0,
                confidence=confidence,
            )
            self._tracking_start_time = time.time()

        elif command == "stop_track":
            self.tracking_manager.stop_tracking()
            self._tracking_start_time = None

        elif command == "reset":
            self.reset()

        return inp

    def process_external_roi(
        self,
        roi: Tuple[float, float, float, float],
        confidence: float = 1.0,
        class_id: Optional[int] = None,
        command: str = "",
    ) -> UnifiedInput:
        """
        Process an external ROI input.

        Args:
            roi: (x, y, w, h) normalized [0, 1]
            confidence: Detection confidence
            class_id: Object class ID
            command: "start_track", "stop_track", "reset", or ""

        Returns:
            UnifiedInput
        """
        inp = self.input_handler.process_roi(
            roi=roi,
            confidence=confidence,
            class_id=class_id,
            command=command,
            timestamp=time.time(),
        )

        if command == "start_track" and inp.bbox_pixel is not None:
            track_id = class_id if class_id is not None else 0
            self.tracking_manager.start_tracking(
                bbox=inp.bbox_pixel,
                track_id=track_id,
                class_id=class_id or 0,
                confidence=confidence,
            )
            self._tracking_start_time = time.time()

        elif command == "stop_track":
            self.tracking_manager.stop_tracking()
            self._tracking_start_time = None

        elif command == "reset":
            self.reset()

        return inp

    # ── Internal Detection Interface ─────────────────────────────────────

    def update_with_detections(
        self, detections: list,
        timestamp: Optional[float] = None,
    ) -> TrackerResult:
        """
        Update tracker with detection results (YOLO or similar).

        Each detection: [x1, y1, x2, y2, track_id, conf, class_id, is_stable]

        Args:
            detections: List of detection rows
            timestamp: Current timestamp

        Returns:
            TrackerResult with normalized errors
        """
        if timestamp is None:
            timestamp = time.time()

        self._frame_count += 1

        # Update tracking state manager
        ts_result = self.tracking_manager.update(detections, timestamp)

        # Build result
        result = TrackerResult(timestamp=timestamp)

        if ts_result['tracking_active'] and ts_result['bbox'] is not None:
            bbox = ts_result['bbox']
            center = ts_result.get('center')
            if center is None:
                center = (
                    (bbox[0] + bbox[2]) / 2.0,
                    (bbox[1] + bbox[3]) / 2.0,
                )

            result.tracking_active = True
            result.bbox = bbox
            result.center_pixel = center
            result.confidence = ts_result.get('confidence', 0.0)
            result.is_predicted = ts_result.get('is_predicted', False)
            result.frames_since_detection = ts_result.get('frames_since_detection', 0)
            result.target_id = self.tracking_manager.selected_track_id
            result.class_id = self.tracking_manager.selected_class_id

            # Compute normalized bbox
            x1, y1, x2, y2 = bbox
            w_px = x2 - x1
            h_px = y2 - y1
            result.bbox_normalized = (
                (x1 + w_px / 2.0) / self.frame_width,
                (y1 + h_px / 2.0) / self.frame_height,
                w_px / self.frame_width,
                h_px / self.frame_height,
            )
            result.center_normalized = (
                center[0] / self.frame_width,
                center[1] / self.frame_height,
            )

            # Compute normalized errors
            result.error_x, result.error_y = self.coord_transformer.compute_normalized_error(
                center[0], center[1],
                self.frame_width, self.frame_height,
            )

            # Compute size error
            result.error_size = self.coord_transformer.compute_size_error(
                w_px * h_px,
                self.frame_width * self.frame_height,
                self.target_size_ratio,
            )

            # Compute angular errors
            result.yaw_error_deg, result.pitch_error_deg = \
                self.coord_transformer.pixel_to_angle_error(
                    center[0], center[1],
                    self.frame_width, self.frame_height,
                )

            # Get velocity
            if self.motion_predictor is not None:
                result.velocity = self.motion_predictor.get_velocity()
                result.acceleration = self.motion_predictor.get_acceleration()

            # Compute tracking quality
            result.tracking_quality = self._compute_quality(result)

            # Update motion predictor
            if self.motion_predictor is not None and not result.is_predicted:
                self.motion_predictor.update(bbox, timestamp)

            # Update BSpline predictor (ego-planner inspired)
            if self.bspline_predictor is not None and not result.is_predicted:
                self.bspline_predictor.update(center, timestamp)
                # Get B-spline predicted trajectory
                result.bspline_trajectory = self.bspline_predictor.get_prediction_trajectory(
                    num_steps=10,
                )
                # Get B-spline predicted velocity at current time
                result.predicted_velocity = self.bspline_predictor.predict_velocity(
                    1.0 / max(self.config.get('publish_rate', 30.0), 1.0),
                )

        return result

    def update_empty(self, timestamp: Optional[float] = None) -> TrackerResult:
        """
        Update tracker without new detections (relies on prediction).

        Args:
            timestamp: Current timestamp

        Returns:
            TrackerResult (may have predicted position)
        """
        return self.update_with_detections([], timestamp)

    def _compute_quality(self, result: TrackerResult) -> float:
        """
        Compute tracking quality metric [0, 1].

        Factors:
        - Detection confidence
        - Frames since last detection (decay)
        - Whether position is predicted
        """
        if not result.tracking_active:
            return 0.0

        quality = result.confidence

        # Penalize prediction
        if result.is_predicted:
            decay = max(0.0, 1.0 - result.frames_since_detection / 10.0)
            quality *= decay * 0.8

        # Penalize edge positions (target near frame edge)
        if result.center_normalized is not None:
            cx, cy = result.center_normalized
            edge_penalty = 1.0 - max(
                0.0,
                min(abs(cx - 0.5) * 2.0 - 0.7, 0.0),
                min(abs(cy - 0.5) * 2.0 - 0.7, 0.0),
            ) / 0.3
            quality *= max(0.5, edge_penalty)

        return max(0.0, min(1.0, quality))

    # ── Result Access ────────────────────────────────────────────────────

    def get_result(self) -> TrackerResult:
        """Get the latest tracking result (does not update)."""
        state = self.tracking_manager.get_current_state()

        result = TrackerResult(timestamp=time.time())
        result.tracking_active = state['tracking_active']
        result.target_id = state['track_id']
        result.class_id = state['class_id']
        result.confidence = state['confidence']
        result.bbox = state['bbox']
        result.frames_since_detection = state['frames_since_detection']

        if state['center'] is not None:
            result.center_pixel = (
                float(state['center'][0]),
                float(state['center'][1]),
            )
            result.center_normalized = (
                state['center'][0] / self.frame_width,
                state['center'][1] / self.frame_height,
            )
            result.error_x, result.error_y = \
                self.coord_transformer.compute_normalized_error(
                    state['center'][0], state['center'][1],
                    self.frame_width, self.frame_height,
                )

        if state['bbox'] is not None:
            x1, y1, x2, y2 = state['bbox']
            w_px = x2 - x1
            h_px = y2 - y1
            result.bbox_normalized = (
                (x1 + w_px / 2.0) / self.frame_width,
                (y1 + h_px / 2.0) / self.frame_height,
                w_px / self.frame_width,
                h_px / self.frame_height,
            )

        return result

    def is_tracking(self) -> bool:
        """Check if the tracker is actively tracking."""
        return self.tracking_manager.is_tracking()

    def reset(self):
        """Reset all tracking state."""
        self.tracking_manager.stop_tracking()
        if self.motion_predictor is not None:
            self.motion_predictor.reset()
        if self.bspline_predictor is not None:
            self.bspline_predictor.reset()
        self.input_handler.reset()
        self._tracking_start_time = None
        self._frame_count = 0
        logger.info("[TrackerCore] Reset complete.")

    def get_stats(self) -> Dict[str, Any]:
        """Get tracker statistics."""
        return {
            'frame_count': self._frame_count,
            'tracking_active': self.tracking_manager.is_tracking(),
            'track_id': self.tracking_manager.selected_track_id,
            'frames_since_detection': self.tracking_manager.frames_since_detection,
            'tracking_duration_s': (
                time.time() - self._tracking_start_time
                if self._tracking_start_time is not None else 0.0
            ),
        }
