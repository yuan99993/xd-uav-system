"""
Tracking State Manager for robust single-object tracking.

Ported from PixEagle src/classes/tracking_state_manager.py

Provides intelligent track-ID management with hybrid matching:
  1. ID match         — exact track_id
  2. Spatial match    — IoU against Kalman-predicted position
  3. Distance match   — center distance when IoU is zero
  4. Prediction       — Kalman filter during occlusion

Author: PixEagle Team / Tracker Team
"""

import logging
import math
from collections import deque
from typing import Optional, Tuple, List, Dict

from tracker.kalman_box_tracker import KalmanBoxTracker
from tracker.geometry_utils import compute_iou, compute_center_distance

logger = logging.getLogger(__name__)


class TrackingStateManager:
    """
    Manages tracking state with hybrid ID + spatial + distance matching.

    Provides robust single-target tracking with:
    1. Primary: Track by ID
    2. Fallback 1: Track by spatial proximity (IoU)
    3. Fallback 2: Track by center distance
    4. Prediction: Kalman filter during occlusion
    """

    def __init__(self, config: Optional[dict] = None, motion_predictor=None):
        """
        Initialize the tracking state manager.

        Args:
            config: Dictionary of configuration parameters
            motion_predictor: Optional MotionPredictor for legacy fallback
        """
        self.config = config or {}
        self.motion_predictor = motion_predictor

        # Core tracking state
        self.selected_track_id: Optional[int] = None
        self.selected_track_id_is_stable: bool = True
        self.selected_class_id: Optional[int] = None
        self.last_known_bbox: Optional[Tuple[int, int, int, int]] = None
        self.last_known_center: Optional[Tuple[int, int]] = None
        self.last_detection_time: float = 0.0

        # Kalman filter
        self.kalman: Optional[KalmanBoxTracker] = None
        self.enable_kalman = self.config.get('enable_kalman_filter', True)
        self.kalman_process_noise = self.config.get('kalman_process_noise', 1.0)
        self.kalman_measurement_noise = self.config.get('kalman_measurement_noise', 1.0)

        # Tracking history
        self.max_history = self.config.get('id_loss_tolerance_frames', 5)
        self.tracking_history = deque(maxlen=max(self.max_history, 10))

        # Strategy configuration
        self.tracking_strategy = self.config.get('tracking_strategy', 'hybrid')
        self.spatial_iou_threshold = self.config.get('spatial_iou_threshold', 0.35)
        self.enable_prediction = self.config.get('enable_prediction_buffer', True)

        # Center-distance matching
        self.enable_distance_matching = self.config.get('enable_center_distance_matching', True)
        self.center_distance_threshold = self.config.get('center_distance_threshold', 2.0)

        # Class flexibility
        self.class_match_flexible = self.config.get('class_match_flexible', True)
        self.class_history_size = self.config.get('class_history_size', 10)
        self.class_history: deque = deque(maxlen=self.class_history_size)

        # Confidence decay
        self.confidence_alpha = self.config.get('confidence_smoothing_alpha', 0.8)
        self.smoothed_confidence = 0.0
        self.confidence_decay_rate = self.config.get('track_confidence_decay_rate', 0.05)

        # Frame counters
        self.frames_since_detection = 0
        self.total_frames = 0

        self._confidence_at_loss: float = 0.0
        self._last_seen_bbox: Optional[Tuple[int, int, int, int]] = None

        logger.info(
            "[TrackingStateManager] Initialized: strategy='%s', tolerance=%d, IoU=%.2f",
            self.tracking_strategy, self.max_history, self.spatial_iou_threshold,
        )

    def start_tracking(self, bbox: Tuple[int, int, int, int],
                       track_id: int, class_id: int,
                       confidence: float = 1.0,
                       track_id_is_stable: bool = True):
        """
        Initialize tracking on a new target.

        Args:
            bbox: Initial bounding box (x1, y1, x2, y2)
            track_id: Detection track ID
            class_id: Object class ID
            confidence: Detection confidence
            track_id_is_stable: Whether the track_id persists across frames
        """
        self.selected_track_id = track_id
        self.selected_track_id_is_stable = track_id_is_stable
        self.selected_class_id = class_id
        self.last_known_bbox = bbox
        self.last_known_center = (
            (bbox[0] + bbox[2]) // 2,
            (bbox[1] + bbox[3]) // 2,
        )

        if self.enable_kalman:
            self.kalman = KalmanBoxTracker(
                bbox,
                process_noise_scale=self.kalman_process_noise,
                measurement_noise_scale=self.kalman_measurement_noise,
            )

        self.smoothed_confidence = confidence
        self.frames_since_detection = 0

        if class_id is not None:
            self.class_history.append(class_id)

        logger.info(
            "[TrackingStateManager] Started tracking: track_id=%d, class=%d, bbox=%s",
            track_id, class_id, bbox,
        )

    def stop_tracking(self):
        """Stop current tracking session."""
        self.selected_track_id = None
        self.selected_class_id = None
        self.last_known_bbox = None
        self.last_known_center = None
        self.kalman = None
        self.smoothed_confidence = 0.0
        self.frames_since_detection = 0
        self.class_history.clear()
        logger.info("[TrackingStateManager] Tracking stopped.")

    def update(self, detections: List[List[float]],
               timestamp: float = 0.0) -> Dict:
        """
        Update tracking state with new detections.

        Each detection row: [x1, y1, x2, y2, track_id, conf, class_id, is_stable]

        Args:
            detections: List of detection rows
            timestamp: Current timestamp

        Returns:
            Dict with tracking result
        """
        self.total_frames += 1
        result = {
            'tracking_active': False,
            'bbox': None,
            'center': None,
            'confidence': 0.0,
            'is_predicted': False,
            'frames_since_detection': self.frames_since_detection,
        }

        if self.selected_track_id is None:
            return result

        matched_detection = None

        # 1. Try ID match
        if self.selected_track_id_is_stable:
            for det in detections:
                det_track_id = int(det[4])
                if det_track_id == self.selected_track_id:
                    matched_detection = det
                    break

        # 2. Try spatial IoU match
        if matched_detection is None and self.last_known_bbox is not None:
            best_iou = 0.0
            best_det = None
            for det in detections:
                det_bbox = (int(det[0]), int(det[1]), int(det[2]), int(det[3]))
                iou = compute_iou(self.last_known_bbox, det_bbox)
                if iou > best_iou:
                    best_iou = iou
                    best_det = det
            if best_iou >= self.spatial_iou_threshold:
                matched_detection = best_det

        # 3. Try center distance match
        if matched_detection is None and self.last_known_center is not None \
                and self.enable_distance_matching:
            best_dist = float('inf')
            best_det = None
            for det in detections:
                det_bbox = (int(det[0]), int(det[1]), int(det[2]), int(det[3]))
                cx = (det_bbox[0] + det_bbox[2]) / 2.0
                cy = (det_bbox[1] + det_bbox[3]) / 2.0
                dx = cx - self.last_known_center[0]
                dy = cy - self.last_known_center[1]
                dist = math.sqrt(dx * dx + dy * dy)
                diag = math.sqrt(
                    (det_bbox[2] - det_bbox[0]) ** 2 +
                    (det_bbox[3] - det_bbox[1]) ** 2
                )
                norm_dist = dist / max(diag, 1.0)
                if norm_dist < best_dist:
                    best_dist = norm_dist
                    best_det = det
            if best_dist <= self.center_distance_threshold:
                matched_detection = best_det

        if matched_detection is not None:
            # Update with detection
            bbox = (int(matched_detection[0]), int(matched_detection[1]),
                    int(matched_detection[2]), int(matched_detection[3]))
            conf = float(matched_detection[5])
            class_id = int(matched_detection[6])
            track_id = int(matched_detection[4])

            self.last_known_bbox = bbox
            self.last_known_center = (
                (bbox[0] + bbox[2]) // 2,
                (bbox[1] + bbox[3]) // 2,
            )
            self.last_detection_time = timestamp
            self.frames_since_detection = 0

            self.smoothed_confidence = (
                self.confidence_alpha * conf +
                (1 - self.confidence_alpha) * self.smoothed_confidence
            )

            if class_id is not None:
                self.class_history.append(class_id)
            if track_id is not None:
                self.selected_track_id = track_id

            if self.kalman is not None:
                self.kalman.update(bbox)
                kalman_bbox = self.kalman.get_state()
            else:
                kalman_bbox = bbox

            result['tracking_active'] = True
            result['bbox'] = kalman_bbox
            result['center'] = self.last_known_center
            result['confidence'] = self.smoothed_confidence
            result['is_predicted'] = False

        else:
            # No detection — use prediction
            self.frames_since_detection += 1
            self.smoothed_confidence = max(
                0.0,
                self.smoothed_confidence - self.confidence_decay_rate,
            )

            if self.kalman is not None and self.enable_prediction:
                pred_bbox = self.kalman.predict()
                self.last_known_bbox = pred_bbox
                self.last_known_center = (
                    (pred_bbox[0] + pred_bbox[2]) // 2,
                    (pred_bbox[1] + pred_bbox[3]) // 2,
                )

                if self.frames_since_detection <= self.max_history:
                    result['tracking_active'] = True
                    result['bbox'] = pred_bbox
                    result['center'] = self.last_known_center
                    result['confidence'] = self.smoothed_confidence
                    result['is_predicted'] = True

        result['frames_since_detection'] = self.frames_since_detection
        return result

    def get_current_state(self) -> Dict:
        """Get current tracking state."""
        return {
            'tracking_active': self.selected_track_id is not None,
            'track_id': self.selected_track_id,
            'class_id': self.selected_class_id,
            'bbox': self.last_known_bbox,
            'center': self.last_known_center,
            'confidence': self.smoothed_confidence,
            'frames_since_detection': self.frames_since_detection,
        }

    def is_tracking(self) -> bool:
        """Check if currently tracking."""
        return self.selected_track_id is not None
