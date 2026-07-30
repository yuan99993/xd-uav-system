"""
Motion Predictor for occlusion handling.

Ported from PixEagle src/classes/motion_predictor.py

Provides velocity-based motion prediction with EMA smoothing and
acceleration-aware kinematic prediction for multi-frame lookahead.
"""

import time
from collections import deque
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class MotionPredictor:
    """
    Predicts object motion during temporary tracking loss.

    Uses velocity-based linear prediction with EMA smoothing to estimate
    where the object will be during brief occlusions.
    """

    def __init__(self, history_size: int = 5, velocity_alpha: float = 0.7,
                 acceleration_alpha: float = 0.5):
        """
        Initialize the motion predictor.

        Args:
            history_size: Number of previous positions to store
            velocity_alpha: EMA smoothing factor for velocity (0-1)
            acceleration_alpha: EMA smoothing factor for acceleration (0-1)
        """
        self.history_size = history_size
        self.velocity_alpha = velocity_alpha
        self.acceleration_alpha = acceleration_alpha

        self.position_history = deque(maxlen=history_size)

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_w = 0.0
        self.velocity_h = 0.0

        self.prev_velocity_x = 0.0
        self.prev_velocity_y = 0.0

        self.accel_x = 0.0
        self.accel_y = 0.0

        self.last_update_time = 0.0

        logger.info(
            "[MotionPredictor] Initialized: history_size=%d, velocity_alpha=%.2f, "
            "acceleration_alpha=%.2f",
            history_size, velocity_alpha, acceleration_alpha,
        )

    def update(self, bbox: Tuple[int, int, int, int], timestamp: float):
        """
        Update motion history with new detection.

        Args:
            bbox: Bounding box (x1, y1, x2, y2)
            timestamp: Detection timestamp
        """
        self.position_history.append((bbox, timestamp))
        self.last_update_time = timestamp

        if len(self.position_history) >= 2:
            self._update_velocity()

    def _update_velocity(self):
        """Compute smoothed velocity and acceleration from position history."""
        if len(self.position_history) < 2:
            return

        (prev_bbox, prev_time) = self.position_history[-2]
        (curr_bbox, curr_time) = self.position_history[-1]

        dt = curr_time - prev_time
        if dt <= 0:
            return

        self.prev_velocity_x = self.velocity_x
        self.prev_velocity_y = self.velocity_y

        prev_cx = (prev_bbox[0] + prev_bbox[2]) / 2
        prev_cy = (prev_bbox[1] + prev_bbox[3]) / 2
        curr_cx = (curr_bbox[0] + curr_bbox[2]) / 2
        curr_cy = (curr_bbox[1] + curr_bbox[3]) / 2

        instant_vx = (curr_cx - prev_cx) / dt
        instant_vy = (curr_cy - prev_cy) / dt

        prev_w = prev_bbox[2] - prev_bbox[0]
        prev_h = prev_bbox[3] - prev_bbox[1]
        curr_w = curr_bbox[2] - curr_bbox[0]
        curr_h = curr_bbox[3] - curr_bbox[1]

        instant_vw = (curr_w - prev_w) / dt
        instant_vh = (curr_h - prev_h) / dt

        alpha = self.velocity_alpha
        self.velocity_x = alpha * instant_vx + (1 - alpha) * self.velocity_x
        self.velocity_y = alpha * instant_vy + (1 - alpha) * self.velocity_y
        self.velocity_w = alpha * instant_vw + (1 - alpha) * self.velocity_w
        self.velocity_h = alpha * instant_vh + (1 - alpha) * self.velocity_h

        if len(self.position_history) >= 3:
            instant_ax = (self.velocity_x - self.prev_velocity_x) / dt
            instant_ay = (self.velocity_y - self.prev_velocity_y) / dt

            acc_alpha = self.acceleration_alpha
            self.accel_x = acc_alpha * instant_ax + (1 - acc_alpha) * self.accel_x
            self.accel_y = acc_alpha * instant_ay + (1 - acc_alpha) * self.accel_y

            max_accel = 500.0
            self.accel_x = max(-max_accel, min(max_accel, self.accel_x))
            self.accel_y = max(-max_accel, min(max_accel, self.accel_y))

    def predict_bbox(self, frames_ahead: int, fps: float = 30.0,
                     use_acceleration: bool = True) -> Optional[Tuple[int, int, int, int]]:
        """
        Predict bounding box N frames into the future.

        Args:
            frames_ahead: Number of frames to predict ahead
            fps: Frames per second
            use_acceleration: Whether to include acceleration term

        Returns:
            Predicted bbox or None if no history
        """
        if not self.position_history:
            return None

        last_bbox, _ = self.position_history[-1]
        x1, y1, x2, y2 = last_bbox
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w / 2.0
        cy = y1 + h / 2.0

        dt = frames_ahead / fps

        if use_acceleration:
            pred_cx = cx + self.velocity_x * dt + 0.5 * self.accel_x * dt * dt
            pred_cy = cy + self.velocity_y * dt + 0.5 * self.accel_y * dt * dt
        else:
            pred_cx = cx + self.velocity_x * dt
            pred_cy = cy + self.velocity_y * dt

        pred_w = w + self.velocity_w * dt
        pred_h = h + self.velocity_h * dt

        pred_w = max(pred_w, 10)
        pred_h = max(pred_h, 10)

        px1 = int(pred_cx - pred_w / 2.0)
        py1 = int(pred_cy - pred_h / 2.0)
        px2 = int(pred_cx + pred_w / 2.0)
        py2 = int(pred_cy + pred_h / 2.0)

        return (px1, py1, px2, py2)

    def get_velocity(self) -> Tuple[float, float]:
        """Get smoothed velocity (px/s)."""
        return (self.velocity_x, self.velocity_y)

    def get_acceleration(self) -> Tuple[float, float]:
        """Get smoothed acceleration (px/s^2)."""
        return (self.accel_x, self.accel_y)

    def reset(self):
        """Clear all history."""
        self.position_history.clear()
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_w = 0.0
        self.velocity_h = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
