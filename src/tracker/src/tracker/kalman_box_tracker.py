"""
Kalman Filter for Bounding Box Tracking (SORT-family).

Ported from PixEagle src/classes/kalman_box_tracker.py

State vector: [cx, cy, s, r, vx, vy, vs]
  cx, cy  = bounding box center
  s       = area (width * height)
  r       = aspect ratio (width / height) — assumed constant
  vx, vy  = center velocity
  vs      = area change rate

Measurement vector: [cx, cy, s, r]
"""

import numpy as np
import logging
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


class KalmanBoxTracker:
    """
    Kalman filter state estimator for a single bounding box.

    Provides continuous position prediction during occlusion with proper
    uncertainty growth, and optimal state correction when measurements arrive.
    """

    def __init__(self, bbox: Tuple[int, int, int, int],
                 process_noise_scale: float = 1.0,
                 measurement_noise_scale: float = 1.0):
        """
        Initialize the Kalman filter with a bounding box measurement.

        Args:
            bbox: Initial bounding box (x1, y1, x2, y2)
            process_noise_scale: Scale factor for process noise Q
            measurement_noise_scale: Scale factor for measurement noise R
        """
        self.dim_x = 7
        self.dim_z = 4

        # State transition matrix F (constant velocity model, dt=1 frame)
        self.F = np.eye(self.dim_x)
        self.F[0, 4] = 1.0
        self.F[1, 5] = 1.0
        self.F[2, 6] = 1.0

        # Measurement matrix H
        self.H = np.zeros((self.dim_z, self.dim_x))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0
        self.H[3, 3] = 1.0

        # Process noise covariance Q
        self.Q = np.eye(self.dim_x)
        self.Q[0, 0] = 1.0
        self.Q[1, 1] = 1.0
        self.Q[2, 2] = 1.0
        self.Q[3, 3] = 0.01
        self.Q[4, 4] = 0.01
        self.Q[5, 5] = 0.01
        self.Q[6, 6] = 0.0001
        self.Q *= process_noise_scale

        # Measurement noise covariance R
        self.R = np.eye(self.dim_z)
        self.R[0, 0] = 1.0
        self.R[1, 1] = 1.0
        self.R[2, 2] = 10.0
        self.R[3, 3] = 10.0
        self.R *= measurement_noise_scale

        # State estimate covariance P (initial uncertainty)
        self.P = np.eye(self.dim_x)
        self.P[0, 0] = 10.0
        self.P[1, 1] = 10.0
        self.P[2, 2] = 10.0
        self.P[3, 3] = 10.0
        self.P[4, 4] = 1000.0
        self.P[5, 5] = 1000.0
        self.P[6, 6] = 1000.0

        # Initialize state from first measurement
        measurement = self._bbox_to_measurement(bbox)
        self.x = np.zeros((self.dim_x, 1))
        self.x[:self.dim_z] = measurement.reshape(self.dim_z, 1)

        # Tracking metadata
        self.time_since_update = 0
        self.hit_count = 1
        self.age = 0

        logger.debug(
            "[KalmanBoxTracker] Initialized with bbox=%s, "
            "process_noise=%s, measurement_noise=%s",
            bbox, process_noise_scale, measurement_noise_scale,
        )

    def _bbox_to_measurement(self, bbox: Tuple[int, int, int, int]) -> np.ndarray:
        """Convert (x1,y1,x2,y2) bbox to measurement vector [cx, cy, s, r]."""
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w / 2.0
        cy = y1 + h / 2.0
        s = w * h
        r = w / max(h, 1e-6)
        return np.array([cx, cy, s, r])

    def _state_to_bbox(self, state: np.ndarray) -> Tuple[int, int, int, int]:
        """Convert state vector to (x1,y1,x2,y2) bbox."""
        cx = state[0, 0]
        cy = state[1, 0]
        s = max(state[2, 0], 1.0)
        r = max(state[3, 0], 0.01)

        w = np.sqrt(s * r)
        h = s / max(w, 1e-6)

        x1 = int(cx - w / 2.0)
        y1 = int(cy - h / 2.0)
        x2 = int(cx + w / 2.0)
        y2 = int(cy + h / 2.0)

        return (x1, y1, x2, y2)

    def predict(self) -> Tuple[int, int, int, int]:
        """
        Advance state by one frame using the motion model.

        Returns:
            Predicted bounding box (x1, y1, x2, y2)
        """
        if self.x[2, 0] + self.x[6, 0] <= 0:
            self.x[6, 0] = 0.0

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        self.age += 1
        self.time_since_update += 1

        return self._state_to_bbox(self.x)

    def update(self, bbox: Tuple[int, int, int, int]):
        """
        Correct state with a new measurement (detection).

        Args:
            bbox: Detected bounding box (x1, y1, x2, y2)
        """
        measurement = self._bbox_to_measurement(bbox)
        z = measurement.reshape(self.dim_z, 1)

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R

        try:
            K = self.P @ self.H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            logger.warning("[KalmanBoxTracker] Singular matrix in Kalman gain, skipping update")
            return

        I = np.eye(self.dim_x)
        self.x = self.x + K @ y
        self.P = (I - K @ self.H) @ self.P

        self.time_since_update = 0
        self.hit_count += 1

    def get_state(self) -> Tuple[int, int, int, int]:
        """Get current estimated bounding box."""
        return self._state_to_bbox(self.x)

    def get_center(self) -> Tuple[float, float]:
        """Get current estimated center position."""
        return (float(self.x[0, 0]), float(self.x[1, 0]))

    def get_velocity(self) -> Tuple[float, float]:
        """Get current estimated velocity (px/frame)."""
        return (float(self.x[4, 0]), float(self.x[5, 0]))

    def is_stale(self, max_age: int = 10) -> bool:
        """Check if tracker has not been updated for too long."""
        return self.time_since_update > max_age
