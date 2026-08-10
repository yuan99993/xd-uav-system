"""
PitchCompensator — 俯仰角补偿器。

Ported from PixEagle MCVelocityChaseFollower pitch compensation logic.

Removes geometric image shifts caused by forward pitch angle
from the vertical tracking error before PID processing.
"""

import time
from collections import deque
from typing import Tuple, Deque, Optional
import logging
import math

logger = logging.getLogger(__name__)


class PitchCompensator:
    """
    Removes geometric image shifts caused by forward pitch.

    Physical principle:
    - When drone pitches forward (θ > 0, nose down):
      Camera optical axis rotates downward by angle θ
      Target appears to shift DOWN in image (+Δy in normalized coords)
    - Without compensation, PID sees spurious "target below center" error
    - Compensation removes this artifact BEFORE PID processing

    Models:
    - linear_velocity: compensation ∝ pitch_angle × forward_velocity
    """

    def __init__(
        self,
        enabled: bool = False,
        model: str = 'linear_velocity',
        gain: float = 0.05,
        smoothing_alpha: float = 0.7,
        data_max_age: float = 0.5,          # seconds
        min_velocity: float = 1.0,           # m/s
        deadband: float = 2.0,               # degrees
        max_angle: float = 45.0,             # degrees
        max_correction: float = 0.3,         # normalized coords
        adaptive_gain: bool = False,
    ):
        """
        Initialize pitch compensator.

        Args:
            enabled: Enable pitch compensation
            model: Compensation model ('linear_velocity')
            gain: Compensation gain factor
            smoothing_alpha: EMA alpha for pitch angle
            data_max_age: Maximum age of pitch data (seconds)
            min_velocity: Minimum forward velocity to apply compensation (m/s)
            deadband: Pitch deadband (±degrees)
            max_angle: Maximum valid pitch angle (degrees)
            max_correction: Maximum correction value (normalized coords)
            adaptive_gain: Enable adaptive gain adjustment
        """
        self.enabled = enabled
        self.model = model
        self.gain = gain
        self.smoothing_alpha = smoothing_alpha
        self.data_max_age = data_max_age
        self.min_velocity = min_velocity
        self.deadband = deadband
        self.max_angle = max_angle
        self.max_correction = max_correction
        self.adaptive_gain = adaptive_gain

        # Internal state
        self.current_pitch_angle = 0.0
        self.smoothed_pitch_angle = 0.0
        self.last_pitch_timestamp: Optional[float] = None
        self.active = False
        self.compensation_value = 0.0
        self.data_valid = False
        self.compensation_history: Deque[Tuple[float, float]] = deque(maxlen=20)

    def update_pitch(self, pitch_angle_deg: float, timestamp: float) -> bool:
        """
        Update pitch angle with validation and smoothing.

        Args:
            pitch_angle_deg: Current pitch angle in degrees
                             (positive = nose up, negative = nose down)
            timestamp: Timestamp of pitch data

        Returns:
            True if pitch data is valid
        """
        if not self.enabled:
            return False

        # Validate
        if not math.isfinite(pitch_angle_deg):
            self.data_valid = False
            return False

        # Clamp
        if abs(pitch_angle_deg) > self.max_angle:
            logger.debug(
                "[PitchCompensator] Angle %.1f exceeds max %.1f, clamping",
                pitch_angle_deg, self.max_angle,
            )
            pitch_angle_deg = max(-self.max_angle, min(self.max_angle, pitch_angle_deg))

        # Freshness check
        if self.last_pitch_timestamp is not None:
            data_age = timestamp - self.last_pitch_timestamp
            if data_age > self.data_max_age:
                self.data_valid = False
                return False

        # EMA smoothing
        if self.smoothed_pitch_angle == 0.0 and self.current_pitch_angle == 0.0:
            self.smoothed_pitch_angle = pitch_angle_deg
        else:
            alpha = self.smoothing_alpha
            self.smoothed_pitch_angle = (
                alpha * pitch_angle_deg +
                (1.0 - alpha) * self.smoothed_pitch_angle
            )

        self.current_pitch_angle = pitch_angle_deg
        self.last_pitch_timestamp = timestamp
        self.data_valid = True
        return True

    def compute_compensation(
        self, forward_velocity: float,
    ) -> float:
        """
        Compute pitch compensation value.

        Args:
            forward_velocity: Current forward velocity (m/s)

        Returns:
            Compensation value to subtract from vertical error (normalized coords)
            Positive = target appears lower due to pitch → subtract to compensate
        """
        if not self.enabled or not self.data_valid:
            self.compensation_value = 0.0
            self.active = False
            return 0.0

        # Deadband check
        if abs(self.smoothed_pitch_angle) < self.deadband:
            self.compensation_value = 0.0
            self.active = False
            return 0.0

        # Velocity threshold
        if abs(forward_velocity) < self.min_velocity:
            self.compensation_value = 0.0
            self.active = False
            return 0.0

        # Compute compensation based on model
        if self.model == 'linear_velocity':
            # compensation = gain × pitch_angle × forward_velocity
            # pitch positive = nose up → target appears HIGHER → NEGATIVE compensation
            compensation = (
                -self.gain * self.smoothed_pitch_angle * forward_velocity
            )
        else:
            compensation = 0.0

        # Adaptive gain
        if self.adaptive_gain:
            # Increase gain at higher velocities
            speed_factor = min(forward_velocity / 5.0, 2.0)
            compensation *= speed_factor

        # Clamp
        compensation = max(
            -self.max_correction,
            min(self.max_correction, compensation),
        )

        self.compensation_value = compensation
        self.active = True
        self.compensation_history.append((time.time(), compensation))

        return compensation

    def reset(self):
        """Reset state."""
        self.current_pitch_angle = 0.0
        self.smoothed_pitch_angle = 0.0
        self.last_pitch_timestamp = None
        self.active = False
        self.compensation_value = 0.0
        self.data_valid = False
        self.compensation_history.clear()
