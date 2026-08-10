"""
AdaptiveDiveClimb — 自适应俯冲/爬升控制器。

Ported from PixEagle MCVelocityChaseFollower adaptive dive/climb logic.

Monitors target vertical pixel rate vs expected rate from commanded velocities,
and applies adaptive corrections to forward/down velocity commands.
"""

import time
from collections import deque
from typing import Tuple, Deque, Optional
import logging

logger = logging.getLogger(__name__)


class AdaptiveDiveClimb:
    """
    Adaptive dive/climb controller for target following.

    Phase 1: Estimates vertical pixel rate from target motion history
    Phase 2: Compares to expected rate from velocity commands
    Phase 3: Applies proportional corrections with oscillation detection
    """

    def __init__(
        self,
        enabled: bool = False,
        smoothing_alpha: float = 0.2,
        warmup_frames: int = 10,
        rate_threshold: float = 5.0,         # px/s deadzone
        max_correction: float = 1.0,          # m/s max correction
        correction_gain: float = 0.3,
        min_confidence: float = 0.6,
        fwd_coupling_enabled: bool = False,
        fwd_coupling_gain: float = 0.1,
        pixel_to_rate_calibration: float = 0.05,
        oscillation_detection: bool = True,
        max_sign_changes: int = 3,
        divergence_timeout: float = 5.0,
        video_height_pixels: float = 480.0,
    ):
        """
        Initialize adaptive dive/climb controller.

        Args:
            enabled: Whether adaptive mode is enabled
            smoothing_alpha: EMA alpha for vertical rate
            warmup_frames: Frames to accumulate before computing rates
            rate_threshold: Deadzone threshold (px/s)
            max_correction: Maximum velocity correction (m/s)
            correction_gain: Proportional gain for correction
            min_confidence: Minimum tracker confidence to enable
            fwd_coupling_enabled: Whether to couple to forward velocity
            fwd_coupling_gain: Forward coupling gain
            pixel_to_rate_calibration: Calibration factor px/s → m/s
            oscillation_detection: Enable oscillation detection
            max_sign_changes: Max sign changes before disabling
            divergence_timeout: Timeout for divergence detection (s)
            video_height_pixels: Video height for rate scaling
        """
        self.enabled = enabled
        self.smoothing_alpha = smoothing_alpha
        self.warmup_frames = warmup_frames
        self.rate_threshold = rate_threshold
        self.max_correction = max_correction
        self.correction_gain = correction_gain
        self.min_confidence = min_confidence
        self.fwd_coupling_enabled = fwd_coupling_enabled
        self.fwd_coupling_gain = fwd_coupling_gain
        self.pixel_to_rate_calibration = pixel_to_rate_calibration
        self.oscillation_detection = oscillation_detection
        self.max_sign_changes = max_sign_changes
        self.divergence_timeout = divergence_timeout
        self.video_height_pixels = video_height_pixels

        # Internal state
        self.active = False
        self.warmup_counter = 0
        self.target_vertical_history: Deque[Tuple[float, float]] = deque(maxlen=30)
        self.smoothed_vertical_rate = 0.0
        self.expected_vertical_rate = 0.0
        self.vertical_rate_error = 0.0
        self.correction_down = 0.0
        self.correction_fwd = 0.0

        # Oscillation / divergence detection
        self.rate_error_sign_history: Deque[Tuple[float, int]] = deque(maxlen=50)
        self.disabled_reason: Optional[str] = None
        self.divergence_start_time: Optional[float] = None

        # Last Y coord
        self.last_target_y: Optional[float] = None

    def update(
        self,
        target_y: float,
        target_confidence: float,
        current_v_down: float,
        current_v_fwd: float,
        lateral_mode: str = 'coordinated_turn',
        timestamp: Optional[float] = None,
    ) -> Tuple[float, float]:
        """
        Update adaptive control and return corrected velocities.

        Args:
            target_y: Normalized target Y coordinate [-1, 1]
            target_confidence: Tracker confidence [0, 1]
            current_v_down: Current down velocity command (m/s)
            current_v_fwd: Current forward velocity command (m/s)
            lateral_mode: 'coordinated_turn' or 'sideslip'
            timestamp: Current timestamp

        Returns:
            (adjusted_v_fwd, adjusted_v_down) in m/s
        """
        if timestamp is None:
            timestamp = time.time()

        if not self.enabled:
            return current_v_fwd, current_v_down

        # Confidence check
        if target_confidence < self.min_confidence:
            self.active = False
            return current_v_fwd, current_v_down

        # Record history
        self.target_vertical_history.append((timestamp, target_y))
        self.warmup_counter += 1

        if self.warmup_counter < self.warmup_frames:
            return current_v_fwd, current_v_down

        if len(self.target_vertical_history) < 2:
            return current_v_fwd, current_v_down

        # Compute vertical rate
        sample_window = min(5, len(self.target_vertical_history))
        recent = list(self.target_vertical_history)[-sample_window:]
        t1, y1 = recent[0]
        t2, y2 = recent[-1]
        dt = t2 - t1

        if dt > 0.001:
            raw_rate = (y2 - y1) / dt
            instantaneous_rate = raw_rate * self.video_height_pixels
            alpha = self.smoothing_alpha
            self.smoothed_vertical_rate = (
                alpha * instantaneous_rate +
                (1.0 - alpha) * self.smoothed_vertical_rate
            )

        # Expected rate from commanded velocity
        self.expected_vertical_rate = (
            current_v_down * self.pixel_to_rate_calibration *
            self.video_height_pixels
        )
        rate_error = self.smoothed_vertical_rate - self.expected_vertical_rate
        self.vertical_rate_error = rate_error
        self.active = True

        # Oscillation detection
        if self.oscillation_detection and self._check_oscillation():
            self.enabled = False
            self.disabled_reason = "oscillation_detected"
            logger.warning("[AdaptiveDiveClimb] Disabled due to oscillation")
            return current_v_fwd, current_v_down

        # Divergence detection
        if self._check_divergence():
            self.enabled = False
            self.disabled_reason = "divergence_detected"
            logger.warning("[AdaptiveDiveClimb] Disabled due to divergence")
            return current_v_fwd, current_v_down

        # Deadzone
        if abs(rate_error) < self.rate_threshold:
            self.correction_down = 0.0
            self.correction_fwd = 0.0
            return current_v_fwd, current_v_down

        # Compute correction
        error_magnitude = rate_error - self._sign(rate_error) * self.rate_threshold
        base_correction = error_magnitude * self.correction_gain
        base_correction = max(-self.max_correction, min(self.max_correction, base_correction))

        # Mode-specific authority
        authority_factor = 0.5 if lateral_mode == 'sideslip' else 1.0
        base_correction *= authority_factor

        # Apply to velocities
        v_down_correction = base_correction * 0.01
        v_fwd_correction = (
            -base_correction * self.fwd_coupling_gain * 0.01
            if self.fwd_coupling_enabled else 0.0
        )

        self.correction_down = v_down_correction
        self.correction_fwd = v_fwd_correction

        adjusted_v_down = current_v_down + v_down_correction
        adjusted_v_fwd = current_v_fwd + v_fwd_correction

        return adjusted_v_fwd, adjusted_v_down

    def _check_oscillation(self) -> bool:
        """Detect oscillation via sign change counting."""
        if abs(self.vertical_rate_error) > 0.1:
            error_sign = int(self._sign(self.vertical_rate_error))
            self.rate_error_sign_history.append((time.time(), error_sign))

        if len(self.rate_error_sign_history) < 4:
            return False

        recent_window = 2.0
        cutoff = time.time() - recent_window
        recent_signs = [(t, s) for t, s in self.rate_error_sign_history if t >= cutoff]

        if len(recent_signs) < 2:
            return False

        sign_changes = sum(
            1 for i in range(1, len(recent_signs))
            if recent_signs[i][1] != recent_signs[i-1][1]
        )

        return sign_changes >= self.max_sign_changes

    def _check_divergence(self) -> bool:
        """Detect divergence: growing error over timeout."""
        error_magnitude = abs(self.vertical_rate_error)
        if error_magnitude > self.rate_threshold * 2.0:
            if self.divergence_start_time is None:
                self.divergence_start_time = time.time()
            else:
                if time.time() - self.divergence_start_time > self.divergence_timeout:
                    return True
        else:
            self.divergence_start_time = None
        return False

    @staticmethod
    def _sign(x: float) -> float:
        """Sign function (0 returns 0)."""
        return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)

    def reset(self):
        """Reset state."""
        self.active = False
        self.warmup_counter = 0
        self.target_vertical_history.clear()
        self.smoothed_vertical_rate = 0.0
        self.expected_vertical_rate = 0.0
        self.vertical_rate_error = 0.0
        self.correction_down = 0.0
        self.correction_fwd = 0.0
        self.rate_error_sign_history.clear()
        self.disabled_reason = None
        self.divergence_start_time = None
