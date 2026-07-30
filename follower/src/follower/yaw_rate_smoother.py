"""
YawRateSmoother — 偏航速率平滑器。

Ported from PixEagle src/classes/followers/yaw_rate_smoother.py

Applies deadzone, rate-of-change limiting, EMA smoothing, and
speed-adaptive scaling to raw yaw rate commands.
"""

from dataclasses import dataclass, field
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)


@dataclass
class YawRateSmoother:
    """
    Yaw-rate smoothing pipeline.

    Features:
    - Deadzone to prevent jitter at low rates
    - Rate-of-change limiting for smooth acceleration
    - EMA smoothing for noise reduction
    - Speed-adaptive scaling
    """

    # Configuration
    enabled: bool = True
    deadzone_deg_s: float = 0.5
    max_rate_change_deg_s2: float = 90.0
    smoothing_alpha: float = 0.7
    enable_speed_scaling: bool = True
    min_speed_threshold: float = 0.5       # m/s
    max_speed_threshold: float = 5.0       # m/s
    low_speed_yaw_factor: float = 0.5

    # Internal state
    last_yaw_rate: float = field(default=0.0, init=False)
    filtered_yaw_rate: float = field(default=0.0, init=False)

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> 'YawRateSmoother':
        """Create from configuration dictionary."""
        return cls(
            enabled=config.get('enabled', True),
            deadzone_deg_s=config.get('deadzone_deg_s', 0.5),
            max_rate_change_deg_s2=config.get('max_rate_change_deg_s2', 90.0),
            smoothing_alpha=config.get('smoothing_alpha', 0.7),
            enable_speed_scaling=config.get('enable_speed_scaling', True),
            min_speed_threshold=config.get('min_speed_threshold', 0.5),
            max_speed_threshold=config.get('max_speed_threshold', 5.0),
            low_speed_yaw_factor=config.get('low_speed_yaw_factor', 0.5),
        )

    def reset(self):
        """Reset internal state."""
        self.last_yaw_rate = 0.0
        self.filtered_yaw_rate = 0.0

    def apply(self, raw_yaw_rate: float, dt: float,
              forward_speed: float = 0.0) -> float:
        """
        Apply full smoothing pipeline.

        Args:
            raw_yaw_rate: Raw PID output (deg/s)
            dt: Time delta since last call (seconds)
            forward_speed: Current forward velocity (m/s)

        Returns:
            Smoothed yaw rate (deg/s)
        """
        if not self.enabled:
            return raw_yaw_rate

        # 1. Deadzone
        yaw_rate = self._apply_deadzone(raw_yaw_rate)

        # 2. Speed-adaptive scaling
        if self.enable_speed_scaling:
            yaw_rate = self._apply_speed_scaling(yaw_rate, forward_speed)

        # 3. Rate-of-change limiting
        yaw_rate = self._apply_rate_limiting(yaw_rate, dt)

        # 4. EMA smoothing
        yaw_rate = self._apply_ema_smoothing(yaw_rate)

        return yaw_rate

    def _apply_deadzone(self, rate: float) -> float:
        """Apply deadzone with smooth transition."""
        if abs(rate) < self.deadzone_deg_s:
            return 0.0
        sign = 1.0 if rate > 0 else -1.0
        return sign * (abs(rate) - self.deadzone_deg_s)

    def _apply_speed_scaling(self, rate: float, speed: float) -> float:
        """Scale yaw authority based on forward speed."""
        if speed >= self.max_speed_threshold:
            return rate
        if speed <= self.min_speed_threshold:
            return rate * self.low_speed_yaw_factor
        t = (speed - self.min_speed_threshold) / (
            self.max_speed_threshold - self.min_speed_threshold
        )
        factor = self.low_speed_yaw_factor + t * (1.0 - self.low_speed_yaw_factor)
        return rate * factor

    def _apply_rate_limiting(self, target_rate: float, dt: float) -> float:
        """Limit rate-of-change for smooth acceleration."""
        if dt <= 0:
            return target_rate
        max_change = self.max_rate_change_deg_s2 * dt
        change = target_rate - self.last_yaw_rate
        change = max(-max_change, min(max_change, change))
        self.last_yaw_rate = self.last_yaw_rate + change
        return self.last_yaw_rate

    def _apply_ema_smoothing(self, rate: float) -> float:
        """Apply EMA smoothing."""
        alpha = self.smoothing_alpha
        self.filtered_yaw_rate = (
            alpha * rate + (1.0 - alpha) * self.filtered_yaw_rate
        )
        return self.filtered_yaw_rate
