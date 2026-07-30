"""
SafetyLimits — 安全限幅类型和验证。

Ported from PixEagle safety_types.py and command_safety.py.

Defines named tuples for velocity, altitude, and rate limits.
Provides validation and clamping utilities for follower commands.
"""

from typing import NamedTuple, Optional, Tuple
import math
import logging

logger = logging.getLogger(__name__)


class VelocityLimits(NamedTuple):
    """Velocity limits in m/s."""
    forward: float = 15.0      # Max forward velocity
    lateral: float = 5.0       # Max lateral (right) velocity
    vertical: float = 3.0      # Max vertical (down) velocity
    max_magnitude: float = 15.0  # Overall magnitude limit


class AltitudeLimits(NamedTuple):
    """Altitude limits in meters."""
    min_altitude: float = 2.0
    max_altitude: float = 120.0
    warning_buffer: float = 5.0
    safety_enabled: bool = True


class RateLimits(NamedTuple):
    """Rate limits in rad/s."""
    yaw: float = 1.57          # ~90 deg/s
    pitch: float = 1.57
    roll: float = 1.57


class SafetyValidator:
    """
    Centralized safety validation for follower commands.

    Validates and clamps velocity, altitude, and rate commands
    against configured limits.
    """

    def __init__(
        self,
        velocity_limits: Optional[VelocityLimits] = None,
        altitude_limits: Optional[AltitudeLimits] = None,
        rate_limits: Optional[RateLimits] = None,
    ):
        self.velocity_limits = velocity_limits or VelocityLimits()
        self.altitude_limits = altitude_limits or AltitudeLimits()
        self.rate_limits = rate_limits or RateLimits()

    def clamp_velocity_forward(self, v: float) -> Tuple[float, bool]:
        """Clamp forward velocity. Returns (clamped_value, was_clamped)."""
        clamped = max(0.0, min(v, self.velocity_limits.forward))
        return clamped, (clamped != v)

    def clamp_velocity_lateral(self, v: float) -> Tuple[float, bool]:
        """Clamp lateral velocity."""
        limit = self.velocity_limits.lateral
        clamped = max(-limit, min(v, limit))
        return clamped, (clamped != v)

    def clamp_velocity_vertical(self, v: float) -> Tuple[float, bool]:
        """Clamp vertical velocity."""
        limit = self.velocity_limits.vertical
        clamped = max(-limit, min(v, limit))
        return clamped, (clamped != v)

    def clamp_yaw_rate(self, rate_rad_s: float) -> Tuple[float, bool]:
        """Clamp yaw rate (rad/s)."""
        limit = self.rate_limits.yaw
        clamped = max(-limit, min(rate_rad_s, limit))
        return clamped, (clamped != rate_rad_s)

    def check_altitude(self, altitude: float) -> Tuple[bool, str]:
        """
        Check if altitude is within safe limits.

        Returns:
            (is_safe, reason)
        """
        if not self.altitude_limits.safety_enabled:
            return True, "safety_disabled"

        if altitude < self.altitude_limits.min_altitude:
            return False, f"too_low ({altitude:.1f}m < {self.altitude_limits.min_altitude}m)"
        if altitude > self.altitude_limits.max_altitude:
            return False, f"too_high ({altitude:.1f}m > {self.altitude_limits.max_altitude}m)"
        return True, "ok"

    def check_altitude_warning(self, altitude: float) -> Tuple[bool, str]:
        """Check if altitude is approaching limits."""
        buf = self.altitude_limits.warning_buffer
        low_warn = self.altitude_limits.min_altitude + buf
        high_warn = self.altitude_limits.max_altitude - buf

        if altitude < low_warn:
            return True, f"approaching_min ({altitude:.1f}m < {low_warn:.1f}m)"
        if altitude > high_warn:
            return True, f"approaching_max ({altitude:.1f}m > {high_warn:.1f}m)"
        return False, "ok"

    def clamp_command_magnitude(
        self, vx: float, vy: float, vz: float,
    ) -> Tuple[float, float, float]:
        """Clamp total velocity magnitude."""
        magnitude = math.sqrt(vx * vx + vy * vy + vz * vz)
        max_mag = self.velocity_limits.max_magnitude
        if magnitude > max_mag:
            scale = max_mag / magnitude
            return vx * scale, vy * scale, vz * scale
        return vx, vy, vz
