"""
VelocityRamper — 前向速度斜坡控制器。

Ported from PixEagle MCVelocityChaseFollower._update_forward_velocity()

Provides configurable acceleration-limited forward velocity ramping
with target loss ramp-down and emergency stop support.
"""

import time
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class VelocityRamper:
    """
    Forward velocity ramp controller.

    Features:
    - Configurable acceleration rate (m/s²)
    - Target velocity switching (normal / loss / emergency)
    - Deadzone for near-target settling
    - Absolute velocity limits
    """

    def __init__(
        self,
        initial_velocity: float = 0.0,
        max_velocity: float = 5.0,
        ramp_rate: float = 0.5,          # m/s²
        loss_stop_velocity: float = 0.0,  # m/s — velocity when target lost
        velocity_deadzone: float = 0.01,  # m/s
        min_velocity_threshold: float = 0.2,  # m/s
    ):
        """
        Initialize velocity ramper.

        Args:
            initial_velocity: Starting forward velocity (m/s)
            max_velocity: Maximum forward velocity (m/s)
            ramp_rate: Acceleration rate (m/s²)
            loss_stop_velocity: Target velocity when target is lost (m/s)
            velocity_deadzone: Deadzone for considering velocity "at target" (m/s)
            min_velocity_threshold: Minimum forward velocity threshold (m/s)
        """
        self.max_velocity = max_velocity
        self.ramp_rate = ramp_rate
        self.loss_stop_velocity = loss_stop_velocity
        self.velocity_deadzone = velocity_deadzone
        self.min_velocity_threshold = min_velocity_threshold

        self.current_velocity = initial_velocity
        self.last_update_time = time.time()

    def update(
        self,
        dt: float,
        target_lost: bool = False,
        emergency_stop: bool = False,
        ramp_down_enabled: bool = True,
    ) -> float:
        """
        Update forward velocity with ramping.

        Args:
            dt: Time delta (seconds)
            target_lost: Whether target is currently lost
            emergency_stop: Whether emergency stop is active
            ramp_down_enabled: Whether to ramp down on target loss

        Returns:
            Current forward velocity (m/s)
        """
        # Determine target velocity
        if emergency_stop:
            target_velocity = 0.0
        elif target_lost and ramp_down_enabled:
            target_velocity = self.loss_stop_velocity
        else:
            target_velocity = self.max_velocity

        # Apply ramping
        velocity_error = target_velocity - self.current_velocity

        if abs(velocity_error) < self.velocity_deadzone:
            self.current_velocity = target_velocity
        else:
            max_change = self.ramp_rate * dt
            velocity_change = max(-max_change, min(max_change, velocity_error))
            self.current_velocity += velocity_change

        # Absolute limits
        self.current_velocity = max(0.0, min(self.current_velocity, self.max_velocity))

        self.last_update_time = time.time()
        return self.current_velocity

    def set_max_velocity(self, max_vel: float):
        """Update max velocity at runtime."""
        self.max_velocity = max_vel

    def set_ramp_rate(self, rate: float):
        """Update ramp rate at runtime."""
        self.ramp_rate = rate

    def reset(self, velocity: float = 0.0):
        """Reset to a specific velocity."""
        self.current_velocity = velocity
        self.last_update_time = time.time()

    def get_velocity(self) -> float:
        """Get current velocity."""
        return self.current_velocity
