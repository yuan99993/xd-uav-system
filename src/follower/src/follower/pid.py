"""
CustomPID — 高级 PID 控制器。

Ported from PixEagle src/classes/followers/custom_pid.py

Features:
- Proportional on Measurement (PoM): 增强稳定性
- Anti-windup using back-calculation: 防止积分饱和
- Configurable output limits
"""

import logging
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


class CustomPID:
    """
    Custom PID controller with PoM and anti-windup.

    Unlike simple_pid.PID, this implementation:
    - Uses Proportional on Measurement for smoother response
    - Implements back-calculation anti-windup
    - Provides reset() for mode switching
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        setpoint: float = 0.0,
        output_limits: Optional[Tuple[float, float]] = None,
        proportional_on_measurement: bool = True,
        enable_anti_windup: bool = True,
        anti_windup_back_calc_coeff: float = 0.1,
    ):
        """
        Initialize CustomPID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            setpoint: Target setpoint
            output_limits: (min, max) output limits
            proportional_on_measurement: Use PoM mode
            enable_anti_windup: Enable anti-windup
            anti_windup_back_calc_coeff: Back-calculation coefficient
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits

        self.proportional_on_measurement = proportional_on_measurement
        self.enable_anti_windup = enable_anti_windup
        self.anti_windup_back_calc_coeff = anti_windup_back_calc_coeff

        # Internal state
        self._integral = 0.0
        self._last_input = 0.0
        self._last_output = 0.0
        self._last_error = 0.0
        self._first_run = True

    def reset(self):
        """Reset all internal state."""
        self._integral = 0.0
        self._last_input = 0.0
        self._last_output = 0.0
        self._last_error = 0.0
        self._first_run = True

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains at runtime."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def __call__(self, input_: float, dt: Optional[float] = None) -> float:
        """
        Compute PID output.

        Args:
            input_: Current measurement value
            dt: Time delta in seconds (None = use 1.0 for frame-based)

        Returns:
            Control output
        """
        if dt is None or dt <= 0:
            dt = 1.0

        # Handle first call
        if self._first_run:
            self._last_input = input_
            self._first_run = False

        # Compute error
        error = self.setpoint - self._last_input if self.proportional_on_measurement else self.setpoint - input_

        # ── Proportional term ────────────────────────────────────────────
        if self.proportional_on_measurement:
            # P term based on measurement change (not error)
            p_term = -self.kp * (input_ - self._last_input)
        else:
            p_term = self.kp * error

        # ── Integral term ────────────────────────────────────────────────
        self._integral += self.ki * error * dt

        # ── Derivative term ──────────────────────────────────────────────
        if self.proportional_on_measurement:
            d_term = -self.kd * (input_ - self._last_input) / dt
        else:
            d_term = self.kd * (error - self._last_error) / dt

        # ── Sum ──────────────────────────────────────────────────────────
        output = p_term + self._integral + d_term

        # ── Anti-windup ──────────────────────────────────────────────────
        if self.enable_anti_windup and self.output_limits is not None:
            if output != self._last_output:
                if output >= self.output_limits[1] or output <= self.output_limits[0]:
                    diff = output - self._last_output
                    self._integral -= diff * self.anti_windup_back_calc_coeff

        # ── Output limits ────────────────────────────────────────────────
        if self.output_limits is not None:
            output = max(self.output_limits[0], min(self.output_limits[1], output))

        # ── Store state ──────────────────────────────────────────────────
        self._last_input = input_
        self._last_error = error
        self._last_output = output

        return output

    @property
    def integral(self) -> float:
        """Get current integral value."""
        return self._integral

    @property
    def last_output(self) -> float:
        """Get last output value."""
        return self._last_output
