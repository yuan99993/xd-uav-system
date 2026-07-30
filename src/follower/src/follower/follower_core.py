"""
FollowerCore — 目标跟随控制核心。

Ported and adapted from PixEagle MCVelocityChaseFollower.

Integrates:
- CustomPID controllers (yaw / lateral / vertical)
- YawRateSmoother (deadzone + rate limit + EMA + speed scaling)
- VelocityRamper (configurable forward acceleration)
- AdaptiveDiveClimb (vertical rate feedback correction)
- PitchCompensator (forward pitch geometric correction)
- TargetLossHandler (timeout-based loss detection)
- SafetyValidator (velocity/altitude/rate limits)
- PX4Interface (MAVROS / MAVSDK telemetry + command)

Control Flow (coordinated_turn mode):
  1. Receive normalized error (ex, ey) from Tracker
  2. Pitch compensation removes geometric distortion from ey
  3. Yaw PID: ex → yaw_rate (rad/s) → YawRateSmoother → deg/s
  4. Down PID: ey → velocity_down (m/s)
  5. Forward: VelocityRamper → velocity_forward (m/s)
  6. Adaptive corrections applied to fwd/down velocities
  7. Safety clamping on all outputs
  8. Output via PX4OffboardCommand → Controller port

Author: Follower Team (adapted from PixEagle)
"""

import time
import math
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass, field

from follower.pid import CustomPID
from follower.yaw_rate_smoother import YawRateSmoother
from follower.velocity_ramper import VelocityRamper
from follower.adaptive_dive_climb import AdaptiveDiveClimb
from follower.pitch_compensator import PitchCompensator
from follower.target_loss_handler import TargetLossHandler
from follower.safety_limits import SafetyValidator, VelocityLimits, AltitudeLimits, RateLimits
from follower.px4_interface import PX4Interface, PX4Telemetry, PX4OffboardCommand
from follower.feasibility_limiter import FeasibilityLimiter

import logging
logger = logging.getLogger(__name__)


@dataclass
class FollowerResult:
    """
    Complete follower computation result.

    Contains: control commands, status, diagnostics, and Controller port data.
    """
    timestamp: float = 0.0

    # ── Control Commands ─────────────────────────────────────────────────
    velocity_forward: float = 0.0      # m/s, body-x
    velocity_right: float = 0.0        # m/s, body-y
    velocity_down: float = 0.0         # m/s, body-z (positive = down)
    yaw_rate_deg_s: float = 0.0        # deg/s
    yaw_rate_raw_deg_s: float = 0.0    # deg/s (before smoothing)
    yaw_smoothing_active: bool = False

    # ── Command Validity ─────────────────────────────────────────────────
    command_valid: bool = False
    target_visible: bool = False
    emergency_stop_active: bool = False

    # ── Adaptive ─────────────────────────────────────────────────────────
    adaptive_active: bool = False
    adaptive_correction_down: float = 0.0
    adaptive_correction_fwd: float = 0.0

    # ── Pitch Compensation ───────────────────────────────────────────────
    pitch_compensation_active: bool = False
    pitch_compensation_value: float = 0.0

    # ── Target Info ──────────────────────────────────────────────────────
    target_error_x: float = 0.0        # Normalized [-1, 1]
    target_error_y: float = 0.0        # Normalized [-1, 1]
    target_lost: bool = False
    target_loss_duration: float = 0.0

    # ── PID Outputs ──────────────────────────────────────────────────────
    pid_yaw_output: float = 0.0        # rad/s
    pid_down_output: float = 0.0       # m/s
    pid_right_output: float = 0.0      # m/s

    # ── Control Mode ─────────────────────────────────────────────────────
    control_mode: str = "coordinated_turn"
    lateral_guidance_mode: str = "coordinated_turn"

    # ── Velocities (post-ramp) ───────────────────────────────────────────
    current_forward_velocity: float = 0.0

    # ── Diagnostics ──────────────────────────────────────────────────────
    altitude_current: float = 0.0
    altitude_safe: bool = True
    update_count: int = 0

    # ── Controller Port (for downstream Controller) ──────────────────────
    suggested_velocity_forward: float = 0.0
    suggested_velocity_right: float = 0.0
    suggested_velocity_down: float = 0.0
    suggested_yaw_rate_deg_s: float = 0.0
    suggestion_valid: bool = False


class FollowerCore:
    """
    Main follower coordinator — computes body-velocity commands from tracker errors.

    Ported from PixEagle MCVelocityChaseFollower, adapted for ROS1/MAVROS.

    Control modes:
    - coordinated_turn: Yaw to track target (no lateral velocity)
    - sideslip: Lateral velocity to track (no yaw)

    Data flow:
        Tracker errors (ex, ey) + PX4 telemetry → PID control → velocity commands
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize FollowerCore.

        Args:
            config: Configuration dictionary (see follower_params.yaml for full schema)
        """
        self.config = config or {}

        # ── Control Mode ─────────────────────────────────────────────────
        self.lateral_guidance_mode = self.config.get('lateral_guidance_mode', 'coordinated_turn')
        self.enable_auto_mode_switching = self.config.get('enable_auto_mode_switching', False)
        self.mode_switch_velocity = self.config.get('mode_switch_velocity', 3.0)
        self.mode_switch_hysteresis = self.config.get('mode_switch_hysteresis', 0.5)
        self.min_mode_switch_interval = self.config.get('min_mode_switch_interval', 2.0)
        self.last_mode_switch_time = 0.0

        # ── Safety Limits ────────────────────────────────────────────────
        self.safety = SafetyValidator(
            velocity_limits=VelocityLimits(
                forward=self.config.get('max_velocity_forward', 8.0),
                lateral=self.config.get('max_velocity_lateral', 3.0),
                vertical=self.config.get('max_velocity_vertical', 2.0),
                max_magnitude=self.config.get('max_velocity_magnitude', 15.0),
            ),
            altitude_limits=AltitudeLimits(
                min_altitude=self.config.get('min_altitude', 2.0),
                max_altitude=self.config.get('max_altitude', 120.0),
                warning_buffer=self.config.get('altitude_warning_buffer', 5.0),
            ),
            rate_limits=RateLimits(
                yaw=math.radians(self.config.get('max_yaw_rate_deg_s', 90.0)),
            ),
        )

        # ── Feasibility Limiter (ego-planner inspired) ───────────────────
        self.feasibility = FeasibilityLimiter(
            max_vel=self.config.get('feasibility_max_vel', 8.0),
            max_acc=self.config.get('feasibility_max_acc', 5.0),
            max_jerk=self.config.get('feasibility_max_jerk', 20.0),
            feasibility_tolerance=self.config.get('feasibility_tolerance', 0.1),
        )

        # ── PX4 Interface ────────────────────────────────────────────────
        self.px4 = PX4Interface()

        # ── Velocity Ramper ──────────────────────────────────────────────
        self.ramper = VelocityRamper(
            initial_velocity=self.config.get('initial_forward_velocity', 0.0),
            max_velocity=self.config.get('max_forward_velocity', 5.0),
            ramp_rate=self.config.get('forward_ramp_rate', 0.5),
            loss_stop_velocity=self.config.get('target_loss_stop_velocity', 0.0),
            velocity_deadzone=self.config.get('forward_velocity_deadzone', 0.01),
            min_velocity_threshold=self.config.get('min_forward_velocity_threshold', 0.2),
        )

        # ── Yaw Rate Smoother ────────────────────────────────────────────
        self.yaw_smoother = YawRateSmoother.from_config({
            'enabled': self.config.get('yaw_smoothing_enabled', True),
            'deadzone_deg_s': self.config.get('yaw_deadzone_deg_s', 0.5),
            'max_rate_change_deg_s2': self.config.get('yaw_max_rate_change_deg_s2', 90.0),
            'smoothing_alpha': self.config.get('yaw_smoothing_alpha', 0.7),
            'enable_speed_scaling': self.config.get('yaw_speed_scaling_enabled', True),
            'min_speed_threshold': self.config.get('yaw_min_speed_threshold', 0.5),
            'max_speed_threshold': self.config.get('yaw_max_speed_threshold', 5.0),
            'low_speed_yaw_factor': self.config.get('yaw_low_speed_factor', 0.5),
        })

        # ── PID Controllers ──────────────────────────────────────────────
        self._init_pid_controllers()

        # ── Adaptive Dive/Climb ──────────────────────────────────────────
        self.adaptive = AdaptiveDiveClimb(
            enabled=self.config.get('adaptive_dive_climb_enabled', False),
            smoothing_alpha=self.config.get('adaptive_smoothing_alpha', 0.2),
            warmup_frames=self.config.get('adaptive_warmup_frames', 10),
            rate_threshold=self.config.get('adaptive_rate_threshold', 5.0),
            max_correction=self.config.get('adaptive_max_correction', 1.0),
            correction_gain=self.config.get('adaptive_correction_gain', 0.3),
            min_confidence=self.config.get('adaptive_min_confidence', 0.6),
            fwd_coupling_enabled=self.config.get('adaptive_fwd_coupling_enabled', False),
            fwd_coupling_gain=self.config.get('adaptive_fwd_coupling_gain', 0.1),
            pixel_to_rate_calibration=self.config.get('pixel_to_rate_calibration', 0.05),
            video_height_pixels=self.config.get('video_height_pixels', 480.0),
        )

        # ── Pitch Compensator ────────────────────────────────────────────
        self.pitch_comp = PitchCompensator(
            enabled=self.config.get('pitch_compensation_enabled', False),
            model=self.config.get('pitch_compensation_model', 'linear_velocity'),
            gain=self.config.get('pitch_compensation_gain', 0.05),
            smoothing_alpha=self.config.get('pitch_smoothing_alpha', 0.7),
            data_max_age=self.config.get('pitch_data_max_age', 0.5),
            min_velocity=self.config.get('pitch_min_velocity', 1.0),
            deadband=self.config.get('pitch_deadband', 2.0),
            max_angle=self.config.get('pitch_max_angle', 45.0),
            max_correction=self.config.get('pitch_max_correction', 0.3),
        )

        # ── Target Loss Handler ──────────────────────────────────────────
        self.target_loss = TargetLossHandler(
            coord_threshold=self.config.get('target_loss_coord_threshold', 1.5),
            loss_timeout=self.config.get('target_loss_timeout', 3.0),
        )

        # ── State ────────────────────────────────────────────────────────
        self.emergency_stop_active = False
        self.altitude_violation_count = 0
        self.last_altitude_check_time = time.time()
        self.altitude_check_interval = self.config.get('altitude_check_interval', 1.0)

        # ── Velocity Smoothing ───────────────────────────────────────────
        self.velocity_smoothing_enabled = self.config.get('command_smoothing_enabled', True)
        self.smoothing_factor = self.config.get('smoothing_factor', 0.3)
        self.smoothed_right_velocity = 0.0
        self.smoothed_down_velocity = 0.0

        # ── Counters ─────────────────────────────────────────────────────
        self.update_count = 0

        logger.info(
            "[FollowerCore] Initialized: mode=%s, fwd_max=%.1f m/s, "
            "yaw_max=%.0f deg/s, adaptive=%s, pitch_comp=%s",
            self.lateral_guidance_mode,
            self.ramper.max_velocity,
            math.degrees(self.safety.rate_limits.yaw),
            self.adaptive.enabled,
            self.pitch_comp.enabled,
        )

    def _init_pid_controllers(self):
        """Initialize PID controllers based on guidance mode."""
        # Default setpoint: center of frame = (0, 0) in normalized coords
        setpoint_x = 0.0
        setpoint_y = 0.0

        # Yaw PID (for coordinated_turn mode) — rad/s internally
        yaw_gains = self.config.get('pid_yaw', {'kp': 2.0, 'ki': 0.05, 'kd': 0.1})
        self.pid_yaw = CustomPID(
            kp=yaw_gains.get('kp', 2.0),
            ki=yaw_gains.get('ki', 0.05),
            kd=yaw_gains.get('kd', 0.1),
            setpoint=setpoint_x,
            output_limits=(-self.safety.rate_limits.yaw, self.safety.rate_limits.yaw),
            proportional_on_measurement=self.config.get(
                'pid_proportional_on_measurement', False,
            ),
            enable_anti_windup=True,
        )

        # Right velocity PID (for sideslip mode)
        right_gains = self.config.get('pid_right', {'kp': 1.5, 'ki': 0.02, 'kd': 0.05})
        self.pid_right = CustomPID(
            kp=right_gains.get('kp', 1.5),
            ki=right_gains.get('ki', 0.02),
            kd=right_gains.get('kd', 0.05),
            setpoint=setpoint_x,
            output_limits=(-self.safety.velocity_limits.lateral,
                           self.safety.velocity_limits.lateral),
            proportional_on_measurement=self.config.get(
                'pid_proportional_on_measurement', False,
            ),
            enable_anti_windup=True,
        )

        # Down velocity PID (vertical tracking)
        down_gains = self.config.get('pid_down', {'kp': 1.0, 'ki': 0.03, 'kd': 0.05})
        self.pid_down = CustomPID(
            kp=down_gains.get('kp', 1.0),
            ki=down_gains.get('ki', 0.03),
            kd=down_gains.get('kd', 0.05),
            setpoint=setpoint_y,
            output_limits=(-self.safety.velocity_limits.vertical,
                           self.safety.velocity_limits.vertical),
            proportional_on_measurement=self.config.get(
                'pid_proportional_on_measurement', False,
            ),
            enable_anti_windup=True,
        )

        self.active_lateral_mode = self.lateral_guidance_mode

        logger.info(
            "[FollowerCore] PIDs initialized: yaw=%s, right=%s, down=%s",
            yaw_gains, right_gains, down_gains,
        )

    # ── Main Computation ─────────────────────────────────────────────────

    def compute(
        self,
        error_x: float,
        error_y: float,
        dt: float,
        target_confidence: float = 1.0,
        error_valid: bool = True,
        timestamp: Optional[float] = None,
    ) -> FollowerResult:
        """
        Main follower computation entry point.

        Computes body-velocity + yaw-rate commands from normalized errors
        produced by the Tracker package.

        Args:
            error_x: Normalized horizontal error [-1, 1] (Tracker → Follower)
            error_y: Normalized vertical error [-1, 1]
            dt: Time delta since last update (seconds)
            target_confidence: Tracker confidence [0, 1]
            error_valid: Whether the error data is valid
            timestamp: Current timestamp

        Returns:
            FollowerResult with control commands and diagnostics
        """
        if timestamp is None:
            timestamp = time.time()

        self.update_count += 1

        result = FollowerResult(timestamp=timestamp)
        result.update_count = self.update_count

        # ── 1. Target Loss Detection ─────────────────────────────────────
        target_valid = self.target_loss.update(
            (error_x, error_y), is_valid=error_valid, timestamp=timestamp,
        )
        result.target_lost = self.target_loss.target_lost
        result.target_loss_duration = self.target_loss.get_loss_duration()
        result.target_visible = target_valid

        # ── 2. Altitude Safety Check ─────────────────────────────────────
        if timestamp - self.last_altitude_check_time >= self.altitude_check_interval:
            self.last_altitude_check_time = timestamp
            telem = self.px4.telemetry
            alt = telem.altitude_rel if telem.altitude_valid else 0.0
            result.altitude_current = alt
            is_safe, reason = self.safety.check_altitude(alt)
            result.altitude_safe = is_safe
            if not is_safe:
                self.altitude_violation_count += 1
                logger.warning("[FollowerCore] Altitude safety violation: %s", reason)

        # ── 3. Forward Velocity Ramp ─────────────────────────────────────
        forward_velocity = self.ramper.update(
            dt=dt,
            target_lost=self.target_loss.target_lost,
            emergency_stop=self.emergency_stop_active,
            ramp_down_enabled=self.config.get('ramp_down_on_target_loss', True),
        )
        result.current_forward_velocity = forward_velocity

        # ── 4. Auto Mode Switching ───────────────────────────────────────
        if self.enable_auto_mode_switching:
            self._auto_switch_mode(forward_velocity, timestamp)

        # ── 5. Pitch Compensation ────────────────────────────────────────
        telem = self.px4.telemetry
        if self.pitch_comp.enabled and telem.attitude_valid:
            self.pitch_comp.update_pitch(telem.pitch_deg, timestamp)
            pitch_correction = self.pitch_comp.compute_compensation(forward_velocity)
            error_y_corrected = error_y - pitch_correction
            result.pitch_compensation_active = self.pitch_comp.active
            result.pitch_compensation_value = pitch_correction
        else:
            error_y_corrected = error_y

        # ── 6. PID Computation ───────────────────────────────────────────
        # PID commands use direct tracker measurements only. Predicted or
        # stale points remain available for diagnostics but cannot steer flight.
        if target_valid and self.active_lateral_mode == 'coordinated_turn':
            yaw_rate_rad_s = self.pid_yaw(-error_x, dt)
            result.pid_yaw_output = yaw_rate_rad_s
            result.yaw_rate_raw_deg_s = math.degrees(yaw_rate_rad_s)
            yaw_rate_deg_s = self.yaw_smoother.apply(
                result.yaw_rate_raw_deg_s, dt, forward_velocity,
            )
            result.yaw_smoothing_active = self.yaw_smoother.enabled
            right_velocity = 0.0
            result.pid_right_output = 0.0
        elif target_valid:
            right_velocity_raw = self.pid_right(-error_x, dt)
            result.pid_right_output = right_velocity_raw
            result.yaw_rate_raw_deg_s = 0.0
            if self.velocity_smoothing_enabled:
                self.smoothed_right_velocity = (
                    self.smoothing_factor * self.smoothed_right_velocity +
                    (1.0 - self.smoothing_factor) * right_velocity_raw
                )
                right_velocity = self.smoothed_right_velocity
            else:
                right_velocity = right_velocity_raw
            yaw_rate_deg_s = 0.0
            result.pid_yaw_output = 0.0
        else:
            self.pid_yaw.reset()
            self.pid_right.reset()
            self.pid_down.reset()
            self.yaw_smoother.reset()
            self.smoothed_right_velocity = 0.0
            self.smoothed_down_velocity = 0.0
            right_velocity = 0.0
            yaw_rate_deg_s = 0.0
            result.pid_yaw_output = 0.0
            result.pid_right_output = 0.0

        if target_valid:
            down_velocity_raw = self.pid_down(-error_y_corrected, dt)
            result.pid_down_output = down_velocity_raw
            if self.velocity_smoothing_enabled:
                self.smoothed_down_velocity = (
                    self.smoothing_factor * self.smoothed_down_velocity +
                    (1.0 - self.smoothing_factor) * down_velocity_raw
                )
                down_velocity = self.smoothed_down_velocity
            else:
                down_velocity = down_velocity_raw
        else:
            down_velocity = 0.0
            result.pid_down_output = 0.0

        # ?? 7. Adaptive Dive/Climb ???????????????????????????????????????
        if target_valid and self.adaptive.enabled:
            forward_velocity, down_velocity = self.adaptive.update(
                target_y=error_y,
                target_confidence=target_confidence,
                current_v_down=down_velocity,
                current_v_fwd=forward_velocity,
                lateral_mode=self.active_lateral_mode,
                timestamp=timestamp,
            )
            result.adaptive_active = self.adaptive.active
            result.adaptive_correction_down = self.adaptive.correction_down
            result.adaptive_correction_fwd = self.adaptive.correction_fwd

        # ── 8. Safety Clamping ───────────────────────────────────────────
        forward_velocity, _ = self.safety.clamp_velocity_forward(forward_velocity)
        right_velocity, _ = self.safety.clamp_velocity_lateral(right_velocity)
        down_velocity, _ = self.safety.clamp_velocity_vertical(down_velocity)
        yaw_rate_deg_s_clamped, _ = self.safety.clamp_yaw_rate(
            math.radians(yaw_rate_deg_s)
        )
        yaw_rate_deg_s = math.degrees(yaw_rate_deg_s_clamped)

        # Magnitude check
        forward_velocity, right_velocity, down_velocity = \
            self.safety.clamp_command_magnitude(
                forward_velocity, right_velocity, down_velocity,
            )

        # ── 9. Feasibility Limiter (ego-planner inspired) ────────────────
        # Apply velocity/acceleration/jerk feasibility constraints
        forward_velocity, right_velocity, down_velocity = \
            self.feasibility.limit(
                forward_velocity, right_velocity, down_velocity,
                timestamp=timestamp,
            )
        forward_velocity, _ = self.safety.clamp_velocity_forward(forward_velocity)
        right_velocity, _ = self.safety.clamp_velocity_lateral(right_velocity)
        down_velocity, _ = self.safety.clamp_velocity_vertical(down_velocity)
        down_velocity = self._enforce_altitude_envelope(down_velocity)

        # ── 10. Emergency Stop ────────────────────────────────────────────
        if self.emergency_stop_active:
            forward_velocity = 0.0
            right_velocity = 0.0
            down_velocity = 0.0
            yaw_rate_deg_s = 0.0

        # ── 11. Populate Result ──────────────────────────────────────────
        result.velocity_forward = forward_velocity
        result.velocity_right = right_velocity
        result.velocity_down = down_velocity
        result.yaw_rate_deg_s = yaw_rate_deg_s

        result.command_valid = target_valid and not self.emergency_stop_active
        result.target_error_x = error_x
        result.target_error_y = error_y
        result.emergency_stop_active = self.emergency_stop_active

        result.control_mode = "velocity_body"
        result.lateral_guidance_mode = self.active_lateral_mode

        # ── 11. Controller Port ──────────────────────────────────────────
        result.suggested_velocity_forward = forward_velocity
        result.suggested_velocity_right = right_velocity
        result.suggested_velocity_down = down_velocity
        result.suggested_yaw_rate_deg_s = yaw_rate_deg_s
        result.suggestion_valid = result.command_valid

        return result

    def _enforce_altitude_envelope(self, velocity_down: float) -> float:
        """Block commands that would move farther beyond a known altitude limit."""
        telem = self.px4.telemetry
        if not telem.altitude_valid:
            return velocity_down
        altitude = telem.altitude_rel
        limits = self.safety.altitude_limits
        if altitude <= limits.min_altitude:
            return min(0.0, velocity_down)
        if altitude >= limits.max_altitude:
            return max(0.0, velocity_down)
        return velocity_down

    def _auto_switch_mode(self, forward_velocity: float, timestamp: float):
        """Automatic lateral guidance mode switching based on velocity."""
        current_time = timestamp

        if current_time - self.last_mode_switch_time < self.min_mode_switch_interval:
            return

        if self.active_lateral_mode == 'sideslip':
            if forward_velocity >= self.mode_switch_velocity + self.mode_switch_hysteresis:
                self._switch_mode('coordinated_turn', current_time)
        else:
            if forward_velocity <= self.mode_switch_velocity - self.mode_switch_hysteresis:
                self._switch_mode('sideslip', current_time)

    def _switch_mode(self, new_mode: str, timestamp: float):
        """Switch lateral guidance mode."""
        if new_mode == self.active_lateral_mode:
            return

        logger.info(
            "[FollowerCore] Mode switch: %s → %s (v_fwd=%.2f m/s)",
            self.active_lateral_mode, new_mode,
            self.ramper.get_velocity(),
        )

        self.active_lateral_mode = new_mode
        self.last_mode_switch_time = timestamp

        # Reset controllers
        self.pid_yaw.reset()
        self.pid_right.reset()
        self.yaw_smoother.reset()
        self.smoothed_right_velocity = 0.0

    # ── Control Methods ──────────────────────────────────────────────────

    def set_emergency_stop(self, active: bool):
        """Activate or deactivate emergency stop."""
        self.emergency_stop_active = active
        if active:
            logger.warning("[FollowerCore] EMERGENCY STOP ACTIVATED")
            self.pid_yaw.reset()
            self.pid_right.reset()
            self.pid_down.reset()
            self.yaw_smoother.reset()
            self.ramper.reset(0.0)
        else:
            logger.info("[FollowerCore] Emergency stop released")

    def set_lateral_mode(self, mode: str):
        """Set lateral guidance mode."""
        if mode in ('coordinated_turn', 'sideslip'):
            self._switch_mode(mode, time.time())
        else:
            logger.warning("[FollowerCore] Unknown mode: %s", mode)

    def reset(self):
        """Reset all controllers and state."""
        self.pid_yaw.reset()
        self.pid_right.reset()
        self.pid_down.reset()
        self.yaw_smoother.reset()
        self.ramper.reset(0.0)
        self.adaptive.reset()
        self.pitch_comp.reset()
        self.target_loss.reset()
        self.feasibility.reset()
        self.emergency_stop_active = False
        self.altitude_violation_count = 0
        self.update_count = 0
        self.smoothed_right_velocity = 0.0
        self.smoothed_down_velocity = 0.0
        logger.info("[FollowerCore] Full reset complete.")

    def get_status(self) -> Dict[str, Any]:
        """Get comprehensive status dictionary."""
        return {
            'mode': self.active_lateral_mode,
            'forward_velocity': self.ramper.get_velocity(),
            'target_lost': self.target_loss.target_lost,
            'loss_duration': self.target_loss.get_loss_duration(),
            'emergency_stop': self.emergency_stop_active,
            'adaptive_active': self.adaptive.active,
            'pitch_comp_active': self.pitch_comp.active,
            'altitude_violations': self.altitude_violation_count,
            'update_count': self.update_count,
        }
