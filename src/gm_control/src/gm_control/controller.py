import time
from dataclasses import dataclass
from typing import Optional, Tuple

from gm_control.adapters.base import GimbalCommandData, GimbalStateData


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class TargetBox:
    valid: bool
    x: float
    y: float
    width: float
    height: float
    confidence: float = 1.0

    @property
    def center(self) -> Tuple[float, float]:
        return self.x + self.width * 0.5, self.y + self.height * 0.5


@dataclass
class ImageSize:
    width: int
    height: int


@dataclass
class ControllerConfig:
    control_mode: str = "rate"
    yaw_kp: float = 35.0
    yaw_ki: float = 0.0
    yaw_kd: float = 0.0
    pitch_kp: float = 25.0
    pitch_ki: float = 0.0
    pitch_kd: float = 0.0
    yaw_integral_limit: float = 1.0
    pitch_integral_limit: float = 1.0
    yaw_sign: float = 1.0
    pitch_sign: float = 1.0
    max_yaw_rate_deg_s: float = 80.0
    max_pitch_rate_deg_s: float = 60.0
    # Deprecated compatibility parameters. Angle commands are intentionally
    # not clamped by gm_control.
    max_yaw_angle_deg: float = 45.0
    max_pitch_angle_deg: float = 30.0
    deadzone_x: float = 0.02
    deadzone_y: float = 0.02
    target_timeout_s: float = 0.3
    target_lost_action: str = "stop"
    target_lost_hold_time_s: float = 0.2
    target_lost_search_yaw_rate_deg_s: float = 15.0
    target_lost_search_pitch_rate_deg_s: float = 0.0
    target_lost_back_to_init_yaw_deg: float = 0.0
    target_lost_back_to_init_pitch_deg: float = 0.0
    target_lost_back_to_init_roll_deg: float = 0.0
    min_confidence: float = 0.0
    smoothing_enabled: bool = False
    smoothing_alpha: float = 0.3
    max_rate_change_deg_s2: float = 200.0


class ImageGimbalController:
    """Convert image target-center error into gimbal rate commands."""

    def __init__(self, config: Optional[ControllerConfig] = None):
        self.config = config or ControllerConfig()
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_time = None
        self.last_target_time = 0.0
        self.target_seen_once = False
        self.manual_search_enabled = False
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.last_command = GimbalCommandData(mode=self.config.control_mode, valid=False)

    def reset(self) -> None:
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_time = None
        self.last_target_time = 0.0
        self.target_seen_once = False
        self.manual_search_enabled = False
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.last_command = GimbalCommandData(mode=self.config.control_mode, valid=False)

    def set_manual_search(self, enabled: bool) -> None:
        """Enable an explicit search request without fabricating a target loss."""
        self.manual_search_enabled = bool(enabled)
        self.last_time = None
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

    def normalized_error(self, target: TargetBox, image_size: ImageSize) -> Tuple[float, float]:
        if image_size.width <= 0 or image_size.height <= 0:
            return 0.0, 0.0

        cx, cy = target.center
        error_x = (cx - image_size.width * 0.5) / (image_size.width * 0.5)
        error_y = (cy - image_size.height * 0.5) / (image_size.height * 0.5)
        return _clamp(error_x, -1.0, 1.0), _clamp(error_y, -1.0, 1.0)

    def update(
        self,
        target: Optional[TargetBox],
        image_size: Optional[ImageSize],
        gimbal_state: Optional[GimbalStateData] = None,
        now: Optional[float] = None,
    ):
        now = time.time() if now is None else now
        cfg = self.config

        has_target = (
            target is not None
            and target.valid
            and target.confidence >= cfg.min_confidence
            and image_size is not None
            and image_size.width > 0
            and image_size.height > 0
        )

        if has_target:
            self.target_seen_once = True
            self.last_target_time = now
        else:
            lost_time = now - self.last_target_time

            # A manual search is an explicit operator request and may start
            # before the first target has ever been detected.
            if self.manual_search_enabled:
                command = self._search_command()
                self.last_command = command
                return command, (0.0, 0.0)

            # Merely enabling tracking must not make a never-seen target look
            # lost. Keep the adapter at its current angle until a target has
            # actually been acquired once.
            if not self.target_seen_once:
                return GimbalCommandData(mode=cfg.control_mode, valid=False), (0.0, 0.0)

            # Ignore short detector gaps. The adapter interprets an invalid
            # command as hold, so this does not cause a spurious movement.
            if lost_time <= cfg.target_timeout_s:
                if (
                    cfg.target_lost_action == "hold_last"
                    and lost_time <= cfg.target_lost_hold_time_s
                    and self.last_command.valid
                ):
                    return self.last_command, (0.0, 0.0)
                return GimbalCommandData(mode=cfg.control_mode, valid=False), (0.0, 0.0)

            self.last_time = now
            self.last_error_x = 0.0
            self.last_error_y = 0.0
            self.integral_x = 0.0
            self.integral_y = 0.0
            command = self._target_lost_command(now)
            if command.valid:
                self.last_command = command
            return command, (0.0, 0.0)

        error_x, error_y = self.normalized_error(target, image_size)

        if abs(error_x) < cfg.deadzone_x:
            error_x = 0.0
        if abs(error_y) < cfg.deadzone_y:
            error_y = 0.0

        dt = 0.0 if self.last_time is None else max(1e-3, now - self.last_time)
        dx = 0.0 if dt == 0.0 else (error_x - self.last_error_x) / dt
        dy = 0.0 if dt == 0.0 else (error_y - self.last_error_y) / dt

        if dt > 0.0:
            self.integral_x = _clamp(
                self.integral_x + error_x * dt,
                -cfg.yaw_integral_limit,
                cfg.yaw_integral_limit,
            )
            self.integral_y = _clamp(
                self.integral_y + error_y * dt,
                -cfg.pitch_integral_limit,
                cfg.pitch_integral_limit,
            )

        yaw_raw = cfg.yaw_sign * (
            cfg.yaw_kp * error_x + cfg.yaw_ki * self.integral_x + cfg.yaw_kd * dx
        )
        pitch_raw = cfg.pitch_sign * (
            cfg.pitch_kp * error_y + cfg.pitch_ki * self.integral_y + cfg.pitch_kd * dy
        )

        self.last_time = now
        self.last_error_x = error_x
        self.last_error_y = error_y

        if cfg.control_mode == "angle":
            current_yaw = gimbal_state.yaw_deg if gimbal_state is not None and gimbal_state.valid else 0.0
            current_pitch = gimbal_state.pitch_deg if gimbal_state is not None and gimbal_state.valid else 0.0
            command = GimbalCommandData(
                mode="angle",
                valid=True,
                yaw_deg=current_yaw + yaw_raw,
                pitch_deg=current_pitch + pitch_raw,
            )
        else:
            yaw_rate = _clamp(yaw_raw, -cfg.max_yaw_rate_deg_s, cfg.max_yaw_rate_deg_s)
            pitch_rate = _clamp(pitch_raw, -cfg.max_pitch_rate_deg_s, cfg.max_pitch_rate_deg_s)
            command = GimbalCommandData(
                mode="rate",
                valid=True,
                yaw_rate_deg_s=yaw_rate,
                pitch_rate_deg_s=pitch_rate,
            )

        command = self._smooth_command(command, dt)
        self.last_command = command
        return command, (error_x, error_y)

    def _target_lost_command(self, now: float) -> GimbalCommandData:
        action = self.config.target_lost_action
        lost_time = now - self.last_target_time

        if action == "hold_last" and lost_time <= self.config.target_lost_hold_time_s:
            return self.last_command

        if action == "search":
            return self._search_command()

        if action == "back_to_init":
            return GimbalCommandData(
                mode="angle",
                valid=True,
                yaw_deg=self.config.target_lost_back_to_init_yaw_deg,
                pitch_deg=self.config.target_lost_back_to_init_pitch_deg,
                roll_deg=self.config.target_lost_back_to_init_roll_deg,
            )

        return GimbalCommandData(mode=self.config.control_mode, valid=False)

    def _search_command(self) -> GimbalCommandData:
        return GimbalCommandData(
            mode="rate",
            valid=True,
            yaw_rate_deg_s=self.config.target_lost_search_yaw_rate_deg_s,
            pitch_rate_deg_s=self.config.target_lost_search_pitch_rate_deg_s,
        )

    def _smooth_command(self, command: GimbalCommandData, dt: float) -> GimbalCommandData:
        cfg = self.config
        if not cfg.smoothing_enabled or dt <= 0.0 or not command.valid or not self.last_command.valid:
            return command

        alpha = _clamp(cfg.smoothing_alpha, 0.0, 1.0)
        max_delta = max(0.0, cfg.max_rate_change_deg_s2) * dt

        if command.mode == "angle":
            yaw = self._smooth_scalar(command.yaw_deg, self.last_command.yaw_deg, alpha, max_delta)
            pitch = self._smooth_scalar(command.pitch_deg, self.last_command.pitch_deg, alpha, max_delta)
            command.yaw_deg = yaw
            command.pitch_deg = pitch
        else:
            yaw_rate = self._smooth_scalar(
                command.yaw_rate_deg_s,
                self.last_command.yaw_rate_deg_s,
                alpha,
                max_delta,
            )
            pitch_rate = self._smooth_scalar(
                command.pitch_rate_deg_s,
                self.last_command.pitch_rate_deg_s,
                alpha,
                max_delta,
            )
            command.yaw_rate_deg_s = yaw_rate
            command.pitch_rate_deg_s = pitch_rate

        return command

    @staticmethod
    def _smooth_scalar(target: float, previous: float, alpha: float, max_delta: float) -> float:
        filtered = alpha * target + (1.0 - alpha) * previous
        delta = _clamp(filtered - previous, -max_delta, max_delta)
        return previous + delta
