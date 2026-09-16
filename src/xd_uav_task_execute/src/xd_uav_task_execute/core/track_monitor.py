"""ROS-independent completion monitor for an xd_uav_track action."""

from dataclasses import dataclass


STATE_ACQUIRING = "acquiring"
STATE_RUNNING = "running"
STATE_SUCCEEDED = "succeeded"
STATE_FAILED = "failed"

ERROR_NONE = ""
ERROR_ACQUISITION_TIMEOUT = "acquisition_timeout"
ERROR_TARGET_LOST = "target_lost"
ERROR_STATUS_TIMEOUT = "status_timeout"
ERROR_EXECUTION_TIMEOUT = "execution_timeout"
ERROR_TRACKER_STOPPED = "tracker_stopped"
ERROR_EMERGENCY_STOP = "emergency_stop"
ERROR_TARGET_MISMATCH = "target_mismatch"
ERROR_PROFILE_MISMATCH = "profile_mismatch"
ERROR_CONTROL_UNAVAILABLE = "control_unavailable"
ERROR_GIMBAL_UNAVAILABLE = "gimbal_unavailable"


@dataclass(frozen=True)
class TrackMonitorPolicy:
    required_tracking_sec: float = 30.0
    acquisition_timeout_sec: float = 10.0
    target_loss_timeout_sec: float = 3.0
    status_timeout_sec: float = 1.0
    maximum_duration_sec: float = 120.0
    minimum_tracking_quality: float = 0.0
    allow_predicted: bool = False
    required_track_id: int = -1
    required_profile: str = ""
    require_control_reference: bool = True

    def __post_init__(self):
        for name in (
            "required_tracking_sec",
            "acquisition_timeout_sec",
            "target_loss_timeout_sec",
            "status_timeout_sec",
            "maximum_duration_sec",
        ):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.maximum_duration_sec < self.required_tracking_sec:
            raise ValueError(
                "maximum_duration_sec must be at least required_tracking_sec"
            )
        if not 0.0 <= float(self.minimum_tracking_quality) <= 1.0:
            raise ValueError("minimum_tracking_quality must be in [0, 1]")


class TrackMonitor:
    """Require one continuous interval of valid, visible target tracking."""

    def __init__(self, started_at: float, policy: TrackMonitorPolicy):
        self.started_at = float(started_at)
        self.policy = policy
        self.state = STATE_ACQUIRING
        self.error = ERROR_NONE
        self.detail = "waiting for a valid tracked target"
        self.last_status_at = None
        self.last_valid_at = None
        self.valid_started_at = None
        self.acquired = False
        self.progress = 0.0
        self.locked_track_id = None
        self.tracker_seen_active = False
        self.pending_error = ERROR_NONE
        self.pending_detail = ""

    @property
    def terminal(self) -> bool:
        return self.state in (STATE_SUCCEEDED, STATE_FAILED)

    def observe(
        self,
        now: float,
        *,
        tracker_active: bool,
        target_visible: bool,
        target_predicted: bool,
        metric_target_valid: bool = False,
        metric_active: bool = False,
        orbit_active: bool = False,
        command_valid: bool,
        state_valid: bool,
        emergency_stop_active: bool,
        tracking_state: str,
        tracking_quality: float,
        track_id: int = 0,
        follower_profile: str = "",
        requested_profile: str = "",
        control_reference_published: bool = True,
        gimbal_state_valid: bool = True,
        gimbal_fallback_active: bool = False,
        invalid_reason: str = "",
    ) -> str:
        if self.terminal:
            return self.state
        stamp = float(now)
        if self.last_status_at is not None and stamp < self.last_status_at:
            return self.state
        self.last_status_at = stamp
        if emergency_stop_active:
            return self._fail(ERROR_EMERGENCY_STOP, "tracker emergency stop is active")
        if not tracker_active:
            if self.tracker_seen_active:
                return self._fail(
                    ERROR_TRACKER_STOPPED, "tracker stopped unexpectedly"
                )
            # A queued status published just before StartTracker(true) can be
            # delivered after the service response. Treat it as startup state;
            # a tracker that never reports active still fails at acquisition
            # timeout with the more useful stopped reason.
            self.pending_error = ERROR_TRACKER_STOPPED
            self.pending_detail = "waiting for tracker active status"
            self.detail = self.pending_detail
            return self.poll(stamp)
        self.tracker_seen_active = True

        expected_profile = (
            str(self.policy.required_profile).strip()
            or str(requested_profile).strip()
        )
        metric_profile = expected_profile.startswith("fw_metric_")
        metric_state = str(tracking_state).strip() in (
            "pursuit", "orbit", "coast", "orbit_coast", "center_hold"
        )
        measured_target = (
            bool(metric_target_valid) and bool(metric_active) and metric_state
            if metric_profile
            else bool(target_visible)
            and (self.policy.allow_predicted or not bool(target_predicted))
        )
        profile_valid = bool(
            not expected_profile
            or (
                str(requested_profile).strip() == expected_profile
                and str(follower_profile).strip() == expected_profile
                and not bool(gimbal_fallback_active)
            )
        )
        expected_track_id = int(self.policy.required_track_id)
        if expected_track_id < 0 and self.locked_track_id is not None:
            expected_track_id = int(self.locked_track_id)
        track_valid = int(track_id) >= 0 and (
            expected_track_id < 0 or int(track_id) == expected_track_id
        )
        gimbal_required = expected_profile.startswith("gm_velocity_")
        gimbal_valid = bool(
            not gimbal_required
            or gimbal_state_valid
        )
        reference_valid = bool(
            not self.policy.require_control_reference
            or control_reference_published
        )
        tracking_state_valid = (
            metric_state if metric_profile else str(tracking_state) == "tracking"
        )
        valid = bool(
            measured_target
            and command_valid
            and state_valid
            and tracking_state_valid
            and float(tracking_quality) >= self.policy.minimum_tracking_quality
            and track_valid
            and profile_valid
            and gimbal_valid
            and reference_valid
        )
        if valid:
            if self.locked_track_id is None:
                self.locked_track_id = int(track_id)
            if self.valid_started_at is None:
                self.valid_started_at = stamp
            self.last_valid_at = stamp
            self.acquired = True
            self.pending_error = ERROR_NONE
            self.pending_detail = ""
            self.state = STATE_RUNNING
            elapsed = max(0.0, stamp - self.valid_started_at)
            self.progress = min(1.0, elapsed / self.policy.required_tracking_sec)
            self.detail = (
                f"continuous valid tracking {elapsed:.1f}/"
                f"{self.policy.required_tracking_sec:.1f}s"
            )
        else:
            self.valid_started_at = None
            self.progress = 0.0
            self.state = STATE_ACQUIRING
            if not profile_valid:
                self.pending_error = ERROR_PROFILE_MISMATCH
                self.pending_detail = (
                    f"requested profile {expected_profile or '<unset>'} is not active "
                    f"(requested={requested_profile or '<unset>'}, "
                    f"active={follower_profile or '<unset>'}, "
                    f"fallback={bool(gimbal_fallback_active)})"
                )
            elif not gimbal_valid:
                self.pending_error = ERROR_GIMBAL_UNAVAILABLE
                self.pending_detail = "gimbal tracking or gimbal state is unavailable"
            elif not track_valid:
                self.pending_error = ERROR_TARGET_MISMATCH
                self.pending_detail = (
                    f"active local track {int(track_id)} does not match "
                    f"required track {expected_track_id}"
                )
            elif not measured_target:
                self.pending_error = ERROR_TARGET_LOST
                self.pending_detail = str(invalid_reason).strip() or (
                    f"waiting for measured target tracking (state={tracking_state})"
                )
            elif not reference_valid or not command_valid or not state_valid:
                self.pending_error = ERROR_CONTROL_UNAVAILABLE
                self.pending_detail = str(invalid_reason).strip() or (
                    "track control reference is not active"
                )
            else:
                self.pending_error = ERROR_TARGET_LOST
                self.pending_detail = str(invalid_reason).strip() or (
                    f"waiting for measured target tracking (state={tracking_state})"
                )
            self.detail = self.pending_detail
        return self.poll(stamp)

    def poll(self, now: float) -> str:
        if self.terminal:
            return self.state
        stamp = float(now)
        if self.progress >= 1.0:
            self.state = STATE_SUCCEEDED
            self.error = ERROR_NONE
            self.detail = "required continuous tracking interval completed"
            return self.state
        if stamp - self.started_at > self.policy.maximum_duration_sec:
            return self._fail(ERROR_EXECUTION_TIMEOUT, "track action timed out")
        if self.last_status_at is None:
            if stamp - self.started_at > self.policy.status_timeout_sec:
                return self._fail(ERROR_STATUS_TIMEOUT, "no tracker status received")
            return self.state
        if stamp - self.last_status_at > self.policy.status_timeout_sec:
            return self._fail(ERROR_STATUS_TIMEOUT, "tracker status timed out")
        if not self.acquired:
            if stamp - self.started_at > self.policy.acquisition_timeout_sec:
                if self.pending_error not in (ERROR_NONE, ERROR_TARGET_LOST):
                    return self._fail(self.pending_error, self.pending_detail)
                return self._fail(
                    ERROR_ACQUISITION_TIMEOUT,
                    "no valid target was acquired before the deadline",
                )
            return self.state
        if (
            self.last_valid_at is not None
            and stamp - self.last_valid_at > self.policy.target_loss_timeout_sec
        ):
            return self._fail(
                self.pending_error or ERROR_TARGET_LOST,
                self.pending_detail or "tracked target was lost for too long",
            )
        return self.state

    def _fail(self, error: str, detail: str) -> str:
        self.state = STATE_FAILED
        self.error = str(error)
        self.detail = str(detail)
        self.valid_started_at = None
        self.progress = 0.0
        return self.state
