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


@dataclass(frozen=True)
class TrackMonitorPolicy:
    required_tracking_sec: float = 30.0
    acquisition_timeout_sec: float = 10.0
    target_loss_timeout_sec: float = 3.0
    status_timeout_sec: float = 1.0
    maximum_duration_sec: float = 120.0
    minimum_tracking_quality: float = 0.0
    allow_predicted: bool = False

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
        command_valid: bool,
        state_valid: bool,
        emergency_stop_active: bool,
        tracking_state: str,
        tracking_quality: float,
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
            return self._fail(ERROR_TRACKER_STOPPED, "tracker stopped unexpectedly")

        measured_target = bool(target_visible) and (
            self.policy.allow_predicted or not bool(target_predicted)
        )
        valid = bool(
            measured_target
            and command_valid
            and state_valid
            and str(tracking_state) == "tracking"
            and float(tracking_quality) >= self.policy.minimum_tracking_quality
        )
        if valid:
            if self.valid_started_at is None:
                self.valid_started_at = stamp
            self.last_valid_at = stamp
            self.acquired = True
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
            self.detail = str(invalid_reason).strip() or (
                f"waiting for valid tracking (state={tracking_state})"
            )
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
                return self._fail(
                    ERROR_ACQUISITION_TIMEOUT,
                    "no valid target was acquired before the deadline",
                )
            return self.state
        if (
            self.last_valid_at is not None
            and stamp - self.last_valid_at > self.policy.target_loss_timeout_sec
        ):
            return self._fail(ERROR_TARGET_LOST, "tracked target was lost for too long")
        return self.state

    def _fail(self, error: str, detail: str) -> str:
        self.state = STATE_FAILED
        self.error = str(error)
        self.detail = str(detail)
        self.valid_started_at = None
        self.progress = 0.0
        return self.state
