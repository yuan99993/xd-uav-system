"""Fail-closed capture clock with explicit, confirmed camera restart segments."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ClockDecision:
    accepted: bool
    new_segment: bool
    reason: str


class CaptureClock:
    def __init__(self, maximum_age=2., future_tolerance=.02):
        self.maximum_age = float(maximum_age)
        self.future_tolerance = float(future_tolerance)
        if not all(math.isfinite(v) and v >= 0 for v in (self.maximum_age, self.future_tolerance)):
            raise ValueError("Invalid capture clock limits")
        self.last_stamp = 0.
        self.last_seq = 0
        self.last_contract = None
        self.last_ros_now = 0.
        self.pending_ros_reset = False
        self.pending_restart = None
        self.segment = 0

    def observe(self, stamp, seq, frame_id, shape, ros_now):
        if math.isfinite(ros_now) and ros_now > 0:
            if self.last_ros_now > 0 and ros_now < self.last_ros_now-.1:
                self.pending_ros_reset = True
            self.last_ros_now = ros_now
        if not math.isfinite(stamp) or stamp <= 0:
            return ClockDecision(False, False, "capture timestamp is zero or invalid")
        if ros_now > 0:
            age = ros_now-stamp
            if age < -self.future_tolerance:
                return ClockDecision(False, False, "capture timestamp is in the future")
            if self.maximum_age and age > self.maximum_age:
                return ClockDecision(False, False, "capture is stale; camera must use the ROS time domain")
        contract = (str(frame_id), tuple(shape))
        new_segment = self.pending_ros_reset or (
            self.last_contract is not None and contract != self.last_contract)
        reason = "clock_or_camera_contract_restart" if new_segment else "capture"
        # A seq reset alone needs two fresh, increasing captures. This prevents
        # one reordered packet from erasing a healthy tracking session.
        sequence_restart = self.last_seq >= 32 and 0 <= seq < min(self.last_seq, 16)
        if sequence_restart and not new_segment:
            pending = self.pending_restart
            if pending is None or stamp <= pending[0] or seq <= pending[1]:
                self.pending_restart = (stamp, seq)
                return ClockDecision(False, False, "confirming camera sequence restart")
            new_segment, reason = True, "confirmed_camera_sequence_restart"
        if not new_segment and stamp <= self.last_stamp:
            self.pending_restart = None
            return ClockDecision(False, False, "capture timestamp is duplicate or out of order")
        if new_segment:
            self.segment += 1
        self.last_stamp, self.last_seq, self.last_contract = stamp, int(seq), contract
        self.pending_ros_reset = False
        self.pending_restart = None
        return ClockDecision(True, new_segment, reason)
