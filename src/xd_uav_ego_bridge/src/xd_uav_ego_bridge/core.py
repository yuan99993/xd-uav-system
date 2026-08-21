"""ROS-independent validation and mapping used by the EGO bridge node."""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class StateSample:
    stamp: float
    frame_id: str
    body_frame_id: str
    position: Vector3
    velocity_world: Vector3
    orientation: Quaternion
    body_rate: Vector3
    state_valid: bool
    localization_valid: bool
    odometry_fresh: bool


@dataclass(frozen=True)
class CommandSample:
    stamp: float
    frame_id: str
    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    yaw: float
    yaw_rate: float
    trajectory_flag: int


@dataclass(frozen=True)
class Validation:
    valid: bool
    reason: str


@dataclass(frozen=True)
class PointCloudSample:
    stamp: float
    frame_id: str
    width: int
    height: int
    point_step: int
    data_size: int


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _valid_age(stamp: float, now: float, timeout: float,
               future_tolerance: float) -> Validation:
    if not _finite((stamp, now, timeout, future_tolerance)):
        return Validation(False, "time_not_finite")
    if stamp <= 0.0:
        return Validation(False, "stamp_not_positive")
    age = now - stamp
    if age < -future_tolerance:
        return Validation(False, "stamp_in_future")
    if age > timeout:
        return Validation(False, "input_stale")
    return Validation(True, "ok")


def validate_state(sample: StateSample, now: float, expected_frame: str,
                   expected_body_frame: str, timeout: float,
                   future_tolerance: float) -> Validation:
    if not sample.state_valid:
        return Validation(False, "state_invalid")
    if not sample.localization_valid:
        return Validation(False, "localization_invalid")
    if not sample.odometry_fresh:
        return Validation(False, "odometry_not_fresh")
    if sample.frame_id != expected_frame:
        return Validation(False, "state_frame_mismatch")
    if expected_body_frame and sample.body_frame_id != expected_body_frame:
        return Validation(False, "body_frame_mismatch")
    if not sample.body_frame_id:
        return Validation(False, "body_frame_empty")
    values = (sample.position + sample.velocity_world + sample.orientation
              + sample.body_rate)
    if not _finite(values):
        return Validation(False, "state_not_finite")
    norm = math.sqrt(sum(value * value for value in sample.orientation))
    if norm < 1.0e-6 or abs(norm - 1.0) > 1.0e-3:
        return Validation(False, "orientation_not_normalized")
    return _valid_age(sample.stamp, now, timeout, future_tolerance)


def validate_command(sample: CommandSample, now: float, expected_frame: str,
                     ready_flag: int, timeout: float,
                     future_tolerance: float,
                     allow_nonfinite_yaw_rate: bool = False) -> Validation:
    if sample.trajectory_flag != ready_flag:
        return Validation(False, "trajectory_not_ready")
    if sample.frame_id != expected_frame:
        return Validation(False, "command_frame_mismatch")
    values = (sample.position + sample.velocity + sample.acceleration
              + (sample.yaw,))
    if not allow_nonfinite_yaw_rate:
        values += (sample.yaw_rate,)
    if not _finite(values):
        return Validation(False, "command_not_finite")
    return _valid_age(sample.stamp, now, timeout, future_tolerance)


def validate_pointcloud(sample: PointCloudSample, now: float,
                        expected_input_frame: str, timeout: float,
                        future_tolerance: float) -> Validation:
    if not sample.frame_id:
        return Validation(False, "cloud_frame_empty")
    if expected_input_frame and sample.frame_id != expected_input_frame:
        return Validation(False, "cloud_frame_mismatch")
    if sample.width <= 0 or sample.height <= 0:
        return Validation(False, "cloud_empty")
    if sample.point_step <= 0:
        return Validation(False, "cloud_layout_invalid")
    required_size = sample.width * sample.height * sample.point_step
    if sample.data_size < required_size:
        return Validation(False, "cloud_data_truncated")
    age = _valid_age(sample.stamp, now, timeout, future_tolerance)
    if not age.valid:
        return Validation(False, "cloud_" + age.reason)
    return Validation(True, "ok")
