"""ROS-independent validation and mapping used by the EGO bridge node."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple


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


@dataclass(frozen=True)
class ReferenceSample:
    stamp: float
    frame_id: str
    coordinate_frame: int
    type_mask: int
    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    yaw: float
    yaw_rate: float


@dataclass(frozen=True)
class VehicleStateSample:
    stamp: float
    vehicle_type: int
    state_valid: bool
    localization_valid: bool
    odometry_fresh: bool


@dataclass(frozen=True)
class PathSample:
    stamp: float
    frame_id: str
    points: Tuple[Vector3, ...]
    pose_frames: Tuple[str, ...]


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


def validate_reference(sample: ReferenceSample, now: float,
                       expected_frame: str, timeout: float,
                       future_tolerance: float,
                       local_inertial_frame: int = 1,
                       force_bit: int = 512,
                       known_mask: int = 4095) -> Validation:
    if sample.frame_id != expected_frame:
        return Validation(False, "reference_frame_mismatch")
    if sample.coordinate_frame != local_inertial_frame:
        return Validation(False, "coordinate_frame_unsupported")
    if sample.type_mask < 0 or sample.type_mask & ~known_mask:
        return Validation(False, "reference_type_mask_unknown_bits")
    if sample.type_mask & force_bit:
        return Validation(False, "force_mode_unsupported")
    enabled_translation = []
    for bit, value in zip((1, 2, 4), sample.position):
        if not sample.type_mask & bit:
            enabled_translation.append(value)
    for bit, value in zip((8, 16, 32), sample.velocity):
        if not sample.type_mask & bit:
            enabled_translation.append(value)
    for bit, value in zip((64, 128, 256), sample.acceleration):
        if not sample.type_mask & bit:
            enabled_translation.append(value)
    if not enabled_translation:
        return Validation(False, "reference_has_no_enabled_field")
    enabled = list(enabled_translation)
    if not sample.type_mask & 1024:
        enabled.append(sample.yaw)
    if not sample.type_mask & 2048:
        enabled.append(sample.yaw_rate)
    if not _finite(tuple(enabled) + (sample.stamp, now)):
        return Validation(False, "reference_not_finite")
    return _valid_age(sample.stamp, now, timeout, future_tolerance)


def switch_delta(previous: ReferenceSample, target: ReferenceSample,
                 position_limit: float, velocity_limit: float) -> Validation:
    position_pairs = [
        (left, right)
        for bit, left, right in zip(
            (1, 2, 4), previous.position, target.position)
        if not previous.type_mask & bit and not target.type_mask & bit
    ]
    velocity_pairs = [
        (left, right)
        for bit, left, right in zip(
            (8, 16, 32), previous.velocity, target.velocity)
        if not previous.type_mask & bit and not target.type_mask & bit
    ]
    if not position_pairs:
        return Validation(False, "switch_position_not_comparable")
    if not _finite(value for pair in position_pairs + velocity_pairs
                   for value in pair):
        return Validation(False, "switch_field_not_finite")
    position_delta = math.sqrt(sum(
        (left - right) ** 2 for left, right in position_pairs))
    velocity_delta = math.sqrt(sum(
        (left - right) ** 2 for left, right in velocity_pairs))
    if position_delta > position_limit:
        return Validation(False, "switch_position_jump")
    if velocity_delta > velocity_limit:
        return Validation(False, "switch_velocity_jump")
    return Validation(True, "ok")


def health_conjunction(values: Sequence[bool],
                       minimum_inputs: int = 1) -> bool:
    return len(values) >= minimum_inputs and all(values)


def validate_vehicle_state(sample: VehicleStateSample, now: float,
                           expected_vehicle_type: int, timeout: float,
                           future_tolerance: float) -> Validation:
    if sample.vehicle_type != expected_vehicle_type:
        return Validation(False, "vehicle_type_mismatch")
    if not sample.state_valid:
        return Validation(False, "state_invalid")
    if not sample.localization_valid:
        return Validation(False, "localization_invalid")
    if not sample.odometry_fresh:
        return Validation(False, "odometry_not_fresh")
    return _valid_age(sample.stamp, now, timeout, future_tolerance)


def validate_path(sample: PathSample, now: float, expected_frame: str,
                  timeout: float, future_tolerance: float,
                  minimum_segment_length: float = 0.05,
                  maximum_points: int = 10000) -> Validation:
    if sample.frame_id != expected_frame:
        return Validation(False, "path_frame_mismatch")
    age = _valid_age(sample.stamp, now, timeout, future_tolerance)
    if not age.valid:
        return Validation(False, "path_" + age.reason)
    if len(sample.points) < 2:
        return Validation(False, "path_too_short")
    if len(sample.points) > maximum_points:
        return Validation(False, "path_too_many_points")
    if len(sample.pose_frames) != len(sample.points):
        return Validation(False, "path_pose_frame_count_mismatch")
    if any(frame and frame != expected_frame for frame in sample.pose_frames):
        return Validation(False, "path_pose_frame_mismatch")
    if not _finite(value for point in sample.points for value in point):
        return Validation(False, "path_not_finite")
    total_length = 0.0
    for previous, current in zip(sample.points, sample.points[1:]):
        length = math.sqrt(sum(
            (current[index] - previous[index]) ** 2 for index in range(3)))
        if length < minimum_segment_length:
            return Validation(False, "path_segment_too_short")
        total_length += length
    if total_length < minimum_segment_length:
        return Validation(False, "path_length_too_short")
    return Validation(True, "ok")


def arrival_reached(position: Vector3, velocity: Vector3, goal: Vector3,
                    position_tolerance: float,
                    speed_tolerance: float) -> bool:
    if not _finite(position + velocity + goal):
        return False
    distance = math.sqrt(sum(
        (position[index] - goal[index]) ** 2 for index in range(3)))
    speed = math.sqrt(sum(component * component for component in velocity))
    return distance <= position_tolerance and speed <= speed_tolerance
