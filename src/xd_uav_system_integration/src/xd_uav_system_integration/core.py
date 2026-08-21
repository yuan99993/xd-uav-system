"""ROS-independent contracts for reference arbitration and profiles."""

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


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
class Validation:
    valid: bool
    reason: str


@dataclass(frozen=True)
class OdometrySample:
    stamp: float
    parent_frame: str
    child_frame: str
    position: Vector3
    orientation: Quaternion
    linear_velocity: Vector3
    pose_variance: Vector3
    velocity_variance: Vector3


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def cross(left: Vector3, right: Vector3) -> Vector3:
    return (left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0])


def mat3_vector(matrix: Sequence[float], vector: Vector3) -> Vector3:
    """Apply a row-major 3x3 rotation matrix to a vector."""
    if len(matrix) != 9:
        raise ValueError("rotation matrix must contain nine values")
    return tuple(sum(matrix[row * 3 + column] * vector[column]
                     for column in range(3)) for row in range(3))


def rigid_body_imu_to_body(linear_acceleration_sensor: Vector3,
                           angular_velocity_sensor: Vector3,
                           angular_acceleration_body: Vector3,
                           body_to_sensor_translation: Vector3,
                           sensor_to_body_rotation: Sequence[float]):
    """Move IMU vectors from an offset sensor origin to the body origin.

    Inputs and output are specific force/angular velocity.  The rigid-body
    relation is a_sensor = a_body + alpha x r + omega x (omega x r), where
    r points from the body origin to the sensor origin and is body-expressed.
    """
    acceleration_sensor_body = mat3_vector(
        sensor_to_body_rotation, linear_acceleration_sensor)
    omega_body = mat3_vector(sensor_to_body_rotation,
                             angular_velocity_sensor)
    tangential = cross(angular_acceleration_body,
                       body_to_sensor_translation)
    centripetal = cross(omega_body,
                        cross(omega_body, body_to_sensor_translation))
    acceleration_body = tuple(acceleration_sensor_body[index] -
                              tangential[index] - centripetal[index]
                              for index in range(3))
    return acceleration_body, omega_body


def validate_shadow_odometry(sample: OdometrySample, now: float,
                             expected_parent: str, expected_child: str,
                             timeout: float,
                             future_tolerance: float,
                             maximum_speed: float = math.inf) -> Validation:
    """Validate Fast-LIO output without relabeling or transforming it."""
    if sample.parent_frame != expected_parent:
        return Validation(False, "shadow_parent_frame_mismatch")
    if sample.child_frame != expected_child:
        return Validation(False, "shadow_child_frame_mismatch")
    values = (sample.position + sample.orientation + sample.linear_velocity
              + sample.pose_variance + sample.velocity_variance
              + (sample.stamp, now, timeout, future_tolerance))
    if not _finite(values):
        return Validation(False, "shadow_not_finite")
    if sample.stamp <= 0.0:
        return Validation(False, "shadow_stamp_not_positive")
    age = now - sample.stamp
    if age < -future_tolerance:
        return Validation(False, "shadow_stamp_in_future")
    if age > timeout:
        return Validation(False, "shadow_stale")
    norm = math.sqrt(sum(value * value for value in sample.orientation))
    if norm < 1.0e-6 or abs(norm - 1.0) > 1.0e-3:
        return Validation(False, "shadow_orientation_not_normalized")
    if any(value <= 0.0 for value in
           sample.pose_variance + sample.velocity_variance):
        return Validation(False, "shadow_covariance_not_positive")
    speed = math.sqrt(sum(value * value for value in sample.linear_velocity))
    if not math.isfinite(maximum_speed) or maximum_speed <= 0.0:
        return Validation(False, "shadow_speed_limit_invalid")
    if speed > maximum_speed:
        return Validation(False, "shadow_ground_speed_exceeded")
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
    if sample.stamp <= 0.0:
        return Validation(False, "reference_stamp_not_positive")
    age = now - sample.stamp
    if age < -future_tolerance:
        return Validation(False, "reference_stamp_in_future")
    if age > timeout:
        return Validation(False, "reference_stale")
    return Validation(True, "ok")


def switch_delta(previous: ReferenceSample, target: ReferenceSample,
                 position_limit: float, velocity_limit: float) -> Validation:
    position_pairs = [
        (a, b) for bit, a, b in zip((1, 2, 4), previous.position, target.position)
        if not previous.type_mask & bit and not target.type_mask & bit]
    velocity_pairs = [
        (a, b) for bit, a, b in zip((8, 16, 32), previous.velocity, target.velocity)
        if not previous.type_mask & bit and not target.type_mask & bit]
    if not position_pairs:
        return Validation(False, "switch_position_not_comparable")
    if not _finite(value for pair in position_pairs + velocity_pairs for value in pair):
        return Validation(False, "switch_field_not_finite")
    position_delta = math.sqrt(sum((a - b) ** 2 for a, b in position_pairs))
    velocity_delta = math.sqrt(sum((a - b) ** 2 for a, b in velocity_pairs))
    if position_delta > position_limit:
        return Validation(False, "switch_position_jump")
    if velocity_delta > velocity_limit:
        return Validation(False, "switch_velocity_jump")
    return Validation(True, "ok")


def health_conjunction(values: Sequence[bool], minimum_inputs: int = 1) -> bool:
    return len(values) >= minimum_inputs and all(values)


def validate_profile(profile: Mapping) -> Sequence[str]:
    errors = []
    if profile.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    vehicles = profile.get("vehicles")
    if not isinstance(vehicles, list) or not vehicles:
        errors.append("vehicles must be a non-empty list")
        vehicles = []
    names = [item.get("name") for item in vehicles if isinstance(item, dict)]
    if len(names) != len(vehicles) or any(not name for name in names):
        errors.append("every vehicle needs a name")
    elif len(set(names)) != len(names):
        errors.append("vehicle names must be unique")

    features = profile.get("features", {})
    ego = features.get("ego", {}) if isinstance(features, dict) else {}
    sead = features.get("sead", {}) if isinstance(features, dict) else {}
    control = profile.get("control", {})
    owner = control.get("reference_owner") if isinstance(control, dict) else None
    if owner not in ("sead", "ego"):
        errors.append("reference_owner must be sead or ego")
    if owner == "ego" and not ego.get("enabled", False):
        errors.append("ego owner requires ego.enabled")
    if owner == "sead" and not sead.get("enabled", False):
        errors.append("sead owner requires sead.enabled")

    simulation = profile.get("simulation", {})
    backend = simulation.get("backend") if isinstance(simulation, dict) else None
    if backend == "px4_gazebo" and ego.get("sensing") == "fake_drone":
        errors.append("px4_gazebo and fake_drone are mutually exclusive")
    if ego.get("enabled", False):
        ids = [item.get("ego_id") for item in vehicles
               if isinstance(item, dict)]
        if any(not isinstance(value, int) or isinstance(value, bool)
               for value in ids):
            errors.append("every EGO vehicle needs an integer ego_id")
        elif sorted(ids) != list(range(len(ids))):
            errors.append("ego_id values must be unique and continuous from 0")
        frames = profile.get("frames", {})
        if not isinstance(frames, dict) or not frames.get("common"):
            errors.append("EGO requires frames.common")
        if not simulation.get("use_sim_time", False):
            errors.append("EGO PX4/Gazebo profile requires use_sim_time")
    return errors
