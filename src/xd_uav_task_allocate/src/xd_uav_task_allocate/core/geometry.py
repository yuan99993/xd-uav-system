"""Coordinate conversion and time-aligned vehicle pose history."""

from collections import deque
from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Deque, Iterable, Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    position_reference: Vector3
    orientation_reference_body: Quaternion
    frame_id: str = ""


@dataclass(frozen=True)
class RigidTransform:
    """Full 3-D rigid transform supplied by the workspace TF tree."""

    translation: Vector3 = (0.0, 0.0, 0.0)
    rotation: Quaternion = (0.0, 0.0, 0.0, 1.0)

    def apply_vector(self, vector: Sequence[float]) -> Vector3:
        return rotate_vector(self.rotation, vector)

    def apply(self, point: Sequence[float]) -> Vector3:
        x, y, z = self.apply_vector(point)
        return (
            self.translation[0] + x,
            self.translation[1] + y,
            self.translation[2] + z,
        )


def _finite(values: Iterable[float]) -> bool:
    return all(isfinite(float(value)) for value in values)


def rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> Vector3:
    """Rotate a vector with an xyzw quaternion without external dependencies."""

    if len(quaternion) < 4 or len(vector) < 3:
        raise ValueError("quaternion/vector has insufficient elements")
    qx, qy, qz, qw = [float(value) for value in quaternion[:4]]
    vx, vy, vz = [float(value) for value in vector[:3]]
    if not _finite((qx, qy, qz, qw, vx, vy, vz)):
        raise ValueError("quaternion/vector contains non-finite values")
    norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError("quaternion norm is zero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    # q * v * q^-1, written as the stable vector form.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _rotation_matrix(quaternion: Sequence[float]):
    qx, qy, qz, qw = [float(value) for value in quaternion[:4]]
    norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError("quaternion norm is zero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)),
        (2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)),
        (2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)),
    )


def rotate_covariance_frd_to_shared(
    covariance: Sequence[float],
    orientation_reference_body: Sequence[float],
    world_from_reference: RigidTransform = RigidTransform(),
) -> Tuple[float, ...]:
    """Rotate a 3x3 FRD covariance through body FLU, odom and shared frames."""

    if len(covariance) < 9 or not _finite(covariance[:9]):
        return (0.0,) * 9
    body = _rotation_matrix(orientation_reference_body)
    shared = _rotation_matrix(world_from_reference.rotation)
    frd_to_flu = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))

    def multiply(left, right):
        return tuple(
            tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
            for row in range(3)
        )

    transform = multiply(multiply(shared, body), frd_to_flu)
    source = tuple(tuple(float(covariance[row * 3 + column]) for column in range(3)) for row in range(3))
    intermediate = multiply(transform, source)
    transpose = tuple(tuple(transform[column][row] for column in range(3)) for row in range(3))
    rotated = multiply(intermediate, transpose)
    return tuple(rotated[row][column] for row in range(3) for column in range(3))


def relative_frd_to_shared(
    relative_frd: Sequence[float],
    pose: PoseSample,
    world_from_reference: RigidTransform = RigidTransform(),
) -> Vector3:
    """Convert detect's body forward/right/down vector to a shared ENU point."""

    if len(relative_frd) < 3:
        raise ValueError("relative_frd has insufficient elements")
    forward, right, down = [float(value) for value in relative_frd[:3]]
    body_flu = (forward, -right, -down)
    relative_reference = rotate_vector(pose.orientation_reference_body, body_flu)
    point_reference = tuple(
        float(pose.position_reference[index]) + relative_reference[index]
        for index in range(3)
    )
    return world_from_reference.apply(point_reference)


class PoseHistory:
    """Small timestamped buffer used to geolocate detections at capture time."""

    def __init__(self, maximum_age_sec: float = 5.0, maximum_size: int = 500):
        self.maximum_age_sec = max(0.1, float(maximum_age_sec))
        self.maximum_size = max(2, int(maximum_size))
        self._samples: Deque[PoseSample] = deque()

    def add(self, sample: PoseSample) -> None:
        if not isfinite(sample.stamp) or sample.stamp <= 0.0:
            return
        self._samples.append(sample)
        while len(self._samples) > self.maximum_size:
            self._samples.popleft()
        newest = self._samples[-1].stamp
        while self._samples and newest - self._samples[0].stamp > self.maximum_age_sec:
            self._samples.popleft()

    def closest(self, stamp: float, maximum_difference_sec: float) -> Optional[PoseSample]:
        if not self._samples or not isfinite(stamp) or stamp <= 0.0:
            return None
        sample = min(self._samples, key=lambda item: abs(item.stamp - stamp))
        if abs(sample.stamp - stamp) > max(0.0, float(maximum_difference_sec)):
            return None
        return sample

    def latest(self) -> Optional[PoseSample]:
        return self._samples[-1] if self._samples else None
