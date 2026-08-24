"""Small execution-state helpers shared by ROS backends."""

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin, sqrt
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class FixedwingTrajectorySample:
    time_from_start: float
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    acceleration: Tuple[float, float, float]
    yaw: float
    yaw_rate: float


def _wrap_angle(value: float) -> float:
    return atan2(sin(float(value)), cos(float(value)))


def fixedwing_trajectory_samples(
    path: Sequence[Sequence[float]], speed_mps: float
) -> List[FixedwingTrajectorySample]:
    """Time-parameterize a sampled fly-through path with continuous P/V/A data."""

    speed = float(speed_mps)
    if not isfinite(speed) or speed <= 0.0:
        raise ValueError("fixed-wing trajectory speed must be positive and finite")
    points: List[Tuple[float, float, float]] = []
    for value in path:
        point = (float(value[0]), float(value[1]), float(value[2]))
        if not all(isfinite(component) for component in point):
            raise ValueError("fixed-wing trajectory position must be finite")
        if points:
            dx = point[0] - points[-1][0]
            dy = point[1] - points[-1][1]
            dz = point[2] - points[-1][2]
            if sqrt(dx * dx + dy * dy + dz * dz) <= 1e-6:
                continue
        points.append(point)
    if len(points) < 2:
        raise ValueError("fixed-wing trajectory needs at least two distinct points")

    times = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        dz = second[2] - first[2]
        times.append(times[-1] + sqrt(dx * dx + dy * dy + dz * dz) / speed)

    velocities = []
    for index in range(len(points)):
        first = points[max(0, index - 1)]
        second = points[min(len(points) - 1, index + 1)]
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        dz = second[2] - first[2]
        distance = sqrt(dx * dx + dy * dy + dz * dz)
        if distance <= 1e-9:
            raise ValueError("fixed-wing trajectory contains an invalid tangent")
        velocities.append(
            (speed * dx / distance, speed * dy / distance, speed * dz / distance)
        )
    yaws = [atan2(value[1], value[0]) for value in velocities]

    accelerations = []
    yaw_rates = []
    for index in range(len(points)):
        lower = max(0, index - 1)
        upper = min(len(points) - 1, index + 1)
        dt = times[upper] - times[lower]
        if dt <= 1e-9:
            accelerations.append((0.0, 0.0, 0.0))
            yaw_rates.append(0.0)
            continue
        accelerations.append(
            tuple(
                (velocities[upper][axis] - velocities[lower][axis]) / dt
                for axis in range(3)
            )
        )
        yaw_rates.append(_wrap_angle(yaws[upper] - yaws[lower]) / dt)

    return [
        FixedwingTrajectorySample(
            time_from_start=times[index],
            position=points[index],
            velocity=velocities[index],
            acceleration=accelerations[index],
            yaw=yaws[index],
            yaw_rate=yaw_rates[index],
        )
        for index in range(len(points))
    ]


def goal_distance(current: Sequence[float], goal: Sequence[float], use_z: bool) -> float:
    """Return XY or XYZ distance without assigning frame semantics."""

    dx = float(current[0]) - float(goal[0])
    dy = float(current[1]) - float(goal[1])
    dz = float(current[2]) - float(goal[2]) if use_z else 0.0
    return sqrt(dx * dx + dy * dy + dz * dz)


def goal_heading(
    current: Sequence[float],
    goal: Sequence[float],
    minimum_distance_m: float = 0.1,
) -> Optional[float]:
    """Return the world-ENU yaw facing the goal, or None near the goal XY."""

    dx = float(goal[0]) - float(current[0])
    dy = float(goal[1]) - float(current[1])
    if hypot(dx, dy) <= max(0.0, float(minimum_distance_m)):
        return None
    return atan2(dy, dx)


def worker_approach_goal(
    current: Sequence[float],
    target: Sequence[float],
    horizontal_standoff_m: float,
) -> tuple:
    """Approach in XY while retaining the worker's current world altitude."""

    dx = float(target[0]) - float(current[0])
    dy = float(target[1]) - float(current[1])
    distance = hypot(dx, dy)
    standoff = max(0.0, float(horizontal_standoff_m))
    if distance <= standoff or distance <= 1e-9:
        x = float(current[0])
        y = float(current[1])
    else:
        travel = distance - standoff
        x = float(current[0]) + dx * travel / distance
        y = float(current[1]) + dy * travel / distance
    return (x, y, float(current[2]))


def fixedwing_waypoint_reached(
    current: Sequence[float],
    segment_start: Sequence[float],
    goal: Sequence[float],
    acceptance_radius_m: float,
    altitude_tolerance_m: float,
    pass_cross_track_limit_m: float,
) -> bool:
    """Accept a fixed-wing waypoint by radius or a bounded fly-through plane."""

    if len(current) < 3 or len(segment_start) < 3 or len(goal) < 3:
        return False
    altitude_error = abs(float(current[2]) - float(goal[2]))
    if altitude_error > max(0.0, float(altitude_tolerance_m)):
        return False
    dx = float(current[0]) - float(goal[0])
    dy = float(current[1]) - float(goal[1])
    if hypot(dx, dy) <= max(0.0, float(acceptance_radius_m)):
        return True

    leg_x = float(goal[0]) - float(segment_start[0])
    leg_y = float(goal[1]) - float(segment_start[1])
    leg_squared = leg_x * leg_x + leg_y * leg_y
    if leg_squared <= 1e-9:
        return False
    beyond = dx * leg_x + dy * leg_y
    if beyond < 0.0:
        return False
    cross_track = abs(dx * leg_y - dy * leg_x) / sqrt(leg_squared)
    return cross_track <= max(0.0, float(pass_cross_track_limit_m))


class ArrivalDwellTracker:
    """Declare arrival only after a goal remains inside tolerance for a dwell."""

    def __init__(self, tolerance_m: float, dwell_sec: float):
        self.tolerance_m = max(0.0, float(tolerance_m))
        self.dwell_sec = max(0.0, float(dwell_sec))
        self._entered_at: Dict[str, float] = {}

    def reset(self, vehicle: str) -> None:
        self._entered_at.pop(str(vehicle), None)

    def reset_all(self) -> None:
        self._entered_at.clear()

    def update(
        self,
        vehicle: str,
        current: Sequence[float],
        goal: Sequence[float],
        now: float,
        use_z: bool = True,
    ) -> bool:
        name = str(vehicle)
        if goal_distance(current, goal, use_z) > self.tolerance_m:
            self.reset(name)
            return False
        entered_at = self._entered_at.setdefault(name, float(now))
        return float(now) - entered_at >= self.dwell_sec
