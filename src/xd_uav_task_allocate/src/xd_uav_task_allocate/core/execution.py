"""Small execution-state helpers shared by ROS backends."""

from math import atan2, hypot, sqrt
from typing import Dict, Optional, Sequence


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
