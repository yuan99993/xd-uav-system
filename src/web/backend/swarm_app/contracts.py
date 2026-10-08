"""Fail-closed Web-to-ROS contracts for terrain swarm task paths."""

from __future__ import annotations

import math
import re
import time
from typing import List

from pydantic import BaseModel, Field, model_validator


_UAV_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


class WorldPoint(BaseModel):
    x: float
    y: float
    z: float

    @model_validator(mode="after")
    def finite(self):
        if not all(math.isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("world coordinates must be finite")
        return self


class VehicleRoute(BaseModel):
    uav_name: str
    goal_id: int = Field(ge=1, le=4294967295)
    nominal_speed_mps: float = Field(default=15.0, gt=0.0, le=200.0)
    points: List[WorldPoint] = Field(min_length=2, max_length=64)

    @model_validator(mode="after")
    def valid_route(self):
        if not _UAV_NAME.fullmatch(self.uav_name):
            raise ValueError("uav_name must be a lowercase ROS name component")
        if not math.isfinite(self.nominal_speed_mps):
            raise ValueError("nominal_speed_mps must be finite")
        for first, second in zip(self.points, self.points[1:]):
            distance = math.sqrt(
                (second.x - first.x) ** 2
                + (second.y - first.y) ** 2
                + (second.z - first.z) ** 2)
            if distance < 1.0:
                raise ValueError("consecutive route points must be at least 1 m apart")
        return self


class SwarmDispatchRequest(BaseModel):
    frame_id: str = "world"
    use_mock: bool = True
    routes: List[VehicleRoute] = Field(min_length=2, max_length=32)

    @model_validator(mode="after")
    def valid_swarm(self):
        if self.frame_id != "world":
            raise ValueError("the terrain planning contract requires frame_id=world")
        names = [route.uav_name for route in self.routes]
        if len(names) != len(set(names)):
            raise ValueError("uav_name values must be unique")
        goal_ids = [route.goal_id for route in self.routes]
        if len(goal_ids) != len(set(goal_ids)):
            raise ValueError("goal_id values must be unique across the dispatch batch")
        return self


class FleetVehicleRequest(BaseModel):
    uav_name: str
    use_mock: bool = True

    @model_validator(mode="after")
    def valid_name(self):
        if not _UAV_NAME.fullmatch(self.uav_name):
            raise ValueError("uav_name must be a lowercase ROS name component")
        return self


class FleetSpawnRequest(FleetVehicleRequest):
    # The backend re-samples Z from the canonical DEM. Supplying the sampled
    # value here lets it reject stale or mismatched browser terrain data.
    world_origin: WorldPoint


class SystemRestartRequest(BaseModel):
    use_mock: bool = True


class FleetOperationRequest(BaseModel):
    uav_names: List[str] = Field(min_length=1, max_length=32)
    use_mock: bool = True
    takeoff_altitude_m: float = Field(default=80.0, gt=0.0, le=500.0)

    @model_validator(mode="after")
    def valid_operation(self):
        if len(self.uav_names) != len(set(self.uav_names)):
            raise ValueError("uav_names must be unique")
        if not all(_UAV_NAME.fullmatch(name) for name in self.uav_names):
            raise ValueError("uav_names must be lowercase ROS name components")
        if not math.isfinite(self.takeoff_altitude_m):
            raise ValueError("takeoff_altitude_m must be finite")
        return self


def ros_stamp(timestamp=None):
    """Return a ROS1 JSON timestamp suitable for rosbridge."""
    value = time.time() if timestamp is None else float(timestamp)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("timestamp must be positive and finite")
    seconds = int(value)
    nanoseconds = int(round((value - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    return {"secs": seconds, "nsecs": nanoseconds}


def build_path_message(route: VehicleRoute, frame_id="world", timestamp=None):
    """Build the exact nav_msgs/Path JSON accepted by the terrain planner."""
    if frame_id != "world":
        raise ValueError("terrain task paths must use frame_id=world")
    stamp = ros_stamp(timestamp)
    poses = []
    for index, point in enumerate(route.points):
        if index + 1 < len(route.points):
            neighbour = route.points[index + 1]
            dx = neighbour.x - point.x
            dy = neighbour.y - point.y
        else:
            neighbour = route.points[index - 1]
            dx = point.x - neighbour.x
            dy = point.y - neighbour.y
        yaw = math.atan2(dy, dx)
        poses.append({
            "header": {
                "seq": route.goal_id,
                "stamp": stamp.copy(),
                "frame_id": frame_id,
            },
            "pose": {
                "position": {"x": point.x, "y": point.y, "z": point.z},
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(0.5 * yaw),
                    "w": math.cos(0.5 * yaw),
                },
            },
        })
    return {
        "header": {
            "seq": route.goal_id,
            "stamp": stamp,
            "frame_id": frame_id,
        },
        "poses": poses,
    }


def path_points(message):
    return [
        (
            float(pose["pose"]["position"]["x"]),
            float(pose["pose"]["position"]["y"]),
            float(pose["pose"]["position"]["z"]),
        )
        for pose in message.get("poses", [])
    ]
