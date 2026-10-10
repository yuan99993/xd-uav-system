"""Additive FastAPI entry point for the fixed-wing mission console.

Run this module instead of app.main to retain every existing endpoint and add
only the /api/swarm surface.
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
import subprocess
import threading

from fastapi import HTTPException, Query
from fastapi.responses import Response
from pydantic import BaseModel, Field

from app.main import app

from .contracts import (
    FleetOperationRequest,
    FleetSpawnRequest,
    FleetVehicleRequest,
    SwarmDispatchRequest,
    SystemRestartRequest,
    path_points,
)
from .gateway import get_gateway, ROS_GATEWAY
from .terrain_frame import DavosdorfWorldFrame


_frame = None
_frame_lock = threading.Lock()


def terrain_frame():
    global _frame
    with _frame_lock:
        if _frame is None:
            _frame = DavosdorfWorldFrame()
        return _frame


class GeoCoordinate(BaseModel):
    longitude: float
    latitude: float
    clearance_m: float = Field(default=80.0, ge=51.0, le=119.0)


class WorldCoordinate(BaseModel):
    x: float
    y: float


class WorldElevationCoordinate(WorldCoordinate):
    clearance_m: float = Field(default=80.0, ge=51.0, le=119.0)


def _frame_or_503():
    try:
        return terrain_frame()
    except (OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=503, detail=str(error)) from error


def _decorate_snapshot(snapshot):
    frame = _frame_or_503()
    output = copy.deepcopy(snapshot)
    display_paths = {}
    for name, message in (snapshot.get("paths") or {}).items():
        try:
            frame_id = message.get("header", {}).get("frame_id", "").lstrip("/")
            if frame_id != "world":
                raise ValueError("terrain result path must use frame_id=world")
            points = path_points(message)
            display_paths[name] = {
                "frame_id": frame_id,
                "goal_id": (
                    int(message["poses"][0]["header"]["seq"])
                    if message.get("poses") else 0),
                "points": frame.display_points(points),
            }
        except (KeyError, TypeError, ValueError) as error:
            display_paths[name] = {"error": str(error), "points": []}
    output["display_paths"] = display_paths

    display_conflicts = []
    status = snapshot.get("status") or {}
    for conflict in status.get("conflicts", []):
        item = copy.deepcopy(conflict)
        try:
            first = conflict["first_position"]
            second = conflict["second_position"]
            first_lng, first_lat = frame.world_to_geo(first["x"], first["y"])
            second_lng, second_lat = frame.world_to_geo(second["x"], second["y"])
            item["first_geo"] = {"lng": first_lng, "lat": first_lat}
            item["second_geo"] = {"lng": second_lng, "lat": second_lat}
        except (KeyError, TypeError, ValueError) as error:
            item["display_error"] = str(error)
        display_conflicts.append(item)
    output["display_conflicts"] = display_conflicts
    return output


def _decorate_fleet(snapshot):
    frame = _frame_or_503()
    output = copy.deepcopy(snapshot)
    runtime = output.get("runtime") or {}
    for item in runtime.values():
        point = item.get("world_position")
        if point:
            longitude, latitude = frame.world_to_geo(point["x"], point["y"])
            item["display_position"] = {
                **point, "lng": longitude, "lat": latitude}
        display_trail = []
        for trail_point in item.get("trail") or []:
            longitude, latitude = frame.world_to_geo(
                trail_point["x"], trail_point["y"])
            display_trail.append({
                **trail_point, "lng": longitude, "lat": latitude})
        item["display_trail"] = display_trail
    return output


def _vehicle_example_or_404(name):
    examples = _frame_or_503().vehicle_examples()
    match = next((item for item in examples if item["uav_name"] == name), None)
    if match is None:
        raise HTTPException(
            status_code=404,
            detail="没有找到经过 DEM 核验的飞机示例：{}".format(name))
    return match


def _spawn_example(request):
    example = copy.deepcopy(_vehicle_example_or_404(request.uav_name))
    frame = _frame_or_503()
    x = float(request.world_origin.x)
    y = float(request.world_origin.y)
    ground = frame.elevation_at_world(x, y)
    if abs(float(request.world_origin.z) - ground) > 1.0:
        raise HTTPException(
            status_code=409,
            detail="所选起点高程与当前 DEM 不一致，请重新在地图选点")
    example["world_origin"] = {"x": x, "y": y, "z": ground}
    example["gazebo_spawn"] = {
        "x": x,
        "y": y,
        "z": ground,
    }
    example["route"]["points"][0] = {
        "x": x, "y": y, "z": ground + 80.0}
    example["route"]["display_points"] = frame.display_points([
        (point["x"], point["y"], point["z"])
        for point in example["route"]["points"]
    ])
    return example


@app.get("/api/swarm/health")
def swarm_health():
    frame = _frame_or_503()
    return {
        "status": "ok",
        "component": "fixedwing_mission_console",
        "default_mode": "mock",
        "ros_contract": {
            "input_type": "nav_msgs/Path",
            "input_topic": "/<uav>/planning/task_path",
            "result_type": "xd_uav_planning/TerrainSwarmStatus",
            "result_topic": "/terrain_swarm/status",
            "frame_id": "world",
        },
        "canonical_dem_match": frame.canonical_match,
    }


@app.get("/api/swarm/capabilities")
def swarm_capabilities():
    return {
        "web": True,
        "offline_mission_editing": True,
        "mock_validation": True,
        "runtime": ROS_GATEWAY.capabilities(),
        "safety": {
            "automatic_deferred_dispatch": False,
            "requires_explicit_redispatch_after_runtime_connect": True,
        },
    }


@app.get("/api/swarm/terrain")
def swarm_terrain():
    return _frame_or_503().metadata()


@app.get("/api/swarm/terrain.png")
def swarm_terrain_overlay():
    return Response(
        content=_frame_or_503().overlay_png(),
        media_type="image/png",
        headers={"Cache-Control": "public, max-age=3600"},
    )


@app.get("/api/swarm/terrain/raster")
def swarm_terrain_raster(max_grid: int = Query(default=180, ge=32, le=320)):
    return _frame_or_503().raster_payload(max_grid=max_grid)


@app.post("/api/swarm/coordinates/to_world")
def swarm_to_world(coordinate: GeoCoordinate):
    try:
        frame = _frame_or_503()
        x, y = frame.geo_to_world(
            coordinate.longitude, coordinate.latitude)
        ground = frame.elevation_at_world(x, y)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    return {
        "frame_id": "world",
        "x": x,
        "y": y,
        "ground_elevation_m": ground,
        "flight_z_m": ground + coordinate.clearance_m,
        "clearance_m": coordinate.clearance_m,
    }


@app.post("/api/swarm/coordinates/elevation")
def swarm_world_elevation(coordinate: WorldElevationCoordinate):
    try:
        ground = _frame_or_503().elevation_at_world(coordinate.x, coordinate.y)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    return {
        "frame_id": "world",
        "x": coordinate.x,
        "y": coordinate.y,
        "ground_elevation_m": ground,
        "flight_z_m": ground + coordinate.clearance_m,
        "clearance_m": coordinate.clearance_m,
    }


@app.post("/api/swarm/coordinates/to_wgs84")
def swarm_to_wgs84(coordinate: WorldCoordinate):
    try:
        longitude, latitude = _frame_or_503().world_to_geo(
            coordinate.x, coordinate.y)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    return {"longitude": longitude, "latitude": latitude}


@app.post("/api/swarm/dispatch")
def swarm_dispatch(request: SwarmDispatchRequest):
    frame = _frame_or_503()
    if not request.use_mock and not frame.canonical_match:
        raise HTTPException(
            status_code=409,
            detail="Web and canonical ROS terrain DEM files do not match")
    try:
        snapshot = get_gateway(request.use_mock).dispatch(request)
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=502, detail=str(error)) from error
    snapshot["dispatched"] = [
        {
            "uav_name": route.uav_name,
            "goal_id": route.goal_id,
            "topic": "/{}/planning/task_path".format(route.uav_name),
            "point_count": len(route.points),
        }
        for route in request.routes
    ]
    return _decorate_snapshot(snapshot)


@app.get("/api/swarm/snapshot")
def swarm_snapshot(use_mock: bool = Query(default=True)):
    try:
        snapshot = get_gateway(use_mock).snapshot()
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=502, detail=str(error)) from error
    return _decorate_snapshot(snapshot)


@app.get("/api/swarm/fleet")
def swarm_fleet(use_mock: bool = Query(default=True)):
    try:
        gateway = get_gateway(use_mock)
        snapshot = gateway.fleet_snapshot()
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=502, detail=str(error)) from error
    return _decorate_fleet(snapshot)


@app.post("/api/swarm/fleet/spawn")
def swarm_fleet_spawn(request: FleetSpawnRequest):
    example = _spawn_example(request)
    try:
        result = get_gateway(request.use_mock).spawn_vehicle(example)
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {"uav_name": request.uav_name, **result}


@app.post("/api/swarm/system/restart")
def swarm_system_restart(request: SystemRestartRequest):
    gateway = get_gateway(request.use_mock)
    if request.use_mock:
        return {"restarting": False, **gateway.reset()}
    try:
        gateway.prepare_restart()
        script = (Path(__file__).resolve().parents[3]
                  / "xd_uav_planning" / "scripts" / "runtime"
                  / "fixedwing_sitl_runtime.sh")
        if not script.is_file() or not os.access(script, os.X_OK):
            raise RuntimeError("固定翼仿真执行系统重启脚本不可用")
        subprocess.Popen(
            [str(script), "restart"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            close_fds=True,
        )
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {
        "accepted": True,
        "restarting": True,
        "message": "仿真执行系统已开始安全重启；Web 将保持运行",
    }


@app.post("/api/swarm/fleet/remove")
def swarm_fleet_remove(request: FleetVehicleRequest):
    try:
        result = get_gateway(request.use_mock).remove_vehicle(request.uav_name)
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {"uav_name": request.uav_name, **result}


@app.post("/api/swarm/fleet/takeoff")
def swarm_fleet_takeoff(request: FleetOperationRequest):
    try:
        result = get_gateway(request.use_mock).takeoff(
            request.uav_names, request.takeoff_altitude_m)
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {"uav_names": request.uav_names, **result}


@app.post("/api/swarm/fleet/land")
def swarm_fleet_land(request: FleetOperationRequest):
    try:
        result = get_gateway(request.use_mock).land(request.uav_names)
    except (ConnectionError, OSError, RuntimeError, ValueError) as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {"uav_names": request.uav_names, **result}
