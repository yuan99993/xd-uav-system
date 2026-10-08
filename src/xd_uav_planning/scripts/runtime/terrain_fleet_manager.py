#!/usr/bin/env python3
"""Safely create and remove complete fixed-wing terrain SITL instances."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import Point
from mavros_msgs.msg import ExtendedState, State

from xd_uav_planning.msg import TerrainFleetStatus, TerrainFleetVehicle
from xd_uav_planning.srv import (
    RemoveTerrainVehicle,
    RemoveTerrainVehicleResponse,
    SpawnTerrainVehicle,
    SpawnTerrainVehicleResponse,
)
from xd_uav_planning.terrain_fleet import (
    FleetAllocator,
    validate_removal_state,
)


@dataclass
class VehicleRuntime:
    name: str
    slot: object
    world_origin: tuple
    gazebo_spawn: tuple
    process: subprocess.Popen
    state_subscriber: object
    extended_subscriber: object
    started_at: float
    state: object = None
    extended_state: object = None
    lifecycle: int = TerrainFleetVehicle.STARTING
    detail: str = "starting"


def _finite_point(values, label):
    if len(values) != 3:
        raise ValueError(label + " must contain exactly three values")
    output = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in output):
        raise ValueError(label + " must be finite")
    return output


class TerrainFleetManager:
    def __init__(self):
        capacity = int(rospy.get_param("~capacity", 8))
        rate = float(rospy.get_param("~publication_rate_hz", 2.0))
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publication_rate_hz must be positive and finite")
        self._allocator = FleetAllocator(capacity)
        self._lock = threading.RLock()
        self._vehicles = {}
        self._roslaunch = shutil.which("roslaunch")
        if not self._roslaunch:
            raise RuntimeError("roslaunch executable is unavailable")
        self._vehicle_launch = str(rospy.get_param(
            "~vehicle_launch", "terrain_fixedwing_swarm_vehicle.launch"))
        self._map_path = str(rospy.get_param("~map_path"))
        if not self._map_path or not Path(self._map_path).is_file():
            raise ValueError("map_path must reference the canonical DEM")
        runtime_default = "/tmp/xd_uav_terrain_fleet_{}".format(os.getuid())
        self._runtime = Path(rospy.get_param("~runtime_dir", runtime_default))
        self._runtime.mkdir(parents=True, exist_ok=True)
        self._publisher = rospy.Publisher(
            rospy.get_param("~status_topic", "/terrain_swarm/fleet"),
            TerrainFleetStatus, queue_size=5, latch=True)
        self._spawn_service = rospy.Service(
            rospy.get_param("~spawn_service", "/terrain_swarm/fleet/spawn"),
            SpawnTerrainVehicle, self._spawn_callback)
        self._remove_service = rospy.Service(
            rospy.get_param("~remove_service", "/terrain_swarm/fleet/remove"),
            RemoveTerrainVehicle, self._remove_callback)
        rospy.on_shutdown(self._shutdown_all)
        initial_delay = float(rospy.get_param("~initial_spawn_delay_s", 1.0))
        if not math.isfinite(initial_delay) or initial_delay < 0.0:
            raise ValueError("initial_spawn_delay_s must be finite and non-negative")
        initial_vehicles = (rospy.get_param("~initial_vehicles", [])
                            if rospy.get_param("~auto_spawn_initial", True)
                            else [])
        for item in initial_vehicles:
            try:
                self._spawn(
                    str(item["uav_name"]),
                    _finite_point(item["world_origin_m"], "world_origin_m"),
                    _finite_point(item["gazebo_spawn_m"], "gazebo_spawn_m"),
                    int(item["preferred_instance"]),
                )
            except (KeyError, OSError, RuntimeError, ValueError) as error:
                rospy.logerr("failed to start initial terrain vehicle: %s", error)
            if initial_delay > 0.0:
                time.sleep(initial_delay)
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate), self._tick)

    def _command(self, name, slot, world_origin, gazebo_spawn):
        arguments = {
            "UAV_NAME": name,
            "instance": slot.instance,
            "system_id": slot.system_id,
            "mavlink_udp_port": slot.mavlink_udp_port,
            "mavlink_tcp_port": slot.mavlink_tcp_port,
            "mavros_fcu_url": slot.mavros_fcu_url,
            "gst_udp_port": slot.gst_udp_port,
            "camera_udp_port": slot.camera_udp_port,
            "gazebo_x": gazebo_spawn[0],
            "gazebo_y": gazebo_spawn[1],
            "gazebo_z": gazebo_spawn[2],
            "world_origin_x": world_origin[0],
            "world_origin_y": world_origin[1],
            "world_origin_z": world_origin[2],
            "terrain_random_seed": 20261005 + slot.instance,
            "map_path": self._map_path,
        }
        return [
            self._roslaunch,
            "xd_uav_planning",
            self._vehicle_launch,
        ] + ["{}:={}".format(key, value) for key, value in arguments.items()]

    def _state_callback(self, name, message):
        with self._lock:
            runtime = self._vehicles.get(name)
            if runtime is not None:
                runtime.state = message

    def _extended_callback(self, name, message):
        with self._lock:
            runtime = self._vehicles.get(name)
            if runtime is not None:
                runtime.extended_state = message

    def _spawn(self, name, world_origin, gazebo_spawn, preferred_instance=None):
        with self._lock:
            slot = self._allocator.allocate(name, preferred_instance)
        log_path = self._runtime / (name + ".log")
        try:
            with log_path.open("ab", buffering=0) as output:
                process = subprocess.Popen(
                    self._command(name, slot, world_origin, gazebo_spawn),
                    stdin=subprocess.DEVNULL,
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    close_fds=True,
                )
        except (OSError, ValueError):
            with self._lock:
                self._allocator.release(name)
            raise
        runtime = VehicleRuntime(
            name=name,
            slot=slot,
            world_origin=world_origin,
            gazebo_spawn=gazebo_spawn,
            process=process,
            state_subscriber=rospy.Subscriber(
                "/{}/mavros/state".format(name), State,
                lambda message, key=name: self._state_callback(key, message),
                queue_size=5),
            extended_subscriber=rospy.Subscriber(
                "/{}/mavros/extended_state".format(name), ExtendedState,
                lambda message, key=name: self._extended_callback(key, message),
                queue_size=5),
            started_at=time.monotonic(),
        )
        with self._lock:
            self._vehicles[name] = runtime
        rospy.loginfo(
            "starting terrain vehicle %s in instance %d (log %s)",
            name, slot.instance, log_path)
        return runtime

    def _spawn_callback(self, request):
        try:
            world_origin = _finite_point(
                (request.world_origin.x, request.world_origin.y,
                 request.world_origin.z), "world_origin")
            gazebo_spawn = _finite_point(
                (request.gazebo_spawn.x, request.gazebo_spawn.y,
                 request.gazebo_spawn.z), "gazebo_spawn")
            runtime = self._spawn(
                str(request.uav_name), world_origin, gazebo_spawn)
            return SpawnTerrainVehicleResponse(
                accepted=True,
                message="{} starting in instance {}".format(
                    runtime.name, runtime.slot.instance))
        except (OSError, RuntimeError, ValueError) as error:
            return SpawnTerrainVehicleResponse(
                accepted=False, message=str(error))

    @staticmethod
    def _on_ground(runtime):
        return bool(
            runtime.extended_state is not None
            and runtime.extended_state.landed_state
            == ExtendedState.LANDED_STATE_ON_GROUND)

    @staticmethod
    def _stop_process(process):
        if process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=15.0)
            return
        except (OSError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5.0)
            return
        except (OSError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=3.0)
        except OSError:
            pass
        except subprocess.TimeoutExpired:
            pass

    def _remove(self, name, enforce_safety=True):
        with self._lock:
            runtime = self._vehicles.get(name)
            if runtime is None:
                raise ValueError("vehicle does not exist: " + name)
            state = runtime.state
            on_ground = self._on_ground(runtime)
            if enforce_safety:
                validate_removal_state(
                    bool(state and state.connected),
                    bool(state and state.armed),
                    on_ground)
                raise RuntimeError(
                    "running-aircraft removal is unavailable: Gazebo Classic "
                    "Plane crashes while unloading its model; stop and restart "
                    "the simulation to clear aircraft safely")
            runtime.lifecycle = TerrainFleetVehicle.STOPPING
            runtime.detail = "stopping"
        # Full-simulator shutdown owns Gazebo model cleanup. Calling
        # /gazebo/delete_model for a Plane crashes this Gazebo Classic build.
        self._stop_process(runtime.process)
        runtime.state_subscriber.unregister()
        runtime.extended_subscriber.unregister()
        with self._lock:
            self._vehicles.pop(name, None)
            self._allocator.release(name)
        rospy.loginfo("removed terrain vehicle %s", name)

    def _remove_callback(self, request):
        try:
            self._remove(str(request.uav_name), enforce_safety=True)
            return RemoveTerrainVehicleResponse(
                accepted=True, message="vehicle removed")
        except (rospy.ROSException, rospy.ServiceException,
                RuntimeError, ValueError) as error:
            return RemoveTerrainVehicleResponse(
                accepted=False, message=str(error))

    def _message(self, runtime):
        output = TerrainFleetVehicle()
        output.uav_name = runtime.name
        output.instance = runtime.slot.instance
        output.lifecycle = runtime.lifecycle
        state = runtime.state
        output.connected = bool(state and state.connected)
        output.armed = bool(state and state.armed)
        output.on_ground = self._on_ground(runtime)
        output.mode = state.mode if state is not None else ""
        output.world_origin = Point(*runtime.world_origin)
        output.gazebo_spawn = Point(*runtime.gazebo_spawn)
        output.detail = runtime.detail
        return output

    def _tick(self, _event):
        with self._lock:
            runtimes = list(self._vehicles.values())
            for runtime in runtimes:
                if runtime.lifecycle == TerrainFleetVehicle.STOPPING:
                    continue
                return_code = runtime.process.poll()
                if return_code is not None:
                    runtime.lifecycle = TerrainFleetVehicle.ERROR
                    runtime.detail = "roslaunch exited with code {}".format(return_code)
                elif runtime.state is not None and runtime.state.connected:
                    runtime.lifecycle = TerrainFleetVehicle.READY
                    runtime.detail = "ready"
                elif time.monotonic() - runtime.started_at > 90.0:
                    runtime.lifecycle = TerrainFleetVehicle.ERROR
                    runtime.detail = "MAVROS connection timeout"
                else:
                    runtime.lifecycle = TerrainFleetVehicle.STARTING
                    runtime.detail = "waiting for MAVROS"
            message = TerrainFleetStatus()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "world"
            message.capacity = self._allocator.capacity
            message.vehicles = [
                self._message(runtime)
                for runtime in sorted(runtimes, key=lambda item: item.slot.instance)
            ]
        self._publisher.publish(message)

    def _shutdown_all(self):
        with self._lock:
            names = list(self._vehicles)
        for name in names:
            try:
                self._remove(name, enforce_safety=False)
            except Exception as error:  # shutdown must continue through all entries
                rospy.logerr("failed to stop terrain vehicle %s: %s", name, error)


def main():
    rospy.init_node("terrain_fleet_manager")
    try:
        TerrainFleetManager()
    except (OSError, RuntimeError, ValueError) as error:
        rospy.logfatal("terrain fleet manager configuration failed: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
