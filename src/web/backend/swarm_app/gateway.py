"""Mock and rosbridge transports for the mountain-swarm console."""

from __future__ import annotations

import copy
import os
from pathlib import Path
import socket
import sys
import threading
import time

from .contracts import build_path_message, path_points


# Reuse the dependency-free, tested swarm conflict engine. This Web layer does not
# carry a second path-conflict implementation.
PLANNING_PYTHON = Path(__file__).resolve().parents[3] / "xd_uav_planning" / "src"
if str(PLANNING_PYTHON) not in sys.path:
    sys.path.insert(0, str(PLANNING_PYTHON))

from xd_uav_planning.terrain_swarm import (  # noqa: E402
    detect_pair_conflict,
    path_length,
    release_conflict_counts,
)


MINIMUM_SEPARATION_M = 133.33334


def _header(stamp):
    return {"seq": 0, "stamp": copy.deepcopy(stamp), "frame_id": "world"}


class MockSwarmGateway:
    """Deterministic no-ROS demonstration using exact production contracts."""

    def __init__(self):
        self._lock = threading.Lock()
        self._paths = {}
        self._speeds = {}
        self._status = None
        self._fleet = {}

    def dispatch(self, request):
        timestamp = time.time()
        paths = {
            route.uav_name: build_path_message(
                route, frame_id=request.frame_id, timestamp=timestamp)
            for route in request.routes
        }
        speeds = {
            route.uav_name: route.nominal_speed_mps for route in request.routes
        }
        status = self._analyse(paths, speeds)
        with self._lock:
            self._paths = paths
            self._speeds = speeds
            self._status = status
        return self.snapshot()

    def _analyse(self, paths, speeds):
        first_path = next(iter(paths.values()))
        stamp = first_path["header"]["stamp"]
        status = {
            "header": _header(stamp),
            "analysis_ready": True,
            "conflict_free": True,
            "detail": "ready:mock_echo_of_task_paths:not_real_terrain_results",
            "vehicles": [],
            "conflicts": [],
        }
        names = list(paths)
        for name in names:
            message = paths[name]
            points = path_points(message)
            goal_id = int(message["poses"][0]["header"]["seq"])
            status["vehicles"].append({
                "uav_name": name,
                "planner_status": {
                    "header": _header(stamp),
                    "goal_id": goal_id,
                    "state": 1,
                    "detail": "terrain_path_ready:mock_echo_not_real_planner",
                },
                "path_available": True,
                "path_points": len(points),
                "path_length_m": path_length(points),
                "nominal_speed_mps": speeds[name],
            })
        for first_index, first_name in enumerate(names):
            for second_name in names[first_index + 1:]:
                first = paths[first_name]
                second = paths[second_name]
                result = detect_pair_conflict(
                    path_points(first), speeds[first_name],
                    path_points(second), speeds[second_name],
                    MINIMUM_SEPARATION_M)
                if result is None:
                    continue
                status["conflicts"].append({
                    "conflict_type": (
                        2 if result.conflict_type == "spatiotemporal" else 1),
                    "first_uav": first_name,
                    "first_goal_id": int(first["poses"][0]["header"]["seq"]),
                    "second_uav": second_name,
                    "second_goal_id": int(second["poses"][0]["header"]["seq"]),
                    "first_position": {
                        "x": result.first_position[0],
                        "y": result.first_position[1],
                        "z": result.first_position[2],
                    },
                    "second_position": {
                        "x": result.second_position[0],
                        "y": result.second_position[1],
                        "z": result.second_position[2],
                    },
                    "separation_m": result.separation_m,
                    "threshold_m": MINIMUM_SEPARATION_M,
                    "predicted_time_valid": result.predicted_time_s is not None,
                    "predicted_time_s": result.predicted_time_s or 0.0,
                })
        blocking, spatial_warnings = release_conflict_counts(
            ("spatiotemporal" if item["conflict_type"] == 2 else "spatial"
            for item in status["conflicts"]),
            spatial_only_blocks_release=False)
        status["conflict_free"] = blocking == 0
        status["detail"] = (
            "ready:mock_echo_of_task_paths:blocking_conflicts={}:"
            "spatial_warnings={}:"
            "not_real_terrain_results"
        ).format(blocking, spatial_warnings)
        return status

    def snapshot(self):
        with self._lock:
            return {
                "mode": "mock",
                "status": copy.deepcopy(self._status),
                "paths": copy.deepcopy(self._paths),
            }

    def fleet_snapshot(self):
        with self._lock:
            return {
                "mode": "mock",
                "capacity": 8,
                "vehicles": copy.deepcopy(list(self._fleet.values())),
                "runtime": {
                    name: {
                        "world_position": copy.deepcopy(item["world_origin"]),
                        "trail": [],
                        "state_valid": True,
                        "stable": True,
                        "airspeed_mps": 0.0,
                        "groundspeed_mps": 0.0,
                        "path_status": None,
                    }
                    for name, item in self._fleet.items()
                },
            }

    def spawn_vehicle(self, example):
        name = example["uav_name"]
        with self._lock:
            if name in self._fleet:
                raise ValueError("vehicle already exists: " + name)
            item = {
                "uav_name": name,
                "instance": len(self._fleet),
                "lifecycle": 1,
                "connected": False,
                "armed": False,
                "on_ground": True,
                "mode": "演示",
                "world_origin": copy.deepcopy(example["world_origin"]),
                "gazebo_spawn": copy.deepcopy(example["gazebo_spawn"]),
                "detail": "演示模式，不连接仿真飞机",
            }
            self._fleet[name] = item
            return {"accepted": True, "message": "演示飞机已添加"}

    def remove_vehicle(self, name):
        with self._lock:
            item = self._fleet.get(name)
            if item is None:
                raise ValueError("vehicle does not exist: " + name)
            if item.get("armed") or not item.get("on_ground"):
                raise ValueError("飞机必须落地并上锁后才能删除")
            self._fleet.pop(name)
        return {"accepted": True, "message": "演示飞机已删除"}

    def takeoff(self, names, _altitude):
        with self._lock:
            missing = [name for name in names if name not in self._fleet]
            if missing:
                raise ValueError("飞机示例不存在：" + "、".join(missing))
            if self._status is None or not self._status.get("analysis_ready"):
                raise ValueError("请先下发任务并等待规划检查完成")
            if not self._status.get("conflict_free"):
                raise ValueError("规划存在冲突，拒绝起飞")
            if set(names) != set(self._paths):
                raise ValueError("起飞机组必须与最近一次任务批次完全一致")
            for name in names:
                self._fleet[name]["armed"] = True
                self._fleet[name]["on_ground"] = False
                self._fleet[name]["mode"] = "演示飞行"
        return {"accepted": True, "message": "演示起飞状态已更新"}

    def land(self, names):
        with self._lock:
            missing = [name for name in names if name not in self._fleet]
            if missing:
                raise ValueError("飞机示例不存在：" + "、".join(missing))
            for name in names:
                self._fleet[name]["armed"] = False
                self._fleet[name]["on_ground"] = True
                self._fleet[name]["mode"] = "演示落地"
        return {"accepted": True, "message": "演示降落状态已更新"}

    def reset(self):
        with self._lock:
            self._paths = {}
            self._speeds = {}
            self._status = None
            self._fleet = {}
        return {"accepted": True, "message": "演示机队与任务已清空"}


class RosbridgeSwarmGateway:
    """Lazy ROS1 rosbridge client for planning and fleet runtime topics."""

    def __init__(self):
        self.host = os.environ.get("SWARM_ROSBRIDGE_HOST", "127.0.0.1")
        self.port = int(os.environ.get("SWARM_ROSBRIDGE_PORT", "9090"))
        self._lock = threading.Lock()
        self._ros = None
        self._publishers = {}
        self._subscribers = {}
        self._paths = {}
        self._status = None
        self._fleet = None
        self._control_states = {}
        self._path_statuses = {}
        self._trails = {}
        self._task_publisher = None
        self._release_publisher = None
        self._last_dispatch = {}
        self._operation_lock = threading.Lock()
        self._takeoff_worker = None
        self._release_generation = 0

    def _ensure_connected(self):
        if self._ros is not None and self._ros.is_connected:
            return
        try:
            import roslibpy
        except ImportError as error:
            raise RuntimeError("roslibpy is unavailable in the Web environment") from error
        ros = roslibpy.Ros(host=self.host, port=self.port)
        ros.run(timeout=5)
        if not ros.is_connected:
            raise RuntimeError("rosbridge connection timed out")
        self._ros = ros
        self._subscribe_status()
        self._subscribe_fleet()

    def capabilities(self):
        """Return a fast, read-only view of optional ROS execution services."""
        reachable = False
        detail = "rosbridge is not reachable"
        try:
            with socket.create_connection((self.host, self.port), timeout=0.2):
                reachable = True
                detail = "rosbridge transport is reachable"
        except OSError as error:
            detail = str(error)
        with self._lock:
            connected = bool(self._ros is not None and self._ros.is_connected)
            fleet = copy.deepcopy(self._fleet)
        vehicles = (fleet or {}).get("vehicles", [])
        return {
            "rosbridge_reachable": reachable,
            "ros_connected": connected,
            "fleet_status_received": fleet is not None,
            "execution_available": connected and fleet is not None,
            "aircraft_count": len(vehicles),
            "endpoint": "{}:{}".format(self.host, self.port),
            "detail": detail,
        }

    def _subscribe_status(self):
        if "/terrain_swarm/status" in self._subscribers:
            return
        import roslibpy
        subscriber = roslibpy.Topic(
            self._ros, "/terrain_swarm/status",
            "xd_uav_planning/TerrainSwarmStatus")
        subscriber.subscribe(self._status_callback)
        self._subscribers["/terrain_swarm/status"] = subscriber

    def _subscribe_path(self, uav_name):
        topic_name = "/{}/planning/terrain_path".format(uav_name)
        if topic_name in self._subscribers:
            return
        import roslibpy
        subscriber = roslibpy.Topic(self._ros, topic_name, "nav_msgs/Path")
        subscriber.subscribe(
            lambda message, name=uav_name: self._path_callback(name, message))
        self._subscribers[topic_name] = subscriber

    def _subscribe_fleet(self):
        topic_name = "/terrain_swarm/fleet"
        if topic_name in self._subscribers:
            return
        import roslibpy
        subscriber = roslibpy.Topic(
            self._ros, topic_name, "xd_uav_planning/TerrainFleetStatus")
        subscriber.subscribe(self._fleet_callback)
        self._subscribers[topic_name] = subscriber

    def _subscribe_runtime(self, uav_name):
        import roslibpy
        control_topic = "/{}/control_manager/state".format(uav_name)
        if control_topic not in self._subscribers:
            subscriber = roslibpy.Topic(
                self._ros, control_topic, "xd_uav_controller/ControlState")
            subscriber.subscribe(
                lambda message, name=uav_name: self._control_callback(name, message))
            self._subscribers[control_topic] = subscriber
        path_topic = "/{}/controller/path_status".format(uav_name)
        if path_topic not in self._subscribers:
            subscriber = roslibpy.Topic(
                self._ros, path_topic, "xd_uav_controller/PathStatus")
            subscriber.subscribe(
                lambda message, name=uav_name: self._path_status_callback(name, message))
            self._subscribers[path_topic] = subscriber

    def _ros_timestamp(self):
        """Read ROS time so Gazebo /use_sim_time requests stay fresh."""
        import roslibpy
        service = roslibpy.Service(self._ros, "/rosapi/get_time", "rosapi/GetTime")
        response = service.call(roslibpy.ServiceRequest({}), timeout=5)
        stamp = response.get("time", {})
        seconds = int(stamp.get("secs", 0))
        nanoseconds = int(stamp.get("nsecs", 0))
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            raise RuntimeError("rosapi returned an invalid ROS time")
        timestamp = seconds + nanoseconds / 1_000_000_000.0
        if timestamp <= 0.0:
            raise RuntimeError("ROS time is not active yet")
        return timestamp

    def _status_callback(self, message):
        vehicles = message.get("vehicles", [])
        received = {
            str(item.get("uav_name", "")): int(
                (item.get("planner_status") or {}).get("goal_id", 0))
            for item in vehicles
        }
        with self._lock:
            expected = self._last_dispatch
            if (expected and received == expected):
                self._status = copy.deepcopy(message)

    def _fleet_callback(self, message):
        vehicles = message.get("vehicles", [])
        with self._lock:
            self._fleet = copy.deepcopy(message)
            active = {str(item.get("uav_name", "")) for item in vehicles}
            self._control_states = {
                name: value for name, value in self._control_states.items()
                if name in active}
            self._path_statuses = {
                name: value for name, value in self._path_statuses.items()
                if name in active}
            self._trails = {
                name: value for name, value in self._trails.items()
                if name in active}
        for item in vehicles:
            name = str(item.get("uav_name", ""))
            if name:
                self._subscribe_runtime(name)
        stale_names = {
            topic.split("/")[1]
            for topic in list(self._subscribers)
            if topic.startswith("/") and (
                topic.endswith("/control_manager/state")
                or topic.endswith("/controller/path_status"))
        } - active
        for name in stale_names:
            for suffix in ("/control_manager/state", "/controller/path_status"):
                topic_name = "/{}{}".format(name, suffix)
                subscriber = self._subscribers.pop(topic_name, None)
                if subscriber is not None:
                    subscriber.unsubscribe()

    def _control_callback(self, uav_name, message):
        with self._lock:
            self._control_states[uav_name] = copy.deepcopy(message)
            fleet = self._fleet or {}
            vehicle = next((item for item in fleet.get("vehicles", [])
                            if item.get("uav_name") == uav_name), None)
            position = message.get("position_odom", {})
            origin = (vehicle or {}).get("world_origin", {})
            if not origin or not position:
                return
            point = {
                "x": float(origin.get("x", 0.0)) + float(position.get("x", 0.0)),
                "y": float(origin.get("y", 0.0)) + float(position.get("y", 0.0)),
                "z": float(origin.get("z", 0.0)) + float(position.get("z", 0.0)),
            }
            trail = self._trails.setdefault(uav_name, [])
            if (not trail or sum((point[key] - trail[-1][key]) ** 2
                                 for key in ("x", "y", "z")) >= 4.0):
                trail.append(point)
                del trail[:-300]

    def _path_status_callback(self, uav_name, message):
        with self._lock:
            self._path_statuses[uav_name] = copy.deepcopy(message)

    def _path_callback(self, uav_name, message):
        poses = message.get("poses") or []
        goal_id = int(
            (poses[0].get("header") or {}).get("seq", 0)
            if poses else (message.get("header") or {}).get("seq", 0))
        with self._lock:
            if self._last_dispatch.get(uav_name) == goal_id:
                self._paths[uav_name] = copy.deepcopy(message)

    def dispatch(self, request):
        self._ensure_connected()
        import roslibpy
        timestamp = self._ros_timestamp()
        deadline = time.monotonic() + 5.0
        requested_names = {route.uav_name for route in request.routes}
        while time.monotonic() < deadline:
            with self._lock:
                fleet = copy.deepcopy(self._fleet)
            online = {
                item.get("uav_name") for item in (fleet or {}).get("vehicles", [])
                if int(item.get("lifecycle", 0)) == 1 and item.get("connected")
            }
            if requested_names <= online:
                break
            time.sleep(0.1)
        else:
            missing = sorted(requested_names - online)
            raise RuntimeError("vehicles are not ready: " + ", ".join(missing))
        vehicles = {
            item.get("uav_name"): item
            for item in (fleet or {}).get("vehicles", [])}
        airborne = sorted(
            name for name in requested_names
            if vehicles.get(name, {}).get("armed")
            or not vehicles.get(name, {}).get("on_ground", False))
        if airborne:
            raise RuntimeError(
                "cannot replace routes while aircraft are airborne: "
                + ", ".join(airborne))

        expected_dispatch = {
            route.uav_name: int(route.goal_id) for route in request.routes}
        with self._lock:
            previous_dispatch = copy.deepcopy(self._last_dispatch)
        reused_goals = sorted(
            name for name, goal_id in expected_dispatch.items()
            if previous_dispatch.get(name) == goal_id)
        if reused_goals:
            raise RuntimeError(
                "new dispatch must use fresh goal IDs for: "
                + ", ".join(reused_goals))
        self._publish_release(False)
        with self._lock:
            self._release_generation += 1
            self._last_dispatch = expected_dispatch
            self._paths = {}
            self._status = None
        # Subscribe before publishing new tasks. Latched paths from a previous
        # batch may arrive immediately, but _path_callback rejects their goal.
        for route in request.routes:
            self._subscribe_path(route.uav_name)
        if self._task_publisher is None:
            self._task_publisher = roslibpy.Topic(
                self._ros, "/terrain_swarm/tasks",
                "xd_uav_planning/TerrainTaskBatch", latch=True)
            self._task_publisher.advertise()
        seconds = int(timestamp)
        nanoseconds = int(round((timestamp - seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds = 0
        stamp = {"secs": seconds, "nsecs": nanoseconds}
        self._task_publisher.publish(roslibpy.Message({
            "header": {"seq": 0, "stamp": stamp, "frame_id": request.frame_id},
            "tasks": [{
                "uav_name": route.uav_name,
                "goal_id": int(route.goal_id),
                "nominal_speed_mps": float(route.nominal_speed_mps),
            } for route in request.routes],
        }))
        time.sleep(0.25)
        dispatched = []
        for route in request.routes:
            topic_name = "/{}/planning/task_path".format(route.uav_name)
            publisher = self._publishers.get(topic_name)
            if publisher is None:
                publisher = roslibpy.Topic(self._ros, topic_name, "nav_msgs/Path")
                publisher.advertise()
                self._publishers[topic_name] = publisher
            publisher.publish(roslibpy.Message(build_path_message(
                route, frame_id=request.frame_id, timestamp=timestamp)))
            dispatched.append({
                "uav_name": route.uav_name,
                "goal_id": route.goal_id,
                "topic": topic_name,
                "point_count": len(route.points),
            })
        return {"mode": "ros", "dispatched": dispatched, **self.snapshot()}

    def snapshot(self):
        self._ensure_connected()
        with self._lock:
            expected = copy.deepcopy(self._last_dispatch)
            paths = {
                name: copy.deepcopy(message)
                for name, message in self._paths.items()
                if name in expected and int(
                    ((message.get("poses") or [{}])[0].get("header") or {})
                    .get("seq", 0)) == expected[name]
            }
            return {
                "mode": "ros",
                "rosbridge": {"host": self.host, "port": self.port},
                "status": copy.deepcopy(self._status),
                "paths": paths,
            }

    def fleet_snapshot(self):
        self._ensure_connected()
        with self._lock:
            fleet = copy.deepcopy(self._fleet) or {
                "capacity": 0, "vehicles": []}
            controls = copy.deepcopy(self._control_states)
            path_statuses = copy.deepcopy(self._path_statuses)
            trails = copy.deepcopy(self._trails)
        runtime = {}
        for vehicle in fleet.get("vehicles", []):
            name = vehicle.get("uav_name")
            control = controls.get(name) or {}
            position = control.get("position_odom") or {}
            origin = vehicle.get("world_origin") or {}
            world_position = None
            if position and origin:
                world_position = {
                    key: float(origin.get(key, 0.0)) + float(position.get(key, 0.0))
                    for key in ("x", "y", "z")
                }
            runtime[name] = {
                "world_position": world_position,
                "trail": trails.get(name, []),
                "state_valid": bool(control.get("state_valid", False)),
                "stable": bool(control.get("stable", False)),
                "airspeed_mps": float(control.get("airspeed", 0.0)),
                "groundspeed_mps": float(control.get("groundspeed", 0.0)),
                "path_status": path_statuses.get(name),
            }
        return {"mode": "ros", **fleet, "runtime": runtime}

    def spawn_vehicle(self, example):
        self._ensure_connected()
        import roslibpy
        service = roslibpy.Service(
            self._ros, "/terrain_swarm/fleet/spawn",
            "xd_uav_planning/SpawnTerrainVehicle")
        response = service.call(roslibpy.ServiceRequest({
            "uav_name": example["uav_name"],
            "world_origin": example["world_origin"],
            "gazebo_spawn": example["gazebo_spawn"],
        }), timeout=10)
        if not response.get("accepted"):
            raise RuntimeError(response.get("message", "vehicle spawn rejected"))
        return response

    def remove_vehicle(self, name):
        self._ensure_connected()
        import roslibpy
        service = roslibpy.Service(
            self._ros, "/terrain_swarm/fleet/remove",
            "xd_uav_planning/RemoveTerrainVehicle")
        response = service.call(
            roslibpy.ServiceRequest({"uav_name": name}), timeout=25)
        if not response.get("accepted"):
            raise RuntimeError(response.get("message", "vehicle removal rejected"))
        return response

    def _publish_release(self, enabled):
        import roslibpy
        if self._release_publisher is None:
            self._release_publisher = roslibpy.Topic(
                self._ros, "/terrain_swarm/release", "std_msgs/Bool", latch=True)
            self._release_publisher.advertise()
        self._release_publisher.publish(roslibpy.Message({"data": bool(enabled)}))

    def prepare_restart(self):
        """Fail closed unless every managed aircraft is safely on ground."""
        self._ensure_connected()
        with self._operation_lock:
            with self._lock:
                fleet = copy.deepcopy(self._fleet) or {}
            unsafe = [
                item.get("uav_name", "unknown")
                for item in fleet.get("vehicles", [])
                if int(item.get("lifecycle", 0)) != 1
                or not item.get("connected")
                or item.get("armed")
                or not item.get("on_ground")
            ]
            if unsafe:
                raise RuntimeError(
                    "以下飞机尚未确认已落地、上锁且连接正常："
                    + "、".join(sorted(unsafe)))
            self._publish_release(False)
            with self._lock:
                self._release_generation += 1
        return {"accepted": True, "message": "安全状态已确认，准备重启"}

    def _takeoff_precheck(self, names):
        with self._lock:
            fleet = copy.deepcopy(self._fleet) or {}
            status = copy.deepcopy(self._status) or {}
            dispatch = copy.deepcopy(self._last_dispatch)
        vehicles = {
            item.get("uav_name"): item for item in fleet.get("vehicles", [])}
        problems = []
        for name in names:
            item = vehicles.get(name)
            if item is None:
                problems.append(name + " 不在线")
                continue
            if int(item.get("lifecycle", 0)) != 1 or not item.get("connected"):
                problems.append(name + " 尚未就绪")
            elif item.get("armed") or not item.get("on_ground"):
                problems.append(name + " 不处于已落地且未解锁状态")
        if set(names) != set(dispatch):
            problems.append("起飞机组必须与最近一次下发的任务批次完全一致")
        if not status.get("analysis_ready"):
            problems.append("航迹安全检查尚未完成")
        elif not status.get("conflict_free"):
            problems.append("航迹安全检查发现冲突")
        status_goals = {
            item.get("uav_name"): int(
                (item.get("planner_status") or {}).get("goal_id", 0))
            for item in status.get("vehicles", [])
        }
        if any(status_goals.get(name) != dispatch.get(name) for name in names):
            problems.append("安全检查结果与最近下发任务不一致")
        if problems:
            raise RuntimeError("；".join(problems))

    def _wait_and_release(self, names, generation):
        deadline = time.monotonic() + 150.0
        while time.monotonic() < deadline:
            with self._lock:
                fleet = copy.deepcopy(self._fleet) or {}
                current_generation = self._release_generation
            if generation != current_generation:
                return
            by_name = {
                item.get("uav_name"): item
                for item in fleet.get("vehicles", [])}
            if all(
                    by_name.get(name, {}).get("armed")
                    and not by_name.get(name, {}).get("on_ground")
                    for name in names):
                with self._lock:
                    if generation != self._release_generation:
                        return
                self._publish_release(True)
                return
            time.sleep(0.2)
        try:
            self.land(names)
        except (ConnectionError, OSError, RuntimeError, ValueError):
            pass

    def takeoff(self, names, altitude):
        self._ensure_connected()
        with self._operation_lock:
            if self._takeoff_worker is not None and self._takeoff_worker.is_alive():
                raise RuntimeError("已有起飞流程正在执行")
            self._takeoff_precheck(names)
            self._publish_release(False)
            with self._lock:
                self._release_generation += 1
                generation = self._release_generation
            import roslibpy
            accepted = []
            try:
                for name in names:
                    service = roslibpy.Service(
                        self._ros, "/{}/control_manager/takeoff".format(name),
                        "xd_uav_controller/Takeoff")
                    response = service.call(roslibpy.ServiceRequest({
                        "altitude": float(altitude)}), timeout=10)
                    if not response.get("success"):
                        raise RuntimeError("{} 起飞被拒绝：{}".format(
                            name, response.get("message", "未提供原因")))
                    accepted.append(name)
            except Exception:
                if accepted:
                    try:
                        self.land(accepted)
                    except Exception:
                        pass
                raise
            self._takeoff_worker = threading.Thread(
                target=self._wait_and_release,
                args=(tuple(names), generation),
                name="terrain-swarm-takeoff",
                daemon=True)
            self._takeoff_worker.start()
        return {
            "accepted": True,
            "message": "起飞请求已由控制管理器接受；全部离地后自动放行航迹",
        }

    def land(self, names):
        self._ensure_connected()
        import roslibpy
        with self._lock:
            self._release_generation += 1
            fleet = copy.deepcopy(self._fleet) or {}
        self._publish_release(False)
        by_name = {
            item.get("uav_name"): item
            for item in fleet.get("vehicles", [])}
        targets = [
            name for name in names
            if name not in by_name
            or by_name[name].get("armed")
            or not by_name[name].get("on_ground", False)
        ]
        if not targets:
            return {
                "accepted": True,
                "message": "所选飞机均已落地并上锁，无需重复降落",
            }
        failures = []
        for name in targets:
            service = roslibpy.Service(
                self._ros, "/{}/control_manager/land".format(name),
                "std_srvs/Trigger")
            try:
                response = service.call(roslibpy.ServiceRequest({}), timeout=10)
                if not response.get("success"):
                    failures.append("{}：{}".format(
                        name, response.get("message", "降落被拒绝")))
            except Exception as error:
                failures.append("{}：{}".format(name, error))
        if failures:
            raise RuntimeError("；".join(failures))
        return {
            "accepted": True,
            "message": (
                "已向 {} 架飞机发出降落请求；固定翼会继续飞行建立进近，"
                "请以‘已落地、未解锁’为完成标志"
            ).format(len(targets)),
        }


MOCK_GATEWAY = MockSwarmGateway()
ROS_GATEWAY = RosbridgeSwarmGateway()


def get_gateway(use_mock):
    return MOCK_GATEWAY if use_mock else ROS_GATEWAY
