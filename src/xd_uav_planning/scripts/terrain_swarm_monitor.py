#!/usr/bin/env python3
"""Observe namespaced terrain paths and publish fail-closed swarm screening."""

import copy
import math
import threading

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from xd_uav_task_allocate.msg import PlannerStatus

from xd_uav_planning.msg import (
    TerrainConflict,
    TerrainSwarmStatus,
    TerrainTaskBatch,
    TerrainVehicleStatus,
)
from xd_uav_planning.terrain_swarm import (
    detect_pair_conflict,
    path_length,
    release_conflict_counts,
)


def _normalize_frame(frame):
    return frame.lstrip("/")


def _points(message):
    return [
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        for pose in message.poses
    ]


def _goal_id(message):
    return int(message.poses[0].header.seq) if message.poses else 0


class TerrainSwarmMonitor:
    def __init__(self):
        self._common_frame = rospy.get_param("~common_frame", "world")
        self._minimum_separation = float(
            rospy.get_param("~minimum_separation_m", 133.33334))
        self._index_segment_length = float(
            rospy.get_param("~index_segment_length_m", 20.0))
        self._spatial_only_blocks_release = rospy.get_param(
            "~spatial_only_blocks_release", True)
        rate = float(rospy.get_param("~publication_rate_hz", 2.0))
        self._task_topic = str(rospy.get_param("~task_topic", ""))
        self._vehicles = rospy.get_param("~vehicles", [])
        self._validate_configuration(rate, allow_empty=bool(self._task_topic))

        self._lock = threading.Lock()
        self._paths = {vehicle["name"]: None for vehicle in self._vehicles}
        self._statuses = {vehicle["name"]: None for vehicle in self._vehicles}
        self._latest_status = None
        self._dirty = True
        self._status_publisher = rospy.Publisher(
            rospy.get_param("~status_topic", "/terrain_swarm/status"),
            TerrainSwarmStatus, queue_size=5, latch=True)
        self._marker_publisher = rospy.Publisher(
            rospy.get_param("~marker_topic", "/terrain_swarm/markers"),
            MarkerArray, queue_size=2, latch=True)
        self._vehicle_subscribers = []
        self._subscribe_vehicles(self._vehicles)
        self._task_subscriber = None
        if self._task_topic:
            self._task_subscriber = rospy.Subscriber(
                self._task_topic, TerrainTaskBatch,
                self._task_callback, queue_size=5)
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate), self._publish)

    def _validate_configuration(self, rate, allow_empty=False):
        if not _normalize_frame(self._common_frame):
            raise ValueError("common_frame must not be empty")
        numeric = (self._minimum_separation, self._index_segment_length, rate)
        if not all(math.isfinite(value) and value > 0.0 for value in numeric):
            raise ValueError("monitor numeric parameters must be positive and finite")
        if not isinstance(self._spatial_only_blocks_release, bool):
            raise ValueError("spatial_only_blocks_release must be boolean")
        if not isinstance(self._vehicles, list):
            raise ValueError("vehicles must be a list")
        if len(self._vehicles) < 2 and not (allow_empty and not self._vehicles):
            raise ValueError("vehicles must contain at least two entries")
        names = set()
        for vehicle in self._vehicles:
            if not isinstance(vehicle, dict):
                raise ValueError("each vehicle entry must be a dictionary")
            required = ("name", "path_topic", "status_topic", "nominal_speed_mps")
            if any(key not in vehicle for key in required):
                raise ValueError("vehicle entry is missing a required field")
            name = str(vehicle["name"])
            speed = float(vehicle["nominal_speed_mps"])
            if (not name or name in names or not vehicle["path_topic"]
                    or not vehicle["status_topic"]
                    or not math.isfinite(speed) or speed <= 0.0):
                raise ValueError("vehicle entry is invalid")
            vehicle["name"] = name
            vehicle["nominal_speed_mps"] = speed
            names.add(name)

    def _subscribe_vehicles(self, vehicles):
        subscribers = []
        for vehicle in vehicles:
            name = vehicle["name"]
            subscribers.append(rospy.Subscriber(
                vehicle["path_topic"], Path,
                lambda message, key=name: self._path_callback(key, message),
                queue_size=2))
            subscribers.append(rospy.Subscriber(
                vehicle["status_topic"], PlannerStatus,
                lambda message, key=name: self._status_callback(key, message),
                queue_size=10))
        self._vehicle_subscribers = subscribers

    def _task_callback(self, message):
        try:
            if _normalize_frame(message.header.frame_id) != _normalize_frame(
                    self._common_frame):
                raise ValueError("task batch frame must match common_frame")
            vehicles = []
            names = set()
            goals = set()
            for task in message.tasks:
                name = str(task.uav_name)
                goal_id = int(task.goal_id)
                speed = float(task.nominal_speed_mps)
                if (not name or name in names or goal_id <= 0 or goal_id in goals
                        or not math.isfinite(speed) or speed <= 0.0):
                    raise ValueError("task batch contains an invalid or duplicate entry")
                vehicles.append({
                    "name": name,
                    "path_topic": "/{}/planning/terrain_path".format(name),
                    "status_topic": "/{}/planning/terrain_status".format(name),
                    "nominal_speed_mps": speed,
                    "expected_goal_id": goal_id,
                })
                names.add(name)
                goals.add(goal_id)
            if len(vehicles) < 2:
                raise ValueError("task batch must contain at least two vehicles")
            old_subscribers = self._vehicle_subscribers
            with self._lock:
                self._vehicles = vehicles
                self._paths = {vehicle["name"]: None for vehicle in vehicles}
                self._statuses = {vehicle["name"]: None for vehicle in vehicles}
                self._latest_status = None
                self._dirty = True
            for subscriber in old_subscribers:
                subscriber.unregister()
            self._subscribe_vehicles(vehicles)
            rospy.loginfo("terrain safety check accepted task batch for: %s",
                          ", ".join(sorted(names)))
        except (TypeError, ValueError) as error:
            rospy.logerr("rejected terrain task batch: %s", error)

    def _path_callback(self, name, message):
        with self._lock:
            if name not in self._paths:
                return
            self._paths[name] = message
            self._dirty = True

    def _status_callback(self, name, message):
        with self._lock:
            if name not in self._statuses:
                return
            self._statuses[name] = message
            self._dirty = True

    def _vehicle_summary(self, vehicle, path, planner_status):
        output = TerrainVehicleStatus()
        output.uav_name = vehicle["name"]
        output.nominal_speed_mps = vehicle["nominal_speed_mps"]
        if planner_status is not None:
            output.planner_status = planner_status
        else:
            output.planner_status.header.frame_id = self._common_frame
            output.planner_status.state = PlannerStatus.IDLE
            output.planner_status.detail = "terrain_status_missing"
        if path is not None:
            try:
                values = _points(path)
                output.path_length_m = path_length(values)
                output.path_points = len(values)
                output.path_available = True
            except ValueError:
                output.path_available = False
        return output

    def _validate_vehicle(self, vehicle, path, planner_status):
        name = vehicle["name"]
        if path is None:
            return "{}:path_missing".format(name)
        if planner_status is None:
            return "{}:status_missing".format(name)
        if _normalize_frame(path.header.frame_id) != _normalize_frame(self._common_frame):
            return "{}:path_frame_mismatch".format(name)
        if len(path.poses) < 2:
            return "{}:path_too_short".format(name)
        goal_id = _goal_id(path)
        if goal_id == 0:
            return "{}:path_goal_id_missing".format(name)
        if ("expected_goal_id" in vehicle
                and goal_id != int(vehicle["expected_goal_id"])):
            return "{}:task_goal_id_mismatch".format(name)
        for pose in path.poses:
            point = pose.pose.position
            if (_normalize_frame(pose.header.frame_id)
                    != _normalize_frame(self._common_frame)):
                return "{}:pose_frame_mismatch".format(name)
            if int(pose.header.seq) != goal_id:
                return "{}:path_goal_id_mismatch".format(name)
            if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
                return "{}:path_not_finite".format(name)
        if int(planner_status.goal_id) != goal_id:
            return "{}:status_goal_id_mismatch".format(name)
        if planner_status.state != PlannerStatus.PLANNING:
            return "{}:planner_not_ready".format(name)
        if not planner_status.detail.startswith("terrain_path_ready:"):
            return "{}:terrain_path_not_ready".format(name)
        try:
            path_length(_points(path))
        except ValueError:
            return "{}:path_geometry_invalid".format(name)
        return None

    def _analyse(self, vehicles, paths, statuses):
        output = TerrainSwarmStatus()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = self._common_frame
        if len(vehicles) < 2:
            output.detail = "waiting_for_task_batch"
            return output
        for vehicle in vehicles:
            name = vehicle["name"]
            output.vehicles.append(self._vehicle_summary(
                vehicle, paths[name], statuses[name]))
            reason = self._validate_vehicle(vehicle, paths[name], statuses[name])
            if reason is not None:
                output.analysis_ready = False
                output.conflict_free = False
                output.detail = "not_ready:" + reason
                return output

        for first_index, first_vehicle in enumerate(vehicles):
            for second_vehicle in vehicles[first_index + 1:]:
                first_path = paths[first_vehicle["name"]]
                second_path = paths[second_vehicle["name"]]
                result = detect_pair_conflict(
                    _points(first_path), first_vehicle["nominal_speed_mps"],
                    _points(second_path), second_vehicle["nominal_speed_mps"],
                    self._minimum_separation, self._index_segment_length)
                if result is None:
                    continue
                conflict = TerrainConflict()
                conflict.conflict_type = (
                    TerrainConflict.SPATIOTEMPORAL
                    if result.conflict_type == "spatiotemporal"
                    else TerrainConflict.SPATIAL)
                conflict.first_uav = first_vehicle["name"]
                conflict.first_goal_id = _goal_id(first_path)
                conflict.second_uav = second_vehicle["name"]
                conflict.second_goal_id = _goal_id(second_path)
                conflict.first_position = Point(*result.first_position)
                conflict.second_position = Point(*result.second_position)
                conflict.separation_m = result.separation_m
                conflict.threshold_m = self._minimum_separation
                conflict.predicted_time_valid = result.predicted_time_s is not None
                if conflict.predicted_time_valid:
                    conflict.predicted_time_s = result.predicted_time_s
                output.conflicts.append(conflict)

        blocking_conflicts, spatial_warnings = release_conflict_counts(
            ("spatiotemporal"
             if conflict.conflict_type == TerrainConflict.SPATIOTEMPORAL
             else "spatial" for conflict in output.conflicts),
            self._spatial_only_blocks_release)
        output.analysis_ready = True
        # conflict_free is the release decision: known spatial-only records
        # remain visible but may be warnings under the configured demo policy.
        output.conflict_free = blocking_conflicts == 0
        output.detail = (
            "ready:vehicles={}:blocking_conflicts={}:spatial_warnings={}:"
            "assumption=simultaneous_start_constant_nominal_speed"
        ).format(len(output.vehicles), blocking_conflicts, spatial_warnings)
        return output

    def _markers(self, status, paths, vehicles):
        output = MarkerArray()
        reset = Marker()
        reset.action = Marker.DELETEALL
        output.markers.append(reset)
        colors = ((0.10, 0.55, 1.0), (0.10, 0.85, 0.35),
                  (1.0, 0.65, 0.10), (0.75, 0.30, 1.0))
        for index, vehicle in enumerate(vehicles):
            path = paths[vehicle["name"]]
            if path is None:
                continue
            marker = Marker()
            marker.header.stamp = status.header.stamp
            marker.header.frame_id = self._common_frame
            marker.ns = "terrain_swarm_paths"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 3.0
            marker.color.r, marker.color.g, marker.color.b = colors[index % len(colors)]
            marker.color.a = 0.9
            marker.points = [pose.pose.position for pose in path.poses]
            output.markers.append(marker)
        for index, conflict in enumerate(status.conflicts):
            marker = Marker()
            marker.header = status.header
            marker.ns = "terrain_swarm_conflicts"
            marker.id = index
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 8.0
            if conflict.conflict_type == TerrainConflict.SPATIOTEMPORAL:
                marker.color.r = 1.0
                marker.color.g = 0.05
                marker.color.b = 0.05
            else:
                marker.color.r = 1.0
                marker.color.g = 0.65
                marker.color.b = 0.10
            marker.color.a = 1.0
            marker.points = [conflict.first_position, conflict.second_position]
            output.markers.append(marker)
        return output

    def _publish(self, _event):
        with self._lock:
            if self._dirty:
                vehicles = copy.deepcopy(self._vehicles)
                paths = copy.deepcopy(self._paths)
                statuses = copy.deepcopy(self._statuses)
                self._dirty = False
            else:
                vehicles = None
                paths = None
                statuses = None
            latest = self._latest_status
        if paths is not None:
            try:
                latest = self._analyse(vehicles, paths, statuses)
            except (TypeError, ValueError) as error:
                latest = TerrainSwarmStatus()
                latest.header.stamp = rospy.Time.now()
                latest.header.frame_id = self._common_frame
                latest.detail = "analysis_failed:{}".format(error)
            with self._lock:
                self._latest_status = latest
        if latest is None:
            return
        latest.header.stamp = rospy.Time.now()
        self._status_publisher.publish(latest)
        with self._lock:
            marker_vehicles = copy.deepcopy(self._vehicles)
            marker_paths = copy.deepcopy(self._paths)
        self._marker_publisher.publish(
            self._markers(latest, marker_paths, marker_vehicles))


def main():
    rospy.init_node("terrain_swarm_monitor")
    try:
        TerrainSwarmMonitor()
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("terrain swarm monitor configuration failed: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
