#!/usr/bin/env python3
"""Release a staged fixed-wing path batch after the safety check clears it."""

import copy
import threading

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from xd_uav_planning.msg import TerrainSwarmStatus, TerrainTaskBatch
from xd_uav_planning.terrain_swarm_gate import GateNotReady, releasable_paths


class TerrainSwarmReleaseGate:
    def __init__(self):
        self._common_frame = rospy.get_param("~common_frame", "world")
        self._task_topic = str(rospy.get_param("~task_topic", ""))
        self._vehicles = rospy.get_param("~vehicles", [])
        if not isinstance(self._vehicles, list):
            raise ValueError("vehicles must be a list")
        if len(self._vehicles) < 2 and not (self._task_topic and not self._vehicles):
            raise ValueError("vehicles must contain at least two entries")
        self._names = []
        self._paths = {}
        self._publishers = {}
        self._released = set()
        self._status = None
        self._enabled = False
        self._lock = threading.Lock()
        self._vehicle_subscribers = []
        self._configure_vehicles(self._vehicles)
        self._subscribers = []
        if self._task_topic:
            self._subscribers.append(rospy.Subscriber(
                self._task_topic, TerrainTaskBatch,
                self._task_callback, queue_size=5))
        self._subscribers.append(rospy.Subscriber(
            rospy.get_param("~status_topic", "/terrain_swarm/status"),
            TerrainSwarmStatus, self._status_callback, queue_size=5))
        self._subscribers.append(rospy.Subscriber(
            rospy.get_param("~release_topic", "/terrain_swarm/release"),
            Bool, self._release_callback, queue_size=2))

    def _configure_vehicles(self, vehicles):
        names = []
        paths = {}
        publishers = {}
        topics = []
        for vehicle in vehicles:
            for key in ("name", "staged_path_topic", "output_path_topic"):
                if key not in vehicle or not str(vehicle[key]):
                    raise ValueError("vehicle entry is missing " + key)
            name = str(vehicle["name"])
            if name in names:
                raise ValueError("vehicle names must be unique")
            names.append(name)
            paths[name] = None
            publishers[name] = rospy.Publisher(
                vehicle["output_path_topic"], Path, queue_size=1, latch=False)
            topics.append((name, vehicle["staged_path_topic"]))
        with self._lock:
            self._names = names
            self._paths = paths
            self._publishers = publishers
            self._released.clear()
            self._status = None
        self._vehicle_subscribers = [rospy.Subscriber(
            topic, Path,
            lambda message, key=name: self._path_callback(key, message),
            queue_size=2) for name, topic in topics]

    def _task_callback(self, message):
        try:
            if message.header.frame_id.lstrip("/") != self._common_frame.lstrip("/"):
                raise ValueError("task batch frame must match common_frame")
            names = set()
            goals = set()
            vehicles = []
            for task in message.tasks:
                name = str(task.uav_name)
                goal_id = int(task.goal_id)
                if not name or name in names or goal_id <= 0 or goal_id in goals:
                    raise ValueError("task batch contains an invalid or duplicate entry")
                vehicles.append({
                    "name": name,
                    "staged_path_topic": "/{}/planning/swarm_staged_path".format(name),
                    "output_path_topic": "/{}/control/reference/path".format(name),
                })
                names.add(name)
                goals.add(goal_id)
            if len(vehicles) < 2:
                raise ValueError("task batch must contain at least two vehicles")
            old_subscribers = self._vehicle_subscribers
            self._configure_vehicles(vehicles)
            for subscriber in old_subscribers:
                subscriber.unregister()
            rospy.loginfo("terrain path release gate accepted task batch for: %s",
                          ", ".join(sorted(names)))
        except (TypeError, ValueError) as error:
            rospy.logerr("rejected terrain task batch at release gate: %s", error)

    def _path_callback(self, name, message):
        with self._lock:
            if name not in self._paths:
                return
            self._paths[name] = message
        self._try_release()

    def _status_callback(self, message):
        with self._lock:
            self._status = message
        self._try_release()

    def _release_callback(self, message):
        with self._lock:
            self._enabled = bool(message.data)
            if not self._enabled:
                self._released.clear()
        self._try_release()

    def _try_release(self):
        with self._lock:
            if not self._enabled:
                return
            status = copy.deepcopy(self._status)
            paths = copy.deepcopy(self._paths)
            names = list(self._names)
            publishers = dict(self._publishers)
        try:
            batch = releasable_paths(
                status, paths, names, self._common_frame)
        except GateNotReady as error:
            rospy.logwarn_throttle(2.0, "terrain swarm release held: %s", error)
            return
        for name, goal_id, path in batch:
            key = (name, goal_id)
            with self._lock:
                if key in self._released:
                    continue
                self._released.add(key)
            publishers[name].publish(path)
            rospy.loginfo("released screened terrain path for %s goal %d",
                          name, goal_id)


def main():
    rospy.init_node("terrain_swarm_release_gate")
    TerrainSwarmReleaseGate()
    rospy.spin()


if __name__ == "__main__":
    main()
