#!/usr/bin/env python3
"""Live planning-layer view of task, replacement paths and no-fly zones."""

import csv
import json
import os
import signal
import sys
import threading
import time

import matplotlib

if not os.environ.get("DISPLAY") and not os.environ.get("MPLBACKEND"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PolygonPatch
import rospy
from nav_msgs.msg import Path
from xd_uav_controller.msg import ControlState
from xd_uav_planning.msg import NoFlyZone
from xd_uav_task_allocate.msg import PlannerStatus


class FixedwingNoFlyVisualizer:
    def __init__(self):
        self._lock = threading.RLock()
        self._started = time.monotonic()
        self._uav_name = rospy.get_param("~uav_name", "uav1")
        default_output = os.path.join(
            "/tmp", "xd_uav_planning_visualizations",
            time.strftime("%Y%m%d_%H%M%S"))
        self._output_dir = rospy.get_param("~output_dir", default_output)
        os.makedirs(self._output_dir, exist_ok=True)
        self._task_path = []
        self._active_path = []
        self._path_history = []
        self._trajectory = []
        self._zones = {}
        self._removed_zones = {}
        self._events = []
        self._last_status = None
        self._last_status_key = None
        self._mission_reached = False
        self._saved = False

        namespace = "/" + self._uav_name
        self._subscribers = [
            rospy.Subscriber(namespace + "/control_manager/state",
                             ControlState, self._state_callback, queue_size=30),
            rospy.Subscriber(namespace + "/planning/task_path",
                             Path, self._task_callback, queue_size=2),
            rospy.Subscriber(namespace + "/control/reference/path",
                             Path, self._active_path_callback, queue_size=5),
            rospy.Subscriber(namespace + "/planning/no_fly_zone",
                             NoFlyZone, self._zone_callback, queue_size=10),
            rospy.Subscriber(namespace + "/planning/status",
                             PlannerStatus, self._status_callback, queue_size=20),
        ]
        self._figure, (self._map_axis, self._info_axis) = plt.subplots(
            1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [2.2, 1.0]})
        self._figure.canvas.mpl_connect("close_event", self._close)
        self._event("visualizer started")

    def _elapsed(self):
        return time.monotonic() - self._started

    def _event(self, event, **fields):
        record = {"elapsed_s": round(self._elapsed(), 3), "event": event}
        record.update(fields)
        with self._lock:
            self._events.append(record)

    @staticmethod
    def _points(message):
        return [(float(pose.pose.position.x),
                 float(pose.pose.position.y),
                 float(pose.pose.position.z)) for pose in message.poses]

    def _state_callback(self, message):
        if not message.state_valid or self._mission_reached:
            return
        point = (self._elapsed(), float(message.position_odom.x),
                 float(message.position_odom.y),
                 float(message.position_odom.z))
        with self._lock:
            if (not self._trajectory or
                    point[0] - self._trajectory[-1][0] >= 0.08):
                self._trajectory.append(point)

    def _task_callback(self, message):
        points = self._points(message)
        with self._lock:
            self._task_path = points
        self._event("task path received", points=len(points))

    def _active_path_callback(self, message):
        points = self._points(message)
        with self._lock:
            if self._active_path:
                self._path_history.append(self._active_path)
            self._active_path = points
        self._event("controller path replaced", points=len(points),
                    controller_path_id=int(message.header.seq))

    def _zone_callback(self, message):
        zone_id = int(message.zone_id)
        with self._lock:
            if message.operation == NoFlyZone.OP_CLEAR:
                self._removed_zones.update(self._zones)
                self._zones.clear()
            elif message.operation == NoFlyZone.OP_REMOVE or not message.enabled:
                removed = self._zones.pop(zone_id, None)
                if removed is not None:
                    self._removed_zones[zone_id] = removed
            elif message.operation == NoFlyZone.OP_UPSERT:
                self._zones[zone_id] = {
                    "vertices": [(float(point.x), float(point.y))
                                 for point in message.polygon.points],
                    "min_altitude": float(message.min_altitude),
                    "max_altitude": float(message.max_altitude),
                }
                self._removed_zones.pop(zone_id, None)
        self._event("no-fly update", operation=int(message.operation),
                    zone_id=zone_id)

    def _status_callback(self, message):
        state = (int(message.goal_id), int(message.state), str(message.detail))
        with self._lock:
            key = state[:2]
            if key == self._last_status_key:
                return
            self._last_status = state
            self._last_status_key = key
            if int(message.state) == PlannerStatus.REACHED:
                self._mission_reached = True
        self._event("planner status", goal_id=state[0], state=state[1],
                    detail=state[2])

    @staticmethod
    def _plot_path(axis, points, *args, **kwargs):
        if points:
            axis.plot([point[0] for point in points],
                      [point[1] for point in points], *args, **kwargs)

    def _draw(self):
        with self._lock:
            task = list(self._task_path)
            active = list(self._active_path)
            history = [list(path) for path in self._path_history[-5:]]
            trajectory = list(self._trajectory)
            zones = dict(self._zones)
            removed_zones = dict(self._removed_zones)
            events = list(self._events[-9:])
            status = self._last_status
        self._map_axis.clear()
        self._map_axis.set_title("Fixed-wing planning / dynamic no-fly")
        self._map_axis.set_xlabel("East / x (m)")
        self._map_axis.set_ylabel("North / y (m)")
        self._map_axis.grid(True, alpha=0.25)
        self._map_axis.set_aspect("equal", adjustable="datalim")
        self._plot_path(self._map_axis, task, "--", color="0.45",
                        linewidth=1.8, label="task path")
        for index, path in enumerate(history):
            self._plot_path(self._map_axis, path, color="tab:orange",
                            alpha=0.15 + 0.10 * index, linewidth=1.0)
        self._plot_path(self._map_axis, active, color="tab:blue",
                        linewidth=2.2, label="current controller path")
        if trajectory:
            actual = [(row[1], row[2], row[3]) for row in trajectory]
            self._plot_path(self._map_axis, actual, color="tab:green",
                            linewidth=2.0, label="aircraft trajectory")
            self._map_axis.scatter(actual[-1][0], actual[-1][1], s=55,
                                   marker="^", color="tab:green")
        for zone_id, zone in sorted(zones.items()):
            vertices = zone["vertices"]
            if len(vertices) >= 3:
                self._map_axis.add_patch(PolygonPatch(
                    vertices, closed=True, facecolor="tab:red", alpha=0.25,
                    edgecolor="darkred", linewidth=2.0,
                    label="no-fly zone" if zone_id == min(zones) else None))
                center_x = sum(point[0] for point in vertices) / len(vertices)
                center_y = sum(point[1] for point in vertices) / len(vertices)
                self._map_axis.text(center_x, center_y, "NFZ %d" % zone_id,
                                    color="darkred", ha="center")
        for zone_id, zone in sorted(removed_zones.items()):
            vertices = zone["vertices"]
            if len(vertices) >= 3:
                self._map_axis.add_patch(PolygonPatch(
                    vertices, closed=True, fill=False, linestyle="--",
                    edgecolor="tab:red", alpha=0.55, linewidth=1.5,
                    label="removed no-fly zone"))
        handles, labels = self._map_axis.get_legend_handles_labels()
        if handles:
            self._map_axis.legend(loc="best")

        self._info_axis.clear()
        self._info_axis.axis("off")
        lines = ["planning-layer observer", "elapsed: %.1f s" % self._elapsed(),
                 "task points: %d" % len(task),
                 "controller replacements: %d" % len(history),
                 "active zones: %s" % (sorted(zones) or "none")]
        if status is not None:
            state_names = {
                PlannerStatus.PLANNING: "PLANNING",
                PlannerStatus.ACTIVE: "ACTIVE",
                PlannerStatus.REACHED: "REACHED",
                PlannerStatus.FAILED: "FAILED",
                PlannerStatus.BLOCKED: "BLOCKED",
            }
            lines.extend(["", "goal: %d" % status[0],
                          "state: " + state_names.get(status[1], str(status[1]))])
        lines.extend(["", "recent events:"])
        lines.extend("%7.2f  %s" % (item["elapsed_s"], item["event"])
                     for item in events)
        self._info_axis.text(0.01, 0.99, "\n".join(lines), va="top",
                             family="monospace", fontsize=9, wrap=True)
        self._figure.tight_layout()
        self._figure.canvas.draw_idle()

    def save(self):
        with self._lock:
            if self._saved:
                return
            self._saved = True
            events = list(self._events)
            trajectory = list(self._trajectory)
            zones = dict(self._zones)
            removed_zones = dict(self._removed_zones)
        self._draw()
        self._figure.savefig(os.path.join(self._output_dir, "summary.png"),
                             dpi=160, bbox_inches="tight")
        with open(os.path.join(self._output_dir, "events.json"), "w",
                  encoding="utf-8") as handle:
            json.dump({"events": events, "final_zones": zones,
                       "removed_zones": removed_zones}, handle,
                      ensure_ascii=False, indent=2)
        with open(os.path.join(self._output_dir, "trajectory.csv"), "w",
                  newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["elapsed_s", "x_m", "y_m", "z_m"])
            writer.writerows(trajectory)
        rospy.loginfo("[FIXEDWING_NOFLY_VIS] saved %s", self._output_dir)

    def _close(self, _event):
        self.save()
        rospy.signal_shutdown("visualizer window closed")

    def _signal(self, signum, _frame):
        self.save()
        rospy.signal_shutdown("signal %d" % signum)

    def run(self):
        signal.signal(signal.SIGINT, self._signal)
        signal.signal(signal.SIGTERM, self._signal)
        rate = rospy.Rate(5)
        try:
            while not rospy.is_shutdown():
                self._draw()
                if matplotlib.get_backend().lower() != "agg":
                    plt.pause(0.001)
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
        finally:
            self.save()


def main():
    rospy.init_node("fixedwing_nofly_visualizer", disable_signals=True)
    FixedwingNoFlyVisualizer().run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
