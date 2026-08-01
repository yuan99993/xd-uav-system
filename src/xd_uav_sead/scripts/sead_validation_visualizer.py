#!/usr/bin/env python3
"""Live validation dashboard and evidence exporter for SEAD V3-V8."""

import base64
import csv
import json
import os
import signal
import struct
import threading
from datetime import datetime

import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


COLORS = {1: "tab:blue", 2: "tab:orange", 3: "tab:green"}


class ValidationVisualizer:
    def __init__(self):
        self.scenario = rospy.get_param("~scenario", "v4").lower()
        self.lock = threading.RLock()
        self.started = rospy.Time.now().to_sec()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        root = rospy.get_param(
            "~output_root",
            "/home/promise/catkin_ws/src/xd-uavsystem-test/.codex-tmp/visualizations",
        )
        self.output_dir = os.path.join(root, "%s_%s" % (self.scenario, stamp))
        os.makedirs(self.output_dir, exist_ok=True)
        self.paths = {i: [] for i in (1, 2, 3)}
        self.events = []
        self.targets = []
        self.zones = {}
        self.assignments = {}
        self.acks = set()
        self.common_hit_time = None
        self.dpga = {}
        self.saved = False

        count = {"v3": 1, "v4": 2, "v5": 3}.get(self.scenario, 0)
        for uid in range(1, count + 1):
            rospy.Subscriber(
                "/uav%d/mavros/local_position/odom" % uid,
                Odometry,
                self._odom,
                callback_args=uid,
                queue_size=100,
            )
        for uid in (1, 2, 3):
            rospy.Subscriber(
                "/uav%d/sead/command" % uid,
                String,
                self._command,
                callback_args=uid,
                queue_size=20,
            )
        rospy.Subscriber("/sead/u2u", String, self._u2u, queue_size=200)

        self.fig, (self.map_ax, self.info_ax) = plt.subplots(1, 2, figsize=(13, 6))
        self.fig.canvas.manager.set_window_title("SEAD %s live validation" % self.scenario.upper())
        self.fig.canvas.mpl_connect("close_event", self._close)
        self.timer = self.fig.canvas.new_timer(interval=500)
        self.timer.add_callback(self._draw)
        self.timer.start()
        rospy.on_shutdown(self.save)
        signal.signal(signal.SIGINT, self._signal)
        signal.signal(signal.SIGTERM, self._signal)
        self._event("visualizer started", scenario=self.scenario)
        rospy.loginfo("[SEAD VIS] live dashboard; output=%s", self.output_dir)

    def _elapsed(self):
        return max(0.0, rospy.Time.now().to_sec() - self.started)

    def _event(self, name, **fields):
        with self.lock:
            item = {"t": self._elapsed(), "event": name}
            item.update(fields)
            self.events.append(item)

    def _odom(self, msg, uid):
        p = msg.pose.pose.position
        with self.lock:
            self.paths[uid].append((self._elapsed(), p.x, p.y, p.z))

    def _command(self, msg, uid):
        try:
            obj = json.loads(msg.data)
            msg_id = int(obj.get("msg_id", 0))
            info = obj.get("info", {})
        except Exception as exc:
            self._event("invalid command", uav=uid, error=str(exc))
            return
        self._event("command", uav=uid, msg_id=msg_id, info=info)
        with self.lock:
            if msg_id == 20:
                self.zones.clear()
            elif msg_id == 21:
                self.zones[int(info.get("zone_id", 0))] = info
            elif msg_id == 18:
                self.targets = [list(p[:2]) for p in info.get("targets", [])]
            elif msg_id == 19:
                point = info.get("point", [])
                if len(point) >= 2:
                    self.targets.append(list(point[:2]))

    @staticmethod
    def _packet(raw):
        obj = json.loads(raw)
        return obj, base64.b64decode(obj.get("blob", ""))

    def _u2u(self, msg):
        try:
            envelope, packet = self._packet(msg.data)
            if not packet:
                return
            msg_id = packet[0]
            src = int(envelope.get("src", -1))
            self._event("u2u", src=src, msg_id=msg_id)
            if msg_id == 17 and len(packet) >= 43:
                nt = packet[36]
                chromosome = []
                if nt:
                    chromosome = [list(packet[43 + i * nt:43 + (i + 1) * nt]) for i in range(5)]
                with self.lock:
                    self.dpga[src] = {"cost": struct.unpack("i", packet[39:43])[0] * 1e-3,
                                      "chromosome": chromosome}
            elif msg_id == 27 and len(packet) >= 4:
                result = {}
                offset = 4
                for _ in range(packet[3]):
                    uid, target_id, x_mm, y_mm = struct.unpack("<BBii", packet[offset:offset + 10])
                    result[int(uid)] = {"target_id": int(target_id), "point": [x_mm * 1e-3, y_mm * 1e-3]}
                    offset += 10
                with self.lock:
                    self.assignments = result
            elif msg_id == 28 and len(packet) >= 4:
                with self.lock:
                    self.acks.add(int(packet[1]))
            elif msg_id == 30 and len(packet) >= 12:
                with self.lock:
                    self.common_hit_time = struct.unpack("<d", packet[4:12])[0]
        except Exception as exc:
            self._event("invalid u2u", error=str(exc))

    def _draw(self):
        with self.lock:
            paths = {k: list(v) for k, v in self.paths.items()}
            zones = dict(self.zones)
            targets = list(self.targets)
            assignments = dict(self.assignments)
            acks = set(self.acks)
            dpga = dict(self.dpga)
            hit = self.common_hit_time
            events = list(self.events)

        self.map_ax.clear()
        self.info_ax.clear()
        self.map_ax.set_title("Live trajectories / zones / tasks")
        self.map_ax.set_xlabel("East / x [m]")
        self.map_ax.set_ylabel("North / y [m]")
        self.map_ax.grid(True, alpha=0.3)
        self.map_ax.axis("equal")
        for uid, samples in paths.items():
            if samples:
                xs, ys = [p[1] for p in samples], [p[2] for p in samples]
                self.map_ax.plot(xs, ys, color=COLORS[uid], label="uav%d" % uid)
                self.map_ax.scatter(xs[-1], ys[-1], color=COLORS[uid], s=45)
        for zid, zone in zones.items():
            pts = zone.get("vertices", [])
            if len(pts) >= 3:
                xs = [p[0] for p in pts] + [pts[0][0]]
                ys = [p[1] for p in pts] + [pts[0][1]]
                self.map_ax.fill(xs, ys, color="tab:red", alpha=0.2, label="zone %s" % zid)
                self.map_ax.plot(xs, ys, color="tab:red")
        for idx, point in enumerate(targets, 1):
            self.map_ax.scatter(point[0], point[1], marker="x", s=80, color="black")
            self.map_ax.text(point[0], point[1], " T%d" % idx)
        for uid, assignment in assignments.items():
            point = assignment["point"]
            self.map_ax.scatter(point[0], point[1], marker="*", s=140, color=COLORS.get(uid, "black"))
            self.map_ax.text(point[0], point[1], " uav%d→T%d" % (uid, assignment["target_id"]))
        if any(paths.values()) or zones or targets or assignments:
            self.map_ax.legend(loc="best")

        self.info_ax.axis("off")
        lines = ["%s   elapsed %.1f s" % (self.scenario.upper(), self._elapsed())]
        if self.scenario in ("v3", "v4", "v5"):
            for uid, samples in paths.items():
                if samples:
                    last = samples[-1]
                    lines.append("uav%d: x=%+.2f y=%+.2f z=%+.2f  samples=%d" % (uid, last[1], last[2], last[3], len(samples)))
        elif self.scenario == "v6":
            lines.append("stored zones: %s" % (sorted(zones) or "等待 airspace_demo"))
            for zid, zone in zones.items():
                lines.append("zone %s: vertices=%d alt=[%s,%s]" % (zid, len(zone.get("vertices", [])), zone.get("minAlt"), zone.get("maxAlt")))
        elif self.scenario == "v7":
            lines.append("DPGA sources: %s" % (sorted(dpga) or "等待 dpga_demo"))
            for uid, state in sorted(dpga.items()):
                lines.append("uav%d cost=%s chromosome=%s" % (uid, state["cost"], state["chromosome"]))
            lines.append("targets: %s" % targets)
        elif self.scenario == "v8":
            lines.append("assignments: %s" % (assignments or "等待 strike_demo"))
            lines.append("ACK UAVs: %s" % sorted(acks))
            lines.append("common_hit_time: %s" % hit)
        lines.append("")
        lines.append("recent events:")
        for item in events[-10:]:
            lines.append("%7.2f  %s" % (item["t"], item["event"]))
        self.info_ax.text(0.01, 0.99, "\n".join(lines), va="top", family="monospace", fontsize=9)
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()

    def save(self):
        with self.lock:
            if self.saved:
                return
            self.saved = True
            snapshot = {
                "scenario": self.scenario,
                "events": self.events,
                "zones": self.zones,
                "targets": self.targets,
                "assignments": self.assignments,
                "acks": sorted(self.acks),
                "common_hit_time": self.common_hit_time,
                "dpga": self.dpga,
            }
            paths = {k: list(v) for k, v in self.paths.items()}
        with open(os.path.join(self.output_dir, "events.json"), "w", encoding="utf-8") as handle:
            json.dump(snapshot, handle, ensure_ascii=False, indent=2)
        with open(os.path.join(self.output_dir, "trajectories.csv"), "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["uav_id", "elapsed_s", "x_m", "y_m", "z_m"])
            for uid, samples in paths.items():
                for row in samples:
                    writer.writerow([uid] + list(row))
        try:
            self._draw()
            self.fig.savefig(os.path.join(self.output_dir, "summary.png"), dpi=160, bbox_inches="tight")
        except Exception as exc:
            rospy.logwarn("[SEAD VIS] PNG save failed: %s", exc)
        rospy.loginfo("[SEAD VIS] evidence saved: %s", self.output_dir)

    def _close(self, _event):
        self.save()
        rospy.signal_shutdown("visualizer window closed")

    def _signal(self, signum, _frame):
        self.save()
        rospy.signal_shutdown("signal %d" % signum)
        plt.close(self.fig)

    def run(self):
        duration = float(rospy.get_param("~headless_duration", 0.0))
        if duration > 0.0:
            deadline = rospy.get_time() + duration
            rate = rospy.Rate(5)
            while not rospy.is_shutdown() and rospy.get_time() < deadline:
                self._draw()
                rate.sleep()
            self.save()
            return
        plt.show()
        self.save()


if __name__ == "__main__":
    rospy.init_node("sead_validation_visualizer", anonymous=True, disable_signals=True)
    ValidationVisualizer().run()
