#!/usr/bin/env python3
"""Live validation dashboard and evidence exporter for SEAD V3-V10."""

import base64
import csv
import json
import os
import signal
import shutil
import struct
import threading
from datetime import datetime

import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from xd_uav_sead.msg import NoFlyZone


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
        self.offset_file = rospy.get_param("~offset_file", "")
        self.expected_run_id = rospy.get_param("~expected_run_id", "")
        self.offsets = {i: (0.0, 0.0, 0.0) for i in (1, 2, 3)}
        self.coordinate_frame = "mavros_local_enu"
        if self.scenario == "v5":
            self.offsets = self._load_offsets(
                self.offset_file,
                self.expected_run_id,
            )
            self.coordinate_frame = "sead_shared_enu"
        elif self.scenario == "v10":
            self.offsets = {1: (0.0, -12.0, 0.0), 2: (0.0, 0.0, 0.0), 3: (0.0, 12.0, 0.0)}
            self.coordinate_frame = "world"
        self.paths = {i: [] for i in (1, 2, 3)}
        self.events = []
        self.targets = []
        self.uav_targets = {}
        self.zones = {}
        self.assignments = {}
        self.acks = set()
        self.common_hit_time = None
        self.dpga = {}
        self.formation_config = {}
        self.formation_point = None
        self.saved = False

        count = {"v3": 1, "v4": 2, "v5": 3, "v9": 1, "v10": 3}.get(self.scenario, 0)
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
        if self.scenario == "v9":
            rospy.Subscriber(
                "/uav1/dynamic_nofly_zone",
                NoFlyZone,
                self._dynamic_nofly,
                queue_size=20,
            )
        elif self.scenario == "v10":
            rospy.Subscriber(
                "/sead/v10/dynamic_nofly_zone",
                NoFlyZone,
                self._dynamic_nofly,
                queue_size=20,
            )

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

    @staticmethod
    def _load_offsets(path, expected_run_id):
        if not path or not os.path.isfile(path):
            raise RuntimeError("V5 offset file missing: %s" % path)
        values = {}
        with open(path, "r", encoding="utf-8") as handle:
            for raw in handle:
                line = raw.strip()
                if not line or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()
        actual_run_id = values.get("VALIDATION_RUN_ID", "")
        if expected_run_id and actual_run_id != expected_run_id:
            raise RuntimeError(
                "V5 offset run mismatch: expected=%s actual=%s"
                % (expected_run_id, actual_run_id)
            )
        result = {}
        for uid in (1, 2, 3):
            try:
                result[uid] = tuple(
                    float(values["U%d_%s" % (uid, axis)])
                    for axis in ("X", "Y", "Z")
                )
            except (KeyError, ValueError) as exc:
                raise RuntimeError("invalid V5 offset for uav%d: %s" % (uid, exc))
        return result

    def _elapsed(self):
        return max(0.0, rospy.Time.now().to_sec() - self.started)

    def _event(self, name, **fields):
        with self.lock:
            item = {"t": self._elapsed(), "event": name}
            item.update(fields)
            self.events.append(item)

    def _odom(self, msg, uid):
        p = msg.pose.pose.position
        offset = self.offsets[uid]
        shared = (p.x + offset[0], p.y + offset[1], p.z + offset[2])
        with self.lock:
            self.paths[uid].append(
                (self._elapsed(), shared[0], shared[1], shared[2], p.x, p.y, p.z)
            )

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
                points = [list(p[:2]) for p in info.get("targets", [])]
                if self.scenario in ("v9", "v10"):
                    self.uav_targets[uid] = points
                else:
                    self.targets = points
            elif msg_id == 19:
                point = info.get("point", [])
                if len(point) >= 2:
                    if self.scenario in ("v9", "v10"):
                        self.uav_targets.setdefault(uid, []).append(list(point[:2]))
                    else:
                        self.targets.append(list(point[:2]))
            elif msg_id == 24:
                self.formation_config = dict(info.get("params", {}))
            elif msg_id == 26:
                point = info.get("point", [])
                if len(point) >= 2:
                    self.formation_point = list(point)

    def _dynamic_nofly(self, msg):
        with self.lock:
            if msg.operation == NoFlyZone.OP_CLEAR:
                self.zones.clear()
            elif msg.operation == NoFlyZone.OP_REMOVE or not msg.enabled:
                self.zones.pop(int(msg.zone_id), None)
            elif msg.operation == NoFlyZone.OP_UPSERT:
                self.zones[int(msg.zone_id)] = {
                    "vertices": [[float(p.x), float(p.y)] for p in msg.polygon.points],
                    "minAlt": float(msg.min_altitude),
                    "maxAlt": float(msg.max_altitude),
                    "valid_until": msg.valid_until.to_sec(),
                    "source": "dynamic_nofly_zone",
                }
        self._event(
            "dynamic no-fly zone",
            operation=int(msg.operation),
            zone_id=int(msg.zone_id),
        )

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
            now = rospy.Time.now().to_sec()
            expired = [
                zid
                for zid, zone in self.zones.items()
                if float(zone.get("valid_until", 0.0) or 0.0) > 0.0
                and now >= float(zone["valid_until"])
            ]
            for zid in expired:
                self.zones.pop(zid, None)
                self.events.append(
                    {"t": self._elapsed(), "event": "no-fly zone expired", "zone_id": zid}
                )
            paths = {k: list(v) for k, v in self.paths.items()}
            zones = dict(self.zones)
            targets = list(self.targets)
            uav_targets = {
                uid: [list(point) for point in points]
                for uid, points in self.uav_targets.items()
            }
            assignments = dict(self.assignments)
            acks = set(self.acks)
            dpga = dict(self.dpga)
            hit = self.common_hit_time
            events = list(self.events)
            formation_config = dict(self.formation_config)
            formation_point = list(self.formation_point) if self.formation_point else None

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
        for uid, points in sorted(uav_targets.items()):
            for idx, point in enumerate(points, 1):
                self.map_ax.scatter(
                    point[0], point[1], marker="x", s=90,
                    color=COLORS.get(uid, "black"),
                )
                self.map_ax.text(
                    point[0], point[1], " U%d-T%d" % (uid, idx),
                    color=COLORS.get(uid, "black"),
                )
        if formation_point is not None:
            self.map_ax.scatter(
                formation_point[0], formation_point[1], marker="P", s=110,
                color="purple", label="formation center",
            )
            self.map_ax.text(
                formation_point[0], formation_point[1], " rally",
                color="purple",
            )
        for uid, assignment in assignments.items():
            point = assignment["point"]
            self.map_ax.scatter(point[0], point[1], marker="*", s=140, color=COLORS.get(uid, "black"))
            self.map_ax.text(point[0], point[1], " uav%d→T%d" % (uid, assignment["target_id"]))
        if any(paths.values()) or zones or targets or uav_targets or assignments or formation_point:
            self.map_ax.legend(loc="best")

        self.info_ax.axis("off")
        lines = ["%s   elapsed %.1f s" % (self.scenario.upper(), self._elapsed())]
        if self.scenario in ("v3", "v4", "v5"):
            lines.append("frame: %s" % self.coordinate_frame)
            for uid, samples in paths.items():
                if samples:
                    last = samples[-1]
                    lines.append("uav%d: x=%+.2f y=%+.2f z=%+.2f  samples=%d" % (uid, last[1], last[2], last[3], len(samples)))
            if self.scenario == "v5":
                lines.append("VEE center/reference: uav2")
                if formation_point:
                    lines.append(
                        "rally: (%+.2f,%+.2f,%+.2f)"
                        % tuple((formation_point + [0.0, 0.0, 0.0])[:3])
                    )
                latest = {
                    uid: samples[-1]
                    for uid, samples in paths.items()
                    if samples
                }
                if all(uid in latest for uid in (1, 2, 3)):
                    def distance(a, b):
                        dx = latest[a][1] - latest[b][1]
                        dy = latest[a][2] - latest[b][2]
                        return (dx * dx + dy * dy) ** 0.5
                    lines.append(
                        "d(2,1)=%.2f  d(2,3)=%.2f  d(1,3)=%.2f m"
                        % (distance(2, 1), distance(2, 3), distance(1, 3))
                    )
                spacing = float(formation_config.get("spacing", 4.0))
                expected_leg = spacing * ((0.95 ** 2 + 0.82 ** 2) ** 0.5)
                expected_wings = spacing * 1.64
                lines.append(
                    "expected VEE: legs=%.2f/%.2f wings=%.2f m"
                    % (expected_leg, expected_leg, expected_wings)
                )
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
        elif self.scenario in ("v9", "v10"):
            lines.append("dynamic no-fly zones: %s" % (sorted(zones) or "等待动态禁飞区"))
            for zid, zone in sorted(zones.items()):
                lines.append(
                    "zone %s: vertices=%d alt=[%s,%s]"
                    % (
                        zid,
                        len(zone.get("vertices", [])),
                        zone.get("minAlt"),
                        zone.get("maxAlt"),
                    )
                )
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
                "uav_targets": self.uav_targets,
                "assignments": self.assignments,
                "acks": sorted(self.acks),
                "common_hit_time": self.common_hit_time,
                "dpga": self.dpga,
                "coordinate_frame": self.coordinate_frame,
                "offsets": {str(k): list(v) for k, v in self.offsets.items()},
                "formation_config": self.formation_config,
                "formation_point": self.formation_point,
            }
            paths = {k: list(v) for k, v in self.paths.items()}
        with open(os.path.join(self.output_dir, "events.json"), "w", encoding="utf-8") as handle:
            json.dump(snapshot, handle, ensure_ascii=False, indent=2)
        with open(os.path.join(self.output_dir, "trajectories.csv"), "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "uav_id", "elapsed_s",
                "shared_x_m", "shared_y_m", "shared_z_m",
                "raw_odom_x_m", "raw_odom_y_m", "raw_odom_z_m",
            ])
            for uid, samples in paths.items():
                for row in samples:
                    writer.writerow([uid] + list(row))
        if self.scenario == "v5" and self.offset_file:
            shutil.copy2(
                self.offset_file,
                os.path.join(self.output_dir, "offsets.env"),
            )
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
