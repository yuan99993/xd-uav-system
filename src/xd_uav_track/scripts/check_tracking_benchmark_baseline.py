#!/usr/bin/env python3
"""Gate the two-tank Gazebo scene against its recorded tracking baseline.

Run while tracking_benchmark.launch and tracking_benchmark_perception.launch
are active. Start scoring only after both tanks have stable confirmed tracks;
warm-up time and TensorRT model loading are deliberately excluded.
"""

import argparse
import collections
import hashlib
import json
import math
import pathlib
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from gazebo_msgs.srv import GetWorldProperties
from std_msgs.msg import String
from xd_uav_track.msg import TrackStateArray


def assign_confirmed_tracks(tracks, truth, maximum_distance_m):
    """One-to-one nearest-world-position match; unmatched tracks are errors."""
    confirmed = [track for track in tracks if track.lifecycle_state == "confirmed"]
    pairs = []
    for index, track in enumerate(confirmed):
        if not track.range_valid or not track.has_relative_position_body:
            continue
        x = float(track.relative_position_body[0])
        y = -float(track.relative_position_body[1])  # detector FRD -> world FLU
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        for public_id, point in truth.items():
            distance = math.hypot(x - point[0], y - point[1])
            if distance <= maximum_distance_m:
                pairs.append((distance, index, public_id))
    matches = {}
    used_tracks = set()
    for _, index, public_id in sorted(pairs):
        if index not in used_tracks and public_id not in matches:
            matches[public_id] = confirmed[index].track_id
            used_tracks.add(index)
    return matches, len(confirmed) - len(used_tracks)


class BaselineMonitor:
    def __init__(self, maximum_distance_m):
        self.lock = threading.Lock()
        self.maximum_distance_m = maximum_distance_m
        self.truth = {}
        self.truth_stamp = 0.0
        self.phase = ""
        self.started_at = None
        self.last_pair_at = None
        self.max_pair_gap_sec = 0.0
        self.last_alive_pair_at = None
        self.max_alive_pair_gap_sec = 0.0
        self.frames = 0
        self.pair_frames = 0
        self.alive_pair_frames = 0
        self.unmatched_confirmed = 0
        self.id_switches = 0
        self.target_ids = {}
        self.maximum_world_projection_error_m = 0.0
        self.invalid_world_measurements = 0
        self.phase_frames = collections.Counter()
        self.truth_skips = 0
        self.partial_events = []
        self.id_switch_events = []
        self.unmatched_events = []
        self.projection_outlier_events = []
        self.detector_diagnostics = {}
        self.truth_sub = rospy.Subscriber(
            "/tracking_benchmark/ground_truth/state_json", String,
            self.on_truth, queue_size=1)
        self.tracks_sub = rospy.Subscriber(
            "/tracking_benchmark/track/tracks_by_source", TrackStateArray,
            self.on_tracks, queue_size=1)
        self.diagnostics_sub = rospy.Subscriber(
            "/diagnostics", DiagnosticArray, self.on_diagnostics, queue_size=1)

    def on_diagnostics(self, message):
        for status in message.status:
            if "shared_yolo" in status.name:
                with self.lock:
                    self.detector_diagnostics = {
                        item.key: item.value for item in status.values}

    def on_truth(self, message):
        try:
            row = json.loads(message.data)
            points = {int(target["public_id"]): target["position_world"][:2]
                      for target in row["targets"]}
            phase = str(row["targets"][0]["phase"])
            stamp = float(row["stamp"])
        except (ValueError, KeyError, IndexError, TypeError):
            return
        with self.lock:
            self.truth, self.phase, self.truth_stamp = points, phase, stamp

    def on_tracks(self, message):
        if message.image_source != "benchmark_fixed":
            return
        now = time.monotonic()
        with self.lock:
            stamp = message.header.stamp.to_sec()
            if (len(self.truth) != 2 or stamp <= 0.0 or
                    abs(stamp - self.truth_stamp) > 0.30):
                self.truth_skips += 1
                return
            matches, _ = assign_confirmed_tracks(
                message.tracks, self.truth, self.maximum_distance_m)
            if self.started_at is None:
                if len(matches) != 2 or len(message.tracks) != 2:
                    return
                self.started_at = now
                self.target_ids = dict(matches)
            self.frames += 1
            self.phase_frames[self.phase] += 1
            confirmed_tracks = {
                track.track_id: track for track in message.tracks
                if track.lifecycle_state == "confirmed"}
            unmatched_ids = set(confirmed_tracks) - set(self.target_ids.values())
            self.unmatched_confirmed += len(unmatched_ids)
            required_ids = {**self.target_ids, **matches}
            alive_ids = {track.track_id for track in message.tracks
                         if track.lifecycle_state in ("confirmed", "occluded")}
            if (len(required_ids) == 2 and
                    set(required_ids.values()) <= alive_ids):
                self.alive_pair_frames += 1
                if self.last_alive_pair_at is not None:
                    self.max_alive_pair_gap_sec = max(
                        self.max_alive_pair_gap_sec, now - self.last_alive_pair_at)
                self.last_alive_pair_at = now
            if unmatched_ids and len(self.unmatched_events) < 10:
                self.unmatched_events.append({
                    "stamp": round(stamp, 3), "phase": self.phase,
                    "truth": self.truth,
                    "tracks": [[track.track_id, track.lifecycle_state,
                                list(track.relative_position_body), list(track.bbox)]
                               for track in message.tracks],
                })
            if set(self.target_ids.values()) <= set(confirmed_tracks):
                self.pair_frames += 1
                if self.last_pair_at is not None:
                    self.max_pair_gap_sec = max(
                        self.max_pair_gap_sec, now - self.last_pair_at)
                self.last_pair_at = now
            elif len(self.partial_events) < 10:
                self.partial_events.append({
                    "stamp": round(stamp, 3), "phase": self.phase,
                    "matched": matches,
                    "tracks": [[track.track_id, track.lifecycle_state,
                                track.frames_since_detection]
                               for track in message.tracks],
                })
            for public_id, track_id in matches.items():
                previous = self.target_ids.get(public_id)
                if previous is not None and previous != track_id:
                    self.id_switches += 1
                    if len(self.id_switch_events) < 10:
                        self.id_switch_events.append({
                            "stamp": round(stamp, 3), "phase": self.phase,
                            "truth_id": public_id, "old_track_id": previous,
                            "new_track_id": track_id, "truth": self.truth,
                            "tracks": [[track.track_id, track.lifecycle_state,
                                        list(track.relative_position_body),
                                        list(track.bbox)]
                                       for track in message.tracks],
                        })
                self.target_ids[public_id] = track_id
            for public_id, track_id in self.target_ids.items():
                track = confirmed_tracks.get(track_id)
                if track is None:
                    continue
                if not track.range_valid or not track.has_relative_position_body:
                    self.invalid_world_measurements += 1
                    continue
                x, y = float(track.relative_position_body[0]), -float(
                    track.relative_position_body[1])
                if not math.isfinite(x) or not math.isfinite(y):
                    self.invalid_world_measurements += 1
                    continue
                point = self.truth[public_id]
                error = math.hypot(x - point[0], y - point[1])
                self.maximum_world_projection_error_m = max(
                    self.maximum_world_projection_error_m, error)
                if (error > self.maximum_distance_m and
                        len(self.projection_outlier_events) < 10):
                    self.projection_outlier_events.append({
                        "stamp": round(stamp, 3), "phase": self.phase,
                        "truth_id": public_id, "track_id": track_id,
                        "error_m": round(error, 3)})

    def report(self):
        with self.lock:
            duration = max(0.0, time.monotonic() - self.started_at) \
                if self.started_at is not None else 0.0
            gap = self.max_pair_gap_sec
            if self.last_pair_at is not None:
                gap = max(gap, time.monotonic() - self.last_pair_at)
            alive_gap = self.max_alive_pair_gap_sec
            if self.last_alive_pair_at is not None:
                alive_gap = max(alive_gap,
                                time.monotonic() - self.last_alive_pair_at)
            return {
                "duration_sec": round(duration, 3),
                "tracker_frames": self.frames,
                "pair_confirmed_frames": self.pair_frames,
                "pair_confirmed_ratio": self.pair_frames / self.frames
                if self.frames else 0.0,
                "pair_alive_frames": self.alive_pair_frames,
                "pair_alive_ratio": self.alive_pair_frames / self.frames
                if self.frames else 0.0,
                "maximum_pair_gap_sec": round(gap, 3),
                "maximum_alive_pair_gap_sec": round(alive_gap, 3),
                "id_switches": self.id_switches,
                "unmatched_confirmed_tracks": self.unmatched_confirmed,
                "maximum_world_projection_error_m": round(
                    self.maximum_world_projection_error_m, 3),
                "invalid_world_measurements": self.invalid_world_measurements,
                "truth_to_public_track_id": dict(self.target_ids),
                "phase_frames": dict(self.phase_frames),
                "truth_sync_skips": self.truth_skips,
                "partial_events": list(self.partial_events),
                "id_switch_events": list(self.id_switch_events),
                "unmatched_events": list(self.unmatched_events),
                "projection_outlier_events": list(self.projection_outlier_events),
                "detector_diagnostics": {
                    key: self.detector_diagnostics.get(key)
                    for key in ("processed_frames", "dropped_image_backlog",
                                "rate_limited_frames", "pixel_tracking_attempts",
                                "pixel_tracking_valid", "pixel_color_valid",
                                "reid_processed", "reid_reused",
                                "inference_ms_ewma", "reid_ms_ewma")},
            }


def frozen_assets_match(report, baseline):
    return all(
        report.get("asset_sha256", {}).get(path) == digest
        for path, digest in baseline.get("asset_sha256", {}).items())


def checks_for(report, baseline, model_names):
    return {
        "frozen_assets": frozen_assets_match(report, baseline),
        "scene_models": (set(baseline["required_models"]) <= set(model_names)
                         and not set(baseline["forbidden_models"]) & set(model_names)),
        "duration": report["duration_sec"] >= baseline["minimum_duration_sec"],
        "tracker_rate": report["tracker_frames"] / max(
            0.001, report["duration_sec"]) >= baseline["minimum_tracker_hz"],
        "simultaneous_confirmed": report["pair_confirmed_ratio"] >=
        baseline["minimum_pair_confirmed_ratio"],
        "simultaneous_alive": report["pair_alive_ratio"] >=
        baseline["minimum_pair_alive_ratio"],
        "maximum_pair_gap": report["maximum_pair_gap_sec"] <=
        baseline["maximum_pair_gap_sec"],
        "maximum_alive_pair_gap": report["maximum_alive_pair_gap_sec"] <=
        baseline["maximum_alive_pair_gap_sec"],
        "id_switches": report["id_switches"] <= baseline["maximum_id_switches"],
        "false_confirmed": report["unmatched_confirmed_tracks"] <=
        baseline["maximum_unmatched_confirmed_tracks"],
        "world_projection_error": report["maximum_world_projection_error_m"] <=
        baseline["maximum_world_projection_error_m"],
        "world_measurement_valid": report["invalid_world_measurements"] == 0,
        "scenario_coverage": all(report["phase_frames"].get(phase, 0) >= 5
                                 for phase in baseline["required_phases"]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=pathlib.Path, default=pathlib.Path(
        __file__).resolve().parents[1] / "config/tracking_benchmark_baseline.json")
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--duration-sec", type=float, default=45.0)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    if args.duration_sec < 10.0:
        parser.error("--duration-sec must be at least 10 seconds")
    baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
    source_root = pathlib.Path(__file__).resolve().parents[2]
    asset_sha256 = {}
    for relative_path in baseline.get("asset_sha256", {}):
        asset = source_root / relative_path
        asset_sha256[relative_path] = hashlib.sha256(
            asset.read_bytes()).hexdigest() if asset.is_file() else None
    rospy.init_node("tracking_benchmark_baseline_check", anonymous=True)
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=10.0)
    model_names = rospy.ServiceProxy(
        "/gazebo/get_world_properties", GetWorldProperties)().model_names
    monitor = BaselineMonitor(baseline["maximum_truth_match_distance_m"])
    warmup_deadline = time.monotonic() + 60.0
    while not rospy.is_shutdown() and time.monotonic() < warmup_deadline:
        with monitor.lock:
            started_at = monitor.started_at
        if started_at is not None:
            break
        time.sleep(0.1)
    if started_at is None:
        raise RuntimeError("two stable truth-matched tracks did not appear within 60 seconds")
    while not rospy.is_shutdown() and time.monotonic() - started_at < args.duration_sec:
        time.sleep(0.1)
    report = monitor.report()
    report["asset_sha256"] = asset_sha256
    report["checks"] = checks_for(report, baseline, model_names)
    report["passed"] = all(report["checks"].values())
    encoded = json.dumps(report, indent=2, sort_keys=True)
    print(encoded)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    return 0 if report["passed"] or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
