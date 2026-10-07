#!/usr/bin/env python3
"""Collect gimbal sweep ID continuity and pixel-assist diagnostics."""

import json
import math
import os
import threading
import time
from collections import deque

import rospy
import tf.transformations as transformations
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from xd_uav_track.msg import GimbalState, TrackStateArray


class GimbalTrackingMetrics:
    def __init__(self):
        self.source_name = str(rospy.get_param("~image_source", "search_gimbal"))
        self.output_path = str(rospy.get_param("~output_path", "")).strip()
        self.match_gate_m = float(rospy.get_param("~truth_match_gate_m", 10.0))
        self.lock = threading.Lock()
        self.started_wall = time.monotonic()
        self.odom = deque(maxlen=40)
        self.truth = None
        self.last_track_id = {}
        self.target_id_history = {}
        self.id_switches = {}
        self.target_sightings = {}
        self.track_messages = 0
        self.frames_with_confirmed_detection = 0
        self.detected_track_observations = 0
        self.metric_track_observations = 0
        self.truth_matched_observations = 0
        self.truth_match_errors = deque(maxlen=512)
        self.gimbal_samples = 0
        self.gimbal_min_yaw = float("inf")
        self.gimbal_max_yaw = float("-inf")
        self.detector_diagnostics = {}
        self.rospy_names = {
            "tracks": rospy.get_param("~tracks_topic", "track/tracks_by_source"),
            "odom": rospy.get_param("~odom_topic", "state_estimator/main/odom"),
            "truth": rospy.get_param("~truth_topic", "ground_truth/state_json"),
            "gimbal": rospy.get_param("~gimbal_state_topic", "track/gimbal_state"),
            "diagnostics": rospy.get_param("~diagnostics_topic", "/diagnostics"),
        }
        rospy.Subscriber(self.rospy_names["tracks"], TrackStateArray,
                         self._tracks, queue_size=20, tcp_nodelay=True)
        rospy.Subscriber(self.rospy_names["odom"], Odometry,
                         self._odometry, queue_size=40, tcp_nodelay=True)
        rospy.Subscriber(self.rospy_names["truth"], String,
                         self._ground_truth, queue_size=2, tcp_nodelay=True)
        rospy.Subscriber(self.rospy_names["gimbal"], GimbalState,
                         self._gimbal, queue_size=5, tcp_nodelay=True)
        rospy.Subscriber(self.rospy_names["diagnostics"], DiagnosticArray,
                         self._diagnostics, queue_size=2, tcp_nodelay=True)
        rospy.on_shutdown(self._write_report)
        rospy.loginfo("gimbal ID monitor ready for image_source=%s", self.source_name)

    def _odometry(self, message):
        stamp = message.header.stamp.to_sec()
        pose = message.pose.pose
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        with self.lock:
            self.odom.append((stamp, (pose.position.x, pose.position.y,
                                      pose.position.z), quaternion))

    def _ground_truth(self, message):
        try:
            value = json.loads(message.data)
            stamp = float(value["stamp"])
            vehicles = {int(row["public_id"]): tuple(float(v) for v in
                        row["position_world"][:3]) for row in value["vehicles"]}
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            self.truth = (stamp, vehicles)

    def _gimbal(self, message):
        if not message.valid or not math.isfinite(message.yaw_rad):
            return
        with self.lock:
            self.gimbal_samples += 1
            self.gimbal_min_yaw = min(self.gimbal_min_yaw, message.yaw_rad)
            self.gimbal_max_yaw = max(self.gimbal_max_yaw, message.yaw_rad)

    def _diagnostics(self, message):
        for status in message.status:
            if "sar_yolo_multi_source_detection" not in status.name:
                continue
            values = {item.key: item.value for item in status.values}
            with self.lock:
                self.detector_diagnostics = {
                    key: value for key, value in values.items()
                    if (key.startswith("source_gimbal_") or key in (
                        "backend", "device", "inference_ms_ewma",
                        "reid_processed", "reid_ms_ewma", "processed_frames"))}

    def _world_position(self, track, capture_stamp):
        if not track.range_valid or not track.has_relative_position_body:
            return None
        with self.lock:
            if not self.odom:
                return None
            odom = min(self.odom, key=lambda row: abs(row[0] - capture_stamp))
        if abs(odom[0] - capture_stamp) > 0.35:
            return None
        # DetectionCandidate stores body FRD; convert it to body FLU before
        # rotating it with the capture-time world/body attitude.
        relative = (float(track.relative_position_body[0]),
                    -float(track.relative_position_body[1]),
                    -float(track.relative_position_body[2]), 0.0)
        rotation = transformations.quaternion_matrix(odom[2])
        offset = rotation.dot(relative)
        return (odom[1][0] + offset[0], odom[1][1] + offset[1],
                odom[1][2] + offset[2])

    def _tracks(self, message):
        if message.image_source != self.source_name:
            return
        stamp = message.header.stamp.to_sec()
        with self.lock:
            self.track_messages += 1
            truth = self.truth
        confirmed = [track for track in message.tracks
                     if track.lifecycle_state == "confirmed" and
                     track.detected and not track.predicted]
        if confirmed:
            with self.lock:
                self.frames_with_confirmed_detection += 1
                self.detected_track_observations += len(confirmed)
        if truth is None or abs(truth[0] - stamp) > 0.55:
            return

        candidates = []
        for track in confirmed:
            position = self._world_position(track, stamp)
            if position is None:
                continue
            with self.lock:
                self.metric_track_observations += 1
            for public_id, gt in truth[1].items():
                distance = math.sqrt(sum((position[i] - gt[i]) ** 2
                                         for i in range(3)))
                if distance <= self.match_gate_m:
                    candidates.append((distance, public_id, int(track.track_id)))
        # Greedy one-to-one association is adequate here because the three
        # simulated vehicles keep wide separation and the gate is conservative.
        used_truth, used_tracks = set(), set()
        for distance, public_id, track_id in sorted(candidates):
            if public_id in used_truth or track_id in used_tracks:
                continue
            used_truth.add(public_id)
            used_tracks.add(track_id)
            with self.lock:
                self.truth_matched_observations += 1
                self.truth_match_errors.append(distance)
                self.target_sightings[public_id] = self.target_sightings.get(public_id, 0) + 1
                previous = self.last_track_id.get(public_id)
                if previous is not None and previous != track_id:
                    self.id_switches[public_id] = self.id_switches.get(public_id, 0) + 1
                self.last_track_id[public_id] = track_id
                self.target_id_history.setdefault(public_id, set()).add(track_id)

    def _report(self):
        with self.lock:
            pan_span = (self.gimbal_max_yaw - self.gimbal_min_yaw
                        if self.gimbal_samples else 0.0)
            return {
                "schema_version": 1,
                "image_source": self.source_name,
                "duration_sec": round(time.monotonic() - self.started_wall, 3),
                "gimbal_samples": self.gimbal_samples,
                "gimbal_pan_min_rad": (self.gimbal_min_yaw
                                        if self.gimbal_samples else None),
                "gimbal_pan_max_rad": (self.gimbal_max_yaw
                                        if self.gimbal_samples else None),
                "gimbal_pan_span_rad": pan_span,
                "track_messages": self.track_messages,
                "frames_with_confirmed_detection": self.frames_with_confirmed_detection,
                "detected_track_observations": self.detected_track_observations,
                "metric_track_observations": self.metric_track_observations,
                "ground_truth_matched_observations": self.truth_matched_observations,
                "world_match_error_p95_m": (sorted(self.truth_match_errors)[
                    int(.95 * (len(self.truth_match_errors) - 1))] if self.truth_match_errors else None),
                "id_switches_by_ground_truth_id": self.id_switches,
                "track_ids_by_ground_truth_id": {
                    str(key): sorted(value) for key, value in self.target_id_history.items()},
                "sightings_by_ground_truth_id": self.target_sightings,
                "detector_diagnostics": self.detector_diagnostics,
            }

    def _write_report(self):
        report = self._report()
        encoded = json.dumps(report, indent=2, sort_keys=True)
        rospy.loginfo("gimbal rotation validation: %s", encoded)
        if not self.output_path:
            return
        if os.path.exists(self.output_path):
            rospy.logerr("refusing to overwrite existing gimbal report: %s",
                         self.output_path)
            return
        parent = os.path.dirname(os.path.abspath(self.output_path))
        os.makedirs(parent, exist_ok=True)
        with open(self.output_path, "x", encoding="utf-8") as output:
            output.write(encoded + "\n")
        rospy.loginfo("saved gimbal rotation validation to %s", self.output_path)


def main():
    rospy.init_node("gimbal_tracking_metrics")
    GimbalTrackingMetrics()
    rospy.spin()


if __name__ == "__main__":
    main()
