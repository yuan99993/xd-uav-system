#!/usr/bin/env python3
"""Strict ROS-only fixed-wing trajectory verifier for SITL.

The verifier consumes the existing tracker outputs plus the real Gazebo model,
MAVROS VFR_HUD and TF tree. A single good sample is deliberately not enough to
pass a run: a valid run must launch the model, keep a finite fresh airspeed and
maintain the complete world-to-FRD transform chain.
"""

import argparse
import json
import math
import os
import sys
import time

import rospy
import rosnode

from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import VFR_HUD, State
from xd_uav_track.msg import TrackStateArray, TrackStatus
import tf2_ros


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1,
                max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


def fit_circle(points):
    """Least-squares circle fit returning (x, y, radius), or None.

    Coordinates are centred first so the normal equations remain well behaved
    in a Gazebo world whose origin is far from the visual target.
    """
    if len(points) < 3:
        return None
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    normal = [[0.0] * 4 for _ in range(3)]
    for x, y in points:
        u, v = x - mean_x, y - mean_y
        row = (u, v, 1.0)
        rhs = -(u * u + v * v)
        for r in range(3):
            for c in range(3):
                normal[r][c] += row[r] * row[c]
            normal[r][3] += row[r] * rhs
    for pivot in range(3):
        best = max(range(pivot, 3), key=lambda row: abs(normal[row][pivot]))
        if abs(normal[best][pivot]) < 1e-9:
            return None
        normal[pivot], normal[best] = normal[best], normal[pivot]
        scale = normal[pivot][pivot]
        normal[pivot] = [value / scale for value in normal[pivot]]
        for row in range(3):
            if row == pivot:
                continue
            scale = normal[row][pivot]
            normal[row] = [normal[row][col] - scale * normal[pivot][col]
                           for col in range(4)]
    a, b, c = (normal[row][3] for row in range(3))
    radius_sq = a * a / 4.0 + b * b / 4.0 - c
    if not math.isfinite(radius_sq) or radius_sq <= 0.0:
        return None
    return mean_x - a / 2.0, mean_y - b / 2.0, math.sqrt(radius_sq)


class Verifier:
    def __init__(self, expected_speed, expected_radius, radius_tolerance,
                 model_name, tf_frames, min_displacement, min_speed,
                 minimum_flight_altitude, force_loss_node="",
                 force_loss_lock_sec=0.0):
        self.expected_speed = expected_speed
        self.expected_radius = expected_radius
        self.radius_tolerance = radius_tolerance
        self.model_name = model_name
        self.tf_frames = tf_frames
        self.min_displacement = min_displacement
        self.min_speed = min_speed
        self.status = None
        self.status_receive_time = None
        self.tracks = None
        self.samples = []
        self.last_status_token = None
        self.last_status_stamp = None
        self.out_of_order_status = 0
        self.out_of_order_track = 0
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.model_seen = False
        self.model_initial_position = None
        self.model_last_position = None
        self.max_model_displacement_m = 0.0
        self.max_model_speed_mps = 0.0
        self.model_samples = 0
        self.airspeed_mps = float("nan")
        self.airspeed_receive_time = None
        self.airspeed_samples = 0
        self.airspeed_fresh_samples = 0
        self.tf_samples = 0
        self.tf_complete_samples = 0
        self.target_model = ""
        self.truth_samples = []
        self.fcu_state = None
        self.fcu_receive_time = None
        self.offboard_samples = 0
        self.health_samples = 0
        self.flight_started = False
        self.flight_health_samples = 0
        self.flight_offboard_samples = 0
        self.minimum_flight_altitude = minimum_flight_altitude
        self.max_tf_age = 0.3
        self.force_loss_node = force_loss_node
        self.force_loss_lock_sec = max(0.0, force_loss_lock_sec)
        self.visible_lock_started = None
        self.forced_loss_attempted = False
        self.forced_loss_succeeded = False
        self.forced_loss_error = ""

    def on_fcu_state(self, message):
        self.fcu_state = message
        self.fcu_receive_time = time.monotonic()

    def on_model_states(self, message):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            return
        position = message.pose[index].position
        velocity = message.twist[index].linear
        point = (float(position.x), float(position.y), float(position.z))
        speed = math.sqrt(float(velocity.x) ** 2 +
                          float(velocity.y) ** 2 +
                          float(velocity.z) ** 2)
        if self.model_initial_position is None:
            self.model_initial_position = point
        self.model_last_position = point
        displacement = math.sqrt(sum((point[i] - self.model_initial_position[i]) ** 2
                                     for i in range(3)))
        self.max_model_displacement_m = max(self.max_model_displacement_m,
                                            displacement)
        if math.isfinite(speed):
            self.max_model_speed_mps = max(self.max_model_speed_mps, speed)
        self.model_seen = True
        self.model_samples += 1
        if point[2] >= self.minimum_flight_altitude:
            self.flight_started = True
        if self.target_model in message.name:
            target = message.pose[message.name.index(self.target_model)].position
            dx, dy = point[0] - target.x, point[1] - target.y
            self.truth_samples.append({
                "time": rospy.Time.now().to_sec(),
                "vehicle_x": point[0], "vehicle_y": point[1],
                "vehicle_z": point[2],
                "target_x": float(target.x), "target_y": float(target.y),
                "radius": math.hypot(dx, dy), "angle": math.atan2(dy, dx),
            })

    def on_airspeed(self, message):
        self.airspeed_mps = float(message.airspeed)
        self.airspeed_receive_time = time.monotonic()

    def sample_external_health(self, max_airspeed_age_sec):
        now = time.monotonic()
        offboard = (self.fcu_state is not None and self.fcu_state.connected and
                self.fcu_state.armed and self.fcu_state.mode == "OFFBOARD" and
                now - self.fcu_receive_time <= 2.5)
        self.health_samples += 1
        if offboard:
            self.offboard_samples += 1
        if self.flight_started:
            self.flight_health_samples += 1
            if offboard:
                self.flight_offboard_samples += 1
        self.airspeed_samples += 1
        if (self.airspeed_receive_time is not None and
                math.isfinite(self.airspeed_mps) and
                now - self.airspeed_receive_time <= max_airspeed_age_sec):
            self.airspeed_fresh_samples += 1

        self.tf_samples += 1
        complete = True
        for parent, child in self.tf_frames:
            try:
                if not self.tf_buffer.can_transform(
                        parent, child, rospy.Time(0), rospy.Duration(0.0)):
                    complete = False
                    break
                transform = self.tf_buffer.lookup_transform(parent, child, rospy.Time(0))
                # Only known static links may carry a zero timestamp.
                if (parent, child) in self.tf_frames[1:3]:
                    age = (rospy.Time.now() - transform.header.stamp).to_sec()
                    if transform.header.stamp.is_zero() or not -0.05 <= age <= self.max_tf_age:
                        complete = False
                        break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                complete = False
                break
        if complete:
            self.tf_complete_samples += 1

    def on_status(self, message):
        self.status_receive_time = time.monotonic()
        stamp = message.header.stamp.to_sec()
        if (self.last_status_stamp is not None and stamp > 0.0 and
                stamp < self.last_status_stamp):
            self.out_of_order_status += 1
        if stamp > 0.0:
            self.last_status_stamp = stamp
        self.status = message

    def on_tracks(self, message):
        stamp = message.header.stamp.to_sec()
        previous_stamp = (self.tracks.header.stamp.to_sec()
                          if self.tracks is not None else 0.0)
        if stamp > 0.0 and previous_stamp > 0.0 and stamp < previous_stamp:
            self.out_of_order_track += 1
        self.tracks = message

    def sample(self):
        if self.status is None:
            return
        status_stamp = self.status.header.stamp.to_sec()
        token = (status_stamp, self.status.tracking_state,
                 self.status.track_id, self.status.command_valid)
        if status_stamp > 0.0 and token == self.last_status_token:
            return
        self.last_status_token = token
        speed = math.hypot(self.status.command_forward,
                           self.status.command_left)
        sample = {
            "time": status_stamp if status_stamp > 0.0 else time.time(),
            # Use reception time for rate limits.  Gazebo can temporarily
            # slow down under headless load, making simulation timestamps
            # non-uniform even though the control callback remains bounded.
            "receive_time": self.status_receive_time or time.monotonic(),
            "profile": self.status.follower_profile,
            "tracking_state": self.status.tracking_state,
            "command_valid": bool(self.status.command_valid),
            "target_visible": bool(self.status.target_visible),
            "target_predicted": bool(self.status.target_predicted),
            "speed_mps": speed,
            "radius_m": None,
            "metric_valid": False,
            "input_age_sec": float(self.status.input_age_sec),
            "source_age_sec": None,
            "course_rad": math.atan2(self.status.command_left,
                                      self.status.command_forward),
        }
        if self.tracks is not None:
            selected = next((track for track in self.tracks.tracks
                             if track.selected), None)
            if (selected is not None and selected.range_valid and
                    selected.detected and not selected.predicted):
                position = selected.relative_position_body
                radius = math.hypot(float(position[0]), float(position[1]))
                if math.isfinite(radius):
                    sample["radius_m"] = radius
                    sample["metric_valid"] = True
                    track_stamp = self.tracks.header.stamp.to_sec()
                    if status_stamp > 0.0 and track_stamp > 0.0:
                        sample["source_age_sec"] = max(
                            0.0, status_stamp - track_stamp)
        self.samples.append(sample)

    def maybe_force_detector_loss(self):
        """Stop a test detector only after a continuous visible orbit lock."""
        if not self.force_loss_node or self.forced_loss_attempted:
            return
        stable = (self.status is not None and
                  bool(self.status.target_visible) and
                  bool(self.status.command_valid) and
                  self.status.tracking_state == "orbit")
        now = time.monotonic()
        if not stable:
            self.visible_lock_started = None
            return
        if self.visible_lock_started is None:
            self.visible_lock_started = now
            return
        if now - self.visible_lock_started < self.force_loss_lock_sec:
            return
        self.forced_loss_attempted = True
        try:
            succeeded, failed = rosnode.kill_nodes([self.force_loss_node])
            self.forced_loss_succeeded = self.force_loss_node in succeeded
            if not self.forced_loss_succeeded:
                self.forced_loss_error = "; ".join(failed) or \
                    "ROS did not confirm detector shutdown"
        except Exception as error:
            self.forced_loss_error = str(error)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/uav1")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--expected-speed", type=float, default=15.0)
    parser.add_argument("--speed-tolerance", type=float, default=2.0)
    parser.add_argument("--expected-radius", type=float, default=80.0,
                        help="orbit radius; 0 disables radius/orbit checks")
    parser.add_argument("--radius-tolerance", type=float, default=20.0)
    parser.add_argument("--minimum-valid-ratio", type=float, default=0.80)
    parser.add_argument("--minimum-fixed-wing-ratio", type=float, default=0.90)
    parser.add_argument("--minimum-speed-ratio", type=float, default=0.80)
    parser.add_argument("--minimum-metric-ratio", type=float, default=0.60)
    parser.add_argument("--minimum-target-visible-ratio", type=float,
                        default=0.60)
    parser.add_argument("--minimum-orbit-ratio", type=float, default=0.50)
    parser.add_argument("--max-course-rate-radps", type=float, default=0.60)
    parser.add_argument("--max-input-age-sec", type=float, default=0.40)
    parser.add_argument("--model-name", default="plane_catapult",
                        help="Gazebo model name used for launch checks")
    parser.add_argument("--min-launch-displacement-m", type=float, default=2.0,
                        help="minimum 3-D displacement after catapult")
    parser.add_argument("--min-launch-speed-mps", type=float, default=8.0,
                        help="minimum Gazebo model speed after catapult")
    parser.add_argument("--max-airspeed-age-sec", type=float, default=0.30)
    parser.add_argument("--minimum-airspeed-fresh-ratio", type=float,
                        default=0.90)
    parser.add_argument("--minimum-tf-ratio", type=float, default=0.95)
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--local-origin-frame", default="")
    parser.add_argument("--odom-frame", default="")
    parser.add_argument("--body-frame", default="")
    parser.add_argument("--frd-frame", default="")
    parser.add_argument("--output", default="")
    parser.add_argument("--validation-level", choices=("integration", "closed_loop"),
                        default="integration")
    parser.add_argument("--target-model", default="")
    parser.add_argument("--minimum-orbit-angle-rad", type=float, default=6.283185307)
    parser.add_argument("--max-altitude-excursion-m", type=float, default=10.0)
    parser.add_argument("--max-tf-age-sec", type=float, default=0.3)
    parser.add_argument("--minimum-flight-altitude-m", type=float, default=5.0,
                        help="minimum Gazebo world Z in the closed-loop flight window")
    parser.add_argument("--orbit-settle-sec", type=float, default=3.0,
                        help="ignore entry/capture transients before truth orbit grading")
    parser.add_argument("--require-center-hold", action="store_true",
                        help="fail if no usable center_hold trajectory is observed")
    parser.add_argument("--minimum-center-hold-angle-rad", type=float,
                        default=1.570796327,
                        help="minimum fitted center_hold angular span; short arcs are ill-conditioned")
    parser.add_argument("--maximum-center-hold-center-drift-m", type=float,
                        default=10.0,
                        help="maximum fitted orbit-centre error from target truth")
    parser.add_argument("--force-loss-node", default="",
                        help="test-only detector node to stop after stable visible orbit")
    parser.add_argument("--force-loss-lock-sec", type=float, default=3.0,
                        help="continuous visible-orbit lock required before force-loss")
    parser.add_argument("--center-hold-only", action="store_true",
                        help="grade safety/center-hold after forced loss, not post-loss visibility")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("verify_fixedwing_sitl", anonymous=True)
    namespace = args.namespace.rstrip("/")
    uav_name = namespace.strip("/")
    local_origin_frame = args.local_origin_frame or (uav_name + "/local_origin")
    odom_frame = args.odom_frame or (uav_name + "/odom")
    body_frame = args.body_frame or (uav_name + "/base_link")
    frd_frame = args.frd_frame or (uav_name + "/base_link_frd")
    tf_frames = ((args.world_frame, local_origin_frame),
                 (local_origin_frame, odom_frame),
                 (odom_frame, body_frame),
                 (body_frame, frd_frame))
    verifier = Verifier(args.expected_speed, args.expected_radius,
                        args.radius_tolerance, args.model_name, tf_frames,
                        args.min_launch_displacement_m,
                        args.min_launch_speed_mps,
                        args.minimum_flight_altitude_m,
                        args.force_loss_node,
                        args.force_loss_lock_sec)
    verifier.target_model = args.target_model
    verifier.max_tf_age = args.max_tf_age_sec
    rospy.Subscriber(namespace + "/mavros/state", State, verifier.on_fcu_state, queue_size=10)
    rospy.Subscriber(namespace + "/track/status", TrackStatus,
                     verifier.on_status, queue_size=10)
    rospy.Subscriber(namespace + "/track/tracks", TrackStateArray,
                     verifier.on_tracks, queue_size=10)
    rospy.Subscriber("/gazebo/model_states", ModelStates,
                     verifier.on_model_states, queue_size=10)
    rospy.Subscriber(namespace + "/mavros/vfr_hud", VFR_HUD,
                     verifier.on_airspeed, queue_size=10)

    deadline = time.monotonic() + max(0.1, args.duration)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        verifier.sample_external_health(args.max_airspeed_age_sec)
        verifier.sample()
        verifier.maybe_force_detector_loss()
        time.sleep(0.05)

    samples = verifier.samples
    valid = [item for item in samples if item["command_valid"]]
    fixed_wing = [item for item in samples
                  if item["profile"] == "fw_velocity_vector"]
    speed_ok = [item for item in fixed_wing
                if item["command_valid"] and
                abs(item["speed_mps"] - args.expected_speed) <=
                args.speed_tolerance]
    radius_samples = [item["radius_m"] for item in samples
                      if item["radius_m"] is not None]
    radius_in_tolerance = [value for value in radius_samples
                           if abs(value - args.expected_radius) <=
                           args.radius_tolerance]
    valid_ratio = float(len(valid)) / max(1, len(samples))
    fixed_wing_ratio = float(len(fixed_wing)) / max(1, len(samples))
    metric_samples = [item for item in samples if item["metric_valid"]]
    metric_ratio = float(len(metric_samples)) / max(1, len(samples))
    target_visible_ratio = float(sum(item["target_visible"] for item in samples)) / \
        max(1, len(samples))
    # coast/orbit is still an orbit reference: the target is propagated in
    # the inertial frame while the image source is temporarily unavailable.
    orbit_samples = [item for item in samples
                     if item["tracking_state"] in
                     ("orbit", "orbit_coast", "center_hold")]
    orbit_ratio = float(len(orbit_samples)) / max(1, len(fixed_wing))
    speed_ratio = float(len(speed_ok)) / max(1, len(fixed_wing))
    radius_ratio = float(len(radius_in_tolerance)) / max(1, len(radius_samples))
    age_samples = [max(item["input_age_sec"],
                       item["source_age_sec"] or 0.0)
                   for item in metric_samples
                   if math.isfinite(item["input_age_sec"])]

    course_rates = []
    previous = None
    for item in samples:
        if (previous is not None and item["command_valid"] and
                previous["command_valid"]):
            # Rate limits are physical wall-time limits.  ROS/Gazebo time can
            # jump or be repeated when the simulator is CPU-bound; using it
            # here creates artificial spikes even though the stream is fresh.
            dt = item["receive_time"] - previous["receive_time"]
            if dt > 1e-4:
                delta = ((item["course_rad"] - previous["course_rad"] +
                          math.pi) % (2.0 * math.pi) - math.pi)
                course_rates.append(abs(delta) / dt)
        previous = item
    max_course_rate = max(course_rates) if course_rates else float("inf")
    freshness_ok = (bool(age_samples) and
                    percentile(age_samples, 0.95) <= args.max_input_age_sec)
    radius_ok = (not args.expected_radius or
                 (radius_ratio >= args.minimum_orbit_ratio and
                  orbit_ratio >= args.minimum_orbit_ratio))
    airspeed_fresh_ratio = (float(verifier.airspeed_fresh_samples) /
                            max(1, verifier.airspeed_samples))
    tf_ratio = (float(verifier.tf_complete_samples) /
                max(1, verifier.tf_samples))
    trajectory_ok = (verifier.model_seen and
                     verifier.max_model_displacement_m >=
                     args.min_launch_displacement_m and
                     verifier.max_model_speed_mps >= args.min_launch_speed_mps)

    result = {
        "validation_level": args.validation_level,
        "proves_visual_closed_loop": False,
        "proves_vehicle_target_closed_loop": False,
        "samples": len(samples),
        "fixed_wing_samples": len(fixed_wing),
        "fixed_wing_ratio": fixed_wing_ratio,
        "command_valid_ratio": valid_ratio,
        "speed_ok_samples": len(speed_ok),
        "speed_ratio": speed_ratio,
        "metric_samples": len(metric_samples),
        "metric_ratio": metric_ratio,
        "target_visible_samples": sum(item["target_visible"] for item in samples),
        "target_visible_ratio": target_visible_ratio,
        "radius_samples": len(radius_samples),
        "radius_in_tolerance_ratio": radius_ratio,
        "orbit_ratio": orbit_ratio,
        "p95_input_age_sec": percentile(age_samples, 0.95),
        "max_course_rate_radps": max_course_rate,
        "out_of_order_status": verifier.out_of_order_status,
        "out_of_order_track": verifier.out_of_order_track,
        "coast_samples": sum(item["tracking_state"] in
                              ("coast", "orbit_coast") for item in samples),
        "orbit_samples": len(orbit_samples),
        "model_name": args.model_name,
        "model_samples": verifier.model_samples,
        "max_model_displacement_m": verifier.max_model_displacement_m,
        "max_model_speed_mps": verifier.max_model_speed_mps,
        "airspeed_samples": verifier.airspeed_samples,
        "airspeed_fresh_samples": verifier.airspeed_fresh_samples,
        "airspeed_fresh_ratio": airspeed_fresh_ratio,
        "tf_frames": [list(frame) for frame in tf_frames],
        "tf_complete_samples": verifier.tf_complete_samples,
        "tf_complete_ratio": tf_ratio,
        "forced_loss_node": args.force_loss_node,
        "forced_loss_attempted": verifier.forced_loss_attempted,
        "forced_loss_succeeded": verifier.forced_loss_succeeded,
        "forced_loss_error": verifier.forced_loss_error,
        "checks": {
            "received_samples": bool(samples),
            "fixed_wing_profile": fixed_wing_ratio >= args.minimum_fixed_wing_ratio,
            "command_valid_ratio": valid_ratio >= args.minimum_valid_ratio,
            "airspeed_bound": args.center_hold_only or
                speed_ratio >= args.minimum_speed_ratio,
            "metric_freshness": args.center_hold_only or freshness_ok,
            "metric_ratio": args.center_hold_only or
                metric_ratio >= args.minimum_metric_ratio,
            "target_visible_coverage":
                args.validation_level != "closed_loop" or bool(args.force_loss_node) or
                target_visible_ratio >= args.minimum_target_visible_ratio,
            "course_rate_bound": max_course_rate <= args.max_course_rate_radps,
            "radius_check": args.center_hold_only or radius_ok,
            "timestamp_order": verifier.out_of_order_status == 0 and
                               verifier.out_of_order_track == 0,
            "catapult_trajectory": trajectory_ok,
            "airspeed_fresh": airspeed_fresh_ratio >=
                              args.minimum_airspeed_fresh_ratio,
            "tf_chain_complete": tf_ratio >= args.minimum_tf_ratio,
            "forced_loss": not args.force_loss_node or
                (verifier.forced_loss_attempted and verifier.forced_loss_succeeded),
        },
    }
    if args.validation_level == "closed_loop":
        truth = verifier.truth_samples
        # Ground truth contains the catapult climb and the inbound capture
        # leg.  Those phases are not an orbit-radius or altitude-hold
        # requirement and can dominate a short SITL run.  Grade the physical
        # trajectory only after the tracker has continuously declared orbit
        # for the configured settling interval; retain the old altitude gate
        # as a fallback when no orbit state was produced so integration runs
        # still report useful vehicle health diagnostics.
        orbit_status_times = [item["time"] for item in samples
                              if item["command_valid"] and
                              item["tracking_state"] in ("orbit", "orbit_coast",
                                                           "center_hold")]
        flight_start_time = None
        if orbit_status_times:
            flight_start_time = min(orbit_status_times) + max(
                0.0, args.orbit_settle_sec)
        if flight_start_time is not None:
            flight_truth = [sample for sample in truth
                            if sample["time"] >= flight_start_time]
        else:
            flight_start = next((index for index, sample in enumerate(truth)
                                 if sample["vehicle_z"] >= args.minimum_flight_altitude_m),
                                len(truth))
            flight_truth = truth[flight_start:]
        angle_deltas = [(b["angle"] - a["angle"] + math.pi) % (2 * math.pi) - math.pi
                        for a, b in zip(flight_truth, flight_truth[1:])]
        angle = sum(angle_deltas)
        absolute_angle = sum(abs(delta) for delta in angle_deltas)
        truth_radius_ratio = sum(abs(s["radius"] - args.expected_radius) <= args.radius_tolerance
                                 for s in flight_truth) / max(1, len(flight_truth))
        altitude_excursion = max((s["vehicle_z"] for s in flight_truth), default=0) - min(
            (s["vehicle_z"] for s in flight_truth), default=0)
        center_hold_times = [item["time"] for item in samples
                             if item["tracking_state"] == "center_hold"]
        center_hold_truth = []
        center_hold_angle = 0.0
        center_hold_drift = None
        center_hold_radius = None
        if center_hold_times:
            start, end = min(center_hold_times), max(center_hold_times)
            center_hold_truth = [sample for sample in truth
                                 if start <= sample["time"] <= end]
            center_hold_fit = fit_circle(
                [(sample["vehicle_x"], sample["vehicle_y"])
                 for sample in center_hold_truth])
            center_hold_deltas = [
                (b["angle"] - a["angle"] + math.pi) % (2 * math.pi) - math.pi
                for a, b in zip(center_hold_truth, center_hold_truth[1:])]
            center_hold_angle = sum(abs(delta) for delta in center_hold_deltas)
            if center_hold_fit is not None and center_hold_truth:
                truth_center_x = sum(sample["target_x"] for sample in center_hold_truth) / \
                    len(center_hold_truth)
                truth_center_y = sum(sample["target_y"] for sample in center_hold_truth) / \
                    len(center_hold_truth)
                center_hold_drift = math.hypot(center_hold_fit[0] - truth_center_x,
                                               center_hold_fit[1] - truth_center_y)
                center_hold_radius = center_hold_fit[2]
        center_hold_usable = (bool(center_hold_times) and
                              center_hold_drift is not None and
                              center_hold_angle >= args.minimum_center_hold_angle_rad)
        center_hold_ok = ((not args.require_center_hold and not center_hold_times) or
                          (center_hold_usable and
                           center_hold_drift <= args.maximum_center_hold_center_drift_m))
        result.update(truth_samples=len(truth), truth_orbit_angle_rad=angle,
                      truth_orbit_absolute_angle_rad=absolute_angle,
                      flight_truth_samples=len(flight_truth),
                      orbit_grading_start_time=flight_start_time,
                      orbit_settle_sec=max(0.0, args.orbit_settle_sec),
                      truth_radius_ratio=truth_radius_ratio,
                      truth_altitude_excursion_m=altitude_excursion,
                      center_hold_samples=len(center_hold_times),
                      center_hold_truth_samples=len(center_hold_truth),
                      center_hold_angle_rad=center_hold_angle,
                      center_hold_fitted_radius_m=center_hold_radius,
                      center_hold_center_drift_m=center_hold_drift,
                      center_hold_fit_usable=center_hold_usable)
        result["checks"].update(
            target_ground_truth=bool(flight_truth) and bool(args.target_model) and
                                args.target_model != args.model_name,
            sustained_offboard=verifier.flight_offboard_samples /
                               max(1, verifier.flight_health_samples) >= 0.95,
            actual_orbit=args.center_hold_only or
                         (bool(flight_truth) and abs(angle) >= args.minimum_orbit_angle_rad),
            actual_radius=args.center_hold_only or truth_radius_ratio >= 0.9,
            altitude_hold=bool(flight_truth) and altitude_excursion <= args.max_altitude_excursion_m,
            sustained_flight=bool(flight_truth) and
                             all(s["vehicle_z"] >= args.minimum_flight_altitude_m
                                 for s in flight_truth),
            center_hold_drift=center_hold_ok)
    result["passed"] = all(result["checks"].values())
    result["proves_vehicle_target_closed_loop"] = (
        args.validation_level == "closed_loop" and result["passed"])
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    print(encoded)
    if args.output:
        output_dir = os.path.dirname(os.path.abspath(args.output))
        os.makedirs(output_dir, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
