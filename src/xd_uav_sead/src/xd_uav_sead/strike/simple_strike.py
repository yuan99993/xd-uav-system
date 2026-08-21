#!/usr/bin/env python3

import math
import time

import dubins
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from xd_uav_sead.planning import pathFollowing as pf
from xd_uav_sead.comms.communication_info import pathFollowingMethod
from xd_uav_sead.planning.GA_SEAD_process import (
    _is_dubins_blocked,
    _point_in_poly,
    plan_path_with_avoidance,
)


class SimpleStrikeManager(object):
    """Independent 3-target simple strike flow; no GA/chromosome dependency."""

    def __init__(
        self,
        targets,
        unknown_targets,
        base_config,
        uav_id,
        uav_config,
        log_jsonl=None,
        airspace=None,
        control_mode="swiftwing_vector",
        reference_frame=None,
    ):
        self.uav_id = int(uav_id)
        self.targets = self.canonical_targets(targets)
        self.unknown_targets = unknown_targets or []
        self.base = base_config or [0.0, 0.0, 0.0]
        self.uav_type = int(uav_config[0]) if uav_config else 2
        self.cruise_speed = float(uav_config[1]) if uav_config else 20.0
        self.rmin = float(uav_config[2]) if uav_config else 35.0
        self.log_jsonl = log_jsonl
        self.airspace = airspace
        self.reference_frame = str(
            reference_frame or f"uav{self.uav_id}/local_origin"
        )
        self.path_pub = (
            rospy.Publisher(
                f"/uav{self.uav_id}/sead/planned_path",
                Path,
                queue_size=1,
                latch=True,
            )
            if rospy.core.is_initialized()
            else None
        )
        self.zones_cache = []
        self.zone_clearance = max(40.0, float(self.rmin) + 20.0)
        self.approach_zone_clearance = max(40.0, float(self.zone_clearance) - 15.0)
        self.final_zone_clearance = float(self.zone_clearance)
        self._last_planner_zones_count = None
        self._last_path_metrics_log_time = 0.0

        self.path_following = pf.CraigReynolds_Path_Following(
            getattr(pathFollowingMethod, "dubinsPath_following_" + "velocity" + "Body_PID"),
            5,
            [],
            3,
            3,
            7,
        )
        self.path_update_flag = False
        self.previous_u2u_time = 0.0
        self.previous_control_time = 0.0
        self.assignment_map = {}
        self.assigned_target = None
        self.path_length = 0.0
        self.remaining_path_length = 0.0
        self.eta = None
        self.use_true_path_eta = True
        self.mission_flag = False
        self.target_reached_reported = False
        self.path_exhausted_reported = False
        self.status_log_time = 0.0
        self.gcs_sent = set()
        self.remote_map = {}
        self.assignment_wait_start_time = 0.0
        self.assignment_wait_timeout = 1.5
        self.assignment_roster_latched = []
        self.leader_id = 1
        self.assignment_received = False
        self.assignment_locked_from_leader = False
        self.assignment_broadcast_sent = False
        self.assignment_broadcast_count = 0
        self.assignment_ack_uavs = set()
        self.assignment_missing_uavs = set()
        self.assignment_broadcast_started_at = 0.0
        self.assignment_broadcast_timeout = 10.0
        self.assignment_broadcast_interval = 0.5
        self.assignment_min_broadcast_count = 3
        self.assignment_ack_complete = False
        self.last_assignment_ack_time = 0.0
        self.assignment_ack_interval = 0.5
        self.assignment_ack_log_time = 0.0
        self.last_assignment_broadcast_time = 0.0
        self.final_target_point = None
        self.phase_heartbeat_log_time = 0.0

        control_mode = str(control_mode or "swiftwing_vector").lower()
        if control_mode in ["swiftwing", "swiftwing_vector", "speed", "speed_swiftwing"]:
            self.simple_strike_control_mode = "swiftwing_vector"
        elif control_mode in ["position", "position_waypoint", "waypoint"]:
            self.simple_strike_control_mode = "position_waypoint"
        else:
            self.simple_strike_control_mode = "swiftwing_vector"

        self.full_path_control_backend = self.simple_strike_control_mode
        if self.simple_strike_control_mode == "swiftwing_vector":
            self.phase = "FULL_PATH_SWIFTWING_VECTOR_SYNC"
            self.simple_strike_execution_mode = "full_path_swiftwing_vector_sync"
        else:
            self.phase = "FULL_PATH_POSITION_WAYPOINT_SYNC"
            self.simple_strike_execution_mode = "full_path_position_waypoint_sync"
        self.full_path_speed_sync_enabled = True
        self.full_path_expected_groundspeed = max(20.0, 1.05 * float(self.cruise_speed))
        self.full_path_speed_min = max(14.0, 0.75 * float(self.cruise_speed))
        self.full_path_speed_max = max(30.0, 1.70 * float(self.cruise_speed))
        self.full_path_speed_hard_max = 36.0
        self.full_path_speed_max = min(
            float(self.full_path_speed_max),
            float(self.full_path_speed_hard_max),
        )
        if self.full_path_speed_max < self.full_path_speed_min:
            self.full_path_speed_max = self.full_path_speed_min
        self.swiftwing_realistic_speed_max = 24.0
        self.swiftwing_command_speed_max = 26.5
        self.terminal_slowdown_distance = 240.0
        self.terminal_speed_max = 24.0
        self.terminal_speed_min = max(10.5, self.full_path_speed_min)
        self.terminal_capture_radius = max(120.0, 2.8 * float(self.rmin))
        self.terminal_cross_track_radius = max(160.0, 3.5 * float(self.rmin))
        self.terminal_relative_sync_enabled = False
        self.terminal_relative_sync_distance = 750.0
        self.terminal_sync_window = 4.0
        self.terminal_global_deadline_ignore_time_left = 12.0
        self.terminal_relative_normal_speed = 17.0
        self.terminal_relative_slow_speed = 13.0
        self.terminal_relative_fast_speed = 21.5
        self.terminal_final_hard_speed_cap_distance = 100.0
        self.terminal_final_hard_speed_cap = 21.5
        self.arrival_error_deadband = 2.0
        self.arrival_error_slow_threshold = -4.0
        self.arrival_error_fast_threshold = 4.0
        self.arrival_error_speed_gain = 0.35
        self.early_arrival_stretch_enabled = False
        self.early_arrival_stretch_distance = 700.0
        self.early_arrival_stretch_error = -5.0
        self.early_arrival_orbit_radius = max(180.0, 2.8 * float(self.rmin))
        self.min_time_left_for_speed_sync = 8.0
        self.deadline_expired_speed = max(21.0, 1.10 * float(self.cruise_speed))
        self.effective_sync_speed_max = (
            min(float(self.full_path_speed_max), float(self.swiftwing_command_speed_max))
            if self.simple_strike_control_mode == "swiftwing_vector"
            else float(self.full_path_speed_max)
        )
        self.full_path_sync_margin = 2.0
        self.full_path_tracking_time_scale = 1.00
        self.full_path_extra_start_margin = 2.0
        self.full_path_common_hit_time = 0.0
        self.full_path_start_time = 0.0
        self.full_path_started_reported = False
        self.full_path_plan_built = False
        self.full_path_path_index = 0
        self.full_path_speed_cmd_prev = float(self.cruise_speed)
        self.full_path_speed_log_time = 0.0
        self.full_path_saturation_log_time = 0.0
        self.full_path_feasibility_reported = False
        self.full_path_control_source_reported = False
        self.full_path_wait_keepalive_log_time = 0.0
        self.last_full_path_common_hit_time_broadcast_time = 0.0
        self.full_path_common_hit_time_broadcast_count = 0
        self.full_path_control_path_source = "unknown"
        self.local_path_ready_broadcast_time = 0.0
        self.team_path_status = {}
        self.path_ready_wait_started_at = 0.0
        self.path_ready_wait_timeout = 8.0
        self.path_ready_status_log_time = 0.0
        self.path_status_broadcast_interval = 0.2
        self.state_broadcast_interval = 0.2
        self.dynamic_sync_enabled = True
        self.dynamic_sync_update_interval = 1.0
        self.last_common_hit_time_update_time = 0.0
        self.common_hit_time_update_count = 0
        self.max_common_hit_time_updates = 120
        self.common_hit_time_update_threshold = 3.0
        self.common_hit_time_global_late_threshold = 2.5
        self.common_hit_time_push_margin = 1.0
        self.common_hit_time_min_push = 1.0
        self.common_hit_time_max_shift_per_update = 2.0
        self.dynamic_sync_min_groundspeed = 8.0
        self.dynamic_sync_eta_speed_floor = max(
            20.0,
            1.05 * float(self.cruise_speed),
        )
        self.dynamic_sync_shortest_feasible_margin = 2.0
        self.dynamic_sync_stop_remaining_to_capture = 35.0
        self.short_path_speed_min = max(12.0, 0.62 * float(self.cruise_speed))
        self._common_hit_time_push_blocked_log_time = 0.0
        self.terminal_eta_distance = 600.0
        self.terminal_direct_guidance_distance = 500.0
        self.terminal_direct_guidance_safe_clearance = max(
            20.0,
            0.5 * float(self.final_zone_clearance),
        )
        self._last_target_distance_for_eta = None
        self._last_sync_remaining_for_eta = None
        self._last_eta_progress_time = None
        self.target_closure_speed_ema = None
        self.sync_progress_speed_ema = None
        self.eta_progress_alpha = 0.35
        self.min_target_closure_speed_for_eta = 3.0
        self.min_sync_progress_speed_for_eta = 3.0
        self._simple_strike_common_hit_time_frozen = False
        self._simple_strike_common_hit_time_freeze_reason = None
        self._simple_strike_common_hit_time_frozen_at = None
        self._common_hit_time_update_ignored_log_time = 0.0
        self._common_hit_time_freeze_blocked_log_time = 0.0
        self.common_hit_time_freeze_distance_to_target = 180.0
        self.prev_dist_to_target = None
        self.strict_plan_fail_relax_after_sec = 3.0
        self.strict_plan_fail_relax_after_count = 60
        self.relaxed_clearance_ratio = 0.25
        self.min_relaxed_clearance = 10.0
        self.max_relaxed_clearance = 25.0
        self._simple_strike_strict_plan_fail_count = 0
        self._simple_strike_strict_plan_fail_first_time = 0.0
        self._simple_strike_relaxed_plan_enabled = False
        self._simple_strike_relaxed_clearance_enabled_logged = False
        self._simple_strike_relaxed_plan_failed_logged = False
        self._simple_strike_plan_fail_summary_log_time = 0.0
        self._simple_strike_last_plan_block_reason = None

        self._planner_zones(log_if_changed=True)
        rospy.logwarn("[SEAD] simple strike mode enabled")
        rospy.logwarn(f"[SEAD] simple strike leader selected: UAV{self.leader_id}")
        rospy.logwarn(f"[SEAD] simple strike canonical targets: {self.targets}")
        rospy.logwarn(
            f"[SEAD] UAV{self.uav_id} zone_clearance="
            f"{self.zone_clearance:.1f}, full_path_control_backend="
            f"{self.full_path_control_backend}"
        )
        rospy.logwarn(
            f"[SEAD] UAV{self.uav_id} simple_strike_execution_mode="
            f"{self.simple_strike_execution_mode}, "
            f"simple_strike_control_mode={self.simple_strike_control_mode}, "
            f"full_path_expected_groundspeed={self.full_path_expected_groundspeed:.1f}, "
            f"full_path_speed_min={self.full_path_speed_min:.1f}, "
            f"full_path_speed_max={self.full_path_speed_max:.1f}, "
            f"effective_sync_speed_max={self.effective_sync_speed_max:.1f}, "
            f"full_path_speed_hard_max={self.full_path_speed_hard_max:.1f}, "
            f"full_path_sync_margin={self.full_path_sync_margin:.1f}"
        )
        self._log_jsonl(
            "simple_strike_mode_selected",
            target_count=len(self.targets),
            simple_strike_execution_mode=self.simple_strike_execution_mode,
            simple_strike_control_mode=self.simple_strike_control_mode,
            use_true_path_eta=self.use_true_path_eta,
            full_path_control_backend=self.full_path_control_backend,
            full_path_speed_sync_enabled=self.full_path_speed_sync_enabled,
            full_path_expected_groundspeed=self.full_path_expected_groundspeed,
            full_path_speed_min=self.full_path_speed_min,
            full_path_speed_max=self.full_path_speed_max,
            full_path_speed_hard_max=self.full_path_speed_hard_max,
            effective_sync_speed_max=self.effective_sync_speed_max,
            swiftwing_realistic_speed_max=self.swiftwing_realistic_speed_max,
            swiftwing_command_speed_max=self.swiftwing_command_speed_max,
            terminal_slowdown_distance=self.terminal_slowdown_distance,
            terminal_speed_max=self.terminal_speed_max,
            terminal_speed_min=self.terminal_speed_min,
            terminal_capture_radius=self.terminal_capture_radius,
            terminal_cross_track_radius=self.terminal_cross_track_radius,
            terminal_relative_sync_enabled=self.terminal_relative_sync_enabled,
            terminal_relative_sync_distance=self.terminal_relative_sync_distance,
            terminal_sync_window=self.terminal_sync_window,
            terminal_global_deadline_ignore_time_left=(
                self.terminal_global_deadline_ignore_time_left
            ),
            terminal_relative_normal_speed=self.terminal_relative_normal_speed,
            terminal_relative_slow_speed=self.terminal_relative_slow_speed,
            terminal_relative_fast_speed=self.terminal_relative_fast_speed,
            terminal_final_hard_speed_cap_distance=(
                self.terminal_final_hard_speed_cap_distance
            ),
            terminal_final_hard_speed_cap=self.terminal_final_hard_speed_cap,
            min_time_left_for_speed_sync=self.min_time_left_for_speed_sync,
            deadline_expired_speed=self.deadline_expired_speed,
            arrival_error_deadband=self.arrival_error_deadband,
            arrival_error_slow_threshold=self.arrival_error_slow_threshold,
            arrival_error_fast_threshold=self.arrival_error_fast_threshold,
            arrival_error_speed_gain=self.arrival_error_speed_gain,
            early_arrival_stretch_enabled=self.early_arrival_stretch_enabled,
            early_arrival_stretch_distance=self.early_arrival_stretch_distance,
            early_arrival_stretch_error=self.early_arrival_stretch_error,
            early_arrival_orbit_radius=self.early_arrival_orbit_radius,
            full_path_sync_margin=self.full_path_sync_margin,
            full_path_tracking_time_scale=self.full_path_tracking_time_scale,
            full_path_extra_start_margin=self.full_path_extra_start_margin,
            path_status_broadcast_interval=self.path_status_broadcast_interval,
            state_broadcast_interval=self.state_broadcast_interval,
            common_hit_time_push_margin=self.common_hit_time_push_margin,
            common_hit_time_min_push=self.common_hit_time_min_push,
            dynamic_sync_min_groundspeed=self.dynamic_sync_min_groundspeed,
            dynamic_sync_eta_speed_floor=self.dynamic_sync_eta_speed_floor,
            dynamic_sync_shortest_feasible_margin=(
                self.dynamic_sync_shortest_feasible_margin
            ),
            dynamic_sync_stop_remaining_to_capture=(
                self.dynamic_sync_stop_remaining_to_capture
            ),
            short_path_speed_min=self.short_path_speed_min,
            terminal_eta_distance=self.terminal_eta_distance,
            terminal_direct_guidance_distance=self.terminal_direct_guidance_distance,
            terminal_direct_guidance_safe_clearance=(
                self.terminal_direct_guidance_safe_clearance
            ),
            eta_progress_alpha=self.eta_progress_alpha,
            min_target_closure_speed_for_eta=(
                self.min_target_closure_speed_for_eta
            ),
            min_sync_progress_speed_for_eta=(
                self.min_sync_progress_speed_for_eta
            ),
            zone_clearance=self.zone_clearance,
            approach_zone_clearance=self.approach_zone_clearance,
            final_zone_clearance=self.final_zone_clearance,
        )
        self._log_jsonl(
            "simple_strike_sync_speed_config",
            control_mode=self.simple_strike_control_mode,
            effective_sync_speed_max=self.effective_sync_speed_max,
            speed_max=self.full_path_speed_max,
            speed_hard_max=self.full_path_speed_hard_max,
            note=(
                "ETA/common_hit_time uses effective_sync_speed_max; "
                "SwiftWing command output is capped by swiftwing_command_speed_max"
            ),
        )
        self._log_jsonl(
            "simple_strike_targets_canonicalized",
            canonical_targets=self.targets,
            target_count=len(self.targets),
        )

    @staticmethod
    def canonical_targets(targets):
        unique = {}
        for idx, target in enumerate(targets or []):
            if target is None or len(target) < 2:
                continue
            key = (round(float(target[0]), 3), round(float(target[1]), 3))
            unique[key] = {
                "target_id": 0,
                "point": [key[0], key[1]],
                "source_index": idx,
            }
        ordered = [unique[key] for key in sorted(unique.keys())]
        for idx, item in enumerate(ordered, start=1):
            item["target_id"] = idx
        return ordered

    @staticmethod
    def build_greedy_assignment(uav_states, targets):
        pairs = []
        for state in uav_states:
            uid = int(state["uav_id"])
            sx, sy = float(state["pos"][0]), float(state["pos"][1])
            for target in targets:
                tx, ty = target["point"]
                distance = float(np.linalg.norm([tx - sx, ty - sy]))
                pairs.append((distance, uid, int(target["target_id"]), target))

        assigned_uavs = set()
        assigned_targets = set()
        assignment = {}
        for _, uid, target_id, target in sorted(pairs, key=lambda x: (x[0], x[1], x[2])):
            if uid in assigned_uavs or target_id in assigned_targets:
                continue
            assigned_uavs.add(uid)
            assigned_targets.add(target_id)
            assignment[uid] = target
            if len(assignment) >= min(len(uav_states), len(targets)):
                break
        return assignment

    @staticmethod
    def _path_length(path):
        if not path or len(path) < 2:
            return 0.0
        total = 0.0
        for p0, p1 in zip(path[:-1], path[1:]):
            total += float(np.linalg.norm([p1[0] - p0[0], p1[1] - p0[1]]))
        return total

    @staticmethod
    def estimate_eta(path_len, cruise_speed):
        return float(path_len) / max(float(cruise_speed), 1.0)

    @staticmethod
    def _json_number(value):
        try:
            value = float(value)
        except Exception:
            return None
        return value if math.isfinite(value) else None

    def _log_jsonl(self, event_type, **kwargs):
        if self.log_jsonl is None:
            return
        try:
            self.log_jsonl(event_type, **kwargs)
        except Exception as ex:
            rospy.logwarn_throttle(2.0, f"[SEAD] simple strike jsonl failed: {ex}")

    def _send_gcs_once(self, xbee, comm_info, gcs, key, message):
        if key in self.gcs_sent:
            return
        self.gcs_sent.add(key)
        try:
            xbee.send_data_async(gcs, comm_info.pack_info_packet(message))
        except Exception as ex:
            rospy.logwarn(f"[SEAD] simple strike GCS feedback failed: {ex}")

    def _planner_zones(self, log_if_changed=False, altitude=None):
        if self.airspace is None:
            self.zones_cache = []
        else:
            try:
                self.zones_cache = list(self.airspace.export_zones_for_planner() or [])
            except Exception as ex:
                self.zones_cache = []
                rospy.logwarn_throttle(
                    2.0,
                    f"[SEAD] UAV{self.uav_id} planner zones export failed: {ex}",
                )

        if altitude is not None:
            altitude = float(altitude)
            self.zones_cache = [
                zone
                for zone in self.zones_cache
                if float(zone.get("minAlt", -float("inf")))
                <= altitude
                <= float(zone.get("maxAlt", float("inf")))
            ]

        zones_count = len(self.zones_cache)
        if log_if_changed and zones_count != self._last_planner_zones_count:
            self._last_planner_zones_count = zones_count
            rospy.logwarn(f"[SEAD] UAV{self.uav_id} planner zones count = {zones_count}")
            self._log_jsonl(
                "simple_strike_planner_zones_loaded",
                zones_count=zones_count,
                zone_clearance=self.zone_clearance,
            )
        return self.zones_cache

    def publish_planned_path(self, path, height):
        if self.path_pub is None:
            return
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.reference_frame
        for point in path or []:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(height)
            yaw = float(point[2]) if len(point) >= 3 else 0.0
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        self.path_pub.publish(message)

    @staticmethod
    def _point_to_segment_distance(px, py, ax, ay, bx, by):
        vx = float(bx) - float(ax)
        vy = float(by) - float(ay)
        wx = float(px) - float(ax)
        wy = float(py) - float(ay)
        denom = vx * vx + vy * vy
        if denom <= 1e-12:
            return float(math.hypot(float(px) - float(ax), float(py) - float(ay)))
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
        proj_x = float(ax) + t * vx
        proj_y = float(ay) + t * vy
        return float(math.hypot(float(px) - proj_x, float(py) - proj_y))

    def _point_to_polygon_edge_distance(self, point_xy, poly):
        if not poly or len(poly) < 2:
            return float("inf")
        try:
            px = float(point_xy[0])
            py = float(point_xy[1])
        except Exception:
            return float("inf")

        min_dist = float("inf")
        for i in range(len(poly)):
            a = poly[i]
            b = poly[(i + 1) % len(poly)]
            if len(a) < 2 or len(b) < 2:
                continue
            dist = self._point_to_segment_distance(
                px,
                py,
                float(a[0]),
                float(a[1]),
                float(b[0]),
                float(b[1]),
            )
            min_dist = min(min_dist, dist)
        return min_dist

    def _min_zone_clearance(self, point_xy, zones=None):
        zones = self._planner_zones() if zones is None else zones
        if not zones:
            return float("inf")
        try:
            x = float(point_xy[0])
            y = float(point_xy[1])
        except Exception:
            return float("inf")

        min_clearance = float("inf")
        for zone in zones:
            poly = zone.get("poly", [])
            if len(poly) < 3:
                continue
            try:
                if _point_in_poly(x, y, poly):
                    return 0.0
                min_clearance = min(
                    min_clearance,
                    self._point_to_polygon_edge_distance([x, y], poly),
                )
            except Exception:
                continue
        return min_clearance

    def _point_in_no_fly(self, point_xy, clearance_threshold=None):
        effective_clearance = float(
            clearance_threshold
            if clearance_threshold is not None
            else self.zone_clearance
        )
        return self._min_zone_clearance(point_xy) < effective_clearance

    def _path_min_zone_clearance(self, path, zones=None):
        zones = self._planner_zones() if zones is None else zones
        if not zones or not path:
            return float("inf")
        min_clearance = float("inf")
        for pt in path:
            if pt is None or len(pt) < 2:
                continue
            min_clearance = min(min_clearance, self._min_zone_clearance(pt, zones=zones))
        return min_clearance

    def _path_points_in_no_fly(self, path, zones=None, clearance_threshold=None):
        effective_clearance = float(
            clearance_threshold
            if clearance_threshold is not None
            else self.zone_clearance
        )
        return self._path_min_zone_clearance(path, zones=zones) < effective_clearance

    def _buffered_planner_zones(self, zones):
        buffered = []
        for zone in zones or []:
            poly = zone.get("poly", [])
            if len(poly) < 3:
                continue
            out = dict(zone)
            out["poly"] = [(float(p[0]), float(p[1])) for p in poly]
            buffered.append(out)
        return buffered

    def _sample_dubins_path(self, sp, gp, rmin, step):
        try:
            return dubins.shortest_path(sp, gp, rmin).sample_many(step)[0]
        except Exception:
            return []

    def _segment_or_path_blocked_detail(
        self,
        sp,
        gp,
        zones=None,
        rmin=None,
        step=5.0,
        clearance_threshold=None,
    ):
        effective_clearance = float(
            clearance_threshold
            if clearance_threshold is not None
            else self.zone_clearance
        )
        zones = self._planner_zones() if zones is None else zones
        if not zones:
            return {
                "blocked": False,
                "dubins_blocked_raw": False,
                "min_zone_clearance": float("inf"),
                "effective_zone_clearance": effective_clearance,
                "path_block_reason": "raw_dubins_safe",
            }

        sp3 = self._as_pose3(sp)
        gp3 = self._as_pose3(gp)
        rmin = max(float(rmin if rmin is not None else self.rmin), 1.0)
        step = max(float(step), 0.5)
        try:
            dubins_blocked_raw = bool(_is_dubins_blocked(sp3, gp3, rmin, zones, step=step))
        except Exception:
            dubins_blocked_raw = True

        sampled = self._sample_dubins_path(sp3, gp3, rmin, step)
        if not sampled:
            sampled = [sp3, gp3]
        min_zone_clearance = self._path_min_zone_clearance(sampled, zones=zones)
        clearance_too_small = min_zone_clearance < effective_clearance + step
        blocked = bool(dubins_blocked_raw or clearance_too_small)

        if dubins_blocked_raw:
            path_block_reason = "inside_polygon"
        elif clearance_too_small:
            path_block_reason = "clearance_too_small"
        else:
            path_block_reason = "raw_dubins_safe"

        return {
            "blocked": blocked,
            "dubins_blocked_raw": dubins_blocked_raw,
            "min_zone_clearance": min_zone_clearance,
            "effective_zone_clearance": effective_clearance,
            "path_block_reason": path_block_reason,
        }

    def _segment_or_path_blocked(self, sp, gp, zones=None, rmin=None, step=5.0):
        detail = self._segment_or_path_blocked_detail(
            sp,
            gp,
            zones=zones,
            rmin=rmin,
            step=step,
        )
        return (
            bool(detail["blocked"]),
            bool(detail["dubins_blocked_raw"]),
            float(detail["min_zone_clearance"]),
        )

    def _log_plan_block_detail(self, detail):
        if not detail:
            return
        min_zone_clearance = detail.get("min_zone_clearance", float("inf"))
        effective_clearance = float(
            detail.get("effective_zone_clearance", self.zone_clearance)
        )
        clearance_text = (
            f"{float(min_zone_clearance):.1f}"
            if math.isfinite(float(min_zone_clearance))
            else "inf"
        )
        rospy.logwarn_throttle(
            2.0,
            f"[SEAD] UAV{self.uav_id} dubins raw blocked = "
            f"{bool(detail.get('dubins_blocked_raw', False))}"
        )
        rospy.logwarn_throttle(
            2.0,
            f"[SEAD] UAV{self.uav_id} min zone clearance = {clearance_text}, "
            f"effective_zone_clearance = {effective_clearance:.1f}"
        )
        if (
            not bool(detail.get("dubins_blocked_raw", False))
            and float(min_zone_clearance) < effective_clearance
        ):
            rospy.logwarn_throttle(
                2.0,
                f"[SEAD] UAV{self.uav_id} buffered no-fly triggered, "
                "clearance < effective_zone_clearance"
            )

    def _path_detail_for_json(self, detail):
        detail = detail or {}
        effective_zone_clearance = detail.get(
            "effective_zone_clearance",
            self.zone_clearance,
        )
        return {
            "dubins_blocked_raw": bool(detail.get("dubins_blocked_raw", False)),
            "min_zone_clearance": self._json_number(
                detail.get("min_zone_clearance", float("inf"))
            ),
            "avoidance_min_zone_clearance": self._json_number(
                detail.get("avoidance_min_zone_clearance", float("inf"))
            ),
            "zone_clearance": float(self.zone_clearance),
            "effective_zone_clearance": float(effective_zone_clearance),
            "path_block_reason": detail.get("path_block_reason", "raw_dubins_safe"),
            "unsafe_fallback_allowed": bool(
                detail.get("unsafe_fallback_allowed", False)
            ),
        }

    def _log_avoidance_failed(self, detail, log_debug=False):
        if not log_debug:
            return
        detail = detail or {}
        min_zone_clearance = detail.get("min_zone_clearance", float("inf"))
        clearance_text = (
            f"{float(min_zone_clearance):.1f}"
            if math.isfinite(float(min_zone_clearance))
            else "inf"
        )
        rospy.logwarn_throttle(
            2.0,
            f"[SEAD] UAV{self.uav_id} avoidance failed, "
            f"path_block_reason={detail.get('path_block_reason')}, "
            f"dubins_blocked_raw={bool(detail.get('dubins_blocked_raw', False))}, "
            f"min_zone_clearance={clearance_text}"
        )

    def _path_blocked_reason_after_avoidance(self, detail):
        if not detail:
            return "raw_dubins_safe"
        if detail.get("path_block_reason") == "inside_polygon":
            return "avoidance_used_due_to_raw_collision"
        if detail.get("path_block_reason") == "clearance_too_small":
            return "avoidance_used_due_to_clearance"
        return detail.get("path_block_reason", "raw_dubins_safe")

    def _path_is_safe_for_clearance(self, path, zones=None, clearance_threshold=None):
        return not self._path_points_in_no_fly(
            path,
            zones=zones,
            clearance_threshold=clearance_threshold,
        )

    def _path_min_clearance_json(self, path, zones=None):
        return self._json_number(self._path_min_zone_clearance(path, zones=zones))

    def _legacy_bool_blocked(self, sp, gp, zones=None, rmin=None, step=5.0):
        blocked, _, _ = self._segment_or_path_blocked(
            sp,
            gp,
            zones=zones,
            rmin=rmin,
            step=step,
        )
        return blocked

    def _path_points_inside_raw_no_fly(self, path, zones=None):
        zones = self._planner_zones() if zones is None else zones
        if not zones or not path:
            return False
        for pt in path:
            if pt is None or len(pt) < 2:
                continue
            x = float(pt[0])
            y = float(pt[1])
            for zone in zones:
                poly = zone.get("poly", [])
                if len(poly) < 3:
                    continue
                try:
                    if _point_in_poly(x, y, poly):
                        return True
                except Exception:
                    continue
        return False

    @staticmethod
    def _wrap_pi(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _as_pose3(point, fallback_yaw=0.0):
        if point is None or len(point) < 2:
            return [0.0, 0.0, float(fallback_yaw)]
        yaw = float(point[2]) if len(point) >= 3 else float(fallback_yaw)
        return [float(point[0]), float(point[1]), yaw]

    def _plan_segment_path(
        self,
        sp,
        gp,
        use_avoidance=True,
        rmin=None,
        speed=None,
        log_debug=False,
        allow_unsafe_fallback=False,
        clearance_threshold=None,
        flight_altitude=None,
    ):
        sp3 = self._as_pose3(sp)
        effective_clearance = float(
            clearance_threshold
            if clearance_threshold is not None
            else self.zone_clearance
        )
        if gp is None or len(gp) < 2:
            return [], False, {
                "dubins_blocked_raw": False,
                "min_zone_clearance": float("inf"),
                "effective_zone_clearance": effective_clearance,
                "path_block_reason": "invalid_goal",
                "unsafe_fallback_allowed": False,
            }
        fallback_yaw = math.atan2(float(gp[1]) - sp3[1], float(gp[0]) - sp3[0])
        gp3 = self._as_pose3(gp, fallback_yaw=fallback_yaw)
        rmin = max(float(rmin if rmin is not None else self.rmin), 1.0)
        speed = max(float(speed if speed is not None else self.cruise_speed), 1.0)
        step = min(max(speed / 5.0, 0.5), 5.0)

        zones = self._planner_zones(
            log_if_changed=True, altitude=flight_altitude
        )
        detail = self._segment_or_path_blocked_detail(
            sp3,
            gp3,
            zones=zones,
            rmin=rmin,
            step=step,
            clearance_threshold=effective_clearance,
        )
        detail["effective_zone_clearance"] = effective_clearance
        detail["unsafe_fallback_allowed"] = bool(allow_unsafe_fallback)
        if log_debug:
            self._log_plan_block_detail(detail)

        if use_avoidance and zones and detail.get("blocked", False):
            try:
                buffered_zones = self._buffered_planner_zones(zones)
                path = plan_path_with_avoidance(
                    sp3,
                    gp3,
                    rmin,
                    buffered_zones or zones,
                    speed,
                    sampling_step=step,
                    clearance=effective_clearance,
                    flight_altitude=flight_altitude,
                )
                if path and len(path) >= 2:
                    avoidance_min_clearance = self._path_min_zone_clearance(
                        path,
                        zones=zones,
                    )
                    detail["avoidance_min_zone_clearance"] = avoidance_min_clearance
                    if avoidance_min_clearance >= effective_clearance:
                        detail["path_block_reason"] = self._path_blocked_reason_after_avoidance(
                            detail
                        )
                        return path, True, detail
                    rospy.logwarn_throttle(
                        2.0,
                        f"[SEAD] UAV{self.uav_id} avoidance planner returned "
                        "samples too close to no-fly"
                    )
            except Exception as ex:
                rospy.logwarn_throttle(
                    2.0,
                    f"[SEAD] UAV{self.uav_id} avoidance planner failed, "
                    f"err={ex}"
                )

        if detail.get("blocked", False):
            if not allow_unsafe_fallback:
                detail["path_block_reason"] = "unsafe_fallback_forbidden"
                detail["unsafe_fallback_allowed"] = False
                self._log_avoidance_failed(detail, log_debug=log_debug)
                return [], False, detail

            detail["path_block_reason"] = "unsafe_fallback_allowed"
            detail["unsafe_fallback_allowed"] = True
            self._log_avoidance_failed(detail, log_debug=log_debug)

        try:
            path = dubins.shortest_path(sp3, gp3, rmin).sample_many(step)[0]
        except Exception as ex:
            self._log_jsonl("simple_strike_error", reason=str(ex))
            rospy.logwarn(f"[SEAD] simple strike path generation failed: {ex}")
            path = []
        if not path:
            path = [sp3, gp3]
        return path, False, detail

    def handle_airspace_update(self, uav_ros, height):
        """Replan an invalidated remaining path, never retaining an unsafe path."""
        path = list(self.path_following.path or [])
        if not self.full_path_plan_built or len(path) < 2:
            return "unchanged"
        index = max(0, min(int(self.full_path_path_index), len(path) - 1))
        remaining = path[index:]
        zones = self._planner_zones(log_if_changed=True, altitude=height)
        if self._path_is_safe_for_clearance(
            remaining,
            zones=zones,
            clearance_threshold=self.final_zone_clearance,
        ):
            return "unchanged"

        old_path = self.path_following.path
        self.path_following.path = []
        self.full_path_plan_built = False
        self.full_path_path_index = 0
        if not self._build_full_path_for_uav(uav_ros, height):
            self.path_following.path = []
            self.full_path_plan_built = False
            self.publish_planned_path([], height)
            self._log_jsonl(
                "dynamic_nofly_replan_failed",
                old_path_points=len(old_path or []),
                zones_count=len(zones),
                action="loiter_required",
            )
            return "failed"
        self._log_jsonl(
            "dynamic_nofly_replanned",
            old_path_points=len(old_path or []),
            new_path_points=len(self.path_following.path or []),
            zones_count=len(zones),
        )
        return "replanned"

    def _update_path_metrics(self, current_xy=None, speed=None, force_log=False):
        full_path = self.path_following.path or []
        self.path_length = self._path_length(full_path)

        idx = max(0, int(getattr(self.path_following, "fw_path_index", 0)))
        idx = min(idx, len(full_path))
        remain = 0.0
        if idx < len(full_path):
            if current_xy is not None:
                try:
                    remain += float(
                        np.linalg.norm(
                            [
                                float(full_path[idx][0]) - float(current_xy[0]),
                                float(full_path[idx][1]) - float(current_xy[1]),
                            ]
                        )
                    )
                except Exception:
                    pass
            for i in range(idx, len(full_path) - 1):
                remain += float(
                    np.linalg.norm(
                        [
                            float(full_path[i + 1][0]) - float(full_path[i][0]),
                            float(full_path[i + 1][1]) - float(full_path[i][1]),
                        ]
                    )
                )

        self.remaining_path_length = max(0.0, float(remain))
        eta_speed = max(float(speed if speed is not None else self.cruise_speed), 1.0)
        if self.use_true_path_eta:
            self.eta = self.remaining_path_length / eta_speed
        else:
            self.eta = self.estimate_eta(self.path_length, eta_speed)

        now = time.time()
        if force_log or now - self._last_path_metrics_log_time >= 1.0:
            self._last_path_metrics_log_time = now
            rospy.logwarn(
                f"[SEAD] UAV{self.uav_id} remaining_path_length = "
                f"{self.remaining_path_length:.1f}"
            )
            rospy.logwarn(f"[SEAD] UAV{self.uav_id} eta(true_path) = {self.eta:.1f}")
            self._log_jsonl(
                "simple_strike_path_metrics_updated",
                phase=self.phase,
                path_length=self.path_length,
                remaining_path_length=self.remaining_path_length,
                eta=self.eta,
                path_index=idx,
                path_points=len(full_path),
                zones_count=len(self.zones_cache),
                zone_clearance=self.zone_clearance,
                final_target_point=self.final_target_point,
                use_true_path_eta=self.use_true_path_eta,
                full_path_control_backend=self.full_path_control_backend,
            )
        return self.remaining_path_length

    def set_remote_map(self, remote_map):
        self.remote_map = dict(remote_map or {})

    def _broadcast_state(self, xbee, comm_info, uav_ros, new_timer):
        if not new_timer.check_period(
            self.state_broadcast_interval,
            self.previous_u2u_time,
        ):
            return
        self.previous_u2u_time = time.time()
        try:
            packet, _ = comm_info.pack_SEAD_packet(
                getattr(uav_ros, "type", self.uav_type),
                getattr(uav_ros, "v", self.cruise_speed),
                getattr(uav_ros, "Rmin", self.rmin),
                [uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw],
                self.base,
                False,
                0.0,
                [],
                [],
                [],
            )
            xbee.send_data_broadcast(packet)
        except Exception as ex:
            rospy.logwarn(f"[SEAD] simple strike state broadcast failed: {ex}")

    def _online_states(self, comm_info, uav_ros):
        states = {}
        for uid, info in getattr(comm_info, "uavs_info", {}).items():
            try:
                uid_int = int(uid)
            except (TypeError, ValueError):
                continue
            pos = info.get("pos", [])
            if len(pos) < 3:
                continue
            states[uid_int] = {
                "uav_id": uid_int,
                "pos": list(pos),
                "v": info.get("v", self.cruise_speed),
                "Rmin": info.get("Rmin", self.rmin),
            }
        if self.uav_id not in states:
            states[self.uav_id] = {
                "uav_id": self.uav_id,
                "pos": [uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw],
                "v": getattr(uav_ros, "v", self.cruise_speed),
                "Rmin": getattr(uav_ros, "Rmin", self.rmin),
            }
        return [states[uid] for uid in sorted(states.keys())]

    def _build_assignment_if_ready(self, xbee, comm_info, gcs, uav_states):
        if self.assignment_map:
            if self.uav_id == self.leader_id:
                self._broadcast_assignment(xbee, comm_info)
            return True
        now = time.time()
        online_ids = sorted(int(state["uav_id"]) for state in uav_states)
        if len(self.targets) != 3:
            if now - self.status_log_time >= 1.0:
                self.status_log_time = now
                rospy.logwarn(
                    f"[SEAD] simple strike waiting roster: online={online_ids}, targets={len(self.targets)}"
                )
            return False

        if online_ids != [1, 2, 3]:
            self.assignment_wait_start_time = 0.0
            self.assignment_roster_latched = []
            if now - self.status_log_time >= 1.0:
                self.status_log_time = now
                rospy.logwarn(f"[SEAD] simple strike waiting roster: online={online_ids}")
            return False

        if self.assignment_roster_latched != online_ids:
            self.assignment_roster_latched = list(online_ids)
            self.assignment_wait_start_time = now
            rospy.logwarn("[SEAD] simple strike roster complete, start stabilization wait")
            return False

        if (now - self.assignment_wait_start_time) < self.assignment_wait_timeout:
            if now - self.status_log_time >= 1.0:
                self.status_log_time = now
                rospy.logwarn(f"[SEAD] simple strike waiting stable roster, online={online_ids}")
            return False

        if len(uav_states) < len(self.targets):
            if now - self.status_log_time >= 1.0:
                self.status_log_time = now
                rospy.logwarn(
                    f"[SEAD] simple strike waiting roster: online={online_ids}, targets={len(self.targets)}"
                )
            return False

        if self.uav_id != self.leader_id:
            if now - self.status_log_time >= 1.0:
                self.status_log_time = now
                rospy.logwarn(f"[SEAD] UAV{self.uav_id} waiting leader assignment broadcast")
            return False

        self.assignment_map = self.build_greedy_assignment(uav_states, self.targets)
        if sorted(self.assignment_map.keys()) != [1, 2, 3]:
            rospy.logwarn(f"[SEAD] simple strike waiting roster: online={online_ids}")
            self.assignment_map = {}
            return False
        self.assignment_locked_from_leader = True
        self.assignment_missing_uavs = set(self._expected_assignment_acks())
        rospy.logwarn(f"[SEAD] simple strike roster: {[s['uav_id'] for s in uav_states]}")
        rospy.logwarn(f"[SEAD] leader assignment built: {self.assignment_map}")
        rospy.logwarn("[SEAD] simple strike assignment locked from leader")
        self._broadcast_assignment(xbee, comm_info)
        self._send_gcs_once(xbee, comm_info, gcs, "assignment_ready", "simple strike assignment ready")
        self._log_jsonl(
            "simple_strike_assignment_built",
            assignment_map=self.assignment_map,
            canonical_targets=self.targets,
        )
        return True

    def _broadcast_assignment(self, xbee, comm_info):
        if self.uav_id != self.leader_id or not self.assignment_map:
            return
        if self.assignment_ack_complete:
            return
        now = time.time()
        expected_acks = self._expected_assignment_acks()
        if not self.assignment_missing_uavs and expected_acks - self.assignment_ack_uavs:
            self.assignment_missing_uavs = set(expected_acks - self.assignment_ack_uavs)
        missing = sorted(self.assignment_missing_uavs)
        if (
            not missing
            and self.assignment_broadcast_count >= self.assignment_min_broadcast_count
        ):
            self.assignment_ack_complete = True
            self.assignment_broadcast_sent = True
            rospy.logwarn("[SEAD] leader assignment ack complete")
            self._log_jsonl(
                "simple_strike_assignment_ack_complete",
                leader_id=self.leader_id,
                target_count=len(self.targets),
                acked_uavs=sorted(self.assignment_ack_uavs),
                missing_uavs=[],
            )
            return

        if not self.assignment_broadcast_started_at:
            self.assignment_broadcast_started_at = now
        if now - self.assignment_broadcast_started_at > self.assignment_broadcast_timeout:
            self.assignment_ack_complete = True
            self.assignment_broadcast_sent = True
            rospy.logwarn(f"[SEAD] leader assignment ack timeout, missing acks: {missing}")
            return

        if now - self.last_assignment_broadcast_time < self.assignment_broadcast_interval:
            if missing and now - self.assignment_ack_log_time >= 1.0:
                self.assignment_ack_log_time = now
                rospy.logwarn(f"[SEAD] leader waiting missing acks: {missing}")
            return
        try:
            packet = comm_info.pack_simple_strike_assignment(
                self.leader_id,
                len(self.targets),
                self.assignment_map,
            )
            if self.assignment_broadcast_count == 0 or not missing:
                xbee.send_data_broadcast(packet)
                self.assignment_broadcast_count += 1
                rospy.logwarn(f"[SEAD] leader broadcast assignment: {self.assignment_map}")
                self._log_jsonl(
                    "simple_strike_assignment_broadcast",
                    leader_id=self.leader_id,
                    target_count=len(self.targets),
                    assignment_map=self.assignment_map,
                    acked_uavs=sorted(self.assignment_ack_uavs),
                    missing_uavs=missing,
                    target_uav_id=0,
                )
            else:
                for uid in missing:
                    remote = self.remote_map.get(uid)
                    if remote is None:
                        rospy.logwarn(f"[SEAD] leader missing remote for UAV{uid}")
                        continue
                    xbee.send_data_async(remote, packet)
                    self.assignment_broadcast_count += 1
                    rospy.logwarn(f"[SEAD] leader unicast assignment to UAV{uid}")
                    self._log_jsonl(
                        "simple_strike_assignment_broadcast",
                        leader_id=self.leader_id,
                        target_count=len(self.targets),
                        assignment_map=self.assignment_map,
                        acked_uavs=sorted(self.assignment_ack_uavs),
                        missing_uavs=missing,
                        target_uav_id=uid,
                    )
            self.last_assignment_broadcast_time = now
        except Exception as ex:
            rospy.logwarn(f"[SEAD] leader broadcast assignment failed: {ex}")

    def _expected_assignment_acks(self):
        return {uid for uid in [1, 2, 3] if uid != self.leader_id}

    def _send_assignment_ack(self, xbee, comm_info):
        if self.uav_id == self.leader_id:
            return
        if xbee is None or comm_info is None:
            return
        now = time.time()
        if now - self.last_assignment_ack_time < self.assignment_ack_interval:
            return
        try:
            ack_packet = comm_info.pack_simple_strike_assignment_ack(
                self.leader_id,
                len(self.targets),
            )
            remote_leader = self.remote_map.get(self.leader_id)
            if remote_leader is None:
                rospy.logwarn(f"[SEAD] UAV{self.uav_id} missing leader remote for assignment ack")
                return
            xbee.send_data_async(remote_leader, ack_packet)
            self.last_assignment_ack_time = now
            rospy.logwarn(f"[SEAD] UAV{self.uav_id} sent assignment ack to leader")
            self._log_jsonl(
                "simple_strike_assignment_ack_sent",
                leader_id=self.leader_id,
                target_count=len(self.targets),
                sender_uav_id=self.uav_id,
                ack_send_time=now,
            )
        except Exception as ex:
            rospy.logwarn(f"[SEAD] UAV{self.uav_id} assignment ack failed: {ex}")

    def apply_remote_assignment(self, payload, xbee=None, comm_info=None):
        if self.assignment_map:
            self._send_assignment_ack(xbee, comm_info)
            return True
        if not isinstance(payload, dict):
            rospy.logwarn(f"[SEAD] UAV{self.uav_id} invalid leader assignment payload")
            return False
        leader_id = int(payload.get("leader_id", -1))
        if leader_id != self.leader_id:
            rospy.logwarn(
                f"[SEAD] UAV{self.uav_id} ignored assignment from leader_id={leader_id}"
            )
            return False
        target_count = int(payload.get("target_count", 0))
        if target_count != len(self.targets):
            rospy.logwarn(
                f"[SEAD] UAV{self.uav_id} ignored assignment target_count={target_count}, local={len(self.targets)}"
            )
            return False
        assignment_map = payload.get("assignment_map", {})
        if sorted(int(uid) for uid in assignment_map.keys()) != [1, 2, 3]:
            rospy.logwarn(
                f"[SEAD] UAV{self.uav_id} ignored incomplete leader assignment: {assignment_map}"
            )
            return False
        self.assignment_map = {
            int(uid): {
                "target_id": int(target.get("target_id", 0)),
                "point": [float(target["point"][0]), float(target["point"][1])],
            }
            for uid, target in assignment_map.items()
        }
        self.assignment_received = True
        self.assignment_locked_from_leader = True
        rospy.logwarn(f"[SEAD] UAV{self.uav_id} applied leader assignment: {self.assignment_map}")
        rospy.logwarn("[SEAD] simple strike assignment locked from leader")
        self._send_assignment_ack(xbee, comm_info)
        self._log_jsonl(
            "simple_strike_assignment_built",
            assignment_map=self.assignment_map,
            canonical_targets=self.targets,
            source="leader_broadcast",
        )
        return True

    def apply_assignment_ack(self, payload):
        if self.uav_id != self.leader_id:
            return False
        if not isinstance(payload, dict):
            return False
        if int(payload.get("leader_id", -1)) != self.leader_id:
            return False
        if int(payload.get("target_count", 0)) != len(self.targets):
            return False
        sender_uav_id = int(payload.get("sender_uav_id", -1))
        expected_acks = self._expected_assignment_acks()
        if sender_uav_id not in expected_acks:
            return False
        if sender_uav_id not in self.assignment_ack_uavs:
            self.assignment_ack_uavs.add(sender_uav_id)
            self.assignment_missing_uavs.discard(sender_uav_id)
            rospy.logwarn(f"[SEAD] leader received assignment ack from UAV{sender_uav_id}")
            self._log_jsonl(
                "simple_strike_assignment_ack_received",
                leader_id=self.leader_id,
                target_count=len(self.targets),
                sender_uav_id=sender_uav_id,
                acked_uavs=sorted(self.assignment_ack_uavs),
                missing_uavs=sorted(expected_acks - self.assignment_ack_uavs),
            )
        missing = set(self.assignment_missing_uavs)
        if (
            not missing
            and self.assignment_broadcast_count >= self.assignment_min_broadcast_count
        ):
            self.assignment_ack_complete = True
            self.assignment_broadcast_sent = True
            rospy.logwarn("[SEAD] leader assignment ack complete")
            self._log_jsonl(
                "simple_strike_assignment_ack_complete",
                leader_id=self.leader_id,
                target_count=len(self.targets),
                acked_uavs=sorted(self.assignment_ack_uavs),
                missing_uavs=[],
            )
        return True

    def _mission_time(self, new_timer):
        try:
            value = new_timer.t()
            if value is not None:
                return float(value)
        except Exception:
            pass
        return time.time()

    def _relaxed_control_clearance(self):
        return min(
            max(
                float(self.final_zone_clearance) * float(self.relaxed_clearance_ratio),
                float(self.min_relaxed_clearance),
            ),
            float(self.max_relaxed_clearance),
        )

    def _reset_strict_plan_fail_state(self):
        self._simple_strike_strict_plan_fail_count = 0
        self._simple_strike_strict_plan_fail_first_time = 0.0
        self._simple_strike_relaxed_plan_enabled = False
        self._simple_strike_relaxed_clearance_enabled_logged = False
        self._simple_strike_relaxed_plan_failed_logged = False
        self._simple_strike_last_plan_block_reason = None

    def _record_strict_plan_failure(self, plan_detail):
        now = time.time()
        if self._simple_strike_strict_plan_fail_first_time <= 0.0:
            self._simple_strike_strict_plan_fail_first_time = now
        self._simple_strike_strict_plan_fail_count += 1
        if isinstance(plan_detail, dict):
            self._simple_strike_last_plan_block_reason = plan_detail.get(
                "path_block_reason",
                self._simple_strike_last_plan_block_reason,
            )
        duration = now - float(self._simple_strike_strict_plan_fail_first_time)
        should_enable_relaxed = (
            duration >= float(self.strict_plan_fail_relax_after_sec)
            or self._simple_strike_strict_plan_fail_count
            >= int(self.strict_plan_fail_relax_after_count)
        )
        enable_reason = (
            "strict_plan_failed_timeout"
            if duration >= float(self.strict_plan_fail_relax_after_sec)
            else "strict_plan_failed_count"
        )
        relaxed_clearance = self._relaxed_control_clearance()
        if should_enable_relaxed and not self._simple_strike_relaxed_plan_enabled:
            self._simple_strike_relaxed_plan_enabled = True
            if not self._simple_strike_relaxed_clearance_enabled_logged:
                self._simple_strike_relaxed_clearance_enabled_logged = True
                self._log_jsonl(
                    "simple_strike_relaxed_clearance_enabled",
                    reason=enable_reason,
                    strict_plan_fail_duration=duration,
                    strict_plan_fail_count=self._simple_strike_strict_plan_fail_count,
                    zone_clearance=self.final_zone_clearance,
                    relaxed_clearance=relaxed_clearance,
                    relaxed_clearance_ratio=self.relaxed_clearance_ratio,
                    last_block_reason=self._simple_strike_last_plan_block_reason,
                )
                rospy.logwarn(
                    f"[SEAD] UAV{self.uav_id} relaxed clearance enabled after "
                    f"{duration:.1f}s/{self._simple_strike_strict_plan_fail_count} "
                    f"strict plan failures: clearance={relaxed_clearance:.1f}"
                )
        return duration, relaxed_clearance

    def _log_plan_fail_summary(
        self,
        relaxed_clearance,
        relaxed_plan_enabled=None,
        force=False,
    ):
        now = time.time()
        if not force and now - self._simple_strike_plan_fail_summary_log_time < 1.0:
            return
        self._simple_strike_plan_fail_summary_log_time = now
        first_time = float(self._simple_strike_strict_plan_fail_first_time)
        duration = now - first_time if first_time > 0.0 else 0.0
        self._log_jsonl(
            "simple_strike_plan_fail_summary",
            strict_plan_fail_count=self._simple_strike_strict_plan_fail_count,
            strict_plan_fail_duration=duration,
            relaxed_plan_enabled=(
                self._simple_strike_relaxed_plan_enabled
                if relaxed_plan_enabled is None
                else bool(relaxed_plan_enabled)
            ),
            last_block_reason=self._simple_strike_last_plan_block_reason,
            zone_clearance=self.final_zone_clearance,
            relaxed_clearance=relaxed_clearance,
        )

    def _freeze_simple_strike_common_hit_time(
        self,
        reason,
        distance_to_target=None,
        candidate_common_hit_time=None,
    ):
        if self.full_path_common_hit_time <= 0.0:
            return False
        if self._simple_strike_common_hit_time_frozen:
            return False
        self._simple_strike_common_hit_time_frozen = True
        self._simple_strike_common_hit_time_freeze_reason = str(reason)
        self._simple_strike_common_hit_time_frozen_at = time.time()
        self._log_jsonl(
            "simple_strike_common_hit_time_frozen",
            common_hit_time=self.full_path_common_hit_time,
            freeze_reason=self._simple_strike_common_hit_time_freeze_reason,
            distance_to_target=distance_to_target,
            phase=self.phase,
            uav_id=self.uav_id,
            candidate_common_hit_time=candidate_common_hit_time,
            simple_strike_control_mode=self.simple_strike_control_mode,
        )
        rospy.logwarn(
            f"[SEAD] UAV{self.uav_id} froze common_hit_time="
            f"{self.full_path_common_hit_time:.3f}, "
            f"reason={self._simple_strike_common_hit_time_freeze_reason}"
        )
        return True

    def _log_common_hit_time_update_ignored(self, candidate_common_hit_time, reason):
        now = time.time()
        if now - self._common_hit_time_update_ignored_log_time < 1.0:
            return
        self._common_hit_time_update_ignored_log_time = now
        self._log_jsonl(
            "simple_strike_common_hit_time_update_ignored",
            frozen_common_hit_time=self.full_path_common_hit_time,
            candidate_common_hit_time=candidate_common_hit_time,
            freeze_reason=self._simple_strike_common_hit_time_freeze_reason,
            update_reason=reason,
            simple_strike_control_mode=self.simple_strike_control_mode,
        )

    def _log_common_hit_time_freeze_blocked(
        self,
        freeze_block_reason,
        time_left=None,
        predicted_arrival_error=None,
        dist_to_target=None,
    ):
        now = time.time()
        if now - self._common_hit_time_freeze_blocked_log_time < 1.0:
            return
        self._common_hit_time_freeze_blocked_log_time = now
        self._log_jsonl(
            "simple_strike_common_hit_time_freeze_blocked",
            freeze_block_reason=freeze_block_reason,
            time_left=time_left,
            predicted_arrival_error=predicted_arrival_error,
            dist_to_target=dist_to_target,
            common_hit_time=self.full_path_common_hit_time,
            simple_strike_control_mode=self.simple_strike_control_mode,
        )

    def _set_simple_strike_common_hit_time(self, candidate_time, reason):
        try:
            candidate_time = float(candidate_time)
        except Exception:
            return False
        if candidate_time <= 0.0:
            return False
        if (
            self._simple_strike_common_hit_time_frozen
            and self.full_path_common_hit_time > 0.0
            and abs(candidate_time - float(self.full_path_common_hit_time)) > 1e-3
        ):
            self._log_common_hit_time_update_ignored(candidate_time, reason)
            return False
        self.full_path_common_hit_time = candidate_time
        return True

    def _distance_to_final_target(self, uav_ros=None, guidance=None):
        if guidance is not None and guidance.get("dist_to_target") is not None:
            return float(guidance["dist_to_target"])
        if uav_ros is None or self.final_target_point is None:
            return None
        try:
            dx = float(self.final_target_point[0]) - float(uav_ros.local_pose[0])
            dy = float(self.final_target_point[1]) - float(uav_ros.local_pose[1])
            return float(math.hypot(dx, dy))
        except Exception:
            return None

    def _maybe_freeze_common_hit_time(self, uav_ros=None, guidance=None, now_abs=None):
        if self._simple_strike_common_hit_time_frozen:
            return False
        if self.full_path_common_hit_time <= 0.0:
            return False
        if now_abs is None:
            now_ref = time.time()
        else:
            now_ref = float(now_abs)

        distance_to_target = self._distance_to_final_target(
            uav_ros=uav_ros,
            guidance=guidance,
        )
        if self.target_reached_reported or self.mission_flag:
            return self._freeze_simple_strike_common_hit_time(
                "target_reached",
                distance_to_target=distance_to_target,
            )
        if (
            distance_to_target is not None
            and distance_to_target
            <= float(self.common_hit_time_freeze_distance_to_target)
        ):
            time_left = float(self.full_path_common_hit_time) - now_ref
            predicted_arrival_error = None
            if guidance is not None:
                predicted_arrival_error = guidance.get("predicted_arrival_error")
            if self.simple_strike_control_mode == "swiftwing_vector":
                self._log_common_hit_time_freeze_blocked(
                    "swiftwing_dynamic_push_until_reached",
                    time_left=time_left,
                    predicted_arrival_error=predicted_arrival_error,
                    dist_to_target=distance_to_target,
                )
                return False
            near_target_freeze_allowed = (
                time_left > 8.0
                and predicted_arrival_error is not None
                and abs(float(predicted_arrival_error)) < 5.0
            )
            if not near_target_freeze_allowed:
                if time_left <= 8.0:
                    freeze_block_reason = "time_left_too_small"
                elif predicted_arrival_error is None:
                    freeze_block_reason = "missing_predicted_arrival_error"
                else:
                    freeze_block_reason = "arrival_error_too_large"
                self._log_common_hit_time_freeze_blocked(
                    freeze_block_reason,
                    time_left=time_left,
                    predicted_arrival_error=predicted_arrival_error,
                    dist_to_target=distance_to_target,
                )
                return False
            return self._freeze_simple_strike_common_hit_time(
                "near_target",
                distance_to_target=distance_to_target,
            )

        path = self.path_following.path or []
        if (
            self.full_path_plan_built
            and len(path) >= 2
            and int(self.full_path_path_index) >= max(0, len(path) - 2)
        ):
            if self.simple_strike_control_mode == "swiftwing_vector":
                self._log_common_hit_time_freeze_blocked(
                    "swiftwing_dynamic_push_until_reached",
                    time_left=float(self.full_path_common_hit_time) - now_ref,
                    predicted_arrival_error=(
                        guidance.get("predicted_arrival_error")
                        if guidance is not None
                        else None
                    ),
                    dist_to_target=distance_to_target,
                )
                return False
            return self._freeze_simple_strike_common_hit_time(
                "target_approach",
                distance_to_target=distance_to_target,
            )

        if self.simple_strike_control_mode == "swiftwing_vector":
            return False
        return False

    def _target_for_uav(self, uav_id):
        try:
            return self.assignment_map.get(int(uav_id))
        except Exception:
            return None

    def _target_id_for_uav(self, uav_id):
        target = self._target_for_uav(uav_id)
        if target is None:
            return None
        try:
            return int(target.get("target_id", 0))
        except Exception:
            return None

    def _build_full_path_for_uav(self, uav_ros, height):
        if self.assigned_target is None:
            self.assigned_target = self.assignment_map.get(self.uav_id)
        if self.assigned_target is None:
            rospy.logwarn(f"[SEAD] UAV{self.uav_id} no full-path assignment yet")
            return False

        final_target_point = self.assigned_target["point"]
        target_x = float(final_target_point[0])
        target_y = float(final_target_point[1])
        self.final_target_point = [target_x, target_y]
        start = [
            float(uav_ros.local_pose[0]),
            float(uav_ros.local_pose[1]),
            float(uav_ros.yaw),
        ]
        heading_to_target = math.atan2(target_y - start[1], target_x - start[0])
        goal = [target_x, target_y, heading_to_target]
        speed = max(float(getattr(uav_ros, "v", self.cruise_speed)), 1.0)
        rmin = max(float(getattr(uav_ros, "Rmin", self.rmin)), 1.0)

        path, used_avoidance, plan_detail = self._plan_segment_path(
            start,
            goal,
            use_avoidance=True,
            rmin=rmin,
            speed=speed,
            log_debug=True,
            allow_unsafe_fallback=False,
            clearance_threshold=self.final_zone_clearance,
            flight_altitude=height,
        )
        control_path_source = "strict_control_path"
        strict_plan_failed = False
        relaxed_control_clearance = None
        relaxed_control_attempted = False
        relaxed_control_path_inside_raw_nofly = None
        if not path:
            strict_plan_failed = True
            strict_plan_fail_duration, relaxed_control_clearance = (
                self._record_strict_plan_failure(plan_detail)
            )
            if not self._simple_strike_relaxed_plan_enabled:
                self._log_plan_fail_summary(relaxed_control_clearance)
                if (
                    self._simple_strike_strict_plan_fail_count == 1
                    or self._simple_strike_strict_plan_fail_count % 20 == 0
                ):
                    self._log_jsonl(
                        "simple_strike_full_path_plan_failed",
                        target_id=self._target_id_for_uav(self.uav_id),
                        final_target_point=[target_x, target_y],
                        start_pose=start,
                        goal_pose=goal,
                        zones_count=len(self.zones_cache),
                        final_zone_clearance=self.final_zone_clearance,
                        strict_plan_failed=True,
                        strict_plan_fail_count=(
                            self._simple_strike_strict_plan_fail_count
                        ),
                        strict_plan_fail_duration=strict_plan_fail_duration,
                        relaxed_control_attempted=False,
                        relaxed_control_clearance=relaxed_control_clearance,
                        suggestion="waiting strict-plan fail timeout before relaxed clearance",
                        **self._path_detail_for_json(plan_detail),
                    )
                return False

            relaxed_control_attempted = True
            relaxed_path, relaxed_used_avoidance, relaxed_detail = self._plan_segment_path(
                start,
                goal,
                use_avoidance=True,
                rmin=rmin,
                speed=speed,
                log_debug=True,
                allow_unsafe_fallback=False,
                clearance_threshold=relaxed_control_clearance,
                flight_altitude=height,
            )
            relaxed_path_safe_for_clearance = bool(
                relaxed_path
                and self._path_is_safe_for_clearance(
                    relaxed_path,
                    zones=self._planner_zones(),
                    clearance_threshold=relaxed_control_clearance,
                )
            )
            relaxed_path_min_zone_clearance = self._path_min_clearance_json(
                relaxed_path,
                zones=self._planner_zones(),
            )
            relaxed_control_path_inside_raw_nofly = bool(
                relaxed_path
                and self._path_points_inside_raw_no_fly(
                    relaxed_path,
                    zones=self._planner_zones(),
                )
            )
            if (
                relaxed_path
                and len(relaxed_path) >= 2
                and not relaxed_control_path_inside_raw_nofly
                and relaxed_path_safe_for_clearance
            ):
                path = relaxed_path
                used_avoidance = relaxed_used_avoidance
                plan_detail = dict(relaxed_detail or {})
                plan_detail["path_block_reason"] = "relaxed_control_path_used"
                plan_detail["relaxed_control_clearance"] = relaxed_control_clearance
                plan_detail["strict_plan_failed"] = True
                control_path_source = "relaxed_control_path"
                relaxed_path_length = self._path_length(relaxed_path)
                self._log_jsonl(
                    "simple_strike_relaxed_plan_success",
                    path_length=relaxed_path_length,
                    path_points=len(relaxed_path),
                    zone_clearance=self.final_zone_clearance,
                    relaxed_clearance=relaxed_control_clearance,
                    relaxed_path_safe_for_clearance=relaxed_path_safe_for_clearance,
                    relaxed_control_path_inside_raw_nofly=(
                        relaxed_control_path_inside_raw_nofly
                    ),
                    relaxed_path_min_zone_clearance=relaxed_path_min_zone_clearance,
                    strict_plan_fail_duration=strict_plan_fail_duration,
                    strict_plan_fail_count=self._simple_strike_strict_plan_fail_count,
                )
                rospy.logwarn(
                    f"[SEAD] UAV{self.uav_id} using relaxed control path: "
                    f"clearance={relaxed_control_clearance:.1f}, "
                    "does not enter raw no-fly polygon"
                )
                self._reset_strict_plan_fail_state()
            else:
                relaxed_path_block_reason = (
                    relaxed_detail.get("path_block_reason")
                    if isinstance(relaxed_detail, dict)
                    else None
                )
                should_log_relaxed_failure = (
                    not self._simple_strike_relaxed_plan_failed_logged
                    or self._simple_strike_strict_plan_fail_count % 20 == 0
                )
                if should_log_relaxed_failure:
                    self._log_jsonl(
                        "simple_strike_relaxed_plan_failed",
                        target_id=self._target_id_for_uav(self.uav_id),
                        final_target_point=[target_x, target_y],
                        zone_clearance=self.final_zone_clearance,
                        relaxed_clearance=relaxed_control_clearance,
                        relaxed_path_safe_for_clearance=relaxed_path_safe_for_clearance,
                        relaxed_control_path_inside_raw_nofly=(
                            relaxed_control_path_inside_raw_nofly
                        ),
                        relaxed_path_min_zone_clearance=relaxed_path_min_zone_clearance,
                        relaxed_path_block_reason=relaxed_path_block_reason,
                        strict_plan_fail_duration=strict_plan_fail_duration,
                        strict_plan_fail_count=self._simple_strike_strict_plan_fail_count,
                    )
                self._log_plan_fail_summary(
                    relaxed_control_clearance,
                    relaxed_plan_enabled=True,
                )
                failure_detail = dict(plan_detail or {})
                failure_detail.update(
                    {
                        "strict_plan_failed": True,
                        "relaxed_control_attempted": relaxed_control_attempted,
                        "relaxed_control_clearance": relaxed_control_clearance,
                        "relaxed_path_safe_for_clearance": (
                            relaxed_path_safe_for_clearance
                        ),
                        "relaxed_control_path_inside_raw_nofly": (
                            relaxed_control_path_inside_raw_nofly
                        ),
                        "relaxed_path_min_zone_clearance": (
                            relaxed_path_min_zone_clearance
                        ),
                        "relaxed_path_block_reason": relaxed_path_block_reason,
                    }
                )
                if should_log_relaxed_failure:
                    self._log_jsonl(
                        "simple_strike_full_path_plan_failed",
                        target_id=self._target_id_for_uav(self.uav_id),
                        final_target_point=[target_x, target_y],
                        start_pose=start,
                        goal_pose=goal,
                        zones_count=len(self.zones_cache),
                        final_zone_clearance=self.final_zone_clearance,
                        strict_plan_failed=True,
                        strict_plan_fail_count=(
                            self._simple_strike_strict_plan_fail_count
                        ),
                        strict_plan_fail_duration=strict_plan_fail_duration,
                        relaxed_control_attempted=relaxed_control_attempted,
                        relaxed_control_clearance=relaxed_control_clearance,
                        relaxed_path_safe_for_clearance=relaxed_path_safe_for_clearance,
                        relaxed_control_path_inside_raw_nofly=(
                            relaxed_control_path_inside_raw_nofly
                        ),
                        relaxed_path_min_zone_clearance=relaxed_path_min_zone_clearance,
                        suggestion="relax clearance or improve bypass planner",
                        **self._path_detail_for_json(failure_detail),
                    )
                    self._simple_strike_relaxed_plan_failed_logged = True
                return False
        else:
            self._reset_strict_plan_fail_state()

        self.path_following.path = path
        self.path_following.fw_path_index = 0
        self.path_following.Rmin = rmin
        self.full_path_path_index = 0
        self.full_path_plan_built = True
        self.full_path_speed_cmd_prev = float(self.cruise_speed)
        self.full_path_control_path_source = control_path_source
        self.path_length = self._path_length(path)
        self.remaining_path_length = self.path_length
        self.eta = self.path_length / max(float(self.full_path_expected_groundspeed), 1.0)
        self.path_update_flag = True
        self.path_exhausted_reported = False
        self.publish_planned_path(path, height)

        rospy.logwarn(
            f"[SEAD] UAV{self.uav_id} full-path planned to target "
            f"{self._target_id_for_uav(self.uav_id)}, path_len={self.path_length:.1f}"
        )
        self._log_jsonl(
            "simple_strike_full_path_planned",
            target_id=self._target_id_for_uav(self.uav_id),
            final_target_point=self.final_target_point,
            path_length=self.path_length,
            eta=self.eta,
            path_points=len(path),
            used_avoidance=used_avoidance,
            plan_detail=self._path_detail_for_json(plan_detail),
            expected_groundspeed=self.full_path_expected_groundspeed,
            speed_min=self.full_path_speed_min,
            speed_max=self.full_path_speed_max,
            effective_sync_speed_max=self.effective_sync_speed_max,
            speed_hard_max=self.full_path_speed_hard_max,
            zones_count=len(self.zones_cache),
            final_zone_clearance=self.final_zone_clearance,
            control_path_source=control_path_source,
            strict_plan_failed=strict_plan_failed,
            relaxed_control_clearance=relaxed_control_clearance,
            remaining_path_length=self.remaining_path_length,
            no_release_point=True,
        )
        return True

    def _estimate_team_full_path_etas(self, uav_states):
        team_etas = {}
        if not self.assignment_map:
            return team_etas
        eta_speed_for_common_time = max(float(self.effective_sync_speed_max), 1.0)

        for state in uav_states or []:
            try:
                uid = int(state.get("uav_id", -1))
            except Exception:
                continue
            if uid not in (1, 2, 3):
                continue
            target = self.assignment_map.get(uid)
            if target is None:
                continue
            pos = state.get("pos", [])
            if len(pos) < 3:
                continue
            point = target.get("point", [])
            if len(point) < 2:
                continue

            start = [float(pos[0]), float(pos[1]), float(pos[2])]
            target_x = float(point[0])
            target_y = float(point[1])
            heading_to_target = math.atan2(target_y - start[1], target_x - start[0])
            goal = [target_x, target_y, heading_to_target]
            rmin = max(float(state.get("Rmin", self.rmin)), 1.0)
            speed = eta_speed_for_common_time
            path, used_avoidance, plan_detail = self._plan_segment_path(
                start,
                goal,
                use_avoidance=True,
                rmin=rmin,
                speed=speed,
                log_debug=False,
                allow_unsafe_fallback=False,
                clearance_threshold=self.final_zone_clearance,
            )
            strict_plan_failed = not bool(path)
            relaxed_plan_used = False
            fallback_distance_eta_used = False
            strict_path_block_reason = (
                plan_detail.get("path_block_reason")
                if isinstance(plan_detail, dict)
                else None
            )
            relaxed_path_block_reason = None
            eta_source = "strict_full_path_eta"

            if not path:
                self._log_jsonl(
                    "simple_strike_full_path_team_eta_plan_failed",
                    estimated_uav_id=uid,
                    target_id=int(target.get("target_id", 0)),
                    point=[target_x, target_y],
                    start_pose=start,
                    eta_stage="strict",
                    **self._path_detail_for_json(plan_detail),
                )
                relaxed_clearance = self._relaxed_control_clearance()
                path, used_avoidance, relaxed_detail = self._plan_segment_path(
                    start,
                    goal,
                    use_avoidance=True,
                    rmin=rmin,
                    speed=speed,
                    log_debug=False,
                    allow_unsafe_fallback=False,
                    clearance_threshold=relaxed_clearance,
                )
                relaxed_path_block_reason = (
                    relaxed_detail.get("path_block_reason")
                    if isinstance(relaxed_detail, dict)
                    else None
                )
                if path:
                    plan_detail = relaxed_detail
                    relaxed_plan_used = True
                    eta_source = "relaxed_full_path_eta"
                else:
                    self._log_jsonl(
                        "simple_strike_full_path_team_eta_plan_failed",
                        estimated_uav_id=uid,
                        target_id=int(target.get("target_id", 0)),
                        point=[target_x, target_y],
                        start_pose=start,
                        eta_stage="relaxed",
                        relaxed_clearance=relaxed_clearance,
                        eta_source="fallback_distance_eta",
                        fallback_distance_eta_used=True,
                        **self._path_detail_for_json(relaxed_detail),
                    )
                    straight_len = float(math.hypot(target_x - start[0], target_y - start[1]))
                    turn_penalty = math.pi * max(rmin, float(self.rmin))
                    path_len = straight_len + turn_penalty
                    eta = path_len / speed
                    fallback_distance_eta_used = True
                    eta_source = "fallback_distance_eta"
                    team_etas[uid] = {
                        "path_length": path_len,
                        "eta": eta,
                        "eta_source": eta_source,
                        "target_id": int(target.get("target_id", 0)),
                        "point": [target_x, target_y],
                        "path_points": 2,
                        "used_avoidance": False,
                        "strict_plan_failed": strict_plan_failed,
                        "relaxed_plan_used": relaxed_plan_used,
                        "fallback_distance_eta_used": fallback_distance_eta_used,
                        "strict_path_block_reason": strict_path_block_reason,
                        "relaxed_path_block_reason": relaxed_path_block_reason,
                        "eta_speed_for_common_time": eta_speed_for_common_time,
                    }
                    continue

            path_len = self._path_length(path)
            eta = path_len / speed
            team_etas[uid] = {
                "path_length": path_len,
                "eta": eta,
                "eta_source": eta_source,
                "target_id": int(target.get("target_id", 0)),
                "point": [target_x, target_y],
                "path_points": len(path),
                "used_avoidance": bool(used_avoidance),
                "strict_plan_failed": strict_plan_failed,
                "relaxed_plan_used": relaxed_plan_used,
                "fallback_distance_eta_used": fallback_distance_eta_used,
                "strict_path_block_reason": strict_path_block_reason,
                "relaxed_path_block_reason": relaxed_path_block_reason,
                "eta_speed_for_common_time": eta_speed_for_common_time,
            }
        return team_etas

    def _broadcast_full_path_common_hit_time(self, xbee, comm_info):
        if self.uav_id != self.leader_id:
            return False
        if self.full_path_common_hit_time <= 0.0:
            return True

        now = time.time()
        if now - self.last_full_path_common_hit_time_broadcast_time < 0.5:
            return True

        try:
            packer = getattr(comm_info, "pack_simple_strike_" + "release" + "_time")
            packet = packer(
                self.leader_id,
                len(self.targets),
                self.full_path_common_hit_time,
            )
            xbee.send_data_broadcast(packet)
            self.last_full_path_common_hit_time_broadcast_time = now
            self.full_path_common_hit_time_broadcast_count += 1
            self._log_jsonl(
                "simple_strike_full_path_common_hit_time_broadcast",
                leader_id=self.leader_id,
                target_count=len(self.targets),
                common_hit_time=self.full_path_common_hit_time,
                broadcast_count=self.full_path_common_hit_time_broadcast_count,
                message_semantics="common_hit_time",
            )
            return True
        except Exception as ex:
            rospy.logwarn(f"[SEAD] leader full-path common_hit_time broadcast failed: {ex}")
            return False

    def apply_common_hit_time(self, payload, xbee=None, comm_info=None):
        if not isinstance(payload, dict):
            return False
        if int(payload.get("leader_id", -1)) != self.leader_id:
            return False
        if int(payload.get("target_count", 0)) != len(self.targets):
            return False
        sender_uav_id = int(payload.get("sender_uav_id", self.leader_id))
        if sender_uav_id != self.leader_id:
            return False
        legacy_key = "release" + "_time"
        try:
            common_hit_time = float(payload.get("common_hit_time", payload.get(legacy_key, 0.0)))
        except (TypeError, ValueError):
            return False
        if common_hit_time <= 0.0:
            return False
        if (
            self.full_path_common_hit_time > 0.0
            and abs(self.full_path_common_hit_time - common_hit_time) <= 1e-3
        ):
            return True

        if not self._set_simple_strike_common_hit_time(
            common_hit_time,
            reason="received_common_hit_time",
        ):
            return True
        rospy.logwarn(
            f"[SEAD] UAV{self.uav_id} received full-path common_hit_time: "
            f"{self.full_path_common_hit_time:.3f}"
        )
        self._log_jsonl(
            "simple_strike_full_path_common_hit_time_received",
            leader_id=self.leader_id,
            sender_uav_id=sender_uav_id,
            target_count=len(self.targets),
            common_hit_time=self.full_path_common_hit_time,
            message_semantics="common_hit_time",
            full_path_control_backend=self.full_path_control_backend,
            simple_strike_control_mode=self.simple_strike_control_mode,
            target_id=self._target_id_for_uav(self.uav_id),
        )
        return True

    def _current_actual_groundspeed(self, uav_ros):
        try:
            local_velo = getattr(uav_ros, "local_velo", None)
            if local_velo is not None and len(local_velo) >= 2:
                return float(math.hypot(float(local_velo[0]), float(local_velo[1])))
        except Exception:
            pass
        return None

    def _arrival_radius_for_sync(self, waypoint_radius=None):
        try:
            wr = float(waypoint_radius) if waypoint_radius is not None else 0.0
        except Exception:
            wr = 0.0
        return max(wr, float(self.terminal_capture_radius))

    def _sync_remaining_path_length(
        self,
        remaining_path_length=None,
        waypoint_radius=None,
    ):
        if remaining_path_length is None:
            remaining = float(self.remaining_path_length)
        else:
            remaining = float(remaining_path_length)

        arrival_radius = self._arrival_radius_for_sync(
            waypoint_radius=waypoint_radius,
        )
        return max(0.0, remaining - arrival_radius)

    def _update_eta_progress_speeds(
        self,
        dist_to_target,
        sync_remaining_path_length,
        now_abs,
    ):
        now_abs = float(now_abs)

        target_closure_speed = None
        sync_progress_speed = None

        if self._last_eta_progress_time is not None:
            dt = now_abs - float(self._last_eta_progress_time)
            if dt > 0.05:
                if self._last_target_distance_for_eta is not None:
                    raw_target_closure = (
                        float(self._last_target_distance_for_eta)
                        - float(dist_to_target)
                    ) / dt
                    if math.isfinite(raw_target_closure):
                        target_closure_speed = max(0.0, raw_target_closure)

                if self._last_sync_remaining_for_eta is not None:
                    raw_sync_progress = (
                        float(self._last_sync_remaining_for_eta)
                        - float(sync_remaining_path_length)
                    ) / dt
                    if math.isfinite(raw_sync_progress):
                        sync_progress_speed = max(0.0, raw_sync_progress)

        self._last_target_distance_for_eta = float(dist_to_target)
        self._last_sync_remaining_for_eta = float(sync_remaining_path_length)
        self._last_eta_progress_time = now_abs

        if target_closure_speed is not None:
            if self.target_closure_speed_ema is None:
                self.target_closure_speed_ema = target_closure_speed
            else:
                self.target_closure_speed_ema = (
                    (1.0 - self.eta_progress_alpha)
                    * float(self.target_closure_speed_ema)
                    + self.eta_progress_alpha * float(target_closure_speed)
                )

        if sync_progress_speed is not None:
            if self.sync_progress_speed_ema is None:
                self.sync_progress_speed_ema = sync_progress_speed
            else:
                self.sync_progress_speed_ema = (
                    (1.0 - self.eta_progress_alpha)
                    * float(self.sync_progress_speed_ema)
                    + self.eta_progress_alpha * float(sync_progress_speed)
                )

        return {
            "target_closure_speed": target_closure_speed,
            "sync_progress_speed": sync_progress_speed,
            "target_closure_speed_ema": self.target_closure_speed_ema,
            "sync_progress_speed_ema": self.sync_progress_speed_ema,
        }

    def _build_local_path_status(
        self,
        uav_ros,
        now_abs,
        dist_to_target=None,
        sync_remaining_path_length=None,
    ):
        actual_groundspeed = self._current_actual_groundspeed(uav_ros)
        predicted_arrival_time = None
        predicted_arrival_error = None
        now_abs = float(now_abs)
        if (
            actual_groundspeed is not None
            and actual_groundspeed > 1.0
            and self.full_path_common_hit_time > 0.0
        ):
            predicted_arrival_time = now_abs + (
                float(self.remaining_path_length) / max(float(actual_groundspeed), 1.0)
            )
            predicted_arrival_error = (
                predicted_arrival_time - float(self.full_path_common_hit_time)
            )
        if dist_to_target is None:
            if self.final_target_point is not None:
                dx = float(self.final_target_point[0]) - float(uav_ros.local_pose[0])
                dy = float(self.final_target_point[1]) - float(uav_ros.local_pose[1])
                dist_to_target = float(math.hypot(dx, dy))
            else:
                dist_to_target = float(self.remaining_path_length)
        else:
            dist_to_target = float(dist_to_target)

        if sync_remaining_path_length is None:
            sync_remaining_path_length = self._sync_remaining_path_length(
                remaining_path_length=self.remaining_path_length,
                waypoint_radius=None,
            )
        else:
            sync_remaining_path_length = float(sync_remaining_path_length)

        arrival_radius_for_sync = float(self._arrival_radius_for_sync())
        target_sync_remaining = max(
            0.0,
            float(dist_to_target) - arrival_radius_for_sync,
        )
        eta_progress = self._update_eta_progress_speeds(
            dist_to_target,
            sync_remaining_path_length,
            now_abs,
        )

        return {
            "uav_id": int(self.uav_id),
            "target_id": self._target_id_for_uav(self.uav_id),
            "path_ready": bool(self.full_path_plan_built),
            "full_path_started": bool(self.full_path_started_reported),
            "phase": self.phase,
            "path_length": float(self.path_length),
            "remaining_path_length": float(self.remaining_path_length),
            "sync_remaining_path_length": float(sync_remaining_path_length),
            "arrival_radius_for_sync": arrival_radius_for_sync,
            "dist_to_target": float(dist_to_target),
            "target_sync_remaining": target_sync_remaining,
            "target_closure_speed": eta_progress["target_closure_speed"],
            "sync_progress_speed": eta_progress["sync_progress_speed"],
            "target_closure_speed_ema": eta_progress["target_closure_speed_ema"],
            "sync_progress_speed_ema": eta_progress["sync_progress_speed_ema"],
            "eta": float(self.eta) if self.eta is not None else None,
            "control_path_source": getattr(
                self,
                "full_path_control_path_source",
                "unknown",
            ),
            "actual_groundspeed": actual_groundspeed,
            "predicted_arrival_time": predicted_arrival_time,
            "predicted_arrival_error": predicted_arrival_error,
            "predicted_arrival_used_for_control": False,
            "stamp": now_abs,
        }

    def _team_terminal_arrival_context(self, now_abs):
        """Return relative terminal sync info based on team_path_status."""
        predicted_by_uav = {}

        for uid in [1, 2, 3]:
            status = self.team_path_status.get(uid)
            if not status:
                continue

            pred = status.get("predicted_arrival_time")
            if pred is not None:
                try:
                    predicted_by_uav[uid] = float(pred)
                    continue
                except Exception:
                    pass

            try:
                remain = float(status.get("remaining_path_length", 0.0))
                gs = status.get("actual_groundspeed")
                gs = float(gs) if gs is not None else 0.0
                if remain > 0.0 and gs > 1.0:
                    predicted_by_uav[uid] = float(now_abs) + remain / gs
            except Exception:
                pass

        if len(predicted_by_uav) < 2:
            return None

        latest_uid = max(predicted_by_uav, key=lambda uid: predicted_by_uav[uid])
        earliest_uid = min(predicted_by_uav, key=lambda uid: predicted_by_uav[uid])
        latest_time = predicted_by_uav[latest_uid]
        earliest_time = predicted_by_uav[earliest_uid]

        own_pred = predicted_by_uav.get(self.uav_id)
        own_vs_latest = None
        if own_pred is not None:
            own_vs_latest = own_pred - latest_time

        return {
            "predicted_by_uav": predicted_by_uav,
            "latest_uid": latest_uid,
            "earliest_uid": earliest_uid,
            "latest_time": latest_time,
            "earliest_time": earliest_time,
            "spread": latest_time - earliest_time,
            "own_predicted_arrival_time": own_pred,
            "own_vs_latest": own_vs_latest,
        }

    def _broadcast_path_status(self, xbee, comm_info, uav_ros, now_abs):
        if not self.full_path_plan_built:
            return False

        now = time.time()
        if now - self.local_path_ready_broadcast_time < self.path_status_broadcast_interval:
            return False
        self.local_path_ready_broadcast_time = now

        try:
            self._update_path_metrics(
                current_xy=[uav_ros.local_pose[0], uav_ros.local_pose[1]],
                speed=self.full_path_expected_groundspeed,
            )
        except Exception:
            pass

        dist_to_target = None
        if self.final_target_point is not None:
            dx = float(self.final_target_point[0]) - float(uav_ros.local_pose[0])
            dy = float(self.final_target_point[1]) - float(uav_ros.local_pose[1])
            dist_to_target = math.hypot(dx, dy)

        sync_remaining_path_length = self._sync_remaining_path_length(
            remaining_path_length=self.remaining_path_length,
            waypoint_radius=None,
        )

        status = self._build_local_path_status(
            uav_ros,
            now_abs,
            dist_to_target=dist_to_target,
            sync_remaining_path_length=sync_remaining_path_length,
        )
        self.team_path_status[int(self.uav_id)] = status

        try:
            packet = comm_info.pack_simple_strike_path_status(
                self.leader_id,
                len(self.targets),
                status,
            )
            xbee.send_data_broadcast(packet)
            self._log_jsonl(
                "simple_strike_path_status_broadcast",
                **status,
            )
            return True
        except Exception as ex:
            rospy.logwarn_throttle(
                2.0,
                f"[SEAD] UAV{self.uav_id} path status broadcast failed: {ex}"
            )
            return False

    def apply_path_status(self, payload):
        if not isinstance(payload, dict):
            return False
        try:
            target_count = int(payload.get("target_count", len(self.targets)))
        except Exception:
            return False
        if target_count != len(self.targets):
            return False

        status = payload.get("status", payload)
        if not isinstance(status, dict):
            return False
        try:
            uid = int(status.get("uav_id", -1))
        except Exception:
            return False
        if uid not in [1, 2, 3]:
            return False
        if not bool(status.get("path_ready", False)):
            return False

        try:
            path_length = float(status.get("path_length", 0.0))
            remaining_path_length = float(
                status.get("remaining_path_length", path_length)
            )
        except Exception:
            return False
        if path_length <= 0.0:
            return False

        def _optional_float(value):
            if value is None:
                return None
            try:
                return float(value)
            except Exception:
                return None

        try:
            sync_remaining_path_length = status.get("sync_remaining_path_length")
            if sync_remaining_path_length is not None:
                sync_remaining_path_length = float(sync_remaining_path_length)
            else:
                sync_remaining_path_length = max(
                    0.0,
                    remaining_path_length - float(self.terminal_capture_radius),
                )
        except Exception:
            sync_remaining_path_length = max(
                0.0,
                remaining_path_length - float(self.terminal_capture_radius),
            )
        arrival_radius_for_sync = _optional_float(
            status.get("arrival_radius_for_sync")
        )
        if arrival_radius_for_sync is None:
            arrival_radius_for_sync = float(self.terminal_capture_radius)

        parsed = {
            "uav_id": uid,
            "path_ready": True,
            "full_path_started": bool(status.get("full_path_started", False)),
            "phase": status.get("phase"),
            "target_id": int(status.get("target_id", 0) or 0),
            "path_length": path_length,
            "remaining_path_length": max(0.0, remaining_path_length),
            "sync_remaining_path_length": max(0.0, sync_remaining_path_length),
            "arrival_radius_for_sync": float(arrival_radius_for_sync),
            "dist_to_target": _optional_float(status.get("dist_to_target")),
            "target_sync_remaining": _optional_float(
                status.get("target_sync_remaining")
            ),
            "target_closure_speed": _optional_float(
                status.get("target_closure_speed")
            ),
            "sync_progress_speed": _optional_float(
                status.get("sync_progress_speed")
            ),
            "target_closure_speed_ema": _optional_float(
                status.get("target_closure_speed_ema")
            ),
            "sync_progress_speed_ema": _optional_float(
                status.get("sync_progress_speed_ema")
            ),
            "eta": _optional_float(status.get("eta")),
            "control_path_source": status.get("control_path_source", "unknown"),
            "actual_groundspeed": _optional_float(status.get("actual_groundspeed")),
            "predicted_arrival_time": _optional_float(
                status.get("predicted_arrival_time")
            ),
            "predicted_arrival_error": _optional_float(
                status.get("predicted_arrival_error")
            ),
            "predicted_arrival_used_for_control": bool(
                status.get("predicted_arrival_used_for_control", False)
            ),
            "stamp": _optional_float(status.get("stamp")),
        }
        self.team_path_status[uid] = parsed
        self._log_jsonl(
            "simple_strike_path_status_received",
            **parsed,
        )
        return True

    def _leader_ensure_full_path_common_hit_time(
        self,
        xbee,
        comm_info,
        uav_states,
        now_abs,
    ):
        if self.uav_id != self.leader_id:
            return False
        if self.full_path_common_hit_time > 0.0:
            return self._broadcast_full_path_common_hit_time(xbee, comm_info)

        now_abs = float(now_abs)
        expected_ids = [1, 2, 3]
        eta_speed_for_common_time = max(float(self.effective_sync_speed_max), 1.0)

        def _status_remaining_length(status):
            try:
                return float(
                    status.get(
                        "remaining_path_length",
                        status.get("path_length", 0.0),
                    )
                )
            except Exception:
                return 0.0

        def _status_sync_remaining_length(status):
            try:
                if status.get("sync_remaining_path_length") is not None:
                    return float(status.get("sync_remaining_path_length"))
            except Exception:
                pass
            return max(
                0.0,
                _status_remaining_length(status) - float(self.terminal_capture_radius),
            )

        ready_ids = [
            uid
            for uid, status in self.team_path_status.items()
            if uid in expected_ids
            and status.get("path_ready")
            and _status_sync_remaining_length(status) > 0.0
        ]
        used_team_path_status = sorted(ready_ids) == expected_ids
        path_status_by_uav = {}
        team_etas = {}

        if used_team_path_status:
            path_status_by_uav = {
                uid: dict(self.team_path_status[uid])
                for uid in expected_ids
            }
            path_lengths_by_uav = {
                uid: _status_sync_remaining_length(path_status_by_uav[uid])
                for uid in expected_ids
            }
            max_eta = max(
                path_len / eta_speed_for_common_time
                for path_len in path_lengths_by_uav.values()
            )
        else:
            if self.path_ready_wait_started_at <= 0.0:
                self.path_ready_wait_started_at = now_abs
            wait_elapsed = now_abs - float(self.path_ready_wait_started_at)
            missing_path_ready = sorted(set(expected_ids) - set(ready_ids))
            if wait_elapsed < float(self.path_ready_wait_timeout):
                if now_abs - self.path_ready_status_log_time >= 1.0:
                    self.path_ready_status_log_time = now_abs
                    rospy.logwarn(
                        f"[SEAD] leader waiting team path_ready: "
                        f"have={sorted(ready_ids)}, missing={missing_path_ready}"
                    )
                    self._log_jsonl(
                        "simple_strike_waiting_team_path_ready",
                        have_path_ready=sorted(ready_ids),
                        missing_path_ready=missing_path_ready,
                        team_path_status=self.team_path_status,
                        wait_elapsed=wait_elapsed,
                    )
                return False

            team_etas = self._estimate_team_full_path_etas(uav_states)
            if sorted(team_etas.keys()) != expected_ids:
                rospy.logwarn_throttle(
                    1.0,
                    f"[SEAD] leader waiting fallback full-path ETAs, "
                    f"have={sorted(team_etas.keys())}"
                )
                return False
            path_lengths_by_uav = {
                int(uid): float(item.get("path_length", 0.0))
                for uid, item in team_etas.items()
            }
            max_eta = max(float(item["eta"]) for item in team_etas.values())

        if any(path_len <= 0.0 for path_len in path_lengths_by_uav.values()):
            rospy.logwarn_throttle(
                1.0,
                f"[SEAD] leader waiting valid full-path lengths: {path_lengths_by_uav}"
            )
            return False

        path_lengths = list(path_lengths_by_uav.values())
        effective_sync_speed_max = min(
            float(self.full_path_speed_max),
            float(self.effective_sync_speed_max),
        )
        duration_min_by_uav = {
            uid: path_len / max(float(effective_sync_speed_max), 0.1)
            for uid, path_len in path_lengths_by_uav.items()
        }
        duration_max_by_uav = {
            uid: path_len / max(float(self.full_path_speed_min), 0.1)
            for uid, path_len in path_lengths_by_uav.items()
        }
        lower_bound_duration = max(duration_min_by_uav.values())
        upper_bound_duration = min(duration_max_by_uav.values())
        min_path_length = min(path_lengths)
        max_path_length = max(path_lengths)
        required_speed_ratio = max_path_length / max(min_path_length, 1.0)
        available_speed_ratio = float(effective_sync_speed_max) / max(
            float(self.full_path_speed_min),
            0.1,
        )
        nominal_duration = max_eta + float(self.full_path_sync_margin)
        base_source = (
            "team_path_status"
            if used_team_path_status
            else "fallback_eta_no_team_path_status"
        )

        if lower_bound_duration <= upper_bound_duration:
            feasible_without_wait_or_loiter = True
            if upper_bound_duration - lower_bound_duration >= 4.0:
                selected_common_duration = min(
                    max(nominal_duration, lower_bound_duration + 2.0),
                    upper_bound_duration - 2.0,
                )
            else:
                selected_common_duration = 0.5 * (
                    lower_bound_duration + upper_bound_duration
                )
            common_duration_source = f"{base_source}_feasible_speed_window"
            reason = "speed_range_feasible"
        else:
            feasible_without_wait_or_loiter = False
            selected_common_duration = lower_bound_duration + 3.0
            common_duration_source = f"{base_source}_infeasible_use_lower_bound_plus_margin"
            reason = "speed_range_infeasible"
            rospy.logwarn(
                "[SEAD] full-path speed sync infeasible: "
                f"required_speed_ratio={required_speed_ratio:.2f}, "
                f"available_speed_ratio={available_speed_ratio:.2f}, "
                "shortest UAV may arrive early"
            )

        realistic_duration_by_uav = {
            uid: (
                path_len / max(float(effective_sync_speed_max), 1.0)
            ) * float(self.full_path_tracking_time_scale)
            for uid, path_len in path_lengths_by_uav.items()
        }
        realistic_lower_bound_duration = max(realistic_duration_by_uav.values())
        selected_common_duration = max(
            float(selected_common_duration),
            float(realistic_lower_bound_duration)
            + float(self.full_path_extra_start_margin),
        )

        current_time_for_common_hit_time = float(now_abs)
        candidate_common_hit_time = (
            float(current_time_for_common_hit_time) + float(selected_common_duration)
        )
        if not self._set_simple_strike_common_hit_time(
            candidate_common_hit_time,
            reason="leader_initial_common_hit_time",
        ):
            return False
        self.full_path_feasibility_reported = True
        self.last_full_path_common_hit_time_broadcast_time = 0.0
        self.full_path_common_hit_time_broadcast_count = 0

        rospy.logwarn(
            f"[SEAD] leader full-path common_hit_time="
            f"{self.full_path_common_hit_time:.3f}, source={common_duration_source}, "
            f"lower={lower_bound_duration:.1f}, upper={upper_bound_duration:.1f}"
        )
        self._log_jsonl(
            "simple_strike_full_path_common_hit_time_set",
            leader_id=self.leader_id,
            target_count=len(self.targets),
            used_team_path_status=used_team_path_status,
            path_status_by_uav=path_status_by_uav,
            path_lengths_by_uav=path_lengths_by_uav,
            sync_path_lengths_by_uav=path_lengths_by_uav,
            arrival_radius_for_sync=self.terminal_capture_radius,
            sync_to_capture_radius=True,
            duration_min_by_uav=duration_min_by_uav,
            duration_max_by_uav=duration_max_by_uav,
            realistic_duration_by_uav=realistic_duration_by_uav,
            min_path_length=min_path_length,
            max_path_length=max_path_length,
            lower_bound_duration=lower_bound_duration,
            realistic_lower_bound_duration=realistic_lower_bound_duration,
            upper_bound_duration=upper_bound_duration,
            required_speed_ratio=required_speed_ratio,
            available_speed_ratio=available_speed_ratio,
            nominal_duration=nominal_duration,
            selected_common_duration=selected_common_duration,
            full_path_tracking_time_scale=self.full_path_tracking_time_scale,
            full_path_extra_start_margin=self.full_path_extra_start_margin,
            current_time_for_common_hit_time=current_time_for_common_hit_time,
            common_duration_source=common_duration_source,
            feasible_without_wait_or_loiter=feasible_without_wait_or_loiter,
            reason=reason,
            speed_min=self.full_path_speed_min,
            speed_max=self.full_path_speed_max,
            speed_hard_max=self.full_path_speed_hard_max,
            effective_sync_speed_max=effective_sync_speed_max,
            full_path_expected_groundspeed=self.full_path_expected_groundspeed,
            eta_speed_for_common_time=eta_speed_for_common_time,
            full_path_common_hit_time=self.full_path_common_hit_time,
            team_etas=team_etas,
        )
        return self._broadcast_full_path_common_hit_time(xbee, comm_info)

    def _status_eta_for_common_time(self, status, now_ref):
        sync_remain = None
        target_sync_remain = None

        try:
            sync_remain = float(status.get("sync_remaining_path_length"))
        except Exception:
            try:
                sync_remain = max(
                    0.0,
                    float(status.get("remaining_path_length", 0.0))
                    - float(self.terminal_capture_radius),
                )
            except Exception:
                sync_remain = 0.0

        try:
            target_sync_remain = float(status.get("target_sync_remaining"))
        except Exception:
            target_sync_remain = None

        if (
            target_sync_remain is not None
            and target_sync_remain < float(self.terminal_eta_distance)
        ):
            closure_speed = status.get("target_closure_speed_ema")
            try:
                closure_speed = float(closure_speed)
            except Exception:
                closure_speed = 0.0

            if closure_speed >= float(self.min_target_closure_speed_for_eta):
                return (
                    target_sync_remain / max(closure_speed, 0.1),
                    "target_closure_terminal_eta",
                    target_sync_remain,
                    closure_speed,
                )

        progress_speed = status.get("sync_progress_speed_ema")
        try:
            progress_speed = float(progress_speed)
        except Exception:
            progress_speed = 0.0

        if progress_speed >= float(self.min_sync_progress_speed_for_eta):
            return (
                sync_remain / max(progress_speed, 0.1),
                "sync_progress_eta",
                sync_remain,
                progress_speed,
            )

        gs = status.get("actual_groundspeed")
        try:
            gs = float(gs) if gs is not None else 0.0
        except Exception:
            gs = 0.0

        eta_speed = max(
            float(gs),
            float(self.dynamic_sync_eta_speed_floor),
        )
        eta_speed = min(
            eta_speed,
            float(self.swiftwing_command_speed_max),
        )
        eta_speed = max(eta_speed, 1.0)

        return (
            sync_remain / eta_speed,
            "floor_groundspeed_eta",
            sync_remain,
            eta_speed,
        )

    def _leader_maybe_update_common_hit_time(self, xbee, comm_info, now_abs=None):
        if not self.dynamic_sync_enabled:
            return False
        if self.uav_id != self.leader_id:
            return False
        if self.full_path_common_hit_time <= 0.0:
            return False
        if self.common_hit_time_update_count >= self.max_common_hit_time_updates:
            return False

        now_wall = time.time()
        now_ref = float(now_abs) if now_abs is not None else now_wall

        if (
            now_wall - self.last_common_hit_time_update_time
            < self.dynamic_sync_update_interval
        ):
            return False

        eta_by_uav = {}
        eta_source_by_uav = {}
        eta_remaining_by_uav = {}
        eta_speed_used_by_uav = {}
        predicted_arrival_by_uav = {}
        status_by_uav = {}

        for uid in [1, 2, 3]:
            status = self.team_path_status.get(uid)
            if not status or not status.get("path_ready"):
                return False

            try:
                if status.get("sync_remaining_path_length") is not None:
                    remain = float(status.get("sync_remaining_path_length"))
                else:
                    remain = max(
                        0.0,
                        float(status.get("remaining_path_length", 0.0))
                        - float(self.terminal_capture_radius),
                    )
            except Exception:
                return False

            status_by_uav[uid] = status
            if remain <= 0.0:
                eta_by_uav[uid] = 0.0
                eta_source_by_uav[uid] = "already_at_capture_boundary"
                eta_remaining_by_uav[uid] = 0.0
                eta_speed_used_by_uav[uid] = None
                predicted_arrival_by_uav[uid] = now_ref
                continue

            eta, eta_source, eta_remaining, eta_speed_used = (
                self._status_eta_for_common_time(status, now_ref)
            )
            eta_by_uav[uid] = eta
            eta_source_by_uav[uid] = eta_source
            eta_remaining_by_uav[uid] = eta_remaining
            eta_speed_used_by_uav[uid] = eta_speed_used
            predicted_arrival_by_uav[uid] = now_ref + eta

        remaining_by_uav = {
            uid: float(status_by_uav[uid].get("remaining_path_length", 0.0))
            for uid in status_by_uav
        }
        sync_remaining_by_uav = {}
        for uid, status in status_by_uav.items():
            try:
                if status.get("sync_remaining_path_length") is not None:
                    sync_remaining_by_uav[uid] = float(
                        status.get("sync_remaining_path_length")
                    )
                else:
                    sync_remaining_by_uav[uid] = max(
                        0.0,
                        float(status.get("remaining_path_length", 0.0))
                        - float(self.terminal_capture_radius),
                    )
            except Exception:
                sync_remaining_by_uav[uid] = 0.0

        if any(
            remain <= float(self.dynamic_sync_stop_remaining_to_capture)
            for remain in sync_remaining_by_uav.values()
        ):
            if now_wall - self._common_hit_time_push_blocked_log_time >= 1.0:
                self._common_hit_time_push_blocked_log_time = now_wall
                self._log_jsonl(
                    "simple_strike_common_hit_time_push_blocked",
                    reason="team_member_near_capture_boundary",
                    sync_remaining_by_uav=sync_remaining_by_uav,
                    dynamic_sync_stop_remaining_to_capture=(
                        self.dynamic_sync_stop_remaining_to_capture
                    ),
                    current_common_hit_time=self.full_path_common_hit_time,
                )
            return False

        feasible_speed_min = min(
            float(self.full_path_speed_min),
            float(self.short_path_speed_min),
        )
        feasible_times = [
            now_ref + (remain / max(float(feasible_speed_min), 0.1))
            for remain in sync_remaining_by_uav.values()
            if remain > 0.0
        ]
        if not feasible_times:
            return False
        latest_feasible_common_hit_time = min(feasible_times) - float(
            self.dynamic_sync_shortest_feasible_margin
        )

        latest_uid = max(
            predicted_arrival_by_uav,
            key=lambda uid: predicted_arrival_by_uav[uid],
        )
        latest_arrival = float(predicted_arrival_by_uav[latest_uid])
        raw_candidate_common_hit_time = (
            latest_arrival + float(self.common_hit_time_push_margin)
        )
        candidate_common_hit_time = min(
            raw_candidate_common_hit_time,
            latest_feasible_common_hit_time,
        )

        if candidate_common_hit_time <= (
            float(self.full_path_common_hit_time)
            + float(self.common_hit_time_min_push)
        ):
            return False

        proposed_shift = (
            candidate_common_hit_time - float(self.full_path_common_hit_time)
        )
        proposed_shift = min(
            proposed_shift,
            float(self.common_hit_time_max_shift_per_update),
        )

        if proposed_shift <= float(self.common_hit_time_min_push):
            return False

        old_common_hit_time = float(self.full_path_common_hit_time)
        new_common_hit_time = old_common_hit_time + proposed_shift
        if not self._set_simple_strike_common_hit_time(
            new_common_hit_time,
            reason="leader_remaining_distance_time_push",
        ):
            return False
        self.common_hit_time_update_count += 1
        self.last_common_hit_time_update_time = now_wall
        self.last_full_path_common_hit_time_broadcast_time = 0.0
        self._broadcast_full_path_common_hit_time(xbee, comm_info)
        self._log_jsonl(
            "simple_strike_full_path_common_hit_time_updated",
            update_reason="remaining_distance_time_push",
            old_common_hit_time=old_common_hit_time,
            new_common_hit_time=self.full_path_common_hit_time,
            proposed_shift=proposed_shift,
            latest_uid=latest_uid,
            latest_arrival=latest_arrival,
            raw_candidate_common_hit_time=raw_candidate_common_hit_time,
            latest_feasible_common_hit_time=latest_feasible_common_hit_time,
            feasible_speed_min=feasible_speed_min,
            dynamic_sync_eta_speed_floor=self.dynamic_sync_eta_speed_floor,
            dynamic_sync_shortest_feasible_margin=(
                self.dynamic_sync_shortest_feasible_margin
            ),
            eta_by_uav=eta_by_uav,
            eta_source_by_uav=eta_source_by_uav,
            eta_remaining_by_uav=eta_remaining_by_uav,
            eta_speed_used_by_uav=eta_speed_used_by_uav,
            predicted_arrival_by_uav=predicted_arrival_by_uav,
            remaining_by_uav=remaining_by_uav,
            sync_remaining_by_uav=sync_remaining_by_uav,
            arrival_radius_for_sync=self.terminal_capture_radius,
            sync_to_capture_radius=True,
            actual_groundspeed_by_uav={
                uid: status_by_uav[uid].get("actual_groundspeed")
                for uid in status_by_uav
            },
            update_count=self.common_hit_time_update_count,
        )
        rospy.logwarn(
            f"[SEAD] leader pushed common_hit_time by {proposed_shift:.1f}s "
            f"to {self.full_path_common_hit_time:.3f}, latest=UAV{latest_uid}"
        )
        return True

    def _set_full_path_swiftwing_vector_control_source(self, uav_ros, reason):
        try:
            if hasattr(uav_ros, "set_offboard_control_source"):
                uav_ros.set_offboard_control_source("swiftwing_vector")
            else:
                uav_ros.offboard_control_source = "swiftwing_vector"
            uav_ros.keepoffboard = None
            uav_ros.defaultoffboard = None
        except Exception as ex:
            rospy.logwarn_throttle(
                2.0,
                f"[SEAD] UAV{self.uav_id} swiftwing_vector control source set failed: {ex}"
            )
            return

        if not self.full_path_control_source_reported:
            self.full_path_control_source_reported = True
            self._log_jsonl(
                "simple_strike_full_path_control_source_set",
                offboard_control_source=getattr(
                    uav_ros,
                    "offboard_control_source",
                    None,
                ),
                keepoffboard=getattr(uav_ros, "keepoffboard", None),
                defaultoffboard=getattr(uav_ros, "defaultoffboard", None),
                full_path_control_backend="swiftwing_vector",
                reason=reason,
            )

    def _set_full_path_position_waypoint_control_source(self, uav_ros, reason):
        try:
            if hasattr(uav_ros, "set_offboard_control_source"):
                uav_ros.set_offboard_control_source("position")
            else:
                uav_ros.offboard_control_source = "position"
            uav_ros.keepoffboard = None
            uav_ros.defaultoffboard = None
        except Exception as ex:
            rospy.logwarn_throttle(
                2.0,
                f"[SEAD] UAV{self.uav_id} position control source set failed: {ex}"
            )
            return

        if not self.full_path_control_source_reported:
            self.full_path_control_source_reported = True
            self._log_jsonl(
                "simple_strike_full_path_control_source_set",
                offboard_control_source=getattr(uav_ros, "offboard_control_source", None),
                keepoffboard=getattr(uav_ros, "keepoffboard", None),
                defaultoffboard=getattr(uav_ros, "defaultoffboard", None),
                full_path_control_backend="position_waypoint",
                reason=reason,
            )

    def _send_full_path_keepalive(self, uav_ros, reason, height=None):
        if self.simple_strike_control_mode == "swiftwing_vector":
            self._set_full_path_swiftwing_vector_control_source(uav_ros, reason)
        else:
            self._set_full_path_position_waypoint_control_source(uav_ros, reason)

        wait_reasons = {
            "waiting_common_hit_time",
            "waiting_full_path_plan",
            "waiting_assignment",
        }
        has_final_target_point = (
            self.final_target_point is not None and len(self.final_target_point) >= 2
        )
        dist_to_final_target = None
        keepalive_to_target_heading = False
        replayed = False
        if (
            self.simple_strike_control_mode == "swiftwing_vector"
            and str(reason) not in wait_reasons
            and hasattr(
                uav_ros,
                "replay_last_swiftwing_vector_setpoint",
            )
        ):
            try:
                replayed = bool(uav_ros.replay_last_swiftwing_vector_setpoint())
            except Exception as ex:
                rospy.logwarn_throttle(
                    2.0,
                    f"[SEAD] UAV{self.uav_id} SwiftWing vector replay failed: {ex}"
                )
                replayed = False

        if replayed:
            try:
                vx, vy, vz, speed, heading, vz_cmd = getattr(
                    uav_ros,
                    "last_swiftwing_vector_cmd",
                    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                )
                v_keepalive = float(speed)
                heading_keepalive = float(heading)
                vz_keepalive = float(vz_cmd)
            except Exception:
                v_keepalive = 0.0
                heading_keepalive = float(getattr(uav_ros, "yaw", 0.0))
                vz_keepalive = 0.0
        else:
            v_keepalive = max(9.5, 0.50 * float(self.cruise_speed))
            if str(reason) in wait_reasons and has_final_target_point:
                dx = float(self.final_target_point[0]) - float(uav_ros.local_pose[0])
                dy = float(self.final_target_point[1]) - float(uav_ros.local_pose[1])
                dist_to_final_target = float(math.hypot(dx, dy))
                heading_keepalive = math.atan2(dy, dx)
                keepalive_to_target_heading = True
            else:
                heading_keepalive = float(getattr(uav_ros, "yaw", 0.0))
                if has_final_target_point:
                    dx = float(self.final_target_point[0]) - float(uav_ros.local_pose[0])
                    dy = float(self.final_target_point[1]) - float(uav_ros.local_pose[1])
                    dist_to_final_target = float(math.hypot(dx, dy))
            vz_keepalive = 0.0
            if self.simple_strike_control_mode == "swiftwing_vector" and hasattr(
                uav_ros,
                "swiftwing_vector_control",
            ):
                uav_ros.swiftwing_vector_control(
                    v_keepalive,
                    heading_keepalive,
                    vz_keepalive,
                )
            elif self.simple_strike_control_mode == "position_waypoint":
                current_alt = float(uav_ros.local_pose[2])
                lookahead = max(80.0, 4.0 * float(self.cruise_speed))
                waypoint = [
                    float(uav_ros.local_pose[0]) + lookahead * math.cos(heading_keepalive),
                    float(uav_ros.local_pose[1]) + lookahead * math.sin(heading_keepalive),
                    float(height) if height is not None else current_alt,
                ]
                uav_ros.guide_to_waypoint(waypoint, heading_keepalive)

        now_wall = time.time()
        event_type = (
            "simple_strike_full_path_waiting_common_hit_time_swiftwing_keepalive"
            if self.simple_strike_control_mode == "swiftwing_vector"
            else "simple_strike_full_path_waiting_common_hit_time_position_keepalive"
        )
        rospy.logwarn_throttle(
            1.0,
            f"[SEAD] UAV{self.uav_id} waiting common_hit_time with "
            f"{self.simple_strike_control_mode} "
            f"keepalive: v={v_keepalive:.1f}, heading={heading_keepalive:.2f}, "
            f"replayed={replayed}, reason={reason}, "
            f"to_target_heading={keepalive_to_target_heading}"
        )
        if now_wall - self.full_path_wait_keepalive_log_time >= 1.0:
            self.full_path_wait_keepalive_log_time = now_wall
            self._log_jsonl(
                event_type,
                v_keepalive=v_keepalive,
                heading_keepalive=heading_keepalive,
                vz_keepalive=vz_keepalive,
                replayed_last_swiftwing_vector_cmd=replayed,
                keepalive_reason=reason,
                has_final_target_point=has_final_target_point,
                keepalive_to_target_heading=keepalive_to_target_heading,
                dist_to_final_target=dist_to_final_target,
                offboard_control_source=getattr(
                    uav_ros,
                    "offboard_control_source",
                    None,
                ),
                simple_strike_control_mode=self.simple_strike_control_mode,
                full_path_control_backend=self.full_path_control_backend,
                reason=reason,
            )

    def _compute_full_path_guidance(self, uav_ros, height, waypoint_radius, now_abs):
        if not self.full_path_plan_built:
            return None
        if self.full_path_common_hit_time <= 0.0:
            rospy.logwarn_throttle(
                1.0,
                f"[SEAD] UAV{self.uav_id} waiting full_path_common_hit_time"
            )
            return None
        if self.final_target_point is None:
            return None

        current_xy = [float(uav_ros.local_pose[0]), float(uav_ros.local_pose[1])]
        final_x = float(self.final_target_point[0])
        final_y = float(self.final_target_point[1])
        dist_to_target = float(
            np.linalg.norm([final_x - current_xy[0], final_y - current_xy[1]])
        )

        self.path_following.fw_path_index = max(0, int(self.full_path_path_index))
        self._update_path_metrics(
            current_xy=current_xy,
            speed=self.full_path_expected_groundspeed,
        )
        arrival_radius = self._arrival_radius_for_sync(
            waypoint_radius=waypoint_radius,
        )
        sync_remaining_path_length = self._sync_remaining_path_length(
            remaining_path_length=self.remaining_path_length,
            waypoint_radius=waypoint_radius,
        )
        time_left = float(self.full_path_common_hit_time) - float(now_abs)
        if time_left <= float(self.min_time_left_for_speed_sync):
            deadline_expired_or_too_close = True
            if sync_remaining_path_length > 150.0:
                v_time = min(
                    float(self.swiftwing_command_speed_max),
                    float(self.terminal_speed_max),
                )
            elif sync_remaining_path_length > 60.0:
                v_time = min(
                    float(self.deadline_expired_speed),
                    float(self.terminal_speed_max),
                )
            else:
                v_time = min(
                    float(self.terminal_final_hard_speed_cap),
                    float(self.terminal_speed_max),
                )
        else:
            deadline_expired_or_too_close = False
            v_time = float(sync_remaining_path_length) / max(float(time_left), 1.0)

        actual_groundspeed = None
        predicted_arrival_time = None
        predicted_arrival_error = None
        try:
            local_velo = getattr(uav_ros, "local_velo", None)
            if local_velo is not None and len(local_velo) >= 2:
                vx = float(local_velo[0])
                vy = float(local_velo[1])
                actual_groundspeed = float(math.hypot(vx, vy))
                predicted_arrival_time = float(now_abs) + (
                    float(self.remaining_path_length) / max(actual_groundspeed, 1.0)
                )
                predicted_arrival_error = (
                    predicted_arrival_time - float(self.full_path_common_hit_time)
                )
        except Exception:
            actual_groundspeed = None
            predicted_arrival_time = None
            predicted_arrival_error = None

        predicted_arrival_used_for_control = False
        terminal_relative_context = None
        terminal_relative_mode = False
        own_vs_latest = None
        team_arrival_spread = None
        terminal_relative_speed_role = "disabled"
        v_feedback = float(v_time)

        if self.simple_strike_control_mode == "swiftwing_vector":
            speed_upper = min(
                float(self.full_path_speed_max),
                float(self.swiftwing_command_speed_max),
            )
        else:
            speed_upper = float(self.full_path_speed_max)

        terminal_speed_limited = False
        if (
            self.simple_strike_control_mode == "swiftwing_vector"
            and dist_to_target < float(self.terminal_slowdown_distance)
        ):
            speed_upper = min(speed_upper, float(self.terminal_speed_max))
            terminal_speed_limited = True
        if (
            self.simple_strike_control_mode == "swiftwing_vector"
            and dist_to_target < float(self.terminal_final_hard_speed_cap_distance)
        ):
            speed_upper = min(speed_upper, float(self.terminal_final_hard_speed_cap))
            terminal_speed_limited = True

        speed_lower = float(self.full_path_speed_min)
        if (
            self.simple_strike_control_mode == "swiftwing_vector"
            and v_feedback < float(self.full_path_speed_min)
            and not deadline_expired_or_too_close
        ):
            speed_lower = min(speed_lower, float(self.short_path_speed_min))

        speed_saturated_low = v_feedback < float(speed_lower)
        speed_saturated_high = v_feedback > float(speed_upper)
        v_limited = min(
            max(v_feedback, float(speed_lower)),
            float(speed_upper),
        )
        v_cmd = 0.25 * float(self.full_path_speed_cmd_prev) + 0.75 * v_limited
        raw_v_cmd = float(v_cmd)
        v_cmd = min(
            max(v_cmd, float(speed_lower)),
            float(speed_upper),
        )
        self.full_path_speed_cmd_prev = v_cmd
        cmd_speed_sent_to_swiftwing = (
            min(float(v_cmd), float(self.swiftwing_command_speed_max))
            if self.simple_strike_control_mode == "swiftwing_vector"
            else float(v_cmd)
        )

        terminal_direct_guidance_active = False
        terminal_direct_guidance_safe = False
        terminal_direct_guidance_block_reason = None

        desire_point = None
        path = self.path_following.path or []
        try:
            if path and len(path) >= 2 and self.full_path_path_index < len(path) - 1:
                desire_point, new_index, _, _ = (
                    self.path_following.get_desirePoint_withWindow(
                        v_cmd,
                        uav_ros.local_pose[0],
                        uav_ros.local_pose[1],
                        uav_ros.yaw,
                        self.full_path_path_index,
                    )
                )
                self.full_path_path_index = max(
                    self.full_path_path_index,
                    int(new_index),
                )
                self.path_following.fw_path_index = self.full_path_path_index
        except Exception as ex:
            rospy.logwarn_throttle(
                1.0,
                f"[SEAD] UAV{self.uav_id} full-path desire point failed: {ex}"
            )
            desire_point = None

        if desire_point is None:
            try:
                wp = self.path_following.get_fixed_wing_waypoint(
                    uav_ros.local_pose[0],
                    uav_ros.local_pose[1],
                    min_dist=10.0,
                    default_z=height,
                    update=False,
                )
                if wp is not None:
                    desire_point = np.array([float(wp[0]), float(wp[1])])
                    self.full_path_path_index = max(
                        self.full_path_path_index,
                        int(getattr(self.path_following, "fw_path_index", 0)),
                    )
            except Exception as ex:
                rospy.logwarn_throttle(
                    1.0,
                    f"[SEAD] UAV{self.uav_id} full-path waypoint fallback failed: {ex}"
                )
        if desire_point is None:
            desire_point = np.array([final_x, final_y])

        if (
            self.simple_strike_control_mode == "swiftwing_vector"
            and dist_to_target < float(self.terminal_direct_guidance_distance)
            and self.final_target_point is not None
        ):
            current_pose = [
                float(uav_ros.local_pose[0]),
                float(uav_ros.local_pose[1]),
                float(getattr(uav_ros, "yaw", 0.0)),
            ]
            final_pose = [
                final_x,
                final_y,
                math.atan2(
                    final_y - float(uav_ros.local_pose[1]),
                    final_x - float(uav_ros.local_pose[0]),
                ),
            ]
            detail = self._segment_or_path_blocked_detail(
                current_pose,
                final_pose,
                zones=self._planner_zones(),
                rmin=max(float(self.rmin), 1.0),
                step=5.0,
                clearance_threshold=float(
                    self.terminal_direct_guidance_safe_clearance
                ),
            )
            terminal_direct_guidance_safe = not bool(detail.get("blocked", False))

            if terminal_direct_guidance_safe:
                desire_point = np.array([final_x, final_y])
                terminal_direct_guidance_active = True
            else:
                terminal_direct_guidance_block_reason = detail.get(
                    "path_block_reason"
                )

        v_z_cmd = 0.3 * (float(height) - float(uav_ros.local_pose[2]))
        v_z_cmd = min(max(v_z_cmd, -3.0), 3.0)

        dx = float(desire_point[0]) - float(uav_ros.local_pose[0])
        dy = float(desire_point[1]) - float(uav_ros.local_pose[1])
        if math.hypot(dx, dy) > 1.0:
            heading_cmd = math.atan2(dy, dx)
        else:
            heading_cmd = float(getattr(uav_ros, "yaw", 0.0))

        path_stretch_active = False
        path_stretch_reason = None
        delay_point = None

        return {
            "current_xy": current_xy,
            "final_target_point": self.final_target_point,
            "dist_to_target": dist_to_target,
            "time_left": time_left,
            "deadline_expired_or_too_close": deadline_expired_or_too_close,
            "deadline_expired_speed": self.deadline_expired_speed,
            "sync_remaining_path_length": sync_remaining_path_length,
            "arrival_radius_for_sync": arrival_radius,
            "sync_to_capture_radius": True,
            "v_des": v_time,
            "v_time": v_time,
            "v_feedback": v_feedback,
            "speed_lower": speed_lower,
            "short_path_speed_min": self.short_path_speed_min,
            "speed_upper": speed_upper,
            "terminal_speed_limited": terminal_speed_limited,
            "terminal_relative_mode": terminal_relative_mode,
            "terminal_relative_context": terminal_relative_context,
            "terminal_relative_speed_role": terminal_relative_speed_role,
            "team_arrival_spread": team_arrival_spread,
            "own_vs_latest": own_vs_latest,
            "v_limited": v_limited,
            "v_cmd": v_cmd,
            "raw_v_cmd": raw_v_cmd,
            "cmd_speed_sent_to_swiftwing": cmd_speed_sent_to_swiftwing,
            "speed_saturated_low": speed_saturated_low,
            "speed_saturated_high": speed_saturated_high,
            "desire_point": desire_point,
            "heading_cmd": heading_cmd,
            "v_z_cmd": v_z_cmd,
            "actual_groundspeed": actual_groundspeed,
            "predicted_arrival_time": predicted_arrival_time,
            "predicted_arrival_error": predicted_arrival_error,
            "predicted_arrival_used_for_control": predicted_arrival_used_for_control,
            "terminal_direct_guidance_active": terminal_direct_guidance_active,
            "terminal_direct_guidance_safe": terminal_direct_guidance_safe,
            "terminal_direct_guidance_block_reason": (
                terminal_direct_guidance_block_reason
            ),
            "path_stretch_active": path_stretch_active,
            "path_stretch_reason": path_stretch_reason,
            "delay_point": delay_point,
        }

    def _send_full_path_control(self, uav_ros, guidance, height):
        if self.simple_strike_control_mode == "swiftwing_vector":
            self._set_full_path_swiftwing_vector_control_source(
                uav_ros,
                reason="active_full_path_swiftwing_vector_sync",
            )
            if not hasattr(uav_ros, "swiftwing_vector_control"):
                rospy.logerr_throttle(
                    1.0,
                    f"[SEAD] UAV{self.uav_id} missing swiftwing_vector_control(); "
                    "cannot run simple strike"
                )
                self._log_jsonl(
                    "simple_strike_swiftwing_vector_missing_interface",
                    target_id=self._target_id_for_uav(self.uav_id),
                    full_path_control_backend="swiftwing_vector",
                )
                return None
            cmd_speed = float(guidance["v_cmd"])
            if self.simple_strike_control_mode == "swiftwing_vector":
                cmd_speed = min(cmd_speed, float(self.swiftwing_command_speed_max))
            ok = uav_ros.swiftwing_vector_control(
                cmd_speed,
                guidance["heading_cmd"],
                guidance["v_z_cmd"],
            )
            if not ok:
                rospy.logwarn_throttle(
                    1.0,
                    f"[SEAD] UAV{self.uav_id} SwiftWing vector publish failed"
                )
                return None
            return {
                "control_interface": "swiftwing_vector_control",
                "control_topic": f"/{getattr(uav_ros, 'uav_name', 'uav')}/control_signal/vector",
                "coordinate_frame": "ENU_VECTOR",
                "raw_v_cmd": guidance["v_cmd"],
                "cmd_speed_sent_to_swiftwing": cmd_speed,
                "swiftwing_command_speed_max": self.swiftwing_command_speed_max,
            }

        self._set_full_path_position_waypoint_control_source(
            uav_ros,
            reason="active_full_path_position_waypoint_sync",
        )
        desire_point = guidance["desire_point"]
        waypoint = [
            float(desire_point[0]),
            float(desire_point[1]),
            float(height),
        ]
        uav_ros.guide_to_waypoint(waypoint, guidance["heading_cmd"])
        return {
            "control_interface": "guide_to_waypoint",
            "control_topic": f"/{getattr(uav_ros, 'uav_name', 'uav')}/mavros/setpoint_raw/local",
            "coordinate_frame": "ENU_POSITION_TARGET",
        }

    def _guide_full_path_swiftwing_vector_sync(
        self,
        uav_ros,
        height,
        waypoint_radius,
        now_abs,
    ):
        guidance = self._compute_full_path_guidance(
            uav_ros,
            height,
            waypoint_radius,
            now_abs,
        )
        if guidance is None:
            if self.full_path_common_hit_time <= 0.0:
                self._send_full_path_keepalive(
                    uav_ros,
                    reason="waiting_common_hit_time",
                    height=height,
            )
            return False

        self._maybe_freeze_common_hit_time(
            uav_ros=uav_ros,
            guidance=guidance,
            now_abs=now_abs,
        )
        control_meta = self._send_full_path_control(uav_ros, guidance, height)
        if control_meta is None:
            return False

        desire_point = guidance["desire_point"]
        desire_point_json = [float(desire_point[0]), float(desire_point[1])]
        event_type = (
            "simple_strike_full_path_swiftwing_vector_active"
            if self.simple_strike_control_mode == "swiftwing_vector"
            else "simple_strike_full_path_position_waypoint_active"
        )
        now_wall = time.time()
        rospy.logwarn_throttle(
            1.0,
            f"[SEAD] UAV{self.uav_id} full_path {self.simple_strike_control_mode} active: "
            f"rem={self.remaining_path_length:.1f}, "
            f"time_left={guidance['time_left']:.1f}, "
            f"v_des={guidance['v_des']:.1f}, v_cmd={guidance['v_cmd']:.1f}, "
            f"heading={guidance['heading_cmd']:.3f}, vz={guidance['v_z_cmd']:.2f}"
        )
        if (
            (guidance["speed_saturated_high"] or guidance["speed_saturated_low"])
            and now_wall - self.full_path_saturation_log_time >= 1.0
        ):
            self.full_path_saturation_log_time = now_wall
            if guidance["speed_saturated_high"]:
                rospy.logwarn(
                    "[SEAD] full-path required speed above max; "
                    "simultaneous hit may be late"
                )
                saturation = "high"
            else:
                rospy.logwarn(
                    "[SEAD] full-path required speed below min; without "
                    "loiter/path-stretch aircraft may arrive early"
                )
                saturation = "low"
            self._log_jsonl(
                "simple_strike_full_path_speed_saturation",
                target_id=self._target_id_for_uav(self.uav_id),
                saturation=saturation,
                common_hit_time=self.full_path_common_hit_time,
                now_abs=now_abs,
                time_left=guidance["time_left"],
                deadline_expired_or_too_close=guidance[
                    "deadline_expired_or_too_close"
                ],
                deadline_expired_speed=guidance["deadline_expired_speed"],
                remaining_path_length=self.remaining_path_length,
                sync_remaining_path_length=guidance["sync_remaining_path_length"],
                arrival_radius_for_sync=guidance["arrival_radius_for_sync"],
                sync_to_capture_radius=guidance["sync_to_capture_radius"],
                dist_to_target=guidance["dist_to_target"],
                v_des=guidance["v_des"],
                v_time=guidance["v_time"],
                v_feedback=guidance["v_feedback"],
                speed_lower=guidance["speed_lower"],
                short_path_speed_min=guidance["short_path_speed_min"],
                speed_upper=guidance["speed_upper"],
                terminal_speed_limited=guidance["terminal_speed_limited"],
                terminal_relative_mode=guidance["terminal_relative_mode"],
                terminal_relative_context=guidance["terminal_relative_context"],
                terminal_relative_speed_role=guidance[
                    "terminal_relative_speed_role"
                ],
                team_arrival_spread=guidance["team_arrival_spread"],
                own_vs_latest=guidance["own_vs_latest"],
                required_speed_raw=guidance["v_des"],
                v_limited=guidance["v_limited"],
                v_cmd=guidance["v_cmd"],
                raw_v_cmd=guidance["raw_v_cmd"],
                cmd_speed_sent_to_swiftwing=control_meta.get(
                    "cmd_speed_sent_to_swiftwing",
                    guidance["cmd_speed_sent_to_swiftwing"],
                ),
                required_speed_cmd=guidance["v_cmd"],
                speed_min=self.full_path_speed_min,
                speed_max=self.full_path_speed_max,
                effective_sync_speed_max=self.effective_sync_speed_max,
                speed_hard_max=self.full_path_speed_hard_max,
                predicted_arrival_error=guidance["predicted_arrival_error"],
                predicted_arrival_used_for_control=guidance[
                    "predicted_arrival_used_for_control"
                ],
                terminal_direct_guidance_active=guidance[
                    "terminal_direct_guidance_active"
                ],
                terminal_direct_guidance_safe=guidance[
                    "terminal_direct_guidance_safe"
                ],
                terminal_direct_guidance_block_reason=guidance[
                    "terminal_direct_guidance_block_reason"
                ],
                simple_strike_control_mode=self.simple_strike_control_mode,
                full_path_control_backend=self.full_path_control_backend,
            )

        if now_wall - self.full_path_speed_log_time >= 1.0:
            self.full_path_speed_log_time = now_wall
            self._log_jsonl(
                event_type,
                simple_strike_control_mode=self.simple_strike_control_mode,
                full_path_control_backend=self.full_path_control_backend,
                target_id=self._target_id_for_uav(self.uav_id),
                final_target_point=self.final_target_point,
                common_hit_time=self.full_path_common_hit_time,
                now_abs=now_abs,
                time_left=guidance["time_left"],
                deadline_expired_or_too_close=guidance[
                    "deadline_expired_or_too_close"
                ],
                deadline_expired_speed=guidance["deadline_expired_speed"],
                path_length=self.path_length,
                remaining_path_length=self.remaining_path_length,
                sync_remaining_path_length=guidance["sync_remaining_path_length"],
                arrival_radius_for_sync=guidance["arrival_radius_for_sync"],
                sync_to_capture_radius=guidance["sync_to_capture_radius"],
                dist_to_target=guidance["dist_to_target"],
                v_des=guidance["v_des"],
                v_time=guidance["v_time"],
                v_feedback=guidance["v_feedback"],
                speed_lower=guidance["speed_lower"],
                short_path_speed_min=guidance["short_path_speed_min"],
                speed_upper=guidance["speed_upper"],
                terminal_speed_limited=guidance["terminal_speed_limited"],
                terminal_relative_mode=guidance["terminal_relative_mode"],
                terminal_relative_context=guidance["terminal_relative_context"],
                terminal_relative_speed_role=guidance[
                    "terminal_relative_speed_role"
                ],
                team_arrival_spread=guidance["team_arrival_spread"],
                own_vs_latest=guidance["own_vs_latest"],
                required_speed_raw=guidance["v_des"],
                v_limited=guidance["v_limited"],
                v_cmd=guidance["v_cmd"],
                raw_v_cmd=guidance["raw_v_cmd"],
                cmd_speed_sent_to_swiftwing=control_meta.get(
                    "cmd_speed_sent_to_swiftwing",
                    guidance["cmd_speed_sent_to_swiftwing"],
                ),
                required_speed_cmd=guidance["v_cmd"],
                speed_min=self.full_path_speed_min,
                speed_max=self.full_path_speed_max,
                effective_sync_speed_max=self.effective_sync_speed_max,
                speed_hard_max=self.full_path_speed_hard_max,
                speed_saturated_low=guidance["speed_saturated_low"],
                speed_saturated_high=guidance["speed_saturated_high"],
                heading_cmd=guidance["heading_cmd"],
                v_z_cmd=guidance["v_z_cmd"],
                desire_point=desire_point_json,
                path_index=self.full_path_path_index,
                actual_groundspeed=guidance["actual_groundspeed"],
                predicted_arrival_time=guidance["predicted_arrival_time"],
                predicted_arrival_error=guidance["predicted_arrival_error"],
                predicted_arrival_used_for_control=guidance[
                    "predicted_arrival_used_for_control"
                ],
                terminal_direct_guidance_active=guidance[
                    "terminal_direct_guidance_active"
                ],
                terminal_direct_guidance_safe=guidance[
                    "terminal_direct_guidance_safe"
                ],
                terminal_direct_guidance_block_reason=guidance[
                    "terminal_direct_guidance_block_reason"
                ],
                path_stretch_active=guidance["path_stretch_active"],
                path_stretch_reason=guidance["path_stretch_reason"],
                delay_point=guidance["delay_point"],
                control_interface=control_meta["control_interface"],
                control_topic=control_meta["control_topic"],
                coordinate_frame=control_meta["coordinate_frame"],
                raw_control_v_cmd=control_meta.get("raw_v_cmd"),
                swiftwing_command_speed_max=control_meta.get(
                    "swiftwing_command_speed_max"
                ),
                offboard_control_source=getattr(uav_ros, "offboard_control_source", None),
                no_release_point=True,
                no_hold_release=True,
                no_final_attack_phase=True,
            )

        arrival_radius = self._arrival_radius_for_sync(
            waypoint_radius=waypoint_radius,
        )
        prev_dist_to_target = self.prev_dist_to_target
        dist_decreasing = True
        if prev_dist_to_target is not None:
            dist_decreasing = (
                guidance["dist_to_target"] <= float(prev_dist_to_target) + 5.0
            )

        passed_target = False
        if (
            prev_dist_to_target is not None
            and float(prev_dist_to_target) < float(self.terminal_cross_track_radius)
            and guidance["dist_to_target"] > float(prev_dist_to_target) + 20.0
        ):
            passed_target = True
        self.prev_dist_to_target = guidance["dist_to_target"]

        if guidance["dist_to_target"] <= arrival_radius or passed_target:
            if not self.target_reached_reported:
                self.target_reached_reported = True
                self.mission_flag = True
                self.phase = "COMPLETE"
                rospy.logwarn(f"[SEAD] UAV{self.uav_id} target reached, entering LOITER")
                self._log_jsonl(
                    "simple_strike_target_reached",
                    assigned_target=self.assigned_target,
                    target_id=self._target_id_for_uav(self.uav_id),
                    final_target_point=self.final_target_point,
                    common_hit_time=self.full_path_common_hit_time,
                    message_semantics="common_hit_time",
                    simple_strike_control_mode=self.simple_strike_control_mode,
                    control_interface=control_meta["control_interface"],
                    reason=(
                        "passed_target_line"
                        if passed_target
                        else "within_accept_radius"
                    ),
                    arrival_radius=arrival_radius,
                    passed_target=passed_target,
                    prev_dist_to_target=prev_dist_to_target,
                    dist_to_target=guidance["dist_to_target"],
                    dist_decreasing=dist_decreasing,
                    no_release_point=True,
                    no_hold_release=True,
                    no_final_attack_phase=True,
                )
                self._freeze_simple_strike_common_hit_time(
                    "target_reached",
                    distance_to_target=guidance["dist_to_target"],
                )
                uav_ros.set_mode("LOITER")
            return True
        return False

    def _run_full_path_swiftwing_vector_sync(
        self,
        xbee,
        comm_info,
        uav_ros,
        new_timer,
        gcs,
        height,
        waypoint_radius,
        uav_states,
        now_abs,
    ):
        if self.simple_strike_control_mode == "swiftwing_vector":
            self._set_full_path_swiftwing_vector_control_source(
                uav_ros,
                reason="enter_full_path_swiftwing_vector_sync",
            )
        else:
            self._set_full_path_position_waypoint_control_source(
                uav_ros,
                reason="enter_full_path_position_waypoint_sync",
            )
        if self.assigned_target is None:
            self.assigned_target = self.assignment_map.get(self.uav_id)
        if self.assigned_target is None:
            if new_timer.check_period(0.02, self.previous_control_time):
                self.previous_control_time = time.time()
                self._send_full_path_keepalive(
                    uav_ros,
                    reason="missing_assigned_target",
                    height=height,
                )
            return

        if self.full_path_start_time <= 0.0:
            self.full_path_start_time = float(now_abs)
        if not self.full_path_plan_built:
            if not self._build_full_path_for_uav(uav_ros, height):
                if new_timer.check_period(0.02, self.previous_control_time):
                    self.previous_control_time = time.time()
                    self._send_full_path_keepalive(
                        uav_ros,
                        reason="waiting_full_path_plan",
                        height=height,
                    )
                return
            self._send_gcs_once(
                xbee,
                comm_info,
                gcs,
                "target_assigned",
                f"UAV{self.uav_id} -> target_id={self.assigned_target['target_id']}",
            )

        if self.full_path_plan_built:
            self._broadcast_path_status(xbee, comm_info, uav_ros, now_abs)

        min_start_lead_time = 20.0
        if self.full_path_common_hit_time > 0.0:
            time_left_for_start = float(self.full_path_common_hit_time) - float(now_abs)
            if time_left_for_start < min_start_lead_time:
                self._log_jsonl(
                    "simple_strike_common_hit_time_stale_for_local_start",
                    common_hit_time=self.full_path_common_hit_time,
                    now_abs=now_abs,
                    time_left=time_left_for_start,
                    remaining_path_length=self.remaining_path_length,
                    path_length=self.path_length,
                    target_id=self._target_id_for_uav(self.uav_id),
                    final_target_point=self.final_target_point,
                )

        if self.uav_id == self.leader_id:
            self._leader_ensure_full_path_common_hit_time(
                xbee,
                comm_info,
                uav_states,
                now_abs,
            )

        if self.full_path_common_hit_time <= 0.0:
            if new_timer.check_period(0.02, self.previous_control_time):
                self.previous_control_time = time.time()
                self._send_full_path_keepalive(
                    uav_ros,
                    reason="waiting_common_hit_time",
                    height=height,
                )
            return

        self._maybe_freeze_common_hit_time(uav_ros=uav_ros, now_abs=now_abs)
        self._leader_maybe_update_common_hit_time(xbee, comm_info, now_abs=now_abs)

        if self.simple_strike_control_mode == "swiftwing_vector":
            self.phase = "FULL_PATH_SWIFTWING_VECTOR_SYNC"
            started_event = "simple_strike_full_path_swiftwing_vector_started"
            started_interface = "swiftwing_vector_control"
            started_topic = f"/{getattr(uav_ros, 'uav_name', 'uav')}/control_signal/vector"
            started_frame = "ENU_VECTOR"
        else:
            self.phase = "FULL_PATH_POSITION_WAYPOINT_SYNC"
            started_event = "simple_strike_full_path_position_waypoint_started"
            started_interface = "guide_to_waypoint"
            started_topic = f"/{getattr(uav_ros, 'uav_name', 'uav')}/mavros/setpoint_raw/local"
            started_frame = "ENU_POSITION_TARGET"
        if not self.full_path_started_reported:
            self.full_path_started_reported = True
            self.team_path_status.setdefault(int(self.uav_id), {})[
                "full_path_started"
            ] = True
            self.team_path_status[int(self.uav_id)]["phase"] = self.phase
            rospy.logwarn(
                f"[SEAD] UAV{self.uav_id} entering {self.phase}"
            )
            self._log_jsonl(
                started_event,
                simple_strike_control_mode=self.simple_strike_control_mode,
                target_id=self._target_id_for_uav(self.uav_id),
                final_target_point=self.final_target_point,
                common_hit_time=self.full_path_common_hit_time,
                control_interface=started_interface,
                control_topic=started_topic,
                coordinate_frame=started_frame,
                full_path_control_backend=self.full_path_control_backend,
                no_release_point=True,
                no_hold_release=True,
                no_final_attack_phase=True,
            )
        self._maybe_freeze_common_hit_time(uav_ros=uav_ros, now_abs=now_abs)

        if not new_timer.check_period(0.02, self.previous_control_time):
            return
        self.previous_control_time = time.time()
        reached = self._guide_full_path_swiftwing_vector_sync(
            uav_ros,
            height,
            waypoint_radius,
            now_abs,
        )
        if reached:
            self._send_gcs_once(
                xbee,
                comm_info,
                gcs,
                "target_reached",
                f"UAV{self.uav_id} target reached",
            )

    def _log_phase_heartbeat(self, uav_ros, now_abs):
        if float(now_abs) - float(self.phase_heartbeat_log_time) < 2.0:
            return
        self.phase_heartbeat_log_time = float(now_abs)
        self._log_jsonl(
            "simple_strike_phase_heartbeat",
            phase=self.phase,
            has_assignment=bool(self.assignment_map.get(self.uav_id)),
            has_common_hit_time=self.full_path_common_hit_time > 0.0,
            local_pose=list(getattr(uav_ros, "local_pose", [])),
            final_target_point=self.final_target_point,
            target_id=self._target_id_for_uav(self.uav_id),
            full_path_control_backend=self.full_path_control_backend,
            simple_strike_control_mode=self.simple_strike_control_mode,
        )

    def run(self, xbee, comm_info, uav_ros, new_timer, gcs, height, waypoint_radius):
        if self.mission_flag:
            return
        self._broadcast_state(xbee, comm_info, uav_ros, new_timer)
        now_abs = self._mission_time(new_timer)
        self._log_phase_heartbeat(uav_ros, now_abs)
        uav_states = self._online_states(comm_info, uav_ros)
        if not self._build_assignment_if_ready(xbee, comm_info, gcs, uav_states):
            if new_timer.check_period(0.02, self.previous_control_time):
                self.previous_control_time = time.time()
                self._send_full_path_keepalive(
                    uav_ros,
                    reason="waiting_assignment",
                    height=height,
                )
            return

        self._run_full_path_swiftwing_vector_sync(
            xbee,
            comm_info,
            uav_ros,
            new_timer,
            gcs,
            height,
            waypoint_radius,
            uav_states,
            now_abs,
        )
