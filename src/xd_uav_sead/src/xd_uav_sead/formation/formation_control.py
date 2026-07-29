from dataclasses import dataclass
from enum import Enum
from math import atan2, cos, sin
from typing import Dict, Iterable, List, Optional

import numpy as np


class FormationShape(Enum):
    VEE = 1
    ECHELON_LEFT = 2
    ECHELON_RIGHT = 3
    TRAIL = 4
    TRIANGLE = 5
    WEDGE_WIDE = 6
    ARROW = 7
    INVERTED_VEE = 8


class FormationPhase(Enum):
    IDLE = 0
    ASSEMBLE = 1
    HOLD = 2
    RELEASE = 3


@dataclass
class FormationConfig:
    enabled: bool = True
    shape: FormationShape = FormationShape.VEE
    spacing: float = 220.0
    standoff_distance: float = 5000.0
    safe_separation: float = 140.0
    altitude_step: float = 20.0
    lookahead_time: float = 4.0
    slot_gain: float = 0.9
    consensus_gain: float = 0.3
    obstacle_gain: float = 1.15
    sync_margin: float = 18.0
    max_delay_comp: float = 1.5
    hold_radius: float = 250.0
    release_slack: float = 0.6
    broadcast_interval: float = 0.2
    leader_id: int = 0
    desired_target_time: float = 0.0


@dataclass
class TeamState:
    uav_id: int
    timestamp: float
    position: np.ndarray
    velocity: np.ndarray
    yaw: float
    phase: int
    slot_id: int
    sync_eta: Optional[float]
    common_target_time: float


def _as_xy(point) -> np.ndarray:
    arr = np.asarray(point[:2], dtype=float)
    if arr.shape != (2,):
        arr = np.zeros(2, dtype=float)
    return arr


def _clip_norm(vec: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-6 or norm <= limit:
        return vec
    return vec * (limit / norm)


def _unit(vec: np.ndarray, fallback=None) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-6:
        if fallback is None:
            return np.array([1.0, 0.0], dtype=float)
        fallback = np.asarray(fallback, dtype=float)
        fb_norm = float(np.linalg.norm(fallback))
        if fb_norm < 1e-6:
            return np.array([1.0, 0.0], dtype=float)
        return fallback / fb_norm
    return vec / norm


def _rotation_from(direction: np.ndarray) -> np.ndarray:
    d = _unit(direction)
    return np.array([[d[0], -d[1]], [d[1], d[0]]], dtype=float)


class FormationController:
    def __init__(self, uav_id: int, config: Optional[FormationConfig] = None):
        self.uav_id = int(uav_id)
        self.config = config or FormationConfig()
        self.remote_states: Dict[int, TeamState] = {}
        self.reset(keep_config=True)

    def reset(self, keep_config: bool = True):
        if not keep_config:
            self.config = FormationConfig()
        self.active = False
        self.released = False
        self.phase = FormationPhase.IDLE
        self.team_ids = [self.uav_id]
        self.leader_id = self.uav_id
        self.target_center = None
        self.entry_center = None
        self.virtual_center = None
        self.approach_dir = np.array([1.0, 0.0], dtype=float)
        self.nominal_altitude = 100.0
        self.cruise_speed = 20.0
        self.last_tick_time = None
        self.last_local_position = None
        self.last_local_update_time = None
        self.formation_started_at = None
        self.hold_started_at = None
        self.common_target_time = 0.0
        self.rally_point = None
        self.rally_loiter_radius = 300.0
        self.rally_started_at = None
        self.rally_shape_locked = False
        self.rally_ready_since = None
        self.delay_pattern_until = 0.0
        self.delay_pattern_slot = -1
        self.delay_pattern_shape = None
        self.delay_pattern_phase = 0.0
        self.trail_hold_anchor = None
        self.trail_to_vee_transition_active = False
        self.trail_to_vee_transition_completed = False
        self.transition_started_at = 0.0
        self.transition_duration = 3.5
        self.transition_center_id = None
        self.transition_front_id = None
        self.transition_rear_id = None
        self.transition_left_wing_id = None
        self.transition_right_wing_id = None
        self.transition_forward_unit = np.array([1.0, 0.0], dtype=float)
        self.transition_center_anchor_xy = None
        self.transition_start_offsets = {}
        self.transition_entry_distance = None
        self.frozen_slot_order = None
        self.frozen_slot_shape = None
        self.trail_histories = {}
        self.trail_history_max_points = 2000
        self.trail_history_min_dist = 2.0
        self.trail_history_max_age = 600.0

    @staticmethod
    def _shape_from_value(value) -> FormationShape:
        try:
            raw = int(value)
        except Exception:
            return FormationShape.VEE
        if raw == 0:
            return FormationShape.VEE
        try:
            return FormationShape(raw)
        except Exception:
            return FormationShape.VEE

    def _transit_shape(self) -> FormationShape:
        return self.config.shape

    def apply_swarm_command(self, params: dict):
        if not params:
            return
        self.config.enabled = bool(params.get("enable", self.config.enabled))
        self.config.shape = self._shape_from_value(
            params.get("shape", self.config.shape.value)
        )
        self.config.spacing = float(params.get("spacing", self.config.spacing))
        self.config.standoff_distance = float(
            params.get("standoff_distance", self.config.standoff_distance)
        )
        self.config.safe_separation = float(
            params.get("safe_separation", self.config.safe_separation)
        )
        self.config.altitude_step = float(
            params.get("altitude_step", self.config.altitude_step)
        )
        self.config.leader_id = int(params.get("leader_id", self.config.leader_id))
        desired_time = float(
            params.get("desired_target_time", self.config.desired_target_time)
        )
        self.config.desired_target_time = desired_time if desired_time > 0.0 else 0.0
        if self.config.desired_target_time > 0.0:
            self.common_target_time = self.config.desired_target_time

    def start_rally(
        self,
        point: List[float],
        loiter_radius: float,
        current_position: List[float],
        cruise_speed: float,
        nominal_altitude: float,
        team_ids: Optional[Iterable[int]] = None,
        now: Optional[float] = None,
    ) -> bool:
        if point is None or len(point) < 2:
            self.active = False
            return False

        rally_alt = (
            float(point[2])
            if len(point) >= 3 and float(point[2]) > 1.0
            else max(float(nominal_altitude), float(current_position[2]), 80.0)
        )
        ids = set(int(uid) for uid in (team_ids or []) if uid is not None)
        ids.add(self.uav_id)

        self.team_ids = sorted(ids)
        self.leader_id = self._resolve_leader(self.team_ids)
        self.rally_point = np.array([float(point[0]), float(point[1]), rally_alt], dtype=float)
        self.rally_loiter_radius = max(float(loiter_radius), self.config.spacing)
        current_xy = _as_xy(current_position)
        self.last_local_position = np.asarray(current_position, dtype=float)
        self.last_local_update_time = now
        self.approach_dir = _unit(self.rally_point[:2] - current_xy, self.approach_dir)
        slot_map = self._slot_map(self.team_ids)
        slot_offset = slot_map.get(self.uav_id, np.zeros(3, dtype=float))
        self.virtual_center = current_xy - _rotation_from(self.approach_dir).dot(
            slot_offset[:2]
        )
        self.target_center = self.rally_point[:2].copy()
        self.entry_center = self.rally_point[:2].copy()
        self.nominal_altitude = rally_alt
        self.cruise_speed = max(float(cruise_speed), 12.0)
        self.active = True
        self.released = False
        self.phase = FormationPhase.ASSEMBLE
        self.last_tick_time = now
        self.hold_started_at = None
        self.formation_started_at = now if now is not None else 0.0
        self.rally_started_at = now if now is not None else 0.0
        self.rally_shape_locked = False
        self.rally_ready_since = None
        self.delay_pattern_until = 0.0
        self.delay_pattern_slot = -1
        self.delay_pattern_shape = None
        self.delay_pattern_phase = 0.0
        self.trail_hold_anchor = current_xy.copy()
        self.trail_to_vee_transition_active = False
        self.trail_to_vee_transition_completed = False
        self.transition_started_at = 0.0
        self.transition_duration = 3.5
        self.transition_center_id = None
        self.transition_front_id = None
        self.transition_rear_id = None
        self.transition_left_wing_id = None
        self.transition_right_wing_id = None
        self.transition_forward_unit = self.approach_dir.copy()
        self.transition_center_anchor_xy = None
        self.transition_start_offsets = {}
        self.transition_entry_distance = None
        self.frozen_slot_order = None
        self.frozen_slot_shape = None
        return True

    def _reference_center_estimate(
        self,
        local_position: np.ndarray,
        slot_map: Dict[int, np.ndarray],
        now: float,
        direction: np.ndarray,
    ) -> Optional[np.ndarray]:
        reference_id = self._reference_id(self.team_ids)
        rot = _rotation_from(direction)
        reference_slot = slot_map.get(reference_id, np.zeros(3, dtype=float))
        if self.uav_id == reference_id:
            return local_position[:2] - rot.dot(reference_slot[:2])

        reference_state = self.remote_states.get(reference_id)
        if reference_state is None:
            return None
        predicted = self._predict_remote_position(reference_state, now)
        return predicted[:2] - rot.dot(reference_slot[:2])

    def _estimate_center(
        self,
        local_position: np.ndarray,
        slot_map: Dict[int, np.ndarray],
        now: float,
        direction: np.ndarray,
    ) -> np.ndarray:
        rot = _rotation_from(direction)
        centers = []
        local_slot = slot_map.get(self.uav_id, np.zeros(3, dtype=float))
        centers.append(local_position[:2] - rot.dot(local_slot[:2]))

        for uid, state in self.remote_states.items():
            if uid not in slot_map:
                continue
            predicted = self._predict_remote_position(state, now)
            centers.append(predicted[:2] - rot.dot(slot_map[uid][:2]))

        if not centers:
            return local_position[:2].copy()
        return np.mean(np.asarray(centers, dtype=float), axis=0)

    def _rally_team_status(
        self,
        local_position: np.ndarray,
        slot_map: Dict[int, np.ndarray],
        center_xy: np.ndarray,
        now: float,
        direction: np.ndarray,
        ignore_reference: bool = False,
    ) -> dict:
        reference_id = self._reference_id(self.team_ids)
        rot = _rotation_from(direction)
        errors = []
        observed = 0

        if not (ignore_reference and self.uav_id == reference_id):
            local_slot = slot_map.get(self.uav_id, np.zeros(3, dtype=float))
            local_expected = center_xy + rot.dot(local_slot[:2])
            errors.append(float(np.linalg.norm(local_position[:2] - local_expected)))
            observed += 1

        for uid in self.team_ids:
            if uid == self.uav_id or (ignore_reference and uid == reference_id):
                continue
            state = self.remote_states.get(uid)
            if state is None:
                continue
            observed += 1
            predicted = self._predict_remote_position(state, now)
            expected = center_xy + rot.dot(slot_map.get(uid, np.zeros(3, dtype=float))[:2])
            errors.append(float(np.linalg.norm(predicted[:2] - expected)))

        return {
            "observed": observed,
            "expected": max(len(self.team_ids) - (1 if ignore_reference else 0), 0),
            "max_error": max(errors) if errors else 0.0,
        }

    def step_rally(
        self,
        local_position: List[float],
        local_yaw: float,
        now: float,
        team_ids: Optional[Iterable[int]] = None,
    ) -> dict:
        if not self.active or self.rally_point is None:
            return {
                "active": False,
                "holding": False,
                "arrived": False,
                "phase": FormationPhase.IDLE.value,
                "slot_id": 0,
            }

        local_pos = np.asarray(local_position, dtype=float)
        self._record_trail_sample(self.uav_id, local_pos, now)
        self.last_local_position = local_pos.copy()
        self.last_local_update_time = now
        dt = (
            0.1
            if self.last_tick_time is None
            else max(0.02, min(now - self.last_tick_time, 1.0))
        )
        self.last_tick_time = now

        center_guess = (
            self.virtual_center.copy()
            if isinstance(self.virtual_center, np.ndarray)
            and self.virtual_center.shape == (2,)
            else local_pos[:2].copy()
        )
        self.approach_dir = _unit(self.rally_point[:2] - center_guess, self.approach_dir)
        self.team_ids = self._ordered_team(team_ids)
        shape = self._transit_shape()
        self._maybe_freeze_shape_slot_order(local_pos, now, self.approach_dir)
        slot_map = self._slot_map(self.team_ids)
        slot_id = self.get_slot_id(self.uav_id)
        reference_id = self._reference_id(self.team_ids)
        is_reference = self.uav_id == reference_id
        slot_offset = slot_map.get(self.uav_id, np.zeros(3, dtype=float))
        reference_center = self._reference_center_estimate(
            local_pos, slot_map, now, self.approach_dir
        )
        center_est = (
            reference_center
            if reference_center is not None
            else self._estimate_center(local_pos, slot_map, now, self.approach_dir)
        )
        remaining_to_rally = float(np.linalg.norm(self.rally_point[:2] - center_est))
        arrival_gate = max(self.config.spacing * 0.45, self.rally_loiter_radius * 0.22)
        team_status = self._rally_team_status(
            local_pos, slot_map, center_est, now, self.approach_dir, ignore_reference=True
        )
        ready_gate = max(24.0, 0.16 * self.config.spacing)
        team_ready = (
            team_status["observed"] >= team_status["expected"]
            and team_status["max_error"] <= ready_gate
        )
        if team_ready:
            if self.rally_ready_since is None:
                self.rally_ready_since = now
        else:
            self.rally_ready_since = None

        if (
            not self.rally_shape_locked
            and self.rally_ready_since is not None
            and (now - self.rally_ready_since) >= 1.5
        ):
            self.rally_shape_locked = True

        loose_gate = max(ready_gate * 3.0, 0.9 * self.config.spacing)
        observed_ratio = (
            min(team_status["observed"] / team_status["expected"], 1.0)
            if team_status["expected"] > 0
            else 1.0
        )
        error_score = 1.0 - min(team_status["max_error"], loose_gate) / loose_gate
        transit_readiness = float(
            np.clip(0.5 * observed_ratio + 0.5 * error_score, 0.0, 1.0)
        )
        shape_deploy_strength = 0.0
        if shape in (
            FormationShape.ARROW,
            FormationShape.INVERTED_VEE,
        ) and not self.rally_shape_locked:
            shape_deploy_strength = self._shape_deploy_strength(
                now,
                remaining_to_rally,
            )

        if self.trail_to_vee_transition_active:
            transition_cmd = self._step_trail_to_vee_centered(
                local_pos,
                now,
                remaining_to_rally,
                slot_id,
            )
            if transition_cmd is not None:
                return transition_cmd

        if shape == FormationShape.TRAIL and remaining_to_rally > arrival_gate:
            self.phase = FormationPhase.ASSEMBLE
            advance_dir = _unit(self.rally_point[:2] - center_est, self.approach_dir)
            self.approach_dir = advance_dir
            trail_spacing = max(self.config.spacing, self.config.safe_separation)
            self.virtual_center = center_est.copy()
            command_xy = local_pos[:2].copy()

            if is_reference:
                lead_step = min(
                    remaining_to_rally,
                    max(
                        self.cruise_speed * max(self.config.lookahead_time * 0.42, 1.2),
                        0.42 * trail_spacing,
                    ),
                )
                command_xy = local_pos[:2] + advance_dir * lead_step
                self.virtual_center = local_pos[:2].copy()
                trail_follow_mode = "leader_direct"
                slot_distance = 0.0
            else:
                trail_spacing = max(
                    float(self.config.spacing),
                    float(self.config.safe_separation),
                )
                slot_distance = trail_spacing * float(slot_id)

                leader_target = self._sample_history_distance_back(
                    reference_id,
                    slot_distance,
                )

                if leader_target is None:
                    command_xy = local_pos[:2] + advance_dir * max(
                        self.cruise_speed * 1.0,
                        0.25 * trail_spacing,
                    )
                    trail_follow_mode = "leader_history_wait"
                else:
                    command_xy = np.asarray(leader_target[:2], dtype=float)
                    trail_follow_mode = "leader_history_direct"

                self.virtual_center = command_xy.copy()

            yaw_target = atan2(
                command_xy[1] - local_pos[1], command_xy[0] - local_pos[0]
            )
            return {
                "active": True,
                "holding": False,
                "arrived": False,
                "phase": self.phase.value,
                "slot_id": slot_id,
                "waypoint": [
                    command_xy[0],
                    command_xy[1],
                    max(self.nominal_altitude + slot_offset[2], 80.0),
                ],
                "yaw": yaw_target,
                "entry_distance": remaining_to_rally,
                "assembling_team": True,
                "trail_follow_mode": trail_follow_mode,
                "trail_reference_id": reference_id,
                "trail_slot_distance": slot_distance,
            }

        if remaining_to_rally > arrival_gate:
            self.phase = FormationPhase.ASSEMBLE
            advance_dir = _unit(self.rally_point[:2] - center_est, self.approach_dir)
            self.approach_dir = advance_dir
            guidance_dir = advance_dir.copy()
            center_motion_dir = advance_dir.copy()
            center_lookahead = min(
                remaining_to_rally,
                max(
                    self.cruise_speed * max(self.config.lookahead_time * 0.5, 2.0),
                    0.75 * self.config.spacing,
                    self.cruise_speed * dt * 8.0,
                ),
            )
            center_progress = (
                1.0 if self.rally_shape_locked else (0.45 + 0.55 * transit_readiness)
            )
            if shape == FormationShape.INVERTED_VEE and shape_deploy_strength > 1e-3:
                center_progress *= (1.0 - 0.48 * shape_deploy_strength)
            self.virtual_center = (
                center_est + center_motion_dir * center_lookahead * center_progress
            )
            rot = _rotation_from(guidance_dir)
            guided_slot_map = self._assemble_guidance_slot_map(
                shape,
                slot_map,
                now,
                guidance_dir,
                locked=self.rally_shape_locked,
                deploy_strength=shape_deploy_strength,
                remaining_distance=remaining_to_rally,
            )
            guided_slot_offset = guided_slot_map.get(self.uav_id, slot_offset)
            slot_target = np.array(
                [
                    self.virtual_center[0] + rot.dot(guided_slot_offset[:2])[0],
                    self.virtual_center[1] + rot.dot(guided_slot_offset[:2])[1],
                    max(self.nominal_altitude + guided_slot_offset[2], 80.0),
                ],
                dtype=float,
            )
            slot_error = slot_target[:2] - local_pos[:2]
            along_error, cross_error, forward, lateral = self._track_components(
                slot_error, guidance_dir
            )
            echelon_deploy_progress = (
                self._echelon_deploy_progress(now, remaining_to_rally)
                if shape in (
                    FormationShape.ECHELON_LEFT,
                    FormationShape.ECHELON_RIGHT,
                )
                and not self.rally_shape_locked
                else 1.0
            )

            lateral_floor = 0.45
            if shape == FormationShape.TRAIL:
                lateral_floor = 1.05
            elif shape in (
                FormationShape.ECHELON_LEFT,
                FormationShape.ECHELON_RIGHT,
            ):
                lateral_floor = 1.02 - 0.18 * echelon_deploy_progress
            elif shape in (
                FormationShape.VEE,
                FormationShape.WEDGE_WIDE,
                FormationShape.TRIANGLE,
                FormationShape.ARROW,
                FormationShape.INVERTED_VEE,
            ):
                lateral_floor = 0.60

            locked_lateral_weight = 0.38
            if shape == FormationShape.WEDGE_WIDE:
                locked_lateral_weight = 0.44
            elif shape == FormationShape.TRAIL:
                locked_lateral_weight = 0.55
            lateral_weight = (
                locked_lateral_weight
                if self.rally_shape_locked
                else max(
                    lateral_floor,
                    0.25 + 0.75 * transit_readiness,
                )
            )
            if not self.rally_shape_locked and is_reference:
                ref_cap = 0.18 if shape in (
                    FormationShape.TRAIL,
                    FormationShape.ECHELON_LEFT,
                    FormationShape.ECHELON_RIGHT,
                ) else 0.28
                lateral_weight = min(lateral_weight, ref_cap)

            if self.rally_shape_locked:
                along_correction = float(
                    np.clip(
                        along_error,
                        -0.18 * self.config.spacing,
                        0.32 * self.config.spacing,
                    )
                )
            elif along_error >= 0.0:
                gain = 1.05 if is_reference else (1.10 + 0.30 * (1.0 - transit_readiness))
                along_correction = along_error * gain
            else:
                if (
                    shape == FormationShape.INVERTED_VEE
                    and is_reference
                    and shape_deploy_strength > 1e-3
                ):
                    reverse_limit = 1.48 * self.config.spacing * shape_deploy_strength
                    along_correction = max(along_error * 0.88, -reverse_limit)
                elif (
                    shape in (
                        FormationShape.ECHELON_LEFT,
                        FormationShape.ECHELON_RIGHT,
                    )
                    and not is_reference
                ):
                    reverse_limit = (
                        0.02 + 0.26 * echelon_deploy_progress
                    ) * self.config.spacing
                    along_correction = max(
                        along_error * (0.05 + 0.40 * echelon_deploy_progress),
                        -reverse_limit,
                    )
                else:
                    reverse_limit = (
                        0.12 * self.config.spacing
                        if is_reference
                        else (
                            0.18 * self.config.spacing
                            if shape == FormationShape.TRAIL
                            else 0.35 * self.config.spacing
                        )
                    )
                    along_correction = max(along_error * 0.25, -reverse_limit)

            cross_correction = cross_error * lateral_weight
            if self.rally_shape_locked:
                cross_correction = float(
                    np.clip(
                        cross_correction,
                        -0.28 * self.config.spacing,
                        0.28 * self.config.spacing,
                    )
                )
            slot_correction = forward * along_correction + lateral * cross_correction

            consensus = np.zeros(2, dtype=float)
            count = 0
            for uid, state in self.remote_states.items():
                if uid not in slot_map:
                    continue
                predicted = self._predict_remote_position(state, now)
                peer_slot = guided_slot_map.get(uid, slot_map[uid])
                expected_rel = rot.dot(peer_slot[:2] - guided_slot_offset[:2])
                consensus += (predicted[:2] - local_pos[:2]) - expected_rel
                count += 1
            if count > 0:
                consensus /= count

            inter_uav = self._inter_uav_repulsion(local_pos, now, slot_map)
            consensus_gain = self.config.consensus_gain
            consensus_correction = consensus
            if self.rally_shape_locked:
                consensus_gain = 0.0
                consensus_correction = np.zeros(2, dtype=float)
                inter_uav *= 0.55
            elif is_reference:
                consensus_gain = 0.0
                consensus_correction = np.zeros(2, dtype=float)
                inter_uav *= 0.55
            else:
                consensus_gain *= 0.18 + 0.55 * transit_readiness
                if shape in (
                    FormationShape.TRAIL,
                    FormationShape.ECHELON_LEFT,
                    FormationShape.ECHELON_RIGHT,
                ):
                    consensus_gain *= 0.55
                inter_uav *= 0.8
                consensus_correction = self._track_weighted_vector(
                    consensus, guidance_dir, lateral_weight
                )
            inverted_axis_correction = np.zeros(2, dtype=float)
            inverted_breakout_correction = np.zeros(2, dtype=float)
            inverted_front_promote_correction = np.zeros(2, dtype=float)
            if (
                shape == FormationShape.INVERTED_VEE
                and not self.rally_shape_locked
                and is_reference
            ):
                inverted_axis_correction = self._inverted_vee_axis_correction(
                    local_pos,
                    now,
                    advance_dir,
                    slot_map,
                )
                inverted_breakout_correction = self._inverted_vee_breakout_correction(
                    local_pos,
                    local_yaw,
                    advance_dir,
                    shape_deploy_strength,
                )
            elif shape == FormationShape.INVERTED_VEE and not self.rally_shape_locked:
                inverted_front_promote_correction = (
                    self._inverted_vee_front_promote_correction(
                        local_pos,
                        now,
                        advance_dir,
                        slot_map,
                    )
                )
            correction = (
                self.config.slot_gain * slot_correction
                + consensus_gain * consensus_correction
                + inter_uav
                + inverted_axis_correction
                + inverted_breakout_correction
                + inverted_front_promote_correction
            )
            correction = _clip_norm(
                correction,
                max(
                    self.cruise_speed
                    * max(
                        self.config.lookahead_time
                        * (0.7 if self.rally_shape_locked else 1.0),
                        1.6 if self.rally_shape_locked else 2.0,
                    ),
                    0.65 * self.config.spacing
                    if self.rally_shape_locked
                    else 1.25 * self.config.spacing,
                ),
            )
            command_xy = local_pos[:2] + correction
            using_delay_pattern = False
            if not self.rally_shape_locked:
                delay_pattern_allowed = shape in (
                    FormationShape.VEE,
                    FormationShape.WEDGE_WIDE,
                    FormationShape.TRIANGLE,
                    FormationShape.ARROW,
                    FormationShape.INVERTED_VEE,
                )
                ahead_margin = max(0.35 * self.config.spacing, 35.0)
                delay_active = (
                    self.delay_pattern_slot == slot_id
                    and self.delay_pattern_shape == shape.value
                    and now < self.delay_pattern_until
                )
                if (
                    delay_active
                    and (
                        not delay_pattern_allowed
                        or is_reference
                        or along_error >= -0.08 * self.config.spacing
                        or abs(cross_error) > 0.50 * self.config.spacing
                        or remaining_to_rally <= max(
                            self.rally_loiter_radius * 0.8,
                            2.0 * self.config.spacing,
                        )
                    )
                ):
                    delay_active = False
                    self.delay_pattern_until = 0.0
                    self.delay_pattern_slot = -1
                    self.delay_pattern_shape = None

                if (
                    not delay_active
                    and delay_pattern_allowed
                    and not is_reference
                    and along_error < -ahead_margin
                    and abs(cross_error) <= 0.32 * self.config.spacing
                    and remaining_to_rally > max(
                        self.rally_loiter_radius * 0.8,
                        2.0 * self.config.spacing,
                    )
                ):
                    self.delay_pattern_until = now + float(
                        np.clip(
                            (-along_error / max(self.cruise_speed, 1.0)) * 1.35,
                            3.0,
                            6.0,
                        )
                    )
                    self.delay_pattern_slot = slot_id
                    self.delay_pattern_shape = shape.value
                    self.delay_pattern_phase = 0.65 * np.pi * slot_id
                    delay_active = True

                if delay_active:
                    using_delay_pattern = True
                    figure8_center = slot_target[:2] - forward * min(
                        -along_error * 0.45,
                        0.55 * self.config.spacing,
                    )
                    figure8_xy = figure8_center + self._figure8_offset(
                        now,
                        guidance_dir,
                        max(0.24 * self.config.spacing, 35.0),
                        phase=self.delay_pattern_phase,
                    )
                    command_xy = local_pos[:2] + _clip_norm(
                        (figure8_xy - local_pos[:2]) + 0.45 * inter_uav,
                        max(
                            self.cruise_speed * max(self.config.lookahead_time * 1.2, 2.2),
                            1.1 * self.config.spacing,
                        ),
                    )

                if not using_delay_pattern:
                    # Keep the reference aircraft on a straight outbound line, while followers
                    # close lateral errors without sacrificing forward progress.
                    follower_forward_floor = 0.40 * self.config.spacing
                    if shape == FormationShape.TRAIL:
                        follower_forward_floor = (
                            0.0
                            if abs(cross_error) > 0.08 * self.config.spacing
                            else 0.10 * self.config.spacing
                        )
                    elif shape in (
                        FormationShape.ECHELON_LEFT,
                        FormationShape.ECHELON_RIGHT,
                    ):
                        if echelon_deploy_progress < 0.35:
                            follower_forward_floor = 0.0
                        elif echelon_deploy_progress < 0.72:
                            follower_forward_floor = 0.08 * self.config.spacing
                        else:
                            follower_forward_floor = (
                                0.18 * self.config.spacing
                                if abs(cross_error) > 0.14 * self.config.spacing
                                else 0.26 * self.config.spacing
                            )
                    inverted_axis_active = (
                        shape == FormationShape.INVERTED_VEE
                        and is_reference
                        and float(np.linalg.norm(inverted_axis_correction)) > 1e-3
                    )
                    inverted_breakout_active = (
                        shape == FormationShape.INVERTED_VEE
                        and is_reference
                        and float(np.linalg.norm(inverted_breakout_correction)) > 1e-3
                    )
                    if (
                        shape == FormationShape.INVERTED_VEE
                        and shape_deploy_strength > 1e-3
                    ):
                        if is_reference:
                            min_forward_progress = -1.42 * self.config.spacing * shape_deploy_strength
                        else:
                            min_forward_progress = min(
                                remaining_to_rally,
                                max(
                                    self.cruise_speed * max(self.config.lookahead_time * 0.22, 0.6),
                                    follower_forward_floor,
                                ),
                            )
                    elif (
                        shape == FormationShape.INVERTED_VEE
                        and is_reference
                        and inverted_axis_active
                    ):
                        min_forward_progress = -0.45 * self.config.spacing
                    elif (
                        shape == FormationShape.INVERTED_VEE
                        and is_reference
                        and inverted_breakout_active
                    ):
                        min_forward_progress = -0.72 * self.config.spacing
                    else:
                        min_forward_progress = min(
                            remaining_to_rally,
                            max(
                                self.cruise_speed * max(self.config.lookahead_time * 0.4, 1.2),
                                0.55 * self.config.spacing if is_reference else follower_forward_floor,
                            ),
                        )
                    if min_forward_progress is not None:
                        forward_progress = float(np.dot(command_xy - local_pos[:2], guidance_dir))
                        if forward_progress < min_forward_progress:
                            command_xy = command_xy + guidance_dir * (
                                min_forward_progress - forward_progress
                            )
                        command_xy = local_pos[:2] + _clip_norm(
                            command_xy - local_pos[:2],
                            max(
                                self.cruise_speed * max(self.config.lookahead_time * 1.4, 3.0),
                                1.35 * self.config.spacing if is_reference else 1.6 * self.config.spacing,
                            ),
                        )
            yaw_target = atan2(
                command_xy[1] - local_pos[1], command_xy[0] - local_pos[0]
            )
            return {
                "active": True,
                "holding": False,
                "arrived": False,
                "phase": self.phase.value,
                "slot_id": slot_id,
                "waypoint": [command_xy[0], command_xy[1], slot_target[2]],
                "yaw": yaw_target,
                "entry_distance": remaining_to_rally,
                "assembling_team": not self.rally_shape_locked,
            }

        self.phase = FormationPhase.HOLD
        if self.hold_started_at is None:
            self.hold_started_at = now
        self.rally_shape_locked = True
        self.delay_pattern_until = 0.0
        self.delay_pattern_slot = -1
        self.delay_pattern_shape = None

        self.virtual_center = self.rally_point[:2].copy()
        slot_target = self.rally_point.copy()
        slot_target[2] = max(self.nominal_altitude + slot_offset[2], 80.0)
        team_size = max(len(self.team_ids), 1)
        orbit_point = self._hold_waypoint(
            slot_id,
            team_size,
            slot_offset,
            slot_target,
            now,
        )
        yaw_target = atan2(
            orbit_point[1] - local_pos[1], orbit_point[0] - local_pos[0]
        )
        return {
            "active": True,
            "holding": True,
            "arrived": True,
            "phase": self.phase.value,
            "slot_id": slot_id,
            "waypoint": orbit_point.tolist(),
            "yaw": yaw_target,
            "loiter_radius": float(
                np.linalg.norm(orbit_point[:2] - self.rally_point[:2])
            ),
        }

    def start_trail_to_vee_centered(
        self,
        local_position: List[float],
        now: float,
        team_ids: Optional[Iterable[int]] = None,
        entry_distance: Optional[float] = None,
    ) -> Optional[dict]:
        if (
            self.trail_to_vee_transition_active
            or self.trail_to_vee_transition_completed
            or self.rally_point is None
        ):
            return None
        if self._transit_shape() != FormationShape.TRAIL:
            return None

        local_pos = np.asarray(local_position, dtype=float)
        track_dir = _unit(self.approach_dir, self.rally_point[:2] - local_pos[:2])
        return self._lock_trail_to_vee_roles(
            local_pos,
            now,
            team_ids,
            track_dir,
            entry_distance=entry_distance,
        )

    def _lock_trail_to_vee_roles(
        self,
        local_position: np.ndarray,
        now: float,
        team_ids: Optional[Iterable[int]],
        track_dir: np.ndarray,
        entry_distance: Optional[float] = None,
    ) -> Optional[dict]:
        ordered = self._ordered_team(team_ids)
        positions = {self.uav_id: local_position[:2].copy()}
        for uid in ordered:
            if uid == self.uav_id:
                continue
            state = self.remote_states.get(uid)
            if state is None:
                continue
            positions[uid] = self._predict_remote_position(state, now)[:2]

        if len(positions) < 3:
            return None

        team3 = sorted(positions.keys())
        forward = _unit(track_dir, self.approach_dir)
        lateral = np.array([-forward[1], forward[0]], dtype=float)
        projected = sorted(
            ((float(np.dot(positions[uid], forward)), uid) for uid in team3),
            reverse=True,
        )
        front_id = projected[0][1]
        center_id = projected[1][1]
        rear_id = projected[2][1]
        center_xy = positions[center_id].copy()

        front_lat = float(np.dot(positions[front_id] - center_xy, lateral))
        rear_lat = float(np.dot(positions[rear_id] - center_xy, lateral))
        line_eps = max(0.08 * self.config.spacing, 5.0)
        if abs(front_lat) <= line_eps and abs(rear_lat) <= line_eps:
            left_wing_id = rear_id
            right_wing_id = front_id
        elif front_lat >= rear_lat:
            left_wing_id = front_id
            right_wing_id = rear_id
        else:
            left_wing_id = rear_id
            right_wing_id = front_id

        self.trail_to_vee_transition_active = True
        self.trail_to_vee_transition_completed = False
        self.transition_started_at = now
        self.transition_duration = 3.5
        self.transition_center_id = center_id
        self.transition_front_id = front_id
        self.transition_rear_id = rear_id
        self.transition_left_wing_id = left_wing_id
        self.transition_right_wing_id = right_wing_id
        self.transition_forward_unit = forward.copy()
        self.transition_center_anchor_xy = center_xy.copy()
        self.transition_entry_distance = entry_distance
        self.transition_start_offsets = {}
        for uid, pos_xy in positions.items():
            rel = pos_xy - center_xy
            self.transition_start_offsets[uid] = np.array(
                [float(np.dot(rel, forward)), float(np.dot(rel, lateral))],
                dtype=float,
            )
        self.delay_pattern_until = 0.0
        self.delay_pattern_slot = -1
        self.delay_pattern_shape = None
        self.rally_shape_locked = False

        return self._transition_info("start", 0.0, entry_distance, center_xy)

    def _transition_info(
        self,
        event: str,
        progress: float,
        entry_distance: Optional[float],
        center_anchor_xy: np.ndarray,
    ) -> dict:
        return {
            "transition_event": event,
            "center_id": self.transition_center_id,
            "front_id": self.transition_front_id,
            "rear_id": self.transition_rear_id,
            "left_wing_id": self.transition_left_wing_id,
            "right_wing_id": self.transition_right_wing_id,
            "entry_distance": entry_distance,
            "progress": float(progress),
            "forward_unit": self.transition_forward_unit.copy(),
            "center_anchor_xy": center_anchor_xy.copy(),
        }

    @staticmethod
    def _transition_smoothstep(progress: float) -> float:
        p = float(np.clip(progress, 0.0, 1.0))
        return p * p * (3.0 - 2.0 * p)

    def _transition_final_offset(self, uid: int) -> np.ndarray:
        spacing = float(self.config.spacing)
        if uid == self.transition_center_id:
            return np.array([0.0, 0.0], dtype=float)
        if uid == self.transition_left_wing_id:
            return np.array([-0.9 * spacing, 0.75 * spacing], dtype=float)
        if uid == self.transition_right_wing_id:
            return np.array([-0.9 * spacing, -0.75 * spacing], dtype=float)
        return np.zeros(2, dtype=float)

    def _step_trail_to_vee_centered(
        self,
        local_pos: np.ndarray,
        now: float,
        entry_distance: float,
        slot_id: int,
    ) -> Optional[dict]:
        if not self.trail_to_vee_transition_active:
            return None

        forward = _unit(self.transition_forward_unit, self.approach_dir)
        lateral = np.array([-forward[1], forward[0]], dtype=float)
        elapsed = max(0.0, now - float(self.transition_started_at))
        raw_progress = elapsed / max(float(self.transition_duration), 1e-3)
        progress = float(np.clip(raw_progress, 0.0, 1.0))
        smooth = self._transition_smoothstep(progress)
        moving_center = (
            self.transition_center_anchor_xy
            + forward * (self.cruise_speed * elapsed * 0.82)
        )
        spacing = float(self.config.spacing)
        uid = self.uav_id

        if uid == self.transition_center_id:
            lead_step = min(max(0.35 * spacing, 20.0), 40.0)
            command_xy = moving_center + forward * lead_step
            center_error = command_xy - local_pos[:2]
            along_error, cross_error, _, _ = self._track_components(center_error, forward)
            along_cmd = max(along_error, 12.0)
            cross_cmd = float(np.clip(cross_error, -0.18 * spacing, 0.18 * spacing))
            command_xy = local_pos[:2] + forward * along_cmd + lateral * cross_cmd
        else:
            start_offset = self.transition_start_offsets.get(
                uid, self._transition_final_offset(uid)
            )
            final_offset = self._transition_final_offset(uid)
            desired_offset = (1.0 - smooth) * start_offset + smooth * final_offset
            desired_xy = (
                moving_center
                + forward * desired_offset[0]
                + lateral * desired_offset[1]
            )
            error_xy = desired_xy - local_pos[:2]
            along_error, cross_error, _, _ = self._track_components(error_xy, forward)

            if uid == self.transition_front_id:
                min_forward = max(4.0, 0.06 * spacing)
                max_forward = max(20.0, 0.32 * spacing)
                lateral_gain = 0.68
            else:
                min_forward = max(8.0, 0.12 * spacing)
                max_forward = max(28.0, 0.48 * spacing)
                lateral_gain = 0.78

            along_cmd = float(np.clip(along_error, min_forward, max_forward))
            cross_cmd = float(
                np.clip(cross_error * lateral_gain, -0.62 * spacing, 0.62 * spacing)
            )
            command_xy = local_pos[:2] + forward * along_cmd + lateral * cross_cmd
            command_xy = local_pos[:2] + _clip_norm(
                command_xy - local_pos[:2],
                max(self.cruise_speed * 1.6, 0.75 * spacing),
            )

        slot_z = 0.0
        try:
            slot_z = self._slot_map(self.team_ids).get(uid, np.zeros(3, dtype=float))[2]
        except Exception:
            slot_z = 0.0

        complete = raw_progress >= 1.0
        event = "complete" if complete else "progress"
        if complete:
            self.trail_to_vee_transition_active = False
            self.trail_to_vee_transition_completed = True
            self.config.shape = FormationShape.VEE
            self.rally_shape_locked = True
            self.frozen_slot_order = [
                self.transition_center_id,
                self.transition_left_wing_id,
                self.transition_right_wing_id,
            ]
            self.frozen_slot_shape = FormationShape.VEE

        transition_order = [
            self.transition_center_id,
            self.transition_left_wing_id,
            self.transition_right_wing_id,
        ]
        transition_slot_id = (
            transition_order.index(uid)
            if uid in transition_order
            else slot_id
        )

        yaw_target = atan2(
            command_xy[1] - local_pos[1], command_xy[0] - local_pos[0]
        )
        return {
            "active": True,
            "holding": False,
            "arrived": False,
            "phase": self.phase.value,
            "slot_id": transition_slot_id,
            "waypoint": [
                command_xy[0],
                command_xy[1],
                max(self.nominal_altitude + slot_z, 80.0),
            ],
            "yaw": yaw_target,
            "entry_distance": entry_distance,
            "assembling_team": True,
            "transition": self._transition_info(
                event,
                progress,
                entry_distance,
                moving_center,
            ),
        }

    def _hold_waypoint(
        self,
        slot_id: int,
        team_size: int,
        slot_offset: np.ndarray,
        slot_target: np.ndarray,
        now: float,
    ) -> np.ndarray:
        elapsed = now - (self.rally_started_at or now)
        center_radius = max(self.rally_loiter_radius, 0.75 * self.config.spacing)
        omega = self.cruise_speed / max(center_radius, 1.0)
        theta = omega * elapsed
        center_xy = np.array(
            [
                self.rally_point[0] + center_radius * np.cos(theta),
                self.rally_point[1] + center_radius * np.sin(theta),
            ],
            dtype=float,
        )
        tangent = np.array([-np.sin(theta), np.cos(theta)], dtype=float)
        rot = np.array(
            [[tangent[0], -tangent[1]], [tangent[1], tangent[0]]],
            dtype=float,
        )
        slot_xy = center_xy + rot.dot(slot_offset[:2])
        z = max(self.nominal_altitude + slot_offset[2], 80.0)
        return np.array([slot_xy[0], slot_xy[1], z], dtype=float)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return float((float(angle) + np.pi) % (2.0 * np.pi) - np.pi)

    def start_mission(
        self,
        targets: List[List[float]],
        base_config: Optional[List[float]],
        current_position: List[float],
        cruise_speed: float,
        nominal_altitude: float,
        team_ids: Optional[Iterable[int]] = None,
        now: Optional[float] = None,
    ) -> bool:
        if not targets or not self.config.enabled:
            self.active = False
            return False

        target_pts = np.asarray([t[:2] for t in targets], dtype=float)
        self.target_center = np.mean(target_pts, axis=0)
        current_xy = _as_xy(current_position)
        source_xy = (
            _as_xy(base_config)
            if base_config is not None and len(base_config) >= 2
            else current_xy
        )
        self.approach_dir = _unit(self.target_center - source_xy, current_xy - source_xy)
        self.entry_center = (
            self.target_center - self.approach_dir * self.config.standoff_distance
        )
        self.virtual_center = current_xy.copy()
        self.last_local_position = np.asarray(current_position, dtype=float)
        self.last_local_update_time = now
        self.nominal_altitude = max(float(nominal_altitude), float(current_position[2]), 80.0)
        self.cruise_speed = max(float(cruise_speed), 12.0)
        ids = set(int(uid) for uid in (team_ids or []) if uid is not None)
        ids.add(self.uav_id)
        self.team_ids = sorted(ids)
        self.leader_id = self._resolve_leader(self.team_ids)
        self.active = True
        self.released = False
        self.phase = FormationPhase.ASSEMBLE
        self.last_tick_time = now
        self.hold_started_at = None
        self.formation_started_at = now if now is not None else 0.0
        self.common_target_time = (
            self.config.desired_target_time if self.config.desired_target_time > 0.0 else 0.0
        )
        self.delay_pattern_until = 0.0
        self.delay_pattern_slot = -1
        self.delay_pattern_shape = None
        self.delay_pattern_phase = 0.0
        self.trail_hold_anchor = current_xy.copy()
        self.frozen_slot_order = None
        self.frozen_slot_shape = None

        if np.linalg.norm(current_xy - self.entry_center) <= self.config.hold_radius:
            self.virtual_center = self.entry_center.copy()
            self.phase = FormationPhase.HOLD
            self.hold_started_at = now
        return True

    def update_remote_state(self, info: dict):
        if not info:
            return
        peer_id = int(info.get("uav_id", 0))
        if peer_id <= 0 or peer_id == self.uav_id:
            return
        state = TeamState(
            uav_id=peer_id,
            timestamp=float(info.get("timestamp", 0.0)),
            position=np.asarray(info.get("position", [0.0, 0.0, 0.0]), dtype=float),
            velocity=np.asarray(info.get("velocity", [0.0, 0.0, 0.0]), dtype=float),
            yaw=float(info.get("yaw", 0.0)),
            phase=int(info.get("phase", FormationPhase.IDLE.value)),
            slot_id=int(info.get("slot_id", 0)),
            sync_eta=info.get("sync_eta", None),
            common_target_time=float(info.get("common_target_time", 0.0)),
        )
        self.remote_states[peer_id] = state
        self._record_trail_sample(peer_id, state.position, state.timestamp)
        if peer_id not in self.team_ids:
            self.team_ids.append(peer_id)
            self.team_ids.sort()
            self.leader_id = self._resolve_leader(self.team_ids)
        remote_common_time = float(info.get("common_target_time", 0.0))
        if remote_common_time > 0.0 and peer_id == self.leader_id:
            self.common_target_time = remote_common_time

    def _resolve_leader(self, ids: Iterable[int]) -> int:
        ids = sorted(set(int(uid) for uid in ids if uid))
        if not ids:
            return self.uav_id
        if self.config.leader_id in ids and self.config.leader_id > 0:
            return self.config.leader_id
        return ids[0]

    def _trail_order(self, ordered):
        ordered = sorted(set(int(uid) for uid in ordered if uid is not None))
        if not ordered:
            return [self.uav_id]

        leader = int(getattr(self.config, "leader_id", 0) or 0)
        if leader in ordered:
            return [leader] + [uid for uid in ordered if uid != leader]

        return ordered

    def _reference_id(self, ids: Iterable[int]) -> int:
        ordered = sorted(set(int(uid) for uid in ids if uid))
        if not ordered:
            return self.uav_id

        shape = self._transit_shape()
        if shape == FormationShape.TRAIL:
            return self._trail_order(ordered)[0]

        if self.frozen_slot_order and self.frozen_slot_shape == shape:
            for uid in self.frozen_slot_order:
                if uid in ordered:
                    return uid
        if shape in (FormationShape.ECHELON_LEFT, FormationShape.ECHELON_RIGHT):
            echelon_order = self._geometric_echelon_slot_order(shape)
            if echelon_order:
                for uid in echelon_order:
                    if uid in ordered:
                        return uid
        if shape in (FormationShape.ARROW, FormationShape.INVERTED_VEE):
            return ordered[len(ordered) // 2]
        if shape in (
            FormationShape.ECHELON_LEFT,
            FormationShape.TRAIL,
        ):
            return ordered[-1]
        if shape == FormationShape.ECHELON_RIGHT:
            return ordered[0]
        if len(ordered) == 3:
            if shape in (
                FormationShape.VEE,
                FormationShape.TRIANGLE,
                FormationShape.WEDGE_WIDE,
                FormationShape.ARROW,
                FormationShape.INVERTED_VEE,
            ):
                return ordered[1]

        if self.config.leader_id in ordered and self.config.leader_id > 0:
            return self.config.leader_id
        return ordered[0]

    def _ordered_team(self, team_ids: Optional[Iterable[int]] = None) -> List[int]:
        ids = set(self.team_ids)
        if team_ids is not None:
            ids.update(int(uid) for uid in team_ids if uid is not None)
        ids.add(self.uav_id)
        ids.update(self.remote_states.keys())
        ordered = sorted(ids)
        if not ordered:
            return [self.uav_id]
        self.leader_id = self._resolve_leader(ordered)
        return ordered

    def _record_trail_sample(self, uid: int, position, now: float):
        uid = int(uid)
        if position is None or len(position) < 2:
            return

        sample = np.array(
            [
                float(position[0]),
                float(position[1]),
                float(position[2]) if len(position) >= 3 else float(self.nominal_altitude),
            ],
            dtype=float,
        )

        hist = self.trail_histories.setdefault(uid, [])

        if hist:
            _, last_p = hist[-1]
            if float(np.linalg.norm(sample[:2] - last_p[:2])) < self.trail_history_min_dist:
                return

        hist.append((float(now), sample))

        cutoff = float(now) - float(self.trail_history_max_age)
        while hist and hist[0][0] < cutoff:
            hist.pop(0)

        if len(hist) > self.trail_history_max_points:
            del hist[:-self.trail_history_max_points]

    def _sample_history_distance_back(self, uid: int, distance_back: float):
        hist = self.trail_histories.get(int(uid), [])

        if len(hist) < 1:
            return None

        if len(hist) == 1:
            return hist[0][1].copy()

        remain = max(float(distance_back), 0.0)

        for i in range(len(hist) - 1, 0, -1):
            _, p1 = hist[i]
            _, p0 = hist[i - 1]

            seg = float(np.linalg.norm(p1[:2] - p0[:2]))
            if seg < 1e-6:
                continue

            if remain <= seg:
                ratio = remain / seg
                return p1 + (p0 - p1) * ratio

            remain -= seg

        return hist[0][1].copy()

    def _formation_target_xy(self) -> Optional[np.ndarray]:
        if isinstance(self.entry_center, np.ndarray) and self.entry_center.shape == (2,):
            return self.entry_center.copy()
        if isinstance(self.rally_point, np.ndarray) and self.rally_point.shape[0] >= 2:
            return self.rally_point[:2].copy()
        if isinstance(self.target_center, np.ndarray) and self.target_center.shape == (2,):
            return self.target_center.copy()
        return None

    def _formation_elapsed(self, now: float) -> float:
        started_at = self.formation_started_at
        if not isinstance(started_at, (int, float)):
            started_at = self.rally_started_at
        if not isinstance(started_at, (int, float)):
            return 0.0
        return max(0.0, now - float(started_at))

    def _echelon_deploy_progress(
        self,
        now: float,
        remaining_distance: Optional[float] = None,
    ) -> float:
        if len(self.team_ids) < 3:
            return 1.0

        elapsed = self._formation_elapsed(now)
        traveled = elapsed * max(self.cruise_speed, 1.0)
        deploy_distance = max(
            2.45 * self.config.spacing,
            5.0 * self.config.safe_separation,
        )
        progress = float(np.clip(traveled / max(deploy_distance, 1.0), 0.0, 1.0))

        if remaining_distance is not None:
            finish_gate = max(self.config.hold_radius * 1.15, 2.3 * self.config.spacing)
            if remaining_distance <= finish_gate:
                return 1.0
            start_gate = max(finish_gate * 2.0, 4.6 * self.config.spacing)
            closeness = float(
                np.clip(
                    (start_gate - remaining_distance)
                    / max(start_gate - finish_gate, 1e-6),
                    0.0,
                    1.0,
                )
            )
            progress = max(progress, closeness)

        return progress

    def _geometric_echelon_slot_order(
        self,
        shape: FormationShape,
        track_dir: Optional[np.ndarray] = None,
        now: Optional[float] = None,
    ) -> Optional[List[int]]:
        if shape not in (FormationShape.ECHELON_LEFT, FormationShape.ECHELON_RIGHT):
            return None
        if (
            not isinstance(self.last_local_position, np.ndarray)
            or self.last_local_position.shape[0] < 2
        ):
            return None

        ordered = self._ordered_team()
        if len(ordered) != 3:
            return None

        sample_time = (
            now
            if isinstance(now, (int, float))
            else self.last_local_update_time
        )
        forward = _unit(
            np.asarray(track_dir, dtype=float) if track_dir is not None else self.approach_dir,
            self.approach_dir,
        )
        lateral = np.array([-forward[1], forward[0]], dtype=float)
        samples = [
            (
                self.uav_id,
                float(np.dot(self.last_local_position[:2], lateral)),
                float(np.dot(self.last_local_position[:2], forward)),
                self.last_local_position[:2].copy(),
            )
        ]

        for uid in ordered:
            if uid == self.uav_id:
                continue
            state = self.remote_states.get(uid)
            if state is None:
                return None
            predicted = (
                self._predict_remote_position(state, sample_time)
                if isinstance(sample_time, (int, float))
                else state.position
            )
            samples.append(
                (
                    uid,
                    float(np.dot(predicted[:2], lateral)),
                    float(np.dot(predicted[:2], forward)),
                    predicted[:2].copy(),
                )
            )

        return self._echelon_slot_order_from_samples(samples, shape)

    def _echelon_slot_order_from_samples(
        self,
        samples: List[tuple],
        shape: FormationShape,
    ) -> Optional[List[int]]:
        if len(samples) != 3:
            return None

        if shape == FormationShape.ECHELON_LEFT:
            side_sign = 1.0
        elif shape == FormationShape.ECHELON_RIGHT:
            side_sign = -1.0
        else:
            return None

        target_xy = self._formation_target_xy()
        if target_xy is not None:
            anchor = min(
                samples,
                key=lambda item: (
                    float(np.linalg.norm(item[3] - target_xy)),
                    -item[2],
                    abs(item[1]),
                    item[0],
                ),
            )
        else:
            anchor = max(samples, key=lambda item: (item[2], -side_sign * item[1], -item[0]))

        followers = [item for item in samples if item[0] != anchor[0]]
        if len(followers) != 2:
            return None

        if target_xy is not None:
            followers.sort(
                key=lambda item: (
                    float(np.linalg.norm(item[3] - target_xy)),
                    -side_sign * item[1],
                    -item[2],
                    item[0],
                )
            )
            return [anchor[0], followers[0][0], followers[1][0]]

        diagonal_spacing = 1.05 * self.config.spacing
        desired_slots = [
            (-diagonal_spacing, side_sign * diagonal_spacing),
            (-2.0 * diagonal_spacing, side_sign * 2.0 * diagonal_spacing),
        ]

        def assignment_cost(assignment: List[tuple]) -> float:
            total = 0.0
            for follower, desired in zip(assignment, desired_slots):
                rel_along = follower[2] - anchor[2]
                rel_cross = follower[1] - anchor[1]
                desired_along, desired_cross = desired
                along_error = rel_along - desired_along
                cross_error = rel_cross - desired_cross
                rear_gap = -rel_along
                side_gap = side_sign * rel_cross
                penalty = 0.0
                if rear_gap < 0.0:
                    penalty += (1.15 * diagonal_spacing + abs(rear_gap)) ** 2
                if side_gap < 0.0:
                    penalty += (1.15 * diagonal_spacing + abs(side_gap)) ** 2
                total += (
                    along_error ** 2
                    + cross_error ** 2
                    + 0.18 * (rear_gap - side_gap) ** 2
                    + penalty
                )
            return total

        assignment_a = [followers[0], followers[1]]
        assignment_b = [followers[1], followers[0]]
        assignment = (
            assignment_a
            if assignment_cost(assignment_a) <= assignment_cost(assignment_b)
            else assignment_b
        )
        return [anchor[0], assignment[0][0], assignment[1][0]]

    def _slot_order(self, team_ids: Optional[Iterable[int]] = None) -> List[int]:
        ordered = self._ordered_team(team_ids)
        if not ordered:
            return [self.uav_id]

        if self.frozen_slot_order and self.frozen_slot_shape == self._transit_shape():
            frozen = [uid for uid in self.frozen_slot_order if uid in ordered]
            if len(frozen) == len(ordered):
                return frozen

        reference_id = self._reference_id(ordered)
        shape = self._transit_shape()

        if shape == FormationShape.TRAIL:
            return self._trail_order(ordered)

        if shape in (FormationShape.ECHELON_LEFT, FormationShape.ECHELON_RIGHT):
            echelon_order = self._geometric_echelon_slot_order(shape)
            if echelon_order:
                geometric = [uid for uid in echelon_order if uid in ordered]
                if len(geometric) == len(ordered):
                    return geometric

        if shape in (
            FormationShape.VEE,
            FormationShape.TRIANGLE,
            FormationShape.WEDGE_WIDE,
            FormationShape.ARROW,
            FormationShape.INVERTED_VEE,
        ):
            left_ids = [uid for uid in ordered if uid < reference_id]
            right_ids = [uid for uid in ordered if uid > reference_id]
            slot_order = [reference_id]
            left_ids.sort()
            right_ids.sort()
            while left_ids or right_ids:
                if left_ids:
                    slot_order.append(left_ids.pop())
                if right_ids:
                    slot_order.append(right_ids.pop(0))
            return slot_order

        if shape in (FormationShape.ECHELON_LEFT, FormationShape.TRAIL):
            return sorted(ordered, reverse=True)

        if shape == FormationShape.ECHELON_RIGHT:
            return sorted(ordered)

        return [reference_id] + [uid for uid in ordered if uid != reference_id]

    def _slot_offset(self, index: int) -> np.ndarray:
        if index == 0:
            return np.array([0.0, 0.0, 0.0], dtype=float)

        shape = self._transit_shape()
        spacing = self.config.spacing
        z_offset = 0.0
        side = 1.0 if index % 2 else -1.0

        if shape == FormationShape.ECHELON_LEFT:
            return np.array(
                [-1.05 * index * spacing, 1.05 * index * spacing, z_offset],
                dtype=float,
            )

        if shape == FormationShape.ECHELON_RIGHT:
            return np.array(
                [-1.05 * index * spacing, -1.05 * index * spacing, z_offset],
                dtype=float,
            )

        if shape == FormationShape.TRAIL:
            return np.array(
                [-0.50 * index * spacing, 0.0, z_offset],
                dtype=float,
            )

        if shape == FormationShape.TRIANGLE:
            layer = (index + 1) // 2
            return np.array(
                [-0.95 * layer * spacing, side * 0.62 * layer * spacing, z_offset],
                dtype=float,
            )

        if shape == FormationShape.WEDGE_WIDE:
            layer = (index + 1) // 2
            return np.array(
                [-0.72 * layer * spacing, side * 1.28 * layer * spacing, z_offset],
                dtype=float,
            )

        if shape == FormationShape.ARROW:
            layer = (index + 1) // 2
            return np.array(
                [
                    -1.18 * layer * spacing,
                    side * 0.46 * layer * spacing,
                    z_offset,
                ],
                dtype=float,
            )

        if shape == FormationShape.INVERTED_VEE:
            layer = (index + 1) // 2
            return np.array(
                [1.18 * layer * spacing, side * 0.72 * layer * spacing, z_offset],
                dtype=float,
            )

        leg = (index + 1) // 2
        return np.array(
            [-0.95 * leg * spacing, side * 0.82 * leg * spacing, z_offset],
            dtype=float,
        )

    def get_slot_id(self, uav_id: Optional[int] = None) -> int:
        uid = self.uav_id if uav_id is None else int(uav_id)
        ordered = self._slot_order()
        return ordered.index(uid) if uid in ordered else 0

    def _slot_map(self, team_ids: Optional[Iterable[int]] = None) -> Dict[int, np.ndarray]:
        ordered = self._slot_order(team_ids)
        return {uid: self._slot_offset(idx) for idx, uid in enumerate(ordered)}

    def _rotation(self) -> np.ndarray:
        d = _unit(self.approach_dir)
        return np.array([[d[0], -d[1]], [d[1], d[0]]], dtype=float)

    @staticmethod
    def _track_weighted_vector(
        vec: np.ndarray,
        track_dir: np.ndarray,
        lateral_weight: float = 1.0,
    ) -> np.ndarray:
        forward = _unit(track_dir)
        lateral = np.array([-forward[1], forward[0]], dtype=float)
        along = float(np.dot(vec, forward))
        cross = float(np.dot(vec, lateral))
        return forward * along + lateral * (cross * lateral_weight)

    @staticmethod
    def _track_components(vec: np.ndarray, track_dir: np.ndarray) -> tuple:
        forward = _unit(track_dir)
        lateral = np.array([-forward[1], forward[0]], dtype=float)
        along = float(np.dot(vec, forward))
        cross = float(np.dot(vec, lateral))
        return along, cross, forward, lateral

    @staticmethod
    def _heading_from_yaw(yaw: float, fallback: np.ndarray) -> np.ndarray:
        if not np.isfinite(yaw):
            return _unit(fallback)
        return _unit(np.array([cos(float(yaw)), sin(float(yaw))], dtype=float), fallback)

    def _team_heading(self, local_yaw: float, fallback: np.ndarray) -> np.ndarray:
        heading_sum = self._heading_from_yaw(local_yaw, fallback)
        count = 1
        for state in self.remote_states.values():
            if not np.isfinite(state.yaw):
                continue
            heading_sum += self._heading_from_yaw(state.yaw, fallback)
            count += 1
        if count <= 1:
            return _unit(heading_sum, fallback)
        return _unit(heading_sum / float(count), fallback)

    def _inverted_vee_breakout_correction(
        self,
        local_position: np.ndarray,
        local_yaw: float,
        track_dir: np.ndarray,
        deploy_strength: float,
    ) -> np.ndarray:
        if deploy_strength <= 1e-3 or not isinstance(self.rally_point, np.ndarray):
            return np.zeros(2, dtype=float)

        forward = _unit(track_dir, self.approach_dir)
        heading = self._heading_from_yaw(local_yaw, forward)
        radial = local_position[:2] - self.rally_point[:2]
        radial_norm = float(np.linalg.norm(radial))
        if radial_norm < 1e-6:
            return np.zeros(2, dtype=float)

        radial_dir = radial / radial_norm
        tangent_cw = np.array([radial_dir[1], -radial_dir[0]], dtype=float)
        tangential_alignment = abs(float(np.dot(heading, tangent_cw)))
        if tangential_alignment < 0.35:
            return np.zeros(2, dtype=float)

        breakout_gain = deploy_strength * tangential_alignment * max(
            0.60,
            1.20 - 0.65 * abs(float(np.dot(heading, forward))),
        )
        return _clip_norm(
            -heading * (1.28 * self.config.spacing * breakout_gain),
            max(
                self.cruise_speed * max(self.config.lookahead_time * 1.0, 2.0),
                1.20 * self.config.spacing,
            ),
        )

    def _inverted_vee_front_promote_correction(
        self,
        local_position: np.ndarray,
        now: float,
        track_dir: np.ndarray,
        slot_map: Dict[int, np.ndarray],
    ) -> np.ndarray:
        if self._transit_shape() != FormationShape.INVERTED_VEE:
            return np.zeros(2, dtype=float)

        reference_id = self._reference_id(self.team_ids)
        if self.uav_id == reference_id or len(slot_map) < 3:
            return np.zeros(2, dtype=float)

        forward = _unit(track_dir, self.approach_dir)
        if reference_id == self.uav_id:
            reference_xy = local_position[:2].copy()
        else:
            reference_state = self.remote_states.get(reference_id)
            if reference_state is None:
                return np.zeros(2, dtype=float)
            reference_xy = self._predict_remote_position(reference_state, now)[:2]

        other_progress = []
        for uid in slot_map:
            if uid in (self.uav_id, reference_id):
                continue
            state = self.remote_states.get(uid)
            if state is None:
                return np.zeros(2, dtype=float)
            peer_xy = self._predict_remote_position(state, now)[:2]
            other_progress.append(float(np.dot(peer_xy - reference_xy, forward)))

        if len(other_progress) != 1:
            return np.zeros(2, dtype=float)

        local_progress = float(np.dot(local_position[:2] - reference_xy, forward))
        peer_progress = other_progress[0]
        behind_gate = -0.12 * self.config.spacing
        if (
            local_progress < behind_gate
            and peer_progress < behind_gate
            and local_progress > peer_progress + 0.08 * self.config.spacing
        ):
            gain = float(
                np.clip(
                    (local_progress - peer_progress) / max(0.45 * self.config.spacing, 1e-6),
                    0.0,
                    1.0,
                )
            )
            return forward * (0.52 * self.config.spacing * gain)
        return np.zeros(2, dtype=float)

    def _inverted_vee_axis_correction(
        self,
        local_position: np.ndarray,
        now: float,
        track_dir: np.ndarray,
        slot_map: Dict[int, np.ndarray],
    ) -> np.ndarray:
        if self._transit_shape() != FormationShape.INVERTED_VEE:
            return np.zeros(2, dtype=float)
        if len(slot_map) < 3 or self.uav_id != self._reference_id(self.team_ids):
            return np.zeros(2, dtype=float)

        forward = _unit(track_dir, self.approach_dir)
        lateral = np.array([-forward[1], forward[0]], dtype=float)

        peers = []
        for uid in slot_map:
            if uid == self.uav_id:
                continue
            state = self.remote_states.get(uid)
            if state is None:
                continue
            predicted = self._predict_remote_position(state, now)
            rel = predicted[:2] - local_position[:2]
            peers.append((predicted[:2], float(np.dot(rel, forward))))

        if len(peers) < 2:
            return np.zeros(2, dtype=float)

        peers.sort(key=lambda item: item[1], reverse=True)
        wing_a = peers[0][0]
        wing_b = peers[1][0]
        wing_mid = 0.5 * (wing_a + wing_b)
        span = float(np.linalg.norm(wing_a - wing_b))
        desired_back = max(0.82 * self.config.spacing, 0.52 * span)
        desired_pos = wing_mid - forward * desired_back

        to_axis = wing_mid - local_position[:2]
        desired_vec = desired_pos - local_position[:2]
        axis_error = float(np.dot(to_axis, lateral))
        along_gap = float(np.dot(desired_vec, forward))
        wing_a_along = float(np.dot(wing_a - local_position[:2], forward))
        wing_b_along = float(np.dot(wing_b - local_position[:2], forward))
        wing_a_cross = float(np.dot(wing_a - wing_mid, lateral))
        wing_b_cross = float(np.dot(wing_b - wing_mid, lateral))
        vee_ready = (
            wing_a_along > 0.24 * self.config.spacing
            and wing_b_along > 0.24 * self.config.spacing
            and wing_a_cross * wing_b_cross < 0.0
            and abs(axis_error) < 0.08 * self.config.spacing
        )
        if vee_ready:
            return np.zeros(2, dtype=float)

        lateral_gain = 1.85 if abs(axis_error) > 0.10 * self.config.spacing else 1.25
        along_gain = 1.15 if along_gap < 0.0 else 0.85
        correction = lateral * (axis_error * lateral_gain) + forward * (along_gap * along_gain)
        return _clip_norm(
            correction,
            max(
                self.cruise_speed * max(self.config.lookahead_time * 1.1, 2.0),
                1.45 * self.config.spacing,
            ),
        )

    def _shape_deploy_strength(
        self,
        now: float,
        remaining_to_rally: float,
    ) -> float:
        if len(self.team_ids) < 3:
            return 0.0
        if remaining_to_rally <= max(self.rally_loiter_radius * 0.9, 1.8 * self.config.spacing):
            return 0.0

        elapsed = max(0.0, now - (self.rally_started_at or now))
        deploy_window = float(
            np.clip(
                1.6 * self.config.spacing / max(self.cruise_speed, 1.0),
                3.0,
                6.5,
            )
        )
        return float(np.clip(1.0 - elapsed / deploy_window, 0.0, 1.0))

    def _shape_uses_frozen_slot_order(self, shape: FormationShape) -> bool:
        return shape in (
            FormationShape.ECHELON_LEFT,
            FormationShape.ECHELON_RIGHT,
            FormationShape.ARROW,
            FormationShape.INVERTED_VEE,
        )

    def _maybe_freeze_shape_slot_order(
        self,
        local_position: np.ndarray,
        now: float,
        track_dir: np.ndarray,
    ):
        shape = self._transit_shape()
        if not self._shape_uses_frozen_slot_order(shape):
            self.frozen_slot_order = None
            self.frozen_slot_shape = None
            return
        if self.frozen_slot_order and self.frozen_slot_shape == shape:
            return

        ordered = self._ordered_team()
        if len(ordered) != 3:
            return

        analysis_dir = track_dir
        forward = _unit(analysis_dir, self.approach_dir)
        lateral = np.array([-forward[1], forward[0]], dtype=float)
        samples = [
            (
                self.uav_id,
                float(np.dot(local_position[:2], lateral)),
                float(np.dot(local_position[:2], forward)),
                local_position[:2].copy(),
            )
        ]

        for uid in ordered:
            if uid == self.uav_id:
                continue
            state = self.remote_states.get(uid)
            if state is None:
                return
            predicted = self._predict_remote_position(state, now)
            samples.append(
                (
                    uid,
                    float(np.dot(predicted[:2], lateral)),
                    float(np.dot(predicted[:2], forward)),
                    predicted[:2].copy(),
                )
            )

        if shape == FormationShape.ARROW:
            center = max(samples, key=lambda item: (item[2], -abs(item[1]), -item[0]))
            wings = [item for item in samples if item[0] != center[0]]
            upper = max(wings, key=lambda item: (item[1], item[2], -item[0]))
            lower = min(wings, key=lambda item: (item[1], -item[2], item[0]))
            self.frozen_slot_order = [center[0], upper[0], lower[0]]
        elif shape == FormationShape.INVERTED_VEE:
            center = min(samples, key=lambda item: (item[2], abs(item[1]), item[0]))
            wings = [item for item in samples if item[0] != center[0]]
            upper = max(wings, key=lambda item: (item[1], item[2], -item[0]))
            lower = min(wings, key=lambda item: (item[1], -item[2], item[0]))
            self.frozen_slot_order = [center[0], upper[0], lower[0]]
        elif shape in (FormationShape.ECHELON_LEFT, FormationShape.ECHELON_RIGHT):
            self.frozen_slot_order = self._echelon_slot_order_from_samples(samples, shape)
        else:
            self.frozen_slot_order = None

        self.frozen_slot_shape = shape if self.frozen_slot_order else None

    def _assemble_guidance_slot_map(
        self,
        shape: FormationShape,
        slot_map: Dict[int, np.ndarray],
        now: float,
        track_dir: np.ndarray,
        locked: bool,
        deploy_strength: float = 0.0,
        remaining_distance: Optional[float] = None,
    ) -> Dict[int, np.ndarray]:
        guided_slot_map = {
            uid: np.asarray(offset, dtype=float).copy()
            for uid, offset in slot_map.items()
        }
        if not guided_slot_map:
            return guided_slot_map

        if shape in (
            FormationShape.ARROW,
            FormationShape.INVERTED_VEE,
        ):
            return {
                uid: self._assemble_guidance_slot(
                    shape,
                    offset,
                    track_dir,
                    locked=locked,
                    deploy_strength=deploy_strength,
                )
                for uid, offset in guided_slot_map.items()
            }

        if (
            shape not in (FormationShape.ECHELON_LEFT, FormationShape.ECHELON_RIGHT)
            or locked
            or len(guided_slot_map) != 3
        ):
            return guided_slot_map

        deploy_progress = self._echelon_deploy_progress(now, remaining_distance)
        side_sign = 1.0 if shape == FormationShape.ECHELON_LEFT else -1.0

        for index, uid in enumerate(list(guided_slot_map.keys())):
            final_slot = guided_slot_map[uid]
            if index == 0:
                guided_slot_map[uid] = np.array([0.0, 0.0, final_slot[2]], dtype=float)
                continue

            turn_start = 0.18 + 0.38 * (index - 1)
            turn_window = 0.34 if index == 1 else 0.24
            turn_progress = float(
                np.clip(
                    (deploy_progress - turn_start) / max(turn_window, 1e-6),
                    0.0,
                    1.0,
                )
            )
            initial_lateral = side_sign * index * 1.08 * self.config.spacing
            lateral = initial_lateral + (final_slot[1] - initial_lateral) * turn_progress
            along = final_slot[0] * turn_progress
            guided_slot_map[uid] = np.array([along, lateral, final_slot[2]], dtype=float)

        return guided_slot_map

    def _assemble_guidance_slot(
        self,
        shape: FormationShape,
        slot_offset: np.ndarray,
        track_dir: np.ndarray,
        locked: bool,
        deploy_strength: float = 0.0,
    ) -> np.ndarray:
        guided = np.asarray(slot_offset, dtype=float).copy()
        if deploy_strength <= 1e-3:
            return guided

        if shape == FormationShape.ARROW:
            if abs(guided[1]) > 1e-6:
                if guided[1] > 0.0:
                    guided[0] += 0.22 * self.config.spacing * deploy_strength
                    guided[1] *= 0.92
                else:
                    guided[0] += 0.10 * self.config.spacing * deploy_strength
                    guided[1] *= 0.98
            return guided

        if shape == FormationShape.INVERTED_VEE:
            if abs(guided[1]) < 1e-6:
                guided[0] -= 1.20 * self.config.spacing * deploy_strength
            else:
                guided[0] += 0.26 * self.config.spacing * deploy_strength
                guided[1] *= 0.94
            return guided

        return guided

    def _figure8_offset(
        self,
        now: float,
        track_dir: np.ndarray,
        amplitude: float,
        phase: float = 0.0,
    ) -> np.ndarray:
        elapsed = max(0.0, now - (self.rally_started_at or now))
        omega = self.cruise_speed / max(amplitude * 1.8, 1.0)
        tau = omega * elapsed + phase
        _, _, forward, lateral = self._track_components(np.zeros(2, dtype=float), track_dir)
        along = 0.38 * amplitude * np.sin(tau)
        cross = amplitude * np.sin(tau) * np.cos(tau)
        return forward * along + lateral * cross

    def _trail_reference_position(self, now: float, fallback_xy: np.ndarray) -> np.ndarray:
        reference_id = self._reference_id(self.team_ids)
        if self.uav_id == reference_id:
            return fallback_xy.copy()
        reference_state = self.remote_states.get(reference_id)
        if reference_state is None:
            return fallback_xy.copy()
        predicted = self._predict_remote_position(reference_state, now)
        return predicted[:2].copy()

    def _slot_target(self, slot_offset: np.ndarray) -> np.ndarray:
        rot = self._rotation()
        xy = self.virtual_center + rot.dot(slot_offset[:2])
        z = self.nominal_altitude + slot_offset[2]
        return np.array([xy[0], xy[1], z], dtype=float)

    def _predict_remote_position(self, state: TeamState, now: float) -> np.ndarray:
        age = max(0.0, min(now - state.timestamp, self.config.max_delay_comp))
        pred = state.position.copy()
        pred[:2] = pred[:2] + state.velocity[:2] * age
        return pred

    @staticmethod
    def _point_in_poly(x: float, y: float, poly) -> bool:
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]
            intersect = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-9) + xi
            )
            if intersect:
                inside = not inside
            j = i
        return inside

    @staticmethod
    def _closest_point_on_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        ab = b - a
        denom = float(np.dot(ab, ab))
        if denom < 1e-9:
            return a.copy()
        t = float(np.dot(point - a, ab) / denom)
        t = max(0.0, min(1.0, t))
        return a + t * ab

    def _closest_point_on_polygon(self, point: np.ndarray, poly) -> np.ndarray:
        best = None
        best_dist = float("inf")
        pts = [np.asarray(p, dtype=float) for p in poly]
        for idx in range(len(pts)):
            a = pts[idx]
            b = pts[(idx + 1) % len(pts)]
            cand = self._closest_point_on_segment(point, a, b)
            dist = float(np.linalg.norm(point - cand))
            if dist < best_dist:
                best = cand
                best_dist = dist
        return best if best is not None else np.asarray(poly[0], dtype=float)

    @staticmethod
    def _segment_intersects(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
        def orient(p, q, r):
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

        o1 = orient(a, b, c)
        o2 = orient(a, b, d)
        o3 = orient(c, d, a)
        o4 = orient(c, d, b)
        return (o1 * o2 < 0.0) and (o3 * o4 < 0.0)

    def _segment_hits_polygon(self, start: np.ndarray, end: np.ndarray, poly) -> bool:
        if self._point_in_poly(end[0], end[1], poly):
            return True
        pts = [np.asarray(p, dtype=float) for p in poly]
        for idx in range(len(pts)):
            if self._segment_intersects(start, end, pts[idx], pts[(idx + 1) % len(pts)]):
                return True
        return False

    def _inter_uav_repulsion(
        self,
        local_position: np.ndarray,
        now: float,
        slot_map: Optional[Dict[int, np.ndarray]] = None,
    ) -> np.ndarray:
        repel = np.zeros(2, dtype=float)
        local_slot = (
            slot_map.get(self.uav_id, np.zeros(3, dtype=float))
            if slot_map is not None
            else None
        )
        base_trigger = max(self.config.safe_separation * 0.92, 35.0)
        for state in self.remote_states.values():
            remote = self._predict_remote_position(state, now)
            diff = local_position[:2] - remote[:2]
            dist = float(np.linalg.norm(diff))
            if dist < 1e-6:
                diff = np.array([-self.approach_dir[1], self.approach_dir[0]], dtype=float)
                dist = float(np.linalg.norm(diff))

            influence = base_trigger
            if (
                slot_map is not None
                and local_slot is not None
                and state.uav_id in slot_map
            ):
                nominal_dist = float(
                    np.linalg.norm(slot_map[state.uav_id][:2] - local_slot[:2])
                )
                if nominal_dist > 1e-6:
                    influence = min(
                        max(base_trigger, 0.65 * nominal_dist),
                        0.82 * nominal_dist,
                    )

            if dist < influence:
                repel += (diff / dist) * (influence - dist)
        return _clip_norm(repel, 0.75 * self.config.spacing)

    def _obstacle_repulsion(
        self,
        current_xy: np.ndarray,
        desired_xy: np.ndarray,
        desired_z: float,
        zones: Optional[List[dict]],
    ) -> np.ndarray:
        if not zones:
            return np.zeros(2, dtype=float)

        repel = np.zeros(2, dtype=float)
        influence = max(self.config.safe_separation * 2.5, self.config.spacing)
        for zone in zones:
            z_min = float(zone.get("minAlt", -1e9))
            z_max = float(zone.get("maxAlt", 1e9))
            if desired_z < (z_min - 5.0) or desired_z > (z_max + 5.0):
                continue

            poly = zone.get("poly", [])
            if len(poly) < 3:
                continue

            nearest = self._closest_point_on_polygon(desired_xy, poly)
            diff = desired_xy - nearest
            dist = float(np.linalg.norm(diff))
            inside = self._point_in_poly(desired_xy[0], desired_xy[1], poly)
            crossing = self._segment_hits_polygon(current_xy, desired_xy, poly)
            if not inside and not crossing and dist >= influence:
                continue

            centroid = np.mean(np.asarray(poly, dtype=float), axis=0)
            if dist < 1e-6:
                diff = desired_xy - centroid
                dist = float(np.linalg.norm(diff))
            if dist < 1e-6:
                diff = np.array([-self.approach_dir[1], self.approach_dir[0]], dtype=float)
                dist = float(np.linalg.norm(diff))

            away = diff / dist
            strength = influence - min(dist, influence)
            if inside or crossing:
                strength += 0.6 * influence
            tangent = np.array([-away[1], away[0]], dtype=float)
            detour_sign = 1.0
            if np.cross(
                np.append(self.approach_dir, 0.0),
                np.append(centroid - current_xy, 0.0),
            )[2] < 0.0:
                detour_sign = -1.0
            repel += away * strength * self.config.obstacle_gain
            repel += tangent * detour_sign * strength * 0.35

        return _clip_norm(repel, 2.0 * self.config.spacing)

    def _update_common_target_time(self, now: float, self_sync_eta: Optional[float]):
        if self.phase != FormationPhase.HOLD:
            return

        if self.uav_id != self.leader_id:
            leader_state = self.remote_states.get(self.leader_id)
            if leader_state is not None and leader_state.common_target_time > 0.0:
                self.common_target_time = leader_state.common_target_time
            return

        etas = []
        if self_sync_eta is not None and self_sync_eta > 0.0:
            etas.append(float(self_sync_eta))
        for state in self.remote_states.values():
            if state.sync_eta is not None and state.sync_eta > 0.0:
                etas.append(float(state.sync_eta))
        if not etas:
            return

        if self.config.desired_target_time > now + 2.0:
            desired = self.config.desired_target_time
        else:
            desired = now + max(etas) + self.config.sync_margin

        if self.common_target_time <= 0.0 or (
            not self.released and desired > self.common_target_time + 1.0
        ):
            self.common_target_time = desired

    def step(
        self,
        local_position: List[float],
        local_velocity: List[float],
        local_yaw: float,
        now: float,
        zones: Optional[List[dict]] = None,
        sync_eta: Optional[float] = None,
        team_ids: Optional[Iterable[int]] = None,
    ) -> dict:
        if not self.active or self.target_center is None or self.entry_center is None:
            return {
                "active": False,
                "released": self.released,
                "phase": FormationPhase.RELEASE.value if self.released else FormationPhase.IDLE.value,
                "slot_id": 0,
                "common_target_time": self.common_target_time,
            }

        local_pos = np.asarray(local_position, dtype=float)
        self.last_local_position = local_pos.copy()
        self.last_local_update_time = now
        local_vel = np.asarray(local_velocity, dtype=float)
        dt = 0.1 if self.last_tick_time is None else max(0.02, min(now - self.last_tick_time, 1.0))
        self.last_tick_time = now

        remaining_to_entry = float(np.linalg.norm(self.entry_center - self.virtual_center))
        if remaining_to_entry > self.config.hold_radius:
            self.phase = FormationPhase.ASSEMBLE
            advance_dir = _unit(self.entry_center - self.virtual_center, self.approach_dir)
            step_size = min(max(self.cruise_speed * dt, 20.0), remaining_to_entry)
            self.virtual_center = self.virtual_center + advance_dir * step_size
        else:
            self.phase = FormationPhase.HOLD
            self.virtual_center = self.entry_center.copy()
            if self.hold_started_at is None:
                self.hold_started_at = now

        self.team_ids = self._ordered_team(team_ids)
        self._maybe_freeze_shape_slot_order(local_pos, now, self.approach_dir)
        slot_map = self._slot_map(self.team_ids)
        slot_id = self.get_slot_id(self.uav_id)
        reference_id = self._reference_id(self.team_ids)
        is_reference = self.uav_id == reference_id

        self._update_common_target_time(now, sync_eta)

        shape = self._transit_shape()
        guided_slot_map = (
            self._assemble_guidance_slot_map(
                shape,
                slot_map,
                now,
                self.approach_dir,
                locked=self.phase != FormationPhase.ASSEMBLE,
                remaining_distance=remaining_to_entry,
            )
            if self.phase == FormationPhase.ASSEMBLE
            else slot_map
        )
        self_slot = guided_slot_map.get(self.uav_id, np.zeros(3, dtype=float))
        slot_target = self._slot_target(self_slot)
        slot_error = slot_target[:2] - local_pos[:2]
        along_error, cross_error, forward, lateral = self._track_components(
            slot_error, self.approach_dir
        )
        echelon_deploy_progress = (
            self._echelon_deploy_progress(now, remaining_to_entry)
            if self.phase == FormationPhase.ASSEMBLE
            and shape in (
                FormationShape.ECHELON_LEFT,
                FormationShape.ECHELON_RIGHT,
            )
            else 1.0
        )
        lateral_gain = 1.0 if self.phase != FormationPhase.ASSEMBLE else 0.65
        if shape == FormationShape.TRAIL:
            lateral_gain = 1.0 if self.phase != FormationPhase.ASSEMBLE else 1.05
        elif shape in (FormationShape.ECHELON_LEFT, FormationShape.ECHELON_RIGHT):
            lateral_gain = (
                1.0
                if self.phase != FormationPhase.ASSEMBLE
                else (1.02 - 0.14 * echelon_deploy_progress)
            )
        if self.phase == FormationPhase.ASSEMBLE and is_reference:
            slot_correction = forward * max(along_error, 0.55 * self.config.spacing) + lateral * (
                cross_error * (0.18 if shape == FormationShape.TRAIL else 0.25)
            )
        else:
            along_gain = 1.0 if along_error >= 0.0 else (
                0.20 if shape == FormationShape.TRAIL else 0.30
            )
            if (
                self.phase == FormationPhase.ASSEMBLE
                and shape in (FormationShape.ECHELON_LEFT, FormationShape.ECHELON_RIGHT)
                and not is_reference
                and along_error < 0.0
            ):
                reverse_limit = (
                    0.02 + 0.24 * echelon_deploy_progress
                ) * self.config.spacing
                along_term = max(
                    along_error * (0.06 + 0.38 * echelon_deploy_progress),
                    -reverse_limit,
                )
            else:
                along_term = along_error * along_gain
            slot_correction = forward * along_term + lateral * (
                cross_error * lateral_gain
            )

        consensus = np.zeros(2, dtype=float)
        count = 0
        for uid, state in self.remote_states.items():
            if uid not in slot_map:
                continue
            predicted = self._predict_remote_position(state, now)
            expected_rel = guided_slot_map.get(uid, slot_map[uid])[:2] - self_slot[:2]
            consensus += (predicted[:2] - local_pos[:2]) - expected_rel
            count += 1
        if count > 0:
            consensus /= count

        inter_uav = self._inter_uav_repulsion(local_pos, now, slot_map)
        obstacle = self._obstacle_repulsion(local_pos[:2], slot_target[:2], slot_target[2], zones)
        consensus_gain = self.config.consensus_gain
        consensus_correction = consensus
        if self.phase == FormationPhase.ASSEMBLE and is_reference:
            consensus_gain = 0.0
            consensus_correction = np.zeros(2, dtype=float)
            inter_uav *= 0.55
        elif self.phase == FormationPhase.ASSEMBLE:
            if shape in (
                FormationShape.TRAIL,
                FormationShape.ECHELON_LEFT,
                FormationShape.ECHELON_RIGHT,
            ):
                consensus_gain *= 0.55
            consensus_correction = self._track_weighted_vector(
                consensus, self.approach_dir, lateral_gain
            )
        correction = (
            self.config.slot_gain * slot_correction
            + consensus_gain * consensus_correction
            + inter_uav
            + obstacle
        )
        correction = _clip_norm(
            correction, max(self.cruise_speed * self.config.lookahead_time, self.config.spacing)
        )
        command_xy = local_pos[:2] + correction
        command_z = max(slot_target[2], 80.0)
        yaw_target = (
            atan2(self.target_center[1] - local_pos[1], self.target_center[0] - local_pos[0])
            if np.linalg.norm(self.target_center - local_pos[:2]) > 1.0
            else float(local_yaw)
        )

        should_release = False
        release_time = None
        if self.phase == FormationPhase.HOLD:
            if sync_eta is not None and sync_eta > 0.0 and self.common_target_time > 0.0:
                release_time = self.common_target_time - sync_eta
                should_release = now + self.config.release_slack >= release_time
            elif self.hold_started_at is not None and (now - self.hold_started_at) >= 1.0:
                should_release = True

        if should_release:
            self.phase = FormationPhase.RELEASE
            self.active = False
            self.released = True
            return {
                "active": False,
                "released": True,
                "phase": FormationPhase.RELEASE.value,
                "slot_id": slot_id,
                "waypoint": [command_xy[0], command_xy[1], command_z],
                "yaw": yaw_target,
                "release_time": release_time,
                "common_target_time": self.common_target_time,
            }

        return {
            "active": True,
            "released": False,
            "phase": self.phase.value,
            "slot_id": slot_id,
            "waypoint": [command_xy[0], command_xy[1], command_z],
            "yaw": yaw_target,
            "entry_distance": float(np.linalg.norm(local_pos[:2] - self.entry_center)),
            "common_target_time": self.common_target_time,
        }
