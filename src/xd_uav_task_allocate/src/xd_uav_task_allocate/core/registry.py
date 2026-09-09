"""World-coordinate observation association and global target lifecycle."""

from dataclasses import dataclass, field
from math import hypot, isfinite, sqrt
from statistics import median
from typing import Dict, Iterable, List, Optional, Set, Tuple


TARGET_CANDIDATE = 0
TARGET_CONFIRMED = 1
TARGET_ASSIGNED = 2
TARGET_EXECUTING = 3
TARGET_COMPLETED = 4
TARGET_STALE = 5
TARGET_VERIFYING = 6


@dataclass(frozen=True)
class TargetObservation:
    source_uav: str
    stamp: float
    class_id: int
    position: Tuple[float, float, float]
    confidence: float
    covariance: Tuple[float, ...] = (0.0,) * 9
    source_mobility_profile: str = "hover"
    track_id: int = -1
    track_id_is_stable: bool = False
    sensor_id: str = ""


@dataclass
class GlobalTargetRecord:
    target_id: int
    class_id: int
    position: List[float]
    covariance: List[float]
    confidence: float
    first_seen: float
    last_seen: float
    observation_count: int = 1
    observer_uavs: Set[str] = field(default_factory=set)
    status: int = TARGET_CANDIDATE
    recent_observations: List[Tuple[float, str]] = field(default_factory=list)
    accumulated_weight: float = 1.0
    observer_mobility_profiles: Dict[str, str] = field(default_factory=dict)
    coarse_evidence_ready: bool = False
    # A detector track is local to one aircraft and sensor.  Several local
    # tracks may therefore be aliases of one physical world target.
    source_tracks: Set[Tuple[str, str, int]] = field(default_factory=set)
    # Keep class identity separate from physical identity.  class_id is the
    # current weighted winner; individual observations remain available for
    # confirmation stability checks.
    class_scores: Dict[int, float] = field(default_factory=dict)
    recent_measurements: List[TargetObservation] = field(default_factory=list)
    # Stable, different tracks observed in the same source frame prove that
    # two nearby records are distinct physical detections.  The task-level
    # spatial safety net must never collapse such a pair.
    known_distinct_target_ids: Set[int] = field(default_factory=set)


@dataclass(frozen=True)
class RegistryUpdate:
    target: GlobalTargetRecord
    created: bool
    newly_confirmed: bool
    newly_evidence_ready: bool


class GlobalTargetRegistry:
    def __init__(
        self,
        association_radius_m: float = 4.0,
        confirmation_hits: int = 5,
        confirmation_window_sec: float = 2.0,
        confirmation_minimum_span_sec: float = 0.25,
        confirmation_distinct_uavs: int = 2,
        stale_timeout_sec: float = 15.0,
        confirmation_mobility_profiles: Optional[Iterable[str]] = None,
        association_covariance_sigma: float = 3.0,
        maximum_association_radius_m: float = 50.0,
        cross_class_association_radius_m: float = 3.0,
        stable_track_maximum_jump_m: float = 20.0,
        class_confirmation_minimum_observations: int = 3,
        class_confirmation_minimum_ratio: float = 0.65,
        maximum_confirmation_position_spread_m: float = 3.0,
        position_history_size: int = 30,
    ):
        self.association_radius_m = max(0.1, float(association_radius_m))
        self.confirmation_hits = max(1, int(confirmation_hits))
        self.confirmation_window_sec = max(0.1, float(confirmation_window_sec))
        self.confirmation_minimum_span_sec = max(
            0.0, float(confirmation_minimum_span_sec)
        )
        self.confirmation_distinct_uavs = max(1, int(confirmation_distinct_uavs))
        self.stale_timeout_sec = max(0.1, float(stale_timeout_sec))
        requested_types = (
            ("hover", "fixedwing")
            if confirmation_mobility_profiles is None
            else tuple(
                str(item).strip().lower()
                for item in confirmation_mobility_profiles
            )
        )
        self.confirmation_mobility_profiles = {
            item for item in requested_types if item in ("hover", "fixedwing")
        }
        if not self.confirmation_mobility_profiles:
            raise ValueError("confirmation_mobility_profiles must not be empty")
        self.association_covariance_sigma = max(
            0.0, float(association_covariance_sigma)
        )
        self.maximum_association_radius_m = max(
            self.association_radius_m, float(maximum_association_radius_m)
        )
        self.cross_class_association_radius_m = max(
            0.0, float(cross_class_association_radius_m)
        )
        self.stable_track_maximum_jump_m = max(
            self.association_radius_m, float(stable_track_maximum_jump_m)
        )
        self.class_confirmation_minimum_observations = max(
            1, int(class_confirmation_minimum_observations)
        )
        self.class_confirmation_minimum_ratio = min(
            1.0, max(0.0, float(class_confirmation_minimum_ratio))
        )
        self.maximum_confirmation_position_spread_m = max(
            0.0, float(maximum_confirmation_position_spread_m)
        )
        self.position_history_size = max(3, int(position_history_size))
        self.targets: Dict[int, GlobalTargetRecord] = {}
        self._source_track_to_target: Dict[Tuple[str, str, int], int] = {}
        self._next_target_id = 1

    @staticmethod
    def _source_track_key(
        observation: TargetObservation,
    ) -> Optional[Tuple[str, str, int]]:
        if not observation.track_id_is_stable or int(observation.track_id) < 0:
            return None
        return (
            str(observation.source_uav),
            str(observation.sensor_id).strip(),
            int(observation.track_id),
        )

    @staticmethod
    def _horizontal_variance(covariance) -> float:
        if len(covariance) < 5:
            return 0.0
        xx = float(covariance[0])
        xy = 0.5 * (float(covariance[1]) + float(covariance[3]))
        yy = float(covariance[4])
        if not all(isfinite(value) for value in (xx, xy, yy)):
            return 0.0
        discriminant = max(0.0, (xx - yy) ** 2 + 4.0 * xy * xy)
        return max(0.0, 0.5 * (xx + yy + sqrt(discriminant)))

    def _association_radius(
        self, target: GlobalTargetRecord, observation: TargetObservation
    ) -> float:
        combined_sigma = sqrt(
            self._horizontal_variance(target.covariance)
            + self._horizontal_variance(observation.covariance)
        )
        return min(
            self.maximum_association_radius_m,
            max(
                self.association_radius_m,
                self.association_covariance_sigma * combined_sigma,
            ),
        )

    @classmethod
    def _observation_weight(cls, observation: TargetObservation) -> float:
        confidence = max(0.05, min(1.0, float(observation.confidence)))
        variance = cls._horizontal_variance(observation.covariance)
        # Legacy publishers often use an all-zero covariance to mean unknown;
        # retain the former confidence-only weighting for that case. For real
        # covariances, a precise hover-profile measurement should dominate a
        # high-altitude fixed-wing ground-plane estimate.
        if variance <= 1e-9:
            return confidence
        return max(1e-3, confidence / max(1.0, variance))

    @staticmethod
    def _same_source_frame(
        target: GlobalTargetRecord, observation: TargetObservation
    ) -> bool:
        return any(
            item.source_uav == observation.source_uav
            and abs(float(item.stamp) - float(observation.stamp)) <= 1e-6
            for item in target.recent_measurements
        )

    def _same_frame_distinct_track(
        self, target: GlobalTargetRecord, observation: TargetObservation
    ) -> bool:
        observation_key = self._source_track_key(observation)
        if observation_key is None:
            return False
        for item in target.recent_measurements:
            if (
                item.source_uav != observation.source_uav
                or abs(float(item.stamp) - float(observation.stamp)) > 1e-6
            ):
                continue
            item_key = self._source_track_key(item)
            if item_key is not None and item_key != observation_key:
                return True
        return False

    def _compatible(self, target: GlobalTargetRecord, observation: TargetObservation) -> bool:
        if target.status == TARGET_STALE:
            return False
        distance = hypot(
            target.position[0] - observation.position[0],
            target.position[1] - observation.position[1],
        )
        if self._same_frame_distinct_track(target, observation):
            return False
        class_conflict = (
            target.class_id >= 0
            and observation.class_id >= 0
            and target.class_id != observation.class_id
        )
        if not class_conflict:
            return distance <= self._association_radius(target, observation)
        # A class label is evidence, not physical identity.  Permit a tighter
        # cross-class gate for temporal class flicker, while keeping two
        # different-class detections from the same camera frame distinct.
        if self._same_source_frame(target, observation):
            return False
        return distance <= min(
            self.cross_class_association_radius_m,
            self._association_radius(target, observation),
        )

    def _find_target(self, observation: TargetObservation) -> Optional[GlobalTargetRecord]:
        track_key = self._source_track_key(observation)
        if track_key is not None:
            track_target = self.targets.get(
                self._source_track_to_target.get(track_key, -1)
            )
            if track_target is not None and track_target.status != TARGET_STALE:
                distance = hypot(
                    track_target.position[0] - observation.position[0],
                    track_target.position[1] - observation.position[1],
                )
                if distance <= self.stable_track_maximum_jump_m:
                    return track_target
        candidates = [target for target in self.targets.values() if self._compatible(target, observation)]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda target: (
                hypot(target.position[0] - observation.position[0], target.position[1] - observation.position[1]),
                target.target_id,
            ),
        )

    @staticmethod
    def _measurement_key(observation: TargetObservation) -> Tuple[float, str]:
        return float(observation.stamp), str(observation.source_uav)

    @staticmethod
    def _class_winner(measurements: Iterable[TargetObservation]):
        scores: Dict[int, float] = {}
        counts: Dict[int, int] = {}
        for measurement in measurements:
            class_id = int(measurement.class_id)
            confidence = max(0.05, min(1.0, float(measurement.confidence)))
            scores[class_id] = scores.get(class_id, 0.0) + confidence
            counts[class_id] = counts.get(class_id, 0) + 1
        if not scores:
            return None, 0.0, 0, scores
        winner = min(
            scores,
            key=lambda class_id: (-scores[class_id], class_id),
        )
        total = sum(scores.values())
        ratio = scores[winner] / total if total > 1e-9 else 0.0
        return winner, ratio, counts[winner], scores

    def _position_stable(
        self, measurements: Iterable[TargetObservation]
    ) -> bool:
        measurements = list(measurements)
        if len(measurements) < self.class_confirmation_minimum_observations:
            return False
        center_x = median(item.position[0] for item in measurements)
        center_y = median(item.position[1] for item in measurements)
        spread = max(
            hypot(item.position[0] - center_x, item.position[1] - center_y)
            for item in measurements
        )
        return spread <= self.maximum_confirmation_position_spread_m

    def _refresh_class_identity(self, target: GlobalTargetRecord) -> None:
        winner, _, _, scores = self._class_winner(target.recent_measurements)
        target.class_scores = scores
        # Once a task-capable target is confirmed, its class must not silently
        # change underneath an assigned worker.  Candidate and verifying
        # targets continue accumulating evidence until confirmation.
        if winner is not None and target.status in (
            TARGET_CANDIDATE,
            TARGET_VERIFYING,
        ):
            target.class_id = int(winner)

    def observe(self, observation: TargetObservation) -> RegistryUpdate:
        mobility_profile = str(observation.source_mobility_profile).strip().lower()
        if mobility_profile not in ("hover", "fixedwing"):
            mobility_profile = "hover"
        target = self._find_target(observation)
        created = target is None
        if target is None:
            confidence = max(0.05, min(1.0, float(observation.confidence)))
            weight = self._observation_weight(observation)
            target = GlobalTargetRecord(
                target_id=self._next_target_id,
                class_id=int(observation.class_id),
                position=list(observation.position),
                covariance=list(observation.covariance[:9]),
                confidence=confidence,
                first_seen=float(observation.stamp),
                last_seen=float(observation.stamp),
                observer_uavs={str(observation.source_uav)},
                recent_observations=[(float(observation.stamp), str(observation.source_uav))],
                accumulated_weight=weight,
                observer_mobility_profiles={
                    str(observation.source_uav): mobility_profile
                },
                recent_measurements=[observation],
            )
            track_key = self._source_track_key(observation)
            if track_key is not None:
                target.source_tracks.add(track_key)
                self._source_track_to_target[track_key] = target.target_id
            target.class_scores = {int(observation.class_id): confidence}
            for other in self.targets.values():
                if self._same_frame_distinct_track(other, observation):
                    target.known_distinct_target_ids.add(other.target_id)
                    other.known_distinct_target_ids.add(target.target_id)
            self.targets[target.target_id] = target
            self._next_target_id += 1
        else:
            observation_key = self._measurement_key(observation)
            same_frame = any(
                abs(item.stamp - observation_key[0]) <= 1e-6
                and item.source_uav == observation_key[1]
                for item in target.recent_measurements
            )
            confidence = max(0.05, min(1.0, float(observation.confidence)))
            weight = self._observation_weight(observation)
            if not same_frame:
                old_weight = min(100.0, target.accumulated_weight)
                total_weight = old_weight + weight
                for axis in range(3):
                    target.position[axis] = (
                        old_weight * target.position[axis]
                        + weight * observation.position[axis]
                    ) / total_weight
                if len(observation.covariance) >= 9:
                    target.covariance = [
                        (
                            old_weight * target.covariance[index]
                            + weight * observation.covariance[index]
                        )
                        / total_weight
                        for index in range(9)
                    ]
                target.accumulated_weight = total_weight
                target.confidence = min(
                    1.0, 0.75 * confidence + 0.25 * target.confidence
                )
                target.last_seen = max(target.last_seen, float(observation.stamp))
                target.observation_count += 1
                target.observer_uavs.add(str(observation.source_uav))
                target.observer_mobility_profiles[
                    str(observation.source_uav)
                ] = mobility_profile
                # Multiple overlapping boxes from the same detector frame are
                # one piece of confirmation evidence, not multiple hits.
                target.recent_observations.append(observation_key)
                target.recent_measurements.append(observation)
            track_key = self._source_track_key(observation)
            if track_key is not None:
                target.source_tracks.add(track_key)
                self._source_track_to_target[track_key] = target.target_id

        cutoff = float(observation.stamp) - self.confirmation_window_sec
        target.recent_observations = sorted(
            (item for item in target.recent_observations if item[0] >= cutoff),
            key=lambda item: (item[0], item[1]),
        )
        target.recent_measurements = sorted(
            (
                item
                for item in target.recent_measurements
                if float(item.stamp) >= cutoff
            ),
            key=lambda item: (float(item.stamp), str(item.source_uav)),
        )[-self.position_history_size :]
        self._refresh_class_identity(target)
        recent_sources = {item[1] for item in target.recent_observations}
        observation_span = (
            target.recent_observations[-1][0] - target.recent_observations[0][0]
            if len(target.recent_observations) >= 2
            else 0.0
        )
        evidence_ready = (
            (
                len(target.recent_observations) >= self.confirmation_hits
                and observation_span >= self.confirmation_minimum_span_sec
            )
            or len(recent_sources) >= self.confirmation_distinct_uavs
        )
        newly_evidence_ready = evidence_ready and not target.coarse_evidence_ready
        if newly_evidence_ready:
            target.coarse_evidence_ready = True

        confirmable_observations = [
            item
            for item in target.recent_observations
            if target.observer_mobility_profiles.get(item[1], "hover")
            in self.confirmation_mobility_profiles
        ]
        confirmable_sources = {item[1] for item in confirmable_observations}
        confirmable_measurements = [
            item
            for item in target.recent_measurements
            if str(item.source_mobility_profile).strip().lower()
            in self.confirmation_mobility_profiles
        ]
        class_winner, class_ratio, class_count, _ = self._class_winner(
            confirmable_measurements
        )
        class_stable = (
            class_winner is not None
            and class_count >= self.class_confirmation_minimum_observations
            and class_ratio >= self.class_confirmation_minimum_ratio
        )
        position_stable = self._position_stable(confirmable_measurements)
        confirmable_span = (
            confirmable_observations[-1][0] - confirmable_observations[0][0]
            if len(confirmable_observations) >= 2
            else 0.0
        )
        newly_confirmed = False
        if (
            target.status in (TARGET_CANDIDATE, TARGET_VERIFYING)
            and class_stable
            and position_stable
            and (
                (
                    len(confirmable_observations) >= self.confirmation_hits
                    and confirmable_span >= self.confirmation_minimum_span_sec
                )
                or len(confirmable_sources) >= self.confirmation_distinct_uavs
            )
        ):
            target.class_id = int(class_winner)
            target.status = TARGET_CONFIRMED
            newly_confirmed = True
        return RegistryUpdate(
            target=target,
            created=created,
            newly_confirmed=newly_confirmed,
            newly_evidence_ready=newly_evidence_ready,
        )

    def set_status(self, target_id: int, status: int) -> bool:
        target = self.targets.get(int(target_id))
        if target is None:
            return False
        target.status = int(status)
        return True

    def remove(self, target_id: int) -> bool:
        """Remove one operator-rejected target without reusing its ID."""

        removed = self.targets.pop(int(target_id), None)
        if removed is None:
            return False
        for track_key in removed.source_tracks:
            if self._source_track_to_target.get(track_key) == removed.target_id:
                self._source_track_to_target.pop(track_key, None)
        for target in self.targets.values():
            target.known_distinct_target_ids.discard(removed.target_id)
        return True

    def clear(self, reset_ids: bool = True) -> None:
        """Clear all fused targets for a fresh mission run."""

        self.targets.clear()
        self._source_track_to_target.clear()
        if reset_ids:
            self._next_target_id = 1

    def expire(self, now: float) -> List[int]:
        expired: List[int] = []
        for target in self.targets.values():
            # Only an unconfirmed candidate may age out. Once a target is
            # confirmed, the coordinator creates a rescue task for it. That
            # task may legitimately wait longer than stale_timeout_sec while
            # the only worker executes earlier tasks. Keeping the confirmed
            # target association-active prevents a later observation of the
            # same object from creating a duplicate task.
            if target.status != TARGET_CANDIDATE:
                continue
            if float(now) - target.last_seen > self.stale_timeout_sec:
                target.status = TARGET_STALE
                expired.append(target.target_id)
        return expired
