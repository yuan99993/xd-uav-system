"""World-coordinate observation association and global target lifecycle."""

from dataclasses import dataclass, field
from math import hypot
from typing import Dict, List, Optional, Set, Tuple


TARGET_CANDIDATE = 0
TARGET_CONFIRMED = 1
TARGET_ASSIGNED = 2
TARGET_EXECUTING = 3
TARGET_COMPLETED = 4
TARGET_STALE = 5


@dataclass(frozen=True)
class TargetObservation:
    source_uav: str
    stamp: float
    class_id: int
    position: Tuple[float, float, float]
    confidence: float
    covariance: Tuple[float, ...] = (0.0,) * 9


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


@dataclass(frozen=True)
class RegistryUpdate:
    target: GlobalTargetRecord
    created: bool
    newly_confirmed: bool


class GlobalTargetRegistry:
    def __init__(
        self,
        association_radius_m: float = 4.0,
        confirmation_hits: int = 5,
        confirmation_window_sec: float = 2.0,
        confirmation_minimum_span_sec: float = 0.25,
        confirmation_distinct_uavs: int = 2,
        stale_timeout_sec: float = 15.0,
    ):
        self.association_radius_m = max(0.1, float(association_radius_m))
        self.confirmation_hits = max(1, int(confirmation_hits))
        self.confirmation_window_sec = max(0.1, float(confirmation_window_sec))
        self.confirmation_minimum_span_sec = max(
            0.0, float(confirmation_minimum_span_sec)
        )
        self.confirmation_distinct_uavs = max(1, int(confirmation_distinct_uavs))
        self.stale_timeout_sec = max(0.1, float(stale_timeout_sec))
        self.targets: Dict[int, GlobalTargetRecord] = {}
        self._next_target_id = 1

    def _compatible(self, target: GlobalTargetRecord, observation: TargetObservation) -> bool:
        if target.status == TARGET_STALE:
            return False
        if target.class_id >= 0 and observation.class_id >= 0 and target.class_id != observation.class_id:
            return False
        return hypot(
            target.position[0] - observation.position[0],
            target.position[1] - observation.position[1],
        ) <= self.association_radius_m

    def _find_target(self, observation: TargetObservation) -> Optional[GlobalTargetRecord]:
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

    def observe(self, observation: TargetObservation) -> RegistryUpdate:
        target = self._find_target(observation)
        created = target is None
        if target is None:
            weight = max(0.05, min(1.0, float(observation.confidence)))
            target = GlobalTargetRecord(
                target_id=self._next_target_id,
                class_id=int(observation.class_id),
                position=list(observation.position),
                covariance=list(observation.covariance[:9]),
                confidence=weight,
                first_seen=float(observation.stamp),
                last_seen=float(observation.stamp),
                observer_uavs={str(observation.source_uav)},
                recent_observations=[(float(observation.stamp), str(observation.source_uav))],
                accumulated_weight=weight,
            )
            self.targets[target.target_id] = target
            self._next_target_id += 1
        else:
            weight = max(0.05, min(1.0, float(observation.confidence)))
            old_weight = min(100.0, target.accumulated_weight)
            total_weight = old_weight + weight
            for axis in range(3):
                target.position[axis] = (
                    old_weight * target.position[axis] + weight * observation.position[axis]
                ) / total_weight
            if len(observation.covariance) >= 9:
                target.covariance = [
                    (old_weight * target.covariance[index] + weight * observation.covariance[index])
                    / total_weight
                    for index in range(9)
                ]
            target.accumulated_weight = total_weight
            target.confidence = min(1.0, 0.75 * weight + 0.25 * target.confidence)
            target.last_seen = max(target.last_seen, float(observation.stamp))
            target.observation_count += 1
            target.observer_uavs.add(str(observation.source_uav))
            observation_key = (float(observation.stamp), str(observation.source_uav))
            if not any(
                abs(existing_stamp - observation_key[0]) <= 1e-6
                and existing_source == observation_key[1]
                for existing_stamp, existing_source in target.recent_observations
            ):
                # Multiple overlapping boxes from the same detector frame are
                # one piece of confirmation evidence, not multiple hits.
                target.recent_observations.append(observation_key)

        cutoff = float(observation.stamp) - self.confirmation_window_sec
        target.recent_observations = [item for item in target.recent_observations if item[0] >= cutoff]
        recent_sources = {item[1] for item in target.recent_observations}
        observation_span = (
            target.recent_observations[-1][0] - target.recent_observations[0][0]
            if len(target.recent_observations) >= 2
            else 0.0
        )
        newly_confirmed = False
        if target.status == TARGET_CANDIDATE and (
            (
                len(target.recent_observations) >= self.confirmation_hits
                and observation_span >= self.confirmation_minimum_span_sec
            )
            or len(recent_sources) >= self.confirmation_distinct_uavs
        ):
            target.status = TARGET_CONFIRMED
            newly_confirmed = True
        return RegistryUpdate(target=target, created=created, newly_confirmed=newly_confirmed)

    def set_status(self, target_id: int, status: int) -> bool:
        target = self.targets.get(int(target_id))
        if target is None:
            return False
        target.status = int(status)
        return True

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
