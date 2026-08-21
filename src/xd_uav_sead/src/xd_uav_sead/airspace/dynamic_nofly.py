"""Validated ROS-facing dynamic no-fly-zone ingestion.

The validator is deliberately independent of rospy so geometry and message
contract behavior can be exercised without a ROS master.  The ROS callback
thread only validates and queues events; flight-control decisions remain in
the onboard main loop.
"""

from dataclasses import dataclass
import math
import threading
from typing import Optional

from xd_uav_sead.airspace.airspace_manager import ZoneDef


class NoFlyZoneValidationError(ValueError):
    pass


@dataclass(frozen=True)
class DynamicNoFlyConfig:
    expected_frame: str
    max_message_age: float = 0.5
    future_stamp_tolerance: float = 0.05
    max_ttl: float = 60.0
    min_altitude_limit: float = -1000.0
    max_altitude_limit: float = 10000.0
    max_abs_coordinate: float = 100000.0
    min_polygon_area: float = 1.0
    max_vertices: int = 64


@dataclass(frozen=True)
class AirspaceUpdateEvent:
    revision: int
    operation: int
    zone_id: int
    accepted: bool
    fault: bool
    reason: str


def _seconds(value) -> float:
    if hasattr(value, "to_sec"):
        return float(value.to_sec())
    if hasattr(value, "secs"):
        return float(value.secs) + float(getattr(value, "nsecs", 0)) * 1e-9
    return float(value)


def _orientation(a, b, c) -> float:
    return ((b[0] - a[0]) * (c[1] - a[1])) - (
        (b[1] - a[1]) * (c[0] - a[0])
    )


def _on_segment(a, b, p, eps=1e-9) -> bool:
    return (
        abs(_orientation(a, b, p)) <= eps
        and min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
        and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps
    )


def _segments_intersect(a, b, c, d, eps=1e-9) -> bool:
    o1 = _orientation(a, b, c)
    o2 = _orientation(a, b, d)
    o3 = _orientation(c, d, a)
    o4 = _orientation(c, d, b)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and (
        (o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)
    ):
        return True
    return (
        _on_segment(a, b, c, eps)
        or _on_segment(a, b, d, eps)
        or _on_segment(c, d, a, eps)
        or _on_segment(c, d, b, eps)
    )


def _polygon_area(vertices) -> float:
    return 0.5 * abs(
        sum(
            vertices[i][0] * vertices[(i + 1) % len(vertices)][1]
            - vertices[(i + 1) % len(vertices)][0] * vertices[i][1]
            for i in range(len(vertices))
        )
    )


def _validate_simple_polygon(vertices, config: DynamicNoFlyConfig):
    if len(vertices) < 3:
        raise NoFlyZoneValidationError("polygon requires at least three vertices")
    if len(vertices) > int(config.max_vertices):
        raise NoFlyZoneValidationError("polygon exceeds max_vertices")
    for index, point in enumerate(vertices):
        if len(point) < 2 or not all(math.isfinite(float(v)) for v in point[:2]):
            raise NoFlyZoneValidationError(f"vertex {index} is not finite")
        if max(abs(float(point[0])), abs(float(point[1]))) > config.max_abs_coordinate:
            raise NoFlyZoneValidationError(f"vertex {index} exceeds coordinate limit")
        nxt = vertices[(index + 1) % len(vertices)]
        if math.hypot(float(nxt[0]) - point[0], float(nxt[1]) - point[1]) <= 1e-6:
            raise NoFlyZoneValidationError("polygon has duplicate adjacent vertices")
    if _polygon_area(vertices) < config.min_polygon_area:
        raise NoFlyZoneValidationError("polygon area is below minimum")
    edge_count = len(vertices)
    for i in range(edge_count):
        a = vertices[i]
        b = vertices[(i + 1) % edge_count]
        for j in range(i + 1, edge_count):
            if j in (i, (i + 1) % edge_count) or (j + 1) % edge_count == i:
                continue
            c = vertices[j]
            d = vertices[(j + 1) % edge_count]
            if _segments_intersect(a, b, c, d):
                raise NoFlyZoneValidationError("polygon self-intersects")


class DynamicNoFlyZoneReceiver:
    SCHEMA_VERSION = 1
    OP_UPSERT = 0
    OP_REMOVE = 1
    OP_CLEAR = 2
    TYPE_NO_FLY = 0

    def __init__(self, airspace, config: DynamicNoFlyConfig):
        if not str(config.expected_frame or "").strip():
            raise ValueError("expected_frame must be non-empty")
        self.airspace = airspace
        self.config = config
        self._events = []
        self._expirations = {}
        self._expired_reported = set()
        self._revision = int(getattr(airspace, "revision", 0))
        self._lock = threading.RLock()

    @property
    def revision(self):
        with self._lock:
            return self._revision

    def _event(self, operation, zone_id, accepted, fault, reason):
        self._revision += 1
        self._events.append(
            AirspaceUpdateEvent(
                revision=self._revision,
                operation=int(operation),
                zone_id=int(zone_id),
                accepted=bool(accepted),
                fault=bool(fault),
                reason=str(reason),
            )
        )

    def _validate_envelope(self, msg, now_sec):
        if int(msg.schema_version) != self.SCHEMA_VERSION:
            raise NoFlyZoneValidationError("unsupported schema_version")
        operation = int(msg.operation)
        if operation not in (self.OP_UPSERT, self.OP_REMOVE, self.OP_CLEAR):
            raise NoFlyZoneValidationError("unsupported operation")
        frame = str(getattr(msg.header, "frame_id", "") or "")
        if frame != self.config.expected_frame:
            raise NoFlyZoneValidationError(
                f"frame mismatch: expected {self.config.expected_frame}, got {frame or '<empty>'}"
            )
        stamp = _seconds(msg.header.stamp)
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise NoFlyZoneValidationError("header.stamp must be non-zero")
        if stamp > now_sec + self.config.future_stamp_tolerance:
            raise NoFlyZoneValidationError("message stamp is in the future")
        if now_sec - stamp > self.config.max_message_age:
            raise NoFlyZoneValidationError("message is stale")
        return operation

    def _validate_upsert(self, msg, now_sec):
        if int(msg.zone_id) <= 0:
            raise NoFlyZoneValidationError("UPSERT requires non-zero zone_id")
        if int(msg.zone_type) != self.TYPE_NO_FLY:
            raise NoFlyZoneValidationError("only TYPE_NO_FLY is supported")
        min_alt = float(msg.min_altitude)
        max_alt = float(msg.max_altitude)
        if not math.isfinite(min_alt) or not math.isfinite(max_alt):
            raise NoFlyZoneValidationError("altitude bounds must be finite")
        if min_alt >= max_alt:
            raise NoFlyZoneValidationError("min_altitude must be below max_altitude")
        if (
            min_alt < self.config.min_altitude_limit
            or max_alt > self.config.max_altitude_limit
        ):
            raise NoFlyZoneValidationError("altitude bounds exceed configured limits")
        valid_until = _seconds(msg.valid_until)
        stamp = _seconds(msg.header.stamp)
        if not math.isfinite(valid_until) or valid_until <= now_sec:
            raise NoFlyZoneValidationError("valid_until must be in the future")
        if valid_until <= stamp:
            raise NoFlyZoneValidationError("valid_until must be after header.stamp")
        if valid_until - stamp > self.config.max_ttl:
            raise NoFlyZoneValidationError("zone TTL exceeds max_ttl")
        vertices = [
            (float(point.x), float(point.y))
            for point in getattr(msg.polygon, "points", [])
        ]
        _validate_simple_polygon(vertices, self.config)
        return ZoneDef(
            zone_id=int(msg.zone_id),
            enabled=bool(msg.enabled),
            zone_type=int(msg.zone_type),
            level2d=0,
            levelH=0,
            minAlt=min_alt,
            maxAlt=max_alt,
            vertices=vertices,
        ), valid_until

    def accept(self, msg, now_sec: float) -> bool:
        operation = int(getattr(msg, "operation", -1))
        zone_id = int(getattr(msg, "zone_id", 0))
        with self._lock:
            try:
                operation = self._validate_envelope(msg, float(now_sec))
                if operation == self.OP_UPSERT:
                    zone, valid_until = self._validate_upsert(msg, float(now_sec))
                    self.airspace.update_zone(zone)
                    self._expirations[zone.zone_id] = valid_until
                    self._expired_reported.discard(zone.zone_id)
                    reason = "zone_upserted"
                elif operation == self.OP_REMOVE:
                    if zone_id <= 0:
                        raise NoFlyZoneValidationError(
                            "REMOVE requires non-zero zone_id"
                        )
                    self.airspace.remove_zone(zone_id)
                    self._expirations.pop(zone_id, None)
                    self._expired_reported.discard(zone_id)
                    reason = "zone_removed"
                else:
                    if zone_id != 0:
                        raise NoFlyZoneValidationError("CLEAR requires zone_id zero")
                    self.airspace.clear()
                    self._expirations.clear()
                    self._expired_reported.clear()
                    reason = "zones_cleared"
                self._event(operation, zone_id, True, False, reason)
                return True
            except (NoFlyZoneValidationError, TypeError, ValueError, AttributeError) as exc:
                self._event(operation, zone_id, False, True, str(exc))
                return False

    def poll_expirations(self, now_sec: float):
        with self._lock:
            for zone_id, valid_until in list(self._expirations.items()):
                if now_sec > valid_until and zone_id not in self._expired_reported:
                    # Retain the last geometry as a conservative keep-out region.
                    self._expired_reported.add(zone_id)
                    self._event(
                        self.OP_UPSERT,
                        zone_id,
                        False,
                        True,
                        "zone input expired; retaining last geometry",
                    )

    def pop_event(self) -> Optional[AirspaceUpdateEvent]:
        with self._lock:
            return self._events.pop(0) if self._events else None

