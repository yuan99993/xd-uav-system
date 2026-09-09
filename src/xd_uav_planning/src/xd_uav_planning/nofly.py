"""ROS-independent fixed-wing no-fly-zone validation and path adjustment."""

from dataclasses import dataclass
import math
import threading

import dubins


class NoFlyZoneError(ValueError):
    pass


class NoSafePathError(RuntimeError):
    pass


@dataclass(frozen=True)
class NoFlyConfig:
    expected_frame: str
    max_message_age: float = 1.0
    future_tolerance: float = 0.05
    max_ttl: float = 3600.0
    min_altitude_limit: float = -1000.0
    max_altitude_limit: float = 10000.0
    max_abs_coordinate: float = 100000.0
    min_polygon_area: float = 1.0
    max_vertices: int = 64


@dataclass(frozen=True)
class Zone:
    zone_id: int
    enabled: bool
    min_altitude: float
    max_altitude: float
    vertices: tuple
    valid_until: float = 0.0


def _seconds(value):
    if hasattr(value, "to_sec"):
        return float(value.to_sec())
    if hasattr(value, "secs"):
        return float(value.secs) + float(getattr(value, "nsecs", 0)) * 1e-9
    return float(value)


def _orientation(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) -
            (b[1] - a[1]) * (c[0] - a[0]))


def _on_segment(a, b, point, epsilon=1e-9):
    return (abs(_orientation(a, b, point)) <= epsilon and
            min(a[0], b[0]) - epsilon <= point[0] <= max(a[0], b[0]) + epsilon and
            min(a[1], b[1]) - epsilon <= point[1] <= max(a[1], b[1]) + epsilon)


def _segments_intersect(a, b, c, d, epsilon=1e-9):
    o1 = _orientation(a, b, c)
    o2 = _orientation(a, b, d)
    o3 = _orientation(c, d, a)
    o4 = _orientation(c, d, b)
    if (((o1 > epsilon and o2 < -epsilon) or (o1 < -epsilon and o2 > epsilon)) and
            ((o3 > epsilon and o4 < -epsilon) or
             (o3 < -epsilon and o4 > epsilon))):
        return True
    return (_on_segment(a, b, c, epsilon) or _on_segment(a, b, d, epsilon) or
            _on_segment(c, d, a, epsilon) or _on_segment(c, d, b, epsilon))


def _polygon_area(vertices):
    return 0.5 * abs(sum(
        vertices[index][0] * vertices[(index + 1) % len(vertices)][1] -
        vertices[(index + 1) % len(vertices)][0] * vertices[index][1]
        for index in range(len(vertices))))


def _validate_polygon(vertices, config):
    if len(vertices) < 3:
        raise NoFlyZoneError("polygon requires at least three vertices")
    if len(vertices) > config.max_vertices:
        raise NoFlyZoneError("polygon exceeds max_vertices")
    for index, point in enumerate(vertices):
        if not all(math.isfinite(value) for value in point):
            raise NoFlyZoneError("polygon contains non-finite vertex")
        if max(abs(point[0]), abs(point[1])) > config.max_abs_coordinate:
            raise NoFlyZoneError("polygon vertex exceeds coordinate limit")
        nxt = vertices[(index + 1) % len(vertices)]
        if math.hypot(nxt[0] - point[0], nxt[1] - point[1]) <= 1e-6:
            raise NoFlyZoneError("polygon has duplicate adjacent vertices")
    if _polygon_area(vertices) < config.min_polygon_area:
        raise NoFlyZoneError("polygon area is below minimum")
    for first in range(len(vertices)):
        a = vertices[first]
        b = vertices[(first + 1) % len(vertices)]
        for second in range(first + 1, len(vertices)):
            if second in (first, (first + 1) % len(vertices)) or (second + 1) % len(vertices) == first:
                continue
            if _segments_intersect(a, b, vertices[second],
                                   vertices[(second + 1) % len(vertices)]):
                raise NoFlyZoneError("polygon self-intersects")


class NoFlyZoneStore:
    """Thread-safe validated zone cache used by initial and online planning."""

    SCHEMA_VERSION = 1
    OP_UPSERT = 0
    OP_REMOVE = 1
    OP_CLEAR = 2
    TYPE_NO_FLY = 0

    def __init__(self, config):
        if not config.expected_frame:
            raise ValueError("expected_frame must be non-empty")
        self.config = config
        self._zones = {}
        self._lock = threading.RLock()
        self.revision = 0
        self.last_reason = "no_zone_received"

    def accept(self, message, now):
        try:
            self._accept(message, float(now))
            return True
        except (AttributeError, TypeError, ValueError, NoFlyZoneError) as error:
            self.last_reason = str(error)
            return False

    def _accept(self, message, now):
        if int(message.schema_version) != self.SCHEMA_VERSION:
            raise NoFlyZoneError("unsupported schema_version")
        operation = int(message.operation)
        if operation not in (self.OP_UPSERT, self.OP_REMOVE, self.OP_CLEAR):
            raise NoFlyZoneError("unsupported operation")
        frame = str(message.header.frame_id or "").strip("/")
        if frame != self.config.expected_frame.strip("/"):
            raise NoFlyZoneError("zone_frame_mismatch")
        stamp = _seconds(message.header.stamp)
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise NoFlyZoneError("zone_stamp_not_positive")
        if stamp > now + self.config.future_tolerance:
            raise NoFlyZoneError("zone_stamp_in_future")
        if now - stamp > self.config.max_message_age:
            raise NoFlyZoneError("zone_message_stale")
        zone_id = int(message.zone_id)
        with self._lock:
            if operation == self.OP_CLEAR:
                if zone_id != 0:
                    raise NoFlyZoneError("CLEAR requires zone_id zero")
                self._zones.clear()
                self.last_reason = "zones_cleared"
            elif operation == self.OP_REMOVE:
                if zone_id <= 0:
                    raise NoFlyZoneError("REMOVE requires non-zero zone_id")
                self._zones.pop(zone_id, None)
                self.last_reason = "zone_removed"
            else:
                if zone_id <= 0:
                    raise NoFlyZoneError("UPSERT requires non-zero zone_id")
                if int(message.zone_type) != self.TYPE_NO_FLY:
                    raise NoFlyZoneError("unsupported zone_type")
                minimum = float(message.min_altitude)
                maximum = float(message.max_altitude)
                if not all(math.isfinite(value) for value in (minimum, maximum)):
                    raise NoFlyZoneError("altitude_not_finite")
                if minimum >= maximum:
                    raise NoFlyZoneError("invalid_altitude_bounds")
                if (minimum < self.config.min_altitude_limit or
                        maximum > self.config.max_altitude_limit):
                    raise NoFlyZoneError("altitude_exceeds_limit")
                valid_until = _seconds(message.valid_until)
                if not math.isfinite(valid_until) or valid_until < 0.0:
                    raise NoFlyZoneError("invalid_valid_until")
                if valid_until:
                    if valid_until <= now or valid_until <= stamp:
                        raise NoFlyZoneError("zone_already_expired")
                    if valid_until - stamp > self.config.max_ttl:
                        raise NoFlyZoneError("zone_ttl_exceeds_limit")
                vertices = tuple((float(point.x), float(point.y))
                                 for point in message.polygon.points)
                _validate_polygon(vertices, self.config)
                self._zones[zone_id] = Zone(
                    zone_id, bool(message.enabled), minimum, maximum,
                    vertices, valid_until)
                self.last_reason = "zone_upserted"
            self.revision += 1

    def snapshot(self):
        # Expired input is retained conservatively until REMOVE/CLEAR/UPSERT.
        with self._lock:
            return tuple(zone for zone in self._zones.values() if zone.enabled)


def remaining_path(points, current_position, minimum_progress=0.0):
    """Return the original route remaining after the nearest forward projection.

    ``minimum_progress`` is distance along the original polyline and prevents a
    self-crossing route or an off-route detour from moving task progress
    backwards when several dynamic zone updates arrive.
    """
    route = [tuple(float(value) for value in point[:3]) for point in points]
    current = tuple(float(value) for value in current_position[:3])
    if len(route) < 2:
        raise ValueError("route requires at least two points")
    if not all(math.isfinite(value) for point in route for value in point):
        raise ValueError("route contains non-finite point")
    if not all(math.isfinite(value) for value in current):
        raise ValueError("current position is not finite")

    cumulative = [0.0]
    for start, end in zip(route, route[1:]):
        cumulative.append(cumulative[-1] + math.sqrt(sum(
            (end[axis] - start[axis]) ** 2 for axis in range(3))))
    floor = max(0.0, min(float(minimum_progress), cumulative[-1]))
    best = None
    for index, (start, end) in enumerate(zip(route, route[1:])):
        segment_length = cumulative[index + 1] - cumulative[index]
        if segment_length <= 1e-9 or cumulative[index + 1] + 1e-9 < floor:
            continue
        vector = tuple(end[axis] - start[axis] for axis in range(3))
        ratio = sum((current[axis] - start[axis]) * vector[axis]
                    for axis in range(3)) / (segment_length * segment_length)
        minimum_ratio = max(0.0, (floor - cumulative[index]) / segment_length)
        ratio = max(minimum_ratio, min(1.0, ratio))
        projected = tuple(start[axis] + ratio * vector[axis]
                          for axis in range(3))
        distance_squared = sum((current[axis] - projected[axis]) ** 2
                               for axis in range(3))
        progress = cumulative[index] + ratio * segment_length
        candidate = (distance_squared, progress, index)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
    if best is None:
        return [current, route[-1]], cumulative[-1]

    _, progress, segment_index = best
    result = [current]
    for point in route[segment_index + 1:]:
        if math.sqrt(sum((point[axis] - result[-1][axis]) ** 2
                         for axis in range(3))) > 1e-6:
            result.append(point)
    if len(result) < 2:
        result.append(route[-1])
    return result, progress


def point_in_polygon(x, y, polygon):
    if len(polygon) < 3:
        return False
    for index, a in enumerate(polygon):
        b = polygon[(index + 1) % len(polygon)]
        if _on_segment(a, b, (x, y)):
            return True
    inside = False
    previous = len(polygon) - 1
    for index, point in enumerate(polygon):
        old = polygon[previous]
        if ((point[1] > y) != (old[1] > y) and
                x < (old[0] - point[0]) * (y - point[1]) /
                (old[1] - point[1]) + point[0]):
            inside = not inside
        previous = index
    return inside


def point_zone_clearance(x, y, zone):
    if point_in_polygon(x, y, zone.vertices):
        return 0.0
    best = float("inf")
    for index, a in enumerate(zone.vertices):
        b = zone.vertices[(index + 1) % len(zone.vertices)]
        vx, vy = b[0] - a[0], b[1] - a[1]
        denominator = vx * vx + vy * vy
        scale = 0.0 if denominator <= 1e-12 else max(
            0.0, min(1.0, ((x - a[0]) * vx + (y - a[1]) * vy) / denominator))
        best = min(best, math.hypot(x - (a[0] + scale * vx),
                                    y - (a[1] + scale * vy)))
    return best


def _zone_overlaps_altitudes(zone, first, second):
    low, high = sorted((float(first), float(second)))
    return high >= zone.min_altitude and low <= zone.max_altitude


def path_is_clear(points, zones, clearance, sample_step):
    """Check the actual piecewise-linear path, including between Path poses."""
    if len(points) < 2:
        return False
    step = max(0.1, float(sample_step))
    required = max(0.0, float(clearance))
    for start, end in zip(points, points[1:]):
        length = math.sqrt(sum((end[i] - start[i]) ** 2 for i in range(3)))
        count = max(1, int(math.ceil(length / step)))
        active = [zone for zone in zones
                  if _zone_overlaps_altitudes(zone, start[2], end[2])]
        for index in range(count + 1):
            ratio = float(index) / count
            point = tuple(start[axis] + ratio * (end[axis] - start[axis])
                          for axis in range(3))
            for zone in active:
                if zone.min_altitude <= point[2] <= zone.max_altitude:
                    distance = point_zone_clearance(point[0], point[1], zone)
                    if distance < required or (required == 0.0 and distance == 0.0):
                        return False
    return True


def path_min_clearance(points, zones, sample_step=1.0):
    best = float("inf")
    if len(points) < 2:
        return best
    step = max(0.1, float(sample_step))
    for start, end in zip(points, points[1:]):
        length = math.sqrt(sum((end[i] - start[i]) ** 2 for i in range(3)))
        count = max(1, int(math.ceil(length / step)))
        for index in range(count + 1):
            ratio = float(index) / count
            x = start[0] + ratio * (end[0] - start[0])
            y = start[1] + ratio * (end[1] - start[1])
            z = start[2] + ratio * (end[2] - start[2])
            for zone in zones:
                if zone.min_altitude <= z <= zone.max_altitude:
                    best = min(best, point_zone_clearance(x, y, zone))
    return best


def _dubins_samples(start, goal, radius, step):
    try:
        path = dubins.shortest_path(tuple(start), tuple(goal), radius)
        samples, _ = path.sample_many(step)
    except Exception:
        return []
    result = [tuple(float(value) for value in sample[:3]) for sample in samples]
    if not result or math.hypot(result[-1][0] - goal[0],
                                result[-1][1] - goal[1]) > 1e-6:
        result.append(tuple(goal))
    return result


def _samples_are_clear(samples, zones, clearance):
    return all(point_zone_clearance(point[0], point[1], zone) >= clearance
               for point in samples for zone in zones)


def _expanded_vertices(zone, margins):
    center_x = sum(point[0] for point in zone.vertices) / len(zone.vertices)
    center_y = sum(point[1] for point in zone.vertices) / len(zone.vertices)
    result = []
    for margin in margins:
        for x, y in zone.vertices:
            dx, dy = x - center_x, y - center_y
            norm = math.hypot(dx, dy)
            if norm > 1e-9:
                result.append((x + dx * margin / norm,
                               y + dy * margin / norm))
    return result


def plan_dubins_segment(start, goal, radius, zones, clearance, sample_step):
    """Heuristic curvature-constrained detour with fail-closed validation."""
    effective = max(0.0, float(clearance)) + float(sample_step)
    direct = _dubins_samples(start, goal, radius, sample_step)
    if direct and _samples_are_clear(direct, zones, effective):
        return direct
    current = tuple(start)
    waypoints = [current]
    for _ in range(20):
        direct = _dubins_samples(current, goal, radius, sample_step)
        if direct and _samples_are_clear(direct, zones, effective):
            waypoints.append(tuple(goal))
            break
        blocked = []
        for zone in zones:
            samples = _dubins_samples(current, goal, radius, sample_step)
            if not samples or not _samples_are_clear(samples, (zone,), effective):
                center = (sum(p[0] for p in zone.vertices) / len(zone.vertices),
                          sum(p[1] for p in zone.vertices) / len(zone.vertices))
                blocked.append((math.hypot(current[0] - center[0],
                                           current[1] - center[1]), zone))
        found = None
        for _, zone in sorted(blocked, key=lambda item: item[0])[:3]:
            candidates = _expanded_vertices(
                zone, (effective, effective + 0.5 * radius,
                       effective + radius, effective + 2.0 * radius))
            options = []
            goal_angle = math.atan2(goal[1] - current[1], goal[0] - current[0])
            for x, y in candidates:
                if any(point_zone_clearance(x, y, other) < effective
                       for other in zones):
                    continue
                travel_angle = math.atan2(y - current[1], x - current[0])
                headings = (travel_angle, math.atan2(goal[1] - y, goal[0] - x),
                            *(index * math.pi / 4.0 for index in range(-4, 4)))
                for heading in headings:
                    waypoint = (x, y, heading)
                    incoming = _dubins_samples(current, waypoint, radius, sample_step)
                    if not incoming or not _samples_are_clear(incoming, zones, effective):
                        continue
                    outgoing = _dubins_samples(waypoint, goal, radius, sample_step)
                    safe_outgoing = outgoing and _samples_are_clear(
                        outgoing, zones, effective)
                    progress = ((x - current[0]) * math.cos(goal_angle) +
                                (y - current[1]) * math.sin(goal_angle))
                    if progress < max(1.0, 0.2 * radius):
                        continue
                    cost = (dubins.shortest_path(current, waypoint, radius).path_length() +
                            dubins.shortest_path(waypoint, goal, radius).path_length())
                    options.append((0 if safe_outgoing else 1, cost, waypoint))
            if options:
                found = min(options, key=lambda item: (item[0], item[1]))[2]
                break
        if found is None or any(math.hypot(found[0] - old[0], found[1] - old[1]) < 1.0
                                for old in waypoints):
            return []
        waypoints.append(found)
        current = found
    else:
        return []
    result = []
    for index in range(len(waypoints) - 1):
        segment = _dubins_samples(waypoints[index], waypoints[index + 1],
                                  radius, sample_step)
        if not segment:
            return []
        result.extend(segment if not result else segment[1:])
    if len(result) < 2 or not _samples_are_clear(result, zones, effective):
        return []
    return result


def adjust_fixedwing_path(points, current_position, current_course, zones,
                          turning_radius, clearance, sample_step):
    """Return ``(points, adjusted)`` for an initial or remaining task Path."""
    original = [tuple(float(value) for value in point[:3]) for point in points]
    if not zones or path_is_clear(original, zones, clearance, sample_step):
        return original, False
    if not all(math.isfinite(value) for value in tuple(current_position) +
               (current_course, turning_radius, clearance, sample_step)):
        raise NoSafePathError("planning state is not finite")
    if turning_radius <= 0.0 or sample_step <= 0.0:
        raise NoSafePathError("invalid fixed-wing planning parameters")
    targets = []
    for index, point in enumerate(original):
        active_at_point = [
            zone for zone in zones
            if zone.min_altitude <= point[2] <= zone.max_altitude
        ]
        point_safe = all(
            point_zone_clearance(point[0], point[1], zone) >= clearance
            for zone in active_at_point)
        if index == len(original) - 1 and not point_safe:
            raise NoSafePathError("task destination is inside no-fly clearance")
        # Path poses describe route geometry, not mandatory task events.  An
        # interior sample covered by a newly supplied keep-out polygon must be
        # bypassed rather than treated as an unreachable Dubins endpoint.
        if point_safe or index == len(original) - 1:
            targets.append(point)
    if math.sqrt(sum((targets[0][axis] - current_position[axis]) ** 2
                     for axis in range(3))) < sample_step:
        targets.pop(0)
    if not targets:
        raise NoSafePathError("task path has no target beyond current position")
    current = (float(current_position[0]), float(current_position[1]),
               float(current_course))
    current_altitude = float(current_position[2])
    adjusted = [(current[0], current[1], current_altitude)]
    for index, target in enumerate(targets):
        if index + 1 < len(targets):
            following = targets[index + 1]
            goal_heading = math.atan2(following[1] - target[1],
                                      following[0] - target[0])
        else:
            goal_heading = math.atan2(target[1] - current[1],
                                      target[0] - current[0])
        active = tuple(zone for zone in zones if _zone_overlaps_altitudes(
            zone, current_altitude, target[2]))
        segment = plan_dubins_segment(
            current, (target[0], target[1], goal_heading), turning_radius,
            active, clearance, sample_step)
        if len(segment) < 2:
            raise NoSafePathError("no safe turn-constrained bypass")
        lengths = [0.0]
        for first, second in zip(segment, segment[1:]):
            lengths.append(lengths[-1] + math.hypot(
                second[0] - first[0], second[1] - first[1]))
        total = max(lengths[-1], 1e-9)
        for sample, distance in zip(segment[1:], lengths[1:]):
            altitude = current_altitude + (target[2] - current_altitude) * distance / total
            adjusted.append((sample[0], sample[1], altitude))
        current = (target[0], target[1], goal_heading)
        current_altitude = target[2]
    if not path_is_clear(adjusted, zones, clearance, sample_step):
        raise NoSafePathError("final adjusted path failed continuous clearance check")
    return adjusted, True
