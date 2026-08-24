"""Coverage path generation and workload-balanced scout assignment."""

from dataclasses import dataclass
from math import acos, atan2, ceil, cos, hypot, isfinite, pi, sin, sqrt
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]
Pose2 = Tuple[float, float, float]


@dataclass(frozen=True)
class SearchAreaDefinition:
    area_id: int
    boundary: Tuple[Point2, ...]
    altitude: float
    lane_spacing: float
    priority: int = 0


@dataclass
class PlannedArea:
    area: SearchAreaDefinition
    path: List[Point3]

    @property
    def length(self) -> float:
        return path_length(self.path)


def path_length(path: Sequence[Sequence[float]]) -> float:
    return sum(
        hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        for a, b in zip(path[:-1], path[1:])
    )


def _scan_intersections(polygon: Sequence[Point2], value: float, horizontal: bool) -> List[float]:
    intersections: List[float] = []
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        a_cross = first[1] if horizontal else first[0]
        b_cross = second[1] if horizontal else second[0]
        a_along = first[0] if horizontal else first[1]
        b_along = second[0] if horizontal else second[1]
        # Half-open test avoids double counting polygon vertices.
        if not ((a_cross <= value < b_cross) or (b_cross <= value < a_cross)):
            continue
        ratio = (value - a_cross) / (b_cross - a_cross)
        intersections.append(a_along + ratio * (b_along - a_along))
    return sorted(intersections)


def lawnmower_path(area: SearchAreaDefinition) -> List[Point3]:
    """Generate an alternating scanline path inside a simple polygon."""

    polygon = list(area.boundary)
    if len(polygon) < 3:
        raise ValueError("search area needs at least three boundary vertices")
    spacing = float(area.lane_spacing)
    if spacing <= 0.0:
        raise ValueError("lane_spacing must be positive")

    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    horizontal = (max(xs) - min(xs)) >= (max(ys) - min(ys))
    cross_min, cross_max = (min(ys), max(ys)) if horizontal else (min(xs), max(xs))
    scan = cross_min + min(0.5 * spacing, 0.5 * (cross_max - cross_min))
    segments: List[Tuple[Point3, Point3]] = []
    while scan < cross_max + 1e-9:
        values = _scan_intersections(polygon, scan, horizontal)
        for index in range(0, len(values) - 1, 2):
            low, high = values[index], values[index + 1]
            if high - low <= 1e-6:
                continue
            if horizontal:
                segments.append(((low, scan, area.altitude), (high, scan, area.altitude)))
            else:
                segments.append(((scan, low, area.altitude), (scan, high, area.altitude)))
        scan += spacing

    path: List[Point3] = []
    reverse = False
    for start, end in segments:
        ordered = (end, start) if reverse else (start, end)
        path.extend(ordered)
        reverse = not reverse
    if not path:
        raise ValueError("search area is too small for the requested lane spacing")
    return path


def verification_search_area(
    target_id: int,
    center: Sequence[float],
    covariance: Sequence[float],
    altitude: float,
    lane_spacing: float,
    minimum_radius: float,
    maximum_radius: float,
    covariance_sigma: float = 3.0,
    fixed_radius: float = None,
) -> PlannedArea:
    """Build a local multirotor raster around a coarse target estimate.

    The square half-width follows the largest horizontal covariance axis, with
    explicit minimum and maximum bounds so a bad coarse estimate cannot create
    either a zero-size route or an unbounded diversion.
    """

    if len(center) < 2:
        raise ValueError("verification center needs x and y")
    x, y = float(center[0]), float(center[1])
    if not all(isfinite(value) for value in (x, y, float(altitude))):
        raise ValueError("verification center and altitude must be finite")
    minimum = max(0.1, float(minimum_radius))
    maximum = max(minimum, float(maximum_radius))
    xx = float(covariance[0]) if len(covariance) >= 1 else 0.0
    xy = (
        0.5 * (float(covariance[1]) + float(covariance[3]))
        if len(covariance) >= 4
        else 0.0
    )
    yy = float(covariance[4]) if len(covariance) >= 5 else 0.0
    if not all(isfinite(value) for value in (xx, xy, yy)):
        xx = xy = yy = 0.0
    largest_variance = max(
        0.0,
        0.5 * (xx + yy + sqrt(max(0.0, (xx - yy) ** 2 + 4.0 * xy * xy))),
    )
    if fixed_radius is None:
        radius = min(
            maximum,
            max(minimum, max(0.0, float(covariance_sigma)) * sqrt(largest_variance)),
        )
    else:
        radius = float(fixed_radius)
        if not isfinite(radius) or radius <= 0.0:
            raise ValueError("fixed verification radius must be positive and finite")
    area = SearchAreaDefinition(
        area_id=int(target_id),
        boundary=(
            (x - radius, y - radius),
            (x + radius, y - radius),
            (x + radius, y + radius),
            (x - radius, y + radius),
        ),
        altitude=float(altitude),
        lane_spacing=float(lane_spacing),
        priority=255,
    )
    return PlannedArea(area=area, path=lawnmower_path(area))


def _mod2pi(angle: float) -> float:
    return float(angle) % (2.0 * pi)


def _dubins_candidates(alpha: float, beta: float, distance: float):
    """Return normalized Dubins words as (length, modes, parameters)."""

    sin_alpha, sin_beta = sin(alpha), sin(beta)
    cos_alpha, cos_beta = cos(alpha), cos(beta)
    cos_difference = cos(alpha - beta)
    candidates = []

    def add(modes, first, second_squared, third):
        if second_squared < -1e-9:
            return
        second = sqrt(max(0.0, second_squared))
        parameters = (_mod2pi(first), second, _mod2pi(third))
        candidates.append((sum(parameters), modes, parameters))

    lsl_squared = (
        2.0
        + distance * distance
        - 2.0 * cos_difference
        + 2.0 * distance * (sin_alpha - sin_beta)
    )
    lsl_tmp = atan2(
        cos_beta - cos_alpha,
        distance + sin_alpha - sin_beta,
    )
    add("LSL", -alpha + lsl_tmp, lsl_squared, beta - lsl_tmp)

    rsr_squared = (
        2.0
        + distance * distance
        - 2.0 * cos_difference
        + 2.0 * distance * (sin_beta - sin_alpha)
    )
    rsr_tmp = atan2(
        cos_alpha - cos_beta,
        distance - sin_alpha + sin_beta,
    )
    add("RSR", alpha - rsr_tmp, rsr_squared, -beta + rsr_tmp)

    lsr_squared = (
        -2.0
        + distance * distance
        + 2.0 * cos_difference
        + 2.0 * distance * (sin_alpha + sin_beta)
    )
    if lsr_squared >= -1e-9:
        second = sqrt(max(0.0, lsr_squared))
        tmp = atan2(
            -cos_alpha - cos_beta,
            distance + sin_alpha + sin_beta,
        ) - atan2(-2.0, second)
        parameters = (
            _mod2pi(-alpha + tmp),
            second,
            _mod2pi(-beta + tmp),
        )
        candidates.append((sum(parameters), "LSR", parameters))

    rsl_squared = (
        distance * distance
        - 2.0
        + 2.0 * cos_difference
        - 2.0 * distance * (sin_alpha + sin_beta)
    )
    if rsl_squared >= -1e-9:
        second = sqrt(max(0.0, rsl_squared))
        tmp = atan2(
            cos_alpha + cos_beta,
            distance - sin_alpha - sin_beta,
        ) - atan2(2.0, second)
        parameters = (
            _mod2pi(alpha - tmp),
            second,
            _mod2pi(beta - tmp),
        )
        candidates.append((sum(parameters), "RSL", parameters))

    rlr_tmp = (
        6.0
        - distance * distance
        + 2.0 * cos_difference
        + 2.0 * distance * (sin_alpha - sin_beta)
    ) / 8.0
    if abs(rlr_tmp) <= 1.0 + 1e-9:
        second = _mod2pi(2.0 * pi - acos(max(-1.0, min(1.0, rlr_tmp))))
        first = _mod2pi(
            alpha
            - atan2(
                cos_alpha - cos_beta,
                distance - sin_alpha + sin_beta,
            )
            + 0.5 * second
        )
        third = _mod2pi(alpha - beta - first + second)
        parameters = (first, second, third)
        candidates.append((sum(parameters), "RLR", parameters))

    lrl_tmp = (
        6.0
        - distance * distance
        + 2.0 * cos_difference
        + 2.0 * distance * (-sin_alpha + sin_beta)
    ) / 8.0
    if abs(lrl_tmp) <= 1.0 + 1e-9:
        second = _mod2pi(2.0 * pi - acos(max(-1.0, min(1.0, lrl_tmp))))
        first = _mod2pi(
            -alpha
            - atan2(
                cos_alpha - cos_beta,
                distance + sin_alpha - sin_beta,
            )
            + 0.5 * second
        )
        third = _mod2pi(beta - alpha - first + second)
        parameters = (first, second, third)
        candidates.append((sum(parameters), "LRL", parameters))
    return candidates


def _advance_dubins(pose: Pose2, mode: str, normalized_length: float, radius: float) -> Pose2:
    x, y, heading = pose
    if mode == "S":
        distance = normalized_length * radius
        return (
            x + distance * cos(heading),
            y + distance * sin(heading),
            heading,
        )
    if mode == "L":
        center_x = x - radius * sin(heading)
        center_y = y + radius * cos(heading)
        new_heading = heading + normalized_length
        return (
            center_x + radius * sin(new_heading),
            center_y - radius * cos(new_heading),
            new_heading,
        )
    center_x = x + radius * sin(heading)
    center_y = y - radius * cos(heading)
    new_heading = heading - normalized_length
    return (
        center_x - radius * sin(new_heading),
        center_y + radius * cos(new_heading),
        new_heading,
    )


def dubins_path(
    start: Pose2,
    goal: Pose2,
    minimum_turn_radius: float,
    waypoint_spacing: float,
) -> List[Pose2]:
    """Sample the shortest forward-only Dubins path between two planar poses."""

    radius = float(minimum_turn_radius)
    spacing = float(waypoint_spacing)
    if radius <= 0.0:
        raise ValueError("minimum_turn_radius must be positive")
    if spacing <= 0.0:
        raise ValueError("waypoint_spacing must be positive")
    dx = float(goal[0]) - float(start[0])
    dy = float(goal[1]) - float(start[1])
    separation = hypot(dx, dy)
    if separation <= 1e-9 and abs(_mod2pi(goal[2] - start[2])) <= 1e-9:
        return [start, goal]
    reference_heading = atan2(dy, dx) if separation > 1e-9 else float(start[2])
    alpha = _mod2pi(float(start[2]) - reference_heading)
    beta = _mod2pi(float(goal[2]) - reference_heading)
    candidates = _dubins_candidates(alpha, beta, separation / radius)
    if not candidates:
        raise ValueError("no feasible Dubins connection was found")
    _, modes, parameters = min(candidates, key=lambda item: item[0])

    samples: List[Pose2] = [(float(start[0]), float(start[1]), float(start[2]))]
    pose = samples[0]
    for mode, parameter in zip(modes, parameters):
        segment_distance = parameter * radius
        if segment_distance <= 1e-9:
            continue
        steps = max(1, int(ceil(segment_distance / spacing)))
        previous = 0.0
        for step in range(1, steps + 1):
            current = parameter * float(step) / float(steps)
            pose = _advance_dubins(pose, mode, current - previous, radius)
            samples.append(pose)
            previous = current
    position_error = hypot(samples[-1][0] - goal[0], samples[-1][1] - goal[1])
    heading_error = abs((_mod2pi(samples[-1][2] - goal[2] + pi)) - pi)
    if position_error > max(1e-5, 1e-7 * radius) or heading_error > 1e-7:
        raise RuntimeError("Dubins integration did not reach the requested goal pose")
    # Analytic formulas and incremental sampling can accumulate small floating
    # point error. Preserve the exact requested endpoint for downstream goals.
    samples[-1] = (float(goal[0]), float(goal[1]), float(goal[2]))
    return samples


def fixedwing_lawnmower_path(
    area: SearchAreaDefinition,
    minimum_turn_radius: float,
    turn_waypoint_spacing: float,
    straight_lead_distance: Optional[float] = None,
) -> List[Point3]:
    """Generate fixed-wing coverage legs with the turns outside the polygon.

    A conventional lawnmower visits adjacent lanes in alternating directions.
    That is a poor fixed-wing path when the lane spacing is much smaller than
    the turn diameter: the shortest Dubins connection is then an RLR/LRL loop.
    Visit the lower and upper halves of the scan lanes alternately instead, so
    successive reversal points are spread as far apart as the area permits.

    Each polygon chord is also extended by a straight lead-in and lead-out.
    Consequently the aircraft has already rolled out before it crosses the
    search boundary and the complete in-polygon chord is a level scan leg.
    Coverage still uses every chord produced by :func:`lawnmower_path`; only
    their visit order and the out-of-polygon transit are changed.
    """

    raw = lawnmower_path(area)
    if len(raw) < 2:
        return raw
    radius = float(minimum_turn_radius)
    lead_distance = radius if straight_lead_distance is None else float(
        straight_lead_distance
    )
    if radius <= 0.0:
        raise ValueError("minimum_turn_radius must be positive")
    if lead_distance < 0.0:
        raise ValueError("straight_lead_distance must be non-negative")

    xs = [point[0] for point in area.boundary]
    ys = [point[1] for point in area.boundary]
    horizontal = (max(xs) - min(xs)) >= (max(ys) - min(ys))

    # Recover the natural low-to-high direction of every scan chord.  The raw
    # multirotor route has already reversed every other chord.
    segments = []
    for first, second in zip(raw[0::2], raw[1::2]):
        first_along = first[0] if horizontal else first[1]
        second_along = second[0] if horizontal else second[1]
        segments.append(
            (first, second) if first_along <= second_along else (second, first)
        )

    # This interleaving is an anti-bandwidth ordering for uniformly spaced
    # lanes.  For 20 lanes it produces 0,10,1,11,... rather than 0,1,2,...,
    # eliminating adjacent-lane reversals whenever the area is wide enough.
    half = (len(segments) + 1) // 2
    visit_order: List[int] = []
    for index in range(half):
        visit_order.append(index)
        upper = index + half
        if upper < len(segments):
            visit_order.append(upper)

    route: List[Point3] = []
    previous_end = None
    previous_heading = 0.0
    for visit_index, segment_index in enumerate(visit_order):
        low, high = segments[segment_index]
        start, end = (low, high) if visit_index % 2 == 0 else (high, low)
        heading = atan2(end[1] - start[1], end[0] - start[0])
        leg_length = hypot(end[0] - start[0], end[1] - start[1])
        direction_x = (end[0] - start[0]) / leg_length
        direction_y = (end[1] - start[1]) / leg_length
        approach = (
            start[0] - lead_distance * direction_x,
            start[1] - lead_distance * direction_y,
            area.altitude,
        )
        departure = (
            end[0] + lead_distance * direction_x,
            end[1] + lead_distance * direction_y,
            area.altitude,
        )
        if previous_end is None:
            route.append(approach)
        else:
            connector = dubins_path(
                (previous_end[0], previous_end[1], previous_heading),
                (approach[0], approach[1], heading),
                radius,
                turn_waypoint_spacing,
            )
            route.extend((pose[0], pose[1], area.altitude) for pose in connector[1:])
        # These three collinear points explicitly delimit the covered chord.
        # The trajectory derivative therefore remains tangent to the scan leg
        # at both polygon crossings instead of starting a bank at the boundary.
        route.extend((start, end, departure))
        previous_end = departure
        previous_heading = heading
    return route


def connect_fixedwing_paths(
    paths: Iterable[Sequence[Point3]],
    minimum_turn_radius: float,
    turn_waypoint_spacing: float,
) -> List[Point3]:
    """Join independently planned fixed-wing paths without discontinuous turns."""

    combined: List[Point3] = []
    for path_value in paths:
        path = list(path_value)
        if not path:
            continue
        if not combined:
            combined.extend(path)
            continue
        if len(combined) < 2 or len(path) < 2:
            combined.extend(path)
            continue
        previous_heading = atan2(
            combined[-1][1] - combined[-2][1],
            combined[-1][0] - combined[-2][0],
        )
        next_heading = atan2(
            path[1][1] - path[0][1],
            path[1][0] - path[0][0],
        )
        connector = dubins_path(
            (combined[-1][0], combined[-1][1], previous_heading),
            (path[0][0], path[0][1], next_heading),
            minimum_turn_radius,
            turn_waypoint_spacing,
        )
        count = max(1, len(connector) - 1)
        start_altitude = float(combined[-1][2])
        altitude_change = float(path[0][2]) - start_altitude
        for index, pose in enumerate(connector[1:], start=1):
            ratio = float(index) / float(count)
            combined.append(
                (
                    pose[0],
                    pose[1],
                    start_altitude + ratio * altitude_change,
                )
            )
        combined.extend(path[1:])
    return combined


def fixedwing_entry_path(
    current: Point3,
    current_heading: float,
    route: Sequence[Point3],
    minimum_turn_radius: float,
    turn_waypoint_spacing: float,
) -> List[Point3]:
    """Add a turn-constrained entry from the aircraft pose to a planned route."""

    planned = list(route)
    if len(planned) < 2:
        return planned
    route_heading = atan2(
        planned[1][1] - planned[0][1],
        planned[1][0] - planned[0][0],
    )
    connector = dubins_path(
        (float(current[0]), float(current[1]), float(current_heading)),
        (planned[0][0], planned[0][1], route_heading),
        minimum_turn_radius,
        turn_waypoint_spacing,
    )
    count = max(1, len(connector) - 1)
    start_altitude = float(current[2])
    altitude_change = float(planned[0][2]) - start_altitude
    entry = [
        (
            pose[0],
            pose[1],
            start_altitude + float(index) / float(count) * altitude_change,
        )
        for index, pose in enumerate(connector[1:], start=1)
    ]
    entry.extend(planned[1:])
    return entry


def assign_areas(
    planned_areas: Iterable[PlannedArea],
    scout_positions: Dict[str, Sequence[float]],
    scout_speeds: Optional[Dict[str, float]] = None,
    vehicle_area_lengths: Optional[Dict[Tuple[str, int], float]] = None,
) -> Dict[str, List[PlannedArea]]:
    """Greedily balance estimated flight time and area-entry travel."""

    if not scout_positions:
        return {}
    result: Dict[str, List[PlannedArea]] = {name: [] for name in scout_positions}
    loads: Dict[str, float] = {name: 0.0 for name in scout_positions}
    current_positions = {
        name: (float(position[0]), float(position[1]))
        for name, position in scout_positions.items()
    }
    speeds = {
        name: max(0.1, float((scout_speeds or {}).get(name, 1.0)))
        for name in scout_positions
    }
    lengths = vehicle_area_lengths or {}
    for planned in sorted(planned_areas, key=lambda item: (-item.area.priority, -item.length, item.area.area_id)):
        first = planned.path[0]
        scout = min(
            scout_positions,
            key=lambda name: (
                loads[name]
                + hypot(
                    first[0] - current_positions[name][0],
                    first[1] - current_positions[name][1],
                ) / speeds[name],
                name,
            ),
        )
        result[scout].append(planned)
        entry_distance = hypot(
            first[0] - current_positions[scout][0],
            first[1] - current_positions[scout][1],
        )
        area_length = float(
            lengths.get((scout, planned.area.area_id), planned.length)
        )
        loads[scout] += (entry_distance + area_length) / speeds[scout]
        last = planned.path[-1]
        current_positions[scout] = (float(last[0]), float(last[1]))
    return result
