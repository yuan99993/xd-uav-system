"""Coverage path generation and workload-balanced scout assignment."""

from dataclasses import dataclass
from math import hypot
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


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


def assign_areas(
    planned_areas: Iterable[PlannedArea],
    scout_positions: Dict[str, Sequence[float]],
) -> Dict[str, List[PlannedArea]]:
    """Greedily balance path workload while accounting for entry distance."""

    if not scout_positions:
        return {}
    result: Dict[str, List[PlannedArea]] = {name: [] for name in scout_positions}
    loads: Dict[str, float] = {name: 0.0 for name in scout_positions}
    for planned in sorted(planned_areas, key=lambda item: (-item.area.priority, -item.length, item.area.area_id)):
        first = planned.path[0]
        scout = min(
            scout_positions,
            key=lambda name: (
                loads[name]
                + hypot(first[0] - float(scout_positions[name][0]), first[1] - float(scout_positions[name][1])),
                name,
            ),
        )
        result[scout].append(planned)
        loads[scout] += planned.length
        loads[scout] += hypot(
            first[0] - float(scout_positions[scout][0]),
            first[1] - float(scout_positions[scout][1]),
        )
    return result
