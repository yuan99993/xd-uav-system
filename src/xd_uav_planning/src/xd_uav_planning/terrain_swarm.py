"""Dependency-free geometry for fixed-wing terrain-path conflict screening."""

from dataclasses import dataclass
import math


_EPSILON = 1.0e-9


@dataclass(frozen=True)
class ConflictResult:
    """Closest witnesses for one conflicting pair of paths."""

    conflict_type: str
    first_position: tuple
    second_position: tuple
    separation_m: float
    predicted_time_s: float = None


def release_conflict_counts(conflict_types, spatial_only_blocks_release=True):
    """Return blocking and spatial-warning counts for a release policy."""
    if not isinstance(spatial_only_blocks_release, bool):
        raise ValueError("spatial_only_blocks_release must be boolean")
    values = tuple(str(value) for value in conflict_types)
    if any(value not in ("spatial", "spatiotemporal") for value in values):
        raise ValueError("unknown conflict type")
    spatial_warnings = sum(value == "spatial" for value in values)
    blocking = sum(
        value == "spatiotemporal" or spatial_only_blocks_release
        for value in values)
    return blocking, spatial_warnings


def _add(a, b):
    return tuple(a[index] + b[index] for index in range(3))


def _subtract(a, b):
    return tuple(a[index] - b[index] for index in range(3))


def _scale(vector, factor):
    return tuple(value * factor for value in vector)


def _dot(a, b):
    return sum(a[index] * b[index] for index in range(3))


def _norm(vector):
    return math.sqrt(_dot(vector, vector))


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _validate_points(points):
    clean = []
    for point in points:
        if len(point) != 3 or not all(math.isfinite(value) for value in point):
            raise ValueError("path points must be finite three-dimensional values")
        converted = tuple(float(value) for value in point)
        if not clean or _norm(_subtract(converted, clean[-1])) > _EPSILON:
            clean.append(converted)
    if len(clean) < 2:
        raise ValueError("a path must contain at least two distinct points")
    return clean


def path_length(points):
    """Return the three-dimensional length of a validated polyline."""
    clean = _validate_points(points)
    return sum(
        _norm(_subtract(clean[index + 1], clean[index]))
        for index in range(len(clean) - 1))


def _subdivide(points, maximum_segment_length):
    if not math.isfinite(maximum_segment_length) or maximum_segment_length <= 0.0:
        raise ValueError("maximum segment length must be positive and finite")
    output = [points[0]]
    for start, end in zip(points, points[1:]):
        delta = _subtract(end, start)
        pieces = max(1, int(math.ceil(_norm(delta) / maximum_segment_length)))
        for index in range(1, pieces + 1):
            output.append(_add(start, _scale(delta, float(index) / pieces)))
    return output


def _closest_segment_points(first_start, first_end, second_start, second_end):
    # Algorithm adapted from the standard finite-segment closest-point
    # derivation in Real-Time Collision Detection (Ericson, 2005).
    first_delta = _subtract(first_end, first_start)
    second_delta = _subtract(second_end, second_start)
    offset = _subtract(first_start, second_start)
    first_length_squared = _dot(first_delta, first_delta)
    second_length_squared = _dot(second_delta, second_delta)
    second_projection = _dot(second_delta, offset)

    if first_length_squared <= _EPSILON and second_length_squared <= _EPSILON:
        first_fraction = 0.0
        second_fraction = 0.0
    elif first_length_squared <= _EPSILON:
        first_fraction = 0.0
        second_fraction = _clamp(
            second_projection / second_length_squared, 0.0, 1.0)
    else:
        first_projection = _dot(first_delta, offset)
        if second_length_squared <= _EPSILON:
            second_fraction = 0.0
            first_fraction = _clamp(
                -first_projection / first_length_squared, 0.0, 1.0)
        else:
            cross_projection = _dot(first_delta, second_delta)
            denominator = (
                first_length_squared * second_length_squared
                - cross_projection * cross_projection)
            if abs(denominator) > _EPSILON:
                first_fraction = _clamp(
                    (cross_projection * second_projection
                     - first_projection * second_length_squared)
                    / denominator,
                    0.0,
                    1.0)
            else:
                first_fraction = 0.0
            second_fraction = (
                cross_projection * first_fraction + second_projection
            ) / second_length_squared
            if second_fraction < 0.0:
                second_fraction = 0.0
                first_fraction = _clamp(
                    -first_projection / first_length_squared, 0.0, 1.0)
            elif second_fraction > 1.0:
                second_fraction = 1.0
                first_fraction = _clamp(
                    (cross_projection - first_projection)
                    / first_length_squared,
                    0.0,
                    1.0)

    first_point = _add(first_start, _scale(first_delta, first_fraction))
    second_point = _add(second_start, _scale(second_delta, second_fraction))
    return first_point, second_point, _norm(_subtract(first_point, second_point))


def _spatial_closest(first_points, second_points, threshold_m,
                     index_segment_length_m):
    # Subdivision does not approximate or alter either polyline. It only gives
    # a bounded segment length so the finite neighbouring-cell search is safe.
    maximum_length = min(index_segment_length_m, threshold_m * 0.25)
    first = _subdivide(first_points, maximum_length)
    second = _subdivide(second_points, maximum_length)
    cell_size = threshold_m
    index = {}
    for segment_index, (start, end) in enumerate(zip(second, second[1:])):
        midpoint = _scale(_add(start, end), 0.5)
        cell = tuple(int(math.floor(value / cell_size)) for value in midpoint)
        index.setdefault(cell, []).append(segment_index)

    best = None
    for first_start, first_end in zip(first, first[1:]):
        midpoint = _scale(_add(first_start, first_end), 0.5)
        cell = tuple(int(math.floor(value / cell_size)) for value in midpoint)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                for dz in range(-2, 3):
                    for second_index in index.get(
                            (cell[0] + dx, cell[1] + dy, cell[2] + dz), ()):
                        result = _closest_segment_points(
                            first_start, first_end,
                            second[second_index], second[second_index + 1])
                        if best is None or result[2] < best[2]:
                            best = result
                            if result[2] <= _EPSILON:
                                return best
    return best


def _timed_segments(points, speed_mps):
    segments = []
    start_time = 0.0
    for start, end in zip(points, points[1:]):
        delta = _subtract(end, start)
        duration = _norm(delta) / speed_mps
        segments.append((start_time, start_time + duration, start, end))
        start_time += duration
    return segments


def _temporal_closest(first_points, first_speed_mps,
                      second_points, second_speed_mps):
    first_segments = _timed_segments(first_points, first_speed_mps)
    second_segments = _timed_segments(second_points, second_speed_mps)
    first_index = 0
    second_index = 0
    best = None
    while (first_index < len(first_segments)
           and second_index < len(second_segments)):
        first = first_segments[first_index]
        second = second_segments[second_index]
        interval_start = max(first[0], second[0])
        interval_end = min(first[1], second[1])
        if interval_start <= interval_end + _EPSILON:
            first_velocity = _scale(
                _subtract(first[3], first[2]), 1.0 / (first[1] - first[0]))
            second_velocity = _scale(
                _subtract(second[3], second[2]), 1.0 / (second[1] - second[0]))
            first_at_start = _add(
                first[2], _scale(first_velocity, interval_start - first[0]))
            second_at_start = _add(
                second[2], _scale(second_velocity, interval_start - second[0]))
            relative_position = _subtract(first_at_start, second_at_start)
            relative_velocity = _subtract(first_velocity, second_velocity)
            relative_speed_squared = _dot(relative_velocity, relative_velocity)
            offset = 0.0
            if relative_speed_squared > _EPSILON:
                offset = _clamp(
                    -_dot(relative_position, relative_velocity)
                    / relative_speed_squared,
                    0.0,
                    max(0.0, interval_end - interval_start))
            first_position = _add(first_at_start, _scale(first_velocity, offset))
            second_position = _add(
                second_at_start, _scale(second_velocity, offset))
            distance = _norm(_subtract(first_position, second_position))
            candidate = (
                first_position, second_position, distance, interval_start + offset)
            if best is None or distance < best[2]:
                best = candidate

        if first[1] <= second[1] + _EPSILON:
            first_index += 1
        if second[1] <= first[1] + _EPSILON:
            second_index += 1
    return best


def detect_pair_conflict(first_points, first_speed_mps,
                         second_points, second_speed_mps,
                         minimum_separation_m,
                         index_segment_length_m=20.0):
    """Classify one pair as spatial, spatiotemporal, or conflict-free.

    Temporal prediction assumes both paths start at t=0 and each aircraft
    follows its polyline at its configured constant nominal speed.
    """
    if (not math.isfinite(first_speed_mps) or first_speed_mps <= 0.0
            or not math.isfinite(second_speed_mps) or second_speed_mps <= 0.0):
        raise ValueError("nominal speeds must be positive and finite")
    if (not math.isfinite(minimum_separation_m)
            or minimum_separation_m <= 0.0):
        raise ValueError("minimum separation must be positive and finite")

    first = _validate_points(first_points)
    second = _validate_points(second_points)
    spatial = _spatial_closest(
        first, second, minimum_separation_m, index_segment_length_m)
    if spatial is None or spatial[2] >= minimum_separation_m:
        return None

    temporal = _temporal_closest(
        first, first_speed_mps, second, second_speed_mps)
    if temporal is not None and temporal[2] < minimum_separation_m:
        return ConflictResult(
            "spatiotemporal", temporal[0], temporal[1], temporal[2], temporal[3])
    return ConflictResult("spatial", spatial[0], spatial[1], spatial[2])
