#!/usr/bin/env python3
"""Pure, time-parameterized path for the single-tank Gazebo benchmark.

Every moving segment uses a quintic time law, giving zero velocity and
acceleration at the ends. The path contains no occlusion phases.
"""

import math


MAX_SPEED_MPS = 3.6
CIRCLE_DURATION_SEC = 15.0
CIRCLE_CENTER = (-13.0, -11.0)
CIRCLE_RADIUS_M = 4.5
START_POINT = (-8.5, -11.0)
BRAVO_OFFSET = (0.0, 16.0)
DECOY_OFFSET = (16.0, 0.0)


def _quintic(value):
    value = max(0.0, min(1.0, value))
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))


def _move_duration(start, end):
    # max(d quintic / dt) = 1.875 * distance / duration.
    return max(0.20, 1.875 * math.hypot(end[0] - start[0], end[1] - start[1]) /
               MAX_SPEED_MPS)


def _build_route():
    segments = []
    cursor = START_POINT
    elapsed = CIRCLE_DURATION_SEC

    def move(end, phase):
        nonlocal cursor, elapsed
        duration = _move_duration(cursor, end)
        segments.append((elapsed, elapsed + duration, cursor, end, phase, False))
        elapsed += duration
        cursor = end

    def hold(duration, phase):
        nonlocal elapsed
        segments.append((elapsed, elapsed + duration, cursor, cursor, phase, False))
        elapsed += duration

    move((-12.0, -4.0), "straight_line")
    move((-12.0, 5.0), "northbound")
    move((-4.0, 5.0), "eastbound")
    move((-4.0, -4.0), "southbound_turn")
    hold(2.0, "stop")
    move((-8.5, -4.0), "westbound")
    move(START_POINT, "return_to_circle")
    return tuple(segments), elapsed


_ROUTE, CYCLE_DURATION_SEC = _build_route()


def alpha(time_sec, offset=(0.0, 0.0)):
    """Return (x, y, phase, expected_occluded) for the unobstructed route."""
    t = float(time_sec) % CYCLE_DURATION_SEC
    ox, oy = offset
    if t < CIRCLE_DURATION_SEC:
        amount = _quintic(t / CIRCLE_DURATION_SEC)
        angle = 2.0 * math.pi * amount
        x = CIRCLE_CENTER[0] + CIRCLE_RADIUS_M * math.cos(angle)
        y = CIRCLE_CENTER[1] + CIRCLE_RADIUS_M * math.sin(angle)
        return x + ox, y + oy, "circle", False

    for start, end, first, second, phase, occluded in _ROUTE:
        if t < end or end == CYCLE_DURATION_SEC:
            amount = 1.0 if end <= start else _quintic((t - start) / (end - start))
            x = first[0] + (second[0] - first[0]) * amount
            y = first[1] + (second[1] - first[1]) * amount
            return x + ox, y + oy, phase, occluded
    return START_POINT[0] + ox, START_POINT[1] + oy, "return_to_circle", False


def bravo(time_sec):
    return alpha(time_sec, BRAVO_OFFSET)[:2] + ("parallel_tank_lane", False)


def decoy(time_sec):
    return alpha(time_sec, DECOY_OFFSET)[:2] + ("parallel_decoy_lane", False)


def heading(trajectory, time_sec, fallback=0.0):
    """Estimate direction from nearby path samples, including stationary holds."""
    center = trajectory(time_sec)
    for delta in (0.05, 0.10, 0.20, 0.40, 0.80):
        after = trajectory(time_sec + delta)
        dx, dy = after[0] - center[0], after[1] - center[1]
        if math.hypot(dx, dy) > 1e-4:
            return math.atan2(dy, dx)
        before = trajectory(time_sec - delta)
        dx, dy = center[0] - before[0], center[1] - before[1]
        if math.hypot(dx, dy) > 1e-4:
            return math.atan2(dy, dx)
    return fallback
