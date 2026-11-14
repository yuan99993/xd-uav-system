"""Small, ROS-independent geometry and preemption policy for formation."""

import math


def validate_offset(offset, minimum_distance):
    if not math.isfinite(minimum_distance) or minimum_distance <= 0:
        raise ValueError("minimum distance must be positive and finite")
    if len(offset) != 3:
        raise ValueError("offset must have three values: forward, left, up")
    value = tuple(float(component) for component in offset)
    if not all(math.isfinite(component) for component in value):
        raise ValueError("offset must be finite")
    if math.sqrt(sum(component * component for component in value)) < minimum_distance:
        raise ValueError("offset is shorter than minimum separation")
    return value


def formation_offsets(shape, follower_count, spacing):
    """Return leader-frame slots following swiftwing's circle/grid layouts.

    The original circle radius scales with team size.  We also keep every
    slot at least one spacing unit from the leader, including a one-follower
    team where the original formula collapses to an unsafe small radius.
    """
    count = int(follower_count)
    gap = float(spacing)
    if count < 1 or not math.isfinite(gap) or gap <= 0:
        raise ValueError("follower_count and spacing must be positive")
    if shape == "circle":
        radius = max(gap, gap * count / (2.0 * math.pi))
        return tuple((radius * math.sin(2.0 * math.pi * i / count),
                      radius * math.cos(2.0 * math.pi * i / count), 0.0)
                     for i in range(count))
    if shape == "rectangle_compact":
        columns = max(1, int(math.sqrt(count + 1)))
        rows = int(math.ceil((count + 1) / columns))
        slots = []
        for column in range(columns):
            for row in range(rows):
                if column == 0 and row == 0:
                    continue
                slots.append((column * gap, -row * gap, 0.0))
                if len(slots) == count:
                    return tuple(slots)
    raise ValueError("shape must be circle or rectangle_compact")


def slot_target(leader_position, leader_yaw, offset, yaw_offset=0.0):
    """Return a follower slot in the leader's position frame (ENU axes)."""
    if not all(math.isfinite(float(value)) for value in
               (*leader_position, leader_yaw, *offset, yaw_offset)):
        raise ValueError("non-finite formation geometry")
    forward, left, up = offset
    cosine, sine = math.cos(leader_yaw), math.sin(leader_yaw)
    return (
        leader_position[0] + cosine * forward - sine * left,
        leader_position[1] + sine * forward + cosine * left,
        leader_position[2] + up,
        math.atan2(math.sin(leader_yaw + yaw_offset),
                   math.cos(leader_yaw + yaw_offset)),
    )


def slot_velocity(leader_velocity, leader_acceleration, heading, offset,
                  max_turn_rate=None, turn_rate=None):
    """Differentiate the moving slot: v_slot = v_leader + omega x R*offset.

    Ground-track curvature is measured from velocity and acceleration in
    the common frame. Unlike commanded bank angle, its sign is unambiguous
    in ENU coordinates. A turn-rate cap is optional.
    """
    values = (*leader_velocity, *leader_acceleration, heading, *offset)
    if max_turn_rate is not None:
        values += (max_turn_rate,)
    if turn_rate is not None:
        values += (turn_rate,)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("non-finite formation velocity")
    if max_turn_rate is not None and max_turn_rate <= 0:
        raise ValueError("max_turn_rate must be positive")
    vx, vy, vz = leader_velocity
    ax, ay, _ = leader_acceleration
    speed_squared = vx * vx + vy * vy
    if turn_rate is None:
        turn_rate = ((vx * ay - vy * ax) / speed_squared
                     if speed_squared > 1e-6 else 0.0)
    if max_turn_rate is not None:
        turn_rate = max(-max_turn_rate, min(max_turn_rate, turn_rate))
    forward, left, _ = offset
    cosine, sine = math.cos(heading), math.sin(heading)
    rx = cosine * forward - sine * left
    ry = sine * forward + cosine * left
    return (vx - turn_rate * ry, vy + turn_rate * rx, vz)


def catchup_velocity(base_velocity, slot_position, own_position, gain,
                     max_correction=None, max_horizontal_speed=None,
                     min_horizontal_speed=0.0):
    """Add slot-error velocity, with optional correction and speed bounds.

    A lagging aircraft asks for more speed; one ahead of its slot slows down.
    Altitude feedback remains with the flight controller.
    """
    values = (*base_velocity, *slot_position, *own_position, gain,
              min_horizontal_speed)
    if max_correction is not None:
        values += (max_correction,)
    if max_horizontal_speed is not None:
        values += (max_horizontal_speed,)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("non-finite catchup input")
    if (gain < 0 or (max_correction is not None and max_correction < 0) or
            (max_horizontal_speed is not None and
             max_horizontal_speed <= 0) or min_horizontal_speed < 0 or
            (max_horizontal_speed is not None and
             min_horizontal_speed > max_horizontal_speed)):
        raise ValueError("invalid catchup speed limits")
    error_x = slot_position[0] - own_position[0]
    error_y = slot_position[1] - own_position[1]
    correction_x = gain * error_x
    correction_y = gain * error_y
    correction_norm = math.hypot(correction_x, correction_y)
    if not math.isfinite(correction_norm):
        raise ValueError("catchup correction overflow")
    if min_horizontal_speed > 0 and max_correction is not None:
        base_speed = math.hypot(base_velocity[0], base_velocity[1])
        if base_speed < 1e-6:
            raise ValueError("fixedwing reference has no course")
        max_correction = min(max_correction, 0.5 * base_speed)
    if max_correction is not None and correction_norm > max_correction:
        scale = max_correction / correction_norm
        correction_x *= scale
        correction_y *= scale
    vx = base_velocity[0] + correction_x
    vy = base_velocity[1] + correction_y
    speed = math.hypot(vx, vy)
    if (max_horizontal_speed is not None and
            speed > max_horizontal_speed):
        scale = max_horizontal_speed / speed
        vx *= scale
        vy *= scale
    elif speed < min_horizontal_speed:
        if speed < 1e-6:
            vx, vy = base_velocity[:2]
            speed = math.hypot(vx, vy)
        if speed < 1e-6:
            raise ValueError("fixedwing reference has no course")
        scale = min_horizontal_speed / speed
        vx *= scale
        vy *= scale
    return (vx, vy, base_velocity[2])


def virtual_center(position, heading, offset):
    """Translate one aircraft's state back to the common formation center."""
    x, y, z, _ = slot_target((0.0, 0.0, 0.0), heading, offset)
    return (position[0] - x, position[1] - y, position[2] - z)


def shared_center(centers):
    """Return the team's mean virtual center and largest horizontal error."""
    points = tuple(tuple(float(value) for value in point) for point in centers)
    if not points or any(len(point) != 3 for point in points) or \
            not all(math.isfinite(value) for point in points for value in point):
        raise ValueError("formation centers must be finite XYZ points")
    mean = tuple(sum(point[axis] for point in points) / len(points)
                 for axis in range(3))
    largest_error = max(math.hypot(point[0] - mean[0],
                                   point[1] - mean[1]) for point in points)
    return mean, largest_error


def acquisition_slots(positions, offsets, heading, altitude):
    """Place any size/layout of fixed member slots around the team's centroid."""
    locations = tuple(tuple(float(value) for value in point)
                      for point in positions)
    offsets = tuple(tuple(float(value) for value in slot)
                    for slot in offsets)
    if (not locations or len(locations) != len(offsets) or
            any(len(point) != 3 for point in locations) or
            any(len(slot) != 3 for slot in offsets) or
            not all(math.isfinite(value) for point in locations
                    for value in point) or
            not all(math.isfinite(value) for slot in offsets
                    for value in slot) or
            not math.isfinite(heading) or not math.isfinite(altitude)):
        raise ValueError("invalid acquisition positions or offsets")
    mean_position = tuple(sum(point[axis] for point in locations) /
                          len(locations) for axis in range(3))
    rotated = tuple(slot_target((0.0, 0.0, 0.0), heading, slot)[:3]
                    for slot in offsets)
    mean_offset = tuple(sum(slot[axis] for slot in rotated) /
                        len(rotated) for axis in range(3))
    anchor = (mean_position[0] - mean_offset[0],
              mean_position[1] - mean_offset[1], altitude)
    return tuple(slot_target(anchor, heading, offset)[:3]
                 for offset in offsets)


def ring_neighbors(members, own_name):
    """Swiftwing's ring Laplacian, with duplicate edges removed for N=2."""
    names = tuple(members)
    if len(names) < 2 or len(set(names)) != len(names) or own_name not in names:
        raise ValueError("ring requires at least two unique members")
    index = names.index(own_name)
    return tuple(dict.fromkeys((names[(index - 1) % len(names)],
                                names[(index + 1) % len(names)])))


def rounded_path(waypoints, radius, loop):
    """Replace horizontal route corners with short circular arcs."""
    points = tuple(tuple(float(value) for value in point)
                   for point in waypoints)
    if len(points) < 2 or any(len(point) != 3 for point in points) or \
            not all(math.isfinite(value) for point in points for value in point):
        raise ValueError("path requires finite XYZ waypoints")
    if not math.isfinite(radius) or radius < 0:
        raise ValueError("corner radius must be finite and nonnegative")
    if radius == 0:
        return points
    if loop and points[-1] == points[0]:
        points = points[:-1]
    if len(points) < 3:
        return points
    result = []
    for index, point in enumerate(points):
        if not loop and index in (0, len(points) - 1):
            result.append(point)
            continue
        previous = points[(index - 1) % len(points)]
        following = points[(index + 1) % len(points)]
        incoming = (point[0] - previous[0], point[1] - previous[1])
        outgoing = (following[0] - point[0], following[1] - point[1])
        incoming_length = math.hypot(*incoming)
        outgoing_length = math.hypot(*outgoing)
        if min(incoming_length, outgoing_length) < 1e-3:
            raise ValueError("path has a zero-length horizontal segment")
        incoming = tuple(value / incoming_length for value in incoming)
        outgoing = tuple(value / outgoing_length for value in outgoing)
        turn = math.atan2(
            incoming[0] * outgoing[1] - incoming[1] * outgoing[0],
            incoming[0] * outgoing[0] + incoming[1] * outgoing[1])
        if abs(turn) < 1e-3:
            result.append(point)
            continue
        if abs(turn) > math.pi - 1e-3:
            raise ValueError("path has a U-turn that cannot be rounded")
        tangent = math.tan(abs(turn) / 2.0)
        distance = min(radius * tangent,
                       0.45 * incoming_length,
                       0.45 * outgoing_length)
        effective_radius = distance / tangent
        entry = (point[0] - incoming[0] * distance,
                 point[1] - incoming[1] * distance,
                 point[2] - (point[2] - previous[2]) *
                 distance / incoming_length)
        exit_point = (point[0] + outgoing[0] * distance,
                      point[1] + outgoing[1] * distance,
                      point[2] + (following[2] - point[2]) *
                      distance / outgoing_length)
        direction = 1.0 if turn > 0 else -1.0
        center = (entry[0] - direction * incoming[1] * effective_radius,
                  entry[1] + direction * incoming[0] * effective_radius)
        first_angle = math.atan2(entry[1] - center[1],
                                 entry[0] - center[0])
        sections = max(2, int(math.ceil(abs(turn) * effective_radius)))
        result.append(entry)
        for section in range(1, sections + 1):
            fraction = section / sections
            angle = first_angle + fraction * turn
            result.append((center[0] + effective_radius * math.cos(angle),
                           center[1] + effective_radius * math.sin(angle),
                           entry[2] + fraction * (exit_point[2] - entry[2])))
    return tuple(result)


def path_reference_state(center, waypoints, speed, lookahead, loop):
    """Return a path preview, its velocity, and its local turn rate."""
    points = tuple(tuple(float(value) for value in point) for point in waypoints)
    if len(points) < 2 or any(len(point) != 3 for point in points):
        raise ValueError("path requires at least two XYZ waypoints")
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError("path waypoint is non-finite")
    if (not all(math.isfinite(value) for value in (*center, speed, lookahead))
            or speed <= 0 or lookahead < 0):
        raise ValueError("invalid path speed, preview or center")
    if loop and points[-1] != points[0]:
        points += (points[0],)
    segments = []
    length_so_far = 0.0
    for start, end in zip(points[:-1], points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 1e-3:
            raise ValueError("path has a zero-length horizontal segment")
        segments.append((start, end, dx, dy, length, length_so_far))
        length_so_far += length
    nearest_distance_squared = float("inf")
    nearest_s = 0.0
    for start, _end, dx, dy, length, beginning in segments:
        fraction = max(0.0, min(1.0,
            ((center[0] - start[0]) * dx +
             (center[1] - start[1]) * dy) / (length * length)))
        px, py = start[0] + fraction * dx, start[1] + fraction * dy
        distance_squared = (center[0] - px)**2 + (center[1] - py)**2
        if distance_squared < nearest_distance_squared:
            nearest_distance_squared = distance_squared
            nearest_s = beginning + fraction * length
    target_s = nearest_s + lookahead
    if loop:
        target_s %= length_so_far
    else:
        target_s = min(target_s, length_so_far)
    for index, (start, end, dx, dy, length, beginning) in enumerate(segments):
        if target_s <= beginning + length or index == len(segments) - 1:
            fraction = max(0.0, min(1.0, (target_s - beginning) / length))
            position = (start[0] + fraction * dx,
                        start[1] + fraction * dy,
                        start[2] + fraction * (end[2] - start[2]))
            velocity = (speed * dx / length, speed * dy / length,
                        speed * (end[2] - start[2]) / length)
            if len(segments) < 2 or (not loop and index == 0):
                turn_rate = 0.0
            else:
                previous = segments[(index - 1) % len(segments)]
                cross = previous[2] * dy - previous[3] * dx
                dot = previous[2] * dx + previous[3] * dy
                heading_change = math.atan2(cross, dot)
                arc_length = 0.5 * (previous[4] + length)
                turn_rate = speed * heading_change / arc_length
            return position, velocity, turn_rate
    raise AssertionError("unreachable path segment")


def path_reference(center, waypoints, speed, lookahead, loop):
    """Project the virtual center onto a shared path and preview along it."""
    position, velocity, _turn_rate = path_reference_state(
        center, waypoints, speed, lookahead, loop)
    return position, velocity


def consensus_velocity(path_velocity, desired_center, own_center,
                       neighbor_centers, gain, convergence_time,
                       max_correction=None):
    """Path velocity plus neighbor-center consensus correction."""
    values = (*path_velocity, *desired_center, *own_center,
              *(value for center in neighbor_centers for value in center),
              gain, convergence_time)
    if max_correction is not None:
        values += (max_correction,)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("non-finite consensus input")
    if (gain < 0 or convergence_time <= 0 or
            (max_correction is not None and max_correction < 0)):
        raise ValueError("invalid consensus gains")
    if not neighbor_centers:
        raise ValueError("consensus requires neighbors")
    vx = path_velocity[0] + (desired_center[0] - own_center[0]) / convergence_time
    vy = path_velocity[1] + (desired_center[1] - own_center[1]) / convergence_time
    path_speed = math.hypot(path_velocity[0], path_velocity[1])
    if (vx * path_velocity[0] + vy * path_velocity[1] < 0.0):
        # A member ahead of the preview point should slow down and let the
        # group catch up; asking it to fly backward destabilizes fixed-wings.
        vx, vy = path_velocity[:2]
    magnitude = math.hypot(vx, vy)
    if magnitude > 1e-6:
        vx, vy = vx * path_speed / magnitude, vy * path_speed / magnitude
    # -gain * L_i * center, where L_i has one edge to each ring neighbor.
    correction_x = -gain * sum(own_center[0] - center[0]
                               for center in neighbor_centers)
    correction_y = -gain * sum(own_center[1] - center[1]
                               for center in neighbor_centers)
    correction_norm = math.hypot(correction_x, correction_y)
    if (max_correction is not None and
            correction_norm > max_correction and correction_norm > 0):
        scale = max_correction / correction_norm
        correction_x *= scale
        correction_y *= scale
    return vx + correction_x, vy + correction_y, path_velocity[2]


def consensus_speed_targets(path_speed, progress_errors, member_speeds,
                            gain, max_speed_adjustment, min_speed,
                            deadband=1.0):
    """Pace a team so lagging members gain speed and others wait for them.

    ``progress_errors`` are positive for members behind the team's mean.
    During a gap, lagging members get a speed increase while members ahead of
    the mean slow down by the same bounded amount. An ahead member already
    moving slowly is not told to speed back up just to match that pace. A
    small deadband avoids speed changes caused by position noise.
    """
    errors = tuple(float(value) for value in progress_errors)
    speeds = tuple(float(value) for value in member_speeds)
    values = (path_speed, gain, max_speed_adjustment, min_speed, deadband,
              *errors, *speeds)
    if (not errors or len(errors) != len(speeds) or
            not all(math.isfinite(float(value)) for value in values) or
            path_speed <= 0 or gain < 0 or max_speed_adjustment < 0 or
            min_speed < 0 or deadband < 0):
        raise ValueError("invalid consensus pacing inputs")

    baseline_speed = max(path_speed, min_speed)
    maximum_lag = max(0.0, max(errors) - deadband)
    if maximum_lag <= 0:
        return tuple(baseline_speed for _ in errors)

    shared_adjustment = min(max_speed_adjustment, gain * maximum_lag)
    targets = []
    for error, speed in zip(errors, speeds):
        if error > deadband:
            targets.append(baseline_speed + shared_adjustment)
        elif error < -deadband:
            pace = max(min_speed, baseline_speed - shared_adjustment)
            targets.append(max(min_speed, min(pace, speed)))
        else:
            targets.append(baseline_speed)
    return tuple(targets)


def blocking_action(action, control_state):
    """An internal flight action always takes control from formation."""
    return action in (
        control_state.ACTION_ARM,
        control_state.ACTION_DISARM,
        control_state.ACTION_TAKEOFF,
        control_state.ACTION_LAND,
        control_state.ACTION_RETURN_HOME,
        control_state.ACTION_EMERGENCY,
    )
