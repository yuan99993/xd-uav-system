#!/usr/bin/env python3
"""Spawn configurable red test boxes in a running Gazebo Classic world."""

import argparse
import math
import random
import re
import sys
from typing import List, Optional, Sequence, Tuple

import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


DEFAULT_AREAS = (
    (0.0, 20.0, 0.0, 20.0),
    (-30.0, 0.0, -30.0, 0.0),
)

Point2 = Tuple[float, float]
Polygon2 = Tuple[Point2, ...]


def _comma_separated_floats(value: str, lengths: Sequence[int], label: str) -> Tuple[float, ...]:
    try:
        numbers = tuple(float(item.strip()) for item in value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{label} must contain only numbers: {value}") from error
    if len(numbers) not in lengths or not all(math.isfinite(item) for item in numbers):
        expected = " or ".join(str(length) for length in lengths)
        raise argparse.ArgumentTypeError(
            f"{label} requires {expected} comma-separated finite numbers: {value}"
        )
    return numbers


def _position(value: str) -> Tuple[float, ...]:
    return _comma_separated_floats(value, (2, 3), "position")


def _area(value: str) -> Tuple[float, float, float, float]:
    numbers = _comma_separated_floats(value, (4,), "area")
    xmin, xmax, ymin, ymax = numbers
    if xmin >= xmax or ymin >= ymax:
        raise argparse.ArgumentTypeError(
            "area order must be xmin,xmax,ymin,ymax with min < max"
        )
    return xmin, xmax, ymin, ymax


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Spawn solid red boxes through /gazebo/spawn_sdf_model. Use either "
            "explicit --position entries or --count for random placement."
        ),
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 spawn_red_boxes.py --position 5,5 --position 15,15\n"
            "  python3 spawn_red_boxes.py --position=-10,-10,0.5 --replace\n"
            "  python3 spawn_red_boxes.py --count 6 --seed 7 --replace\n"
            "  python3 spawn_red_boxes.py --count 3 --area=-25,-5,-25,-5\n\n"
            "  python3 spawn_red_boxes.py --count 6 --search-area-topic "
            "/task_allocate/search_areas --ground-z 0 --replace\n\n"
            "For a negative explicit coordinate, use --position=-10,-10.\n"
            "A two-value position is X,Y; Z is then placed on --ground-z."
        ),
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--position",
        action="append",
        type=_position,
        metavar="X,Y[,Z]",
        help="exact world position; repeat once per box",
    )
    mode.add_argument(
        "--count",
        type=int,
        metavar="N",
        help="number of boxes placed randomly across the configured areas",
    )
    parser.add_argument(
        "--area",
        action="append",
        type=_area,
        metavar="XMIN,XMAX,YMIN,YMAX",
        help=(
            "random-placement rectangle; repeat for multiple areas\n"
            "default: 0,20,0,20 and -30,0,-30,0"
        ),
    )
    parser.add_argument(
        "--search-area-topic",
        default=None,
        metavar="TOPIC",
        help=(
            "wait for xd_uav_task_allocate/SearchAreaArray and place random "
            "boxes inside all published polygons; use with --count"
        ),
    )
    parser.add_argument(
        "--search-area-timeout",
        type=float,
        default=30.0,
        metavar="SECONDS",
        help="maximum wait for SearchAreaArray (default: 30)",
    )
    parser.add_argument(
        "--size",
        nargs=3,
        type=float,
        default=(1.0, 1.0, 1.0),
        metavar=("X", "Y", "Z"),
        help="box dimensions in metres (default: 1 1 1)",
    )
    parser.add_argument(
        "--ground-z",
        type=float,
        default=4.5,
        help="ground height used when an explicit Z is omitted (default: 4.5)",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=0,
        help="minimum clearance between each box face and an area edge (default: 0)",
    )
    parser.add_argument(
        "--min-spacing",
        type=float,
        default=2.0,
        help="minimum XY centre spacing for random boxes (default: 2.0)",
    )
    parser.add_argument("--seed", type=int, default=None, help="repeatable random seed")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="box yaw in degrees")
    parser.add_argument("--prefix", default="red_box", help="Gazebo model-name prefix")
    parser.add_argument("--start-index", type=int, default=1, help="first model index")
    parser.add_argument(
        "--replace",
        action="store_true",
        help="delete all existing PREFIX_N models before spawning",
    )
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="make boxes movable; boxes are static by default",
    )
    parser.add_argument("--reference-frame", default="world")
    parser.add_argument("--spawn-service", default="/gazebo/spawn_sdf_model")
    parser.add_argument("--delete-service", default="/gazebo/delete_model")
    parser.add_argument(
        "--world-properties-service", default="/gazebo/get_world_properties"
    )
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print generated model positions without contacting Gazebo",
    )
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.count is not None and args.count <= 0:
        parser.error("--count must be greater than zero")
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", args.prefix):
        parser.error("--prefix must start with a letter and contain only letters, digits, or '_'")
    if args.start_index < 0:
        parser.error("--start-index cannot be negative")
    if len(args.size) != 3 or any(not math.isfinite(item) or item <= 0.0 for item in args.size):
        parser.error("all --size values must be finite and greater than zero")
    if not math.isfinite(args.ground_z):
        parser.error("--ground-z must be finite")
    if not math.isfinite(args.margin) or args.margin < 0.0:
        parser.error("--margin must be finite and non-negative")
    if not math.isfinite(args.min_spacing) or args.min_spacing < 0.0:
        parser.error("--min-spacing must be finite and non-negative")
    if not math.isfinite(args.yaw_deg):
        parser.error("--yaw-deg must be finite")
    if not math.isfinite(args.service_timeout) or args.service_timeout <= 0.0:
        parser.error("--service-timeout must be finite and greater than zero")
    if (
        not math.isfinite(args.search_area_timeout)
        or args.search_area_timeout <= 0.0
    ):
        parser.error("--search-area-timeout must be finite and greater than zero")
    if args.search_area_topic is not None:
        if args.count is None:
            parser.error("--search-area-topic requires --count")
        if args.area:
            parser.error("--search-area-topic cannot be combined with --area")
        if not args.search_area_topic.strip():
            parser.error("--search-area-topic cannot be empty")


def _random_positions(
    count: int,
    areas: Sequence[Tuple[float, float, float, float]],
    size: Sequence[float],
    ground_z: float,
    margin: float,
    min_spacing: float,
    generator: random.Random,
) -> List[Tuple[float, float, float]]:
    usable_areas = []
    half_x = float(size[0]) / 2.0
    half_y = float(size[1]) / 2.0
    for xmin, xmax, ymin, ymax in areas:
        bounds = (
            xmin + margin + half_x,
            xmax - margin - half_x,
            ymin + margin + half_y,
            ymax - margin - half_y,
        )
        if bounds[0] > bounds[1] or bounds[2] > bounds[3]:
            raise ValueError(
                f"area {(xmin, xmax, ymin, ymax)} is too small for size and margin"
            )
        usable_areas.append(bounds)

    result: List[Tuple[float, float, float]] = []
    attempts_per_box = 1000
    centre_z = float(ground_z) + float(size[2]) / 2.0
    for index in range(count):
        # Cycling areas keeps the requested quantity approximately balanced.
        xmin, xmax, ymin, ymax = usable_areas[index % len(usable_areas)]
        for _ in range(attempts_per_box):
            candidate = (generator.uniform(xmin, xmax), generator.uniform(ymin, ymax), centre_z)
            if all(
                math.hypot(candidate[0] - old[0], candidate[1] - old[1]) >= min_spacing
                for old in result
            ):
                result.append(candidate)
                break
        else:
            raise ValueError(
                "could not satisfy --min-spacing; reduce the count/spacing or enlarge the areas"
            )
    return result


def _polygon_area(polygon: Polygon2) -> float:
    return 0.5 * abs(
        sum(
            first[0] * second[1] - second[0] * first[1]
            for first, second in zip(polygon, polygon[1:] + polygon[:1])
        )
    )


def _point_in_polygon(point: Point2, polygon: Polygon2) -> bool:
    """Return true for points inside or on a simple polygon boundary."""

    x, y = point
    inside = False
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        ax, ay = first
        bx, by = second
        cross = (x - ax) * (by - ay) - (y - ay) * (bx - ax)
        if abs(cross) <= 1e-9 and (
            min(ax, bx) - 1e-9 <= x <= max(ax, bx) + 1e-9
            and min(ay, by) - 1e-9 <= y <= max(ay, by) + 1e-9
        ):
            return True
        if (ay > y) != (by > y):
            intersection_x = ax + (y - ay) * (bx - ax) / (by - ay)
            if x < intersection_x:
                inside = not inside
    return inside


def _point_segment_distance(point: Point2, first: Point2, second: Point2) -> float:
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    squared_length = dx * dx + dy * dy
    if squared_length <= 1e-18:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    ratio = (
        (point[0] - first[0]) * dx + (point[1] - first[1]) * dy
    ) / squared_length
    ratio = max(0.0, min(1.0, ratio))
    closest = (first[0] + ratio * dx, first[1] + ratio * dy)
    return math.hypot(point[0] - closest[0], point[1] - closest[1])


def _has_boundary_clearance(
    point: Point2, polygon: Polygon2, clearance: float
) -> bool:
    if not _point_in_polygon(point, polygon):
        return False
    return all(
        _point_segment_distance(point, first, second) + 1e-9 >= clearance
        for first, second in zip(polygon, polygon[1:] + polygon[:1])
    )


def _random_positions_in_polygons(
    count: int,
    polygons: Sequence[Polygon2],
    size: Sequence[float],
    ground_z: float,
    margin: float,
    min_spacing: float,
    generator: random.Random,
) -> List[Tuple[float, float, float]]:
    if not polygons:
        raise ValueError("SearchAreaArray contains no usable polygons")
    # Requiring the centre to remain this far from every polygon edge is
    # conservative but guarantees the complete box footprint stays inside,
    # regardless of --yaw-deg.
    clearance = math.hypot(float(size[0]) / 2.0, float(size[1]) / 2.0) + margin
    usable = []
    for index, polygon in enumerate(polygons):
        if len(polygon) < 3 or not all(
            math.isfinite(value) for point in polygon for value in point
        ):
            raise ValueError(f"search polygon {index} is invalid")
        area = _polygon_area(polygon)
        if area <= 1e-9:
            raise ValueError(f"search polygon {index} has zero area")
        xs = [point[0] for point in polygon]
        ys = [point[1] for point in polygon]
        bounds = (
            min(xs) + clearance,
            max(xs) - clearance,
            min(ys) + clearance,
            max(ys) - clearance,
        )
        if bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
            raise ValueError(
                f"search polygon {index} is too small for box size and margin"
            )
        usable.append((polygon, area, bounds))

    total_area = sum(item[1] for item in usable)
    centre_z = float(ground_z) + float(size[2]) / 2.0
    result: List[Tuple[float, float, float]] = []
    selected_indices = []
    if count >= len(usable):
        # "All published areas" should be literal when enough boxes were
        # requested: seed every polygon once, then distribute the remainder
        # proportional to polygon area.
        selected_indices.extend(range(len(usable)))
    else:
        selected_indices.extend(generator.sample(range(len(usable)), count))
    while len(selected_indices) < count:
        selection = generator.uniform(0.0, total_area)
        accumulated = 0.0
        selected_index = len(usable) - 1
        for index, item in enumerate(usable):
            accumulated += item[1]
            if selection <= accumulated:
                selected_index = index
                break
        selected_indices.append(selected_index)
    generator.shuffle(selected_indices)

    for selected_index in selected_indices:
        polygon, _, bounds = usable[selected_index]
        for _ in range(5000):
            candidate = (
                generator.uniform(bounds[0], bounds[1]),
                generator.uniform(bounds[2], bounds[3]),
                centre_z,
            )
            if not _has_boundary_clearance(candidate[:2], polygon, clearance):
                continue
            if any(
                math.hypot(candidate[0] - old[0], candidate[1] - old[1])
                < min_spacing
                for old in result
            ):
                continue
            result.append(candidate)
            break
        else:
            raise ValueError(
                "could not place all boxes inside the search polygons; reduce "
                "--count/--size/--margin/--min-spacing or enlarge the areas"
            )
    return result


def _ensure_ros_node() -> None:
    if not rospy.core.is_initialized():
        rospy.init_node("spawn_red_boxes", anonymous=True)


def _search_area_polygons(
    topic: str, timeout: float, reference_frame: str
) -> List[Polygon2]:
    try:
        from xd_uav_task_allocate.msg import SearchAreaArray
    except ImportError as error:
        raise ValueError(
            "xd_uav_task_allocate messages are unavailable; source the workspace"
        ) from error
    _ensure_ros_node()
    try:
        message = rospy.wait_for_message(topic, SearchAreaArray, timeout=timeout)
    except (rospy.ROSException, rospy.ROSInterruptException) as error:
        raise ValueError(f"could not receive SearchAreaArray from {topic}: {error}") from error
    message_frame = message.header.frame_id.lstrip("/")
    requested_frame = str(reference_frame).lstrip("/")
    if not message_frame:
        raise ValueError("SearchAreaArray.header.frame_id is empty")
    if message_frame != requested_frame:
        raise ValueError(
            f"search areas use frame '{message_frame}', but boxes use "
            f"reference frame '{requested_frame}'; transform the areas first"
        )
    return [
        tuple((float(point.x), float(point.y)) for point in area.boundary.points)
        for area in message.areas
    ]


def _explicit_positions(
    raw_positions: Sequence[Tuple[float, ...]],
    size_z: float,
    ground_z: float,
) -> List[Tuple[float, float, float]]:
    default_z = float(ground_z) + float(size_z) / 2.0
    return [
        (float(item[0]), float(item[1]), float(item[2]) if len(item) == 3 else default_z)
        for item in raw_positions
    ]


def _model_sdf(name: str, size: Sequence[float], static: bool) -> str:
    size_text = "{:.6f} {:.6f} {:.6f}".format(*size)
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>{str(static).lower()}</static>
    <link name="box_link">
      <collision name="box_collision">
        <geometry><box><size>{size_text}</size></box></geometry>
      </collision>
      <visual name="box_visual">
        <cast_shadows>true</cast_shadows>
        <geometry><box><size>{size_text}</size></box></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.1 0 0 1</specular>
          <emissive>0.15 0 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _pose(position: Sequence[float], yaw_degrees: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    quaternion = quaternion_from_euler(0.0, 0.0, math.radians(yaw_degrees))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    return pose


def _spawn(args: argparse.Namespace, positions: Sequence[Tuple[float, float, float]]) -> int:
    _ensure_ros_node()
    try:
        rospy.wait_for_service(args.spawn_service, timeout=args.service_timeout)
        spawn_model = rospy.ServiceProxy(args.spawn_service, SpawnModel)
        delete_model: Optional[rospy.ServiceProxy] = None
        if args.replace:
            rospy.wait_for_service(args.delete_service, timeout=args.service_timeout)
            rospy.wait_for_service(
                args.world_properties_service, timeout=args.service_timeout
            )
            delete_model = rospy.ServiceProxy(args.delete_service, DeleteModel)
            get_world_properties = rospy.ServiceProxy(
                args.world_properties_service, GetWorldProperties
            )
    except (rospy.ROSException, rospy.ROSInterruptException) as error:
        print(f"Gazebo service unavailable: {error}", file=sys.stderr)
        return 2

    failures = 0
    if delete_model is not None:
        try:
            world = get_world_properties()
            if not world.success:
                print(
                    f"Could not list old Gazebo models: {world.status_message}",
                    file=sys.stderr,
                )
                return 2
            model_pattern = re.compile(rf"^{re.escape(args.prefix)}_[0-9]+$")
            for old_name in world.model_names:
                if not model_pattern.fullmatch(old_name):
                    continue
                response = delete_model(old_name)
                if response.success:
                    print(f"removed old model {old_name}")
                else:
                    failures += 1
                    print(
                        f"FAILED to remove {old_name}: {response.status_message}",
                        file=sys.stderr,
                    )
        except (rospy.ServiceException, rospy.ROSInterruptException) as error:
            print(f"Could not clear old {args.prefix}_N models: {error}", file=sys.stderr)
            return 2

    for offset, position in enumerate(positions):
        name = f"{args.prefix}_{args.start_index + offset}"
        try:
            response = spawn_model(
                name,
                _model_sdf(name, args.size, static=not args.dynamic),
                "",
                _pose(position, args.yaw_deg),
                args.reference_frame,
            )
        except (rospy.ServiceException, rospy.ROSInterruptException) as error:
            failures += 1
            print(f"FAILED {name}: {error}", file=sys.stderr)
            continue
        if response.success:
            print(
                f"spawned {name}: x={position[0]:.3f}, y={position[1]:.3f}, "
                f"z={position[2]:.3f}"
            )
        else:
            failures += 1
            print(f"FAILED {name}: {response.status_message}", file=sys.stderr)
    return 1 if failures else 0


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    _validate_args(parser, args)

    try:
        if args.position is not None:
            positions = _explicit_positions(args.position, args.size[2], args.ground_z)
        elif args.search_area_topic is not None:
            polygons = _search_area_polygons(
                args.search_area_topic,
                args.search_area_timeout,
                args.reference_frame,
            )
            positions = _random_positions_in_polygons(
                args.count,
                polygons,
                args.size,
                args.ground_z,
                args.margin,
                args.min_spacing,
                random.Random(args.seed),
            )
        else:
            positions = _random_positions(
                args.count,
                args.area or DEFAULT_AREAS,
                args.size,
                args.ground_z,
                args.margin,
                args.min_spacing,
                random.Random(args.seed),
            )
    except ValueError as error:
        parser.error(str(error))

    if args.dry_run:
        for offset, position in enumerate(positions):
            name = f"{args.prefix}_{args.start_index + offset}"
            print(f"{name}: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
        return 0
    return _spawn(args, positions)


if __name__ == "__main__":
    sys.exit(main())
