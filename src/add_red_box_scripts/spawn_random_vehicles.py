#!/usr/bin/env python3
"""Spawn random static vehicle models inside published task search areas."""

import argparse
import math
import os
import random
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel

from spawn_red_boxes import (
    Polygon2,
    _ensure_ros_node,
    _has_boundary_clearance,
    _polygon_area,
    _pose,
    _search_area_polygons,
)


DEFAULT_MODELS = (
    "bus",
    "car_beetle",
    "car_golf",
    "car_lexus",
    "car_opel",
    "car_polo",
    "car_volvo",
)

# Conservative XY footprints used only for boundary and mutual-overlap checks.
# Gazebo still renders the original model meshes without scaling them.
MODEL_FOOTPRINTS = {
    "bus": (12.0, 3.2),
    "car_beetle": (4.5, 2.2),
    "car_golf": (4.8, 2.2),
    "car_lexus": (5.0, 2.3),
    "car_opel": (4.8, 2.2),
    "car_polo": (4.5, 2.2),
    "car_volvo": (4.8, 2.2),
}


@dataclass(frozen=True)
class VehicleTemplate:
    model_type: str
    sdf_xml: str
    length: float
    width: float
    z_offset: float

    @property
    def clearance_radius(self) -> float:
        return math.hypot(self.length / 2.0, self.width / 2.0)


@dataclass(frozen=True)
class VehiclePlacement:
    template: VehicleTemplate
    x: float
    y: float
    z: float
    yaw_degrees: float
    area_index: int


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Randomly spawn Gazebo vehicle models inside every polygon received "
            "from xd_uav_task_allocate/SearchAreaArray."
        ),
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Example:\n"
            "  python3 spawn_random_vehicles.py --count 14 --replace\n"
            "  python3 spawn_random_vehicles.py --count 6 --models car_golf car_volvo\n"
            "  python3 spawn_random_vehicles.py --count 10 --seed 7 --dry-run\n\n"
            "Start this script before a one-shot 'rostopic pub -1' command so it "
            "is already waiting for the search-area message."
        ),
    )
    parser.add_argument(
        "--count",
        type=int,
        default=10,
        metavar="N",
        help="number of vehicles to spawn (default: 10)",
    )
    parser.add_argument(
        "--models",
        nargs="+",
        choices=DEFAULT_MODELS,
        default=list(DEFAULT_MODELS),
        metavar="MODEL",
        help="allowed vehicle types (default: all seven supported models)",
    )
    parser.add_argument(
        "--search-area-topic",
        default="/task_allocate/search_areas",
        metavar="TOPIC",
        help="SearchAreaArray input topic (default: /task_allocate/search_areas)",
    )
    parser.add_argument(
        "--search-area-timeout",
        type=float,
        default=60.0,
        metavar="SECONDS",
        help="maximum wait for a search-area message (default: 60)",
    )
    parser.add_argument(
        "--model-root",
        default=None,
        metavar="DIRECTORY",
        help="optional Gazebo model directory; normal Gazebo paths are searched by default",
    )
    parser.add_argument(
        "--ground-z",
        type=float,
        default=0.0,
        help="world ground height underneath the vehicles (default: 0)",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=1.0,
        help="extra clearance from each search-area boundary in metres (default: 1)",
    )
    parser.add_argument(
        "--min-gap",
        type=float,
        default=2.0,
        help="minimum gap between conservative vehicle footprints (default: 2)",
    )
    parser.add_argument("--seed", type=int, default=None, help="repeatable random seed")
    parser.add_argument(
        "--prefix", default="search_vehicle", help="Gazebo model-name prefix"
    )
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument(
        "--replace",
        action="store_true",
        help="delete existing PREFIX_N vehicles before spawning the new random set",
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
        help="read areas and print the random layout without changing Gazebo",
    )
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.count <= 0:
        parser.error("--count must be greater than zero")
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", args.prefix):
        parser.error("--prefix must start with a letter and contain only letters, digits, or '_'")
    if args.start_index < 0:
        parser.error("--start-index cannot be negative")
    for label in ("ground_z", "margin", "min_gap", "search_area_timeout", "service_timeout"):
        if not math.isfinite(float(getattr(args, label))):
            parser.error("--{} must be finite".format(label.replace("_", "-")))
    if args.margin < 0.0 or args.min_gap < 0.0:
        parser.error("--margin and --min-gap must be non-negative")
    if args.search_area_timeout <= 0.0 or args.service_timeout <= 0.0:
        parser.error("timeouts must be greater than zero")
    if not args.search_area_topic.strip():
        parser.error("--search-area-topic cannot be empty")


def _model_search_roots(explicit_root: Optional[str]) -> List[Path]:
    raw_roots = []
    if explicit_root:
        raw_roots.append(explicit_root)
    raw_roots.extend(
        item for item in os.environ.get("GAZEBO_MODEL_PATH", "").split(os.pathsep) if item
    )
    raw_roots.extend(
        (
            str(Path.home() / ".gazebo" / "models"),
            "/usr/share/gazebo-11/models",
            "/usr/share/gazebo/models",
        )
    )
    roots = []
    for raw_root in raw_roots:
        root = Path(raw_root).expanduser().resolve()
        if root not in roots:
            roots.append(root)
    return roots


def _find_model_sdf(model_type: str, roots: Sequence[Path]) -> Path:
    for root in roots:
        candidate = root / model_type / "model.sdf"
        if candidate.is_file():
            return candidate
    raise ValueError(
        "could not find {}/model.sdf under: {}".format(
            model_type, ", ".join(str(root) for root in roots)
        )
    )


def _load_template(model_type: str, roots: Sequence[Path]) -> VehicleTemplate:
    sdf_path = _find_model_sdf(model_type, roots)
    try:
        tree = ET.parse(str(sdf_path))
    except (ET.ParseError, OSError) as error:
        raise ValueError("could not read {}: {}".format(sdf_path, error)) from error
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise ValueError("{} has no <model> element".format(sdf_path))

    # The downloaded car files contain sample world X/Y/yaw values in their
    # top-level pose. Remove those values so SpawnModel's initial_pose is the
    # only horizontal pose. Preserve Z because car_opel needs its 0.8 m lift.
    z_offset = 0.0
    model_pose = model.find("pose")
    if model_pose is not None and model_pose.text:
        values = model_pose.text.split()
        if len(values) >= 3:
            try:
                z_offset = float(values[2])
            except ValueError as error:
                raise ValueError("invalid model pose in {}".format(sdf_path)) from error
        model_pose.text = "0 0 0 0 0 0"
    model.set("name", "MODEL_INSTANCE_NAME")
    length, width = MODEL_FOOTPRINTS[model_type]
    return VehicleTemplate(
        model_type=model_type,
        sdf_xml=ET.tostring(root, encoding="unicode"),
        length=length,
        width=width,
        z_offset=z_offset,
    )


def _select_templates(
    count: int,
    allowed: Sequence[VehicleTemplate],
    generator: random.Random,
) -> List[VehicleTemplate]:
    if count < len(allowed):
        result = generator.sample(list(allowed), count)
    else:
        result = list(allowed)
        result.extend(generator.choice(allowed) for _ in range(count - len(allowed)))
    generator.shuffle(result)
    return result


def _area_assignments(
    count: int, polygons: Sequence[Polygon2], generator: random.Random
) -> List[int]:
    areas = [_polygon_area(polygon) for polygon in polygons]
    if any(area <= 1e-9 for area in areas):
        raise ValueError("SearchAreaArray contains an invalid zero-area polygon")
    total_area = sum(areas)
    assignments = list(range(len(polygons))) if count >= len(polygons) else []
    while len(assignments) < count:
        draw = generator.uniform(0.0, total_area)
        accumulated = 0.0
        selected = len(polygons) - 1
        for index, area in enumerate(areas):
            accumulated += area
            if draw <= accumulated:
                selected = index
                break
        assignments.append(selected)
    generator.shuffle(assignments)
    return assignments


def _random_placements(
    templates: Sequence[VehicleTemplate],
    polygons: Sequence[Polygon2],
    ground_z: float,
    margin: float,
    min_gap: float,
    generator: random.Random,
) -> List[VehiclePlacement]:
    if not polygons:
        raise ValueError("SearchAreaArray contains no usable polygons")
    assignments = _area_assignments(len(templates), polygons, generator)
    bounds = []
    for index, polygon in enumerate(polygons):
        if len(polygon) < 3:
            raise ValueError("search polygon {} has fewer than three points".format(index))
        xs = [point[0] for point in polygon]
        ys = [point[1] for point in polygon]
        bounds.append((min(xs), max(xs), min(ys), max(ys)))

    result = []
    for template, area_index in zip(templates, assignments):
        polygon = polygons[area_index]
        xmin, xmax, ymin, ymax = bounds[area_index]
        clearance = template.clearance_radius + margin
        if xmax - xmin <= 2.0 * clearance or ymax - ymin <= 2.0 * clearance:
            raise ValueError(
                "search polygon {} is too small for {} and --margin".format(
                    area_index + 1, template.model_type
                )
            )
        for _ in range(10000):
            x = generator.uniform(xmin + clearance, xmax - clearance)
            y = generator.uniform(ymin + clearance, ymax - clearance)
            if not _has_boundary_clearance((x, y), polygon, clearance):
                continue
            if any(
                math.hypot(x - old.x, y - old.y)
                < template.clearance_radius
                + old.template.clearance_radius
                + min_gap
                for old in result
            ):
                continue
            result.append(
                VehiclePlacement(
                    template=template,
                    x=x,
                    y=y,
                    z=float(ground_z) + template.z_offset,
                    yaw_degrees=generator.uniform(-180.0, 180.0),
                    area_index=area_index,
                )
            )
            break
        else:
            raise ValueError(
                "could not place all vehicles; reduce --count/--margin/--min-gap "
                "or publish larger search areas"
            )
    return result


def _instance_sdf(template: VehicleTemplate, instance_name: str) -> str:
    return template.sdf_xml.replace("MODEL_INSTANCE_NAME", instance_name, 1)


def _delete_previous(args: argparse.Namespace) -> int:
    try:
        rospy.wait_for_service(args.delete_service, timeout=args.service_timeout)
        rospy.wait_for_service(
            args.world_properties_service, timeout=args.service_timeout
        )
        delete_model = rospy.ServiceProxy(args.delete_service, DeleteModel)
        get_world_properties = rospy.ServiceProxy(
            args.world_properties_service, GetWorldProperties
        )
        world = get_world_properties()
        if not world.success:
            print("Could not list Gazebo models: {}".format(world.status_message), file=sys.stderr)
            return 2
        pattern = re.compile(r"^{}_[0-9]+$".format(re.escape(args.prefix)))
        for old_name in world.model_names:
            if not pattern.fullmatch(old_name):
                continue
            response = delete_model(old_name)
            if not response.success:
                print(
                    "FAILED to remove {}: {}".format(old_name, response.status_message),
                    file=sys.stderr,
                )
                return 2
            print("removed old model {}".format(old_name))
    except (rospy.ROSException, rospy.ServiceException, rospy.ROSInterruptException) as error:
        print("Could not clear old vehicles: {}".format(error), file=sys.stderr)
        return 2
    return 0


def _spawn(args: argparse.Namespace, placements: Sequence[VehiclePlacement]) -> int:
    _ensure_ros_node()
    if args.replace and _delete_previous(args) != 0:
        return 2
    try:
        rospy.wait_for_service(args.spawn_service, timeout=args.service_timeout)
        spawn_model = rospy.ServiceProxy(args.spawn_service, SpawnModel)
    except (rospy.ROSException, rospy.ROSInterruptException) as error:
        print("Gazebo spawn service unavailable: {}".format(error), file=sys.stderr)
        return 2

    failures = 0
    for offset, placement in enumerate(placements):
        name = "{}_{}".format(args.prefix, args.start_index + offset)
        try:
            response = spawn_model(
                name,
                _instance_sdf(placement.template, name),
                "",
                _pose((placement.x, placement.y, placement.z), placement.yaw_degrees),
                args.reference_frame,
            )
        except (rospy.ServiceException, rospy.ROSInterruptException) as error:
            failures += 1
            print("FAILED {}: {}".format(name, error), file=sys.stderr)
            continue
        if response.success:
            print(
                "spawned {} ({}) in area {}: x={:.3f}, y={:.3f}, z={:.3f}, yaw={:.1f}".format(
                    name,
                    placement.template.model_type,
                    placement.area_index + 1,
                    placement.x,
                    placement.y,
                    placement.z,
                    placement.yaw_degrees,
                )
            )
        else:
            failures += 1
            print("FAILED {}: {}".format(name, response.status_message), file=sys.stderr)
    return 1 if failures else 0


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    _validate_args(parser, args)
    generator = random.Random(args.seed)
    try:
        roots = _model_search_roots(args.model_root)
        allowed = [_load_template(model_type, roots) for model_type in args.models]
        polygons = _search_area_polygons(
            args.search_area_topic,
            args.search_area_timeout,
            args.reference_frame,
        )
        placements = _random_placements(
            _select_templates(args.count, allowed, generator),
            polygons,
            args.ground_z,
            args.margin,
            args.min_gap,
            generator,
        )
    except ValueError as error:
        parser.error(str(error))

    if args.dry_run:
        for offset, placement in enumerate(placements):
            print(
                "{}_{} ({}) area={}: x={:.3f}, y={:.3f}, z={:.3f}, yaw={:.1f}".format(
                    args.prefix,
                    args.start_index + offset,
                    placement.template.model_type,
                    placement.area_index + 1,
                    placement.x,
                    placement.y,
                    placement.z,
                    placement.yaw_degrees,
                )
            )
        return 0
    return _spawn(args, placements)


if __name__ == "__main__":
    sys.exit(main())
