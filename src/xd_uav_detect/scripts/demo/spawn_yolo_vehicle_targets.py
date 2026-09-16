#!/usr/bin/env python3
"""Spawn Gazebo vehicles selected for nadir and forward-view YOLO tests."""

import os
import sys

# See spawn_random_vehicles.py: avoid importing catkin's devel wrapper as the
# sibling implementation module.
SCRIPT_DIRECTORY = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from spawn_random_vehicles import run_profile


# These large, textured models remain distinctive from above while retaining a
# normal road-vehicle silhouette from a low, forward-looking camera.  Their
# expected COCO outputs are car (2), bus (5), or truck (7).
YOLO_TARGET_MODELS = (
    "bus",
    "fire_truck",
    "ambulance",
    "pickup",
)


def main() -> int:
    return run_profile(
        default_models=YOLO_TARGET_MODELS,
        default_prefix="yolo_vehicle_target",
        script_name="spawn_yolo_vehicle_targets.py",
        description=(
            "Randomly spawn large Gazebo vehicle targets selected for both a "
            "fixed-wing nadir camera and a multirotor forward camera. Areas are "
            "read from xd_uav_task_allocate/SearchAreaArray."
        ),
    )


if __name__ == "__main__":
    sys.exit(main())
