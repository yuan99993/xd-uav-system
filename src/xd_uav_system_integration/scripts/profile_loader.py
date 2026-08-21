#!/usr/bin/env python3
"""Validate a profile and render its component graph as roslaunch XML."""

import argparse
import json
import sys

import yaml

from xd_uav_system_integration.core import validate_profile
from xd_uav_system_integration.profile import (
    build_launch_plan,
    render_roslaunch,
    validate_component_profile,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("profile")
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument(
        "--emit-launch", metavar="PATH",
        help="write validated schema-v2 component graph as roslaunch XML")
    parser.add_argument(
        "--stage", choices=("base", "runtime", "all"), default="base",
        help="schema-v2 stage to display or emit; default: base")
    arguments = parser.parse_args()
    with open(arguments.profile, "r", encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict):
        print("profile root must be a mapping", file=sys.stderr)
        return 2
    schema_version = profile.get("schema_version")
    errors = (validate_component_profile(profile) if schema_version == 2
              else validate_profile(profile))
    if errors:
        for error in errors:
            print("ERROR: " + error, file=sys.stderr)
        return 2
    summary = {
        "valid": True,
        "dry_run": arguments.dry_run,
        "vehicles": [item["name"] for item in profile["vehicles"]],
        "reference_owner": profile["control"].get(
            "initial_owner", profile["control"].get("reference_owner")),
        "common_frame": profile.get("frames", {}).get("common"),
    }
    if schema_version == 2:
        plan = build_launch_plan(profile)
        summary["stage"] = arguments.stage
        summary["components"] = [
            item.component_id for item in plan
            if arguments.stage == "all" or item.stage == arguments.stage]
        if arguments.emit_launch:
            with open(arguments.emit_launch, "w", encoding="utf-8") as stream:
                stream.write(render_roslaunch(plan, arguments.stage))
            summary["launch_file"] = arguments.emit_launch
    print(json.dumps(summary, sort_keys=True))
    if schema_version == 1 and not arguments.dry_run:
        print("schema v1 is validation-only; use schema v2 for launch generation",
              file=sys.stderr)
        return 3
    if schema_version == 2 and not arguments.dry_run and not arguments.emit_launch:
        print("choose --dry-run or --emit-launch PATH", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
