#!/usr/bin/env python3
"""Render/check the shared EO payload in committed runtime SDF snapshots."""

import argparse
from pathlib import Path
import sys

from jinja2 import Environment, FileSystemLoader, StrictUndefined


PROFILES = (
    {
        "files": ("x500_gimbal/model.sdf", "x500_gimbal/sensor_demo.sdf"),
        "parent_link": "base_link",
        "mesh_uri_prefix": "model://x500_gimbal/meshes",
        "ros_namespace": "/uav1",
        "frame_prefix": "uav1",
        "mount_pose": "0.0918 0.0015 0.02 0 0 0",
        "yaw_pose": "0.0658 0.0015 -0.08 0 0 0",
        "pitch_pose": "0 0 -0.1441 0 0 0",
    },
    {
        "files": ("plane_gimbal/plane.sdf",),
        "parent_link": "base_link",
        "mesh_uri_prefix": "model://plane_gimbal/meshes",
        "ros_namespace": "/uav1",
        "frame_prefix": "uav1",
        "mount_pose": "0.0918 0.0015 -0.0159 0 0 0",
        "yaw_pose": "0.0658 0.0015 -0.1159 0 0 0",
        "pitch_pose": "0 0 -0.18 0 0 0",
    },
)


def replace_payload(document, rendered):
    markers = (
        "<!-- XD_UAV_DETECT_EO_GIMBAL_BEGIN -->",
        "<!-- xd_uav_detect EO payload:",
    )
    starts = [document.find(marker) for marker in markers]
    starts = [position for position in starts if position >= 0]
    if not starts:
        raise ValueError("payload start marker not found")
    start = min(starts)
    end = document.rfind("\n  </model>")
    if end <= start:
        raise ValueError("model closing tag not found after payload")
    return document[:start] + rendered.rstrip() + document[end:]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--write", action="store_true",
        help="update snapshots; default behavior only checks for drift")
    args = parser.parse_args()

    models_dir = Path(__file__).resolve().parents[2] / "models"
    environment = Environment(
        loader=FileSystemLoader(str(models_dir)),
        undefined=StrictUndefined,
        autoescape=False,
        keep_trailing_newline=True,
    )
    payload_macro = environment.get_template(
        "common/eo_gimbal.sdf.jinja").module.eo_gimbal

    drifted = []
    for profile in PROFILES:
        render_args = {key: value for key, value in profile.items()
                       if key != "files"}
        rendered = payload_macro(
            **render_args, camera_enabled=True, range_enabled=True,
            roll_enabled=False)
        for relative_path in profile["files"]:
            path = models_dir / relative_path
            original = path.read_text(encoding="utf-8")
            expected = replace_payload(original, rendered)
            if expected == original:
                continue
            drifted.append(relative_path)
            if args.write:
                path.write_text(expected, encoding="utf-8")

    if drifted and not args.write:
        print("Generated payload snapshots are stale:", file=sys.stderr)
        for path in drifted:
            print("  " + path, file=sys.stderr)
        print("Run render_payload_models.py --write", file=sys.stderr)
        return 1
    if drifted:
        print("Updated " + ", ".join(drifted))
    else:
        print("Payload snapshots are consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
