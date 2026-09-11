#!/usr/bin/env python3
"""Render the catapult SDF with a per-vehicle ROS namespace.

The PX4 launch file consumes the generated XML through a rosparam and
gazebo_ros/spawn_model's ``-param`` mode.  Keeping the template outside the
binary avoids a second model copy for every UAV while preserving the existing
SDF interface.
"""

import argparse
import copy
import pathlib
import re
import sys
import xml.etree.ElementTree as ET


def read_text(path: str, description: str) -> str:
    try:
        return pathlib.Path(path).read_text(encoding="utf-8")
    except OSError as error:
        raise RuntimeError("unable to read %s: %s" % (description, error))


def named_child(model, tag: str, name: str):
    matches = [child for child in model.findall(tag)
               if child.get("name") == name]
    if len(matches) != 1:
        raise RuntimeError("expected exactly one %s named %s" % (tag, name))
    return matches[0]


def render_flattened(base_text: str, payload_text: str) -> str:
    """Inject payload/catapult elements into the stock plane root model.

    Keeping the PX4 MAVLink interface and its nested airspeed sensor under the
    same root model is important. If the entire stock plane is nested below a
    catapult wrapper, Gazebo's airspeed plugin and MAVLink interface derive
    different transport topic prefixes and PX4 never receives differential
    pressure samples.
    """
    try:
        base_root = ET.fromstring(base_text)
        payload_root = ET.fromstring(payload_text)
    except ET.ParseError as error:
        raise RuntimeError("invalid SDF XML: %s" % error)

    base_model = base_root.find("model")
    payload_model = payload_root.find("model")
    if base_model is None or payload_model is None:
        raise RuntimeError("both SDF files must contain one root model")

    # Fail closed if a different PX4 model is supplied. These elements are the
    # contract that preserves real airspeed and actuator transport.
    named_child(base_model, "link", "base_link")
    named_child(base_model, "joint", "airspeed_joint")
    named_child(base_model, "plugin", "mavlink_interface")

    base_model.set("name", payload_model.get("name", "plane_catapult"))
    payload_link = copy.deepcopy(named_child(payload_model, "link", "payload_link"))
    payload_joint = copy.deepcopy(named_child(payload_model, "joint", "payload_fixed_joint"))
    catapult_plugin = copy.deepcopy(named_child(payload_model, "plugin", "catapult_plugin"))

    parent = payload_joint.find("parent")
    link_name = catapult_plugin.find("link_name")
    if parent is None or link_name is None:
        raise RuntimeError("payload joint or catapult link_name is incomplete")
    parent.text = "base_link"
    link_name.text = "base_link"

    base_model.extend((payload_link, payload_joint, catapult_plugin))
    return "<?xml version=\"1.0\"?>\n" + ET.tostring(
        base_root, encoding="unicode") + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--template", required=True)
    parser.add_argument(
        "--base-sdf", default="",
        help="stock PX4 plane SDF to flatten before adding the payload")
    parser.add_argument("--uav-name", required=True)
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_\-/]*", args.uav_name):
        parser.error("uav-name must be a ROS-safe non-empty name")
    try:
        text = read_text(args.template, "SDF template")
        rendered = text.replace("__UAV_NAME__", args.uav_name)
        if args.base_sdf:
            rendered = render_flattened(
                read_text(args.base_sdf, "base plane SDF"), rendered)
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 2
    sys.stdout.write(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
