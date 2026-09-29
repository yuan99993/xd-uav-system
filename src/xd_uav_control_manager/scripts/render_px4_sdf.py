#!/usr/bin/env python3
"""Render or retarget a PX4 Gazebo Classic SDF on stdout."""

import argparse
import os
import xml.etree.ElementTree as ET

import jinja2
import numpy as np
import rospkg


def render_template(path, environment_root, arguments):
    environment = jinja2.Environment(
        loader=jinja2.FileSystemLoader(environment_root),
        undefined=jinja2.StrictUndefined)
    template = environment.get_template(os.path.relpath(path, environment_root))
    return template.render({
        "np": np,
        "rospack": rospkg.RosPack(),
        "mavlink_tcp_port": arguments.mavlink_tcp_port,
        "mavlink_udp_port": arguments.mavlink_udp_port,
        "serial_enabled": 0,
        "serial_device": "/dev/ttyACM0",
        "serial_baudrate": 921600,
        "mavlink_id": arguments.mavlink_id,
        "cam_component_id": 100,
        "gst_udp_port": arguments.gst_udp_port,
        "video_uri": arguments.video_uri,
        "mavlink_cam_udp_port": arguments.mavlink_cam_udp_port,
        "hil_mode": 0,
        "ros_version": int(os.environ.get("ROS_VERSION", "1")),
    })


def retarget_static_sdf(path, arguments):
    tree = ET.parse(path)
    root = tree.getroot()
    replacements = {
        "mavlink_tcp_port": str(arguments.mavlink_tcp_port),
        "mavlink_udp_port": str(arguments.mavlink_udp_port),
        "mavlink_id": str(arguments.mavlink_id),
        "gst_udp_port": str(arguments.gst_udp_port),
        "video_uri": str(arguments.video_uri),
        "mavlink_cam_udp_port": str(arguments.mavlink_cam_udp_port),
    }
    found_interface = False
    for plugin in root.iter("plugin"):
        plugin_name = plugin.attrib.get("name", "").lower()
        filename = plugin.attrib.get("filename", "").lower()
        if "mavlink" not in plugin_name and "mavlink" not in filename:
            continue
        found_interface = True
        for tag, value in replacements.items():
            child = plugin.find(tag)
            if child is not None:
                child.text = value
    if not found_interface:
        raise RuntimeError("SDF has no MAVLink interface plugin: " + path)
    return ET.tostring(root, encoding="unicode")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--environment-root", required=True)
    parser.add_argument("--mavlink-tcp-port", type=int, required=True)
    parser.add_argument("--mavlink-udp-port", type=int, required=True)
    parser.add_argument("--mavlink-id", type=int, required=True)
    parser.add_argument("--gst-udp-port", type=int, required=True)
    parser.add_argument("--video-uri", required=True)
    parser.add_argument("--mavlink-cam-udp-port", type=int, required=True)
    arguments = parser.parse_args()

    if arguments.source.endswith(".sdf.jinja"):
        rendered = render_template(
            arguments.source, arguments.environment_root, arguments)
    else:
        rendered = retarget_static_sdf(arguments.source, arguments)
    print(rendered)


if __name__ == "__main__":
    main()
