#!/usr/bin/env python3
"""Emit a PX4 plane SDF with per-instance identity and MAVLink TCP port."""

import argparse
import sys
import xml.etree.ElementTree as ET


def prepare(path, model_name, tcp_port):
    tree = ET.parse(path)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise ValueError("SDF has no model element")
    model.set("name", model_name)
    plugins = [
        node for node in model.findall("plugin")
        if node.get("name") == "mavlink_interface"
    ]
    if len(plugins) != 1:
        raise ValueError("SDF must contain exactly one mavlink_interface plugin")
    port = plugins[0].find("mavlink_tcp_port")
    if port is None:
        port = ET.SubElement(plugins[0], "mavlink_tcp_port")
    port.text = str(int(tcp_port))
    return ET.tostring(root, encoding="unicode")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    parser.add_argument("model_name")
    parser.add_argument("tcp_port", type=int)
    args = parser.parse_args()
    sys.stdout.write(prepare(args.path, args.model_name, args.tcp_port))


if __name__ == "__main__":
    main()
