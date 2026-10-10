#!/usr/bin/env python3
"""Fail closed when a Gazebo DEM cache no longer matches its authority."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify(source, prepared, manifest_path, model_sdf):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "xd-uav-gazebo-dem/v1":
        raise RuntimeError("unsupported Gazebo DEM manifest schema")
    if manifest.get("vertical_datum") != "MSL":
        raise RuntimeError("Gazebo DEM vertical datum must be MSL")
    for path, key in ((source, "source"), (prepared, "prepared")):
        expected = manifest[key]
        if path.name != expected["name"]:
            raise RuntimeError("{} filename does not match manifest".format(key))
        if _sha256(path) != expected["sha256"]:
            raise RuntimeError("{} SHA-256 does not match manifest".format(key))

    geometry = manifest["gazebo_heightmap"]
    expected_size = [geometry["size_x_m"], geometry["size_y_m"],
                     geometry["size_z_m"]]
    expected_position = geometry["position_z_m"]
    heightmaps = ET.parse(model_sdf).getroot().findall(".//heightmap")
    if len(heightmaps) != 2:
        raise RuntimeError("model must define matching collision and visual heightmaps")
    for heightmap in heightmaps:
        uri = heightmap.findtext("uri", default="")
        if not uri.endswith("/" + prepared.name):
            raise RuntimeError("model heightmap URI does not match prepared DEM")
        size = [float(value) for value in heightmap.findtext("size").split()]
        position = [float(value) for value in heightmap.findtext("pos").split()]
        if len(size) != 3 or not all(
                math.isclose(actual, expected, abs_tol=1.0e-6)
                for actual, expected in zip(size, expected_size)):
            raise RuntimeError("model heightmap size does not match manifest")
        if len(position) != 3 or not math.isclose(
                position[2], expected_position, abs_tol=1.0e-6):
            raise RuntimeError("model heightmap position does not match manifest")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--prepared", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--model-sdf", required=True, type=Path)
    args = parser.parse_args()
    verify(args.source.resolve(), args.prepared.resolve(),
           args.manifest.resolve(), args.model_sdf.resolve())
    print("Gazebo DEM manifest verified")


if __name__ == "__main__":
    main()
