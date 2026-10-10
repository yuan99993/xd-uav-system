#!/usr/bin/env python3
"""Build a disposable Gazebo world from an authoritative terrain package."""

import argparse
import json
from pathlib import Path
import shutil
import tempfile

from prepare_gazebo_dem import prepare


def _write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def build(manifest_path, runtime_dir, samples):
    manifest_path = manifest_path.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "xd-uav-terrain-package/v1":
        raise RuntimeError("unsupported terrain package schema")
    if manifest.get("vertical_datum") != "MSL":
        raise RuntimeError("terrain package vertical datum must be MSL")
    source = Path(manifest["canonical_dem"]).resolve()
    if not source.is_file():
        raise RuntimeError("terrain package GeoTIFF is missing")

    staging = Path(tempfile.mkdtemp(prefix="terrain-runtime-",
                                    dir=str(runtime_dir.parent.resolve())))
    try:
        model = staging / "models" / "active_terrain"
        heightmap = model / "materials" / "heightmaps" / "terrain.tif"
        gazebo_manifest = model / "terrain-manifest.json"
        prepare(source, heightmap, gazebo_manifest, samples, "MSL")
        prepared = json.loads(gazebo_manifest.read_text(encoding="utf-8"))
        geometry = prepared["gazebo_heightmap"]
        size = "{} {} {}".format(
            geometry["size_x_m"], geometry["size_y_m"],
            geometry["size_z_m"])
        sdf = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="active_terrain">
    <static>true</static>
    <link name="terrain">
      <collision name="terrain_collision"><geometry><heightmap>
        <uri>model://active_terrain/materials/heightmaps/terrain.tif</uri>
        <size>{size}</size><pos>0 0 0</pos><sampling>1</sampling>
      </heightmap></geometry></collision>
      <visual name="terrain_visual"><cast_shadows>true</cast_shadows><geometry><heightmap>
        <uri>model://active_terrain/materials/heightmaps/terrain.tif</uri>
        <size>{size}</size><pos>0 0 0</pos>
        <texture><diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse><normal>file://media/materials/textures/flat_normal.png</normal><size>80</size></texture>
        <blend><min_height>0</min_height><fade_dist>180</fade_dist></blend><sampling>1</sampling>
      </heightmap></geometry></visual>
    </link>
  </model>
</sdf>
""".format(size=size)
        _write(model / "model.sdf", sdf)
        _write(model / "model.config", """<?xml version="1.0"?>
<model><name>Active mission terrain</name><version>1.0</version>
<sdf version="1.6">model.sdf</sdf>
<description>Generated from the active xd-uav terrain package.</description></model>
""")
        camera_distance = max(geometry["size_x_m"], geometry["size_y_m"])
        world = """<?xml version="1.0"?>
<sdf version="1.6"><world name="terrain_mission">
  <include><uri>model://sun</uri></include>
  <include><uri>model://active_terrain</uri></include>
  <physics name="fixedwing_sitl" type="ode"><real_time_update_rate>250</real_time_update_rate><max_step_size>0.004</max_step_size></physics>
  <scene><ambient>0.42 0.46 0.50 1</ambient><background>0.55 0.68 0.78 1</background><shadows>true</shadows></scene>
  <gui fullscreen="0"><camera name="overview"><pose>{d} -{d} {z} 0 0.48 2.35</pose></camera></gui>
</world></sdf>
""".format(d=camera_distance * 0.7, z=camera_distance * 0.55)
        _write(staging / "terrain_mission.world", world)
        _write(staging / "terrain-package.json",
               json.dumps(manifest, ensure_ascii=False, indent=2,
                          sort_keys=True) + "\n")
        if runtime_dir.exists():
            shutil.rmtree(runtime_dir)
        staging.replace(runtime_dir)
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--runtime-dir", required=True, type=Path)
    parser.add_argument("--samples", type=int, default=513)
    args = parser.parse_args()
    args.runtime_dir.parent.mkdir(parents=True, exist_ok=True)
    build(args.manifest, args.runtime_dir.resolve(), args.samples)


if __name__ == "__main__":
    main()
