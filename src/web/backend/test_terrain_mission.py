"""Regression tests for arbitrary WGS84 DEM activation; no network or ROS."""

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from osgeo import gdal


BACKEND = Path(__file__).resolve().parent
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))
PLANNING_TOOLS = BACKEND.parents[1] / "xd_uav_planning" / "scripts" / "tools"
if str(PLANNING_TOOLS) not in sys.path:
    sys.path.insert(0, str(PLANNING_TOOLS))

from app.terrain import TerrainMap
from app.terrain_mission import activate_terrain, wgs84_wkt
from swarm_app.terrain_frame import TerrainWorldFrame
from prepare_gazebo_dem import prepare as prepare_gazebo_dem
from prepare_terrain_runtime import _overview_camera_pose


class TerrainMissionPackageTest(unittest.TestCase):
    def test_large_terrain_keeps_camera_inside_a_local_render_distance(self):
        pose = _overview_camera_pose(
            {"size_x_m": 98000.0, "size_y_m": 97000.0},
            {"prepared_min": 340.0, "prepared_max": 2640.0})
        self.assertEqual((4200.0, -4200.0, 4790.0), pose)

    def test_wgs84_grid_becomes_projected_authoritative_package(self):
        rows, cols = 48, 64
        elevation = (1234.5
                     + np.arange(rows, dtype=np.float32)[:, None] * 0.4
                     + np.arange(cols, dtype=np.float32)[None, :] * 0.2)
        terrain = TerrainMap(
            elevation=elevation,
            geo_transform=(103.70, 0.001, 0.0, 31.10, 0.0, -0.001),
            wkt=wgs84_wkt(), rows=rows, cols=cols)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch("app.terrain_mission.RUNTIME_DIR", root), patch(
                    "app.terrain_mission.ACTIVE_MANIFEST", root / "active.json"):
                manifest = activate_terrain(
                    terrain, "arbitrary-online-region",
                    {"source": "online_terrarium", "vertical_datum": "MSL"})
                active = json.loads((root / "active.json").read_text())
                self.assertEqual(manifest["terrain_id"], active["terrain_id"])
                self.assertTrue(manifest["horizontal_crs"]["authority"].startswith("EPSG:326"))
                self.assertEqual("MSL", manifest["vertical_datum"])
                self.assertTrue(Path(manifest["canonical_dem"]).is_file())
                gazebo_dem = root / "gazebo.tif"
                prepare_gazebo_dem(
                    Path(manifest["canonical_dem"]), gazebo_dem,
                    root / "gazebo.json", 513, "MSL")
                prepared = gdal.Open(str(gazebo_dem), gdal.GA_ReadOnly)
                self.assertIsNone(prepared.GetRasterBand(1).GetNoDataValue())
                self.assertTrue(np.isfinite(prepared.ReadAsArray()).all())
                frame = TerrainWorldFrame(manifest)
                x, y = 137.25, -92.5
                column = ((x + frame.width_m / 2.0) / frame.width_m
                          * (prepared.RasterXSize - 1))
                row = ((frame.height_m / 2.0 - y) / frame.height_m
                       * (prepared.RasterYSize - 1))
                column0, row0 = math.floor(column), math.floor(row)
                x_fraction, y_fraction = column - column0, row - row0
                values = prepared.ReadAsArray()
                expected_surface = (
                    (1.0 - y_fraction) * (
                        (1.0 - x_fraction) * values[row0, column0]
                        + x_fraction * values[row0, column0 + 1])
                    + y_fraction * (
                        (1.0 - x_fraction) * values[row0 + 1, column0]
                        + x_fraction * values[row0 + 1, column0 + 1]))
                self.assertAlmostEqual(
                    float(expected_surface),
                    frame.gazebo_spawn_elevation_at_world(x, y), places=5)
                metadata = frame.metadata()
                self.assertEqual(manifest["terrain_id"], metadata["terrain_id"])
                self.assertFalse(metadata["sample_terrain"])
                self.assertEqual(8, len(metadata["vehicle_examples"]))
                longitude, latitude = frame.world_to_geo(0.0, 0.0)
                x, y = frame.geo_to_world(longitude, latitude)
                self.assertAlmostEqual(0.0, x, places=3)
                self.assertAlmostEqual(0.0, y, places=3)

    def test_nodata_fails_closed(self):
        elevation = np.ones((8, 8), dtype=np.float32)
        elevation[2, 3] = np.nan
        terrain = TerrainMap(
            elevation=elevation,
            geo_transform=(103.7, 0.001, 0.0, 31.1, 0.0, -0.001),
            wkt=wgs84_wkt(), rows=8, cols=8)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch("app.terrain_mission.RUNTIME_DIR", root), patch(
                    "app.terrain_mission.ACTIVE_MANIFEST", root / "active.json"):
                with self.assertRaisesRegex(ValueError, "NoData"):
                    activate_terrain(terrain, "invalid", {"source": "test"})


if __name__ == "__main__":
    unittest.main()
