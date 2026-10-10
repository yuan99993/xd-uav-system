"""Regression tests for arbitrary WGS84 DEM activation; no network or ROS."""

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np


BACKEND = Path(__file__).resolve().parent
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))

from app.terrain import TerrainMap
from app.terrain_mission import activate_terrain, wgs84_wkt
from swarm_app.terrain_frame import TerrainWorldFrame


class TerrainMissionPackageTest(unittest.TestCase):
    def test_wgs84_grid_becomes_projected_authoritative_package(self):
        rows, cols = 48, 64
        elevation = np.full((rows, cols), 1234.5, dtype=np.float32)
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
                frame = TerrainWorldFrame(manifest)
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
