"""Terrain loading contract tests; no network access required."""

from pathlib import Path
import sys
import unittest

from fastapi import HTTPException


BACKEND = Path(__file__).resolve().parent
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))

from app import main


class TerrainGridTest(unittest.TestCase):
    def setUp(self):
        self.previous = (
            main._current_terrain,
            main._current_terrain_name,
            main._current_terrain_meta,
        )
        main._current_terrain = None
        main._current_terrain_name = None
        main._current_terrain_meta = {}

    def tearDown(self):
        (
            main._current_terrain,
            main._current_terrain_name,
            main._current_terrain_meta,
        ) = self.previous

    def request(self, elevation):
        return main.LoadGridRequest(
            elevation=elevation,
            x_min=8.0,
            y_min=46.0,
            x_max=8.1,
            y_max=46.1,
            zoom=12,
            n_tiles=1,
        )

    def test_all_nodata_grid_is_rejected_without_creating_terrain(self):
        with self.assertRaises(HTTPException) as raised:
            main.load_grid(self.request([[None, None], [None, None]]))
        self.assertEqual(400, raised.exception.status_code)
        self.assertIsNone(main._current_terrain)

    def test_valid_grid_enables_raster_and_planning_state(self):
        info = main.load_grid(self.request([[1000.0, 1010.0], [1020.0, 1030.0]]))
        self.assertEqual("online_terrarium", info["terrain_meta"]["source"])
        self.assertEqual(1000.0, info["elevation_min"])
        self.assertEqual(1030.0, info["elevation_max"])
        raster = main.terrain_raster(max_grid=32)
        self.assertEqual(2, raster["rows"])
        self.assertEqual(2, raster["cols"])


if __name__ == "__main__":
    unittest.main()
