"""Display projection for the fixed Davosdorf terrain-planning world frame."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import sys

import numpy as np

from app.terrain import GDAL_OK, _png_bytes, load_terrain
from app.terrain_resource import CANONICAL_DAVOSDORF_DEM


WEB_ROOT = Path(__file__).resolve().parents[2]
# Keep the old names as API/test compatibility aliases, but both Web modes now
# load the one tracked source used by ROS.  The ignored Web handoff copy is not
# part of the runtime contract.
CANONICAL_DEM = CANONICAL_DAVOSDORF_DEM
WEB_DEM = CANONICAL_DEM


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class DavosdorfWorldFrame:
    """Map local ROS world metres to WGS84 solely for browser display.

    The official terrain planner loads the raster into a grid_map centred at
    (0, 0). The raster CRS remains the authority for geographic display; this
    class does not use or replace the per-aircraft datum in xd_uav_detect.
    """

    def __init__(self):
        if not GDAL_OK:
            raise RuntimeError("GDAL is required for the fixed DEM display projection")
        if not CANONICAL_DEM.is_file():
            raise RuntimeError(
                "Canonical Davosdorf DEM is missing: {}".format(CANONICAL_DEM))
        from osgeo import osr

        # Browser display must remain deterministic and offline. If an
        # optional high-accuracy datum grid is absent, PROJ uses its packaged
        # fallback instead of attempting a network download at runtime.
        if hasattr(osr, "SetPROJEnableNetwork"):
            osr.SetPROJEnableNetwork(False)

        # Relocatable Conda prefixes can retain a stale PROJ search entry
        # before their valid sys.prefix/share/proj directory. Select only
        # directories that actually contain the database, preferring the
        # active environment so its schema matches the loaded libproj.
        proj_candidates = [Path(sys.prefix) / "share" / "proj"]
        if hasattr(osr, "GetPROJSearchPaths"):
            proj_candidates.extend(Path(path) for path in osr.GetPROJSearchPaths())
        proj_paths = []
        for candidate in proj_candidates:
            if (candidate / "proj.db").is_file() and str(candidate) not in proj_paths:
                proj_paths.append(str(candidate))
        if proj_paths and hasattr(osr, "SetPROJSearchPaths"):
            osr.SetPROJSearchPaths(proj_paths)

        self.terrain = load_terrain(str(CANONICAL_DEM))
        transform = self.terrain.geo_transform
        if abs(transform[2]) > 1.0e-12 or abs(transform[4]) > 1.0e-12:
            raise RuntimeError("rotated DEM geotransforms are not supported")
        self.center_easting = transform[0] + transform[1] * self.terrain.cols / 2.0
        self.center_northing = transform[3] + transform[5] * self.terrain.rows / 2.0
        self.width_m = abs(transform[1] * self.terrain.cols)
        self.height_m = abs(transform[5] * self.terrain.rows)
        projected = osr.SpatialReference()
        if projected.ImportFromWkt(self.terrain.wkt) != 0:
            raise RuntimeError("DEM projection WKT is invalid")
        geographic = osr.SpatialReference()
        if geographic.ImportFromEPSG(4326) != 0:
            raise RuntimeError("failed to initialize EPSG:4326")
        if hasattr(osr, "OAMS_TRADITIONAL_GIS_ORDER"):
            projected.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
            geographic.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        self._to_geo = osr.CoordinateTransformation(projected, geographic)
        self._to_projected = osr.CoordinateTransformation(geographic, projected)

        self.canonical_dem_sha256 = _sha256(CANONICAL_DEM)
        # Retain the response fields consumed by the existing frontend.  They
        # now describe a single shared file instead of comparing two copies.
        self.web_dem_sha256 = self.canonical_dem_sha256
        self.canonical_match = True

    def world_to_geo(self, x, y):
        if not all(math.isfinite(value) for value in (x, y)):
            raise ValueError("world coordinates must be finite")
        longitude, latitude, _ = self._to_geo.TransformPoint(
            self.center_easting + x, self.center_northing + y, 0.0)
        if not (-180.0 <= longitude <= 180.0 and -90.0 <= latitude <= 90.0):
            raise ValueError("projection produced an invalid WGS84 position")
        return longitude, latitude

    def geo_to_world(self, longitude, latitude):
        if (not all(math.isfinite(value) for value in (longitude, latitude))
                or not -180.0 <= longitude <= 180.0
                or not -90.0 <= latitude <= 90.0):
            raise ValueError("WGS84 coordinates are invalid")
        easting, northing, _ = self._to_projected.TransformPoint(
            longitude, latitude, 0.0)
        return easting - self.center_easting, northing - self.center_northing

    def _geo_bounds(self):
        points = [
            self.world_to_geo(x, y)
            for x in (-self.width_m / 2.0, self.width_m / 2.0)
            for y in (-self.height_m / 2.0, self.height_m / 2.0)
        ]
        return {
            "west": min(point[0] for point in points),
            "south": min(point[1] for point in points),
            "east": max(point[0] for point in points),
            "north": max(point[1] for point in points),
        }

    def display_points(self, points):
        output = []
        for x, y, z in points:
            longitude, latitude = self.world_to_geo(x, y)
            output.append({
                "x": x,
                "y": y,
                "z": z,
                "lng": longitude,
                "lat": latitude,
            })
        return output

    def elevation_at_world(self, x, y):
        if not all(math.isfinite(value) for value in (x, y)):
            raise ValueError("world coordinates must be finite")
        row, column = self._grid_index_at_world(x, y)
        elevation = float(self.terrain.elevation[row, column])
        if not math.isfinite(elevation):
            raise ValueError("DEM elevation is unavailable at this coordinate")
        return elevation

    def _grid_index_at_world(self, x, y):
        transform = self.terrain.geo_transform
        easting = self.center_easting + x
        northing = self.center_northing + y
        column = int(round((easting - transform[0]) / transform[1] - 0.5))
        row = int(round((northing - transform[3]) / transform[5] - 0.5))
        if not 0 <= row < self.terrain.rows or not 0 <= column < self.terrain.cols:
            raise ValueError("world coordinate is outside the DEM")
        return row, column

    def launch_site_metrics(self, x, y):
        """Evaluate a fixed-wing ground start against the canonical DEM."""
        row, column = self._grid_index_at_world(x, y)
        if (row < 1 or row + 1 >= self.terrain.rows
                or column < 1 or column + 1 >= self.terrain.cols):
            raise ValueError("launch site is too close to the DEM boundary")
        transform = self.terrain.geo_transform
        values = self.terrain.elevation
        neighbourhood = values[row - 1:row + 2, column - 1:column + 2]
        if not np.isfinite(neighbourhood).all():
            raise ValueError("launch site contains unavailable DEM elevations")
        dz_dx = ((values[row, column + 1] - values[row, column - 1])
                 / (2.0 * abs(transform[1])))
        # Raster rows increase southward while world Y increases northward.
        dz_dy = ((values[row - 1, column] - values[row + 1, column])
                 / (2.0 * abs(transform[5])))
        return {
            "ground_elevation_msl_m": float(values[row, column]),
            "slope_rad": math.atan(math.hypot(float(dz_dx), float(dz_dy))),
            "relief_3x3_m": float(np.max(neighbourhood) - np.min(neighbourhood)),
        }

    def validate_launch_site(self, x, y, maximum_slope_rad=0.10,
                             maximum_relief_m=3.0):
        metrics = self.launch_site_metrics(x, y)
        if metrics["slope_rad"] > maximum_slope_rad:
            raise ValueError(
                "launch site slope {:.3f} rad exceeds {:.3f} rad".format(
                    metrics["slope_rad"], maximum_slope_rad))
        if metrics["relief_3x3_m"] > maximum_relief_m:
            raise ValueError(
                "launch site local relief {:.2f} m exceeds {:.2f} m".format(
                    metrics["relief_3x3_m"], maximum_relief_m))
        return metrics

    def nearest_launch_site(self, nominal_x, nominal_y, search_radius_m=300.0):
        """Find the nearest DEM cell that passes the same public spawn gate."""
        resolution = min(
            abs(self.terrain.geo_transform[1]),
            abs(self.terrain.geo_transform[5]))
        steps = int(math.floor(search_radius_m / resolution))
        offsets = [
            (math.hypot(dx, dy), abs(dy), abs(dx), dx, dy)
            for row in range(-steps, steps + 1)
            for column in range(-steps, steps + 1)
            for dx, dy in ((column * resolution, row * resolution),)
            if math.hypot(dx, dy) <= search_radius_m
        ]
        for _, _, _, dx, dy in sorted(offsets):
            x = nominal_x + dx
            y = nominal_y + dy
            try:
                metrics = self.validate_launch_site(x, y)
                return x, y, metrics
            except ValueError:
                continue
        raise ValueError("no terrain-safe launch site exists near the nominal point")

    def vehicle_examples(self, count=8):
        examples = []
        for index in range(int(count)):
            name = "uav{}".format(index + 1)
            nominal_x = 600.0
            # Preserve the three flight-verified routes. Later examples use a
            # longer leg so their DEM endpoint elevation does not itself
            # exceed the fixed-wing 0.15 rad climb/descent contract.
            end_x = 1200.0 if index < 3 else 2400.0
            nominal_y = -2000.0 + 300.0 * index
            start_x, y, launch_site = self.nearest_launch_site(
                nominal_x, nominal_y)
            start_ground = launch_site["ground_elevation_msl_m"]
            end_ground = self.elevation_at_world(end_x, y)
            route = {
                "uav_name": name,
                "goal_id": 6201 + index,
                "nominal_speed_mps": 15.0,
                "points": [
                    {"x": start_x, "y": y, "z": start_ground + 80.0},
                    {"x": end_x, "y": y, "z": end_ground + 80.0},
                ],
            }
            route["display_points"] = self.display_points([
                (point["x"], point["y"], point["z"])
                for point in route["points"]
            ])
            examples.append({
                "uav_name": name,
                "world_origin": {"x": start_x, "y": y, "z": start_ground},
                "launch_site": launch_site,
                "gazebo_spawn": {
                    "x": start_x,
                    "y": y,
                    "z": start_ground,
                },
                "route": route,
            })
        return examples

    def metadata(self):
        examples = self.vehicle_examples()
        defaults = [example["route"] for example in examples[:3]]
        return {
            "frame_id": "world",
            "projection": "EPSG:21781",
            "display_projection": "EPSG:4326",
            "world_origin_projected": {
                "easting": self.center_easting,
                "northing": self.center_northing,
            },
            "world_bounds": {
                "x_min": -self.width_m / 2.0,
                "y_min": -self.height_m / 2.0,
                "x_max": self.width_m / 2.0,
                "y_max": self.height_m / 2.0,
            },
            "geo_bounds": self._geo_bounds(),
            "elevation_min_m": float(np.nanmin(self.terrain.elevation)),
            "elevation_max_m": float(np.nanmax(self.terrain.elevation)),
            "gazebo_frame": {
                "horizontal_mapping": "gazebo_x_y_equals_world_x_y",
                "vertical_mapping": "gazebo_z_equals_elevation_msl_m",
            },
            "launch_site_policy": {
                "maximum_slope_rad": 0.10,
                "maximum_relief_3x3_m": 3.0,
                "selection": "nearest_valid_dem_cell_within_300m",
            },
            "web_dem_sha256": self.web_dem_sha256,
            "canonical_dem_sha256": self.canonical_dem_sha256,
            "canonical_dem_match": self.canonical_match,
            "default_routes": defaults,
            "vehicle_examples": examples,
        }

    def raster_payload(self, max_grid=180):
        """Return a bounded world-frame DEM grid for the mission 3-D view."""
        limit = max(32, min(int(max_grid), 320))
        row_indices = np.linspace(
            0, self.terrain.rows - 1,
            min(self.terrain.rows, limit), dtype=int)
        column_indices = np.linspace(
            0, self.terrain.cols - 1,
            min(self.terrain.cols, limit), dtype=int)
        elevation = self.terrain.elevation[np.ix_(row_indices, column_indices)]
        return {
            "frame_id": "world",
            "vertical_datum": "MSL",
            "rows": int(elevation.shape[0]),
            "cols": int(elevation.shape[1]),
            "width_m": self.width_m,
            "height_m": self.height_m,
            "world_bounds": {
                "x_min": -self.width_m / 2.0,
                "y_min": -self.height_m / 2.0,
                "x_max": self.width_m / 2.0,
                "y_max": self.height_m / 2.0,
            },
            "elevation_min_m": float(np.nanmin(self.terrain.elevation)),
            "elevation_max_m": float(np.nanmax(self.terrain.elevation)),
            "elevation": [
                [None if np.isnan(value) else round(float(value), 2)
                 for value in row]
                for row in elevation
            ],
        }

    def overlay_png(self):
        return _png_bytes(self.terrain.hillshade_rgba(max_size=1600))
