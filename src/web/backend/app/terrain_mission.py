"""Create and resolve the authoritative terrain package shared by Web and ROS.

Browser DEM tiles are only a transport.  An activated mission always owns a
projected, metre-based GeoTIFF plus a manifest with an immutable content ID.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Optional

import numpy as np

from .terrain import GDAL_OK, TerrainMap


SCHEMA = "xd-uav-terrain-package/v1"
RUNTIME_DIR = Path(os.environ.get(
    "XD_UAV_TERRAIN_RUNTIME",
    "/tmp/xd-uav-terrain-mission-{}".format(os.getuid()),
))
ACTIVE_MANIFEST = RUNTIME_DIR / "active.json"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _configure_proj(osr) -> None:
    if hasattr(osr, "SetPROJEnableNetwork"):
        osr.SetPROJEnableNetwork(False)
    candidates = [Path(sys.prefix) / "share" / "proj"]
    if hasattr(osr, "GetPROJSearchPaths"):
        candidates.extend(Path(value) for value in osr.GetPROJSearchPaths())
    valid = []
    for candidate in candidates:
        if (candidate / "proj.db").is_file() and str(candidate) not in valid:
            valid.append(str(candidate))
    if not valid:
        raise RuntimeError("PROJ database is unavailable")
    if hasattr(osr, "SetPROJSearchPaths"):
        osr.SetPROJSearchPaths(valid)


def configure_projection_runtime() -> None:
    """Select the PROJ database belonging to the active Web environment."""
    if not GDAL_OK:
        return
    from osgeo import gdal, osr
    _configure_proj(osr)
    paths = osr.GetPROJSearchPaths() if hasattr(osr, "GetPROJSearchPaths") else []
    if paths:
        gdal.SetConfigOption("PROJ_LIB", paths[0])


def wgs84_wkt() -> str:
    if not GDAL_OK:
        raise RuntimeError("GDAL is required to define the WGS84 terrain CRS")
    from osgeo import osr
    configure_projection_runtime()
    spatial_ref = _axis_order(osr.SpatialReference(), osr)
    if spatial_ref.ImportFromEPSG(4326) != 0:
        raise RuntimeError("failed to initialize EPSG:4326")
    return spatial_ref.ExportToWkt()


def _axis_order(spatial_ref, osr):
    if hasattr(osr, "OAMS_TRADITIONAL_GIS_ORDER"):
        spatial_ref.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    return spatial_ref


def _authority(spatial_ref) -> Optional[str]:
    spatial_ref.AutoIdentifyEPSG()
    name = spatial_ref.GetAuthorityName(None)
    code = spatial_ref.GetAuthorityCode(None)
    return "{}:{}".format(name, code) if name and code else None


def _write_json_atomic(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", prefix=path.name + ".",
            suffix=".tmp", dir=str(path.parent), delete=False) as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def _source_spatial_ref(terrain: TerrainMap, osr):
    source = _axis_order(osr.SpatialReference(), osr)
    if not terrain.wkt or source.ImportFromWkt(terrain.wkt) != 0:
        raise ValueError("terrain must declare a valid horizontal CRS")
    if not (source.IsGeographic() or source.IsProjected()):
        raise ValueError("terrain CRS must be geographic or projected")
    return source


def _corners(terrain: TerrainMap):
    gt = terrain.geo_transform
    return [
        (gt[0] + column * gt[1] + row * gt[2],
         gt[3] + column * gt[4] + row * gt[5])
        for column in (0, terrain.cols)
        for row in (0, terrain.rows)
    ]


def _largest_valid_rectangle(valid: np.ndarray):
    """Return top, left, bottom, right for the largest all-True rectangle."""
    heights = np.zeros(valid.shape[1], dtype=np.int64)
    best = (0, 0, 0, 0)
    best_area = 0
    for row in range(valid.shape[0]):
        heights = np.where(valid[row], heights + 1, 0)
        stack = []
        for column in range(valid.shape[1] + 1):
            height = int(heights[column]) if column < valid.shape[1] else 0
            start = column
            while stack and stack[-1][1] > height:
                left, previous_height = stack.pop()
                area = previous_height * (column - left)
                if area > best_area:
                    best_area = area
                    best = (row + 1 - previous_height, left,
                            row + 1, column)
                start = left
            if height and (not stack or stack[-1][1] < height):
                stack.append((start, height))
    return best


def activate_terrain(terrain: TerrainMap, name: str, metadata: Optional[dict] = None) -> dict:
    """Canonicalize the in-memory terrain and atomically make it active."""
    if not GDAL_OK:
        raise RuntimeError("GDAL is required to activate a terrain for ROS/Gazebo")
    from osgeo import gdal, osr

    gdal.UseExceptions()
    _configure_proj(osr)
    values = np.asarray(terrain.elevation, dtype=np.float32)
    if values.ndim != 2 or values.shape != (terrain.rows, terrain.cols):
        raise ValueError("terrain elevation matrix shape is invalid")
    if not np.isfinite(values).all():
        raise ValueError(
            "terrain contains NoData or non-finite cells; choose a fully covered area")
    metadata = dict(metadata or {})
    if metadata.get("vertical_datum") != "MSL":
        raise ValueError("terrain vertical datum must be explicitly declared as MSL")
    gt = tuple(float(value) for value in terrain.geo_transform)
    if abs(gt[2]) > 1.0e-12 or abs(gt[4]) > 1.0e-12 or gt[1] <= 0 or gt[5] >= 0:
        raise ValueError("terrain must be a north-up georeferenced raster")

    source_srs = _source_spatial_ref(terrain, osr)
    wgs84 = _axis_order(osr.SpatialReference(), osr)
    wgs84.ImportFromEPSG(4326)
    to_geo = osr.CoordinateTransformation(source_srs, wgs84)
    geo_corners = [to_geo.TransformPoint(x, y)[:2] for x, y in _corners(terrain)]
    longitudes = [point[0] for point in geo_corners]
    latitudes = [point[1] for point in geo_corners]
    if not all(math.isfinite(value) for value in longitudes + latitudes):
        raise ValueError("terrain CRS could not be transformed to WGS84")
    west, east = min(longitudes), max(longitudes)
    south, north = min(latitudes), max(latitudes)
    if not (-180 <= west < east <= 180 and -80 <= south < north <= 84):
        raise ValueError("terrain must be a non-polar WGS84 region without dateline wrapping")
    center_lon = (west + east) / 2.0
    center_lat = (south + north) / 2.0
    zone = int(math.floor((center_lon + 180.0) / 6.0)) + 1
    zone = max(1, min(60, zone))
    epsg = (32600 if center_lat >= 0 else 32700) + zone
    target_srs = _axis_order(osr.SpatialReference(), osr)
    if target_srs.ImportFromEPSG(epsg) != 0:
        raise RuntimeError("failed to initialize local UTM projection")

    driver = gdal.GetDriverByName("MEM")
    source = driver.Create("", terrain.cols, terrain.rows, 1, gdal.GDT_Float32)
    source.SetGeoTransform(gt)
    source.SetProjection(source_srs.ExportToWkt())
    source.GetRasterBand(1).WriteArray(values)
    source.GetRasterBand(1).SetUnitType("m")

    preview = gdal.Warp("", source, format="VRT", dstSRS=target_srs.ExportToWkt(),
                        resampleAlg="bilinear", multithread=False)
    if preview is None:
        raise RuntimeError("GDAL could not project the terrain")
    preview_gt = preview.GetGeoTransform()
    resolution = max(abs(preview_gt[1]), abs(preview_gt[5]))
    if not math.isfinite(resolution) or resolution <= 0:
        raise RuntimeError("projected terrain resolution is invalid")
    width_m = preview.RasterXSize * abs(preview_gt[1])
    height_m = preview.RasterYSize * abs(preview_gt[5])
    if max(width_m, height_m) > 250000.0:
        raise ValueError("terrain region exceeds the supported 250 km mission extent")

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix="package-", dir=str(RUNTIME_DIR)))
    try:
        canonical = staging / "terrain.tif"
        reprojected = staging / "terrain-reprojected.tif"
        warped = gdal.Warp(
            str(reprojected), source, format="GTiff",
            dstSRS=target_srs.ExportToWkt(), xRes=resolution, yRes=resolution,
            resampleAlg="bilinear", dstNodata=float("nan"), multithread=False,
            creationOptions=["COMPRESS=DEFLATE", "PREDICTOR=3", "TILED=YES"],
        )
        if warped is None:
            raise RuntimeError("GDAL could not create the canonical GeoTIFF")
        warped.FlushCache()
        warped = None
        dataset = gdal.Open(str(reprojected), gdal.GA_ReadOnly)
        output = dataset.GetRasterBand(1).ReadAsArray().astype(np.float64)
        nodata = dataset.GetRasterBand(1).GetNoDataValue()
        valid = np.isfinite(output)
        if nodata is not None:
            valid &= ~np.isclose(output, nodata)
        if not valid.all():
            top, left, bottom, right = _largest_valid_rectangle(valid)
            retained = (bottom - top) * (right - left)
            if retained < 0.50 * int(valid.sum()) or retained < 16:
                raise ValueError(
                    "terrain cannot be projected to a complete rectangular metre grid")
            cropped = gdal.Translate(
                str(canonical), dataset, format="GTiff",
                srcWin=[left, top, right - left, bottom - top],
                creationOptions=["COMPRESS=DEFLATE", "PREDICTOR=3", "TILED=YES"])
            if cropped is None:
                raise RuntimeError("GDAL could not crop the complete terrain footprint")
            cropped.FlushCache()
            cropped = None
            dataset = None
            reprojected.unlink()
        else:
            dataset = None
            reprojected.replace(canonical)
        dataset = gdal.Open(str(canonical), gdal.GA_ReadOnly)
        output = dataset.GetRasterBand(1).ReadAsArray().astype(np.float64)
        if not np.isfinite(output).all():
            raise RuntimeError("canonical terrain still contains unavailable cells")
        output_gt = dataset.GetGeoTransform()
        if not math.isclose(abs(output_gt[1]), abs(output_gt[5]), rel_tol=1.0e-9):
            raise RuntimeError("canonical terrain does not have square metre pixels")
        projected_bounds = {
            "west": output_gt[0],
            "south": output_gt[3] + output_gt[5] * dataset.RasterYSize,
            "east": output_gt[0] + output_gt[1] * dataset.RasterXSize,
            "north": output_gt[3],
        }
        projected_to_geo = osr.CoordinateTransformation(target_srs, wgs84)
        canonical_geo_corners = [
            projected_to_geo.TransformPoint(x, y)[:2]
            for x in (projected_bounds["west"], projected_bounds["east"])
            for y in (projected_bounds["south"], projected_bounds["north"])
        ]
        canonical_longitudes = [point[0] for point in canonical_geo_corners]
        canonical_latitudes = [point[1] for point in canonical_geo_corners]
        output_rows = dataset.RasterYSize
        output_cols = dataset.RasterXSize
        output_minimum = float(output.min())
        output_maximum = float(output.max())
        dataset = None
        digest = _sha256(canonical)
        terrain_id = "utm{}-{}".format(epsg, digest[:16])
        final_dir = RUNTIME_DIR / "packages" / terrain_id
        if final_dir.exists():
            shutil.rmtree(staging)
        else:
            final_dir.parent.mkdir(parents=True, exist_ok=True)
            staging.replace(final_dir)
        canonical = final_dir / "terrain.tif"
        manifest = {
            "schema": SCHEMA,
            "terrain_id": terrain_id,
            "name": str(name or "terrain"),
            "canonical_dem": str(canonical.resolve()),
            "canonical_dem_sha256": digest,
            "horizontal_crs": {
                "authority": _authority(target_srs) or "EPSG:{}".format(epsg),
                "wkt": target_srs.ExportToWkt(),
                "linear_unit": "metre",
            },
            "vertical_datum": "MSL",
            "world_frame": {
                "frame_id": "world",
                "x": "projected_easting_minus_dem_center_easting_m",
                "y": "projected_northing_minus_dem_center_northing_m",
                "z": "elevation_msl_m",
            },
            "wgs84_bounds": {
                "west": min(canonical_longitudes),
                "south": min(canonical_latitudes),
                "east": max(canonical_longitudes),
                "north": max(canonical_latitudes),
            },
            "projected_bounds_m": projected_bounds,
            "dimensions": {"rows": output_rows,
                           "cols": output_cols,
                           "resolution_m": abs(output_gt[1])},
            "elevation_m": {"minimum": output_minimum,
                            "maximum": output_maximum},
            "source": metadata,
        }
        _write_json_atomic(final_dir / "manifest.json", manifest)
        _write_json_atomic(ACTIVE_MANIFEST, manifest)
        return manifest
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise


def active_manifest() -> Optional[dict]:
    if not ACTIVE_MANIFEST.is_file():
        return None
    payload = json.loads(ACTIVE_MANIFEST.read_text(encoding="utf-8"))
    if payload.get("schema") != SCHEMA:
        raise RuntimeError("active terrain package schema is unsupported")
    dem = Path(payload.get("canonical_dem", ""))
    if not dem.is_file() or _sha256(dem) != payload.get("canonical_dem_sha256"):
        raise RuntimeError("active terrain package is missing or has changed")
    return payload
