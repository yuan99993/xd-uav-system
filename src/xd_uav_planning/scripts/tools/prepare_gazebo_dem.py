#!/usr/bin/env python3
"""Prepare a georeferenced DEM for Gazebo Classic's heightmap contract.

The source GeoTIFF remains the terrain authority.  Gazebo Classic requires a
square 2^n+1 raster, so this tool creates a nearest-neighbour GeoTIFF cache and
an auditable manifest without changing CRS, horizontal extent, height values,
or vertical datum.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

import numpy as np
from osgeo import gdal, osr


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _configure_proj():
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
    gdal.SetConfigOption("PROJ_LIB", valid[0])


def _authority(spatial_ref):
    spatial_ref.AutoIdentifyEPSG()
    name = spatial_ref.GetAuthorityName(None)
    code = spatial_ref.GetAuthorityCode(None)
    return "{}:{}".format(name, code) if name and code else None


def _finite_elevation(dataset, label):
    band = dataset.GetRasterBand(1)
    values = band.ReadAsArray().astype(np.float64)
    nodata = band.GetNoDataValue()
    valid = np.isfinite(values)
    if nodata is not None:
        valid &= ~np.isclose(values, nodata)
    if not valid.all():
        raise RuntimeError(
            "{} contains NoData or non-finite cells; fill them explicitly "
            "before generating a Gazebo collision surface".format(label))
    return values


def prepare(source, output, manifest, samples, vertical_datum):
    if samples < 3 or ((samples - 1) & (samples - 2)) != 0:
        raise ValueError("samples must be 2^n+1")
    if vertical_datum not in {"MSL"}:
        raise ValueError("vertical datum must currently be MSL")

    source = source.resolve()
    output = output.resolve()
    manifest = manifest.resolve()
    _configure_proj()
    gdal.UseExceptions()
    dataset = gdal.Open(str(source), gdal.GA_ReadOnly)
    if dataset is None or dataset.RasterCount != 1:
        raise RuntimeError("source must be a readable single-band GeoTIFF")
    transform = dataset.GetGeoTransform(can_return_null=True)
    if transform is None or abs(transform[2]) > 1.0e-12 or abs(transform[4]) > 1.0e-12:
        raise RuntimeError("rotated or ungeoreferenced DEMs are unsupported")
    spatial_ref = dataset.GetSpatialRef()
    if spatial_ref is None or not spatial_ref.IsProjected():
        raise RuntimeError("DEM must declare a projected CRS")
    if not math.isclose(spatial_ref.GetLinearUnits(), 1.0, abs_tol=1.0e-12):
        raise RuntimeError("DEM projected coordinates must use metres")
    source_values = _finite_elevation(dataset, "source DEM")

    output.parent.mkdir(parents=True, exist_ok=True)
    manifest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
            prefix=output.name + ".", suffix=".tmp.tif",
            dir=str(output.parent), delete=False) as stream:
        temporary = Path(stream.name)
    try:
        translated = gdal.Translate(
            str(temporary), dataset, format="GTiff", width=samples,
            height=samples, resampleAlg="nearest",
            creationOptions=["COMPRESS=DEFLATE", "PREDICTOR=3"])
        if translated is None:
            raise RuntimeError("GDAL failed to generate the Gazebo DEM")
        translated.FlushCache()
        translated = None
        prepared = gdal.Open(str(temporary), gdal.GA_ReadOnly)
        prepared_values = _finite_elevation(prepared, "prepared DEM")
        prepared_transform = prepared.GetGeoTransform()
        source_bounds = [
            transform[0],
            transform[3] + transform[5] * dataset.RasterYSize,
            transform[0] + transform[1] * dataset.RasterXSize,
            transform[3],
        ]
        prepared_bounds = [
            prepared_transform[0],
            prepared_transform[3] + prepared_transform[5] * prepared.RasterYSize,
            prepared_transform[0] + prepared_transform[1] * prepared.RasterXSize,
            prepared_transform[3],
        ]
        if not np.allclose(source_bounds, prepared_bounds, atol=1.0e-6):
            raise RuntimeError("prepared DEM changed the projected extent")
        temporary.replace(output)
    finally:
        if temporary.exists():
            temporary.unlink()

    payload = {
        "schema": "xd-uav-gazebo-dem/v1",
        "source": {
            "name": source.name,
            "sha256": _sha256(source),
            "width": dataset.RasterXSize,
            "height": dataset.RasterYSize,
        },
        "prepared": {
            "name": output.name,
            "sha256": _sha256(output),
            "width": samples,
            "height": samples,
        },
        "coordinate_reference_system": _authority(spatial_ref),
        "vertical_datum": vertical_datum,
        "world_frame": {
            "x": "projected_easting_minus_dem_center_easting_m",
            "y": "projected_northing_minus_dem_center_northing_m",
            "z": "elevation_msl_m",
        },
        "projected_bounds_m": {
            "west": source_bounds[0], "south": source_bounds[1],
            "east": source_bounds[2], "north": source_bounds[3],
        },
        "elevation_m": {
            "source_min": float(source_values.min()),
            "source_max": float(source_values.max()),
            "prepared_min": float(prepared_values.min()),
            "prepared_max": float(prepared_values.max()),
        },
        "gazebo_heightmap": {
            "size_x_m": float(source_bounds[2] - source_bounds[0]),
            "size_y_m": float(source_bounds[3] - source_bounds[1]),
            "size_z_m": float(prepared_values.max() - prepared_values.min()),
            "position_z_m": 0.0,
            "dem_loader_preserves_minimum_msl_m": float(prepared_values.min()),
        },
        "resampling": "nearest",
    }
    with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", prefix=manifest.name + ".",
            suffix=".tmp", dir=str(manifest.parent), delete=False) as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        temporary_manifest = Path(stream.name)
    temporary_manifest.replace(manifest)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--samples", type=int, default=513)
    parser.add_argument("--vertical-datum", default="MSL")
    args = parser.parse_args()
    prepare(args.source, args.output, args.manifest, args.samples,
            args.vertical_datum)


if __name__ == "__main__":
    main()
