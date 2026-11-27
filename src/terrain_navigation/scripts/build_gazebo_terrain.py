#!/usr/bin/env python3
"""Generate a Gazebo Classic heightmap world from Davos georeferenced rasters."""
import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom

try:
    import numpy as np
    from osgeo import gdal, osr
    from PIL import Image
except ImportError as exc:
    sys.exit("Missing dependency {}. Install: sudo apt install python3-gdal python3-numpy python3-pil".format(exc))

ROOT = Path(__file__).resolve().parents[1]
DEFAULTS = {
    "dem": ROOT / "resources/davosdorf.tif",
    "color": ROOT / "resources/davosdorf_color.tif",
    "world_template": ROOT / "resources/davosdorf.world",
    "output_world": ROOT / "resources/davosdorf_terrain.world",
    "output_dir": ROOT / "resources/gazebo_terrain",
}
IMAGE_SIZE = 1025
MAP_MARGIN = 100.0


def resample_grid(data, x, y):
    """Bilinearly sample a raster on one-dimensional source pixel axes."""
    in_h, in_w = data.shape[:2]
    x = np.clip(np.asarray(x, dtype=np.float64), 0, in_w-1)
    y = np.clip(np.asarray(y, dtype=np.float64), 0, in_h-1)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    x1, y1 = np.minimum(x0 + 1, in_w - 1), np.minimum(y0 + 1, in_h - 1)
    dx, dy = (x - x0)[None, :], (y - y0)[:, None]
    if data.ndim == 2:
        a, b = data[y0[:, None], x0[None, :]], data[y0[:, None], x1[None, :]]
        c, d = data[y1[:, None], x0[None, :]], data[y1[:, None], x1[None, :]]
        return a*(1-dx)*(1-dy) + b*dx*(1-dy) + c*(1-dx)*dy + d*dx*dy
    dx, dy = dx[:, :, None], dy[:, :, None]
    a, b = data[y0[:, None], x0[None, :], :], data[y0[:, None], x1[None, :], :]
    c, d = data[y1[:, None], x0[None, :], :], data[y1[:, None], x1[None, :], :]
    return a*(1-dx)*(1-dy) + b*dx*(1-dy) + c*(1-dx)*dy + d*dx*dy

def sample(data, x, y):
    h, w = data.shape[:2]
    x, y = min(max(float(x), 0), w-1), min(max(float(y), 0), h-1)
    x0, y0 = int(math.floor(x)), int(math.floor(y))
    x1, y1 = min(x0+1, w-1), min(y0+1, h-1)
    dx, dy = x-x0, y-y0
    return (data[y0,x0]*(1-dx)*(1-dy) + data[y0,x1]*dx*(1-dy)
            + data[y1,x0]*(1-dx)*dy + data[y1,x1]*dx*dy)


def strip_whitespace(node):
    if node.text is not None and not node.text.strip():
        node.text = None
    if node.tail is not None and not node.tail.strip():
        node.tail = None
    for child in node:
        strip_whitespace(child)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name, default in DEFAULTS.items():
        p.add_argument("--" + name.replace("_", "-"), type=Path, default=default)
    p.add_argument("--image-size", type=int, default=IMAGE_SIZE,
                   help="heightmap side length; must be 2^n+1 (default: 513)")
    p.add_argument("--flip-rows", action="store_true",
                   help="flip north/south if required by your Gazebo image orientation")
    a = p.parse_args()
    if a.image_size < 3 or ((a.image_size-1) & (a.image_size-2)):
        p.error("--image-size must be 2^n+1, such as 257 or 513")
    for name in ("dem", "color", "world_template"):
        if not getattr(a, name).is_file():
            p.error("input file not found: {}".format(getattr(a, name)))

    xml_parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.parse(str(a.world_template), xml_parser).getroot()
    world = root.find("world")
    spherical = world.find("spherical_coordinates") if world is not None else None
    if spherical is None:
        p.error("world template must have <spherical_coordinates>")
    lat = float(spherical.findtext("latitude_deg"))
    lon = float(spherical.findtext("longitude_deg"))
    world_alt = float(spherical.findtext("elevation"))

    dem = gdal.Open(str(a.dem), gdal.GA_ReadOnly)
    if dem is None or dem.RasterCount != 1:
        p.error("DEM must be a readable single-band raster")
    gt = dem.GetGeoTransform()
    if abs(gt[2]) > 1e-9 or abs(gt[4]) > 1e-9:
        p.error("rotated DEM rasters are not supported")
    src = dem.ReadAsArray().astype(np.float64)
    nodata = dem.GetRasterBand(1).GetNoDataValue()
    valid = np.isfinite(src)
    if nodata is not None:
        valid &= src != nodata
    if not valid.all():
        p.error("DEM contains NoData cells; fill them before building terrain")

    source_srs, geo_srs = osr.SpatialReference(), osr.SpatialReference()
    source_srs.ImportFromWkt(dem.GetProjection())
    geo_srs.ImportFromEPSG(4326)
    source_srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    geo_srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    tx = osr.CoordinateTransformation(geo_srs, source_srs)
    origin_e, origin_n, _ = tx.TransformPoint(lon, lat)
    height, width = src.shape
    px = (origin_e - gt[0])/gt[1] - 0.5
    py = (origin_n - gt[3])/gt[5] - 0.5
    if not (0 <= px <= width-1 and 0 <= py <= height-1):
        p.error("world GPS origin is outside DEM (pixel {:.2f}, {:.2f})".format(px, py))

    # Gazebo Classic ODE heightmap collisions are centered at world X/Y zero and
    # cannot be shifted horizontally. Place the requested GPS origin at the
    # center pixel, and include the entire source raster in a square grid.
    raster_x_min = gt[0] - origin_e
    raster_x_max = gt[0] + gt[1]*width - origin_e
    raster_y_min = gt[3] + gt[5]*height - origin_n
    raster_y_max = gt[3] - origin_n
    half_extent = max(abs(raster_x_min), abs(raster_x_max),
                      abs(raster_y_min), abs(raster_y_max))
    terrain_side = 2.0 * math.ceil((half_extent + MAP_MARGIN)/100.0)*100.0
    world_x = np.linspace(-terrain_side/2, terrain_side/2, a.image_size)
    world_y = np.linspace(terrain_side/2, -terrain_side/2, a.image_size)
    src_x = (origin_e + world_x - gt[0])/gt[1] - 0.5
    src_y = (origin_n + world_y - gt[3])/gt[5] - 0.5
    heights = resample_grid(src, src_x, src_y)
    if a.flip_rows:
        heights = heights[::-1, :]
    zmin, zmax = float(src.min()), float(src.max())
    if zmax <= zmin:
        p.error("DEM has no elevation variation")
    gray = np.rint(np.clip((heights-zmin)/(zmax-zmin), 0, 1)*255).astype(np.uint8)

    origin_h = float(sample(src, px, py))
    origin_h_resampled = float(heights[a.image_size//2, a.image_size//2])
    # 8-bit quantization is offset so the center launch point lies at local z ~= 0.
    origin_gray = round((origin_h_resampled-zmin)/(zmax-zmin)*255)
    z_offset = -(origin_gray/255.0)*(zmax-zmin)
    size, pos = (terrain_side, terrain_side, zmax-zmin), (0.0, 0.0, z_offset)

    a.output_dir.mkdir(parents=True, exist_ok=True)
    height_path = a.output_dir / "davosdorf_heightmap.png"
    Image.fromarray(gray, mode="L").save(str(height_path))

    color_ds = gdal.Open(str(a.color), gdal.GA_ReadOnly)
    if color_ds is None or color_ds.RasterCount < 3:
        p.error("color raster must contain RGB bands")
    color_gt = color_ds.GetGeoTransform()
    if (color_ds.RasterXSize != width or color_ds.RasterYSize != height or
            any(abs(x-y) > 1e-9 for x, y in zip(color_gt, gt)) or
            color_ds.GetProjection() != dem.GetProjection()):
        p.error("color raster must share the DEM grid and projection")
    rgb_src = np.stack([color_ds.GetRasterBand(i).ReadAsArray() for i in (1,2,3)], axis=-1)
    rgb = np.clip(resample_grid(rgb_src.astype(np.float64), src_x, src_y),
                  0, 255).astype(np.uint8)
    if a.flip_rows:
        rgb = rgb[::-1, :, :]
    color_path = a.output_dir / "davosdorf_orthophoto.png"
    Image.fromarray(rgb, mode="RGB").save(str(color_path))

    huri, curi = height_path.resolve().as_uri(), color_path.resolve().as_uri()
    # Drop the flat asphalt floor; preserve the existing GPS/world settings.
    for include in list(world.findall("include")):
        if (include.findtext("uri") or "").strip() in ("model://asphalt_plane", "model://ground_plane"):
            world.remove(include)

    model = ET.SubElement(world, "model", {"name": "davosdorf_terrain"})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "terrain_link"})
    ET.SubElement(link, "pose").text = "0 0 0 0 0 0"
    for tag, name in (("collision","terrain_collision"), ("visual","terrain_visual")):
        obj = ET.SubElement(link, tag, {"name": name})
        geom = ET.SubElement(obj, "geometry")
        hm = ET.SubElement(geom, "heightmap")
        if tag == "visual":
            texture_node = ET.SubElement(hm, "texture")
            ET.SubElement(texture_node, "size").text = str(terrain_side)
            ET.SubElement(texture_node, "diffuse").text = curi
            ET.SubElement(texture_node, "normal").text = "file://media/materials/textures/flat_normal.png"
        ET.SubElement(hm, "uri").text = huri
        ET.SubElement(hm, "size").text = "{:.6f} {:.6f} {:.6f}".format(*size)
        ET.SubElement(hm, "pos").text = "{:.6f} {:.6f} {:.6f}".format(*pos)

    strip_whitespace(root)
    pretty = minidom.parseString(ET.tostring(root, encoding="utf-8")).toprettyxml(
        indent="  ", encoding="utf-8")
    a.output_world.parent.mkdir(parents=True, exist_ok=True)
    a.output_world.write_bytes(pretty)
    print("Generated world: {}".format(a.output_world))
    print("Heightmap: {} ({}x{}, 8-bit, 2^n+1)".format(height_path, a.image_size, a.image_size))
    print("Orthophoto: {}".format(color_path))
    print("Heightmap footprint: {:.1f}x{:.1f} m; relief {:.1f} m".format(terrain_side, terrain_side, size[2]))
    print("Heightmap center is the GPS origin at local ENU (0, 0, 0).")
    print("DEM at GPS origin: {:.2f} m; world datum: {:.7f}, {:.7f}, {:.2f} m".format(
        origin_h, lat, lon, world_alt))
    print("Height offset aligns the terrain beneath the launch point to local z ~= 0.")


if __name__ == "__main__":
    main()
