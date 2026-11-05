"""WGS84 coordinates to the mission's shared world frame."""

from math import cos, isfinite, radians, sin, sqrt
from typing import Sequence, Tuple


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_world(
    latitude: float,
    longitude: float,
    altitude: float,
    datum: Sequence[float],
    enu_to_world_yaw: float = 0.0,
    world_position: Sequence[float] = (0.0, 0.0, 0.0),
) -> Tuple[float, float, float]:
    """Convert WGS84 latitude/longitude/ellipsoid-height to ``world``.

    The conversion mirrors ``xd_uav_world_tf_manager``: both points are
    converted to ECEF, differenced, projected into the datum's ENU frame, then
    rotated and translated into the configured world frame.
    """
    lat, lon, alt = float(latitude), float(longitude), float(altitude)
    datum_lat, datum_lon, datum_alt = (float(value) for value in datum)
    yaw = float(enu_to_world_yaw)
    origin = tuple(float(value) for value in world_position)
    if len(origin) != 3:
        raise ValueError("world_position must have three values")
    if not all(isfinite(value) for value in (lat, lon, alt, datum_lat, datum_lon, datum_alt, yaw) + origin):
        raise ValueError("geodetic coordinate or world georeference is not finite")
    if not (-90.0 <= lat <= 90.0 and -90.0 <= datum_lat <= 90.0):
        raise ValueError("latitude is outside [-90, 90]")
    if not (-180.0 <= lon <= 180.0 and -180.0 <= datum_lon <= 180.0):
        raise ValueError("longitude is outside [-180, 180]")

    point_ecef = _geodetic_to_ecef(lat, lon, alt)
    datum_ecef = _geodetic_to_ecef(datum_lat, datum_lon, datum_alt)
    dx, dy, dz = (point_ecef[i] - datum_ecef[i] for i in range(3))

    phi = radians(datum_lat)
    lam = radians(datum_lon)
    sp, cp = sin(phi), cos(phi)
    sl, cl = sin(lam), cos(lam)
    east = -sl * dx + cl * dy
    north = -sp * cl * dx - sp * sl * dy + cp * dz
    up = cp * cl * dx + cp * sl * dy + sp * dz

    cy, sy = cos(yaw), sin(yaw)
    return (
        origin[0] + cy * east - sy * north,
        origin[1] + sy * east + cy * north,
        origin[2] + up,
    )


def rotate_enu_covariance_to_world(
    covariance: Sequence[float], enu_to_world_yaw: float
) -> Tuple[float, ...]:
    """Rotate a row-major ENU 3x3 covariance into the world axes."""
    values = tuple(float(value) for value in covariance[:9])
    if len(values) != 9 or not all(isfinite(value) for value in values):
        return (0.0,) * 9
    yaw = float(enu_to_world_yaw)
    cy, sy = cos(yaw), sin(yaw)
    rotation = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    matrix = tuple(tuple(values[3 * row + col] for col in range(3)) for row in range(3))
    return tuple(
        sum(
            rotation[row][i] * matrix[i][j] * rotation[col][j]
            for i in range(3)
            for j in range(3)
        )
        for row in range(3)
        for col in range(3)
    )


def _geodetic_to_ecef(latitude: float, longitude: float, altitude: float):
    phi, lam = radians(latitude), radians(longitude)
    sin_phi, cos_phi = sin(phi), cos(phi)
    radius = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
    return (
        (radius + altitude) * cos_phi * cos(lam),
        (radius + altitude) * cos_phi * sin(lam),
        (radius * (1.0 - WGS84_E2) + altitude) * sin_phi,
    )
