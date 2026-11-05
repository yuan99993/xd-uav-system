#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""target_geolocation: 飞机 GPS + 云台姿态 + 测距 -> 目标 GPS 坐标。"""

from .geodesy import (  # noqa: F401
    WGS84_A, WGS84_B, WGS84_E2, WGS84_F,
    bearing_deg, distance_3d, ecef_to_geodetic, enu_to_geodetic,
    geodetic_to_ecef, geodetic_to_enu, horizontal_distance,
)
from .solver import (  # noqa: F401
    GIMBAL_DEVICE_FLAGS_YAW_LOCK, los_unit_ned, mat_mul, mat_vec,
    ned_to_enu, quat_from_euler_zyx, quat_to_rotmat, rotmat_to_rpy_deg,
    rpy_to_rotmat, solve_target, solution_from_records,
)

__version__ = '1.0.0'
