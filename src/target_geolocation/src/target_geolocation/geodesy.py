#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WGS84 大地坐标 <-> 局部 ENU 平移量 的互换（纯 Python，无第三方依赖）。

为什么不能用 "度 ≈ 米/111320" 的近似：
    子午圈曲率半径 M 随纬度从 6335 km(赤道) 变到 6399 km(极点)，卯酉圈半径 N
    从 6378 km 变到 6399 km。用固定 111320 换算，1 km 距离就会带来 1~3 m 的
    系统误差，对目标打点（往往要求米级）不可接受。

这里用的是 WGS84 椭球上的曲率半径模型（与 GeographicLib 的 Direct 问题在
短距离上等价）：
    Δφ = north / M(φ)              M = a(1-e²)/(1-e²sin²φ)^{3/2}
    Δλ = east  / (N(φ)·cosφ)       N = a/(1-e²sin²φ)^{1/2}
    Δh = up
用中点纬度迭代（fixed-point）求 Δφ，因为 M 本身依赖 φ。1 km 量级误差 < 1 mm，
10 km 量级 < 5 cm —— 远超测距/姿态本身的噪声。
"""

import math

# ---------------------------------------------------------------- WGS84 常数
WGS84_A = 6378137.0                      # 长半轴 [m]
WGS84_F = 1.0 / 298.257223563            # 扁率
WGS84_B = WGS84_A * (1.0 - WGS84_F)      # 短半轴 [m]
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)     # 第一偏心率平方


def _curvature_radii(phi_rad):
    """给定大地纬度(rad)，返回 (子午圈曲率半径 M, 卯酉圈曲率半径 N)，单位 m。"""
    s = math.sin(phi_rad)
    w = math.sqrt(1.0 - WGS84_E2 * s * s)
    M = WGS84_A * (1.0 - WGS84_E2) / (w * w * w)
    N = WGS84_A / w
    return M, N


def enu_to_geodetic(lat_deg, lon_deg, alt_m, east, north, up, max_iter=4):
    """从参考点 (lat,lon,alt) 沿 ENU 平移 (east,north,up) 米，返回目标经纬高。

    :return: (lat_deg, lon_deg, alt_m)
    """
    phi0 = math.radians(lat_deg)
    lam0 = math.radians(lon_deg)

    dphi = 0.0
    dlam = 0.0
    for _ in range(max_iter):
        phi_mid = phi0 + 0.5 * dphi
        _M, N = _curvature_radii(phi_mid)
        cos_mid = max(math.cos(phi_mid), 1e-9)     # 极点附近保护
        new_dphi = north / _M
        new_dlam = east / (N * cos_mid)
        converged = (abs(new_dphi - dphi) < 1e-13 and
                     abs(new_dlam - dlam) < 1e-13)
        dphi, dlam = new_dphi, new_dlam
        if converged:
            break

    lat = math.degrees(phi0 + dphi)
    lon = math.degrees(lam0 + dlam)
    # 经度归一化到 [-180,180)
    if lon >= 180.0:
        lon -= 360.0
    elif lon < -180.0:
        lon += 360.0
    return lat, lon, alt_m + up


def geodetic_to_enu(lat0_deg, lon0_deg, lat1_deg, lon1_deg):
    """参考点 -> 目标点的局部 ENU 水平偏移 (east, north)，单位 m。

    enu_to_geodetic 的逆运算（同一曲率半径模型），用于校验与真值比对。
    """
    phi0 = math.radians(lat0_deg)
    dphi = math.radians(lat1_deg - lat0_deg)
    dlam = math.radians(lon1_deg - lon0_deg)
    # 跨 ±180 经线时取最短路径
    if dlam > math.pi:
        dlam -= 2.0 * math.pi
    elif dlam < -math.pi:
        dlam += 2.0 * math.pi

    phi_mid = phi0 + 0.5 * dphi
    M, N = _curvature_radii(phi_mid)
    north = dphi * M
    east = dlam * N * max(math.cos(phi_mid), 1e-9)
    return east, north


# ------------------------------------------------------------------ ECEF 辅助
def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    """大地坐标 -> 地心地固坐标 ECEF (x,y,z)，单位 m。"""
    phi = math.radians(lat_deg)
    lam = math.radians(lon_deg)
    s = math.sin(phi)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    x = (N + alt_m) * math.cos(phi) * math.cos(lam)
    y = (N + alt_m) * math.cos(phi) * math.sin(lam)
    z = (N * (1.0 - WGS84_E2) + alt_m) * s
    return x, y, z


def ecef_to_geodetic(x, y, z):
    """ECEF -> 大地坐标（Bowring 闭式解，无需迭代）。"""
    p = math.hypot(x, y)
    if p < 1e-9:                     # 极点
        lat = math.copysign(math.pi / 2.0, z)
        return math.degrees(lat), 0.0, abs(z) - WGS84_B

    lam = math.atan2(y, x)
    ep2 = (WGS84_A ** 2 - WGS84_B ** 2) / (WGS84_B ** 2)   # 第二偏心率平方
    theta = math.atan2(z * WGS84_A, p * WGS84_B)
    st, ct = math.sin(theta), math.cos(theta)
    lat = math.atan2(z + ep2 * WGS84_B * st ** 3,
                     p - WGS84_E2 * WGS84_A * ct ** 3)
    s = math.sin(lat)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(lam), alt


def distance_3d(lat1, lon1, alt1, lat2, lon2, alt2):
    """两点三维直线距离 [m]（ECEF 弦长，短距离下与大地线差 < 1e-6 m）。"""
    x1, y1, z1 = geodetic_to_ecef(lat1, lon1, alt1)
    x2, y2, z2 = geodetic_to_ecef(lat2, lon2, alt2)
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


def horizontal_distance(lat1, lon1, lat2, lon2):
    """水平距离 [m]（先取 ENU 水平量再用勾股，避免弦长把高度差混进来）。"""
    e, n = geodetic_to_enu(lat1, lon1, lat2, lon2)
    return math.hypot(e, n)


def bearing_deg(lat1, lon1, lat2, lon2):
    """从点1到点2的方位角（度，0=正北，顺时针为正）。"""
    e, n = geodetic_to_enu(lat1, lon1, lat2, lon2)
    return (math.degrees(math.atan2(e, n)) + 360.0) % 360.0
