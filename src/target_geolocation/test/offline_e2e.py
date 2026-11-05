#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""offline_e2e —— 不装 ROS 也能跑的端到端精度评估（同样的正向模型 + 反向解算）。

    python3 test/offline_e2e.py

它复刻 target_geolocation_node 的数据通路：
    真值目标 -> 正向模型构造云台姿态+斜距(加噪声) -> solver.solve_target -> 与真值比
输出误差统计和"姿态误差/作用距离 -> 定位误差"敏感度表，用来判断实机上
哪些误差源值得花钱（云台反馈精度 vs 测距精度 vs 时间同步）。
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from target_geolocation import geodesy, solver  # noqa: E402

UAV_LAT, UAV_LON, UAV_ALT = 47.397742, 8.545594, 488.0


def forward_model(uav, target, att_noise_deg, range_noise_m, rng=random):
    """由几何真值构造 (云台四元数, 斜距)，模拟仿真/实机传感器输出。"""
    uav_lat, uav_lon, uav_alt = uav
    t_lat, t_lon, t_alt = target
    east, north = geodesy.geodetic_to_enu(uav_lat, uav_lon, t_lat, t_lon)
    up = t_alt - uav_alt
    slant = math.sqrt(east * east + north * north + up * up)

    yaw = math.degrees(math.atan2(east, north)) + rng.gauss(0.0, att_noise_deg)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, up / slant)))) \
        + rng.gauss(0.0, att_noise_deg)
    q = solver.quat_from_euler_zyx(0.0, math.radians(pitch), math.radians(yaw))
    return q, slant + rng.gauss(0.0, range_noise_m)


def run_case(distance_m, agl_m, att_noise_deg, range_noise_m, n=500, bearing=60.0):
    """固定几何下重复采样 n 次，返回 (水平误差列表, 垂直误差列表)。"""
    t_lat, t_lon, t_alt = geodesy.enu_to_geodetic(
        UAV_LAT, UAV_LON, UAV_ALT,
        distance_m * math.sin(math.radians(bearing)),
        distance_m * math.cos(math.radians(bearing)),
        -agl_m)
    target = (t_lat, t_lon, t_alt)

    horiz, vert = [], []
    for _ in range(n):
        q, slant = forward_model((UAV_LAT, UAV_LON, UAV_ALT), target,
                                 att_noise_deg, range_noise_m)
        res = solver.solve_target(UAV_LAT, UAV_LON, UAV_ALT, q, slant)
        horiz.append(geodesy.horizontal_distance(res['lat'], res['lon'], t_lat, t_lon))
        vert.append(abs(res['alt'] - t_alt))
    return horiz, vert


def _stat(xs):
    xs = sorted(xs)
    return (sum(xs) / len(xs), xs[min(len(xs) - 1, int(0.95 * len(xs)))], xs[-1])


def main():
    random.seed(20261028)
    print('=' * 78)
    print('离线端到端精度评估（飞机 %.6f, %.6f, %.0f m）' % (UAV_LAT, UAV_LON, UAV_ALT))
    print('=' * 78)

    print('\n[A] 典型工况：平距 300 m / 高差 120 m / 姿态噪声 0.1° / 测距噪声 5 cm')
    h, v = run_case(300.0, 120.0, 0.1, 0.05)
    mh, p95h, maxh = _stat(h)
    mv, p95v, maxv = _stat(v)
    print('    水平误差 mean=%.2f m  p95=%.2f m  max=%.2f m' % (mh, p95h, maxh))
    print('    垂直误差 mean=%.2f m  p95=%.2f m  max=%.2f m' % (mv, p95v, maxv))

    print('\n[B] 姿态误差敏感度（测距噪声固定 5 cm，平距 300 m / 高差 120 m）')
    print('    yaw 误差 -> 水平位移 L·cosθ·δ；pitch 误差 -> 主要是高度位移 L·δ，')
    print('    只有 L·sinθ·δ 落到水平面上，所以下视越陡垂直误差越敏感。')
    print('    %-14s %-14s %-16s' % ('姿态噪声σ', '水平 mean', '理论 L·cosθ·σ'))
    for sigma in (0.05, 0.1, 0.3, 0.5, 1.0, 2.0):
        h, _ = run_case(300.0, 120.0, sigma, 0.05, n=300)
        m, _, _ = _stat(h)
        L = math.hypot(300.0, 120.0)
        cos_t = 300.0 / L
        print('    %-12.2f°  %-12.2f m  %-14.2f m'
              % (sigma, m, L * cos_t * math.radians(sigma)))

    print('\n[C] 作用距离敏感度（姿态噪声 0.5°，高差 100 m）')
    print('    %-14s %-12s %-12s' % ('平距', '水平 mean', '垂直 mean'))
    for dist in (50.0, 100.0, 200.0, 400.0, 800.0, 1500.0):
        h, v = run_case(dist, 100.0, 0.5, 0.05, n=300)
        mh, _, _ = _stat(h)
        mv, _, _ = _stat(v)
        print('    %-12.0f m  %-10.2f m  %-10.2f m' % (dist, mh, mv))

    print('\n[D] 时间不同步的影响（飞机 10 m/s 前飞，姿态 0.5°，平距 300 m）')
    base_e, base_n = 300.0 * math.sin(math.radians(60)), 300.0 * math.cos(math.radians(60))
    for skew_ms in (0, 10, 50, 100, 200):
        errs = []
        for _ in range(200):
            # GPS 与姿态相差 skew_ms：解算用的是 skew 之前的飞机位置
            d = 10.0 * skew_ms / 1000.0
            stale_lat, stale_lon, stale_alt = geodesy.enu_to_geodetic(
                UAV_LAT, UAV_LON, UAV_ALT, -d * math.sin(math.radians(60)),
                -d * math.cos(math.radians(60)), 0.0)
            t_lat, t_lon, t_alt = geodesy.enu_to_geodetic(
                UAV_LAT, UAV_LON, UAV_ALT, base_e, base_n, -120.0)
            q, slant = forward_model((UAV_LAT, UAV_LON, UAV_ALT), (t_lat, t_lon, t_alt),
                                     0.5, 0.05)
            res = solver.solve_target(stale_lat, stale_lon, stale_alt, q, slant)
            errs.append(geodesy.horizontal_distance(res['lat'], res['lon'], t_lat, t_lon))
        m, _, _ = _stat(errs)
        print('    %-8d ms -> 水平 mean=%.2f m' % (skew_ms, m))

    print('\n完成。以上数值即"同样的传感器噪声下，本节点能达到的定位精度"。')


if __name__ == '__main__':
    main()
