#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解算核心的数学单元测试 —— 不依赖 ROS，在本机/CI 直接跑：

    python3 test/test_solver.py -v

覆盖：
  1) ENU<->经纬高 互换的往返一致性；
  2) 纯北向偏移与"子午线弧长数值积分"的独立比对（验证曲率半径模型）；
  3) 下视/前视几何的解析解；
  4) YAW_LOCK=0（叠加罗盘航向）与 YAW_LOCK=1 的等价性；
  5) roll 不影响 +X 光轴、boresight 与图像偏角的符号方向；
  6) 随机目标点的正向建模-反解闭环（端到端几何自检）。
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from target_geolocation import geodesy, solver  # noqa: E402


def _meridian_arc_m(phi0, phi1, n=2000):
    """子午线弧长 ∫M(φ)dφ，Simpson 积分（独立于 enu_to_geodetic 的实现）。"""
    if n % 2:
        n += 1
    h = (phi1 - phi0) / n
    total = 0.0
    for i in range(n + 1):
        phi = phi0 + i * h
        m, _ = geodesy._curvature_radii(phi)
        w = 1 if i in (0, n) else (4 if i % 2 else 2)
        total += w * m
    return total * h / 3.0


def _meridian_lat_for_north(lat_deg, north_m):
    """用数值积分 + 二分求"向北走 north_m 米后的纬度"，作为独立参考值。"""
    phi0 = math.radians(lat_deg)
    lo, hi = phi0 - 0.01, phi0 + 0.01
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if _meridian_arc_m(phi0, mid) < north_m:
            lo = mid
        else:
            hi = mid
    return math.degrees(0.5 * (lo + hi))


class TestGeodesy(unittest.TestCase):

    def test_round_trip_enu(self):
        """ENU 平移 -> 经纬高 -> 再反解，应回到原点 (误差 < 1e-6 m)。"""
        cases = [
            (47.397742, 8.545594, 488.0, 0.0, 0.0, 0.0),
            (47.397742, 8.545594, 488.0, 300.0, -450.0, -120.0),
            (0.0, 0.0, 0.0, 1000.0, 1000.0, 0.0),
            (68.5, 179.9995, 20.0, 800.0, 600.0, 5.0),      # 跨 180° 经线
            (-33.9, 151.2, 30.0, -2500.0, 1800.0, -40.0),
        ]
        for lat, lon, alt, e, n, u in cases:
            tlat, tlon, talt = geodesy.enu_to_geodetic(lat, lon, alt, e, n, u)
            e2, n2 = geodesy.geodetic_to_enu(lat, lon, tlat, tlon)
            self.assertAlmostEqual(e2, e, delta=1e-6, msg='east 往返不一致')
            self.assertAlmostEqual(n2, n, delta=1e-6, msg='north 往返不一致')
            self.assertAlmostEqual(talt, alt + u, delta=1e-9)

    def test_north_against_meridian_integral(self):
        """纯北向偏移：与子午线弧长数值积分对比，误差应远小于 1 mm。"""
        for lat0 in (0.0, 23.5, 45.0, 60.0, 78.0):
            for north in (100.0, 1000.0, 5000.0):
                lat1, _, _ = geodesy.enu_to_geodetic(lat0, 10.0, 0.0, 0.0, north, 0.0)
                ref = _meridian_lat_for_north(lat0, north)
                # 把纬度误差换算成米再断言
                err_m = abs(lat1 - ref) * math.radians(1.0) * 111319.5 / 1.0
                self.assertLess(err_m, 1e-3, 'north=%g lat0=%g 误差 %.4f m'
                                % (north, lat0, err_m))

    def test_east_uses_prime_vertical_radius(self):
        """纯东向偏移：Δλ = east/(N·cosφ)，N 取当前纬度。"""
        lat0, lon0, east = 47.0, 8.0, 1000.0
        lat1, lon1, _ = geodesy.enu_to_geodetic(lat0, lon0, 0.0, east, 0.0, 0.0)
        _, N = geodesy._curvature_radii(math.radians(lat0))
        expect = lon0 + math.degrees(east / (N * math.cos(math.radians(lat0))))
        self.assertAlmostEqual(lon1, expect, delta=1e-7)
        self.assertAlmostEqual(lat1, lat0, delta=1e-9)

    def test_ecef_round_trip(self):
        x, y, z = geodesy.geodetic_to_ecef(47.397742, 8.545594, 488.0)
        lat, lon, alt = geodesy.ecef_to_geodetic(x, y, z)
        self.assertAlmostEqual(lat, 47.397742, delta=1e-9)
        self.assertAlmostEqual(lon, 8.545594, delta=1e-9)
        self.assertAlmostEqual(alt, 488.0, delta=1e-6)

    def test_horizontal_distance_and_bearing(self):
        lat0, lon0 = 47.0, 8.0
        lat1, lon1, _ = geodesy.enu_to_geodetic(lat0, lon0, 0.0, 100.0, 0.0, 0.0)
        self.assertAlmostEqual(geodesy.horizontal_distance(lat0, lon0, lat1, lon1),
                               100.0, delta=1e-6)
        self.assertAlmostEqual(geodesy.bearing_deg(lat0, lon0, lat1, lon1), 90.0, delta=1e-6)


class TestSolver(unittest.TestCase):

    UAV = dict(lat=47.397742, lon=8.545594, alt=488.0)

    def _q(self, roll_deg, pitch_deg, yaw_deg):
        return solver.quat_from_euler_zyx(math.radians(roll_deg),
                                          math.radians(pitch_deg),
                                          math.radians(yaw_deg))

    def test_nadir_downward(self):
        """pitch=-90 垂直下视：目标应在飞机正下方，高度差 = 斜距。"""
        res = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                  self._q(0, -90, 0), 100.0)
        self.assertAlmostEqual(res['ground_range'], 0.0, delta=1e-6)
        self.assertAlmostEqual(res['alt'], self.UAV['alt'] - 100.0, delta=1e-6)
        self.assertAlmostEqual(res['lat'], self.UAV['lat'], delta=1e-9)
        self.assertAlmostEqual(res['elevation_deg'], -90.0, delta=1e-6)

    def test_level_forward_bearing(self):
        """pitch=0、yaw=45、斜距 100：水平距离 100，方位 45°，等高。"""
        res = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                  self._q(0, 0, 45), 100.0)
        self.assertAlmostEqual(res['ground_range'], 100.0, delta=1e-6)
        self.assertAlmostEqual(res['bearing_deg'], 45.0, delta=1e-6)
        self.assertAlmostEqual(res['alt'], self.UAV['alt'], delta=1e-6)
        self.assertFalse(res['looking_up'])

    def test_roll_does_not_move_plus_x_axis(self):
        """绕机体 +X 的 roll 不改变 +X 光轴指向。"""
        r0 = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                 self._q(0, -30, 70), 200.0)
        r1 = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                 self._q(35, -30, 70), 200.0)
        self.assertAlmostEqual(r0['lat'], r1['lat'], delta=1e-9)
        self.assertAlmostEqual(r0['lon'], r1['lon'], delta=1e-9)
        self.assertAlmostEqual(r0['alt'], r1['alt'], delta=1e-9)

    def test_yaw_lock_flag_equivalence(self):
        """YAW_LOCK=0 + 航向 30° 应等价于 YAW_LOCK=1 且世界 yaw 多加 30°。"""
        body_yaw = 20.0
        heading = 30.0
        q_locked = self._q(0, -25, body_yaw + heading)       # 世界系 yaw
        q_body = self._q(0, -25, body_yaw)                   # 机体系 yaw
        a = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                q_locked, 150.0, yaw_locked=True)
        b = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                q_body, 150.0, yaw_locked=False, heading_deg=heading)
        for k in ('lat', 'lon', 'alt'):
            self.assertAlmostEqual(a[k], b[k], delta=1e-9, msg=k)
        self.assertAlmostEqual(b['bearing_deg'], 50.0, delta=1e-6)

    def test_boresight_yaw_offset_shifts_bearing(self):
        """安装偏差 +10° 应让方位角正好 +10°。"""
        res = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                  self._q(0, 0, 0), 100.0,
                                  boresight_rpy_deg=(0.0, 0.0, 10.0))
        self.assertAlmostEqual(res['bearing_deg'], 10.0, delta=1e-6)

    def test_image_offset_direction(self):
        """图像右偏(alpha>0) 使方位角增大；下偏(beta>0) 使俯角增大。

        注意放大效应：像素偏角是绕机体竖轴(向下)的旋转，俯角 45° 时
        5° 的图像右偏对应 atan(tan5°/cos45°)=7.05° 的方位角变化。
        """
        pitch_deg, alpha_deg = -45.0, 5.0
        right = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                    self._q(0, pitch_deg, 0), 100.0,
                                    img_offset_rad=(math.radians(alpha_deg), 0.0))
        expect = math.degrees(math.atan(math.tan(math.radians(alpha_deg)) /
                                        math.cos(math.radians(-pitch_deg))))
        self.assertAlmostEqual(right['bearing_deg'], expect, delta=1e-6)
        self.assertAlmostEqual(expect, 7.0532, delta=1e-3)

        # 正前方(pitch=0)时无放大，方位角变化就等于偏角
        level = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                    self._q(0, 0, 0), 100.0,
                                    img_offset_rad=(math.radians(alpha_deg), 0.0))
        self.assertAlmostEqual(level['bearing_deg'], alpha_deg, delta=1e-6)

        down = solver.solve_target(self.UAV['lat'], self.UAV['lon'], self.UAV['alt'],
                                   self._q(0, pitch_deg, 0), 100.0,
                                   img_offset_rad=(0.0, math.radians(alpha_deg)))
        self.assertLess(down['elevation_deg'], pitch_deg)

    def test_forward_model_closed_loop(self):
        """随机目标点 -> 构造"恰好指向它"的云台姿态与斜距 -> 反解应完全复原。"""
        lat0, lon0, alt0 = 47.397742, 8.545594, 500.0
        for bearing in (0.0, 37.0, 90.0, 181.0, 275.0, 359.0):
            for dist in (30.0, 200.0, 1500.0):
                for drop in (10.0, 120.0):
                    tlat, tlon, talt = geodesy.enu_to_geodetic(
                        lat0, lon0, alt0, 0.0, 0.0, 0.0)
                    e = dist * math.sin(math.radians(bearing))
                    n = dist * math.cos(math.radians(bearing))
                    tlat, tlon, talt = geodesy.enu_to_geodetic(lat0, lon0, alt0,
                                                               e, n, -drop)
                    slant = math.hypot(dist, drop)
                    pitch = math.degrees(math.atan2(-drop, dist))
                    q = self._q(0.0, pitch, bearing)
                    res = solver.solve_target(lat0, lon0, alt0, q, slant)
                    err = geodesy.distance_3d(res['lat'], res['lon'], res['alt'],
                                              tlat, tlon, talt)
                    self.assertLess(err, 1e-6,
                                    'bearing=%.0f dist=%.0f drop=%.0f 误差 %.6f m'
                                    % (bearing, dist, drop, err))


if __name__ == '__main__':
    unittest.main(verbosity=2)
