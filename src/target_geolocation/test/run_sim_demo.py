#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_sim_demo —— 不装 ROS 也能"真跑"节点，打印逐帧测量结果与误差统计。

    python3 test/run_sim_demo.py

它复用 test_node_dryrun.py 里的 rospy 桩，把 scripts/ 下的
sim_target_sensor.py 与 target_geolocation_node.py **原样**实例化，
按真实时序逐帧喂数据（20 Hz 虚拟时钟），输出：

  1. 三个典型几何下的逐帧结果（真值 / 解算 / 误差）
  2. 每个几何的误差统计（水平 mean/p95/max，垂直）
  3. 移动目标跟踪场景
  4. 关掉图像偏角、开/关平滑的对比

注意：这是"传感器噪声模型下的精度上限"，不是仿真器画面/物理的精度。
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import test_node_dryrun as T  # noqa: E402  复用 rospy 桩与脚本加载器
from target_geolocation import geodesy  # noqa: E402

T._install_ros_stubs()

UAV = (47.397742, 8.545594, 488.0)


def make_gps(lat, lon, alt, stamp):
    g = T.NavSatFix()
    g.header.stamp = stamp
    g.latitude, g.longitude, g.altitude = lat, lon, alt
    g.status.status = 0
    return g


def scenario(name, distance_m, height_agl, att_noise_deg=0.10,
             range_noise_m=0.05, samples=100, motion_radius_m=0.0,
             smooth_alpha=0.4, show_rows=4, uav_speed_mps=0.0):
    """跑一个场景，返回统计 dict，并打印逐帧表格。"""
    T.Time._clock = 1000.0
    T.ROSPY_PARAMS.clear()
    T.ROSPY_PARAMS.update({
        # --- 合成传感器 ---
        '~truth_bearing_deg': 60.0,
        '~truth_distance_m': distance_m,
        '~truth_height_agl': height_agl,
        '~att_noise_deg': att_noise_deg,
        '~range_noise_std': range_noise_m,
        '~motion_radius_m': motion_radius_m,
        '~motion_period_s': 20.0,
        '~range_topic': '/sim/range',
        '~gimbal_att_topic': '/sim/gimbal/attitude',
        # --- 解算节点 ---
        '~use_img_offset': False,
        '~smooth_alpha': smooth_alpha,
    })

    sim = T._load_script('sim_target_sensor.py')
    node_mod = T._load_node_module()
    sensor = sim.SimTargetSensor()
    node = node_mod.TargetGeolocationNode()

    print('\n' + '=' * 92)
    print('场景: %s | 平距 %.0f m 高差 %.0f m | 姿态噪声 %.2f° 测距噪声 %.1f cm'
          % (name, distance_m, height_agl, att_noise_deg, range_noise_m * 100))
    print('=' * 92)
    print('  %-4s %-24s %-24s %-10s %-9s %-9s' %
          ('#', '真值 lat,lon,alt', '解算 lat,lon,alt', '水平误差', '垂直误差', '斜距'))

    horiz, vert = [], []
    for i in range(samples):
        # 飞机位置（可选匀速前飞，用于观察时间同步/滞后）
        t = (T.Time._clock - 1000.0)
        uav_lat, uav_lon, uav_alt = geodesy.enu_to_geodetic(
            UAV[0], UAV[1], UAV[2], 0.0, uav_speed_mps * t, 0.0)
        gps = make_gps(uav_lat, uav_lon, uav_alt, T.Time(T.Time._clock))

        sensor._gps_cb(gps)
        sensor._tick(None)
        if not sensor.pub_truth.published:
            continue
        truth = sensor.pub_truth.published[-1]
        rng = sensor.pub_range.published[-1]
        att = sensor.pub_att.published[-1]

        node._gps_cb(gps)
        node._range_cb(rng)
        node._gimbal_att_cb(att)
        node._tick(None)
        if not node.pub_fix.published:
            continue
        est = node.pub_fix.published[-1]

        h = geodesy.horizontal_distance(est.latitude, est.longitude,
                                        truth.latitude, truth.longitude)
        v = abs(est.altitude - truth.altitude)
        # 前 5 帧是缓存填充/平滑收敛期，不计入统计
        if i >= 5:
            horiz.append(h)
            vert.append(v)

        if i < show_rows:
            print('  %-4d %-24s %-24s %-10s %-9s %-9s' % (
                i,
                '%.7f,%.7f,%.1f' % (truth.latitude, truth.longitude, truth.altitude),
                '%.7f,%.7f,%.1f' % (est.latitude, est.longitude, est.altitude),
                '%.2f m' % h, '%.2f m' % v, '%.1f m' % rng.range))

    if not horiz:
        print('  !! 该场景没有有效解')
        return {}

    sol = node.pub_solution.published[-1].data
    print('  ...')
    print('  最后解算的局部几何: 平距 %.1f m 方位 %.1f° 仰角 %.1f°'
          % (sol[7], sol[8], sol[9]))
    print('  统计 (n=%d): 水平 mean=%.2f  p95=%.2f  max=%.2f m | 垂直 mean=%.2f  max=%.2f m'
          % (len(horiz), _mean(horiz), _p95(horiz), max(horiz),
             _mean(vert), max(vert)))
    return dict(n=len(horiz), h_mean=_mean(horiz), h_p95=_p95(horiz),
                h_max=max(horiz), v_mean=_mean(vert), v_max=max(vert))


def _mean(xs):
    return sum(xs) / float(len(xs))


def _p95(xs):
    s = sorted(xs)
    return s[min(len(s) - 1, int(0.95 * len(s)))]


def main():
    print('#' * 92)
    print('# target_geolocation 干跑演示：sim_target_sensor -> target_geolocation_node')
    print('# 飞机悬停 @ %.6f, %.6f, %.0f m，目标锚定在方位 60°，云台指向目标' % UAV)
    print('#' * 92)

    r1 = scenario('近距静态目标', 100.0, 60.0)
    r2 = scenario('中距静态目标(典型)', 300.0, 120.0)
    r3 = scenario('远距静态目标', 800.0, 150.0)

    print('\n' + '#' * 92)
    print('# 姿态噪声的影响（平距 300 m / 高差 120 m）')
    print('#' * 92)
    print('  %-14s %-10s %-10s %-10s' % ('姿态噪声', '水平mean', 'p95', '垂直mean'))
    for sigma in (0.05, 0.10, 0.30, 0.50, 1.00):
        r = scenario('σ=%.2f°' % sigma, 300.0, 120.0, att_noise_deg=sigma,
                     samples=80, show_rows=0)
        if r:
            print('  %-12.2f°  %-8.2f m %-8.2f m %-8.2f m'
                  % (sigma, r['h_mean'], r['h_p95'], r['v_mean']))

    print('\n' + '#' * 92)
    print('# 移动目标跟踪（目标绕半径 30 m 圆运动，周期 20 s，中距）')
    print('#' * 92)
    scenario('移动目标', 300.0, 120.0, motion_radius_m=30.0, samples=120, show_rows=3)
    # 同一场景关掉平滑：噪声变大但滞后消失，说明跟踪移动目标时不该开平滑
    scenario('移动目标, 无平滑', 300.0, 120.0, motion_radius_m=30.0,
             smooth_alpha=0.0, samples=120, show_rows=0)

    print('\n' + '#' * 92)
    print('# 飞机以 8 m/s 前飞 + 关闭平滑（观察动态滞后与抖动）')
    print('#' * 92)
    scenario('前飞 8m/s, 无平滑', 300.0, 120.0, smooth_alpha=0.0,
             samples=100, show_rows=2, uav_speed_mps=8.0)

    print('\n完成。上面"水平误差"就是本节点在当前噪声模型下能给出的测量精度。')


if __name__ == '__main__':
    main()
