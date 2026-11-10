#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""locate_live —— 用正在运行的 Project AirSim 真实数据解算目标位置。

    python3 live/locate_live.py --uri http://192.168.2.99:11311/ --robot Drone1

它从 ROS master 上订阅三路数据（纯标准库实现的 TCPROS 客户端，不需要装 ROS）：
    <base>/sensors/GPS/gps                       NavSatFix   飞机 GPS
    <base>/sensors/FrontDistance/distance_sensor Range       机体前置测距仪斜距
    <base>/sensors/IMU/imu_kinematics            Imu         机体姿态（= 光轴姿态）

然后按 P_target = P_uav + L·d 解算，并打印局部 ENU 偏移与经纬度。

关于 IMU 四元数的坐标系：本场景没有云台，光轴就是机体前向轴，指向完全由 IMU
姿态决定。而 Project AirSim 原生是 NED/FRD，ROS 桥又可能转成 REP-103 的
ENU/FLU —— 两者算出来的方位是"关于正北镜像"的，相差几十度。脚本把两种解释
都算出来（ENU 为 ROS 标准优先），并给出判定方法。
"""

import argparse
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import ros1_client as R                      # noqa: E402
from target_geolocation import geodesy, solver  # noqa: E402


def _quat_norm(q_xyzw):
    x, y, z, w = q_xyzw
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (w / n, x / n, y / n, z / n)


def _yaw_pitch_roll_deg(q):
    w, x, y, z = q
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def solve_both(gps, q, rng):
    """按两种坐标系解释各算一次，返回 [(label, bearing, east, north, up, lat, lon, alt)]

    结论依据（Project AirSim ROS Bridge 文档）：
        "Project AirSim and the ROS Bridge both use the NED coordinate system.
         Because both environments share this convention, no conversion is
         required for position, orientation, or velocity data. X: Forward(North)
         Y: Right(East) Z: Down"
    所以 IMU 四元数是 FRD->NED，即 MAVLink/PX4 习惯，也是本包 solver 的默认约定。
    现场佐证：三台机 actual_pose 只在 y 上排开 45 m，而 GPS 只在**经度**上相差
    45 m -> map 的 y 轴就是"东"，即 NED。
    """
    r = solver.quat_to_rotmat(q)                 # 机体 -> 世界（按四元数直接解释）
    vx, vy, vz = (r[0], r[3], r[6])              # R·(1,0,0)：机体前向轴在世界系的分量
    out = []

    # 解释 B（Project AirSim 实际约定）：机体 FRD -> 世界 NED，再换到 ENU
    d_e, d_n, d_u = rng * vy, rng * vx, -rng * vz
    lat, lon, alt = geodesy.enu_to_geodetic(gps['lat'], gps['lon'], gps['alt'], d_e, d_n, d_u)
    out.append(('NED/FRD  (Project AirSim / MAVLink)  <- 主答案',
                (math.degrees(math.atan2(d_e, d_n)) + 360) % 360,
                d_e, d_n, d_u, lat, lon, alt))

    # 解释 A（若桥做过 REP-103 转换才会成立）：机体 FLU -> 世界 ENU
    de, dn, du = rng * vx, rng * vy, rng * vz
    lat, lon, alt = geodesy.enu_to_geodetic(gps['lat'], gps['lon'], gps['alt'], de, dn, du)
    out.append(('ENU/FLU  (ROS REP-103，仅当桥做过转换)  参考',
                (math.degrees(math.atan2(de, dn)) + 360) % 360,
                de, dn, du, lat, lon, alt))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uri', default='http://192.168.2.99:11311/')
    ap.add_argument('--robot', default='Drone1')
    ap.add_argument('--scene', default='SceneBasicDrone')
    ap.add_argument('--samples', type=int, default=30)
    ap.add_argument('--timeout', type=float, default=10.0)
    args = ap.parse_args()

    base = '/Sim/%s/robots/%s' % (args.scene, args.robot)
    topics = {
        'gps': base + '/sensors/GPS/gps',
        'range': base + '/sensors/FrontDistance/distance_sensor',
        'imu': base + '/sensors/IMU/imu_kinematics',
    }
    host = args.uri.split('//')[1].split(':')[0]

    print('连接 %s  载具 %s' % (args.uri, args.robot))
    data, errs = {}, {}

    def grab(k):
        try:
            m = R.Master(args.uri)               # 每个线程独立代理，ServerProxy 非线程安全
            data[k] = m.subscribe(topics[k], timeout=args.timeout,
                                  max_msgs=args.samples, host_override=host)
        except Exception as exc:
            errs[k] = '%s: %s' % (type(exc).__name__, exc)

    threads = [threading.Thread(target=grab, args=(k,)) for k in topics]
    [t.start() for t in threads]
    [t.join() for t in threads]

    for k, v in errs.items():
        print('  [%s] 读取失败: %s' % (k, v))
    if not all(k in data for k in topics):
        return 1

    # ---- 取最新一帧（按时间戳对齐）----
    gps = max(data['gps'], key=lambda m: m['header']['stamp'])
    imu = max(data['imu'], key=lambda m: m['header']['stamp'])
    ranges = data['range']
    finite = [m for m in ranges if math.isfinite(m['range']) and m['range'] > 0]
    if not finite:
        print('\n!! 测距仪 %d 帧全部为 inf（前方 %.0f m 内无回波），无法定位目标'
              % (len(ranges), ranges[-1]['max_range']))
        return 2
    rng_msg = max(finite, key=lambda m: m['header']['stamp'])

    q = _quat_norm(imu['q_xyzw'])
    yaw, pitch, roll = _yaw_pitch_roll_deg(q)
    dt_rng = abs(rng_msg['header']['stamp'] - gps['header']['stamp'])
    dt_imu = abs(imu['header']['stamp'] - gps['header']['stamp'])

    print('\n--- 原始测量 ---')
    print('  飞机 GPS      : %.7f, %.7f, %.3f m  (frame=%s)'
          % (gps['lat'], gps['lon'], gps['alt'], gps['header']['frame_id']))
    print('  测距仪斜距    : %.3f m   (%d/%d 帧有效, 范围 %.2f~%.3f m, fov %.2f°)'
          % (rng_msg['range'], len(finite), len(ranges),
             min(m['range'] for m in finite), max(m['range'] for m in finite),
             math.degrees(rng_msg['fov'])))
    print('  机体姿态      : yaw %.2f°  pitch %.2f°  roll %.2f°' % (yaw, pitch, roll))
    print('  时间对齐      : |range-gps| %.3fs   |imu-gps| %.3fs' % (dt_rng, dt_imu))

    print('\n--- 解算结果 ---')
    for label, bearing, de, dn, du, lat, lon, alt in solve_both(gps, q, rng_msg['range']):
        print('  [%s]' % label)
        print('     方位 %.2f°  平距 %.2f m  ENU 偏移 E%+.2f N%+.2f U%+.2f m'
              % (bearing, math.hypot(de, dn), de, dn, du))
        print('     目标 GPS  %.7f, %.7f, %.3f m' % (lat, lon, alt))
    return 0


if __name__ == '__main__':
    sys.exit(main())
