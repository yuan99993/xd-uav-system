#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""live/locate_gimbal.py —— 用 uav1 的实测数据在线定位目标 GPS。

数据源（用户指定）:
  距离:   /Sim/SceneBasicDrone/robots/Drone1/sensors/FrontDistance/distance_sensor
  云台:   /uav1/gimbal/attitude          std_msgs/Float64MultiArray [yaw, pitch, roll] (deg)
  飞机:   /uav1/mavros/global_position/global          NavSatFix (AMSL)
  航向:   /uav1/mavros/global_position/compass_hdg     Float64 (deg)

几何（已由 live/probe_gimbal_yaw.py 实测验证）:
  /uav1/gimbal/attitude 的 yaw 是【机体系】量，世界方位角 = yaw + 航向 (mod 360)
  验证依据: 同一时刻 /uav1/mavlink/from 的 #285 GIMBAL_DEVICE_ATTITUDE_STATUS
            给出的世界 yaw 恒为 89.6°，而 bridge = #285_yaw - 航向 严格成立。
  pitch 为"上正"，-19.85° 表示下俯 19.85°。

用法:
  python3 live/locate_gimbal.py --uri http://192.168.2.99:11311/
  python3 live/locate_gimbal.py --n 5 --gap 0.5      # 连续打印 5 次
"""
import argparse
import math
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from ros1_client import Master                       # noqa: E402
from target_geolocation.geodesy import (             # noqa: E402
    enu_to_geodetic, horizontal_distance, bearing_deg, distance_3d)

T_RANGE = '/Sim/SceneBasicDrone/robots/Drone1/sensors/FrontDistance/distance_sensor'
T_GIMBAL = '/uav1/gimbal/attitude'
T_GPS = '/uav1/mavros/global_position/global'
T_HDG = '/uav1/mavros/global_position/compass_hdg'
T_RELALT = '/uav1/mavros/global_position/rel_alt'


def grab(uri, plan, timeout=8.0):
    """并发订阅多路话题，各自独立 Master 实例（ServerProxy 非线程安全）。"""
    out, err = {}, {}

    def w(k, tp, n):
        try:
            out[k] = Master(uri).subscribe(tp, timeout=timeout, max_msgs=n)
        except Exception as e:                        # noqa: BLE001
            err[k] = repr(e)

    ths = []
    for k, (tp, n) in plan.items():
        t = threading.Thread(target=w, args=(k, tp, n))
        t.daemon = True
        t.start()
        ths.append(t)
    for t in ths:
        t.join(timeout=timeout + 10.0)
    return out, err


def pick(msgs, key):
    if not msgs:
        return None
    return min(msgs, key=lambda m: abs(m['_t'] - key)) if key else msgs[-1]


def solve(lat, lon, alt, hdg, gyaw, gpitch, rng, yaw_mode='body'):
    """返回目标位置字典。yaw_mode: 'body'(yaw+heading) | 'world'(yaw 直接用)。"""
    bearing = (gyaw + hdg) % 360.0 if yaw_mode == 'body' else (gyaw % 360.0)
    el = math.radians(gpitch)                          # 上正
    br = math.radians(bearing)
    horiz = rng * math.cos(el)
    east = horiz * math.sin(br)
    north = horiz * math.cos(br)
    up = rng * math.sin(el)
    tlat, tlon, talt = enu_to_geodetic(lat, lon, alt, east, north, up)
    return dict(lat=tlat, lon=tlon, alt=talt, bearing=bearing,
                east=east, north=north, up=up,
                horiz=horiz, slant=rng, el=gpitch)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uri', default='http://192.168.2.99:11311/')
    ap.add_argument('--n', type=int, default=1, help='打印几次')
    ap.add_argument('--gap', type=float, default=1.0, help='两次之间间隔秒')
    args = ap.parse_args()

    for i in range(args.n):
        if i:
            import time
            time.sleep(args.gap)
        run_once(args.uri, i + 1, args.n)


def run_once(uri, idx, total):
    plan = {'rng': (T_RANGE, 40), 'gim': (T_GIMBAL, 40),
            'gps': (T_GPS, 5), 'hdg': (T_HDG, 40), 'ral': (T_RELALT, 5)}
    bag, err = grab(uri, plan)

    for k in err:
        print('[warn] %s 订阅失败: %s' % (k, err[k]))

    rng_msgs = [m for m in (bag.get('rng') or [])
                if m.get('range') is not None and math.isfinite(m['range'])
                and 0.05 < m['range'] < 1000.0]
    gim = bag.get('gim') or []
    gps = bag.get('gps') or []
    hdg = bag.get('hdg') or []
    ral = bag.get('ral') or []

    if not (rng_msgs and gim and gps and hdg):
        print('[×] 数据不全: range=%d gimbal=%d gps=%d hdg=%d'
              % (len(rng_msgs), len(gim), len(gps), len(hdg)))
        if rng_msgs == []:
            print('    距离传感器无有效回波（inf/超量程），等它测到东西再跑。')
        return

    # 时间对齐：以距离帧为基准，各取最近的一帧
    t0 = rng_msgs[-1]['_t']
    r = min(rng_msgs, key=lambda m: abs(m['_t'] - t0))
    g = min(gim, key=lambda m: abs(m['_t'] - t0))
    p = min(gps, key=lambda m: abs(m['_t'] - t0))
    h = min(hdg, key=lambda m: abs(m['_t'] - t0))
    ra = pick(ral, t0)

    gyaw, gpitch, groll = (float(g['data'][0]), float(g['data'][1]), float(g['data'][2]))
    skew = max(abs(r['_t'] - t0), abs(g['_t'] - t0), abs(p['_t'] - t0), abs(h['_t'] - t0))

    print('=' * 78)
    print(' 第 %d/%d 次    数据时差 %.3f s' % (idx, total, skew))
    print('=' * 78)
    print(' 飞机 GPS   : lat=%+.8f  lon=%+.8f  alt=%.3f m (AMSL)'
          % (p['lat'], p['lon'], p['alt']))
    if ra:
        print(' 相对高度   : %.3f m' % float(ra['data']))
    print(' 航向       : %.3f deg (0=北, 顺时针)' % float(h['data']))
    print(' 距离传感器 : %.3f m' % r['range'])
    print(' 云台姿态   : yaw=%8.3f  pitch=%8.3f (上正)  roll=%6.3f  deg'
          % (gyaw, gpitch, groll))

    sol_b = solve(p['lat'], p['lon'], p['alt'], float(h['data']), gyaw, gpitch, r['range'], 'body')
    sol_w = solve(p['lat'], p['lon'], p['alt'], float(h['data']), gyaw, gpitch, r['range'], 'world')

    print('-' * 78)
    print(' 【采用】世界方位角 = 云台yaw + 航向 = %.3f + %.3f = %.3f deg'
          % (gyaw, float(h['data']), sol_b['bearing']))
    print('   水平分量 %.3f m   垂直分量 %+.3f m   斜距 %.3f m   俯仰 %.3f deg'
          % (sol_b['horiz'], sol_b['up'], sol_b['slant'], sol_b['el']))
    print()
    print('   >>> 目标 GPS = %.8f, %.8f,  %.3f m (AMSL)'
          % (sol_b['lat'], sol_b['lon'], sol_b['alt']))
    print()

    # 地面一致性：若目标在地面，其高度应≈飞机高度 - L*sin(俯角)
    ground_alt = p['alt'] + sol_b['up']
    print(' 地面命中校验: 飞机高度 %.2f m + 垂直分量 %+.2f m = %.2f m'
          % (p['alt'], sol_b['up'], ground_alt))
    print('              目标离飞机水平距离 %.2f m, 方位 %.2f deg'
          % (sol_b['horiz'], sol_b['bearing']))
    if sol_b['up'] < 0:
        print('              => 射线朝下，落点在地面以上 %.2f m 的高度' % ground_alt)

    print('-' * 78)
    print(' 【对照】若把云台yaw 当世界系直接用: 方位 %.3f deg' % sol_w['bearing'])
    print('         目标 GPS = %.8f, %.8f, %.3f m   (偏差 %.1f m)'
          % (sol_w['lat'], sol_w['lon'], sol_w['alt'],
             horizontal_distance(sol_b['lat'], sol_b['lon'], sol_w['lat'], sol_w['lon'])))
    print('─' * 78)


if __name__ == '__main__':
    main()
