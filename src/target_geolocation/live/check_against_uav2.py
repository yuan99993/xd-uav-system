#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""live/check_against_uav2.py —— 用 uav2 的 GPS 当真值，统计激光云台定位的误差。

uav2 落在地面上静止，激光靶点就是它，所以直接拿它的 GPS 比。

除了直接的经纬高误差，还把误差拆成三个**物理量**（更好定位问题）：
  Δ斜距  = 测距 - 真值斜距         -> 测距仪零偏 / 靶点不是机体中心
  Δ方位  = 测量方位 - 真值方位     -> 云台 yaw / 航向 / 安装偏角
  Δ俯仰  = 测量俯仰 - 真值俯仰     -> 云台 pitch / 安装偏角

用法:
  python3 live/check_against_uav2.py --uri http://192.168.2.99:11311/ --n 40
"""
import argparse
import math
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from ros1_client import Master                                   # noqa: E402
from target_geolocation.geodesy import (                         # noqa: E402
    enu_to_geodetic, geodetic_to_enu, horizontal_distance,
    bearing_deg, distance_3d)

T_RANGE = '/Sim/SceneBasicDrone/robots/Drone1/sensors/FrontDistance/distance_sensor'
T_GIMBAL = '/uav1/gimbal/attitude'
T_GPS1 = '/uav1/mavros/global_position/global'
T_HDG = '/uav1/mavros/global_position/compass_hdg'
T_GPS2 = '/uav2/mavros/global_position/global'


def grab(uri, n_range, timeout=10.0):
    # 云台/航向频率低于测距时，多收几倍才能覆盖整段测距的时间窗
    plan = {'rng': (T_RANGE, n_range), 'gim': (T_GIMBAL, n_range * 4),
            'gps1': (T_GPS1, 10), 'hdg': (T_HDG, n_range * 4),
            'gps2': (T_GPS2, 10)}
    bag, err = {}, {}

    def w(k, tp, n):
        try:
            bag[k] = Master(uri).subscribe(tp, timeout=timeout, max_msgs=n)
        except Exception as e:                                    # noqa: BLE001
            err[k] = repr(e)

    ths = [threading.Thread(target=w, args=(k, tp, n)) for k, (tp, n) in plan.items()]
    for t in ths:
        t.daemon = True
        t.start()
    for t in ths:
        t.join(timeout + 10.0)
    return bag, err


def pct(v, p):
    if not v:
        return float('nan')
    s = sorted(v)
    k = min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))
    return s[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uri', default='http://192.168.2.99:11311/')
    ap.add_argument('--n', type=int, default=40, help='采样帧数')
    ap.add_argument('--skew', type=float, default=0.25, help='允许的最大时间差 [s]')
    args = ap.parse_args()

    bag, err = grab(args.uri, args.n)
    for k in err:
        print('[warn] %s 订阅失败: %s' % (k, err[k]))

    rng = [m for m in (bag.get('rng') or [])
           if m.get('range') is not None and math.isfinite(m['range'])
           and 0.05 < m['range'] < 1000.0]
    gim, gps1, hdg, gps2 = (bag.get('gim') or []), (bag.get('gps1') or []), \
        (bag.get('hdg') or []), (bag.get('gps2') or [])

    if not (rng and gim and gps1 and hdg and gps2):
        print('[×] 数据不全: rng=%d gim=%d gps1=%d hdg=%d gps2=%d'
              % (len(rng), len(gim), len(gps1), len(hdg), len(gps2)))
        return

    u2 = gps2[-1]
    print('uav2 真值: lat=%+.8f lon=%+.8f alt=%.3f m AMSL (n=%d 帧)'
          % (u2['lat'], u2['lon'], u2['alt'], len(gps2)))

    rows = []
    for r in rng:
        g = min(gim, key=lambda m: abs(m['_t'] - r['_t']))
        p = min(gps1, key=lambda m: abs(m['_t'] - r['_t']))
        h = min(hdg, key=lambda m: abs(m['_t'] - r['_t']))
        skew = max(abs(g['_t'] - r['_t']), abs(p['_t'] - r['_t']), abs(h['_t'] - r['_t']))
        if skew > args.skew:
            continue

        yaw, pitch = float(g['data'][0]), float(g['data'][1])
        brg = (yaw + float(h['data'])) % 360.0
        el = math.radians(pitch)
        br = math.radians(brg)
        horiz = r['range'] * math.cos(el)
        east = horiz * math.sin(br)
        north = horiz * math.cos(br)
        up = r['range'] * math.sin(el)
        tlat, tlon, talt = enu_to_geodetic(p['lat'], p['lon'], p['alt'], east, north, up)

        # ---- 真值：uav1 -> uav2 的矢量 ----
        te, tn = geodetic_to_enu(p['lat'], p['lon'], u2['lat'], u2['lon'])
        tu = u2['alt'] - p['alt']
        t_slant = math.sqrt(te ** 2 + tn ** 2 + tu ** 2)
        t_brg = (math.degrees(math.atan2(te, tn))) % 360.0
        t_el = math.degrees(math.asin(max(-1.0, min(1.0, tu / t_slant))))

        rows.append(dict(
            rng=r['range'], brg=brg, el=pitch,
            t_slant=t_slant, t_brg=t_brg, t_el=t_el,
            d_slant=r['range'] - t_slant,
            d_brg=((brg - t_brg + 180.0) % 360.0) - 180.0,
            d_el=pitch - t_el,
            d_h=horizontal_distance(u2['lat'], u2['lon'], tlat, tlon),
            # 水平误差按"沿真值视线 / 垂直真值视线"做矢量分解：
            # 横向小 = 指向准；径向大 = 只是打得近（激光停在靶标表面）
            d_along=(east - te) * math.sin(math.radians(t_brg))
            + (north - tn) * math.cos(math.radians(t_brg)),
            d_cross=(east - te) * math.cos(math.radians(t_brg))
            - (north - tn) * math.sin(math.radians(t_brg)),
            d_v=talt - u2['alt'],
            d_all=distance_3d(u2['lat'], u2['lon'], u2['alt'], tlat, tlon, talt),
            tlat=tlat, tlon=tlon, talt=talt,
            lat1=p['lat'], lon1=p['lon'], alt1=p['alt'], hdg=float(h['data']),
        ))

    if not rows:
        print('[×] 没有满足时间对齐的样本（skew > %.2f s）' % args.skew)
        return

    print('有效样本: %d / %d 帧\n' % (len(rows), len(rng)))

    print('--- 逐帧 ---')
    print('   # |   航向   | 云台yaw | 测距  | 真值斜距 |  Δ斜距 | 测量方位 | 真值方位 |  Δ方位 |'
          ' 测量俯仰 | 真值俯仰 | Δ俯仰 |  水平误差 | 垂直误差')
    print('  ' + '-' * 128)
    for i, x in enumerate(rows):
        print('  %2d | %7.2f | %7.2f | %5.2f | %8.2f | %+6.2f | %8.2f | %8.2f | %+6.2f |'
              ' %8.2f | %8.2f | %+6.2f | %8.2f | %+7.2f'
              % (i + 1, x['hdg'], (x['brg'] - x['hdg']) % 360.0, x['rng'], x['t_slant'],
                 x['d_slant'], x['brg'], x['t_brg'], x['d_brg'], x['el'], x['t_el'],
                 x['d_el'], x['d_h'], x['d_v']))

    def block(key, name, unit, signed=False):
        v = [x[key] for x in rows]
        m = sum(v) / len(v)
        sd = math.sqrt(sum((y - m) ** 2 for y in v) / max(1, len(v) - 1))
        print('  %-10s mean=%+8.3f  std=%7.3f  p95=%8.3f  max=%8.3f  %s'
              % (name, m, sd, pct([abs(y) for y in v], 95),
                 max(abs(y) for y in v), unit))
        return m, sd

    print('\n--- 统计（n=%d） ---' % len(rows))
    block('d_h', '水平误差', 'm')
    block('d_cross', '  ├ 横向', 'm')
    block('d_along', '  └ 径向', 'm')
    block('d_v', '垂直误差', 'm')
    block('d_all', '三维误差', 'm')
    print()
    ms, sds = block('d_slant', 'Δ斜距', 'm')
    mb, sdb = block('d_brg', 'Δ方位', 'deg')
    me, sde = block('d_el', 'Δ俯仰', 'deg')

    print('\n--- 误差分解（哪一项在拖后腿） ---')
    print('  测距零偏      : %+.2f m' % ms)
    print('  方位角偏差    : %+.2f deg  -> 在 %.1f m 处折合 %+.2f m 横向'
          % (mb, sum(x['t_slant'] for x in rows) / len(rows),
             sum(x['t_slant'] for x in rows) / len(rows) * math.radians(mb)))
    print('  俯仰角偏差    : %+.2f deg  -> 折合垂直 %+.2f m / 水平 %+.2f m'
          % (me,
             sum(x['t_slant'] for x in rows) / len(rows) * math.radians(me),
             sum(x['t_slant'] for x in rows) / len(rows) * math.radians(me)
             * math.tan(math.radians(19.85))))

    L = sum(x['t_slant'] for x in rows) / len(rows)
    print('\n  横向(指向)误差占比: %.1f%%   -> %s'
          % (100.0 * sum(abs(x['d_cross']) for x in rows) / max(1e-9, sum(x['d_h'] for x in rows)),
             '指向很准，误差几乎全在"打多远"上'
             if sum(abs(x['d_cross']) for x in rows) < 0.25 * sum(x['d_h'] for x in rows)
             else '横向也有明显偏差，要查 yaw/航向'))

    cand = [('测距', abs(ms)), ('方位', abs(L * math.radians(mb))),
            ('俯仰', abs(L * math.radians(me)))]
    cand.sort(key=lambda kv: -kv[1])
    print('\n  主导项: %s（%.2f m）' % (cand[0][0], cand[0][1]))
    print('  注意: 径向偏短是"激光停在靶标表面"的物理结果，横向才是真正的指向误差。')

    last = rows[-1]
    print('\n  最后一次解算: %.8f, %.8f, %.3f' % (last['tlat'], last['tlon'], last['talt']))
    print('  真值        : %.8f, %.8f, %.3f' % (u2['lat'], u2['lon'], u2['alt']))


if __name__ == '__main__':
    main()
