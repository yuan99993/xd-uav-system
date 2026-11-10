#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判决性实验：uav1 云台 yaw 是世界系还是机体系？

同时订阅:
  /uav1/mavros/global_position/compass_hdg   机体航向 (deg, 0=北, 顺时针为正)
  /uav1/gimbal/attitude                     云台姿态 [yaw, pitch(上正), roll] (deg)
  /uav1/mavlink/from                         原始 MAVLink, 取 #285 GIMBAL_DEVICE_ATTITUDE_STATUS

判据:
  - 若 (云台yaw - 航向) 近似恒定  ->  云台 yaw 是机体系(相对机头), 需要加航向
  - 若 (云台yaw - 航向) 随航向变化而漂移, 且云台yaw 自己跟航向无关  ->  云台 yaw 是世界系, 直接用
  - #285 flags bit4(YAW_LOCK) 置位 -> MAVLink 语义上就是世界系(地球系) yaw

用法:
  python3 live/probe_gimbal_yaw.py --uri http://192.168.2.99:11311/ --secs 12
"""
import argparse
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros1_client import Master, _parse_mavlink  # noqa: E402


def fmt(v, w=9, p=2):
    if v is None:
        return ' ' * (w - 2) + '--'
    return ('%%%d.%df' % (w, p)) % v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uri', default='http://192.168.2.99:11311/')
    ap.add_argument('--secs', type=float, default=12.0)
    ap.add_argument('--ns', default='/uav1')
    args = ap.parse_args()

    ns = args.ns.rstrip('/')
    t_hdg = ns + '/mavros/global_position/compass_hdg'
    t_att = ns + '/gimbal/attitude'
    t_mav = ns + '/mavlink/from'
    t_nav = ns + '/mavros/global_position/global'

    m = Master(args.uri)
    print('master: %s' % args.uri)

    # ---- 主订阅: 云台姿态 (高频, 用它驱动时间轴) ----
    streams = {
        'att': (t_att, {'timeout': 8.0, 'max_msgs': 400}),
        'hdg': (t_hdg, {'timeout': 8.0, 'max_msgs': 400}),
        'mav': (t_mav, {'timeout': 8.0, 'max_msgs': 4000}),
        'nav': (t_nav, {'timeout': 8.0, 'max_msgs': 5}),
    }

    import threading
    bag = {}
    err = {}

    def worker(key, topic, kw):
        try:
            bag[key] = Master(args.uri).subscribe(topic, **kw)   # ServerProxy 非线程安全，各线程独立实例
        except Exception as e:  # noqa: BLE001
            err[key] = repr(e)

    ths = []
    for k, (tp, kw) in streams.items():
        t = threading.Thread(target=worker, args=(k, tp, kw))
        t.daemon = True
        t.start()
        ths.append(t)
    for t in ths:
        t.join(timeout=args.secs + 12.0)

    for k in err:
        print('  [warn] %s 订阅失败: %s' % (k, err[k]))

    att = bag.get('att') or []
    hdg = bag.get('hdg') or []
    mav = bag.get('mav') or []
    nav = bag.get('nav') or []

    print('样本: att=%d hdg=%d mav=%d nav=%d' % (len(att), len(hdg), len(mav), len(nav)))
    if not att:
        print('云台姿态无数据，无法判决。')
        return

    # ---- 解析 #285 ----
    q285 = []          # (t, yaw_deg_world, pitch_deg_up_pos, flags, locked)
    msgid_hist = {}
    for mm in mav:
        mid = mm.get('msgid')
        msgid_hist[mid] = msgid_hist.get(mid, 0) + 1
        if mid != 285:
            continue
        try:
            payload = mm.get('payload')
            if not payload or len(payload) < 28:
                continue
            # GIMBAL_DEVICE_ATTITUDE_STATUS:
            #   time_boot_ms u32 | q[4] float | target_system u8 | target_component u8
            #   flags u16 | failure_flags u32 | ... | delta_yaw float | delta_yaw_velocity float
            q = struct.unpack('<4f', payload[4:20])
            flags = struct.unpack('<H', payload[22:24])[0]
            qw, qx, qy, qz = q
            # 四元数 -> 世界系 yaw (ZYX, NED: yaw 0=北, 顺时针正) / pitch (抬头正)
            siny = 2.0 * (qw * qz + qx * qy)
            cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.degrees(math.atan2(siny, cosy))
            sinp = 2.0 * (qw * qy - qz * qx)
            sinp = max(-1.0, min(1.0, sinp))
            pitch = math.degrees(math.asin(sinp))
            q285.append(dict(t=mm.get('_t',0.0), yaw=yaw, pitch=pitch,
                             flags=flags, locked=bool(flags & 16)))
        except Exception:  # noqa: BLE001
            continue

    print('MAVLink 消息种类: %s' % sorted(msgid_hist.items(), key=lambda kv: -kv[1])[:10])
    print('#285 帧数: %d' % len(q285))

    att_yaw = [float(a['data'][0]) for a in att if len(a.get('data', [])) >= 3]
    att_pit = [float(a['data'][1]) for a in att if len(a.get('data', [])) >= 3]
    hd = [float(h['data']) for h in hdg if h.get('data') is not None]

    def stat(v, name, unit='deg'):
        if not v:
            print('  %-10s 无数据' % name)
            return
        print('  %-10s n=%4d  mean=%8.3f  min=%8.3f  max=%8.3f  %s'
              % (name, len(v), sum(v) / len(v), min(v), max(v), unit))

    print('\n--- 各路数据统计 ---')
    stat(att_yaw, '桥接yaw')
    stat(att_pit, '桥接pitch')
    stat(hd, '航向')
    if q285:
        stat([x['yaw'] for x in q285], '#285 yaw')
        stat([x['pitch'] for x in q285], '#285 pitch')

    # ---- 关键: flags ----
    if q285:
        flags = q285[0]['flags']
        print('\n#285 flags = 0x%04X (bit4 YAW_LOCK=%d)' % (flags, int(bool(flags & 16))))
        print('  -> %s' % ('世界系 yaw (地球系, YAW_LOCK 置位)' if flags & 16
                           else 'yaw 未锁定世界系 -> 可能是机体系'))

    # ---- 判决表: 云台yaw 与 航向 的关系 ----
    print('\n--- 判决表 (云台yaw 与 航向 的差) ---')
    print('  时间偏移(s) |   航向   | 桥接yaw | #285yaw | d=(yaw-hdg) | 桥接pitch | #285pitch')
    print('  ' + '-' * 82)

    att_t0 = att[0]['_t'] if att else 0.0
    hdg_t0 = hdg[0]['_t'] if hdg else 0.0
    mav_t0 = q285[0]['t'] if q285 else 0.0

    rows = 10
    idxs = [int(round(i * (len(att) - 1) / max(1, rows - 1))) for i in range(rows)] if len(att) > 1 else [0]
    ds = []
    for i in idxs:
        a = att[i]
        rel = a['_t'] - att_t0
        # 航向最接近的一条
        hbest = min(hdg, key=lambda h: abs(h['_t'] - a['_t'])) if hdg else None
        hval = float(hbest['data']) if hbest else None
        yv = float(a['data'][0]) if len(a.get('data', [])) >= 3 else None
        pv = float(a['data'][1]) if len(a.get('data', [])) >= 3 else None
        qbest = min(q285, key=lambda q: abs(q['t'] - a['_t'])) if q285 else None
        qy = qbest['yaw'] if qbest else None
        qp = qbest['pitch'] if qbest else None
        d = (yv - hval) if (yv is not None and hval is not None) else None
        if d is not None:
            ds.append(d)
        print('  %10.3f | %s | %s | %s | %s | %s | %s'
              % (rel, fmt(hval), fmt(yv), fmt(qy), fmt(d), fmt(pv), fmt(qp)))

    if len(ds) >= 3:
        mean_d = sum(ds) / len(ds)
        spread = max(ds) - min(ds)
        hdg_spread = (max(hd) - min(hd)) if len(hd) > 1 else 0.0
        print('\n  (云台yaw - 航向) : mean=%+.3f  spread=%.3f deg' % (mean_d, spread))
        print('  航向变化范围     : %.3f deg' % hdg_spread)
        print()
        if hdg_spread < 1.0:
            print('  [!] 航向几乎不变(%.2f deg)，本段数据无法区分世界系/机体系。' % hdg_spread)
            print('      但若 (云台yaw-航向) 的 mean 与 #285 yaw 接近而两者都在 60~70 度，')
            print('      结合 flag 判断即可。建议让飞机转弯后再跑一次。')
        elif spread < 0.5 * hdg_spread:
            print('  ==> d 基本恒定 (spread %.2f << 航向变化 %.2f)' % (spread, hdg_spread))
            print('      结论: 云台 yaw 是【机体系】(相对机头)，用的时候必须加航向。')
        elif abs(mean_d) > 0.5 * hdg_spread and spread > 0.5 * hdg_spread:
            print('  ==> d 跟着航向一起变，云台yaw 本身稳定')
            print('      结论: 云台 yaw 是【世界系】(已含航向)，直接当绝对方位用。')
        else:
            print('  ==> 数据区分度不足，需要更明显的机动。')

    # ---- 顺便: 桥接 pitch 与 #285 pitch 的符号关系 ----
    if q285 and att_pit:
        mp = sum(att_pit) / len(att_pit)
        mq = sum([x['pitch'] for x in q285]) / len(q285)
        print('\n  pitch 符号核对: 桥接 mean=%+.3f deg (上正)  #285 mean=%+.3f deg (上正)'
              % (mp, mq))
        print('    -> %s' % ('两者同号，桥接 pitch 已按"上正"给出' if mp * mq > 0
                            else '两者异号，需注意桥接 pitch 的符号约定'))

    if nav:
        n = nav[-1]
        print('\n  uav1 GPS: lat=%.8f lon=%.8f alt=%.3f' % (n['lat'], n['lon'], n['alt']))


if __name__ == '__main__':
    main()
