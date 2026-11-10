#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench/cpu_profile.py —— 量化解算节点到底吃多少 CPU（不需要 ROS）。

做法：把热路径拆成几个可独立计时的环节，各跑 N 次取平均，最后折算成
"在 X Hz 下占单核百分之几"。这样能把"感觉慢"换成数字。

用法:
  python3 bench/cpu_profile.py
  python3 bench/cpu_profile.py --n 20000 --hz 10
"""
import argparse
import math
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))

from target_geolocation import geodesy, solver           # noqa: E402


def timeit(fn, n, *a, **kw):
    # 预热，避开首次调用时的 import/分支预测开销
    for _ in range(min(200, n)):
        fn(*a, **kw)
    t0 = time.perf_counter()
    for _ in range(n):
        fn(*a, **kw)
    dt = time.perf_counter() - t0
    return dt / n * 1e6                                   # µs/次


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=20000)
    ap.add_argument('--hz', type=float, default=10.0, help='节点发布频率')
    ap.add_argument('--mav-hz', type=float, default=250.0, help='mavlink 帧率')
    args = ap.parse_args()

    lat, lon, alt = -37.71755, 144.83375, 9.94
    q = solver.quat_from_euler_zyx(0.0, math.radians(-19.848), math.radians(89.7))
    rng = 47.4

    print('=' * 74)
    print(' 解算热路径逐环节耗时 (n=%d)' % args.n)
    print('=' * 74)

    t_enu = timeit(geodesy.enu_to_geodetic, args.n, lat, lon, alt, 44.5, 0.3, -16.1)
    print('  enu_to_geodetic (4 次迭代)      %8.2f us' % t_enu)

    t_solve = timeit(solver.solve_target, args.n, lat, lon, alt, q, rng)
    print('  solve_target (含上式)           %8.2f us' % t_solve)

    # 模拟一次完整 tick：解算 + 4 条消息的对象构造
    class _P(object):
        __slots__ = ('x', 'y', 'z')

    def one_tick():
        r = solver.solve_target(lat, lon, alt, q, rng)
        pts = [tuple((r['east'], r['north'], r['up'])) for _ in range(4)]
        return r, pts

    t_tick = timeit(one_tick, args.n)
    print('  一次 tick（解算+4条消息）        %8.2f us' % t_tick)

    print()
    print('  -> 在 %.0f Hz 下，解算部分占用单核 %.4f%%'
          % (args.hz, t_tick * args.hz / 1e6 * 100.0))
    print('  -> 定时器空转（输入未变时直接返回）：一次 tick 约 %.2f us'
          % timeit(lambda: None, args.n))
    print('  -> 在 100 Hz 下，也只占 %.3f%%' % (t_tick * 100 / 1e6 * 100.0))

    # ---------------------------------------------------------------- mavlink
    print()
    print('=' * 74)
    print(' MAVLink 路径：每帧都重建+解析 vs 先按 msgid 过滤')
    print('=' * 74)

    payload64 = (0x3F8F5C2900000000, 0x0000000000000000, 0x0000000000000000,
                 0x0000000000000000, 0x0000000000000000)
    other64 = (0x1111111122222222,) * 5

    def build_and_parse(words):
        """当前实现：不论什么消息都拼帧 + pymavlink 解析。"""
        payload = b''.join(struct.pack('<Q', w) for w in words)
        header = struct.pack('<BBBBBBB', 253, len(payload), 0, 0, 1, 1, 1)
        header += bytes([0x1D, 0x01, 0x00])               # msgid=285 小端
        frame = header + payload + struct.pack('<H', 0)
        return len(frame)

    def filter_then_decode(msgid, words):
        """优化后：非 285 直接 return；是 285 才解 4 个 float。"""
        if msgid != 285:
            return None
        payload = b''.join(struct.pack('<Q', w) for w in words)
        return struct.unpack('<4f', payload[4:20])

    t_old = timeit(build_and_parse, args.n, other64)
    t_new = timeit(filter_then_decode, args.n, 30, other64)
    print('  旧（每帧拼帧+解析，非285也做）   %8.2f us/帧' % t_old)
    print('  新（先判 msgid，非285直接返回）  %8.2f us/帧' % t_new)
    print('  单帧省 %.2f us，%.0f Hz 下省 %.4f%% 单核'
          % (t_old - t_new, args.mav_hz,
             (t_old - t_new) * args.mav_hz / 1e6 * 100.0))
    print()
    print('  按 %.0f Hz 估算：旧 %.4f%% 单核 -> 新 %.4f%% 单核（省 %.0f 倍）'
          % (args.mav_hz, t_old * args.mav_hz / 1e6 * 100.0,
             t_new * args.mav_hz / 1e6 * 100.0, t_old / t_new))

    # -------------------------------------------------- 图像 base64 的代价
    print()
    print('=' * 74)
    print(' 参照：把图像 base64 后发 websocket 的代价（不是本节点的开销）')
    print('=' * 74)
    import base64
    for kb in (200, 1024, 3072):
        buf = b'\x00' * (kb * 1024)
        n = max(20, args.n // 50)
        t = timeit(base64.b64encode, n, buf)
        print('  %5d KB 单帧 base64 %7.1f us  -> 30 fps 下 %6.1f%% 单核'
              % (kb, t, t * 30 / 1e6 * 100.0))


if __name__ == '__main__':
    main()
