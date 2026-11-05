#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""进程 CPU 时间序列：把总占用、热点线程占用、播放器 /status 计数对齐到同一条时间轴。

用来回答「为什么 CPU 偶尔会高」这类问题——单次 top 只给一个数，看不出是稳态
还是抖动，也看不出它在什么条件下抖。

用法:
    python bench/cpu_timeseries.py --pid 2786136 --seconds 60 --interval 5
"""
from __future__ import print_function

import argparse
import json
import sys
import time

try:
    import paramiko
except ImportError:
    sys.exit('需要 paramiko: python -m pip install paramiko')

HOST, USER, PWD = '192.168.2.99', 'nvidia', 'nvidia'
HZ = 100


def conn():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PWD, timeout=12,
              banner_timeout=15, auth_timeout=15)
    return c


def sh(cli, cmd, timeout=25):
    _i, o, e = cli.exec_command(cmd, timeout=timeout)
    return o.read().decode('utf-8', 'replace')


def total_ticks(cli, pid):
    out = sh(cli, 'cat /proc/%s/stat 2>/dev/null' % pid)
    lp, rp = out.find('('), out.rfind(')')
    if lp < 0:
        return None
    rest = out[rp + 2:].split()
    try:
        return int(rest[11]) + int(rest[12])
    except (IndexError, ValueError):
        return None


def thread_ticks(cli, pid):
    """返回 {tid: (comm, ticks)}"""
    cmd = ("for t in /proc/%s/task/*; do tid=${t##*/}; "
           "read -r line < $t/stat 2>/dev/null || continue; "
           "echo \"$tid|$line\"; done" % pid)
    out = sh(cli, cmd)
    res = {}
    for ln in out.splitlines():
        if '|' not in ln:
            continue
        tid, line = ln.split('|', 1)
        lp, rp = line.find('('), line.rfind(')')
        if lp < 0:
            continue
        try:
            rest = line[rp + 2:].split()
            res[int(tid)] = (line[lp + 1:rp], int(rest[11]) + int(rest[12]))
        except (IndexError, ValueError):
            continue
    return res


def status(cli):
    out = sh(cli, 'curl -s -m 3 http://127.0.0.1:8099/status')
    try:
        return json.loads(out)
    except Exception:                             # noqa: BLE001
        return {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pid', default='2786136')
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--interval', type=float, default=5)
    args = ap.parse_args()

    cli = conn()
    t0 = time.time()
    prev_tot = total_ticks(cli, args.pid)
    prev_thr = thread_ticks(cli, args.pid)
    prev_st = status(cli)
    prev_t = t0

    print('pid=%s  采样 %ds / 每 %.0fs' % (args.pid, args.seconds, args.interval))
    hdr = ('%-6s %7s %7s %6s %7s %9s %9s %6s'
           % ('t[s]', '总%', 'jb%', 'fps', 'kbps', 'shapeReq', 'optReq', 'holes'))
    print(hdr)
    print('-' * len(hdr))

    peaks = []
    while time.time() - t0 < args.seconds:
        time.sleep(args.interval)
        now = time.time()
        el = now - prev_t
        tot = total_ticks(cli, args.pid)
        thr = thread_ticks(cli, args.pid)
        st = status(cli)
        if tot is None or prev_tot is None:
            print('  进程不在了')
            break
        tot_pct = (tot - prev_tot) / float(HZ) / el * 100.0
        # 找 jitterbuffer 线程（可能有多个，取增量最大的）
        jb_pct = 0.0
        for tid, (comm, ticks) in thr.items():
            if comm == 'rtpjitterbuffer' and tid in prev_thr:
                p = (ticks - prev_thr[tid][1]) / float(HZ) / el * 100.0
                jb_pct = max(jb_pct, p)
        d_shape = (st.get('shapeReqs') or 0) - (prev_st.get('shapeReqs') or 0)
        d_opt = (st.get('optReqs') or 0) - (prev_st.get('optReqs') or 0)
        print('%-6.0f %6.1f%% %6.1f%% %6s %7s %9s %9s %6s'
              % (now - t0, tot_pct, jb_pct, st.get('fps'), st.get('kbps'),
                 d_shape, d_opt, st.get('holes')))
        peaks.append(tot_pct)
        prev_tot, prev_thr, prev_st, prev_t = tot, thr, st, now

    if peaks:
        print('-' * len(hdr))
        print('总占用  平均 %.1f%%  最高 %.1f%%  最低 %.1f%%  (n=%d)'
              % (sum(peaks) / len(peaks), max(peaks), min(peaks), len(peaks)))
    cli.close()


if __name__ == '__main__':
    main()
