#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按线程做精确 CPU 增量测量（读 /proc/<pid>/task/*/stat 两次求差）。

top -b -n 1 的 %CPU 在 batch 首次迭代口径不稳（可能是累计平均），
这里用两次快照的 utime+stime 做差，配合 HZ=100 直接算出真实占用率，
和 top 的相互作用无关。同时输出线程的 wchan / 状态，判断是「在算」还是「在等」。

用法:
    python bench/thread_cpu.py --pid 167369 --seconds 5
"""
from __future__ import print_function

import argparse
import sys
import time

try:
    import paramiko
except ImportError:
    sys.exit('需要 paramiko: python -m pip install paramiko')

HOST, USER, PWD = '192.168.2.99', 'nvidia', 'nvidia'
HZ = 100  # Linux 用户态 HZ，Jetson L4T 固定 100


def read_threads(cli, pid):
    """返回 {tid: (comm, ticks, state)}，ticks = utime+stime。"""
    cmd = ("for t in /proc/%s/task/*; do "
           "tid=${t##*/}; "
           "read -r line < $t/stat 2>/dev/null || continue; "
           "echo \"$tid|$line\"; done" % pid)
    _i, o, _e = cli.exec_command(cmd, timeout=25)
    out = o.read().decode('utf-8', 'replace')
    res = {}
    for ln in out.splitlines():
        if '|' not in ln:
            continue
        tid, line = ln.split('|', 1)
        # comm 可能含空格，被包在 (...) 里
        lp = line.find('(')
        rp = line.rfind(')')
        if lp < 0 or rp < 0:
            continue
        comm = line[lp + 1:rp]
        rest = line[rp + 2:].split()
        # rest[0]=state, rest[11]=utime, rest[12]=stime  (从 state 起算)
        try:
            state = rest[0]
            ticks = int(rest[11]) + int(rest[12])
        except (IndexError, ValueError):
            continue
        res[int(tid)] = (comm, ticks, state)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pid', default='167369')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--top', type=int, default=12)
    args = ap.parse_args()

    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, username=USER, password=PWD, timeout=12,
                banner_timeout=15, auth_timeout=15)

    a = read_threads(cli, args.pid)
    t0 = time.time()
    time.sleep(args.seconds)
    b = read_threads(cli, args.pid)
    el = time.time() - t0

    rows = []
    for tid, (comm, ticks, state) in b.items():
        if tid not in a:
            continue
        d = ticks - a[tid][1]
        pct = d / float(HZ) / el * 100.0
        rows.append((pct, tid, comm, state, d))
    rows.sort(reverse=True)

    print('pid=%s  窗口=%.2fs  线程数=%d' % (args.pid, el, len(rows)))
    print('%-7s %-22s %5s %6s %8s' % ('TID', 'COMM', 'STATE', 'CPU%', 'ticks'))
    print('-' * 56)
    tot = 0.0
    for pct, tid, comm, state, d in rows[:args.top]:
        print('%-7d %-22s %5s %5.1f%% %8d' % (tid, comm[:22], state, pct, d))
        tot += pct
    print('-' * 56)
    print('前 %d 线程合计 %.1f%%' % (min(args.top, len(rows)), tot))

    cli.close()
    return rows


if __name__ == '__main__':
    main()
