#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对 Jetson 上 ps_native_player 的热线程做 gdb 栈采样。

用 attach/detach 轮询拿函数级热点。每次 attach 会 stop-the-world 约 0.3-1s，
8 次采样分散在 ~12s 内，画面会轻微顿挫，但不改任何进程状态、不重启服务。

用法:
    python bench/sample_threads.py --pid 167369 --rounds 8
"""
from __future__ import print_function

import argparse
import collections
import re
import sys
import time

try:
    import paramiko
except ImportError:
    sys.exit('需要 paramiko: python -m pip install paramiko')

HOST, USER, PWD = '192.168.2.99', 'nvidia', 'nvidia'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pid', default='167369')
    ap.add_argument('--thread-name', default='rtpjitterbuffer')
    ap.add_argument('--rounds', type=int, default=8)
    ap.add_argument('--gap', type=float, default=1.5)
    ap.add_argument('--depth', type=int, default=14)
    args = ap.parse_args()

    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, username=USER, password=PWD, timeout=12,
                banner_timeout=15, auth_timeout=15)

    # 先问一下有几号 gdb 线程叫这个名字
    cmd0 = ('echo nvidia | sudo -S -p "" gdb -p %s -batch -nx '
            '-ex "set pagination off" -ex "set confirm off" '
            '-ex "thread find %s" -ex detach 2>&1 | grep "has target name"'
            % (args.pid, args.thread_name))
    _i, o, _e = cli.exec_command(cmd0, timeout=45)
    found = o.read().decode('utf-8', 'replace')
    idxs = [int(m) for m in re.findall(r'Thread (\d+) has target name', found)]
    print('匹配线程（gdb 编号）: %s' % idxs)
    if not idxs:
        print('没找到，退出')
        return
    tid = idxs[0]

    tops = collections.Counter()
    for r in range(args.rounds):
        cmd = ('echo nvidia | sudo -S -p "" gdb -p %s -batch -nx '
               '-ex "set pagination off" -ex "set confirm off" '
               '-ex "thread %d" -ex "bt %d" -ex detach 2>&1'
               % (args.pid, tid, args.depth))
        _i, o, _e = cli.exec_command(cmd, timeout=45)
        txt = o.read().decode('utf-8', 'replace')
        frames = re.findall(r'^#(\d+)\s+(?:0x[0-9a-f]+ in )?([^(\n]+)', txt,
                            re.M)
        if frames:
            top = frames[0][1].strip()
            tops[top] += 1
            print('\n--- round %d  top=%s' % (r + 1, top))
            for n, fn in frames[:6]:
                print('   #%s %s' % (n, fn.strip()))
        else:
            print('\n--- round %d  (无栈)' % (r + 1))
        time.sleep(args.gap)

    print('\n' + '=' * 66)
    print('栈顶函数分布（%d 次采样）' % args.rounds)
    print('=' * 66)
    for fn, c in tops.most_common():
        print('%4d 次  %s' % (c, fn))

    cli.close()


if __name__ == '__main__':
    main()
