#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 TID（LWP）精确采样某个线程的用户态调用栈。

GStreamer 线程同名，gdb 的 `thread find <name>` 会返回多个编号，按名字采样
很容易打偏。这里先 attach 读 `info threads`，用 (LWP <tid>) 把 gdb 编号锁定，
再反复 attach 采样同一线程。

用法:
    python bench/sample_tid.py --pid 167369 --tid 2734562 --rounds 10
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


def gdb(cli, pid, tid, depth=16, probe=False):
    exs = ['-ex "set pagination off"', '-ex "set confirm off"']
    if probe:
        exs.append('-ex "info threads"')
    else:
        exs += ['-ex "thread %d"' % tid, '-ex "bt %d"' % depth]
    cmd = ('echo nvidia | sudo -S -p "" gdb -p %s -batch -nx %s -ex detach 2>&1'
           % (pid, ' '.join(exs)))
    _i, o, _e = cli.exec_command(cmd, timeout=50)
    return o.read().decode('utf-8', 'replace')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pid', default='167369')
    ap.add_argument('--tid', default='2734562')
    ap.add_argument('--rounds', type=int, default=10)
    ap.add_argument('--gap', type=float, default=0.8)
    ap.add_argument('--depth', type=int, default=16)
    args = ap.parse_args()

    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, username=USER, password=PWD, timeout=12,
                banner_timeout=15, auth_timeout=15)

    txt = gdb(cli, args.pid, args.tid, probe=True)
    # 实际格式: `  32   Thread 0xffff1234 (LWP 2734562) "rtpjitterbuffer" ...`
    m = re.search(r'^\s*(\d+)\s+Thread 0x[0-9a-f]+\s+\(LWP %s\)' % args.tid,
                  txt, re.M)
    if not m:
        print('没在 info threads 里找到 LWP %s，输出片段：' % args.tid)
        for ln in txt.splitlines():
            if 'LWP' in ln:
                print('   ', ln.strip())
        cli.close()
        return
    idx = int(m.group(1))
    print('LWP %s 对应 gdb 线程 #%d' % (args.tid, idx))

    tops = collections.Counter()
    frames_all = collections.Counter()
    for r in range(args.rounds):
        txt = gdb(cli, args.pid, idx, depth=args.depth)
        fr = re.findall(r'^#(\d+)\s+(?:0x[0-9a-f]+ in )?([^(\n]+)', txt, re.M)
        if fr:
            top = fr[0][1].strip()
            tops[top] += 1
            for _n, fn in fr:
                frames_all[fn.strip()] += 1
            print('\n--- round %d  top=%s' % (r + 1, top))
            for n, fn in fr[:7]:
                print('   #%s %s' % (n, fn.strip()))
        else:
            print('\n--- round %d  (无栈)' % (r + 1))
        time.sleep(args.gap)

    print('\n' + '=' * 68)
    print('栈顶分布（%d 次）' % args.rounds)
    print('=' * 68)
    for fn, c in tops.most_common():
        print('%4d 次  %s' % (c, fn))
    print('\n' + '=' * 68)
    print('出现在栈里的函数（次数越多越深/越常见）')
    print('=' * 68)
    for fn, c in frames_all.most_common(20):
        print('%4d  %s' % (c, fn))
    cli.close()


if __name__ == '__main__':
    main()
