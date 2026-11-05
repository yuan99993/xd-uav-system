#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A/B 验证：给 ps_native_player 打开 sink sync 后，rtpjitterbuffer 的自旋是否消失。

流程（全程约 30 秒，画面会中断 8-12 秒后自动恢复）：
  1. 测当前进程的线程级 CPU 基线
  2. PS_SINK_SYNC=1 重启播放器（走官方 restart_ps.sh）
  3. 等它重新连上信令、重建解码链
  4. 再测一次线程级 CPU，并读 /status 确认还在正常出帧
  5. 打印前后对比 + 回滚命令（不自动回滚，由人判断）

用法:
    python bench/verify_sync_fix.py            # 完整 A/B
    python bench/verify_sync_fix.py --restore  # 回到 sync=False（原行为）
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
RESTART = '/home/nvidia/workspace/stream/runtime/restart_ps.sh'


def conn():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PWD, timeout=12,
              banner_timeout=15, auth_timeout=15)
    return c


def sh(cli, cmd, timeout=40):
    _i, o, e = cli.exec_command(cmd, timeout=timeout)
    out = o.read().decode('utf-8', 'replace')
    err = e.read().decode('utf-8', 'replace')
    return out, err


def find_pid(cli):
    out, _ = sh(cli, "pgrep -f 'ps_native_player[.]py' | head -1")
    s = out.strip()
    return int(s) if s.isdigit() else None


def thread_cpu(cli, pid, seconds=6.0):
    """返回 [(cpu%, tid, comm, state)]，按占用降序。"""
    def snap():
        cmd = ("for t in /proc/%s/task/*; do tid=${t##*/}; "
               "read -r line < $t/stat 2>/dev/null || continue; "
               "echo \"$tid|$line\"; done" % pid)
        out, _ = sh(cli, cmd)
        res = {}
        for ln in out.splitlines():
            if '|' not in ln:
                continue
            tid, line = ln.split('|', 1)
            lp, rp = line.find('('), line.rfind(')')
            if lp < 0 or rp < 0:
                continue
            comm = line[lp + 1:rp]
            rest = line[rp + 2:].split()
            try:
                res[int(tid)] = (comm, int(rest[11]) + int(rest[12]), rest[0])
            except (IndexError, ValueError):
                continue
        return res

    a = snap()
    t0 = time.time()
    time.sleep(seconds)
    b = snap()
    el = time.time() - t0
    rows = []
    for tid, (comm, ticks, state) in b.items():
        if tid not in a:
            continue
        pct = (ticks - a[tid][1]) / float(HZ) / el * 100.0
        rows.append((pct, tid, comm, state))
    rows.sort(reverse=True)
    return rows


def status(cli):
    out, _ = sh(cli, 'curl -s -m 4 http://127.0.0.1:8099/status')
    try:
        return json.loads(out)
    except Exception:                             # noqa: BLE001
        return None


def show(rows, label, n=6):
    print('\n%s' % label)
    print('  %-7s %-20s %5s %7s' % ('TID', 'COMM', 'STATE', 'CPU%'))
    print('  ' + '-' * 42)
    tot = 0.0
    for pct, tid, comm, state in rows:
        tot += pct
    for pct, tid, comm, state in rows[:n]:
        print('  %-7d %-20s %5s %6.1f%%' % (tid, comm[:20], state, pct))
    print('  ' + '-' * 42)
    print('  全进程合计 %.1f%%  （单核=100%%）' % tot)
    return tot


def wait_ready(cli, timeout=45):
    """等 /status 里 decoding=true 且 fps>0。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = status(cli)
        if st and st.get('decoding') and (st.get('fps') or 0) > 0:
            return st, time.time() - t0
        time.sleep(2)
    return status(cli), time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--restore', action='store_true', help='回到 sync=False')
    ap.add_argument('--window', type=float, default=6.0)
    args = ap.parse_args()

    cli = conn()

    if args.restore:
        pid = find_pid(cli)
        print('回滚到原行为（sync=False）…')
        out, err = sh(cli, 'bash %s' % RESTART, timeout=60)
        print(out.strip()[-400:] or err.strip()[-300:])
        time.sleep(8)
        st = status(cli)
        print('status: fps=%s decoding=%s' % (st and st.get('fps'),
                                               st and st.get('decoding')))
        cli.close()
        return

    pid0 = find_pid(cli)
    if not pid0:
        sys.exit('没找到播放器进程')
    print('当前播放器 pid=%s' % pid0)
    base = thread_cpu(cli, pid0, args.window)
    t_base = show(base, '【A】改动前（sink sync=False）')
    st0 = status(cli)
    print('  参考: fps=%s kbps=%s %sx%s' % (
        st0 and st0.get('fps'), st0 and st0.get('kbps'),
        st0 and st0.get('width'), st0 and st0.get('height')))

    print('\n>>> PS_SINK_SYNC=1 重启播放器（画面将中断约 10 秒）…')
    out, err = sh(cli, 'PS_SINK_SYNC=1 bash %s' % RESTART, timeout=70)
    print(out.strip()[-500:] or err.strip()[-300:])

    st, waited = wait_ready(cli, timeout=50)
    print('重新就绪用时 %.1fs' % waited)
    if st:
        print('  status: decoding=%s fps=%s kbps=%s' % (
            st.get('decoding'), st.get('fps'), st.get('kbps')))

    pid1 = find_pid(cli)
    print('新 pid=%s' % pid1)
    time.sleep(3)
    after = thread_cpu(cli, pid1, args.window)
    t_after = show(after, '【B】改动后（sink sync=True）')

    print('\n' + '=' * 60)
    print('结论：全进程单核占用 %.1f%%  ->  %.1f%%   (变化 %+.1f 个百分点)'
          % (t_base, t_after, t_after - t_base))
    if t_after < t_base * 0.5:
        print('自旋消失，改动有效。建议保留 sync=True。')
    else:
        print('收益不明显，需换方案（去掉外部 jitterbuffer）。')
        print('回滚: PS_SINK_SYNC 不设即可，或 python %s --restore' % sys.argv[0])
    print('=' * 60)
    cli.close()


if __name__ == '__main__':
    main()
