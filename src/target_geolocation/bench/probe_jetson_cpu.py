#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在 Jetson 上取证：视频链路各进程的线程级 CPU 归属。

用法（在开发机上）:
    python bench/probe_jetson_cpu.py [--host 192.168.2.99] [--pids 167369,165525]

设计要点:
  * 只读命令，不改任何系统状态
  * 每条命令独立 exec，避免长命令触发 paramiko EOFError
  * 线程级 top -H 是判断 GStreamer 里"哪个元件在吃 CPU"最直接的手段，
    线程名会直接显示 rtpjitterbuf / v4l2dec / nveglglessink 等
"""
from __future__ import print_function

import argparse
import sys
import time

try:
    import paramiko
except ImportError:
    sys.exit('需要 paramiko: python -m pip install paramiko')

DEFAULT_PIDS = '167369,165525,164147'


def connect(host, user, password, port=22):
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(hostname=host, port=port, username=user, password=password,
                timeout=12, banner_timeout=15, auth_timeout=15)
    return cli


def run(cli, cmd, timeout=20):
    """执行一条短命令，返回 (rc, out, err)。反复重连由调用方负责。"""
    try:
        _in, out, err = cli.exec_command(cmd, timeout=timeout)
        o = out.read().decode('utf-8', 'replace')
        e = err.read().decode('utf-8', 'replace')
        return out.channel.recv_exit_status(), o, e
    except Exception as exc:                      # noqa: BLE001
        return -1, '', '%s: %s' % (type(exc).__name__, exc)


def section(title):
    print('\n' + '=' * 72)
    print(title)
    print('=' * 72)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.2.99')
    ap.add_argument('--user', default='nvidia')
    ap.add_argument('--password', default='nvidia')
    ap.add_argument('--pids', default=DEFAULT_PIDS)
    args = ap.parse_args()

    pids = [p.strip() for p in args.pids.split(',') if p.strip()]
    cli = connect(args.host, args.user, args.password)
    print('已连接 %s@%s' % (args.user, args.host))

    # 1) 整机概况
    section('1. 整机负载')
    for cmd in ('nproc', 'uptime', 'free -m | head -3',
                "cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null",
                "nvpmodel -q 2>/dev/null | head -8"):
        rc, o, e = run(cli, cmd)
        print('$ %s' % cmd)
        print((o or e).rstrip())

    # 2) 线程级 CPU —— 核心证据
    for pid in pids:
        section('2. 线程级 CPU  pid=%s' % pid)
        rc, o, e = run(cli, 'cat /proc/%s/comm' % pid)
        print('进程名: %s' % o.strip())
        rc, o, e = run(cli, 'ls /proc/%s/task | wc -l' % pid)
        print('线程数: %s' % o.strip())
        rc, o, e = run(
            cli,
            'top -H -b -n 1 -p %s 2>/dev/null | tail -n +7 | head -18' % pid,
            timeout=25)
        print(o.rstrip() or e.rstrip())

    # 3) 显示环境：Xorg :3 到底是什么
    section('3. 显示环境 (DISPLAY :3)')
    rc, o, e = run(cli, "ps -eo pid,pcpu,etime,comm,args | grep -E 'X(org|vfb|dummy)|Xwayland' | grep -v grep")
    print(o.rstrip() or '(无输出)')
    rc, o, e = run(cli, "DISPLAY=:3 xdpyinfo 2>/dev/null | grep -E 'dimensions|depth of root|vendor string'")
    print(o.rstrip() or '(xdpyinfo 不可用/无 DISPLAY)')

    # 4) 视频进程的完整命令行 + 环境变量里与 GST/GL 相关的项
    for pid in pids:
        section('4. 进程环境  pid=%s' % pid)
        rc, o, e = run(cli, "tr '\\0' '\\n' < /proc/%s/environ 2>/dev/null | "
                            "grep -Ei '^(DISPLAY|GST_|LIBGL|NV|__GL|EGL|CUDA)' | sort | head -25" % pid)
        print(o.rstrip() or '(读取受限或无相关变量)')

    # 5) 该进程打开的设备/管道（判断是否真在用 NVDEC）
    for pid in pids:
        section('5. 关键 fd  pid=%s' % pid)
        rc, o, e = run(cli, "ls -l /proc/%s/fd 2>/dev/null | "
                            "grep -E 'nvidia|video|dri|dma|X11|socket' | head -14" % pid)
        print(o.rstrip() or '(无匹配)')

    cli.close()
    print('\n取证完成。')


if __name__ == '__main__':
    main()
