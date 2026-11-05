#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Jetson 上的关键文本文件拉回本地看（只读）。"""
from __future__ import print_function

import os
import sys

try:
    import paramiko
except ImportError:
    sys.exit('需要 paramiko')

HOST, USER, PWD = '192.168.2.99', 'nvidia', 'nvidia'
FILES = [
    '/home/nvidia/workspace/stream/player/ps_native_player.py',
    '/home/nvidia/workspace/stream/runtime/xorg3.conf',
]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '_remote')


def main():
    if not os.path.isdir(OUT):
        os.makedirs(OUT)
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, username=USER, password=PWD, timeout=12,
                banner_timeout=15, auth_timeout=15)
    sftp = cli.open_sftp()
    for path in FILES:
        name = os.path.basename(path)
        dst = os.path.join(OUT, name)
        try:
            sftp.get(path, dst)
            print('OK   %s -> %s (%d bytes)' % (path, dst, os.path.getsize(dst)))
        except Exception as exc:                  # noqa: BLE001
            print('FAIL %s: %s' % (path, exc))
    sftp.close()
    cli.close()


if __name__ == '__main__':
    main()
