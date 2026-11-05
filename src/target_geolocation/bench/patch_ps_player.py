#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 ps_native_player.py 打「性能开关」补丁（在 Jetson 上运行）。

原则：
  * 默认行为完全不变 —— 三个新开关默认关闭，等价于改动前的行为
  * 先备份再改，改完做 py_compile 语法校验，失败自动回滚
  * 每处替换都校验命中次数，防止误改

开关（通过环境变量，只在启动时读一次）：
  PS_SINK_SYNC=1      nveglglessink 改为 sync=True（按时钟渲染，给 jitterbuffer
                      一个下游节流点，治它 73% 单核的自旋）
  PS_JB_LATENCY=N     rtpjitterbuffer 的 latency 毫秒数，默认 50
  PS_DEC_MAXPERF=1    nvv4l2decoder 打开 enable-max-performance

用法：
    python3 /tmp/patch_ps_player.py [--revert]
"""
from __future__ import print_function

import argparse
import os
import py_compile
import shutil
import sys
import time

TARGET = '/home/nvidia/workspace/stream/player/ps_native_player.py'

HELPER = '''
def _envflag(name, default=False):
    """读布尔型环境变量：1/true/yes/on 为真，其余为假，未设则用 default。"""
    v = os.environ.get(name)
    if v is None or v.strip() == '':
        return default
    return v.strip().lower() in ('1', 'true', 'yes', 'on')

'''

PATCHES = [
    # (唯一锚点, 替换后内容, 说明)
    (
        'FILL_WINDOW = True\n',
        'FILL_WINDOW = True\n' + HELPER,
        '插入 _envflag() 辅助函数',
    ),
    (
        "        self.jitter.set_property('latency', 50)\n",
        "        # PS_JB_LATENCY: 缓冲深度(ms)。太小会丢包/抖动，太大会加延迟。\n"
        "        self.jitter.set_property(\n"
        "            'latency', int(os.environ.get('PS_JB_LATENCY', '50')))\n",
        'jitterbuffer latency 可配',
    ),
    (
        "        self.dec = self._mk('nvv4l2decoder')\n",
        "        self.dec = self._mk('nvv4l2decoder')\n"
        "        if _envflag('PS_DEC_MAXPERF', False):\n"
        "            # 少留参考帧、最大化硬件吞吐，代价是轻微画质让步\n"
        "            try:\n"
        "                self.dec.set_property('enable-max-performance', True)\n"
        "            except Exception as _ex:\n"
        "                print('[ps] enable-max-performance 不可用:', repr(_ex),\n"
        "                      flush=True)\n",
        '解码器 max-performance 开关',
    ),
    (
        "        self.sink.set_property('sync', False)\n",
        "        # PS_SINK_SYNC=1: 让 sink 按时钟渲染。sync=False 时下游不节流，\n"
        "        # rtpjitterbuffer 在「队列空 + live 源」下会自旋烧 CPU（实测 73%\n"
        "        # 单核，栈顶恒为 sched_yield）。开启后多出约 latency 的显示延迟。\n"
        "        self.sink.set_property('sync', _envflag('PS_SINK_SYNC', False))\n",
        'sink sync 开关（核心修复）',
    ),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--revert', action='store_true', help='从备份恢复')
    ap.add_argument('--target', default=TARGET)
    args = ap.parse_args()

    bak = args.target + '.bak-perf'
    if args.revert:
        if not os.path.exists(bak):
            sys.exit('没有备份 %s' % bak)
        shutil.copy2(bak, args.target)
        print('已从备份恢复: %s' % bak)
        return

    with open(args.target, 'r', encoding='utf-8') as f:
        src = f.read()

    if '_envflag' in src:
        print('看起来已经打过补丁（检测到 _envflag），跳过。要重来先 --revert')
        return

    if not os.path.exists(bak):
        shutil.copy2(args.target, bak)
        print('已备份 -> %s' % bak)

    out = src
    for anchor, repl, desc in PATCHES:
        n = out.count(anchor)
        if n != 1:
            sys.exit('锚点命中 %d 次（应为 1），中止：%s | %r'
                     % (n, desc, anchor[:60]))
        out = out.replace(anchor, repl, 1)
        print('  [ok] %s' % desc)

    tmp = args.target + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(out)
    try:
        py_compile.compile(tmp, doraise=True)
    except Exception as exc:                      # noqa: BLE001
        os.unlink(tmp)
        sys.exit('语法校验失败，未改动原文件：%s' % exc)

    shutil.move(tmp, args.target)
    os.chmod(args.target, 0o755)
    print('\n补丁完成。默认行为不变；要用新行为，加环境变量重启：')
    print('  PS_SINK_SYNC=1 bash /home/nvidia/workspace/stream/runtime/restart_ps.sh')
    print('回滚: python3 %s --revert' % os.path.abspath(__file__))


if __name__ == '__main__':
    main()
