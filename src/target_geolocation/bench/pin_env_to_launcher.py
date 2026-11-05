#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 PS_SINK_SYNC=1 固化进 start_ps_view.sh（在 Jetson 上运行）。

为什么需要固化：sync=False -> sync=True 的收益只在进程启动时读一次环境变量，
不写进启动脚本的话，下次重启播放器就退回旧行为（自旋 39%~73% 单核）。
"""
import io
import shutil
import sys

P = '/home/nvidia/workspace/stream/runtime/start_ps_view.sh'
BAK = P + '.bak-sync'

BLOCK = (
    '\n'
    '# sink 按时钟渲染。sync=False 时下游不节流，rtpjitterbuffer 在 live 源下\n'
    '# 会自旋烧 CPU（Jetson 实测占单核 39%~73%，栈顶恒为 sched_yield）。\n'
    '# 开启后同一路流降到 8% 左右，代价是多约一帧（50ms）的显示延迟。\n'
    '# 想压低延迟可同时调 PS_JB_LATENCY（毫秒，默认 50），太小会丢包抖动。\n'
    'export PS_SINK_SYNC=${PS_SINK_SYNC:-1}\n'
)

ANCHOR = 'export DISPLAY=${DISPLAY:-:3}\n'


def main():
    s = io.open(P, encoding='utf-8').read()
    if 'PS_SINK_SYNC' in s:
        print('已存在 PS_SINK_SYNC，跳过')
        return
    n = s.count(ANCHOR)
    if n != 1:
        sys.exit('锚点命中 %d 次（应为 1），中止' % n)
    shutil.copy2(P, BAK)
    io.open(P, 'w', encoding='utf-8').write(s.replace(ANCHOR, ANCHOR + BLOCK, 1))
    print('已写入，备份 -> %s' % BAK)


if __name__ == '__main__':
    main()
