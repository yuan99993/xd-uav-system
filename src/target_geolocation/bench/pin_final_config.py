#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固化 sync + max-lateness 组合（在 Jetson 上运行）。

背景（实测数据，同一路 1280x720@60fps 流）：
    sync=False                    -> 94.9% 单核，满帧（原始行为）
    sync=True, max-lateness=20ms  -> 33.7% 单核，但周期性丢帧（fps 掉到 0.9）
    sync=True, max-lateness=-1    -> 59.2% 单核，满帧不丢帧  <-- 采用

sync=True 给 jitterbuffer 一个下游节流点，消掉大部分忙等；但 basesink 默认
20ms 的 max-lateness 会把 RTP 时间戳换算后偏晚的帧整批丢掉，所以必须放宽。
"""
import io
import shutil
import sys

P = '/home/nvidia/workspace/stream/runtime/start_ps_view.sh'
BAK = P + '.bak-final'

OLD = 'export PS_SINK_SYNC=${PS_SINK_SYNC:-0}\n'
NEW = (
    'export PS_SINK_SYNC=${PS_SINK_SYNC:-1}\n'
    '\n'
    '# 放宽 basesink 的 max-lateness。sync=True 时默认只有 20ms，RTP 时间戳\n'
    '# 换算后偏晚的帧会被成批丢弃（实测 fps 一度掉到 0.9，日志刷\n'
    '# gst_base_sink_is_too_late）。-1 = 永不判 late。\n'
    '# 组合效果：94.9% -> 59.2% 单核，且全程 60fps 不丢帧。\n'
    'export PS_MAX_LATENESS=${PS_MAX_LATENESS:--1}\n'
)


def main():
    s = io.open(P, encoding='utf-8').read()
    if 'PS_MAX_LATENESS' in s:
        print('已固化过，跳过')
        return
    if s.count(OLD) != 1:
        sys.exit('锚点命中 %d 次，中止' % s.count(OLD))
    shutil.copy2(P, BAK)
    io.open(P, 'w', encoding='utf-8').write(s.replace(OLD, NEW, 1))
    print('已写入，备份 -> %s' % BAK)


if __name__ == '__main__':
    main()
