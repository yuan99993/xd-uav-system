#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 ps_native_player.py 追加两个开关（在 Jetson 上运行）。

新增：
  PS_NO_JB=1        跳过外部 rtpjitterbuffer，webrtcbin 的 pad 直接进 rtph264depay。
                    webrtcbin 内部的 rtpbin 本身已经带了 jitterbuffer（采样时能看到
                    三个同名线程，其中两个属于它），外面这个属于重复处理。
  PS_MAX_LATENESS=N 设置 sink 的 max-lateness（纳秒，-1 = 永不判 late）。
                    sync=True 时默认 20ms，RTP 时间戳换算出偏晚就成批丢帧
                    （实测 fps 掉到 0.9），把它放大/关掉可以满帧。

默认都不生效，行为与现在完全一致。
"""
import io
import shutil
import sys

TARGET = '/home/nvidia/workspace/stream/player/ps_native_player.py'
BAK = TARGET + '.bak-jb'

# 1) jitterbuffer 改成可选
OLD_JB = """        self.jitter = self._mk('rtpjitterbuffer', 'jitter')
        # PS_JB_LATENCY: 缓冲深度(ms)。太小会丢包/抖动，太大会加延迟。
"""
NEW_JB = """        # PS_NO_JB=1: 不建外部 jitterbuffer，依赖 webrtcbin 内部 rtpbin 的那个。
        _no_jb = _envflag('PS_NO_JB', False)
        self.jitter = None if _no_jb else self._mk('rtpjitterbuffer', 'jitter')
        if self.jitter is not None:
            # PS_JB_LATENCY: 缓冲深度(ms)。太小会丢包/抖动，太大会加延迟。
"""

# latency 那两行必须缩进进新的 if
OLD_LAT = """        self.jitter.set_property(
            'latency', int(os.environ.get('PS_JB_LATENCY', '50')))
"""
NEW_LAT = """            self.jitter.set_property(
                'latency', int(os.environ.get('PS_JB_LATENCY', '50')))
"""

# 2) 链首元素随开关变化
OLD_CHAIN = """        chain = [self.jitter, self.depay, self.parse, self.dec, self.conv,
                 self.capsf, self.sink]
"""
NEW_CHAIN = """        _head = [] if self.jitter is None else [self.jitter]
        chain = _head + [self.depay, self.parse, self.dec, self.conv,
                         self.capsf, self.sink]
"""

# 3) pad.link 的目标随开关变化
OLD_LINK = """        ret = pad.link(self.jitter.get_static_pad('sink'))
"""
NEW_LINK = """        _sink_pad = (self.depay if self.jitter is None
                     else self.jitter).get_static_pad('sink')
        ret = pad.link(_sink_pad)
"""

# 4) max-lateness
OLD_SINK = """        self.sink.set_property('sync', _envflag('PS_SINK_SYNC', False))
"""
NEW_SINK = """        self.sink.set_property('sync', _envflag('PS_SINK_SYNC', False))
        # PS_MAX_LATENESS（纳秒）：sync=True 时默认 20ms，RTP 时间戳算出来偏晚
        # 就成批丢帧。设大或设 -1 可以满帧，代价是这些迟到的帧会让延迟上涨。
        _ml = os.environ.get('PS_MAX_LATENESS', '').strip()
        if _ml:
            try:
                self.sink.set_property('max-lateness', int(_ml))
                print('[ps] max-lateness = %s ns' % _ml, flush=True)
            except Exception as _ex:
                print('[ps] max-lateness 设置失败:', repr(_ex), flush=True)
"""

PATCHES = [
    (OLD_JB, NEW_JB, 'jitterbuffer 可选（PS_NO_JB）'),
    (OLD_LAT, NEW_LAT, 'latency 缩进进 if'),
    (OLD_CHAIN, NEW_CHAIN, '解码链首元素随开关'),
    (OLD_LINK, NEW_LINK, 'pad.link 目标随开关'),
    (OLD_SINK, NEW_SINK, 'sink max-lateness'),
]


def main():
    s = io.open(TARGET, encoding='utf-8').read()
    if 'PS_NO_JB' in s:
        print('已打过该补丁，跳过')
        return
    shutil.copy2(TARGET, BAK)
    out = s
    for anchor, repl, desc in PATCHES:
        if out.count(anchor) != 1:
            sys.exit('锚点命中 %d 次（应为 1）：%s' % (out.count(anchor), desc))
        out = out.replace(anchor, repl, 1)
        print('  [ok] %s' % desc)
    tmp = TARGET + '.tmp'
    io.open(tmp, 'w', encoding='utf-8').write(out)
    import py_compile
    try:
        py_compile.compile(tmp, doraise=True)
    except Exception as exc:                      # noqa: BLE001
        import os
        os.unlink(tmp)
        sys.exit('语法校验失败，未改动：%s' % exc)
    import os
    os.replace(tmp, TARGET)
    os.chmod(TARGET, 0o755)
    print('\n完成。备份 -> %s' % BAK)


if __name__ == '__main__':
    main()
