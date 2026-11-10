#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 CDP 的 CPU Profiler 抓页面热点函数（在 Jetson 上运行）。

回答「chrome 主线程 36% 到底在算什么」：采 3 秒 JS 调用栈，按自身耗时聚合，
输出 top 函数。同时给出 Performance 的间隔增量（Task/Script/Layout 各占多少），
判断是 JS 在算、还是在排版、还是在等渲染。

只读诊断，不改页面任何状态。
用法: python3 profile_page.py [--seconds 3]
"""
from __future__ import print_function

import json
import sys
import time

sys.path.insert(0, '/tmp')
from probe_page import CDP, get_pages     # noqa: E402


def metrics_delta(cdp, seconds):
    cdp.call('Performance.enable')
    a = {m['name']: m['value']
         for m in cdp.call('Performance.getMetrics')['metrics']}
    time.sleep(seconds)
    b = {m['name']: m['value']
         for m in cdp.call('Performance.getMetrics')['metrics']}
    d = {}
    for k in b:
        if k in a:
            d[k] = (b[k] - a[k]) / seconds     # 每秒占比
    return d


def main():
    seconds = 3.0
    if '--seconds' in sys.argv:
        seconds = float(sys.argv[sys.argv.index('--seconds') + 1])

    pages = get_pages(9334)
    page = next((p for p in pages if p.get('type') == 'page'), None)
    if not page:
        sys.exit('没找到 page')
    cdp = CDP(page['webSocketDebuggerUrl'])

    try:
        print('=== Performance 间隔增量（每秒占比，>1 表示多线程并行）===')
        d = metrics_delta(cdp, 2.0)
        for k in ('TaskDuration', 'ScriptDuration', 'LayoutDuration',
                  'RecalcStyleDuration', 'JSHeapUsedSize'):
            if k in d:
                if 'Heap' in k:
                    print('  %-20s %.1f MB' % (k, d[k] / 1e6))
                else:
                    print('  %-20s %5.1f%%' % (k, d[k] * 100.0))

        print('\n=== CPU Profile（采 %.0f 秒）===' % seconds)
        cdp.call('Profiler.enable')
        cdp.call('Profiler.setSamplingInterval', {'interval': 1000})
        cdp.call('Profiler.start')
        time.sleep(seconds)
        prof = cdp.call('Profiler.stop', timeout=40)['profile']

        nodes = {n['id']: n for n in prof['nodes']}
        samples = prof.get('samples') or []
        deltas = prof.get('timeDeltas') or []
        # 用 timeDeltas 加权：每次采样的时间间隔即该帧的耗时
        self_us = {}
        for i, sid in enumerate(samples):
            dt = deltas[i] if i < len(deltas) else 0
            if dt < 0 or dt > 1e6:            # 丢弃异常间隔
                dt = 1000
            self_us[sid] = self_us.get(sid, 0) + dt
        total = sum(self_us.values()) or 1

        rows = []
        for nid, us in self_us.items():
            n = nodes.get(nid, {})
            cf = n.get('callFrame', {})
            fn = cf.get('functionName') or '(anonymous)'
            url = (cf.get('url') or '').split('/')[-1][:26]
            rows.append((us, fn, url, cf.get('lineNumber', -1)))
        rows.sort(reverse=True)

        print('  总采样耗时 %.2f 秒，共 %d 个函数' % (total / 1e6, len(rows)))
        print('  %-34s %7s  %s' % ('函数', '占比', '文件:行'))
        print('  ' + '-' * 68)
        acc = 0.0
        for us, fn, url, ln in rows[:15]:
            pct = us / total * 100.0
            acc += pct
            print('  %-34s %6.1f%%  %s:%d' % (fn[:34], pct, url, ln))
        print('  ' + '-' * 68)
        print('  前 15 个函数合计 %.1f%%' % acc)
    finally:
        cdp.close()


if __name__ == '__main__':
    main()
