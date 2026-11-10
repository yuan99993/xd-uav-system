#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""限制/恢复 GeoFusion 页面的渲染帧率（在 Jetson 上运行，走 CDP）。

原理：包装 window.requestAnimationFrame，丢弃间隔不足的调用，把重绘节流到
指定 fps。**不修改任何源码**，刷新页面（Page.reload）即完全恢复。

用法:
    python3 limit_fps.py status            # 看当前 fps 与是否已限流
    python3 limit_fps.py limit 30          # 限到 30fps
    python3 limit_fps.py restore           # 刷新页面，恢复正常
"""
from __future__ import print_function

import json
import sys
import time

sys.path.insert(0, '/tmp')
from probe_page import CDP, get_pages, EXPR_FPS     # noqa: E402

EXPR_STATUS = """
({
  limited: !!window.__rafLimitFps,
  target: window.__rafLimitFps || null,
  canvases: document.querySelectorAll('canvas').length
})
"""


def limit_js(fps):
    # 每个动画帧回调都检查时间间隔；不足间隔的帧延后到下一帧继续等，
    # 保证回调不会被丢弃（只是被推迟），避免依赖 rAF 的动画停住。
    return """
(function(){
  var FPS = %d;
  var INTERVAL = 1000.0 / FPS;
  if (window.__rafLimitFps) { window.__rafLimitFps = FPS; return 'updated'; }
  window.__rafLimitFps = FPS;
  var orig = window.requestAnimationFrame.bind(window);
  var last = -1e9;
  window.requestAnimationFrame = function(cb){
    return orig(function(t){
      var iv = 1000.0 / window.__rafLimitFps;
      if (t - last < iv - 1.5) { window.requestAnimationFrame(cb); return; }
      last = t;
      cb(t);
    });
  };
  return 'patched';
})()
""" % fps


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else 'status'
    pages = get_pages(9334)
    page = next((p for p in pages if p.get('type') == 'page'), None)
    if not page:
        sys.exit('没找到 page')
    cdp = CDP(page['webSocketDebuggerUrl'])
    try:
        if action == 'status':
            print('状态: %s' % json.dumps(cdp.js(EXPR_STATUS), ensure_ascii=False))
            r = cdp.js(EXPR_FPS, await_promise=True)
            print('实测: %.1f fps' % r['fps'])

        elif action == 'limit':
            fps = int(sys.argv[2]) if len(sys.argv) > 2 else 30
            print('注入: %s' % cdp.js(limit_js(fps)))
            time.sleep(1.5)
            for i in range(2):
                r = cdp.js(EXPR_FPS, await_promise=True)
                print('  第 %d 次实测: %.1f fps' % (i + 1, r['fps']))
                time.sleep(1)

        elif action == 'restore':
            cdp.call('Page.enable')
            cdp.call('Page.reload', {'ignoreCache': False})
            print('已刷新页面，限流补丁随之清除')
            time.sleep(4)
            r = cdp.js(EXPR_FPS, await_promise=True)
            print('恢复后: %.1f fps' % r['fps'])
        else:
            print('未知动作: %s' % action)
    finally:
        cdp.close()


if __name__ == '__main__':
    main()
