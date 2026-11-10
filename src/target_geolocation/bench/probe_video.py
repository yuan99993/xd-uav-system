#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查 GeoFusion 页面里的视频：来源、分辨率、真实解码帧率、是否在丢帧。

回答「UE 端把帧率从 60 降到 24，解码端能不能省 CPU」的前提是：
  1. 页面里到底有没有 <video>，几路，多大
  2. 它真实解码多少帧/秒（getVideoPlaybackQuality 的增量）
  3. 有没有丢帧
  4. 视频是喂给 WebGL 纹理还是直接显示

只读探测，不改页面。
用法: python3 probe_video.py [--seconds 3]
"""
from __future__ import print_function

import json
import sys
import time

sys.path.insert(0, '/tmp')
from probe_page import CDP, get_pages     # noqa: E402

EXPR_LIST = """
(function(){
  var out=[];
  var vs=document.querySelectorAll('video');
  for (var i=0;i<vs.length;i++){
    var v=vs[i];
    var q = v.getVideoPlaybackQuality ? v.getVideoPlaybackQuality() : null;
    var r=v.getBoundingClientRect();
    out.push({
      i:i,
      w:v.videoWidth, h:v.videoHeight,
      cw:Math.round(r.width), ch:Math.round(r.height),
      paused:!!v.paused, muted:!!v.muted,
      hasStream: !!v.srcObject,
      streamType: v.srcObject ? (v.srcObject.constructor && v.srcObject.constructor.name) : null,
      src: (v.src || '').slice(0,60),
      total: q ? q.totalVideoFrames : null,
      dropped: q ? q.droppedVideoFrames : null,
      decoded: q ? q.totalVideoFrames : null,
      display: getComputedStyle(v).display,
      opacity: getComputedStyle(v).opacity
    });
  }
  return out;
})()
"""

# 页面若用 WebGL 把 video 当纹理，video 可能 display:none；
# 顺便看看 canvas 是不是在每帧 drawImage(video)
EXPR_CANVAS = """
(function(){
  var out=[];
  var cs=document.querySelectorAll('canvas');
  for(var i=0;i<cs.length;i++){
    var c=cs[i], r=c.getBoundingClientRect();
    out.push({i:i, bw:c.width, bh:c.height,
              cw:Math.round(r.width), ch:Math.round(r.height),
              style:c.getAttribute('style')||''});
  }
  return out;
})()
"""


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
        vids = cdp.js(EXPR_LIST) or []
        print('=== <video> 元素：%d 个 ===' % len(vids))
        for v in vids:
            print('  [%d] %sx%s  显示尺寸 %sx%s  paused=%s muted=%s'
                  % (v['i'], v['w'], v['h'], v['cw'], v['ch'],
                     v['paused'], v['muted']))
            print('      srcObject=%s (%s)  display=%s opacity=%s'
                  % (v['hasStream'], v['streamType'], v['display'], v['opacity']))
            if v['src']:
                print('      src=%s' % v['src'])

        cs = cdp.js(EXPR_CANVAS) or []
        print('\n=== <canvas>：%d 个 ===' % len(cs))
        for c in cs:
            print('  [%d] backing %dx%d  显示 %dx%d  style=%s'
                  % (c['i'], c['bw'], c['bh'], c['cw'], c['ch'], c['style'][:40]))

        if not vids:
            print('\n没有 <video> 元素。画面可能完全由 WebGL/Canvas 绘制。')
            return

        print('\n=== 真实解码帧率（采样 %.0f 秒）===' % seconds)
        a = {v['i']: (v['total'], v['dropped']) for v in vids}
        time.sleep(seconds)
        b = cdp.js(EXPR_LIST) or []
        for v in b:
            i = v['i']
            if i not in a or a[i][0] is None or v['total'] is None:
                continue
            dt = seconds
            nf = v['total'] - a[i][0]
            nd = (v['dropped'] or 0) - (a[i][1] or 0)
            print('  [%d] %.1f fps  丢帧 %d  (共 %s 帧已解码)'
                  % (i, nf / dt, nd, v['total']))
    finally:
        cdp.close()


if __name__ == '__main__':
    main()
