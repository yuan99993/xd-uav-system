#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 Chrome DevTools Protocol 探测页面真实渲染情况（在 Jetson 上运行）。

只做只读探测，不改页面状态：
  1. WebGL 后端是什么 —— 若为 SwiftShader / llvmpipe 说明是软件光栅化，
     那 63% 的 CPU 是 CPU 在画图，换硬件才治本
  2. requestAnimationFrame 的真实帧率 —— 页面到底按多少 fps 重绘
  3. 视口尺寸 / devicePixelRatio —— 判断是否在超采样

零依赖：手写 RFC6455 客户端，只用标准库。
用法: python3 probe_page.py [--port 9334]
"""
from __future__ import print_function

import base64
import json
import os
import socket
import struct
import sys
import time

HOST = '127.0.0.1'


# ---------------------------------------------------------------- WebSocket
def ws_connect(host, port, path, timeout=10):
    s = socket.create_connection((host, port), timeout=timeout)
    key = base64.b64encode(os.urandom(16)).decode()
    req = ('GET %s HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\n'
           'Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n'
           'Sec-WebSocket-Version: 13\r\n\r\n' % (path, host, port, key))
    s.sendall(req.encode())
    buf = b''
    while b'\r\n\r\n' not in buf:
        chunk = s.recv(4096)
        if not chunk:
            raise RuntimeError('握手时连接关闭')
        buf += chunk
    head, _, rest = buf.partition(b'\r\n\r\n')
    if b'101' not in head.split(b'\r\n')[0]:
        raise RuntimeError('握手失败: %r' % head[:200])
    return s, rest


def ws_send(s, text):
    data = text.encode()
    mask = os.urandom(4)
    hdr = bytearray([0x81])
    n = len(data)
    if n < 126:
        hdr.append(0x80 | n)
    elif n < 65536:
        hdr.append(0x80 | 126)
        hdr += struct.pack('>H', n)
    else:
        hdr.append(0x80 | 127)
        hdr += struct.pack('>Q', n)
    hdr += mask
    s.sendall(bytes(hdr) + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))


def ws_recv(s, buf=b''):
    """读一条完整消息（处理分片与 ping）。"""
    payload = b''
    while True:
        while len(buf) < 2:
            buf += s.recv(65536)
        b0, b1 = buf[0], buf[1]
        opcode = b0 & 0x0F
        fin = b0 & 0x80
        ln = b1 & 0x7F
        off = 2
        if ln == 126:
            while len(buf) < 4:
                buf += s.recv(65536)
            ln = struct.unpack('>H', buf[2:4])[0]
            off = 4
        elif ln == 127:
            while len(buf) < 10:
                buf += s.recv(65536)
            ln = struct.unpack('>Q', buf[2:10])[0]
            off = 10
        while len(buf) < off + ln:
            buf += s.recv(65536)
        payload += buf[off:off + ln]
        buf = buf[off + ln:]
        if opcode == 0x9:                       # ping -> pong
            s.sendall(bytes([0x8A, 0x80]) + os.urandom(4))
            continue
        if fin:
            return payload.decode('utf-8', 'replace'), buf


class CDP(object):
    def __init__(self, wsurl):
        assert wsurl.startswith('ws://')
        rest = wsurl[5:]
        hostport, _, path = rest.partition('/')
        host, _, port = hostport.partition(':')
        self.s, self.buf = ws_connect(host, int(port or 80), '/' + path)
        self.n = 0

    def call(self, method, params=None, timeout=15):
        self.n += 1
        mid = self.n
        msg = {'id': mid, 'method': method}
        if params:
            msg['params'] = params
        ws_send(self.s, json.dumps(msg))
        deadline = time.time() + timeout
        while time.time() < deadline:
            txt, self.buf = ws_recv(self.s, self.buf)
            try:
                obj = json.loads(txt)
            except ValueError:
                continue
            if obj.get('id') == mid:
                if 'error' in obj:
                    raise RuntimeError(obj['error'])
                return obj.get('result', {})
        raise RuntimeError('CDP 超时: %s' % method)

    def js(self, expr, await_promise=False, timeout=20):
        r = self.call('Runtime.evaluate', {
            'expression': expr,
            'returnByValue': True,
            'awaitPromise': await_promise,
        }, timeout=timeout)
        res = r.get('result', {})
        if res.get('subtype') == 'error':
            raise RuntimeError(res.get('description', 'js error'))
        return res.get('value')

    def close(self):
        try:
            self.s.close()
        except Exception:                        # noqa: BLE001
            pass


EXPR_GL = """
(function(){
  try{
    var c=document.createElement('canvas');
    var g=c.getContext('webgl2')||c.getContext('webgl');
    if(!g) return 'no-webgl';
    var d=g.getExtension('WEBGL_debug_renderer_info');
    var r=d?g.getParameter(d.UNMASKED_RENDERER_WEBGL):g.getParameter(g.RENDERER);
    var v=d?g.getParameter(d.UNMASKED_VENDOR_WEBGL):'';
    return v+' | '+r;
  }catch(e){return 'err:'+e.message;}
})()
"""

EXPR_FPS = """
new Promise(function(res){
  var n=0, t0=performance.now();
  function f(){
    n++;
    var dt=performance.now()-t0;
    if(dt<1500) requestAnimationFrame(f); else res({fps:n*1000/dt, frames:n, ms:Math.round(dt)});
  }
  requestAnimationFrame(f);
})
"""

EXPR_VIEW = """
({w:innerWidth, h:innerHeight, dpr:devicePixelRatio,
  vis:document.visibilityState, title:document.title,
  canvases:document.querySelectorAll('canvas').length})
"""


def get_pages(port):
    import urllib.request
    url = 'http://%s:%d/json/list' % (HOST, port)
    with urllib.request.urlopen(url, timeout=6) as r:
        return json.loads(r.read().decode('utf-8', 'replace'))


def main():
    port = 9334
    if '--port' in sys.argv:
        port = int(sys.argv[sys.argv.index('--port') + 1])

    pages = get_pages(port)
    page = next((p for p in pages if p.get('type') == 'page'), None)
    if not page:
        sys.exit('没找到 page 目标')
    print('页面: %s  (%s)' % (page.get('title'), page.get('url')))

    cdp = CDP(page['webSocketDebuggerUrl'])
    try:
        print('\n[WebGL 后端]')
        print('  %s' % cdp.js(EXPR_GL))

        print('\n[视口]')
        print('  %s' % json.dumps(cdp.js(EXPR_VIEW), ensure_ascii=False))

        print('\n[实际渲染帧率] (测 1.5 秒)')
        r = cdp.js(EXPR_FPS, await_promise=True)
        print('  %.1f fps  (%d 帧 / %.0f ms)' % (r['fps'], r['frames'], r['ms']))

        print('\n[Performance 指标]')
        try:
            cdp.call('Performance.enable')
            m = cdp.call('Performance.getMetrics').get('metrics', [])
            want = ('FramesPerSecond', 'TaskDuration', 'ScriptDuration',
                    'LayoutDuration', 'RecalcStyleDuration', 'JSHeapUsedSize')
            d = {x['name']: x['value'] for x in m if x['name'] in want}
            for k in want:
                if k in d:
                    v = d[k]
                    print('  %-22s %.3f' % (k, v))
        except Exception as exc:                 # noqa: BLE001
            print('  (不可用: %s)' % exc)
    finally:
        cdp.close()


if __name__ == '__main__':
    main()
