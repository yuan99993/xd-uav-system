#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UE 像素流原生播放器（Jetson Orin）—— 全硬件解码 + 原生窗口叠加 + 键鼠回传

设计要点（与既有 Web 方案并存，不修改原脚本）：
  * 解码显示链路全部在 NVIDIA 硬件里完成：
        webrtcbin -> rtpjitterbuffer -> rtph264depay -> h264parse
                  -> nvv4l2decoder(NVDEC) -> nvvidconv -> nveglglessink
    系统未安装任何软件 H264 解码器，因此不存在软解回落路径。
  * 视频直接渲染进一个 override-redirect 的 X 窗口（默认 300,48,1620,911），
    窗口始终 mapped，显示/隐藏用 stack Above/Below 控制（藏到全屏 Chromium 后面），
    这样 EGL surface 始终有效、也不会遮挡页面顶栏/侧栏/底部 tabbar。
  * 键鼠按 UE 5.2 PixelStreaming 二进制协议经 DataChannel 回传：
        MouseEnter 70 / MouseLeave 71
        MouseDown 72 {u8 button, u16 x, u16 y}
        MouseUp   73 {u8 button, u16 x, u16 y}
        MouseMove 74 {u16 x, u16 y, u16 dx, u16 dy}   (dx/dy 为 int16 补码)
        MouseWheel75 {i16 delta, u16 x, u16 y}       (±120/格)
        KeyDown   60 {u8 keyCode, u8 isRepeat}
        KeyUp     61 {u8 keyCode}
        KeyPress  62 {u16 char}
    x/y 是 0..65535 归一化坐标；keyCode 为浏览器 KeyboardEvent.keyCode。
  * DataChannel 打开后立刻发 RequestQualityControl(1)（UE 只给控制者发帧），
    再发 IFrameRequest(0) 让 UE 立刻出一个关键帧。

用法：
    python3 ps_native_player.py --ue-host 192.168.5.4 --ue-port 80
控制（页面调用）：
    POST /show /hide            显示 / 隐藏（stack Above/Below，不销毁 EGL surface）
                                ⚠️ 断流时（stalled）即使 /show 也不抬起来：
                                没有画面就藏到页面后面，让页面画「等待连接」
    POST /shape  {rects:[[x,y,w,h]...]}   从窗口形状里挖洞（DOM 弹层透出）
    POST /geometry {x,y,w,h}    改变窗口屏幕矩形（全屏「视频铺满」/还原）
    POST /reconnect             重启播放器进程（彻底重建 ICE/DTLS/解码链）
    GET  /status                运行态 + 实时统计（fps/码率/分辨率/帧数/几何）
"""

import argparse
import asyncio
import ctypes
import json
import os
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstSdp', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GstSdp, GstWebRTC, GstVideo, GLib  # noqa: E402

import websockets  # noqa: E402
from Xlib import X, display, Xcursorfont  # noqa: E402
from Xlib.ext import shape as xshape  # noqa: E402

# 连续多少个统计周期（1s/次）收不到帧就判定为「断流」
# 停仿真后 UE 停止发帧，但 sink 会把最后一帧留在窗口里，必须主动清掉。
STALL_TICKS = 2

# ---- 画面是否铺满窗口 ------------------------------------------------------
# True  = sink 不保持宽高比，把画面**拉伸铺满整个窗口**（窗口里没有任何黑边）。
# False = 保持 16:9 letterbox，窗口比 16:9 宽时左右留黑带。
#
# 为什么默认 True：页面那边（ueNative.ts 的 winRect）已经把原生窗口改成**等于**
# .ue-view-box 的整块矩形（1919x1031，宽高比 1.86），此时若 sink 仍保持 16:9，
# 它自己会在窗口里再 letterbox 一次 —— 左右各 43px 的黑带，正是用户要求去掉的。
# 关掉之后画面横向拉伸约 4.7%（1920x1080 -> 1919x1031），比两条黑带可接受。
#
# ⚠️ 这个开关和 _norm() 是**绑定**的：铺满时窗口坐标就是画面坐标的线性缩放，
#    保持宽高比时才需要减掉黑边偏移。改这里必须同时看 _norm()。
FILL_WINDOW = True

# ---- 协商看门狗 ------------------------------------------------------------
# webrtcbin 1.16 的旧 transport 一旦 COMM_LOST 就再也收不回来：UE 会发新 offer、
# 我们也能回 answer，但一帧都收不到（channel:false / fps:0），画面永久黑住。
# 重建解码链不够，只有重启进程能救（见 reconnect() 的说明）。
# 以前只能靠人在页面上点「重新连接」，现在自己判：answer 发出去 NEG_FAIL_SEC 秒
# 还没收到任何一帧算一次失败，连续 NEG_FAIL_MAX 次就 execv 重启。
NEG_FAIL_SEC = 8
NEG_FAIL_MAX = 2

# ---- 死链路看门狗 ----------------------------------------------------------
# 协商看门狗只在「answer 发出去之后」布防。还有一种更隐蔽的死法：**压根没有新
# offer**——UE 重启了，但 SS 进程没重启，播放器的 WebSocket 一直活着，于是
# _ws_forever 的 except 分支（唯一会重建 webrtcbin 的地方）永远不触发，播放器
# 就永久停在 connected=true / channel=false / frames=0 / stalled=true 上，页面
# 一片黑，而且没有任何自愈（2026-10-24 19:33 实测）。
#
# 判据只用 **datachannel 已关闭**（chan_open=False）：
#   * UE 进程没了 -> PeerConnection 跟着消失 -> SCTP 断开 -> chan_open=False
#   * 只是「停仿真」-> UE 不发帧但连接还在 -> chan_open=True，**不能重启**
# 再加上「SS 当前报有 streamer」（last_ids 非空）—— UE 根本没起来时不做无谓重启，
# 否则 UE 关着的那段时间会变成每 10 秒重启一次的循环。
DEAD_RESTART_SEC = 10

# ---------------------------------------------------------------- 协议常量
MSG_IFRAME_REQUEST = 0
MSG_REQUEST_QUALITY_CONTROL = 1
MSG_KEY_DOWN = 60
MSG_KEY_UP = 61
MSG_KEY_PRESS = 62
MSG_MOUSE_ENTER = 70
MSG_MOUSE_LEAVE = 71
MSG_MOUSE_DOWN = 72
MSG_MOUSE_UP = 73
MSG_MOUSE_MOVE = 74
MSG_MOUSE_WHEEL = 75

# ---------------------------------------------------------------- 键位映射
# X11 keysym -> 浏览器 KeyboardEvent.keyCode
JS_KEY = {
    0x08: 8, 0x09: 9, 0x0D: 13, 0x1B: 27, 0x20: 32,
    0xFF08: 8, 0xFF09: 9, 0xFF0D: 13, 0xFF1B: 27, 0xFF8D: 13,
    0xFFE1: 16, 0xFFE2: 253,   # Shift_L / Shift_R
    0xFFE3: 17, 0xFFE4: 254,   # Control_L / Control_R
    0xFFE9: 18, 0xFFEA: 255,   # Alt_L / Alt_R
    0xFFE5: 20,                # Caps_Lock
    0xFF7F: 144, 0xFF14: 145, 0xFF13: 19, 0xFF67: 93,
    0xFF50: 36, 0xFF57: 35, 0xFF55: 33, 0xFF56: 34,
    0xFF63: 45, 0xFFFF: 46,
    0xFF51: 37, 0xFF52: 38, 0xFF53: 39, 0xFF54: 40,
    0xFF61: 44,                # Print
    # 标点（浏览器 keyCode）
    0x2D: 189, 0x3D: 187, 0x5B: 219, 0x5D: 221, 0x3B: 186, 0x27: 222,
    0x60: 192, 0x5C: 220, 0x2C: 188, 0x2E: 190, 0x2F: 191,
}
for _i in range(12):
    JS_KEY[0xFFBE + _i] = 112 + _i          # F1..F12
for _i in range(10):
    JS_KEY[0xFFB0 + _i] = 96 + _i           # 小键盘 0..9
JS_KEY.update({0xFFAA: 106, 0xFFAB: 107, 0xFFAD: 109, 0xFFAE: 110, 0xFFAF: 111})

# 需要 Shift 的符号：keysym -> (无 Shift, Shift)
PUNCT_CHARS = {
    0x2D: ('-', '_'), 0x3D: ('=', '+'), 0x5B: ('[', '{'), 0x5D: (']', '}'),
    0x5C: ('\\', '|'), 0x3B: (';', ':'), 0x27: ("'", '"'), 0x60: ('`', '~'),
    0x2C: (',', '<'), 0x2E: ('.', '>'), 0x2F: ('/', '?'),
}
SHIFT_DIGITS = {0x30: ')', 0x31: '!', 0x32: '@', 0x33: '#', 0x34: '$',
                0x35: '%', 0x36: '^', 0x37: '&', 0x38: '*', 0x39: '('}


def js_keycode(ks):
    if ks in JS_KEY:
        return JS_KEY[ks]
    if 0x61 <= ks <= 0x7A:
        return ks - 32
    if 0x41 <= ks <= 0x5A:
        return ks
    if 0x30 <= ks <= 0x39:
        return ks
    if 0x20 <= ks <= 0x7E:
        return ks
    return None


def js_char(ks, shift, caps):
    if 0x61 <= ks <= 0x7A:
        return chr(ks - 32) if (shift ^ caps) else chr(ks)
    if 0x30 <= ks <= 0x39:
        return SHIFT_DIGITS.get(ks, chr(ks)) if shift else chr(ks)
    if ks == 0x20:
        return ' '
    if ks in PUNCT_CHARS:
        return PUNCT_CHARS[ks][1 if shift else 0]
    if ks in (0xFF0D, 0xFF8D):
        return '\r'
    return None


class NativePlayer(object):
    def __init__(self, args):
        self.args = args
        self.chan = None
        self.chan_open = False
        self.win_visible = False
        self.focus_owned = False
        self.prev_focus = None
        self.last_x = None
        self.last_y = None
        self.shift = False
        self.caps = False
        self.pressed = set()
        self.pending_ice = []
        self.remote_set = False
        self.connected = False
        self.chain_linked = False
        self._promises = []          # 防止 GstPromise 被回收
        self._chain_elems = []       # 解码链元素，重建时逐个 remove
        self._rebuilding = False

        # X Shape 洞穿：页面把「需要显示在视频之上」的 DOM 矩形报过来，
        # 这里从窗口形状里把它们减掉，于是 DOM 弹层能透过窗口显示，
        # 而视频其余部分照常渲染 —— 不再需要整窗隐藏（那样会黑一下）。
        self.pending_holes = []
        self.holes_dirty = False
        self._shape_key = None
        self._hole_count = -1
        self.shape_applies = 0     # 真正下发过 X Shape 的次数（诊断用）
        self.shape_reqs = 0        # 页面 /shape 调用次数（诊断用）
        self.opt_reqs = 0          # CORS 预检次数（诊断用）

        # 窗口几何可被页面改写（全屏「视频铺满」用）：args.x/y/w/h 始终是当前值，
        # 鼠标归一化、洞形裁剪都以它为准，所以 resize 时直接就地更新。
        self.pending_geom = None
        self.geom_dirty = False

        # 实时统计（页面「信息 / 设置」面板用）：探针只累加计数，1s 汇总一次
        self.started_at = time.time()
        self.stat = {'fps': 0.0, 'kbps': 0.0, 'width': 0, 'height': 0,
                     'frames': 0, 'codec': 'H264'}
        self._st_frames = 0
        self._st_bytes = 0
        self._st_t0 = time.time()
        self._stats_started = False

        # 可见性由「页面想不想看」+「有没有画面」共同决定：
        #   want_visible 由页面 /show /hide 设置；
        #   stalled      连续 STALL_TICKS 秒没收到帧即为 True（停仿真 / UE 断开）。
        # 初始 stalled=True：第一帧到来之前不把窗口抬起来（否则是一块黑挡在上面）。
        self.want_visible = False
        self.vis_dirty = False
        self.force_stack = False
        self.stalled = True
        self._ever_frames = False
        self._idle_ticks = 0
        # 连续多少次 listStreamers 返回空列表。_poll_streamers 据此退避：
        # 仿真没开时没必要 2s 一次空问，但也不能退太久 —— UE 起来后要靠这条
        # 轮询发现新 streamer，退避过长就变成「仿真起来了画面还黑十几秒」。
        self._empty_lists = 0
        # 协商看门狗：上一次 answer 的时刻 / 那一刻的累计帧数 / 连续失败次数
        self._answer_at = 0.0
        self._frames_at_answer = 0
        self._neg_fail = 0
        # 死链路看门狗：channel 关闭且无帧的起始时刻（0 = 未进入该状态）
        self._dead_since = 0.0

        self._init_gst()
        self._init_x()

    # ------------------------------------------------------------ GStreamer
    def _init_gst(self):
        Gst.init(None)
        self.pipe = Gst.Pipeline.new('ps-native')
        self.webrtc = Gst.ElementFactory.make('webrtcbin', 'recv')
        if self.webrtc is None:
            raise RuntimeError('webrtcbin 不可用（缺少 gstreamer plugins bad / nice）')
        if self.args.stun:
            self.webrtc.set_property('stun-server', self.args.stun)
        # webrtcbin 1.16 默认 bundle-policy=none，answer 里不会生成 a=group:BUNDLE，
        # UE 侧 application m-line 归不到同一 transport -> DataChannel 永远不开。
        self.webrtc.set_property('bundle-policy',
                                 GstWebRTC.WebRTCBundlePolicy.MAX_BUNDLE)
        self.pipe.add(self.webrtc)
        self.webrtc.connect('on-ice-candidate', self._on_local_ice)
        self.webrtc.connect('on-data-channel', self._on_data_channel)
        self.webrtc.connect('pad-added', self._on_pad_added)

        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self._on_bus_msg)

    def _on_bus_msg(self, bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print('[ps] GST ERROR:', err.message, '|', dbg, flush=True)
        elif t == Gst.MessageType.EOS:
            print('[ps] EOS', flush=True)
        elif t == Gst.MessageType.WARNING:
            err, dbg = msg.parse_warning()
            print('[ps] GST WARN:', err.message, '|', dbg, flush=True)

    def _mk(self, factory, name=None):
        e = Gst.ElementFactory.make(factory, name)
        if e is None:
            raise RuntimeError('缺少 GStreamer 元素: %s' % factory)
        return e

    def _on_pad_added(self, webrtc, pad):
        caps = pad.get_current_caps() or pad.query_caps(None)
        if caps is None or caps.get_size() == 0:
            return
        s = caps.get_structure(0)
        media = s.get_value('media') if s.has_field('media') else None
        enc = s.get_value('encoding-name') if s.has_field('encoding-name') else None
        print('[ps] pad-added media=%s enc=%s' % (media, enc), flush=True)
        if media != 'video' or (enc or '').upper() != 'H264':
            return
        if self.chain_linked:
            return
        self._build_chain(pad)

    def _build_chain(self, pad):
        """收到 H264 视频 pad 后才建解码链，避免未连接时 pipeline 处于 ASYNC。"""
        self.jitter = self._mk('rtpjitterbuffer', 'jitter')
        self.jitter.set_property('latency', 50)
        self.depay = self._mk('rtph264depay')
        self.parse = self._mk('h264parse')
        self.dec = self._mk('nvv4l2decoder')
        self.conv = self._mk('nvvidconv')
        self.sink = self._mk('nveglglessink', 'videosink')
        self.sink.set_property('sync', False)
        # 见文件顶部 FILL_WINDOW：铺满窗口，窗口里不留黑边
        self.sink.set_property('force-aspect-ratio', not FILL_WINDOW)

        # nveglglessink 报 "cannot handle Legacy NVMM Buffers"，nvvidconv 输出
        # 必须落到系统内存（否则 caps 会协商成 memory:NVMM 直接透传给 sink）。
        self.capsf = self._mk('capsfilter')
        self.capsf.set_property('caps',
                                Gst.Caps.from_string('video/x-raw,format=RGBA'))
        chain = [self.jitter, self.depay, self.parse, self.dec, self.conv,
                 self.capsf, self.sink]
        self._chain_elems = list(chain)
        for e in chain:
            self.pipe.add(e)
        for a, b in zip(chain, chain[1:]):
            if not a.link(b):
                raise RuntimeError('link 失败: %s -> %s' % (a.get_name(), b.get_name()))

        ret = pad.link(self.jitter.get_static_pad('sink'))
        print('[ps] pad.link -> %s' % ret, flush=True)
        for e in chain:
            e.sync_state_with_parent()

        self._set_window_handle(self.sink, self.win.id)
        self._attach_stats_probes()
        self.chain_linked = True
        print('[ps] 解码链已建立：NVDEC + nveglglessink（硬件解码）', flush=True)

    def _rebuild_pipeline(self):
        """整链重建：停仿真 → 再启动仿真 的恢复路径。

        实测现象：UE 侧重建 session 后会重新发 offer，我们也能正常回 answer，
        但 webrtcbin 1.16 复用第一次 session 的 pad / transport —— pad-added
        不会再触发（或触发时 caps 还是空的），解码链永远收不到 buffer，
        表现为 frames 卡死、fps 0、画面永久黑住。浏览器每次都新建
        RTCPeerConnection 所以没这个问题。

        所以这里在「信令重新连上 / streamer 断开」这两个时间点（都在新 offer
        之前）把 webrtcbin + 解码链整体拆掉重建，让新 offer 落在一个全新的
        webrtcbin 上，行为与首次启动完全一致。

        X 窗口不重建：几何、Shape 洞穿、页面侧状态全部不受影响。
        """
        if self._rebuilding:
            return False
        self._rebuilding = True
        try:
            self.chain_linked = False
            old = self.pipe
            if old is not None:
                old.set_state(Gst.State.NULL)
                # 等真的落到 NULL 再 remove，否则 remove 会失败；1s 兜底
                old.get_state(Gst.SECOND)
                try:
                    old.get_bus().remove_signal_watch()
                except Exception:
                    pass
                for e in self._chain_elems:
                    try:
                        old.remove(e)
                    except Exception:
                        pass
                try:
                    old.remove(self.webrtc)
                except Exception:
                    pass
            self._chain_elems = []
            self.jitter = self.depay = self.parse = None
            self.dec = self.conv = self.capsf = self.sink = None
            self.webrtc = None
            self.pipe = None

            # 新 session 的中间状态一起清掉
            self.remote_set = False
            self.pending_ice = []
            self.chan = None
            self.chan_open = False
            self._idle_ticks = 0
            self._st_frames = 0
            self._st_bytes = 0
            self._st_t0 = time.time()
            # stat['frames'] 下面会被清零，看门狗的基准帧数必须跟着清，
            # 否则会拿「0 != 旧基准」误判成「已经收到帧」
            self._answer_at = 0.0
            self._frames_at_answer = 0
            self._neg_fail = 0
            # 死链路看门狗的计时也从头算：刚重建完，得给新 offer 留出时间，
            # 不能拿「重建前已经死了多久」直接判死。
            self._dead_since = 0.0
            self.stat.update({'fps': 0.0, 'kbps': 0, 'width': 0,
                              'height': 0, 'frames': 0})

            self._init_gst()
            self.pipe.set_state(Gst.State.PLAYING)
            # 重建 = 这条 session 已经死了，新帧到来之前什么都放不出来。
            # 必须立刻清黑 + 收起窗口，否则旧 sink 留在窗口上的最后一帧会
            # 一直冻在屏幕上（看起来像「停止仿真后画面还在」）。
            self.stalled = True
            self._clear_window()
            self.vis_dirty = True
            print('[ps] 解码链已重建，等待新 offer', flush=True)
        except Exception as ex:
            print('[ps] 解码链重建失败:', repr(ex), flush=True)
        finally:
            self._rebuilding = False
        return False

    async def _rebuild_async(self, why):
        """在 GLib 线程里跑 _rebuild_pipeline，并等它做完再继续（避免新 offer
        落在半拆掉的 webrtcbin 上）。"""
        if not (self.chain_linked or self._chain_elems):
            return
        print('[ps] %s：重建解码链（新 session 需要全新 webrtcbin）' % why, flush=True)
        ev = threading.Event()

        def _do():
            try:
                self._rebuild_pipeline()
            finally:
                ev.set()
            return False

        GLib.idle_add(_do)
        try:
            await self.loop.run_in_executor(None, ev.wait, 6.0)
        except Exception:
            pass

    # ---------------------------------------------------------------- 实时统计
    def _attach_stats_probes(self):
        """在 depay 出口（压缩字节）和解码器出口（帧）各挂一个探针。

        探针跑在 GStreamer 的流线程里，只做整数累加（GIL 下足够安全），
        真正的换算交给 GLib 主循环里 1s 一次的 _tick_stats。
        """
        try:
            self.depay.get_static_pad('src').add_probe(
                Gst.PadProbeType.BUFFER, self._probe_bytes)
            self.sink.get_static_pad('sink').add_probe(
                Gst.PadProbeType.BUFFER, self._probe_frame)
            print('[ps] 统计探针已挂载', flush=True)
        except Exception as ex:
            print('[ps] 统计探针挂载失败:', repr(ex), flush=True)
        if not self._stats_started:
            self._stats_started = True
            GLib.timeout_add(1000, self._tick_stats)

    def _probe_bytes(self, pad, info):
        buf = info.get_buffer()
        if buf is not None:
            self._st_bytes += buf.get_size()
        return Gst.PadProbeReturn.OK

    def _probe_frame(self, pad, info):
        self._st_frames += 1
        caps = pad.get_current_caps()
        if caps is not None and caps.get_size() > 0:
            s = caps.get_structure(0)
            try:
                self.stat['width'] = int(s.get_value('width'))
                self.stat['height'] = int(s.get_value('height'))
            except Exception:
                pass
        return Gst.PadProbeReturn.OK

    def _tick_stats(self):
        now = time.time()
        dt = now - self._st_t0
        if dt <= 0.05:
            return True
        frames = self._st_frames
        self.stat['fps'] = round(frames / dt, 1)
        # 只算 H264 裸流字节（不含 RTP/UDP 头），与播放器页的 Video Bitrate 同量级
        self.stat['kbps'] = int(round(self._st_bytes * 8.0 / 1000.0 / dt))
        self.stat['frames'] += frames
        self._st_frames = 0
        self._st_bytes = 0
        self._st_t0 = now

        # ---- 断流检测 ----------------------------------------------------
        # 停仿真后 UE 不再发帧，但 nveglglessink 会把最后一帧一直留在窗口里，
        # 看起来像「视频还在」。所以只要连续 2 秒收不到帧，就把窗口清黑并藏到
        # 页面后面，让页面显示「等待连接」。
        if frames > 0:
            self._ever_frames = True
            self._idle_ticks = 0
            if self.stalled:
                self.stalled = False
                print('[ps] 收到新帧，恢复显示', flush=True)
                self._refresh_visible()
        elif self._ever_frames and self.chain_linked:
            self._idle_ticks += 1
            if self._idle_ticks >= STALL_TICKS and not self.stalled:
                self.stalled = True
                print('[ps] 连续 %d 秒无帧 -> 清黑并隐藏窗口' % self._idle_ticks, flush=True)
                self._clear_window()
                self._refresh_visible()

        # ---- 协商看门狗 ----------------------------------------------------
        # 只在「answer 已经发出去」之后才判 —— 没 offer 的时候（仿真没启动、
        # 或者 UE 刚重启还没上线）本来就不该有帧，不能当成协商失败。
        if self._answer_at:
            if self.stat['frames'] != self._frames_at_answer:
                # 收到帧了：这次协商是好的，计数清零
                self._neg_fail = 0
                self._answer_at = 0.0
            elif now - self._answer_at >= NEG_FAIL_SEC:
                self._neg_fail += 1
                print('[ps] answer 后 %d 秒仍无帧（连续第 %d 次）'
                      % (NEG_FAIL_SEC, self._neg_fail), flush=True)
                if self._neg_fail >= NEG_FAIL_MAX:
                    print('[ps] 协商反复失败 -> 重启播放器进程', flush=True)
                    sys.stdout.flush()
                    sys.stderr.flush()
                    try:
                        os.execv(sys.executable,
                                 [sys.executable, '-u', sys.argv[0]] + sys.argv[1:])
                    except Exception as ex:
                        print('[ps] execv 失败:', repr(ex), flush=True)
                        self._neg_fail = 0
                # 重新计时，否则每秒都会再判一次
                self._answer_at = now

        # ---- 死链路看门狗 --------------------------------------------------
        # 见文件顶部 DEAD_RESTART_SEC 的说明。这里覆盖的是「协商看门狗管不到的
        # 那一种」：没有新 offer、没有协商失败、但连接其实已经死了。
        # 三个条件缺一不可：
        #   stalled      —— 已经连续 STALL_TICKS 秒没有帧
        #   not chan_open —— datachannel 没了（停仿真时它是 True，不能重启）
        #   last_ids     —— SS 现在确实报着一个 streamer（UE 没关着）
        if (self.stalled and not self.chan_open and self._ever_frames
                and self.last_ids and not self._rebuilding):
            if not self._dead_since:
                self._dead_since = now
                print('[ps] channel 已关闭且无帧，%d 秒后仍无恢复就重启'
                      % DEAD_RESTART_SEC, flush=True)
            elif now - self._dead_since >= DEAD_RESTART_SEC:
                print('[ps] 链路已死 %d 秒 -> 重启播放器进程'
                      % DEAD_RESTART_SEC, flush=True)
                sys.stdout.flush()
                sys.stderr.flush()
                try:
                    os.execv(sys.executable,
                             [sys.executable, '-u', sys.argv[0]] + sys.argv[1:])
                except Exception as ex:
                    print('[ps] execv 失败:', repr(ex), flush=True)
                    self._dead_since = 0.0
        else:
            self._dead_since = 0.0
        return True

    def stats_snapshot(self):
        d = dict(self.stat)
        d['uptime'] = int(time.time() - self.started_at)
        d['geom'] = [int(self.args.x), int(self.args.y),
                     int(self.args.w), int(self.args.h)]
        d['stalled'] = bool(self.stalled)
        d['want'] = bool(self.want_visible)
        return d

    @staticmethod
    def _set_window_handle(sink, xid):
        try:
            libgstvideo = ctypes.CDLL('libgstvideo-1.0.so.0')
            getptr = ctypes.pythonapi.PyCapsule_GetPointer
            getptr.restype = ctypes.c_void_p
            getptr.argtypes = [ctypes.py_object, ctypes.c_char_p]
            elem = getptr(sink.__gpointer__, None)
            libgstvideo.gst_video_overlay_set_window_handle(
                ctypes.c_void_p(elem), ctypes.c_void_p(xid))
            print('[ps] overlay window handle -> 0x%x' % xid, flush=True)
        except Exception as ex:
            print('[ps] set window handle 失败:', repr(ex), flush=True)

    def _on_local_ice(self, webrtc, mline, cand):
        self._send_json({'type': 'iceCandidate',
                         'candidate': {'sdpMid': str(mline),
                                       'sdpMlineIndex': int(mline),
                                       'candidate': cand}})

    def _on_data_channel(self, webrtc, chan):
        label = chan.get_property('label')
        print('[ps] data-channel label=%r' % label, flush=True)
        self.chan = chan
        chan.connect('on-open', self._on_chan_open)
        chan.connect('on-close', self._on_chan_close)
        try:
            chan.connect('on-message', self._on_chan_msg)
        except TypeError:
            # GStreamer 1.16 的 GstWebRTCDataChannel 没有 on-message 信号，
            # 只影响 UE->播放器消息的日志，不影响键鼠回传。
            print('[ps] 本版本无 on-message 信号，跳过', flush=True)

    def _on_chan_open(self, chan):
        self.chan_open = True
        print('[ps] data-channel OPEN -> RequestQualityControl + IFrameRequest', flush=True)
        self.send_bytes(bytes([MSG_REQUEST_QUALITY_CONTROL]))
        self.send_bytes(bytes([MSG_IFRAME_REQUEST]))

    def _on_chan_close(self, chan):
        self.chan_open = False
        print('[ps] data-channel CLOSE', flush=True)

    def _on_chan_msg(self, chan, data):
        try:
            raw = bytes(data.get_data())
        except Exception:
            raw = b''
        # UE -> 播放器：TestEcho 等，仅记录
        print('[ps] chan msg %d bytes: %r' % (len(raw), raw[:32]), flush=True)

    def send_bytes(self, data):
        if not self.chan or not self.chan_open:
            return False
        try:
            self.chan.emit('send-data', GLib.Bytes.new(data))
            return True
        except Exception as ex:
            print('[ps] send-data 失败:', repr(ex), flush=True)
            return False

    # ------------------------------------------------------------ X11 窗口/输入
    def _make_cursor(self, glyph):
        """用 X 内置的 'cursor' 字体造一个字形光标（left_ptr = 标准箭头）。

        ⚠️ python-xlib 没有 Display.create_glyph_cursor —— 这个方法挂在 Font
           对象上（Xlib/xobject/fontable.py:90），签名是：
               Font.create_glyph_cursor(mask, source_char, mask_char,
                                       foreground, background)
           其中 mask 传的是「掩码字体的 id」（不是 Font 对象），foreground /
           background 各是一个 (r, g, b) 三元组、分量为 16 位（0..65535）。
           掩码字体直接复用同一个 font，掩码字符码用 glyph+1（字形后紧跟它的
           1-bit 掩码）。白色字形 + 黑边。
        """
        f = self.disp.open_font('cursor')
        return f.create_glyph_cursor(
            f.id, glyph, glyph + 1,
            (65535, 65535, 65535),
            (0, 0, 0))

    def _init_x(self):
        self.disp = display.Display()
        self.scr = self.disp.screen()
        a = self.args

        # ---- 光标：为什么这里必须显式设 ----
        # X 协议规定：窗口的 cursor 属性为 None 时「动态继承」父窗口的 cursor，
        # 一直上溯到 root；root 的 cursor 也是 None 的话，服务器就回落到内置的
        # 默认光标 —— 也就是那个 X 形（crosshair 样的默认 glyph）。
        # 这台机器跑的是没有窗口管理器的裸 Xorg :3，没人执行过 xsetroot，
        # 所以 root 的 cursor 一直是 None，于是任何 cursor=None 的窗口都是 X。
        # Chromium 是用 XFixes 自己设 cursor 的，所以页面正文 / 标签上一直正常，
        # 只有我们这块 override-redirect 视频窗口是 X —— 和现场观察一致。
        # 两件事都做：
        #   1) 给本窗口显式设 cursor（正解，作用域只有这一块）；
        #   2) 顺手把 root 的 cursor 也设上（兜底：将来任何忘了设 cursor 的
        #      override-redirect 窗口都不会再退化回 X 形）。
        try:
            self.cur_arrow = self._make_cursor(Xcursorfont.left_ptr)
            self.scr.root.change_attributes(cursor=self.cur_arrow)
            print('[ps] 光标已设置 (left_ptr)，root cursor 同步兜底', flush=True)
        except Exception as ex:
            print('[ps] 光标创建失败，沿用系统默认:', repr(ex), flush=True)
            self.cur_arrow = None

        kw = dict(
            background_pixel=0,
            override_redirect=True,
            event_mask=(X.ExposureMask | X.ButtonPressMask | X.ButtonReleaseMask |
                        X.PointerMotionMask | X.KeyPressMask | X.KeyReleaseMask |
                        X.EnterWindowMask | X.LeaveWindowMask))
        if self.cur_arrow is not None:
            kw['cursor'] = self.cur_arrow
        self.win = self.scr.root.create_window(
            a.x, a.y, a.w, a.h, 0, self.scr.root_depth,
            X.InputOutput, X.CopyFromParent, **kw)
        self.win.map()
        self.win.configure(stack_mode=X.Below)   # 初始藏到 Chromium 后面
        self.disp.flush()
        GLib.timeout_add(8, self._poll_x)

    def _poll_x(self):
        try:
            while self.disp.pending_events():
                self._handle_x(self.disp.next_event())
            if self.vis_dirty:
                self.vis_dirty = False
                self._refresh_visible()
            if self.geom_dirty:
                self.geom_dirty = False
                self._apply_geometry()
            if self.holes_dirty:
                self.holes_dirty = False
                self._apply_holes()
        except Exception as ex:
            print('[ps] X 事件处理异常:', repr(ex), flush=True)
        return True

    # ---------------------------------------------------------------- 窗口几何
    def set_geometry(self, x, y, w, h):
        """页面要求改变视频窗口的屏幕矩形（全屏「铺满」/还原时调用）。

        与 set_holes 一样：HTTP 线程只记参数，X 调用留给主循环。
        """
        try:
            self.pending_geom = (int(x), int(y), int(w), int(h))
        except Exception:
            return
        self.geom_dirty = True

    def _apply_geometry(self):
        if not self.pending_geom or not self.win:
            return
        x, y, w, h = self.pending_geom
        if w < 200 or h < 150:            # 页面折叠/隐藏时的 0 尺寸，忽略
            return
        cur = (int(self.args.x), int(self.args.y), int(self.args.w), int(self.args.h))
        if (x, y, w, h) == cur:
            return
        self.args.x, self.args.y, self.args.w, self.args.h = x, y, w, h
        try:
            self.win.configure(x=x, y=y, width=w, height=h)
            self.disp.flush()
        except Exception as ex:
            print('[ps] 窗口 resize 失败:', repr(ex), flush=True)
            return
        self._shape_key = None            # 尺寸变了，洞形必须按新尺寸重算
        self.holes_dirty = True
        self._set_render_rect(w, h)
        print('[ps] geometry -> %d,%d %dx%d' % (x, y, w, h), flush=True)

    def _set_render_rect(self, w, h):
        """告诉 sink 新的渲染区域：窗口变大后如果不更新，画面会停在旧尺寸。"""
        if not getattr(self, 'sink', None):
            return
        try:
            libgstvideo = ctypes.CDLL('libgstvideo-1.0.so.0')
            getptr = ctypes.pythonapi.PyCapsule_GetPointer
            getptr.restype = ctypes.c_void_p
            getptr.argtypes = [ctypes.py_object, ctypes.c_char_p]
            elem = getptr(self.sink.__gpointer__, None)
            libgstvideo.gst_video_overlay_set_render_rectangle(
                ctypes.c_void_p(elem), ctypes.c_int(0), ctypes.c_int(0),
                ctypes.c_int(w), ctypes.c_int(h))
            print('[ps] render rectangle -> %dx%d' % (w, h), flush=True)
        except Exception as ex:
            print('[ps] set_render_rectangle 失败:', repr(ex), flush=True)

    def reconnect(self):
        """页面「重新连接」：直接重启本进程（os.execv）。

        为什么不是「只断开信令让 _ws_forever 重连」：实测那样做之后，UE 会发新
        offer、我们也能回 answer，但 SCTP 通道已经 COMM_LOST，webrtcbin 1.16 的
        旧 transport 收不回来 —— 表现为 channel:false、fps:0，画面永久黑住。
        进程重启是最干脆的重建：X 窗口、pipeline、ICE/DTLS 全部重来。
        页面侧无需配合：30ms 的 geometry/shape 同步 + 2s 的 /show 心跳会自动
        把新窗口摆回原位、重新挖洞。
        """
        print('[ps] 手动重连：重启播放器进程', flush=True)
        sys.stdout.flush()
        sys.stderr.flush()
        try:
            os.execv(sys.executable, [sys.executable, '-u', sys.argv[0]] + sys.argv[1:])
        except Exception as ex:
            print('[ps] execv 失败:', repr(ex), flush=True)
            return False
        return True

    # ---------------------------------------------------------------- X Shape
    def set_holes(self, rects):
        """记录要从视频窗口里挖掉的矩形（由 HTTP 线程调用）。

        rects: [[x, y, w, h], ...]，相对视频窗口左上角的 CSS 像素。
        python-xlib 的 Display 不是线程安全的，所以这里只置脏标记，
        真正的 X 请求交给主循环（_poll_x，8ms 一次）统一下发。
        """
        try:
            clean = []
            for r in (rects or []):
                if len(r) >= 4:
                    clean.append([int(r[0]), int(r[1]), int(r[2]), int(r[3])])
            self.pending_holes = clean
        except Exception:
            self.pending_holes = []
        self.holes_dirty = True

    @staticmethod
    def _visible_rects(w, h, holes):
        """把「洞」取补，得到窗口真正可见的矩形集合（横带扫描）。

        为什么要在本地算补集，而不是「Set 整窗 + 逐个 Subtract」：
        Set 整窗再逐个 Subtract 是「先让窗口一个洞都没有，再一个个挖回来」。
        X 服务端在 Set 生效之后、Subtract 还没到齐的那一瞬间，窗口是全尺寸的
        —— 视频会盖满整个屏幕，紧接着又被挖开。屏幕上看到的就是一道黑框闪过
        （实测每开关一次设置面板必现一帧，录像里是 1080x1080 的黑块）。
        补集只发**一次** Set，形状从旧值直接跳到新值，中间不存在「没有洞」的态。

        矩形数会变多（每个洞的上下边各切一条横带），但请求只有一次；
        上下相邻、区间完全相同的横带会被合并，避免矩形数爆炸。
        """
        rs = []
        for r in holes:
            x0 = max(0, int(r[0]))
            y0 = max(0, int(r[1]))
            x1 = min(w, int(r[0]) + int(r[2]))
            y1 = min(h, int(r[1]) + int(r[3]))
            if x1 - x0 >= 1 and y1 - y0 >= 1:
                rs.append((x0, y0, x1, y1))
        if not rs:
            return [(0, 0, w, h)]
        ys = sorted(set([0, h] + [r[1] for r in rs] + [r[3] for r in rs]))
        bands = []                     # [(y0, y1, [(x0, x1), ...])]
        for i in range(len(ys) - 1):
            ya, yb = ys[i], ys[i + 1]
            if yb <= ya:
                continue
            spans = sorted((r[0], r[2]) for r in rs if r[1] <= ya and r[3] >= yb)
            iv = []
            cur = 0
            for a, b in spans:
                if b <= cur:
                    continue
                if a > cur:
                    iv.append((cur, a))
                cur = b
            if cur < w:
                iv.append((cur, w))
            if bands and bands[-1][1] == ya and bands[-1][2] == iv:
                bands[-1] = (bands[-1][0], yb, iv)
            else:
                bands.append((ya, yb, iv))
        out = []
        for ya, yb, iv in bands:
            for a, b in iv:
                out.append((a, ya, b - a, yb - ya))
        return out

    def _apply_holes(self):
        if not self.win:
            return
        w, h = int(self.args.w), int(self.args.h)
        rects = self._visible_rects(w, h, self.pending_holes)
        # 按「可见区域」判重，而不是按「洞的个数」判重：
        # 页面每 1s 会重发一次洞列表，其中常有一个被别的洞完全盖住的冗余矩形
        # （实测洞数在 197/198 之间来回跳），但它并不改变可见区域。按个数判重
        # 会每秒白重建一次形状，白闪一次；按区域判重就一个请求都不发。
        key = (w, h, tuple(rects))
        if key == self._shape_key:
            return
        self._shape_key = key
        try:
            xshape.rectangles(self.win, xshape.SO.Set, xshape.SK.Bounding,
                              0, 0, 0, rects)
            self.disp.flush()
            self.shape_applies += 1
            if len(self.pending_holes) != self._hole_count:
                self._hole_count = len(self.pending_holes)
                print('[ps] shape holes=%d visible=%d' %
                      (self._hole_count, len(rects)), flush=True)
        except Exception as ex:
            print('[ps] shape 失败:', repr(ex), flush=True)

    def show(self):
        """页面要求显示（HTTP 线程调用）——只置意图，X 请求交给主循环。

        每次都置 force_stack：窗口虽然一直是「已显示」状态，但 kiosk Chromium
        重启 / 页面 reload 之后会新建一个 X 窗口并映射在最上层，把我们这块
        override-redirect 窗口盖住 —— 表现是「页面停在仿真视窗、播放器
        visible:true、fps 也正常，但屏幕上那块是黑的」（黑的是页面的 .ue-view-box）。
        没有窗口管理器，没人会帮我们重新置顶，所以靠页面 2s 一次的 /show 心跳
        重新 raise 一次。stack_mode=Above 对已经在上层的窗口是幂等的，不会闪。
        """
        self.want_visible = True
        self.vis_dirty = True
        self.force_stack = True

    def hide(self):
        """页面要求隐藏（HTTP 线程调用）。"""
        self.want_visible = False
        self.vis_dirty = True

    def _refresh_visible(self):
        """按「页面想看」且「有画面」决定窗口是否抬到最上层（必须在主循环里调）。

        没有画面时（stalled）坚决不抬：停仿真后 UE 不再发帧，sink 会把最后一帧
        一直留在窗口里，抬起来就是「视频还在」的假象。藏到页面后面，页面自己画
        「等待 UE 视频连接」的黑底提示。
        """
        if not self.win:
            return
        should = self.want_visible and not self.stalled
        force = self.force_stack
        self.force_stack = False
        if should == self.win_visible and not force:
            return
        try:
            self.win.configure(stack_mode=X.Above if should else X.Below)
            self.disp.flush()
        except Exception as ex:
            print('[ps] 窗口层叠切换失败:', repr(ex), flush=True)
            return
        if should == self.win_visible:
            # force 触发的「重复置顶」：状态没变，不打日志、不动洞形
            return
        self.win_visible = should
        if should:
            self.holes_dirty = True      # 重新上屏后把当前洞形补上
            print('[ps] window SHOW', flush=True)
        else:
            self._release_focus()
            print('[ps] window HIDE', flush=True)

    def _clear_window(self):
        """把窗口刷成黑色。

        nveglglessink 渲染完最后一帧后不会再清屏，窗口内容会一直留着；
        断流时主动填黑，这样即使窗口因为某些原因又露出来，也不会显示旧画面。
        """
        try:
            g = self.win.get_geometry()
            gc = self.win.create_gc(foreground=self.scr.black_pixel)
            self.win.fill_rectangle(gc, 0, 0, g.width, g.height)
            self.disp.flush()
            gc.free()
            print('[ps] 窗口已清黑 %dx%d' % (g.width, g.height), flush=True)
        except Exception as ex:
            print('[ps] 清黑失败:', repr(ex), flush=True)

    def _mark_stalled(self):
        """立即判定断流（信令明确告知 streamer 断开时用，不等 2s）。"""
        if self.stalled:
            return False
        self.stalled = True
        print('[ps] streamer 断开 -> 清黑并隐藏窗口', flush=True)
        self._clear_window()
        self._refresh_visible()
        return False

    def _norm(self, wx, wy):
        """窗口坐标 -> 0..65535 归一化画面坐标（回传给 UE 的鼠标位置）。

        铺满模式（FILL_WINDOW，默认）下窗口里没有黑边，画面就是整块窗口，
        所以是纯线性缩放。若把 FILL_WINDOW 关回去（保持 16:9），必须走下面的
        letterbox 分支把黑边偏移减掉 —— 否则鼠标会整体偏到左上角。
        """
        w, h = self.args.w, self.args.h
        if FILL_WINDOW:
            nx = wx / float(w)
            ny = wy / float(h)
        else:
            va = self.video_aspect()
            wa = float(w) / float(h)
            if wa > va:                       # 左右黑边
                cw = h * va
                ox = (w - cw) / 2.0
                nx = (wx - ox) / cw
                ny = wy / float(h)
            else:                             # 上下黑边
                ch = w / va
                oy = (h - ch) / 2.0
                nx = wx / float(w)
                ny = (wy - oy) / ch
        nx = 0.0 if nx < 0 else (1.0 if nx > 1 else nx)
        ny = 0.0 if ny < 0 else (1.0 if ny > 1 else ny)
        return int(nx * 65535), int(ny * 65535)

    def video_aspect(self):
        try:
            n, d = self.args.video_aspect.split(':')
            return float(n) / float(d)
        except Exception:
            return 16.0 / 9.0

    def _handle_x(self, ev):
        t = ev.type
        if t == X.MotionNotify:
            x = ev.root_x - self.args.x
            y = ev.root_y - self.args.y
            nx, ny = self._norm(x, y)
            if self.last_x is None:
                dx = dy = 0
            else:
                w, h = float(self.args.w), float(self.args.h)
                dx = int((x - self.last_x) / w * 65534.0)
                dy = int((y - self.last_y) / h * 65534.0)
                dx = max(-32768, min(32767, dx))
                dy = max(-32768, min(32767, dy))
            self.last_x, self.last_y = x, y
            self.send_bytes(struct.pack('<BHHHH', MSG_MOUSE_MOVE,
                                        nx, ny, dx & 0xFFFF, dy & 0xFFFF))
        elif t == X.EnterNotify:
            self.last_x = self.last_y = None
            self.send_bytes(bytes([MSG_MOUSE_ENTER]))
            self._take_focus()
        elif t == X.LeaveNotify:
            self.send_bytes(bytes([MSG_MOUSE_LEAVE]))
            self._release_focus()
        elif t == X.ButtonPress:
            b = ev.detail
            nx, ny = self._norm(ev.root_x - self.args.x, ev.root_y - self.args.y)
            if b == 4 or b == 5:
                delta = 120 if b == 4 else -120
                self.send_bytes(struct.pack('<BhHH', MSG_MOUSE_WHEEL, delta, nx, ny))
            elif 1 <= b <= 3:
                self.send_bytes(struct.pack('<BBHH', MSG_MOUSE_DOWN, b - 1, nx, ny))
        elif t == X.ButtonRelease:
            b = ev.detail
            if 1 <= b <= 3:
                nx, ny = self._norm(ev.root_x - self.args.x, ev.root_y - self.args.y)
                self.send_bytes(struct.pack('<BBHH', MSG_MOUSE_UP, b - 1, nx, ny))
        elif t == X.KeyPress:
            self._on_key_press(ev.detail)
        elif t == X.KeyRelease:
            self._on_key_release(ev.detail)

    def _key_sym(self, kc):
        try:
            return self.disp.keycode_to_keysym(kc, 0)
        except Exception:
            return 0

    def _on_key_press(self, kc):
        ks = self._key_sym(kc)
        code = js_keycode(ks)
        if code is None:
            return
        repeat = 1 if kc in self.pressed else 0
        self.pressed.add(kc)
        if ks in (0xFFE1, 0xFFE2):
            self.shift = True
        if ks == 0xFFE5:
            self.caps = not self.caps
        self.send_bytes(struct.pack('<BBB', MSG_KEY_DOWN, code & 0xFF, repeat))
        ch = js_char(ks, self.shift, self.caps)
        if ch:
            self.send_bytes(struct.pack('<BH', MSG_KEY_PRESS, ord(ch[0])))

    def _on_key_release(self, kc):
        ks = self._key_sym(kc)
        self.pressed.discard(kc)
        if ks in (0xFFE1, 0xFFE2):
            self.shift = False
        code = js_keycode(ks)
        if code is None:
            return
        self.send_bytes(struct.pack('<BB', MSG_KEY_UP, code & 0xFF))

    def _take_focus(self):
        if self.focus_owned:
            return
        try:
            cur = self.disp.get_input_focus().focus
            curid = getattr(cur, 'id', None)
            if curid is None:
                try:
                    curid = int(cur)
                except Exception:
                    curid = -1
            if curid != int(self.win.id):
                self.prev_focus = cur
        except Exception:
            self.prev_focus = None
        try:
            self.win.set_input_focus(X.RevertToParent, X.CurrentTime)
            self.disp.flush()
            self.focus_owned = True
        except Exception as ex:
            print('[ps] set_input_focus 失败:', repr(ex), flush=True)

    def _release_focus(self):
        if not self.focus_owned:
            return
        w = self.prev_focus
        self.focus_owned = False
        if w is None:
            return
        try:
            w.set_input_focus(X.RevertToParent, X.CurrentTime)
            self.disp.flush()
        except Exception:
            pass

    # ------------------------------------------------------------ WebRTC
    def _on_offer(self, sdp_text):
        if 'H264/90000' not in sdp_text:
            print('[ps] 警告: UE offer 中没有 H264（引擎当前应为 VP8）。'
                  '请在 Web「仿真配置」选择 H264 后重新启动仿真。', flush=True)
        def work():
            try:
                r = GstSdp.SDPMessage.new_from_text(sdp_text)
                desc = GstWebRTC.WebRTCSessionDescription.new(
                    GstWebRTC.WebRTCSDPType.OFFER, r[1] if isinstance(r, tuple) else r)
                p = Gst.Promise.new()
                self._promises.append(p)
                self.webrtc.emit('set-remote-description', desc, p)
                self.remote_set = True
                for m, c in self.pending_ice:
                    self.webrtc.emit('add-ice-candidate', m, c)
                self.pending_ice = []
                pa = Gst.Promise.new_with_change_func(self._on_answer, self.webrtc)
                self._promises.append(pa)
                self.webrtc.emit('create-answer', None, pa)
            except Exception as ex:
                print('[ps] 处理 offer 失败:', repr(ex), flush=True)
            return False
        GLib.idle_add(work)

    def _on_answer(self, promise, webrtc):
        try:
            reply = promise.get_reply()
            if reply is None:
                print('[ps] create-answer 无 reply', flush=True)
                return
            ans = reply.get_value('answer')
            t = ans.sdp.as_text()
            if not isinstance(t, str):
                t = [x for x in t if isinstance(x, str)][0]
            t = fix_rejected_fp(t)
            filtered = filter_answer_h264(t)
            r2 = GstSdp.SDPMessage.new_from_text(t)
            msg2 = r2[1] if isinstance(r2, tuple) else r2
            ans2 = GstWebRTC.WebRTCSessionDescription.new(
                GstWebRTC.WebRTCSDPType.ANSWER, msg2)
            p = Gst.Promise.new()
            self._promises.append(p)
            self.webrtc.emit('set-local-description', ans2, p)
            self._send_json({'type': 'answer', 'sdp': filtered})
            # 协商看门狗从这里开始计时（见 _tick_stats 末尾）
            self._answer_at = time.time()
            self._frames_at_answer = self.stat['frames']
            print('[ps] answer 已发送（已过滤为仅 H264，%d 字节）' % len(filtered), flush=True)
        except Exception as ex:
            print('[ps] 生成 answer 失败:', repr(ex), flush=True)

    def _send_json(self, obj):
        if not self.ws:
            return
        txt = json.dumps(obj)
        asyncio.run_coroutine_threadsafe(self.ws.send(txt), self.loop)

    def add_remote_ice(self, mline, cand):
        if not cand:
            return
        if self.remote_set:
            try:
                self.webrtc.emit('add-ice-candidate', int(mline), cand)
            except Exception as ex:
                print('[ps] add-ice-candidate 失败:', repr(ex), flush=True)
        else:
            self.pending_ice.append((int(mline), cand))

    # ------------------------------------------------------------ 信令
    def start_signalling(self):
        self.ws = None
        self.streamer_id = None
        self.last_ids = None
        self.loop = asyncio.new_event_loop()
        t = threading.Thread(target=self._ws_thread, daemon=True)
        t.start()

    def _ws_thread(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._ws_forever())

    async def _ws_forever(self):
        url = 'ws://%s:%d/' % (self.args.ue_host, self.args.ue_port)
        while True:
            try:
                async with websockets.connect(url, max_size=16 * 1024 * 1024,
                                              ping_interval=5, ping_timeout=10) as ws:
                    self.ws = ws
                    self.streamer_id = None
                    self.last_ids = None
                    print('[ps] 已连接信令 %s' % url, flush=True)
                    # 信令掉线 = UE 那边重启过仿真，旧 session 已经死了。
                    # 必须在订阅 / 收到新 offer 之前把 webrtcbin 换掉。
                    await self._rebuild_async('信令重连')
                    await ws.send(json.dumps({'type': 'listStreamers'}))
                    poll = asyncio.create_task(self._poll_streamers(ws))
                    try:
                        async for raw in ws:
                            await self._on_signal(raw)
                    finally:
                        poll.cancel()
            except Exception as ex:
                print('[ps] 信令断开: %r，2s 后重连' % (ex,), flush=True)
            self.ws = None
            self.streamer_id = None
            self.chan = None
            self.chan_open = False
            self.remote_set = False
            await asyncio.sleep(2)

    async def _poll_streamers(self, ws):
        """**无条件**轮询 streamer 列表（每 2s 一次 listStreamers）。

        原来是「只在 streamer_id is None 时才问」，有两个致命场景：
        1. UE 晚于播放器启动 —— 已订阅过就再也不问，UE 上线也发现不了；
        2. **UE 重启但 SS 没重启** —— 播放器的 WebSocket 一直活着，SS 不会给
           一个「已经订阅过」的播放器重推 streamerList。于是播放器永远停在旧
           streamer 上：connected=true / channel=false / frames=0 / stalled=true，
           页面全黑且无自愈（2026-10-24 19:33 实测，只能手动 /reconnect 才恢复）。

        listStreamers 的响应只有几十字节，2s 一次的开销可以忽略；真正的动作
        都放在 _on_signal 里判（列表没变就什么都不做）。
        """
        while True:
            # 空闲退避（_empty_lists 的维护见 _on_signal 的 streamerList 分支）：
            # 列表一直为空时 2s → 4s → 6s → 8s → 10s 封顶，省掉仿真关闭期间
            # 无意义的空轮询；一旦看到 streamer 立刻回到 2s。
            await asyncio.sleep(2 if self._empty_lists < 2 else min(10, 2 * self._empty_lists))
            if ws.closed:
                return
            try:
                await ws.send(json.dumps({'type': 'listStreamers'}))
            except Exception:
                return

    async def _on_signal(self, raw):
        try:
            msg = json.loads(raw)
        except Exception:
            return
        t = msg.get('type')
        if t == 'streamerList':
            ids = msg.get('ids') or msg.get('streamerList') or []
            if ids != self.last_ids:
                print('[ps] streamers: %s' % (ids,), flush=True)
                self.last_ids = ids
            # 空列表计数：只驱动 _poll_streamers 的退避，不参与下面的状态机
            self._empty_lists = 0 if ids else self._empty_lists + 1
            if not ids:
                # UE 下线 / 还没上线。必须把订阅作废：否则它用**同一个**
                # streamerId 重新上线时（默认就叫 DefaultStreamer），下面那句
                # `ids[0] != self.streamer_id` 判不出来，播放器会一直挂在旧
                # session 上 —— frames=0 / channel=false 永久黑屏。
                if self.streamer_id is not None:
                    print('[ps] streamer 列表为空，作废旧订阅 %s'
                          % self.streamer_id, flush=True)
                    self.streamer_id = None
                    GLib.idle_add(self._mark_stalled)
                    # 旧 session 的 webrtcbin 已经没用了，顺手换掉，
                    # 免得新 offer 落在半死的 transport 上
                    await self._rebuild_async('streamer 下线')
                return
            if ids[0] != self.streamer_id:
                if self.streamer_id is not None:
                    # 换了 streamer = 换了一个全新 session。旧 webrtcbin 的
                    # transport 在 SCTP COMM_LOST 之后收不回来（见 reconnect()），
                    # 必须先重建，否则新 offer 也能 answer，但永远收不到帧。
                    print('[ps] streamer 变更 %s -> %s，重建解码链'
                          % (self.streamer_id, ids[0]), flush=True)
                    await self._rebuild_async('streamer 变更')
                print('[ps] 订阅 streamer %s' % ids[0], flush=True)
                await self.ws.send(json.dumps({'type': 'subscribe', 'streamerId': ids[0]}))
                self.streamer_id = ids[0]
        elif t == 'streamerDisconnected':
            print('[ps] UE streamer 已断开，等待重新上线', flush=True)
            self.streamer_id = None
            GLib.idle_add(self._mark_stalled)
            # 重新上线的 streamer 会带新 session 的 offer，同样要全新的 webrtcbin
            await self._rebuild_async('streamer 断开')
        elif t == 'offer':
            print('[ps] 收到 UE offer（%d 字节）' % len(msg.get('sdp', '')), flush=True)
            self._on_offer(msg.get('sdp', ''))
        elif t == 'iceCandidate':
            c = msg.get('candidate') or {}
            idx = c.get('sdpMLineIndex', c.get('sdpMlineIndex', c.get('sdpMlineindex', 0)))
            GLib.idle_add(self.add_remote_ice, int(idx or 0), c.get('candidate', ''))
        elif t == 'playerConnected':
            print('[ps] playerConnected %s' % (msg,), flush=True)
        else:
            print('[ps] 信令消息: %s' % t, flush=True)


def fix_rejected_fp(text):
    """修正 webrtcbin create-answer 对被 reject m-line 的指纹处理。

    UE offer 的 audio 是 recvonly，webrtcbin 自动建的 transceiver 也是 recvonly，
    求交失败 -> media 1 rejected。被 reject 的 m-line 原样保留了 **UE offer 里的
    a=fingerprint**，而 accepted 的 m-line 换成了 webrtcbin 自己的，于是
    set-local-description 报
        "Fingerprint in media 1 differs from previous fingerprint"
    整个本地描述不生效 -> DTLS/ICE 都不启动 -> UE 侧 ICE 21s 超时、没有画面。

    这里把 port=0（被 reject）的 m-line 的 a=fingerprint 换成参考指纹即可。
    """
    lines = text.split('\r\n')
    ref = None
    for ln in lines:
        if ln.startswith('a=fingerprint:'):
            ref = ln
            break
    if not ref:
        return text
    out = []
    rejected = False
    for ln in lines:
        if ln.startswith('m='):
            parts = ln.split(' ')
            rejected = len(parts) > 1 and parts[1] == '0'
            out.append(ln)
            continue
        if rejected and ln.startswith('a=fingerprint:'):
            out.append(ref)
            continue
        out.append(ln)
    return '\r\n'.join(out)


def filter_answer_h264(text):
    """把 answer 中 video 段里非 H264 的 payload 去掉。

    UE 5.2 的 Streamer 解析 answer 时按出现顺序取第一个它支持的编码器；
    webrtcbin 默认会同时给 VP8/H264，UE 会优先选 VP8，导致 Jetson 无法硬解。
    这里只保留 H264 的 rtpmap/fmtp/rtcp-fb 与 m=video 的 payload 列表。
    """
    lines = text.split('\r\n')
    out = []
    video_pts = None          # 当前 video 段保留的 payload type
    video_dropping = False
    i = 0
    while i < len(lines):
        ln = lines[i]
        if ln.startswith('m='):
            kind = ln[2:].split(' ')[0]
            if kind == 'video':
                parts = ln.split(' ')
                pts = parts[3:]
                keep = []
                # 先扫描本段所有 rtpmap，确定哪些是 H264
                h264_pts = set()
                j = i + 1
                while j < len(lines) and not lines[j].startswith('m='):
                    l2 = lines[j]
                    if l2.startswith('a=rtpmap:'):
                        seg = l2[len('a=rtpmap:'):].split(' ', 1)
                        if len(seg) == 2 and seg[1].upper().startswith('H264'):
                            h264_pts.add(seg[0])
                    j += 1
                keep = [p for p in pts if p in h264_pts]
                if not keep:
                    # 没有 H264 就保持原样（至少不至于把流彻底废掉）
                    out.append(ln)
                    video_pts = None
                    video_dropping = False
                else:
                    out.append(' '.join(parts[:3] + keep))
                    video_pts = set(keep)
                    video_dropping = True
            else:
                out.append(ln)
                video_pts = None
                video_dropping = False
            i += 1
            continue

        if video_dropping:
            drop = False
            for pfx in ('a=rtpmap:', 'a=fmtp:', 'a=rtcp-fb:'):
                if ln.startswith(pfx):
                    rest = ln[len(pfx):]
                    pt = rest.split(' ', 1)[0]
                    if pt != '*' and pt not in video_pts:
                        drop = True
                    break
            if drop:
                i += 1
                continue
        out.append(ln)
        i += 1
    return '\r\n'.join(out)


# ---------------------------------------------------------------- HTTP 控制
class CtlHandler(BaseHTTPRequestHandler):
    player = None

    # 页面在 http://127.0.0.1:8080，控制口在 8099 —— 跨源，必须给 CORS，
    # 且带 JSON body 的 POST 会先发 OPTIONS 预检，所以 OPTIONS 也要应答。
    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.player.opt_reqs += 1
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length') or 0)
        except Exception:
            n = 0
        raw = self.rfile.read(n) if n > 0 else b''

        if self.path.startswith('/shape'):
            self.player.shape_reqs += 1
            rects = []
            if raw:
                try:
                    rects = (json.loads(raw.decode('utf-8', 'replace')).get('rects') or [])
                except Exception:
                    rects = []
            self.player.set_holes(rects)
            self._json({'ok': True, 'holes': len(rects)})
            return
        if self.path.startswith('/geometry'):
            geom = None
            if raw:
                try:
                    o = json.loads(raw.decode('utf-8', 'replace'))
                    geom = [o.get('x'), o.get('y'), o.get('w'), o.get('h')]
                except Exception:
                    geom = None
            if not geom or any(v is None for v in geom):
                self._json({'ok': False, 'error': 'bad geometry'}, 400)
                return
            self.player.set_geometry(*geom)
            self._json({'ok': True, 'geom': [int(v) for v in geom]})
            return
        if self.path.startswith('/reconnect'):
            self._json({'ok': self.player.reconnect()})
            return
        if self.path.startswith('/show'):
            self.player.show()
        elif self.path.startswith('/hide'):
            self.player.hide()
        else:
            self.send_error(404)
            return
        self._json({'ok': True})

    def do_GET(self):
        snap = self.player.stats_snapshot()
        self._json({
            'visible': self.player.win_visible,
            'channel': bool(self.player.chan_open),
            'connected': bool(self.player.ws is not None),
            'decoding': bool(self.player.chain_linked),
            'streamer': getattr(self.player, 'streamer_id', None),
            'holes': len(self.player.pending_holes),
            'shapeReqs': self.player.shape_reqs,
            'shapeApplies': self.player.shape_applies,
            'optReqs': self.player.opt_reqs,
            'fps': snap['fps'],
            'kbps': snap['kbps'],
            'width': snap['width'],
            'height': snap['height'],
            'frames': snap['frames'],
            'uptime': snap['uptime'],
            'geom': snap['geom'],
            'stalled': snap['stalled'],
            'want': snap['want'],
        })

    def log_message(self, fmt, *a):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ue-host', default='192.168.5.4')
    ap.add_argument('--ue-port', type=int, default=80,
                    help='像素流 player 端口（与 Web 页面同一入口）')
    ap.add_argument('--x', type=int, default=300)
    ap.add_argument('--y', type=int, default=48)
    ap.add_argument('--w', type=int, default=1620)
    ap.add_argument('--h', type=int, default=911)
    ap.add_argument('--video-aspect', default='16:9')
    ap.add_argument('--ctl-port', type=int, default=8099)
    ap.add_argument('--stun', default='stun://stun.l.google.com:19302')
    ap.add_argument('--show-at-start', action='store_true')
    ap.add_argument('--dry-run', action='store_true',
                    help='只校验元素/窗口/协议，不连信令')
    args = ap.parse_args()

    p = NativePlayer(args)

    srv = ThreadingHTTPServer(('127.0.0.1', args.ctl_port), CtlHandler)
    CtlHandler.player = p
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('[ps] 控制接口 http://127.0.0.1:%d/{show,hide,shape,geometry,reconnect,status}' % args.ctl_port, flush=True)

    if args.dry_run:
        for f in ('webrtcbin', 'rtpjitterbuffer', 'rtph264depay', 'h264parse',
                  'nvv4l2decoder', 'nvvidconv', 'nveglglessink'):
            e = Gst.ElementFactory.make(f, None)
            print('[ps] element %-18s %s' % (f, 'OK' if e is not None else 'MISSING'), flush=True)
        print('[ps] dry-run: 元素与窗口就绪', flush=True)
        if args.show_at_start:
            p.show()
        GLib.timeout_add(1500, lambda: (main_loop.quit(), False)[-1])
        main_loop.run()
        return

    p.pipe.set_state(Gst.State.PLAYING)
    p.start_signalling()
    if args.show_at_start:
        p.show()

    main_loop.run()


main_loop = GLib.MainLoop()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
