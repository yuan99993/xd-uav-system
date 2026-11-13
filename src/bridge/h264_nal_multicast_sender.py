#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
h264_nal_multicast_sender.py  (程序 1 / 2: 发送端, 完全裸流版)

订阅 sensor_msgs/Image -> Jetson 硬编码器或 ffmpeg(libx264) 编码成 H.264 Annex-B 裸流(ES)
-> 把**裸流字节**直接切成定长 UDP 数据报发出(组播或单播),
   **绝不添加任何私有帧头 / 分片头 / NAL 序号**。

线上载荷 == 纯 H.264 裸流字节, 可被任何标准解码器(VLC / ffplay / GStreamer / ffmpeg)直接消费:
  * VLC:   udp://@:32797 (组播)            (组播需带组地址; 单播无需组地址)
  * 本仓库接收端 h264_nal_multicast_receiver.py 把字节流喂给 ffmpeg 解码 -> 重发 ROS Image

为什么"完全裸流"可行(且不需要私有协议):
  * ffmpeg 输出 Annex-B(00 00 00 01 起始码), NAL 边界自描述, 接收端不需要我们给边界信息。
  * 每条 UDP 数据报承载一段裸流字节(默认 1400B, 远小于 MTU, 避免 IP 分片);
    接收端按到达顺序拼接成连续字节流交给 ffmpeg, ffmpeg 自行解析起始码组 NAL。
    因此完全不需要任何私有分片/重组协议。
  * 代价(裸 UDP 固有): 不保证有序/不丢。LAN/本机通常有序且不丢; 个别包丢失会在该 GOP 内
    产生解码错误, 直到下一个 IDR(repeat-headers=1 保证周期重发 SPS/PPS)自愈。

参数:
  _image_topic  订阅的 Image 话题 (默认 DownCamera/scene_camera/image)
  _group/_port  组播地址/端口 (默认 224.0.5.4 / 32797)
  _local_ip     组播发送网卡的本机 IPv4 地址 (默认 192.168.2.102)
  _ttl          组播 TTL (默认 4)
  _fps/_gop     目标输出帧率/关键帧间隔 (默认 30 / 30)。
                 发送端会按 _fps 对输入图像做抽帧(throttle): 源帧率高于 _fps 时只转发
                 约 _fps 帧/秒、丢弃多余的帧; 源帧率低于 _fps 则全部转发。
                 即设 _fps:=10 即把输出压到 10fps。
  _bitrate      ffmpeg -b:v (默认 2M; 参考 output_5s.h264 实测 ~2.4Mbps@30fps —— 注意文件名"5s"不实, 文件实为 ~50s/1495帧, 非 5s; 如需更高画质可调 _bitrate:=8M)
  _encoder      auto(默认: Jetson 硬编码可用时使用 jetson) | jetson | libx264
  _profile      H.264 profile (可选 high | baseline, 默认 high;
                  high=参考 output_5s.h264 (High/profile_idc 100 + CABAC);
                  baseline=Constrained Baseline, 兼容老硬解/低复杂度解码器)
  _level        libx264 的 H.264 level (默认 4.2；Jetson 编码器自动选择)
  _unicast_host 非空则改为单播发往该 IP

典型运行:
  python h264_nal_multicast_sender.py \
      _image_topic:=/Sim/SceneBasicDrone/robots/Drone1/sensors/DownCamera/scene_camera/image \
      _group:=224.0.5.4 _port:=32797 _ttl:=4 _fps:=30 _gop:=30
  # 单播: 追加 _unicast_host:=192.168.5.4
"""

import os
import shutil
import rospy
import numpy as np
import socket
import subprocess
import threading
import time
from fractions import Fraction

from sensor_msgs.msg import Image

CHUNK = 1400   # 每包裸流字节数(远小于 MTU, 避免 IP 分片)


class H264RawSender:
    def __init__(self):
        # self.image_topic = rospy.get_param("~image_topic",
        #     "/uav1/fixed_camera/tracking_overlay")
        # self.image_topic = rospy.get_param("~image_topic",
        #     "/uav1/sar_yolo_detector/smart/annotated")
        self.image_topic = rospy.get_param("~image_topic",
            "/uav1/track/phase_annotated")
        self.group = rospy.get_param("~group", "224.0.5.4")
        self.port = int(rospy.get_param("~port", "32797"))
        self.local_ip = rospy.get_param("~local_ip", "192.168.2.102")
        self.ttl = int(rospy.get_param("~ttl", "4"))
        self.fps = float(rospy.get_param("~fps", "30"))
        if self.fps <= 0:
            raise ValueError("~fps 必须大于 0")
        self.gop = int(rospy.get_param("~gop", "30"))
        self.bitrate = rospy.get_param("~bitrate", "2M")
        # 硬编码默认 profile=high(对齐参考文件 output_5s.h264: High + CABAC + level 4.2);
        # 仍需兼容老硬解时可临时传 _profile:=baseline 覆盖。
        self.profile = rospy.get_param("~profile", "high")
        if self.profile not in ("high", "baseline"):
            rospy.logwarn("[h264_tx] 未知 profile=%r, 仅支持 high/baseline, 回退 high",
                          self.profile)
            self.profile = "high"
        self.level = rospy.get_param("~level", "4.2")
        self.ffmpeg_path = rospy.get_param("~ffmpeg_path", "ffmpeg")
        self.encoder = rospy.get_param("~encoder", "auto")
        if self.encoder not in ("auto", "jetson", "libx264"):
            raise ValueError("~encoder 只支持 auto、jetson、libx264")
        if self.encoder == "auto":
            self.encoder = ("jetson" if os.path.exists("/dev/nvhost-msenc")
                            and shutil.which("gst-launch-1.0") else "libx264")
        if self.encoder == "jetson" and not shutil.which("gst-launch-1.0"):
            raise RuntimeError("Jetson 编码需要 gst-launch-1.0")
        self.unicast_host = rospy.get_param("~unicast_host", "")  # 非空则单播发往该 IP

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        if not self.unicast_host:
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, self.ttl)
            # 指定板卡上的组播出口网卡，避免多网卡环境下由路由表选错接口。
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                                 socket.inet_aton(self.local_ip))

        self.proc = None
        self._pending_cv = threading.Condition()
        self._pending = None
        self._stop = threading.Event()
        self._started = False
        self._w = self._h = 0
        self._submitted = 0
        self._sent_frames = 0
        self._tx_bytes = 0
        self._write_s = 0.0
        self._recv = 0             # 收到的图像消息数(用于对比源帧率 vs 编码帧率)
        self._dropped = 0          # 因抽帧被丢弃的帧数
        self._overwritten = 0      # 编码忙时被更新帧覆盖的帧数
        self._next_fwd = 0.0       # 下一次允许入队的时间(单调时钟)

        if self.unicast_host:
            rospy.loginfo("[h264_tx] 模式=单播 -> %s:%d (完全裸流)", self.unicast_host, self.port)
        else:
            rospy.loginfo("[h264_tx] 模式=组播 %s:%d ttl=%d 出口=%s (完全裸流)",
                          self.group, self.port, self.ttl, self.local_ip)
        rospy.loginfo("[h264_tx] subscribing %s", self.image_topic)
        self._encode_thr = threading.Thread(target=self._encoder_loop, daemon=True)
        self._encode_thr.start()
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback,
                                    queue_size=1, buff_size=2**24)
        self._stat_interval = 5.0
        # 定时用墙钟线程(不用 rospy.Timer): rospy.Timer 依赖 /clock,
        # 模拟时间(/clock)回退时抛 ROSTimeMovedBackwardsException 挂掉诊断线程。
        self._stat_thr = threading.Thread(target=self._stat_loop, daemon=True)
        self._stat_thr.start()
        self._watch = threading.Thread(target=self._watchdog, daemon=True)
        self._watch.start()
        rospy.on_shutdown(self.shutdown)

    # ------------------------------------------------------------------ #
    def _start_encoder(self, w, h):
        if self.encoder == "jetson":
            # fdsrc/videoparse 接收完整 BGR 帧；NVMM 转换后交给 Jetson 硬编码器。
            # fdsink 的 stdout 仍是 Annex-B 裸流，网络格式不变。
            fps = Fraction(self.fps).limit_denominator(1000)
            bitrate = str(self.bitrate).strip().upper()
            if bitrate.endswith("M"):
                bitrate_bps = int(float(bitrate[:-1]) * 1000000)
            elif bitrate.endswith("K"):
                bitrate_bps = int(float(bitrate[:-1]) * 1000)
            else:
                bitrate_bps = int(bitrate)
            cmd = [
                "gst-launch-1.0", "-q",
                "fdsrc", "fd=0", "blocksize=%d" % (w * h * 3), "!",
                "videoparse", "format=bgr", "width=%d" % w, "height=%d" % h,
                "framerate=%d/%d" % (fps.numerator, fps.denominator), "!",
                "videoconvert", "!", "video/x-raw,format=I420", "!",
                "nvvidconv", "!", "video/x-raw(memory:NVMM),format=NV12", "!",
                "nvv4l2h264enc", "bitrate=%d" % bitrate_bps,
                "iframeinterval=%d" % self.gop, "idrinterval=%d" % self.gop,
                "profile=%d" % (4 if self.profile == "high" else 0),
                "insert-sps-pps=true", "insert-aud=true", "maxperf-enable=true", "!",
                "h264parse", "config-interval=-1", "!", "fdsink", "fd=1", "sync=false",
            ]
        else:
            cmd = self._ffmpeg_cmd(w, h)
        rospy.loginfo("[h264_tx] encoder=%s: %s", self.encoder, " ".join(cmd))
        try:
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                         stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL, bufsize=0)
            self._reader = threading.Thread(target=self._reader_thread, daemon=True)
            self._reader.start()
        except Exception as e:  # noqa: BLE001
            rospy.logerr("[h264_tx] 无法启动编码器: %s", e)
            self.proc = None

    def _ffmpeg_cmd(self, w, h):
        # High: 用 8x8dct + CABAC 强制 profile_idc=100(参考 output_5s.h264 的 High+CABAC)
        # Baseline: 不带 8x8dct/cabac(baseline 不支持, 否则 x264 报错或忽略 profile 请求)
        xp = "keyint=%d:min-keyint=%d:repeat-headers=1:bframes=0:aud=1:profile=%s" % (
            self.gop, self.gop, self.profile)
        if self.profile == "high":
            xp += ":8x8dct=1:cabac=1"
        cmd = [
            self.ffmpeg_path, "-hide_banner", "-loglevel", "error",
            "-fflags", "nobuffer", "-flags", "low_delay",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", "%dx%d" % (w, h), "-framerate", "%.3f" % self.fps,
            "-i", "-", "-an",
            "-c:v", "libx264",
            "-profile:v", self.profile,
            "-level", self.level,
            "-preset", "ultrafast", "-tune", "zerolatency",
            "-x264-params", xp,
            "-b:v", str(self.bitrate),
            "-pix_fmt", "yuv420p",
            "-f", "h264", "-",      # 输出 Annex-B 裸流到 stdout
        ]
        return cmd

    # ------------------------------------------------------------------ #
    def _reader_thread(self):
        """读 ffmpeg stdout 的裸流字节, 按 CHUNK 切成原始 UDP 数据报(无帧头)发出。"""
        dst = (self.unicast_host, self.port) if self.unicast_host else (self.group, self.port)
        tail = b""
        while not self._stop.is_set():
            if self.proc is None or self.proc.stdout is None:
                break
            data = self.proc.stdout.read(CHUNK)
            if not data:
                if self.proc.poll() is not None:
                    break
                continue
            # 两种编码器都插入 AUD；它的 NAL 起始码可统计真正输出的帧数。
            stream = tail + data
            frames = stream.count(b"\x00\x00\x01\x09")
            tail = stream[-3:]
            try:
                self.sock.sendto(data, dst)
                self._sent_frames += frames
                self._tx_bytes += len(data)
            except Exception as e:  # noqa: BLE001
                rospy.logerr_throttle(5.0, "[h264_tx] sendto 失败: %s", e)

    # ------------------------------------------------------------------ #
    def _stat_loop(self):
        """墙钟版本的统计定时器: 不用 rospy.Timer(/clock), 时间回退不抛异常。"""
        while not self._stop.is_set():
            time.sleep(self._stat_interval)
            if not self._stop.is_set():
                self._stat_cb(None)

    # ------------------------------------------------------------------ #
    def _stat_cb(self, event):
        if self._started and (self._submitted or self._dropped or self._recv):
            recv_fps = self._recv / self._stat_interval
            submit_fps = self._submitted / self._stat_interval
            sent_fps = self._sent_frames / self._stat_interval
            rospy.loginfo(
                "[h264_tx] 接收=%.1f fps | 提交编码=%.1f fps | 实际发出=%.1f fps "
                "| 码率=%.2f Mbps | 抽帧=%d 覆盖=%d | 管道写入均时=%.1f ms "
                "(最近 %.0fs, 目标 %.1f fps, encoder=%s)",
                recv_fps, submit_fps, sent_fps,
                self._tx_bytes * 8 / self._stat_interval / 1e6,
                self._dropped, self._overwritten,
                self._write_s * 1000 / max(1, self._submitted),
                self._stat_interval, self.fps, self.encoder)
            self._submitted = 0
            self._sent_frames = 0
            self._tx_bytes = 0
            self._write_s = 0.0
            self._dropped = 0
            self._overwritten = 0
            self._recv = 0
        elif self._started:
            rospy.logwarn("[h264_tx] 编码器已启动但 %.0fs 内未收到新帧(检查话题是否仍在发图)"
                          % self._stat_interval)

    # ------------------------------------------------------------------ #
    def _watchdog(self):
        while not self._stop.is_set():
            if self.proc is not None and self.proc.poll() is not None:
                rc = self.proc.returncode
                if not self._stop.is_set():
                    rospy.logerr("[h264_tx] 编码器已退出 (returncode=%s)! "
                                 "编码链路中断。检查编码参数和输入分辨率。", rc)
                break
            # 用墙钟 sleep(不依赖 /clock): 只检测 ffmpeg 是否退出, 不需要 ROS 时间,
            # 避免模拟时间回退时抛 ROSTimeMovedBackwardsException 杀死 watchdog 线程。
            time.sleep(2.0)

    # ------------------------------------------------------------------ #
    def image_callback(self, msg):
        h, w = msg.height, msg.width
        if self._started and (w != self._w or h != self._h):
            rospy.logwarn_throttle(5.0,
                "[h264_tx] 分辨率变化 (%dx%d -> %dx%d), 跳过本帧", self._w, self._h, w, h)
            return

        self._recv += 1             # 收到一帧有效图像(用于诊断源帧率)

        # --- 帧率限制: 按目标 _fps 抽帧, 丢弃多余帧 ---
        # 源帧率高于 _fps 时只转发约 _fps 帧/秒, 让输出真正降到指定帧率;
        # 编码器的输入帧率也等于 _fps。允许四分之一帧间隔的到达抖动，
        # 避免 30fps 来源偶有提前到达时被误丢成较低帧率。
        min_dt = 1.0 / self.fps
        now = time.monotonic()
        if self._next_fwd > 0 and now + min_dt * 0.25 < self._next_fwd:
            self._dropped += 1
            return                      # 超出目标帧率, 丢弃本帧(不喂给 ffmpeg)
        self._next_fwd = max(now, self._next_fwd) + min_dt

        # ROS 回调不等待编码器：只保留最新帧，避免管道阻塞拖慢订阅。
        with self._pending_cv:
            if self._pending is not None:
                self._overwritten += 1
            self._pending = msg
            self._pending_cv.notify()

    def _encoder_loop(self):
        while not self._stop.is_set():
            with self._pending_cv:
                while self._pending is None and not self._stop.is_set():
                    self._pending_cv.wait(timeout=1.0)
                if self._stop.is_set():
                    break
                msg = self._pending
                self._pending = None
            if not self._started:
                self._w, self._h = msg.width, msg.height
                self._start_encoder(self._w, self._h)
                self._started = True
                if self.proc is None:
                    return
                rospy.loginfo("[h264_tx] 首帧 %dx%d enc=%s, 编码器已启动",
                              self._w, self._h, msg.encoding)
            if msg.width != self._w or msg.height != self._h:
                continue
            self._encode_image(msg)

    def _encode_image(self, msg):
        h, w = msg.height, msg.width

        # 转成编码器需要的 bgr24 裸字节。
        # bgr8 且无视差行 padding 时, ROS 数据本身已是 (h,w,3) 行主序裸字节,
        # 直接 write(msg.data) 省掉一次 6MB numpy 拷贝(1080p 每帧约 6.2MB,
        # 可以降低拷贝开销)。
        try:
            if msg.encoding == "bgr8" and len(msg.data) == h * w * 3:
                raw = msg.data
            elif msg.encoding in ("bgr8", "rgb8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                arr = arr.reshape(h, -1)[:, : w * 3].reshape(h, w, 3)
                if msg.encoding == "rgb8":
                    arr = arr[:, :, ::-1]
                raw = arr.tobytes()
            elif msg.encoding in ("mono8", "8UC1"):
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                arr = arr.reshape(h, -1)[:, : w].reshape(h, w)
                raw = np.stack([arr, arr, arr], axis=-1).tobytes()
            else:
                rospy.logwarn_throttle(5.0,
                    "[h264_tx] 不支持的 encoding=%s, 跳过", msg.encoding)
                return
        except Exception as e:  # noqa: BLE001
            rospy.logerr_throttle(5.0, "[h264_tx] 解析图像失败: %s", e)
            return

        if self.proc is None or self.proc.stdin is None:
            return
        try:
            start = time.monotonic()
            self.proc.stdin.write(raw)
            self._write_s += time.monotonic() - start
            self._submitted += 1
        except Exception as e:  # noqa: BLE001
            rospy.logerr_throttle(5.0, "[h264_tx] 写编码器 stdin 失败: %s", e)

    # ------------------------------------------------------------------ #
    def shutdown(self):
        rospy.loginfo("[h264_tx] shutting down...")
        self._stop.set()
        with self._pending_cv:
            self._pending_cv.notify_all()
        if self.proc is not None:
            try:
                if self.proc.stdin:
                    self.proc.stdin.close()
            except Exception:  # noqa: BLE001
                pass
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2)
            except Exception:  # noqa: BLE001
                try:
                    self.proc.kill()
                except Exception:  # noqa: BLE001
                    pass
        try:
            self.sock.close()
        except Exception:  # noqa: BLE001
            pass


def main():
    rospy.init_node("h264_nal_multicast_sender", anonymous=True)
    try:
        H264RawSender()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
