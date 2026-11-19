#!/usr/bin/env python3
"""ProjectAirSim camera video stream receiver -- reference decoder example.

This is the client side of the optional camera video streaming implemented in
ProjectAirSim (UnrealCamera -> NVENC -> RTP/H.264 -> UDP multicast).

Protocol summary
----------------
Control channel (UDP, default port 49999), one text command per datagram:

    SUBSCRIBE   <camera> <image_type> <client_id>
    HEARTBEAT   <camera> <image_type> <client_id>
    UNSUBSCRIBE <camera> <image_type> <client_id>
    KEYFRAME    <camera> <image_type>

The server answers with a single line:

    OK <multicast_ip> <port>      on a successful SUBSCRIBE
    OK                            on HEARTBEAT / KEYFRAME / UNSUBSCRIBE
    ERR <detail>                  otherwise

A client must send a HEARTBEAT at least every 3 seconds (the server drops
clients that go silent).  `client_id` is the address the client uses for
control traffic, i.e. "<local_ip>:<local_udp_port>".

`camera` is the stream name "<robot>/<camera>" (e.g. "Drone1/FrontCamera"), or a
bare "<camera>" name while only one robot has that camera.  `image_type` is a
name ("scene", "depth_planar", ...) or its numeric value.  Any capture of the
robot configuration with "capture-enabled": true can be subscribed; the GPU
encoder is created on the first SUBSCRIBE and released shortly after the last
client leaves, so nothing is encoded while the simulation is unwatched.

Video channel
-------------
Plain RTP over UDP.  Payload type 96, H.264 (RFC 6184), single NAL unit
packets and FU-A fragmentation.  SPS/PPS are sent ahead of every IDR frame.

Requirements
------------
    # Jetson default: GStreamer with nvv4l2decoder/nvvidconv (hardware decode)
    # Optional software fallback:
    pip install av            # then pass --decoder software
    pip install opencv-python # optional, only needed to open a window
    # Optional ROS publishing also requires ROS1 rospy + sensor_msgs.

Examples
--------
    # Show the stream from FrontCamera (multicast endpoint taken from the
    # server's SUBSCRIBE reply)
    python video_stream_receiver.py --server 192.168.1.20

    # Headless: decode but only print statistics
    python video_stream_receiver.py --server 192.168.1.20 --no-display

    # Publish decoded frames into ROS for sar_yolo_detector
    python video_stream_receiver.py --server 192.168.1.20 \
        --camera Drone1/FrontCamera --no-display \
        --ros-image-topic /uav1/front_camera/image_raw \
        --ros-camera-info-topic /Sim/SceneBasicDrone/robots/Drone1/sensors/FrontCamera/scene_camera/camera_info

    # Explicit endpoint (e.g. when the control reply does not carry one)
    python video_stream_receiver.py --server 192.168.1.20 \
        --group 239.255.42.99 --port 50000

Press Ctrl+C to stop; the client sends UNSUBSCRIBE before exiting.
"""

import argparse
import socket
import struct
import sys
import time

RTP_PAYLOAD_TYPE = 96
MAX_DATAGRAM = 65535
RECV_BUFFER_BYTES = 8 * 1024 * 1024


# ---------------------------------------------------------------------------
# Control channel
# ---------------------------------------------------------------------------


class ControlClient:
    """SUBSCRIBE/HEARTBEAT/UNSUBSCRIBE client for the video control port."""

    def __init__(self, server, port, camera, image_type):
        self.server = server
        self.port = port
        self.camera = camera
        self.image_type = image_type

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)
        self.sock.bind(("", 0))

        # The server keys subscriptions by the client's source address, so
        # work out which local address the server will see.
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((server, port))
            local_ip = probe.getsockname()[0]
        except OSError:
            local_ip = "127.0.0.1"
        finally:
            probe.close()

        self.client_id = "%s:%d" % (local_ip, self.sock.getsockname()[1])

    def _send(self, line):
        try:
            self.sock.sendto((line + "\n").encode("ascii"), (self.server, self.port))
        except OSError as exc:
            print("[ctrl] send failed: %s" % exc, file=sys.stderr)

    def _command(self, verb, with_client=True):
        if with_client:
            self._send("%s %s %s %s" % (verb, self.camera, self.image_type, self.client_id))
        else:
            self._send("%s %s %s" % (verb, self.camera, self.image_type))
        try:
            data, _ = self.sock.recvfrom(2048)
        except socket.timeout:
            return None
        return data.decode("ascii", "replace").strip()

    def subscribe(self):
        """Returns (group, port), None on timeout, or False if rejected."""
        reply = self._command("SUBSCRIBE")
        if reply is None:
            print("[ctrl] no reply to SUBSCRIBE (is the sim running?)", file=sys.stderr)
            return None
        if not reply.startswith("OK"):
            print("[ctrl] SUBSCRIBE rejected: %s" % reply, file=sys.stderr)
            return False

        parts = reply.split()
        if len(parts) >= 3:
            try:
                return parts[1], int(parts[2])
            except ValueError:
                pass
        return "", 0  # server accepted but did not report an endpoint

    def heartbeat(self):
        return self._command("HEARTBEAT")

    def request_keyframe(self):
        self._command("KEYFRAME", with_client=False)

    def unsubscribe(self):
        self._command("UNSUBSCRIBE")

    def close(self):
        self.sock.close()


# ---------------------------------------------------------------------------
# H.264 decoders
# ---------------------------------------------------------------------------


class BgrVideoFrame:
    """Small adapter so GStreamer frames match the existing frame API."""

    def __init__(self, image):
        self.image = image

    def to_ndarray(self, format="bgr24"):
        if format not in ("bgr24", "bgr"):
            raise ValueError("GStreamer frames are available as BGR only")
        return self.image


class GStreamerH264Decoder:
    """Decode Annex-B H.264 with Jetson's NVIDIA V4L2 decoder."""

    def __init__(self):
        try:
            import gi
            gi.require_version("Gst", "1.0")
            gi.require_version("GstVideo", "1.0")
            from gi.repository import Gst, GstVideo
        except Exception as exc:
            raise RuntimeError("PyGObject GStreamer bindings are unavailable: {}".format(
                exc)) from exc

        Gst.init(None)
        self.Gst = Gst
        self.GstVideo = GstVideo
        self.pipeline = Gst.Pipeline.new("jetson-h264-decoder")
        self.appsrc = Gst.ElementFactory.make("appsrc", "h264-input")
        self.parser = Gst.ElementFactory.make("h264parse", "h264-parser")
        self.decoder = Gst.ElementFactory.make("nvv4l2decoder", "h264-hardware-decoder")
        self.converter = Gst.ElementFactory.make("nvvidconv", "video-converter")
        self.output_caps = Gst.ElementFactory.make("capsfilter", "bgrx-caps")
        self.appsink = Gst.ElementFactory.make("appsink", "decoded-frame-output")

        elements = (
            self.appsrc, self.parser, self.decoder, self.converter,
            self.output_caps, self.appsink,
        )
        names = ("appsrc", "h264parse", "nvv4l2decoder", "nvvidconv",
                 "capsfilter", "appsink")
        missing = [name for name, element in zip(names, elements) if element is None]
        if missing:
            raise RuntimeError("missing GStreamer element(s): {}".format(
                ", ".join(missing)))

        self.appsrc.set_property(
            "caps",
            Gst.Caps.from_string(
                "video/x-h264,stream-format=byte-stream,alignment=au"),
        )
        self.appsrc.set_property("format", Gst.Format.TIME)
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("do-timestamp", True)
        self.appsrc.set_property("block", False)
        self.parser.set_property("config-interval", -1)
        self.output_caps.set_property(
            "caps", Gst.Caps.from_string("video/x-raw,format=BGRx"))
        self.appsink.set_property("emit-signals", False)
        self.appsink.set_property("sync", False)
        self.appsink.set_property("max-buffers", 1)
        self.appsink.set_property("drop", True)

        for element in elements:
            self.pipeline.add(element)
        for left, right in zip(elements, elements[1:]):
            if not left.link(right):
                self.pipeline.set_state(Gst.State.NULL)
                raise RuntimeError("could not link GStreamer H.264 decoder pipeline")

        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("could not start nvv4l2decoder pipeline")
        print("[decode] using NVIDIA nvv4l2decoder hardware decoding", flush=True)

    def decode_access_unit(self, data):
        buffer = self.Gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)
        flow = self.appsrc.emit("push-buffer", buffer)
        if flow != self.Gst.FlowReturn.OK:
            raise RuntimeError("GStreamer appsrc rejected H.264 data: {}".format(flow))

        frames = []
        while True:
            sample = self.appsink.emit("try-pull-sample", 0)
            if sample is None:
                break
            frames.append(self._sample_to_frame(sample))
        return frames

    def _sample_to_frame(self, sample):
        info = self.GstVideo.VideoInfo()
        if not info.from_caps(sample.get_caps()):
            raise RuntimeError("could not read decoded video frame dimensions")
        width = int(info.width)
        height = int(info.height)
        stride = int(info.stride[0])
        buffer = sample.get_buffer()
        mapped, map_info = buffer.map(self.Gst.MapFlags.READ)
        if not mapped:
            raise RuntimeError("could not map decoded video frame")
        try:
            import numpy as np
            raw = np.frombuffer(map_info.data, dtype=np.uint8)
            rows = raw[:stride * height].reshape((height, stride))
            bgrx = rows[:, :width * 4].reshape((height, width, 4))
            return BgrVideoFrame(bgrx[:, :, :3].copy())
        finally:
            buffer.unmap(map_info)

    def close(self):
        self.pipeline.set_state(self.Gst.State.NULL)


class GStreamerRtpH264Decoder:
    """Receive RTP, depacketize, and hardware-decode without Python per-packet work."""

    def __init__(self, group, port, local_ip=""):
        try:
            import gi
            gi.require_version("Gst", "1.0")
            gi.require_version("GstVideo", "1.0")
            from gi.repository import Gst, GstVideo
        except Exception as exc:
            raise RuntimeError("GStreamer RTP reception is unavailable: {}".format(exc)) from exc

        Gst.init(None)
        self.Gst = Gst
        self.GstVideo = GstVideo
        self.frames_this_second = 0
        self.decoded_frames = 0
        self.pipeline = Gst.Pipeline.new("jetson-rtp-h264-receiver")
        self.source = Gst.ElementFactory.make("udpsrc", "rtp-udp-source")
        self.jitter = Gst.ElementFactory.make("rtpjitterbuffer", "rtp-jitter-buffer")
        self.depay = Gst.ElementFactory.make("rtph264depay", "h264-depayloader")
        self.parser = Gst.ElementFactory.make("h264parse", "h264-parser")
        self.decoder = Gst.ElementFactory.make("nvv4l2decoder", "h264-hardware-decoder")
        self.converter = Gst.ElementFactory.make("nvvidconv", "video-converter")
        self.output_caps = Gst.ElementFactory.make("capsfilter", "bgrx-caps")
        self.appsink = Gst.ElementFactory.make("appsink", "decoded-frame-output")
        elements = (self.source, self.jitter, self.depay, self.parser,
                    self.decoder, self.converter, self.output_caps, self.appsink)
        names = ("udpsrc", "rtpjitterbuffer", "rtph264depay", "h264parse",
                 "nvv4l2decoder", "nvvidconv", "capsfilter", "appsink")
        missing = [name for name, element in zip(names, elements) if element is None]
        if missing:
            raise RuntimeError("missing GStreamer element(s): {}".format(", ".join(missing)))

        multicast = bool(group and group not in ("0.0.0.0", "unicast"))
        self.source.set_property("port", int(port))
        self.source.set_property("buffer-size", RECV_BUFFER_BYTES)
        if multicast:
            self.source.set_property("address", str(group))
            self.source.set_property("auto-multicast", True)
        else:
            self.source.set_property("address", local_ip or "0.0.0.0")
        self.source.set_property(
            "caps", Gst.Caps.from_string(
                "application/x-rtp,media=video,clock-rate=90000,"
                "encoding-name=H264,payload=96"
            )
        )
        self.jitter.set_property("latency", 0)
        self.jitter.set_property("drop-on-latency", True)
        self.jitter.set_property("do-lost", True)
        self.parser.set_property("config-interval", -1)
        for property_name in ("low-latency-mode", "enable-max-performance"):
            if self.decoder.find_property(property_name) is not None:
                self.decoder.set_property(property_name, True)
        self.output_caps.set_property(
            "caps", Gst.Caps.from_string("video/x-raw,format=BGRx")
        )
        self.appsink.set_property("emit-signals", False)
        self.appsink.set_property("sync", False)
        self.appsink.set_property("max-buffers", 1)
        self.appsink.set_property("drop", True)
        for element in elements:
            self.pipeline.add(element)
        for left, right in zip(elements, elements[1:]):
            if not left.link(right):
                self.pipeline.set_state(Gst.State.NULL)
                raise RuntimeError("could not link GStreamer RTP decode pipeline")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("could not start GStreamer RTP decode pipeline")
        self.bus = self.pipeline.get_bus()
        print("[decode] GStreamer RTP depay + NVIDIA nvv4l2decoder", flush=True)

    def packet_stats(self):
        stats = self.jitter.get_property("stats")
        if stats is None:
            return 0, 0
        return int(stats.get_value("num-pushed")), int(stats.get_value("num-lost"))

    def _sample_to_frame(self, sample):
        info = self.GstVideo.VideoInfo()
        if not info.from_caps(sample.get_caps()):
            raise RuntimeError("could not read decoded video frame dimensions")
        width, height, stride = int(info.width), int(info.height), int(info.stride[0])
        buffer = sample.get_buffer()
        mapped, map_info = buffer.map(self.Gst.MapFlags.READ)
        if not mapped:
            raise RuntimeError("could not map decoded video frame")
        try:
            import numpy as np
            raw = np.frombuffer(map_info.data, dtype=np.uint8)
            rows = raw[:stride * height].reshape((height, stride))
            bgrx = rows[:, :width * 4].reshape((height, width, 4))
            return BgrVideoFrame(bgrx[:, :, :3].copy())
        finally:
            buffer.unmap(map_info)

    def pull_frame(self, timeout_ms=100):
        sample = self.appsink.emit(
            "try-pull-sample", int(timeout_ms) * self.Gst.MSECOND
        )
        error = self.bus.pop_filtered(self.Gst.MessageType.ERROR)
        if error is not None:
            detail, debug = error.parse_error()
            raise RuntimeError("GStreamer RTP pipeline failed: {} ({})".format(detail, debug))
        if sample is None:
            return None
        self.decoded_frames += 1
        self.frames_this_second += 1
        return self._sample_to_frame(sample)

    def close(self):
        self.pipeline.set_state(self.Gst.State.NULL)


def create_h264_decoder(mode):
    if mode == "hardware":
        return GStreamerH264Decoder()

    try:
        import av
    except ImportError as exc:
        raise RuntimeError("software decoding requires PyAV: pip install av") from exc
    try:
        av.logging.set_level(av.logging.ERROR)
    except Exception:  # noqa: BLE001
        pass
    print("[decode] using PyAV software H.264 decoding", flush=True)
    return av.CodecContext.create("h264", "r")


# ---------------------------------------------------------------------------
# RTP / H.264 depacketizer
# ---------------------------------------------------------------------------


class H264Depacketizer:
    """Reassembles RTP H.264 (single NAL + FU-A) into Annex-B access units."""

    def __init__(self, decoder, on_frame, on_corrupt, dump_path=None):
        self.decoder = decoder
        self.on_frame = on_frame
        self.on_corrupt = on_corrupt
        self.dump_path = dump_path

        self.access_unit = bytearray()
        self.fu_buffer = None
        self.last_seq = None
        self.timestamp = 0
        self.received_packets = 0
        self.lost_packets = 0
        self.decoded_frames = 0
        self.frames_this_second = 0

    # -- RTP ---------------------------------------------------------------

    def push(self, packet):
        if len(packet) < 12:
            return
        if (packet[0] >> 6) != 2:  # RTP version must be 2
            return

        csrc_count = packet[0] & 0x0F
        marker = (packet[1] >> 7) & 0x01
        payload_type = packet[1] & 0x7F
        if payload_type != RTP_PAYLOAD_TYPE:
            return

        offset = 12 + 4 * csrc_count
        if packet[0] & 0x10:  # header extension
            if len(packet) < offset + 4:
                return
            extension_words = struct.unpack("!H", packet[offset + 2:offset + 4])[0]
            offset += 4 + 4 * extension_words
        if offset >= len(packet):
            return

        sequence = struct.unpack("!H", packet[2:4])[0]
        timestamp = struct.unpack("!I", packet[4:8])[0]

        self.received_packets += 1
        if self.last_seq is not None and sequence != ((self.last_seq + 1) & 0xFFFF):
            # Packet loss: the current access unit is incomplete, throw it away
            # and ask the server for a fresh IDR.
            self.lost_packets += 1
            self.access_unit = bytearray()
            self.fu_buffer = None
            self.on_corrupt()
        self.last_seq = sequence
        self.timestamp = timestamp

        self._handle_payload(packet[offset:])

        if marker:
            self._flush_access_unit()

    def _handle_payload(self, payload):
        if not payload:
            return
        nal_type = payload[0] & 0x1F

        if nal_type == 28:  # FU-A
            if len(payload) < 2:
                return
            start = payload[1] & 0x80
            end = payload[1] & 0x40
            fu_type = payload[1] & 0x1F
            if start:
                self.fu_buffer = bytearray([(payload[0] & 0xE0) | fu_type])
            if self.fu_buffer is None:
                return
            self.fu_buffer += payload[2:]
            if end:
                self._append_nal(bytes(self.fu_buffer))
                self.fu_buffer = None
        elif 1 <= nal_type <= 23:  # single NAL unit packet
            self._append_nal(bytes(payload))

    def _append_nal(self, nal):
        self.access_unit += b"\x00\x00\x00\x01" + nal

    # -- decoding ----------------------------------------------------------

    def _flush_access_unit(self):
        if not self.access_unit:
            return
        data = bytes(self.access_unit)
        self.access_unit = bytearray()

        if self.dump_path:
            try:
                with open(self.dump_path, "ab") as handle:
                    handle.write(data)
            except OSError:
                pass

        try:
            if hasattr(self.decoder, "decode_access_unit"):
                frames = self.decoder.decode_access_unit(data)
            else:
                frames = []
                for packet in self.decoder.parse(data):
                    frames.extend(self.decoder.decode(packet))
        except Exception as exc:  # noqa: BLE001 - decoder errors mean "resync"
            print("[decode] H.264 decode failed: %s" % exc, file=sys.stderr)
            self.on_corrupt()
            try:
                self.decoder.flush_buffers()
            except Exception:  # noqa: BLE001
                pass
            return

        for frame in frames:
            self.decoded_frames += 1
            self.frames_this_second += 1
            try:
                self.on_frame(frame)
            except Exception as exc:  # noqa: BLE001 - keep callback errors visible
                print("[frame] output callback failed: %s" % exc, file=sys.stderr)


# ---------------------------------------------------------------------------
# Video socket
# ---------------------------------------------------------------------------


def open_video_socket(group, port, local_ip, timeout=0.2):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECV_BUFFER_BYTES)
    except OSError:
        pass

    if group and group not in ("0.0.0.0", "unicast"):
        sock.bind(("", port))
        membership = struct.pack(
            "=4s4s", socket.inet_aton(group), socket.inet_aton("0.0.0.0")
        )
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        print("[video] joined multicast group %s:%d" % (group, port))
    else:
        sock.bind((local_ip or "", port))
        print("[video] listening for unicast on %s:%d" % (local_ip or "0.0.0.0", port))

    sock.settimeout(timeout)
    return sock


# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------


class FrameDisplay:
    def __init__(self, enabled, scale, window_title):
        self.enabled = enabled
        self.scale = scale
        self.window_title = window_title
        self.cv2 = None
        if not enabled:
            return
        try:
            import cv2  # noqa: WPS433 (optional dependency)
        except ImportError:
            print(
                "[display] OpenCV is not installed (pip install opencv-python); "
                "continuing without a window",
                file=sys.stderr,
            )
            return
        self.cv2 = cv2

    def show(self, frame):
        if self.cv2 is None:
            return
        image = frame.to_ndarray(format="bgr24")
        if self.scale != 1.0:
            height, width = image.shape[:2]
            image = self.cv2.resize(
                image, (max(1, int(width * self.scale)), max(1, int(height * self.scale)))
            )
        self.cv2.imshow(self.window_title, image)
        self.cv2.waitKey(1)

    def close(self):
        if self.cv2 is not None:
            try:
                self.cv2.destroyAllWindows()
            except Exception:  # noqa: BLE001
                pass


class RosImagePublisher:
    """Publish decoded images and relay matching CameraInfo into ROS topics."""

    def __init__(self, image_topic, camera_info_topic="",
                 camera_info_output_topic="",
                 node_name="airsim_video_receiver"):
        try:
            import rospy  # noqa: WPS433 (optional dependency)
            from sensor_msgs.msg import CameraInfo, Image  # noqa: WPS433
        except ImportError as exc:
            raise RuntimeError(
                "ROS publishing requires rospy and sensor_msgs in this Python environment"
            ) from exc

        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=True)

        self.rospy = rospy
        self.Image = Image
        self.frame_id = ""
        self.publisher = rospy.Publisher(
            image_topic, Image, queue_size=1, tcp_nodelay=True
        )
        self.camera_info_subscriber = None
        self.camera_info_publisher = None
        if camera_info_topic:
            if camera_info_output_topic:
                self.camera_info_publisher = rospy.Publisher(
                    camera_info_output_topic,
                    CameraInfo,
                    queue_size=1,
                    latch=True,
                )
            self.camera_info_subscriber = rospy.Subscriber(
                camera_info_topic,
                CameraInfo,
                self._camera_info_callback,
                queue_size=1,
            )

        print("[ros] publishing decoded images on %s" % image_topic)
        if camera_info_topic:
            print("[ros] relaying CameraInfo from %s" % camera_info_topic)
        if self.camera_info_publisher is not None:
            print("[ros] publishing matching CameraInfo on %s" % camera_info_output_topic)

    def _camera_info_callback(self, message):
        # Use the source calibration's optical frame for the decoded image.
        if not self.frame_id:
            self.frame_id = message.header.frame_id
        if self.camera_info_publisher is not None:
            self.camera_info_publisher.publish(message)

    def publish(self, frame):
        import numpy as np
        image = np.ascontiguousarray(frame.to_ndarray(format="bgr24"))
        height, width = image.shape[:2]
        message = self.Image()
        message.header.stamp = self.rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.height = height
        message.width = width
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = width * 3
        message.data = image.tobytes(order="C")
        self.publisher.publish(message)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Receive and decode a ProjectAirSim camera video stream.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--server", help="simulator IP address", default="192.168.2.98")
    parser.add_argument("--control-port", type=int, default=49999,
                        help="video control port")
    parser.add_argument("--camera", default="FrontCamera",
                        help='stream name "<robot>/<camera>" or a bare camera id')
    parser.add_argument("--image-type", default="scene",
                        help="image type name or number (e.g. scene, depth_vis, 0)")
    parser.add_argument("--group", default=None,
                        help="override the multicast group from the control reply")
    parser.add_argument("--port", type=int, default=None,
                        help="override the video UDP port from the control reply")
    parser.add_argument("--local-ip", default="",
                        help="bind address for unicast reception")
    parser.add_argument("--no-display", action="store_true",
                        help="decode without opening a window")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="display scale factor")
    parser.add_argument("--decoder", choices=("hardware", "software"),
                        default="hardware",
                        help="H.264 decoder backend (Jetson hardware by default)")
    parser.add_argument("--dump", default=None,
                        help="append the received H.264 Annex-B stream to this file")
    parser.add_argument("--ros-image-topic", default=None,
                        help="publish decoded frames as sensor_msgs/Image on this ROS topic")
    parser.add_argument("--ros-camera-info-topic", default=None,
                        help="source CameraInfo topic; inferred from --camera when qualified")
    parser.add_argument("--ros-camera-info-output-topic", default=None,
                        help="ROS topic for relayed CameraInfo; inferred from --ros-image-topic")
    parser.add_argument("--ros-node-name", default="airsim_video_receiver",
                        help="base name for the optional ROS node")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="reconnect after this many seconds without RTP packets (0 disables inactivity detection)")
    parser.add_argument("--subscribe-retry-sec", type=float, default=2.0,
                        help="wait this many seconds before retrying a timed-out SUBSCRIBE")
    args = parser.parse_args(argv)
    if (args.ros_camera_info_topic or args.ros_camera_info_output_topic) and not args.ros_image_topic:
        parser.error("CameraInfo ROS options require --ros-image-topic")
    if args.subscribe_retry_sec <= 0:
        parser.error("--subscribe-retry-sec must be greater than zero")
    if args.timeout < 0:
        parser.error("--timeout cannot be negative")
    return args


def camera_info_topic_for_camera(camera):
    """Build the ProjectAirSim CameraInfo topic for a qualified camera name."""
    parts = camera.strip("/").split("/", 1)
    if len(parts) != 2 or not all(parts):
        return ""
    robot, camera_name = parts
    return (
        "/Sim/SceneBasicDrone/robots/%s/sensors/%s/scene_camera/camera_info"
        % (robot, camera_name)
    )


def camera_info_output_topic_for_image(image_topic):
    """Return the conventional matching CameraInfo topic for a ROS image topic."""
    topic = image_topic.rstrip("/")
    if topic.endswith("/image_raw"):
        return topic[:-len("image_raw")] + "camera_info"
    if "/" in topic:
        return topic.rsplit("/", 1)[0] + "/camera_info"
    return topic + "/camera_info"


def main(argv=None):
    args = parse_args(argv)

    display = FrameDisplay(not args.no_display, args.scale, "ProjectAirSim " + args.camera)

    ros_publisher = None
    if args.ros_image_topic:
        camera_info_topic = (
            args.ros_camera_info_topic
            or camera_info_topic_for_camera(args.camera)
        )
        camera_info_output_topic = (
            args.ros_camera_info_output_topic
            or camera_info_output_topic_for_image(args.ros_image_topic)
        )
        try:
            ros_publisher = RosImagePublisher(
                image_topic=args.ros_image_topic,
                camera_info_topic=camera_info_topic,
                camera_info_output_topic=camera_info_output_topic,
                node_name=args.ros_node_name,
            )
        except RuntimeError as exc:
            print("[ros] %s" % exc, file=sys.stderr)
            return 2

    def on_frame(frame):
        if ros_publisher is not None:
            ros_publisher.publish(frame)
        display.show(frame)

    retry_delay = args.subscribe_retry_sec
    print("[video] persistent reconnect enabled (retry delay %.1fs)" % retry_delay,
          flush=True)

    try:
        while True:
            control = None
            video = None
            decoder = None
            depacketizer = None
            subscribed = False
            try:
                control = ControlClient(
                    args.server, args.control_port, args.camera, args.image_type)
                print("[ctrl] client id %s" % control.client_id, flush=True)

                while True:
                    endpoint = control.subscribe()
                    if endpoint is not None and endpoint is not False:
                        subscribed = True
                        break
                    print("[ctrl] retrying SUBSCRIBE in %.1fs" % retry_delay,
                          flush=True)
                    time.sleep(retry_delay)

                reply_group, reply_port = endpoint
                group = args.group if args.group is not None else reply_group
                port = args.port if args.port is not None else reply_port
                if not port:
                    raise RuntimeError(
                        "simulator did not report a video endpoint; pass "
                        "--group/--port explicitly")

                # Let GStreamer receive/depacketize RTP directly on the Jetson
                # hot path. Keep the Python socket path for software decode and
                # optional Annex-B dumps.
                native_rtp = args.decoder == "hardware" and not args.dump
                if native_rtp:
                    try:
                        decoder = GStreamerRtpH264Decoder(group, port, args.local_ip)
                    except Exception as exc:
                        print("[decode] GStreamer RTP path unavailable (%s); "
                              "using socket depacketizer" % exc,
                              file=sys.stderr, flush=True)
                        native_rtp = False
                if not native_rtp:
                    video = open_video_socket(group, port, args.local_ip)
                    decoder = create_h264_decoder(args.decoder)

                state = {"last_keyframe_request": 0.0}

                def request_keyframe():
                    now = time.monotonic()
                    if now - state["last_keyframe_request"] < 1.0:
                        return
                    state["last_keyframe_request"] = now
                    control.request_keyframe()

                if not native_rtp:
                    depacketizer = H264Depacketizer(
                        decoder=decoder,
                        on_frame=on_frame,
                        on_corrupt=request_keyframe,
                        dump_path=args.dump,
                    )

                print("[video] receiving H.264 from %s (%s) on UDP %s:%d"
                      % (args.camera, args.image_type, group or "unicast", port),
                      flush=True)

                last_heartbeat = 0.0
                last_packet = time.monotonic()
                last_stats = last_packet
                missed_heartbeats = 0
                last_packet_count = 0
                last_lost_count = 0

                while True:
                    now = time.monotonic()
                    if now - last_heartbeat >= 1.0:
                        last_heartbeat = now
                        heartbeat_reply = control.heartbeat()
                        if heartbeat_reply is None or not heartbeat_reply.startswith("OK"):
                            missed_heartbeats += 1
                            print("[ctrl] HEARTBEAT failed (%d/3): %s"
                                  % (missed_heartbeats, heartbeat_reply),
                                  file=sys.stderr, flush=True)
                            if missed_heartbeats >= 3:
                                raise ConnectionError(
                                    "three consecutive HEARTBEAT failures")
                        else:
                            missed_heartbeats = 0

                    if args.timeout > 0 and now - last_packet > args.timeout:
                        raise TimeoutError(
                            "no RTP packets for %.1fs" % args.timeout)

                    if native_rtp:
                        frame = decoder.pull_frame(timeout_ms=100)
                        packet_count, lost_count = decoder.packet_stats()
                        if packet_count != last_packet_count:
                            last_packet = time.monotonic()
                            last_packet_count = packet_count
                        if lost_count > last_lost_count:
                            request_keyframe()
                            last_lost_count = lost_count
                        if frame is not None:
                            on_frame(frame)
                    else:
                        try:
                            packet, _ = video.recvfrom(MAX_DATAGRAM)
                        except socket.timeout:
                            continue
                        last_packet = time.monotonic()
                        depacketizer.push(packet)

                    if last_packet - last_stats >= 1.0:
                        if native_rtp:
                            fps = decoder.frames_this_second
                            decoder.frames_this_second = 0
                            packets, lost = decoder.packet_stats()
                            frames = decoder.decoded_frames
                        else:
                            fps = depacketizer.frames_this_second
                            depacketizer.frames_this_second = 0
                            packets = depacketizer.received_packets
                            lost = depacketizer.lost_packets
                            frames = depacketizer.decoded_frames
                        last_stats = last_packet
                        print(
                            "[stats] %.1f fps | packets %d | lost %d | frames %d"
                            % (fps, packets, lost, frames),
                            flush=True,
                        )
            except KeyboardInterrupt:
                print("\n[video] stopping")
                break
            except Exception as exc:  # noqa: BLE001 - runtime failures are recoverable
                print("[video] session failed: %s; retrying in %.1fs"
                      % (exc, retry_delay), file=sys.stderr, flush=True)
            finally:
                if control is not None:
                    if subscribed:
                        try:
                            control.unsubscribe()
                        except Exception:
                            pass
                    try:
                        control.close()
                    except Exception:
                        pass
                if video is not None:
                    try:
                        video.close()
                    except Exception:
                        pass
                if decoder is not None:
                    close_decoder = getattr(decoder, "close", None)
                    if close_decoder is not None:
                        try:
                            close_decoder()
                        except Exception:
                            pass

            time.sleep(retry_delay)
    except KeyboardInterrupt:
        print("\n[video] stopping")
    finally:
        display.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
