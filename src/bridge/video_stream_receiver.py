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
    pip install av            # PyAV, decodes H.264
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
        """Returns (group, port) on success, None on failure."""
        reply = self._command("SUBSCRIBE")
        if reply is None:
            print("[ctrl] no reply to SUBSCRIBE (is the sim running?)", file=sys.stderr)
            return None
        if not reply.startswith("OK"):
            print("[ctrl] SUBSCRIBE rejected: %s" % reply, file=sys.stderr)
            return None

        parts = reply.split()
        if len(parts) >= 3:
            try:
                return parts[1], int(parts[2])
            except ValueError:
                pass
        return "", 0  # server accepted but did not report an endpoint

    def heartbeat(self):
        self._command("HEARTBEAT")

    def request_keyframe(self):
        self._command("KEYFRAME", with_client=False)

    def unsubscribe(self):
        self._command("UNSUBSCRIBE")

    def close(self):
        self.sock.close()


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
            for packet in self.decoder.parse(data):
                for frame in self.decoder.decode(packet):
                    self.decoded_frames += 1
                    self.frames_this_second += 1
                    self.on_frame(frame)
        except Exception:  # noqa: BLE001 - any decoder error means "resync"
            self.on_corrupt()
            try:
                self.decoder.flush_buffers()
            except Exception:  # noqa: BLE001
                pass


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
        image = frame.to_ndarray(format="bgr24")
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
    parser.add_argument("--timeout", type=float, default=0.0,
                        help="exit after this many seconds without packets (0 = never)")
    args = parser.parse_args(argv)
    if (args.ros_camera_info_topic or args.ros_camera_info_output_topic) and not args.ros_image_topic:
        parser.error("CameraInfo ROS options require --ros-image-topic")
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

    try:
        import av  # noqa: WPS433 (optional dependency, checked late for a clean error)
    except ImportError:
        print(
            "PyAV is required to decode H.264: pip install av",
            file=sys.stderr,
        )
        return 2

    try:
        av.logging.set_level(av.logging.ERROR)
    except Exception:  # noqa: BLE001
        pass

    decoder = av.CodecContext.create("h264", "r")
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

    control = ControlClient(args.server, args.control_port, args.camera, args.image_type)
    print("[ctrl] client id %s" % control.client_id)

    endpoint = control.subscribe()
    if endpoint is None:
        control.close()
        return 1

    reply_group, reply_port = endpoint
    group = args.group if args.group is not None else reply_group
    port = args.port if args.port is not None else reply_port

    if not port:
        print(
            "[ctrl] the simulator did not report a video endpoint; pass "
            "--group/--port explicitly",
            file=sys.stderr,
        )
        control.unsubscribe()
        control.close()
        return 1

    video = open_video_socket(group, port, args.local_ip)

    state = {"last_keyframe_request": 0.0}

    def request_keyframe():
        now = time.time()
        if now - state["last_keyframe_request"] < 1.0:
            return
        state["last_keyframe_request"] = now
        control.request_keyframe()

    depacketizer = H264Depacketizer(
        decoder=decoder,
        on_frame=on_frame,
        on_corrupt=request_keyframe,
        dump_path=args.dump,
    )

    print("[video] waiting for H.264 packets from %s (%s)..."
          % (args.camera, args.image_type))

    last_heartbeat = 0.0
    last_packet = time.time()
    last_stats = time.time()

    try:
        while True:
            now = time.time()

            if now - last_heartbeat >= 1.0:
                last_heartbeat = now
                control.heartbeat()

            if args.timeout > 0 and (now - last_packet) > args.timeout:
                print("[video] no packets for %.1fs, giving up" % args.timeout,
                      file=sys.stderr)
                break

            try:
                packet, _ = video.recvfrom(MAX_DATAGRAM)
            except socket.timeout:
                continue

            last_packet = time.time()
            depacketizer.push(packet)

            if time.time() - last_stats >= 1.0:
                fps = depacketizer.frames_this_second
                depacketizer.frames_this_second = 0
                last_stats = time.time()
                print(
                    "[stats] %.1f fps | packets %d | lost %d | frames %d"
                    % (
                        fps,
                        depacketizer.received_packets,
                        depacketizer.lost_packets,
                        depacketizer.decoded_frames,
                    ),
                    flush=True,
                )
    except KeyboardInterrupt:
        print("\n[video] stopping")
    finally:
        control.unsubscribe()
        control.close()
        video.close()
        display.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
