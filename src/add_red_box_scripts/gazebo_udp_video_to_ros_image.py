#!/usr/bin/env python3
"""Publish the Typhoon H480 Gazebo RTP/H264 stream as a ROS Image topic."""

import argparse
import logging
import os
import signal
import subprocess
import tempfile
import time
from typing import Optional

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header


LOGGER = logging.getLogger("gazebo_udp_video_to_ros_image")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge Gazebo/PX4 RTP H264 video to a ROS Image topic."
    )
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5600)
    parser.add_argument("--payload-type", type=int, default=96)
    parser.add_argument("--topic", default="/uav1/cgo3_camera/image_raw")
    parser.add_argument("--frame-id", default="uav1/cgo3_camera_link")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--queue-size", type=int, default=1)
    parser.add_argument(
        "--ros-node-name", default="typhoon_udp_video_to_ros_image"
    )
    parser.add_argument("--ffmpeg-bin", default="ffmpeg")
    parser.add_argument("--restart-delay", type=float, default=1.0)
    return parser.parse_args(rospy.myargv()[1:])


def make_sdp(host: str, port: int, payload_type: int) -> str:
    return (
        "v=0\n"
        f"o=- 0 0 IN IP4 {host}\n"
        "s=Gazebo PX4 Camera\n"
        f"c=IN IP4 {host}\n"
        "t=0 0\n"
        f"m=video {port} RTP/AVP {payload_type}\n"
        f"a=rtpmap:{payload_type} H264/90000\n"
    )


def start_ffmpeg(args: argparse.Namespace, sdp_path: str) -> subprocess.Popen:
    filters = f"fps={args.fps},scale={args.width}:{args.height}"
    command = [
        args.ffmpeg_bin,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-nostdin",
        "-protocol_whitelist",
        "file,udp,rtp",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-f",
        "sdp",
        "-i",
        sdp_path,
        "-an",
        "-vf",
        filters,
        "-pix_fmt",
        "bgr24",
        "-f",
        "rawvideo",
        "pipe:1",
    ]
    LOGGER.info("Starting ffmpeg for UDP port %d", args.udp_port)
    return subprocess.Popen(command, stdout=subprocess.PIPE)


def read_exact(pipe, size: int) -> Optional[bytes]:
    chunks = []
    remaining = size
    while remaining > 0 and not rospy.is_shutdown():
        chunk = pipe.read(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def publish_loop(args: argparse.Namespace, sdp_path: str) -> None:
    publisher = rospy.Publisher(args.topic, Image, queue_size=args.queue_size)
    frame_size = args.width * args.height * 3
    sequence = 0
    while not rospy.is_shutdown():
        process = start_ffmpeg(args, sdp_path)
        try:
            if process.stdout is None:
                raise RuntimeError("ffmpeg stdout pipe was not created")
            while not rospy.is_shutdown():
                frame = read_exact(process.stdout, frame_size)
                if frame is None:
                    LOGGER.warning("ffmpeg stream ended; restarting")
                    break
                message = Image()
                message.header = Header(
                    seq=sequence, stamp=rospy.Time.now(), frame_id=args.frame_id
                )
                message.height = args.height
                message.width = args.width
                message.encoding = "bgr8"
                message.is_bigendian = 0
                message.step = args.width * 3
                message.data = frame
                publisher.publish(message)
                sequence += 1
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
            if not rospy.is_shutdown():
                time.sleep(max(args.restart_delay, 0.1))


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = parse_args()
    rospy.init_node(args.ros_node_name, anonymous=True, disable_signals=True)
    with tempfile.NamedTemporaryFile("w", suffix=".sdp", delete=False) as sdp_file:
        sdp_file.write(make_sdp(args.udp_host, args.udp_port, args.payload_type))
        sdp_path = sdp_file.name

    def shutdown(_signum, _frame) -> None:
        rospy.signal_shutdown("signal received")

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    LOGGER.info(
        "Typhoon UDP %s:%d -> %s (%dx%d)",
        args.udp_host,
        args.udp_port,
        args.topic,
        args.width,
        args.height,
    )
    try:
        publish_loop(args, sdp_path)
    finally:
        try:
            os.unlink(sdp_path)
        except OSError:
            pass


if __name__ == "__main__":
    main()
