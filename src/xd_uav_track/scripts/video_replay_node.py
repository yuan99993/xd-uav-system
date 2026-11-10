#!/usr/bin/env python3
"""Paced, bounded-memory one/two-source image replay for perception tests.

Inputs may be video files or directories of naturally numbered image frames.
Two inputs share one publish clock; frame n of each source has exactly the same
ROS capture stamp. This establishes transport synchronization, not camera
calibration or a claim that the two views depict the same physical instant.
"""

import glob
import os
import re
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from xd_uav_track.msg import TrackStateArray


def _natural_key(path):
    return [int(part) if part.isdigit() else part.lower()
            for part in re.split(r"(\d+)", os.path.basename(path))]


class FrameReader:
    def __init__(self, path, start_frame=0):
        if not path or not os.path.exists(path):
            raise ValueError("video/image directory does not exist: %r" % path)
        self.path = path
        self.capture = None
        self.files = None
        self.index = max(0, int(start_frame))
        if os.path.isdir(path):
            self.files = sorted((name for name in glob.glob(os.path.join(path, "*"))
                                 if name.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))),
                                key=_natural_key)
            if not self.files or self.index >= len(self.files):
                raise ValueError("no readable image frames at requested start: %s" % path)
        else:
            self.capture = cv2.VideoCapture(path)
            if not self.capture.isOpened():
                raise ValueError("cannot open video: %s" % path)
            if self.index:
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, self.index)

    def read(self, step=1):
        if self.files is not None:
            if self.index >= len(self.files):
                return None
            frame = cv2.imread(self.files[self.index], cv2.IMREAD_COLOR)
            self.index += step
            return frame
        ok, frame = self.capture.read()
        if ok:
            for _ in range(step - 1):
                if not self.capture.grab():
                    break
            self.index += step
        return frame if ok else None

    def close(self):
        if self.capture is not None:
            self.capture.release()


def main():
    rospy.init_node("video_replay")
    path_a = rospy.get_param("~video_a", "")
    path_b = rospy.get_param("~video_b", "")
    fps = float(rospy.get_param("~fps", 8.0))
    if not 0.1 <= fps <= 60.0:
        raise ValueError("fps must be in [0.1, 60]")
    max_frames = max(0, int(rospy.get_param("~max_frames", 0)))
    max_width = max(0, int(rospy.get_param("~max_width", 1280)))
    frame_step = max(1, int(rospy.get_param("~frame_step", 1)))
    lockstep = bool(rospy.get_param("~lockstep", False))
    source_fps = float(rospy.get_param("~source_fps", 30.0))
    if source_fps <= 0:
        raise ValueError("source_fps must be positive")
    track_timeout = max(0.5, float(rospy.get_param("~track_timeout_sec", 10.0)))
    source_names = [rospy.get_param("~source_a", "replay_fixed"),
                    rospy.get_param("~source_b", "replay_gimbal")]
    readers = [FrameReader(path_a, rospy.get_param("~start_frame_a", 0))]
    if path_b:
        try:
            readers.append(FrameReader(path_b, rospy.get_param("~start_frame_b", 0)))
        except Exception:
            readers[0].close()
            raise
    topics = [rospy.get_param("~topic_a", "fixed_camera/image_raw"),
              rospy.get_param("~topic_b", "gimbal_camera/image_raw")]
    frames = [rospy.get_param("~frame_a", "replay_fixed_camera"),
              rospy.get_param("~frame_b", "replay_gimbal_camera")]
    publishers = [rospy.Publisher(topics[i], Image, queue_size=1)
                  for i in range(len(readers))]
    marker_publishers = [rospy.Publisher(
        "fixed_camera/frame_marker" if i == 0 else "gimbal_camera/frame_marker",
        Header, queue_size=1) for i in range(len(readers))]
    detector_ready = threading.Event()
    detector_error = [""]
    processed_frames = [0]
    expected_diagnostic = "/%s/sar_yolo_multi_source_detection: shared_yolo" % (
        rospy.get_namespace().strip("/"))

    def on_diagnostic(message):
        for status in message.status:
            if status.name != expected_diagnostic:
                continue
            values = {item.key: item.value for item in status.values}
            detector_error[0] = values.get("startup_error", "")
            processed_frames[0] = int(values.get("processed_frames", "0"))
            if values.get("startup_ready", "").lower() == "true":
                detector_ready.set()

    diagnostic_subscriber = rospy.Subscriber("/diagnostics", DiagnosticArray,
                                             on_diagnostic, queue_size=2)
    tracks_condition = threading.Condition()
    completed_tracks = set()

    def on_tracks(message):
        with tracks_condition:
            completed_tracks.add((message.image_source, message.header.stamp.to_nsec()))
            tracks_condition.notify_all()

    tracks_subscriber = rospy.Subscriber("track/tracks_by_source", TrackStateArray,
                                         on_tracks, queue_size=8) if lockstep else None
    bridge = CvBridge()
    period = 1.0 / fps
    published = 0
    try:
        # Allow subscribers to connect without caching an unbounded video backlog.
        deadline = time.monotonic() + max(0.0, float(rospy.get_param(
            "~startup_wait_sec", 90.0)))
        while not rospy.is_shutdown() and time.monotonic() < deadline and \
                (not detector_ready.is_set() or
                 any(pub.get_num_connections() == 0 for pub in publishers)):
            if detector_error[0]:
                raise RuntimeError("detector startup failed: " + detector_error[0])
            time.sleep(0.05)
        if not rospy.is_shutdown() and not detector_ready.is_set():
            raise RuntimeError("detector did not report startup_ready within timeout")
        if not rospy.is_shutdown() and any(pub.get_num_connections() == 0
                                           for pub in publishers):
            raise RuntimeError("video replay image subscriber did not connect")
        baseline_processed = processed_frames[0]
        next_frame_wall = time.monotonic()
        source_first_frame = None
        source_base_stamp = time.time()
        while not rospy.is_shutdown() and (not max_frames or published < max_frames):
            delay = next_frame_wall - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            original_frames = [reader.index + 1 for reader in readers]
            if source_first_frame is None:
                source_first_frame = original_frames[0]
            images = [reader.read(frame_step) for reader in readers]
            if any(image is None for image in images):
                break  # Paired replay ends with the shorter input; no stale repeats.
            if max_width:
                images = [cv2.resize(image, (max_width, max(1, round(
                    image.shape[0] * max_width / image.shape[1]))),
                    interpolation=cv2.INTER_AREA) if image.shape[1] > max_width
                    else image for image in images]
            stamp = rospy.Time.from_sec(
                source_base_stamp + (original_frames[0] - source_first_frame) / source_fps
                if lockstep else time.time())
            for index, image in enumerate(images):
                msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
                msg.header.stamp = stamp
                msg.header.seq = published
                msg.header.frame_id = frames[index]
                publishers[index].publish(msg)
                marker = Header()
                marker.stamp = stamp
                marker.frame_id = str(original_frames[index])
                marker_publishers[index].publish(marker)
            published += 1
            if lockstep:
                expected = {(source_names[index], stamp.to_nsec())
                            for index in range(len(readers))}
                with tracks_condition:
                    deadline = time.monotonic() + track_timeout
                    while not rospy.is_shutdown() and not expected.issubset(completed_tracks):
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            raise RuntimeError("tracker did not finish source frame %s" %
                                               original_frames)
                        tracks_condition.wait(timeout=min(0.2, remaining))
                    completed_tracks.difference_update(expected)
            next_frame_wall = (time.monotonic() + period if lockstep else
                               max(next_frame_wall + period, time.monotonic()))
        # The detector is intentionally latest-only. Give the final inference
        # and tracker callbacks a bounded chance to publish before required
        # replay-node exit shuts down the launch and closes the recorder.
        drain_deadline = time.monotonic() + max(0.0, float(rospy.get_param(
            "~drain_timeout_sec", 20.0)))
        while not rospy.is_shutdown() and time.monotonic() < drain_deadline and \
                processed_frames[0] <= baseline_processed:
            time.sleep(0.05)
        if processed_frames[0] > baseline_processed:
            time.sleep(0.5)
    finally:
        for reader in readers:
            reader.close()
        rospy.loginfo("video replay finished: paired frames=%d sources=%d", published, len(readers))


if __name__ == "__main__":
    main()
