#!/usr/bin/env python3
"""PNG, frame pairing and bounded delivery without ROS master or GPU."""

from collections import deque
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "python"))
from xd_uav_track.perception_video import LatestPerceptionVideoEncoder


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def is_zero(self):
        return self.seconds == 0

    def to_sec(self):
        return self.seconds

    def __sub__(self, other):
        return Stamp(self.seconds - other.seconds)

    def __eq__(self, other):
        return isinstance(other, Stamp) and self.seconds == other.seconds


def header(seq=91):
    return SimpleNamespace(seq=seq, stamp=Stamp(10), frame_id="fixed_camera_optical")


def snapshot(width=180, height=140):
    target = SimpleNamespace(track_id=7, target_uid="session-a:fixed_rgb:7",
        class_name="car", class_valid=True, selected=True,
        bbox_xyxy=[60, 80, 130, 120], center_px=[95., 100.],
        measurement_source="detector", lifecycle_state="confirmed",
        position_valid=True, position_frame="base_link_frd",
        position_stamp=Stamp(10), position_xyz=[10., 3., -5.],
        position_covariance=[1., 0., 0., 0., 1., 0., 0., 0., 1.])
    return SimpleNamespace(header=header(), frame_seq=7,
        image_source="fixed_rgb", image_width=width, image_height=height,
        session_id="session-a", state_version=13, model_epoch=4,
        model_id="train7", mode="detect", targets=[target])


class Frame:
    def __init__(self):
        self.header = header()
        self.frame_revision = 0
        self.targets = None
        self.image = SimpleNamespace(header=header(), format="", data=b"")


class Publisher:
    def __init__(self, connections=1):
        self.messages = []
        self.connections = connections
        self.condition = threading.Condition()

    def get_num_connections(self):
        return self.connections

    def publish(self, message):
        with self.condition:
            self.messages.append(deepcopy(message))
            self.condition.notify_all()

    def wait(self, count=1):
        with self.condition:
            return self.condition.wait_for(lambda: len(self.messages) >= count, 2.0)


def decoded(message):
    return cv2.imdecode(np.frombuffer(message.image.data, np.uint8), cv2.IMREAD_COLOR)


class VideoTest(unittest.TestCase):
    def encoder(self, publisher, **kwargs):
        encoder = LatestPerceptionVideoEncoder(publisher, frame_factory=Frame, **kwargs)
        self.addCleanup(encoder.close)
        return encoder

    def test_png_preserves_exact_pixels_and_capture_metadata(self):
        image = np.random.RandomState(12).randint(0, 256, (71, 117, 3), np.uint8)
        targets = snapshot(117, 71)
        publisher = Publisher()
        encoder = self.encoder(publisher)
        self.assertTrue(encoder.submit(image, targets))
        self.assertTrue(publisher.wait())
        result = publisher.messages[0]
        np.testing.assert_array_equal(image, decoded(result))
        self.assertEqual((71, 117, 3), decoded(result).shape)
        self.assertEqual("bgr8; png compressed bgr8", result.image.format)
        self.assertEqual(b"\x89PNG\r\n\x1a\n", result.image.data[:8])
        self.assertEqual(7, result.header.seq)
        self.assertEqual(7, result.image.header.seq)
        self.assertEqual(targets.header.stamp, result.image.header.stamp)
        self.assertEqual(targets.header.frame_id, result.header.frame_id)
        self.assertEqual(91, targets.header.seq, "source metadata must stay immutable")
        for field in ("session_id", "state_version", "model_epoch", "model_id", "mode",
                      "image_source", "frame_seq", "image_width", "image_height"):
            self.assertEqual(getattr(targets, field), getattr(result.targets, field))
        self.assertEqual([10., 3., -5.], result.targets.targets[0].position_xyz)

    def test_metadata_is_snapshotted_and_only_latest_frame_waits(self):
        publisher = Publisher()
        encoder = self.encoder(publisher, rate_hz=60)
        started, release = threading.Event(), threading.Event()
        real_encode = cv2.imencode

        def blocking_encode(*args):
            started.set()
            release.wait(2)
            return real_encode(*args)

        self.addCleanup(release.set)
        with patch("xd_uav_track.perception_video.cv2.imencode", side_effect=blocking_encode):
            first_targets = snapshot(20, 20)
            encoder.submit(np.full((20, 20, 3), 1, np.uint8), first_targets)
            self.assertTrue(started.wait(1))
            first_targets.session_id = "mutated-session"
            first_targets.header.seq = 1000
            first_targets.targets[0].position_xyz[0] = 999
            for value in range(2, 101):
                encoder.submit(np.full((20, 20, 3), value, np.uint8), snapshot(20, 20))
            with encoder._condition:
                self.assertEqual(100, encoder._pending[0])
                self.assertEqual(100, encoder._pending[1][0, 0, 0])
            release.set()
            self.assertTrue(publisher.wait(2))
        self.assertEqual([1, 100], [item.frame_revision for item in publisher.messages])
        self.assertEqual("session-a", publisher.messages[0].targets.session_id)
        self.assertEqual(91, publisher.messages[0].targets.header.seq)
        self.assertEqual(10., publisher.messages[0].targets.targets[0].position_xyz[0])
        np.testing.assert_array_equal(np.full((20, 20, 3), 100, np.uint8),
                                      decoded(publisher.messages[1]))

    def test_disconnected_subscriber_never_encodes_or_snapshots(self):
        publisher = Publisher(connections=0)
        encoder = self.encoder(publisher)
        targets = snapshot(20, 20)
        with patch("xd_uav_track.perception_video.cv2.imencode") as encode, \
             patch("xd_uav_track.perception_video.deepcopy") as copy:
            self.assertFalse(encoder.submit(np.zeros((20, 20, 3), np.uint8), targets))
            self.assertIsNone(encoder._pending)
            encode.assert_not_called()
            copy.assert_not_called()

    def test_disconnect_drops_inflight_and_pending(self):
        publisher = Publisher()
        encoder = self.encoder(publisher, rate_hz=60)
        started, release = threading.Event(), threading.Event()
        real_encode = cv2.imencode

        def blocking_encode(*args):
            started.set()
            release.wait(2)
            return real_encode(*args)

        self.addCleanup(release.set)
        with patch("xd_uav_track.perception_video.cv2.imencode", side_effect=blocking_encode) as encode:
            image = np.zeros((20, 20, 3), np.uint8)
            encoder.submit(image, snapshot(20, 20))
            self.assertTrue(started.wait(1))
            encoder.submit(image, snapshot(20, 20))
            publisher.connections = 0
            release.set()
            encoder.close()
        self.assertEqual(1, encode.call_count)
        self.assertEqual([], publisher.messages)
        self.assertIsNone(encoder._pending)

    def test_rate_limit_and_original_size_limits(self):
        publisher, dropped = Publisher(), []
        encoder = self.encoder(publisher, rate_hz=20, max_pixels=400, log_drop=dropped.append)
        encode_times, real_encode = [], cv2.imencode

        def timed_encode(*args):
            encode_times.append(time.monotonic())
            return real_encode(*args)

        with patch("xd_uav_track.perception_video.cv2.imencode", side_effect=timed_encode) as encode:
            self.assertFalse(encoder.submit(np.zeros((21, 20, 3), np.uint8), snapshot(20, 21)))
            self.assertFalse(encoder.submit(np.zeros((20, 20, 3), np.uint8), snapshot(21, 20)))
            encode.assert_not_called()
            image = np.zeros((20, 20, 3), np.uint8)
            encoder.submit(image, snapshot(20, 20))
            self.assertTrue(publisher.wait())
            encoder.submit(image, snapshot(20, 20))
            self.assertTrue(publisher.wait(2))
            self.assertGreaterEqual(encode_times[1] - encode_times[0], .045)
        self.assertEqual(2, len(dropped))
        self.assertEqual((20, 20, 3), decoded(publisher.messages[0]).shape)
        event = threading.Event()
        byte_limited = self.encoder(Publisher(), max_encoded_bytes=1,
                                   log_drop=lambda _: event.set())
        byte_limited.submit(image, snapshot(20, 20))
        self.assertTrue(event.wait(1))
        self.assertEqual([], byte_limited._publisher.messages)

    def test_shutdown_is_bounded_and_does_not_publish_after_stop(self):
        publisher = Publisher()
        encoder = self.encoder(publisher)
        started, release = threading.Event(), threading.Event()
        real_encode = cv2.imencode

        def blocking_encode(*args):
            started.set()
            release.wait(3)
            return real_encode(*args)

        self.addCleanup(release.set)
        with patch("xd_uav_track.perception_video.cv2.imencode", side_effect=blocking_encode):
            image = np.zeros((20, 20, 3), np.uint8)
            encoder.submit(image, snapshot(20, 20))
            self.assertTrue(started.wait(1))
            begin = time.monotonic()
            encoder.close()
            self.assertLess(time.monotonic() - begin, 1.5)
            self.assertFalse(encoder.submit(image, snapshot(20, 20)))
            release.set()
            encoder._thread.join(1)
        self.assertFalse(encoder._thread.is_alive())
        self.assertEqual([], publisher.messages)

    def test_bad_encoding_recovers_without_killing_worker(self):
        publisher, error = Publisher(), threading.Event()
        encoder = self.encoder(publisher, rate_hz=60, log_drop=lambda _: error.set())
        with patch("xd_uav_track.perception_video.cv2.imencode", side_effect=RuntimeError("bad codec")):
            encoder.submit(np.zeros((20, 20, 3), np.uint8), snapshot(20, 20))
            self.assertTrue(error.wait(1))
        encoder.submit(np.ones((20, 20, 3), np.uint8), snapshot(20, 20))
        self.assertTrue(publisher.wait())


class Bridge:
    def imgmsg_to_cv2(self, message, desired_encoding):
        return message.pixels

    def cv2_to_imgmsg(self, pixels, encoding):
        return SimpleNamespace(header=header(), pixels=pixels.copy(),
                               width=pixels.shape[1], height=pixels.shape[0])


def load_overlay(params=None):
    rospy = ModuleType("rospy")
    rospy.get_param = lambda key, default=None: (params or {}).get(key, default)
    rospy.publishers, rospy.shutdown_callbacks = [], []

    def make_publisher(topic, message_type, **kwargs):
        publisher = Publisher(connections=0)
        publisher.topic, publisher.message_type, publisher.kwargs = topic, message_type, kwargs
        rospy.publishers.append(publisher)
        return publisher

    rospy.Publisher = make_publisher
    rospy.Subscriber = lambda *args, **kwargs: SimpleNamespace(args=args, kwargs=kwargs)
    rospy.logwarn_throttle = lambda *args: None
    rospy.on_shutdown = rospy.shutdown_callbacks.append
    messages = ModuleType("xd_uav_track.msg")
    messages.PerceptionTargetArray, messages.PerceptionVideoFrame = SimpleNamespace, Frame
    cv_bridge = ModuleType("cv_bridge")
    cv_bridge.CvBridge, cv_bridge.CvBridgeError = Bridge, RuntimeError
    sensor_messages = ModuleType("sensor_msgs.msg")
    sensor_messages.Image = SimpleNamespace
    sensor = ModuleType("sensor_msgs")
    sensor.msg = sensor_messages
    with patch.dict(sys.modules, {"rospy": rospy, "cv_bridge": cv_bridge,
            "xd_uav_track.msg": messages, "sensor_msgs": sensor,
            "sensor_msgs.msg": sensor_messages}):
        spec = importlib.util.spec_from_file_location("overlay_video_fixture",
                                                    str(PACKAGE / "scripts/perception_overlay_node.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module, module.PerceptionOverlay() if params else None


class OverlayVideoTest(unittest.TestCase):
    def test_publication_configuration_and_disable_switch(self):
        module, overlay = load_overlay({"~image_source": "fixed_rgb"})
        self.addCleanup(overlay._video_encoder.close)
        publishers = module.rospy.publishers
        self.assertEqual(2, len(publishers))
        self.assertEqual("perception/video", publishers[1].topic)
        self.assertEqual({"queue_size": 1, "latch": False}, publishers[1].kwargs)
        self.assertEqual([overlay._video_encoder.close], module.rospy.shutdown_callbacks)
        module, overlay = load_overlay({"~image_source": "fixed_rgb", "~enable_web_video": False})
        self.assertIsNone(overlay._video_encoder)
        self.assertEqual(1, len(module.rospy.publishers))

    def test_newer_update_supersedes_older_inflight_drawing(self):
        module, _ = load_overlay()
        overlay = module.PerceptionOverlay.__new__(module.PerceptionOverlay)
        overlay._source = "fixed_rgb"
        overlay._lock, overlay._bridge = threading.Lock(), Bridge()
        overlay._publisher = Publisher()
        older, newer = snapshot(), snapshot()
        newer.targets = []
        overlay._pending = older
        camera = SimpleNamespace(header=header(seq=7),
            pixels=np.zeros((140, 180, 3), np.uint8), width=180, height=140)
        overlay._matching_image = lambda _: camera
        real_decode = overlay._bridge.imgmsg_to_cv2

        def newer_arrives(*args, **kwargs):
            overlay._pending = newer
            return real_decode(*args, **kwargs)

        overlay._bridge.imgmsg_to_cv2 = newer_arrives
        overlay._try_publish(older)
        self.assertEqual([], overlay._publisher.messages)
        self.assertIs(newer, overlay._pending)

    def test_matching_requires_exact_capture_and_agrees_when_sequences_known(self):
        module, _ = load_overlay()
        overlay = module.PerceptionOverlay.__new__(module.PerceptionOverlay)
        overlay._lock = threading.Lock()
        targets = snapshot()
        camera = SimpleNamespace(header=header(seq=8))
        overlay._images = deque([camera])
        self.assertIsNone(overlay._matching_image(targets),
                          "known sequence mismatch cannot use timestamp fallback")
        camera.header.seq = 0
        self.assertIs(camera, overlay._matching_image(targets),
                      "a sequence-less driver can match an exact capture stamp")
        camera.header.stamp = Stamp(10.0005)
        self.assertIsNone(overlay._matching_image(targets),
                          "a sub-millisecond neighbour is not the referenced frame")
        targets.frame_seq = 0
        camera.header.seq, camera.header.stamp = 8, Stamp(10)
        self.assertIs(camera, overlay._matching_image(targets),
                      "unknown target sequence still requires an exact stamp")
        targets.header.stamp = camera.header.stamp = Stamp(0)
        self.assertIsNone(overlay._matching_image(targets))

    def test_targets_wait_for_correct_image_instead_of_encoding_neighbour(self):
        module, _ = load_overlay()
        overlay = module.PerceptionOverlay.__new__(module.PerceptionOverlay)
        overlay._source = "fixed_rgb"
        overlay._lock, overlay._bridge = threading.Lock(), Bridge()
        overlay._publisher = Publisher()
        video_publisher = Publisher()
        overlay._video_encoder = LatestPerceptionVideoEncoder(
            video_publisher, rate_hz=60, frame_factory=Frame)
        self.addCleanup(overlay._video_encoder.close)
        previous = SimpleNamespace(header=header(seq=6),
            pixels=np.zeros((140, 180, 3), np.uint8), width=180, height=140)
        overlay._images, overlay._pending = deque([previous]), None
        targets = snapshot()
        targets.header.stamp = Stamp(10.0005)
        with patch("xd_uav_track.perception_video.cv2.imencode", wraps=cv2.imencode) as encode:
            overlay._target_callback(targets)
            self.assertEqual([], overlay._publisher.messages)
            self.assertEqual([], video_publisher.messages)
            self.assertIs(targets, overlay._pending)
            encode.assert_not_called()
            correct = deepcopy(previous)
            correct.header.seq, correct.header.stamp = 7, Stamp(10.0005)
            correct.pixels.fill(255)
            overlay._image_callback(correct)
            self.assertTrue(video_publisher.wait())
        result = video_publisher.messages[0]
        self.assertEqual(7, result.targets.frame_seq)
        self.assertEqual(Stamp(10.0005), result.targets.header.stamp)
        # Compare untouched pixels: the class label above y=80 occupies y=60.
        np.testing.assert_array_equal(correct.pixels[130:140], decoded(result)[130:140])

    def test_same_capture_lost_update_clears_boxes_and_keeps_native_pair(self):
        module, _ = load_overlay()
        overlay = module.PerceptionOverlay.__new__(module.PerceptionOverlay)
        overlay._source, overlay._maximum_delta = "fixed_rgb", .02
        overlay._lock, overlay._bridge = threading.Lock(), Bridge()
        overlay._publisher = Publisher()
        video_publisher = Publisher()
        overlay._video_encoder = LatestPerceptionVideoEncoder(
            video_publisher, rate_hz=60, frame_factory=Frame)
        self.addCleanup(overlay._video_encoder.close)
        original = np.random.RandomState(3).randint(0, 256, (140, 180, 3), np.uint8)
        camera = SimpleNamespace(header=header(seq=0), pixels=original,
                                 width=180, height=140)
        overlay._images, overlay._pending = deque([camera]), None
        targets = snapshot()
        overlay._target_callback(targets)
        self.assertTrue(video_publisher.wait())
        detected_frame = video_publisher.messages[0]
        lost = deepcopy(targets)
        lost.targets[0].measurement_source, lost.targets[0].lifecycle_state = "stale", "lost"
        lost.targets[0].position_valid = False
        lost.targets[0].bbox_xyxy = [0, 0, 0, 0]
        lost.state_version += 1
        overlay._target_callback(lost)
        self.assertTrue(video_publisher.wait(2))
        lost_frame = video_publisher.messages[1]
        self.assertGreater(lost_frame.frame_revision, detected_frame.frame_revision)
        self.assertEqual(detected_frame.header.stamp, lost_frame.header.stamp)
        self.assertEqual(7, lost_frame.header.seq)
        self.assertEqual(0, camera.header.seq, "raw camera header must not be mutated")
        np.testing.assert_array_equal(original, camera.pixels)
        np.testing.assert_array_equal(original[60:], decoded(lost_frame)[60:])
        self.assertFalse(np.array_equal(original[60:], decoded(detected_frame)[60:]))
        self.assertEqual("stale", lost_frame.targets.targets[0].measurement_source)
        self.assertFalse(lost_frame.targets.targets[0].position_valid)
        self.assertEqual((140, 180, 3), decoded(lost_frame).shape)
        np.testing.assert_array_equal(overlay._publisher.messages[-1].pixels,
                                      decoded(lost_frame))


if __name__ == "__main__":
    unittest.main()
