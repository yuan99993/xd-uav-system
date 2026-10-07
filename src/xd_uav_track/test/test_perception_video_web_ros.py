#!/usr/bin/env python3
"""Native PNG, frame-bound selections and source loss through real rosbridge.

Synthetic inputs feed the actual C++ identity manager, perception manager and
overlay nodes. Model READY describes the synthetic inputs only: no engine is
loaded, and a model-switch request must reject as UNAVAILABLE.
"""

import asyncio
import base64
from collections import OrderedDict, deque
import json
import threading
import time
import unittest
from urllib.request import urlopen

import cv2
import numpy as np
import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tornado.websocket import websocket_connect
from xd_uav_track.msg import DetectionArray, DetectionCandidate


class VideoWebContractTest(unittest.TestCase):
    ROOT = "/web_video_test/perception/"
    NAMESPACE = "/web_video_test/"
    WIDTH, HEIGHT = 320, 240
    BOX = (96, 92, 144, 124)
    SOURCES = ("fixed_rgb", "gimbal_rgb")

    def setUp(self):
        if not rospy.core.is_initialized():
            self.skipTest("run with rostest perception_video_web.test")
        self.stopped = threading.Event()
        self.frame_lock = threading.RLock()
        self.frozen_sources = set()
        self.last_stamps = {}
        self.originals = OrderedDict()
        self.packets = deque(maxlen=96)
        self.request_number = 0
        self.sequence = 0
        self.metric_ready_wall = time.monotonic() + 2.0
        self.worker_error = None
        self.images = {
            source: rospy.Publisher(self.NAMESPACE + name + "_camera/image_raw",
                                    Image, queue_size=1)
            for source, name in zip(self.SOURCES, ("fixed", "gimbal"))
        }
        self.detections = rospy.Publisher(self.NAMESPACE + "detect/detections",
                                          DetectionArray, queue_size=2)
        self.odom = rospy.Publisher(self.NAMESPACE + "state_estimator/main/odom",
                                   Odometry, queue_size=1)
        self.model_status = rospy.Publisher(
            self.NAMESPACE + "sar_yolo_multi_source_detection/model_status",
            DiagnosticArray, queue_size=1, latch=True)
        self.worker = threading.Thread(target=self.frames,
                                       name="synthetic_video_frames", daemon=True)
        self.worker.start()
        self.addCleanup(self.stop)

    def stop(self):
        self.stopped.set()
        self.worker.join(timeout=2)
        for publisher in list(self.images.values()) + [
                self.detections, self.odom, self.model_status]:
            publisher.unregister()

    @staticmethod
    def stamp_key(stamp):
        if isinstance(stamp, dict):
            return int(stamp["secs"]), int(stamp["nsecs"])
        return stamp.secs, stamp.nsecs

    def frames(self):
        try:
            texture = np.random.RandomState(43).randint(
                20, 220, (self.BOX[3] - self.BOX[1],
                          self.BOX[2] - self.BOX[0], 3)).astype(np.uint8)
            while not self.stopped.is_set() and not rospy.is_shutdown():
                started = time.monotonic()
                status = DiagnosticArray()
                status.header.stamp = rospy.Time.now()
                status.status = [DiagnosticStatus(
                    name="synthetic_input_contract", level=DiagnosticStatus.OK,
                    message="Synthetic DetectionArray inputs; no detector engine",
                    values=[KeyValue(key=key, value=value) for key, value in (
                        ("state", "READY"), ("active_model_id", "synthetic-input"),
                        ("model_epoch", "1"), ("backend", "synthetic-input"),
                        ("class_names", '["car","ar-car","tank","m142"]'))])]
                self.model_status.publish(status)
                for source_index, source in enumerate(self.SOURCES):
                    # Holding the lock through publication lets freeze_source()
                    # return the precise last capture, with no in-flight frame.
                    with self.frame_lock:
                        if source in self.frozen_sources:
                            continue
                        self.sequence += 1
                        stamp = rospy.Time.now()
                        image = np.empty((self.HEIGHT, self.WIDTH, 3), np.uint8)
                        image[:] = (17 + source_index, 23, 31)
                        x1, y1, x2, y2 = self.BOX
                        image[y1:y2, x1:x2] = texture
                        image[220:232, 272:288] = (self.sequence % 251, 72, 180)
                        message = Image()
                        message.header.seq = self.sequence
                        message.header.stamp = stamp
                        message.header.frame_id = source + "_optical"
                        message.width, message.height = self.WIDTH, self.HEIGHT
                        message.encoding = "bgr8"
                        message.step = self.WIDTH * 3
                        message.data = image.tobytes()
                        self.originals[(source, self.stamp_key(stamp))] = image
                        self.last_stamps[source] = self.stamp_key(stamp)
                        while len(self.originals) > 48:
                            self.originals.popitem(last=False)
                        self.images[source].publish(message)
                        odom = Odometry()
                        odom.header.stamp = stamp
                        odom.header.frame_id = "world"
                        odom.pose.pose.orientation.w = 1.0
                        self.odom.publish(odom)
                        # Real inference also arrives after image capture.
                        self.stopped.wait(0.025)
                        candidate = DetectionCandidate(
                            track_id=-1, class_id=2, confidence=0.95,
                            has_bbox=True, bbox=list(self.BOX),
                            has_normalized_bbox=True,
                            normalized_bbox=[120 / self.WIDTH, 108 / self.HEIGHT,
                                             48 / self.WIDTH, 32 / self.HEIGHT],
                            appearance_embedding=[1.0, 0.0, 0.0, 0.0],
                            has_relative_position_body=True, range_valid=True,
                            relative_position_body=[10.0, 2.0, -5.0],
                            position_covariance=[0.25, 0, 0, 0, 0.25, 0, 0, 0, 0.25])
                        if source == "gimbal_rgb" and time.monotonic() < self.metric_ready_wall:
                            candidate.has_relative_position_body = False
                            candidate.range_valid = False
                        detections = DetectionArray(
                            image_source=source, image_width=self.WIDTH,
                            image_height=self.HEIGHT, candidates=[candidate],
                            model_epoch=1,
                            class_names=["car", "ar-car", "tank", "m142"])
                        detections.header = message.header
                        self.detections.publish(detections)
                self.stopped.wait(max(0.0, 0.125 - (time.monotonic() - started)))
        except Exception as error:
            self.worker_error = error
            self.stopped.set()

    async def freeze_displayed_source(self, socket, source, uid, state_version):
        # A latest-only transport may coalesce captures. Freeze a frame that
        # actually reached the browser while it is still the camera's latest,
        # instead of requiring an arbitrary final publication to be delivered.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            frame = await self.video(socket, source, lambda f:
                f["targets"]["state_version"] >= state_version and any(
                    t["target_uid"] == uid and t["selected"]
                    and t["lifecycle_state"] == "confirmed"
                    for t in f["targets"]["targets"]),
                seconds=max(0.01, deadline - time.monotonic()))
            stamp = self.stamp_key(frame["targets"]["header"]["stamp"])
            with self.frame_lock:
                if self.last_stamps[source] == stamp:
                    self.frozen_sources.add(source)
                    rospy.loginfo("VIDEO_WEB_FREEZE_DISPLAYED source=%s stamp=%s revision=%s",
                                  source, stamp, frame["frame_revision"])
                    return frame
        raise RuntimeError("no displayed active capture caught up to its camera")

    async def check_http(self):
        deadline = time.monotonic() + 6
        while True:
            try:
                with urlopen("http://127.0.0.1:24179/perception_runtime.json",
                             timeout=2) as response:
                    self.assertEqual("no-store", response.headers.get("Cache-Control"))
                    runtime = json.loads(response.read(1048576))
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                await asyncio.sleep(0.1)
        self.assertEqual("web_video_test", runtime["uav_name"])
        self.assertEqual(24178, runtime["websocket_port"])
        with urlopen("http://127.0.0.1:24179/", timeout=2) as response:
            self.assertEqual("no-store", response.headers.get("Cache-Control"))
            html = response.read(1048576).decode("utf-8")
        for asset in ("perception_video.js", "perception_client.js", "perception_ui.js"):
            self.assertIn(asset, html)
        with urlopen("http://127.0.0.1:24179/perception_api.json", timeout=2) as response:
            self.assertEqual("no-store", response.headers.get("Cache-Control"))
            manifest = json.loads(response.read(1048576))
        self.assertEqual("/{UAV_NAME}/perception/", manifest["namespace_template"])
        self.assertEqual("xd_uav_track/PerceptionVideoFrame", manifest["subscribe"]["video"]["type"])
        self.assertFalse(manifest["video"]["resize"])
        self.assertEqual("targets.frame_seq", manifest["video"]["frame_seq_field"])

    async def connect(self):
        deadline = time.monotonic() + 12
        while True:
            try:
                return await websocket_connect("ws://127.0.0.1:24178",
                                               connect_timeout=1)
            except Exception:
                if time.monotonic() > deadline:
                    raise
                await asyncio.sleep(0.1)

    async def read(self, socket, predicate, seconds=6):
        deadline = time.monotonic() + seconds
        for packet in tuple(self.packets):
            if predicate(packet):
                self.packets.remove(packet)
                return packet
        while time.monotonic() < deadline:
            self.assertIsNone(self.worker_error, str(self.worker_error))
            raw = await asyncio.wait_for(socket.read_message(),
                                         max(0.01, deadline - time.monotonic()))
            if raw is None:
                raise RuntimeError("rosbridge websocket unexpectedly closed")
            packet = json.loads(raw)
            if predicate(packet):
                return packet
            self.packets.append(packet)
        raise RuntimeError("bounded websocket wait timed out")

    async def state(self, socket):
        self.request_number += 1
        request = "state-{}".format(self.request_number)
        socket.write_message(json.dumps(dict(
            op="call_service", id=request, service=self.ROOT + "state/get", args={})))
        packet = await self.read(socket, lambda p: p.get("op") == "service_response"
                                 and p.get("id") == request)
        self.assertTrue(packet["result"], packet)
        self.assertTrue(packet["values"]["success"], packet)
        return packet["values"]["state"]

    async def command(self, socket, suffix, **fields):
        state = await self.state(socket)
        self.request_number += 1
        request = "video-command-{}".format(self.request_number)
        now = rospy.Time.now()
        message = dict(request_id=request, session_id=state["session_id"],
                       expected_state_version=state["state_version"],
                       issued_at=dict(secs=now.secs, nsecs=now.nsecs),
                       ttl_sec=6.0, **fields)
        socket.write_message(json.dumps(dict(op="publish", topic=self.ROOT + suffix,
                                            msg=message)))
        result = (await self.read(socket, lambda p:
                  p.get("topic") == self.ROOT + "command_result"
                  and p["msg"]["request_id"] == request
                  and p["msg"]["stage"] in ("APPLIED", "REJECTED", "FAILED")))["msg"]
        return result

    async def video(self, socket, source, predicate=lambda frame: True, seconds=6):
        return (await self.read(socket, lambda p:
                p.get("topic") == self.ROOT + "video"
                and p["msg"]["targets"]["image_source"] == source
                and predicate(p["msg"]), seconds))["msg"]

    def validate_video(self, frame):
        targets = frame["targets"]
        self.assertEqual(self.WIDTH, targets["image_width"])
        self.assertEqual(self.HEIGHT, targets["image_height"])
        stamp = self.stamp_key(frame["header"]["stamp"])
        self.assertEqual(stamp, self.stamp_key(targets["header"]["stamp"]))
        self.assertEqual(stamp, self.stamp_key(frame["image"]["header"]["stamp"]))
        # rospy assigns the top-level Header.seq a transport publish sequence.
        # The nested image/targets retain the source capture identity instead.
        self.assertEqual(frame["image"]["header"]["seq"], targets["frame_seq"])
        self.assertEqual(targets["header"]["frame_id"], frame["header"]["frame_id"])
        self.assertEqual(frame["header"]["frame_id"], frame["image"]["header"]["frame_id"])
        self.assertIn("png", frame["image"]["format"].lower())
        data = frame["image"]["data"]
        data = base64.b64decode(data, validate=True) if isinstance(data, str) else bytes(data)
        self.assertEqual(b"\x89PNG\r\n\x1a\n", data[:8])
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(decoded)
        self.assertEqual((self.HEIGHT, self.WIDTH, 3), decoded.shape)
        with self.frame_lock:
            original = self.originals.get((targets["image_source"], stamp))
        self.assertIsNotNone(original, "video must reference a bounded, recent source capture")
        # PNG preserves pixels away from the overlay, including a per-capture marker.
        np.testing.assert_array_equal(original[210:235, 260:310],
                                      decoded[210:235, 260:310])
        return decoded, original

    def selection(self, frame, kind="roi", uid="", **fields):
        targets = frame["targets"]
        return dict(selection_type=kind, image_source=targets["image_source"],
                    frame_seq=targets["frame_seq"],
                    capture_stamp=targets["header"]["stamp"],
                    target_uid=uid, track_id=-1, tracker_algorithm="template",
                    point_norm=[120 / self.WIDTH, 108 / self.HEIGHT],
                    roi_norm=[96 / self.WIDTH, 92 / self.HEIGHT,
                              48 / self.WIDTH, 32 / self.HEIGHT], **fields)

    async def scenario(self):
        rospy.wait_for_service(self.ROOT + "state/get", timeout=12)
        await self.check_http()
        socket = await self.connect()
        try:
            for name in ("video", "state", "command_result", "trajectory"):
                socket.write_message(json.dumps(dict(
                    op="subscribe", topic=self.ROOT + name,
                    queue_length=16 if name == "command_result" else 1)))
            for suffix, message_type in (
                    ("target/select", "SelectPerceptionTargetCommand"),
                    ("mode/set", "SetPerceptionModeCommand"),
                    ("model/set", "SetDetectorModelCommand")):
                socket.write_message(json.dumps(dict(
                    op="advertise", topic=self.ROOT + suffix,
                    type="xd_uav_track/" + message_type)))
            # Confirm both cameras have complete frame/metadata bundles through
            # rosbridge's allow-list, using actual C++ detections and localization.
            initial = {}
            for source in self.SOURCES:
                initial[source] = await self.video(socket, source, lambda f:
                    f["targets"]["mode"] == "detect" and any(
                        t["measurement_source"] == "detector" and t["position_valid"]
                        for t in f["targets"]["targets"]), seconds=10)
                decoded, original = self.validate_video(initial[source])
                self.assertFalse(np.array_equal(original[92, 96], decoded[92, 96]),
                                 "the web video must contain the detection overlay")
                target = next(t for t in initial[source]["targets"]["targets"]
                              if t["position_valid"])
                self.assertEqual(list(self.BOX), target["bbox_xyxy"])
                np.testing.assert_allclose([120, 108], target["center_px"], atol=0.1)
                self.assertEqual("web_video_test/base_link_frd", target["position_frame"])
                self.assertEqual(initial[source]["targets"]["header"]["stamp"],
                                 target["position_stamp"])
                np.testing.assert_allclose([10, 2, -5], target["position_xyz"], atol=0.01)
            fixed_id = initial["fixed_rgb"]["targets"]["targets"][0]["track_id"]
            await self.video(socket, "gimbal_rgb", lambda f: any(
                t["track_id"] == fixed_id and t["position_valid"] for t in f["targets"]["targets"]), seconds=8)
            state = await self.state(socket)
            self.assertEqual("detect", state["mode"])
            self.assertTrue(state["model_ready"])
            self.assertEqual("synthetic-input", state["active_model_id"])

            # Every interactive selection refers to the frame actually displayed.
            frame = await self.video(socket, "fixed_rgb", lambda f:
                any(t["measurement_source"] == "detector" for t in f["targets"]["targets"]))
            result = await self.command(socket, "target/select", **self.selection(frame))
            self.assertEqual("APPLIED", result["stage"], result)
            state = await self.state(socket)
            uid, track_id = state["selected_target_uid"], state["selected_track_id"]
            self.assertTrue(uid)
            self.assertEqual("track", state["mode"])
            for kind in ("point", "track_id"):
                frame = await self.video(socket, "fixed_rgb", lambda f:
                    f["targets"]["state_version"] >= state["state_version"]
                    and any(t["target_uid"] == uid and t["selected"]
                            and t["lifecycle_state"] == "confirmed"
                            for t in f["targets"]["targets"]))
                self.validate_video(frame)
                result = await self.command(socket, "target/select",
                                            **self.selection(frame, kind, uid if kind == "track_id" else ""))
                self.assertEqual("APPLIED", result["stage"], result)
                state = await self.state(socket)
                self.assertEqual(uid, state["selected_target_uid"])
                self.assertEqual(track_id, state["selected_track_id"])
            stale = self.selection(frame, "point")
            stale["frame_seq"] += 100000
            result = await self.command(socket, "target/select", **stale)
            self.assertEqual("REJECTED", result["stage"], result)
            self.assertEqual("STALE_FRAME", result["error_code"], result)
            result = await self.command(socket, "mode/set", mode="fusion")
            self.assertEqual("APPLIED", result["stage"], result)
            state = await self.state(socket)
            self.assertEqual("fusion", state["mode"])
            self.assertEqual(uid, state["selected_target_uid"])
            fused = await self.video(socket, "fixed_rgb", lambda f:
                f["targets"]["mode"] == "fusion" and any(
                    t["target_uid"] == uid and t["selected"]
                    for t in f["targets"]["targets"]))
            self.validate_video(fused)
            path = (await self.read(socket, lambda p:
                    p.get("topic") == self.ROOT + "trajectory"
                    and p["msg"]["target_uid"] == uid and p["msg"]["points"]))["msg"]
            self.assertLessEqual(len(path["points"]), 80)

            # No shared detector subscriber exists. A synthetic READY status
            # must never turn into a fabricated successful engine transaction.
            before = await self.state(socket)
            result = await self.command(socket, "model/set", model_id="missing-web-test-model")
            self.assertEqual("REJECTED", result["stage"], result)
            self.assertEqual("UNAVAILABLE", result["error_code"], result)
            after = await self.state(socket)
            for field in ("active_model_id", "model_epoch", "state_version",
                          "selected_target_uid", "mode"):
                self.assertEqual(before[field], after[field], field)

            # Source timeout republishes semantics for the exact same capture.
            # The video revision must advance so a frontend cannot keep a box
            # alive solely because image stamp/seq did not change.
            active = await self.freeze_displayed_source(
                socket, "fixed_rgb", uid, after["state_version"])
            frozen_stamp = self.stamp_key(active["targets"]["header"]["stamp"])
            self.validate_video(active)
            lost = await self.video(socket, "fixed_rgb", lambda f:
                self.stamp_key(f["header"]["stamp"]) == frozen_stamp and any(
                    t["target_uid"] == uid and t["lifecycle_state"] == "lost"
                    and t["measurement_source"] == "stale"
                    for t in f["targets"]["targets"]), seconds=5)
            self.assertGreater(lost["frame_revision"], active["frame_revision"])
            lost_image, original = self.validate_video(lost)
            target = next(t for t in lost["targets"]["targets"] if t["target_uid"] == uid)
            self.assertEqual([0, 0, 0, 0], target["bbox_xyxy"])
            self.assertFalse(target["position_valid"])
            np.testing.assert_array_equal(original[92:125, 95:146],
                                          lost_image[92:125, 95:146])
            await self.read(socket, lambda p:
                p.get("topic") == self.ROOT + "state"
                and p["msg"]["selected_target_uid"] == uid
                and p["msg"]["operating_state"] == "LOST", seconds=3)
            state = await self.state(socket)
            self.assertEqual("LOST", state["operating_state"])

            result = await self.command(socket, "target/select", selection_type="clear")
            self.assertEqual("APPLIED", result["stage"], result)
            state = await self.state(socket)
            self.assertEqual("", state["selected_target_uid"])
            self.assertFalse(state["selection_active"])
            result = await self.command(socket, "mode/set", mode="detect")
            self.assertEqual("APPLIED", result["stage"], result)
            with self.frame_lock:
                self.frozen_sources.discard("fixed_rgb")
            restored = await self.video(socket, "fixed_rgb", lambda f:
                f["targets"]["mode"] == "detect" and any(
                    t["measurement_source"] == "detector" and not t["selected"]
                    for t in f["targets"]["targets"]))
            self.validate_video(restored)
            rospy.loginfo("VIDEO_WEB_CONTRACT_PASS native PNG, exact metadata, FRD positions, "
                          "frame-bound ROI/point/UID, mode switch, model UNAVAILABLE, "
                          "same-frame LOST revision and clear, HTTP runtime/assets; "
                          "no GPU/flight inputs")
        finally:
            socket.close()

    def test_actual_rosbridge_native_video_and_interactions(self):
        asyncio.run(asyncio.wait_for(self.scenario(), timeout=60))


if __name__ == "__main__":
    rospy.init_node("test_perception_video_web_ros")
    rostest.rosrun("xd_uav_track", "video_web_contract", VideoWebContractTest)
