#!/usr/bin/env python3
"""Regression for train7 vehicle-class flips in prediction ROI recovery."""

import importlib.util
from pathlib import Path
import threading
import time
import unittest

import numpy as np

from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "multi_source_detection_node.py"
SPEC = importlib.util.spec_from_file_location("multi_source_detection_node", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def detection(class_id, bbox, confidence=0.8):
    x1, y1, x2, y2 = bbox
    return NormalizedDetection(
        track_id=-1, track_id_is_stable=False, class_id=class_id,
        confidence=confidence, aabb_xyxy=bbox,
        center_xy=((x1 + x2) // 2, (y1 + y2) // 2))


class _Backend:
    def __init__(self, class_id):
        self.class_id = class_id
        self.calls = 0

    def detect(self, *_args):
        self.calls += 1
        return "detect", [detection(self.class_id, (20, 20, 60, 60))]


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class PredictionRoiClassBridgeTest(unittest.TestCase):
    @staticmethod
    def node(backend):
        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        node._sources = {"fixed": {
            "roi_recovery_enabled": True,
            "roi_padding_ratio": 1.0,
            "roi_target_short_side": 56,
            "roi_min_padding_ratio": 0.35,
            "roi_max_padding_ratio": 1.0,
            "roi_min_short_side": 24,
        }}
        node._tracking_lock = threading.Lock()
        node._tracked_boxes = {
            "fixed": ((100, 100, 140, 140), 2, time.monotonic())}
        node._priority_source_timeout_sec = 1.0
        node._compatible_class_ids = frozenset((0, 1, 2, 3))
        node._backend = backend
        node._confidence = 0.10
        node._iou = 0.45
        node._maximum_detections = 30
        node._roi_inference_image_size = 640
        node._roi_attempts = 0
        node._roi_ms_ewma = 0.0
        node._roi_hits = 0
        return node

    def test_full_frame_other_vehicle_class_satisfies_tank_roi_hint(self):
        backend = _Backend(0)
        node = self.node(backend)
        full_frame = [detection(0, (101, 101, 141, 141))]
        output = node._recover_with_prediction_roi(
            "fixed", np.zeros((300, 300, 3), np.uint8), full_frame)
        self.assertIs(output, full_frame)
        self.assertEqual(backend.calls, 0)

    def test_roi_recovers_vehicle_but_rejects_disallowed_class(self):
        image = np.zeros((300, 300, 3), np.uint8)
        compatible = self.node(_Backend(1))
        recovered = compatible._recover_with_prediction_roi("fixed", image, [])
        self.assertEqual(len(recovered), 1)
        self.assertEqual(recovered[0].class_id, 1)
        self.assertEqual(compatible._roi_hits, 1)

        incompatible = self.node(_Backend(4))
        self.assertEqual(incompatible._recover_with_prediction_roi(
            "fixed", image, []), [])
        self.assertEqual(incompatible._roi_hits, 0)

    def test_selected_pixel_hint_is_not_disabled_by_lifecycle_or_age(self):
        from xd_uav_track.msg import TrackState, TrackStateArray
        import rospy

        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        node._source_name_by_image_source = {"benchmark_fixed": "fixed"}
        node._tracking_lock = threading.Lock()
        node._tracked_boxes = {}
        node._pixel_track_hints = {}
        node._other_confirmed_boxes = {}
        node._pixel_assist_max_tracks = 4
        node._allowed_class_ids = frozenset((0, 1, 2, 3))
        node._priority_source = ""
        node._priority_source_wall = 0.0

        message = TrackStateArray()
        message.image_source = "benchmark_fixed"
        message.header.stamp = rospy.Time.from_sec(12.0)
        track = TrackState()
        track.selected = True
        track.lifecycle_state = "lost"
        track.frames_since_detection = 2
        track.bbox = [10, 20, 50, 60]
        track.class_id = 1
        track.track_id = 7
        message.tracks = [track]
        node._tracking_callback(message)
        self.assertEqual(node._pixel_track_hints["fixed"][0][-1], "lost")

        track.frames_since_detection = 20
        node._tracking_callback(message)
        self.assertIn("fixed", node._pixel_track_hints)

        track.frames_since_detection = 200
        track.lifecycle_state = "tentative"
        node._tracking_callback(message)
        self.assertIn("fixed", node._pixel_track_hints)

        track.selected = False
        node._tracking_callback(message)
        self.assertNotIn("fixed", node._pixel_track_hints)

    def test_pixel_flow_runs_on_hits_and_only_fills_detection_misses(self):
        from types import SimpleNamespace
        import rospy

        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        node._tracking_lock = threading.Lock()
        node._pixel_track_hints = {"fixed": [(
            (10, 20, 50, 60), 1, 7, 12.1, time.monotonic(), (0.0, 0.0), "lost")]}
        node._pixel_previous_frames = {"fixed": {
            "gray": np.zeros((80, 80), np.uint8), "stamp": 12.0,
            "boxes": {7: (10, 20, 50, 60)}}}
        node._pixel_assist_hint_timeout_sec = 2.2
        node._pixel_assist_max_frame_gap_sec = 0.8
        node._pixel_assist_confidence = 0.45
        node._compatible_class_ids = frozenset((0, 1, 2, 3))
        node._pixel_tracking_attempts = 0
        node._pixel_tracking_valid = 0
        node._pixel_assist_frames = 0
        node._pixel_assist_rejected = 0
        node._pixel_yolo_rejected = 0
        node._other_confirmed_boxes = {"fixed": []}
        node._pixel_assist_ms_ewma = 0.0
        flow_boxes = iter(((12, 20, 52, 60), (14, 20, 54, 60)))
        node._flow_bbox = lambda *_args: next(flow_boxes)

        message = SimpleNamespace(header=SimpleNamespace(
            stamp=rospy.Time.from_sec(12.1)))
        image = np.zeros((80, 80, 3), np.uint8)
        output, assists = node._add_pixel_assist("fixed", message, image, [])
        self.assertEqual(len(assists), 1)
        self.assertEqual(assists[0].confidence, 0.45)
        self.assertEqual(len(output), 1)

        node._pixel_track_hints["fixed"] = [(
            (12, 20, 52, 60), 1, 7, 12.2, time.monotonic(), (0.0, 0.0), "confirmed")]
        message.header.stamp = rospy.Time.from_sec(12.2)
        observed = [detection(1, (12, 20, 52, 60), confidence=0.9)]
        output, assists = node._add_pixel_assist(
            "fixed", message, image, observed)
        self.assertFalse(assists)
        self.assertIsNot(output, observed)
        self.assertEqual(output[0].aabb_xyxy, (13, 20, 53, 60))
        self.assertEqual(node._pixel_tracking_attempts, 2)
        self.assertEqual(node._pixel_tracking_valid, 2)

    def test_locked_flow_rejects_a_detector_jump_without_reassigning_id(self):
        from types import SimpleNamespace
        import rospy

        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        node._tracking_lock = threading.Lock()
        node._pixel_track_hints = {"fixed": [(
            (10, 20, 50, 60), 1, 7, 12.1, time.monotonic(), (0.0, 0.0), "confirmed")]}
        node._pixel_previous_frames = {"fixed": {
            "gray": np.zeros((100, 100), np.uint8), "stamp": 12.0,
            "boxes": {7: (10, 20, 50, 60)}}}
        node._other_confirmed_boxes = {"fixed": [(7, (10, 20, 50, 60))]}
        node._pixel_assist_hint_timeout_sec = 2.2
        node._pixel_assist_max_frame_gap_sec = 0.8
        node._pixel_assist_confidence = 0.45
        node._compatible_class_ids = frozenset((0, 1, 2, 3))
        node._pixel_tracking_attempts = 0
        node._pixel_tracking_valid = 0
        node._pixel_assist_frames = 0
        node._pixel_assist_rejected = 0
        node._pixel_yolo_rejected = 0
        node._pixel_assist_ms_ewma = 0.0
        node._flow_bbox = lambda *_args: (12, 20, 52, 60)
        message = SimpleNamespace(header=SimpleNamespace(
            stamp=rospy.Time.from_sec(12.1)))
        jumped = detection(1, (35, 20, 75, 60), 0.9)
        output, assists = node._add_pixel_assist(
            "fixed", message, np.zeros((100, 100, 3), np.uint8), [jumped])
        self.assertEqual(node._pixel_yolo_rejected, 1)
        self.assertEqual(len(output), 1)
        self.assertIs(output[0], assists[0])
        self.assertEqual(assists[0].aabb_xyxy, (12, 20, 52, 60))
        self.assertEqual(node._pixel_previous_frames["fixed"]["boxes"][7],
                         (12, 20, 52, 60))

    def test_two_confirmed_tracks_get_independent_pixel_assists(self):
        from types import SimpleNamespace
        import rospy

        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        node._tracking_lock = threading.Lock()
        now = time.monotonic()
        first, second = (10, 20, 40, 50), (100, 20, 130, 50)
        node._pixel_track_hints = {"fixed": [
            (first, 1, 7, 12.1, now, (0.0, 0.0), "confirmed"),
            (second, 1, 8, 12.1, now, (0.0, 0.0), "confirmed")]}
        node._other_confirmed_boxes = {"fixed": [(7, first), (8, second)]}
        node._pixel_previous_frames = {"fixed": {
            "gray": np.zeros((80, 160), np.uint8), "stamp": 12.0,
            "boxes": {7: first, 8: second}}}
        node._pixel_assist_hint_timeout_sec = 2.2
        node._pixel_assist_max_frame_gap_sec = 0.8
        node._pixel_assist_confidence = 0.45
        node._compatible_class_ids = frozenset((0, 1, 2, 3))
        node._pixel_tracking_attempts = 0
        node._pixel_tracking_valid = 0
        node._pixel_assist_frames = 0
        node._pixel_assist_rejected = 0
        node._pixel_yolo_rejected = 0
        node._pixel_assist_ms_ewma = 0.0
        node._flow_bbox = lambda _old, _new, box, _dt, _velocity: tuple(
            value + 2 if index % 2 == 0 else value
            for index, value in enumerate(box))
        message = SimpleNamespace(header=SimpleNamespace(
            stamp=rospy.Time.from_sec(12.1)))
        output, assists = node._add_pixel_assist(
            "fixed", message, np.zeros((80, 160, 3), np.uint8), [])
        self.assertEqual(len(output), 2)
        self.assertEqual(len(assists), 2)
        self.assertEqual([item.aabb_xyxy for item in assists],
                         [(12, 20, 42, 50), (102, 20, 132, 50)])
        self.assertEqual(node._pixel_tracking_attempts, 2)

    def test_motion_lead_guides_reanchor_without_creating_a_box(self):
        from types import SimpleNamespace
        import rospy

        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        node._tracking_lock = threading.Lock()
        node._pixel_track_hints = {"fixed": [(
            (10, 20, 50, 60), 1, 7, 12.0, time.monotonic(),
            (100.0, 0.0), "confirmed")]}
        node._pixel_previous_frames = {}
        node._other_confirmed_boxes = {"fixed": []}
        node._pixel_assist_hint_timeout_sec = 2.2
        node._pixel_assist_max_frame_gap_sec = 0.8
        node._compatible_class_ids = frozenset((0, 1, 2, 3))
        node._pixel_assist_ms_ewma = 0.0
        message = SimpleNamespace(header=SimpleNamespace(
            stamp=rospy.Time.from_sec(12.2)))
        observed = [detection(2, (31, 20, 71, 60))]
        output, assists = node._add_pixel_assist(
            "fixed", message, np.zeros((100, 120, 3), np.uint8), observed)
        self.assertFalse(assists)
        self.assertEqual(output[0].aabb_xyxy, (31, 20, 71, 60))
        self.assertEqual(node._pixel_previous_frames["fixed"]["boxes"][7],
                         (31, 20, 71, 60))

    def test_camera_detection_area_excludes_scenery_not_target_lane(self):
        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        node._masked_detections = 0
        items = [detection(3, (529, 45, 613, 147)),
                 detection(2, (135, 310, 245, 395))]
        output = node._filter_detection_area(
            {"detection_area": (0.06, 0.05, 0.73, 0.98)}, items, 640, 480)
        self.assertEqual(output, items[1:])
        self.assertEqual(node._masked_detections, 1)

    def test_lk_flow_produces_a_bounded_box_translation(self):
        import cv2

        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        node._pixel_assist_min_features = 4
        node._pixel_assist_fb_error_px = 1.5
        node._pixel_assist_max_velocity_px_sec = 300.0
        old_points = np.array(
            [[[15, 25]], [[25, 25]], [[35, 25]],
             [[15, 35]], [[25, 35]], [[35, 35]]], dtype=np.float32)
        original_features = cv2.goodFeaturesToTrack
        original_flow = cv2.calcOpticalFlowPyrLK
        calls = {"count": 0}

        def fake_features(*_args, **_kwargs):
            return old_points.copy()

        def fake_flow(_previous, _current, points, _next=None, **_kwargs):
            calls["count"] += 1
            delta = np.array([[[1.0, 0.0]]], dtype=np.float32)
            moved = points + delta if calls["count"] == 1 else points - delta
            return moved, np.ones((len(points), 1), dtype=np.uint8), None

        cv2.goodFeaturesToTrack = fake_features
        cv2.calcOpticalFlowPyrLK = fake_flow
        try:
            image = np.zeros((80, 80), dtype=np.uint8)
            moved = node._flow_bbox(image, image, (10, 20, 50, 60), 0.1)
        finally:
            cv2.goodFeaturesToTrack = original_features
            cv2.calcOpticalFlowPyrLK = original_flow
        self.assertEqual(moved, (11, 20, 51, 60))

    def test_slow_reid_never_blocks_detector_publication(self):
        from types import SimpleNamespace
        import rospy

        node = MODULE.MultiSourceDetectionNode.__new__(
            MODULE.MultiSourceDetectionNode)
        publisher = _Publisher()
        node._sources = {"fixed": {
            "xd_publisher": publisher, "vision_publisher": None,
            "reid_enabled": True, "image_source": "benchmark_fixed",
            "sensor_id": "fixed_camera", "detector_name": "test",
            "model_version": "test"}}
        node._filter_oversized = lambda items, *_args: items
        node._recover_with_prediction_roi = lambda _name, _frame, items: items
        node._add_pixel_assist = lambda _name, _message, _frame, items: (items, ())
        node._reid_condition = threading.Condition()
        node._pending_reid = {}
        node._recent_reid = {}
        node._reid_cache_age_sec = 0.75
        node._reid_reused = 0
        node._reid_dropped_backlog = 0
        node._compatible_class_ids = frozenset((0, 1, 2, 3))
        image = SimpleNamespace(
            header=SimpleNamespace(stamp=rospy.Time.from_sec(12.0)),
            width=100, height=100)
        frame = np.zeros((100, 100, 3), np.uint8)

        node._publish("fixed", image, frame, [detection(1, (10, 20, 50, 60))])
        self.assertEqual(len(publisher.messages), 1)
        self.assertIn("fixed", node._pending_reid)
        self.assertEqual(publisher.messages[0].candidates[0].appearance_embedding, [])

        node._recent_reid["fixed"] = {
            "stamp": 12.0,
            "entries": [{"bbox": (10, 20, 50, 60), "class_id": 1,
                         "embedding": [0.6, 0.8]}]}
        image.header.stamp = rospy.Time.from_sec(12.2)
        node._publish("fixed", image, frame, [detection(2, (11, 20, 51, 60))])
        self.assertEqual(len(publisher.messages), 2)
        self.assertEqual(publisher.messages[1].candidates[0].appearance_embedding,
                         [0.6, 0.8])
        self.assertEqual(node._reid_reused, 1)


if __name__ == "__main__":
    unittest.main()
