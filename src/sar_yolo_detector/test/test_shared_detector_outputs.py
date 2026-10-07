#!/usr/bin/env python3
"""The shared TensorRT detector also supports a true single-camera output."""

import importlib.util
from pathlib import Path
import unittest
import threading
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
import rospy
from sensor_msgs.msg import Image
from xd_uav_track.msg import PerceptionState, PerceptionTargetArray, PerceptionTarget
from xd_uav_track.msg import DetectionArray, DetectionCandidate
from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "multi_source_detection_node.py"
SPEC = importlib.util.spec_from_file_location("multi_source_detection_node", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SharedDetectorOutputTest(unittest.TestCase):
    def test_roi_inference_still_uses_gpu_lock_and_releases_it(self):
        node=MODULE.MultiSourceDetectionNode.__new__(MODULE.MultiSourceDetectionNode)
        node._gpu_lock=threading.Lock(); node._tracking_lock=threading.Lock()
        node._sources={'fixed':{'roi_recovery_enabled':True,'roi_attempts':0}}
        node._tracked_boxes={'fixed':((20,20,80,70),2,MODULE.time.monotonic())}
        node._priority_source_timeout_sec=1.; node._compatible_class_ids=frozenset([0,1,2,3])
        node._confidence=.1; node._iou=.5; node._maximum_detections=30
        node._roi_attempts=0; node._roi_inference_image_size=640; node._roi_ms_ewma=0.
        def detect(*args):
            self.assertTrue(node._gpu_lock.locked())
            return None,[]
        node._backend=SimpleNamespace(detect=detect)
        detections=[]
        result=node._recover_with_prediction_roi('fixed',np.zeros((160,240,3),np.uint8),detections)
        self.assertEqual(result,detections)
        self.assertEqual(node._roi_attempts,1)
        self.assertFalse(node._gpu_lock.locked())

    def test_gpu_lock_covers_inference_not_cpu_publication(self):
        rospy.rostime.set_rostime_initialized(True)
        for batch_size in (1,2):
            with self.subTest(batch_size=batch_size):
                node=MODULE.MultiSourceDetectionNode.__new__(MODULE.MultiSourceDetectionNode)
                node._gpu_lock=threading.Lock()
                node._confidence=.1; node._iou=.5; node._maximum_detections=30
                node._maximum_age_sec=0.; node._inference_batches=0; node._processed=0
                node._received=2; node._dropped_backlog=0
                seen=[]
                def detect(*args):
                    self.assertTrue(node._gpu_lock.locked())
                    return None,[]
                def detect_many(frames,*args):
                    return [detect(frame) for frame in frames]
                node._backend=SimpleNamespace(detect=detect,detect_many=detect_many)
                def publish(name,message,frame,detections,generation):
                    self.assertFalse(node._gpu_lock.locked())
                    seen.append(name)
                node._publish=publish
                items=[]
                for i in range(batch_size):
                    message,_=self.input()
                    frame=np.zeros((message.height,message.width,3),np.uint8)
                    message.encoding='bgr8'; message.step=message.width*3
                    message.data=frame.tobytes()
                    items.append((str(i),message))
                with patch.object(MODULE.rospy,'loginfo_throttle'):
                    node._process_batch(items)
                self.assertEqual(seen,[str(i) for i in range(batch_size)])
                self.assertEqual(node._processed,batch_size)

    def test_independent_pixel_feedback_is_exact_frame_visual_only(self):
        node=self.node(Publisher(),Publisher())
        node._source_name_by_image_source={'fixed_rgb':'fixed'}
        node._pixel_observations={}
        node._pixel_assist_max_tracks=8
        node._pixel_assist_confidence=.45
        node._compatible_class_ids=frozenset([0,1,2,3])
        message,_=self.input()
        target=PerceptionTarget(track_id=77,class_id=2,lifecycle_state='confirmed',
            measurement_source='visual_tracker',tracking_quality=.8,
            bbox_xyxy=[10,20,60,70],center_px=[35.,45.])
        observed=PerceptionTargetArray(header=message.header,image_source='fixed_rgb',
            image_width=128,image_height=96,model_epoch=1,targets=[target])
        node._pixel_observations_callback(observed)
        detections,assists=node._merge_pixel_observations('fixed',message,[])
        self.assertEqual(len(detections),1)
        self.assertEqual(detections[0].track_id,-1)
        self.assertEqual(assists[0].confidence,.45)
        message.header.stamp=rospy.Time(13)
        self.assertEqual(node._merge_pixel_observations('fixed',message,[]),([],()))
        # Snapshots are bounded and predicted boxes never masquerade as flow.
        target.measurement_source='prediction'
        for seq in range(20):
            observed.header.stamp=rospy.Time(20+seq)
            node._pixel_observations_callback(__import__('copy').deepcopy(observed))
        self.assertEqual(len(node._pixel_observations['fixed']),16)
        message.header.stamp=rospy.Time(39)
        self.assertEqual(node._merge_pixel_observations('fixed',message,[]),([],()))

    def test_camera_clock_restart_invalidates_inflight_generation(self):
        rospy.rostime.set_rostime_initialized(True)
        node = self.node(Publisher(), None)
        node._sources["fixed"]["last_stamp"] = rospy.Time(30)
        node._received = node._dropped_backlog = 0
        node._pause_without_subscribers = False
        node._require_stamp = node._reject_out_of_order = True
        node._maximum_age_sec = 0
        node._source_rate_available = lambda *_: True
        node._condition = threading.Condition()
        node._runtime_generation = 1
        node._runtime_reset_requested = threading.Event()
        node._pending = {}
        message, _ = self.input()
        message.header.stamp = rospy.Time(1)
        node._image_callback("fixed", message)
        self.assertEqual(2, node._runtime_generation)
        self.assertTrue(node._runtime_reset_requested.is_set())
        self.assertEqual(rospy.Time(1), node._sources["fixed"]["last_stamp"])
        self.assertIs(message, node._pending["fixed"])
        duplicate, _ = self.input()
        duplicate.header.stamp = rospy.Time(1)
        node._image_callback("fixed", duplicate)
        self.assertIs(message, node._pending["fixed"])
        self.assertEqual(2, node._runtime_generation)

    def test_mode_reset_is_batch_boundary_and_heartbeat_does_not_repeat_it(self):
        node = MODULE.MultiSourceDetectionNode.__new__(MODULE.MultiSourceDetectionNode)
        node._tracking_lock = threading.Lock()
        node._condition, node._reid_condition = threading.Condition(), threading.Condition()
        node._source_name_by_image_source = {}
        node._sources = {"fixed": {"last_accept_wall": 12.0}}
        node._switcher = SimpleNamespace(take_ready=lambda: None)
        for name in ("_pending", "_pending_reid", "_recent_reid", "_tracked_boxes",
                     "_pixel_track_hints", "_other_confirmed_boxes", "_pixel_previous_frames"):
            setattr(node, name, {"old": object()})
        node._perception_callback(PerceptionState(session_id="s", mode="detect", model_epoch=1))
        node._perception_callback(PerceptionState(session_id="s", mode="track", model_epoch=1))
        self.assertTrue(node._runtime_reset_requested.is_set())
        self.assertTrue(node._pending, "callback must not clear in-flight worker state")
        node._commit_switch()
        self.assertFalse(node._runtime_reset_requested.is_set())
        for name in ("_pending", "_pending_reid", "_recent_reid", "_tracked_boxes",
                     "_pixel_track_hints", "_other_confirmed_boxes", "_pixel_previous_frames"):
            self.assertEqual({}, getattr(node, name))
        self.assertEqual(0, node._sources["fixed"]["last_accept_wall"])
        generation = node._runtime_generation
        node._perception_callback(PerceptionState(session_id="s", mode="track", model_epoch=1, state_version=99))
        self.assertEqual(generation, node._runtime_generation)
        self.assertFalse(node._runtime_reset_requested.is_set())

    def test_old_mode_reid_job_is_rejected_even_with_same_model_epoch(self):
        node = MODULE.MultiSourceDetectionNode.__new__(MODULE.MultiSourceDetectionNode)
        node._runtime_generation = 2
        node._model_epoch = 1
        node._encode_reid_cache("fixed", None, DetectionArray(model_epoch=1), [], generation=1)

    def test_old_epoch_reid_worker_and_cache_cannot_pollute_new_model(self):
        node=MODULE.MultiSourceDetectionNode.__new__(MODULE.MultiSourceDetectionNode)
        node._model_epoch=2
        old=DetectionArray(model_epoch=1)
        # Must return before even accessing an encoder or camera configuration.
        node._encode_reid_cache("fixed",None,old,[])
        node._reid_condition=threading.Condition()
        node._recent_reid={"fixed":{"epoch":1,"stamp":1,"entries":[]}}
        output=DetectionArray(model_epoch=2,candidates=[DetectionCandidate()])
        node._attach_recent_reid("fixed",output,[0])
        self.assertEqual([],output.candidates[0].appearance_embedding)

    def test_real_scheduler_changes_rates_and_fails_open_to_search(self):
        node = MODULE.MultiSourceDetectionNode.__new__(MODULE.MultiSourceDetectionNode)
        node._tracking_lock = threading.Lock()
        node._source_name_by_image_source = {"fixed_rgb":"fixed"}
        node._track_anchor_fps = 1.0
        node._track_standby_fps = .5
        node._current_priority_source = lambda: "fixed"
        source = {"image_source":"fixed_rgb", "priority_max_fps":8,
                  "standby_max_fps":4, "discovery_max_fps":6, "last_accept_wall":9.8}
        state = PerceptionState(mode="track",operating_state="TRACKING",
                                selected_image_source="fixed_rgb", selection_active=True)
        with patch.object(MODULE.time, "monotonic", return_value=9.8):
            node._perception_callback(state)
            self.assertTrue(node._source_rate_available("fixed", source))
        with patch.object(MODULE.time, "monotonic", return_value=10.0):
            node._perception_callback(state)
            self.assertFalse(node._source_rate_available("fixed", source))
            state.operating_state = "LOST"
            self.assertTrue(node._source_rate_available("fixed", source))
            source["last_accept_wall"] = 9.8
            state.mode = "fusion"; state.operating_state = "TRACKING"
            self.assertTrue(node._source_rate_available("fixed", source))
            source["last_accept_wall"] = 9.8
            gimbal={key:value for key,value in source.items() if not key.startswith('rate_')}
            gimbal['image_source']='gimbal_rgb'
            self.assertTrue(node._source_rate_available("gimbal", gimbal),
                            "fusion standby uses discovery rate, not selected-source throttle")
            source["last_accept_wall"] = 9.8
            state.mode = "track"
            node._perception_state_wall = 6.0
            self.assertTrue(node._source_rate_available("fixed", source))

    @staticmethod
    def node(xd_publisher, vision_publisher):
        node = MODULE.MultiSourceDetectionNode.__new__(MODULE.MultiSourceDetectionNode)
        node._tracking_lock = threading.Lock()
        node._sources = {"fixed": {
            "xd_publisher": xd_publisher,
            "vision_publisher": vision_publisher,
            "image_source": "fixed_rgb",
            "sensor_id": "fixed_camera",
            "detector_name": "sar_yolo_detector",
            "model_version": "test",
            "reid_enabled": False,
        }}
        node._filter_oversized = lambda detections, *_: detections
        node._model_epoch = 1
        node._active_model_id = "train7"
        node._source_model_sha256 = "test"
        node._class_names = ["car", "ar-car", "tank", "m142"]
        node._recover_with_prediction_roi = lambda _, __, detections: detections
        node._filter_detection_area = lambda _, detections, *__: detections
        node._add_pixel_assist = lambda *args: (args[-1], ())
        return node

    @staticmethod
    def input():
        image = Image()
        image.width, image.height = 128, 96
        image.header.stamp.secs = 12
        detection = NormalizedDetection(
            track_id=-1, track_id_is_stable=False, class_id=2,
            confidence=0.82, aabb_xyxy=(10, 20, 60, 70), center_xy=(35, 45))
        return image, detection

    def test_vision_only_output_does_not_build_xd_identity(self):
        vision = Publisher()
        node = self.node(None, vision)
        image, detection = self.input()
        node._publish("fixed", image, np.zeros((96, 128, 3), np.uint8), [detection])
        self.assertEqual(1, len(vision.messages))
        self.assertEqual(1, len(vision.messages[0].detections))
        self.assertEqual(2, vision.messages[0].detections[0].results[0].id)

    def test_xd_output_leaves_identity_to_tracker(self):
        xd = Publisher()
        node = self.node(xd, None)
        image, detection = self.input()
        node._publish("fixed", image, np.zeros((96, 128, 3), np.uint8), [detection])
        candidate, = xd.messages[0].candidates
        self.assertEqual(-1, candidate.track_id)
        self.assertFalse(candidate.track_id_is_stable)
        self.assertEqual("detector", candidate.measurement_source)
        self.assertEqual("fixed_rgb", xd.messages[0].image_source)

    def test_pixel_assist_is_not_a_yolo_observation(self):
        xd, vision = Publisher(), Publisher()
        node = self.node(xd, vision)
        image, detection = self.input()
        node._add_pixel_assist = lambda *args: ([detection], [detection])
        node._publish("fixed", image, np.zeros((96, 128, 3), np.uint8), [])
        self.assertEqual("visual_tracker", xd.messages[0].candidates[0].measurement_source)
        self.assertEqual([], vision.messages[0].detections)


if __name__ == "__main__":
    unittest.main()
