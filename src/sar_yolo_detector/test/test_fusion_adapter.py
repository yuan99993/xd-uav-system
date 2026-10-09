"""ROS message boundary tests with deterministic detections and real OpenCV KCF."""
import importlib.util
import json
import copy
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import PointStamped
from sar_yolo_detector.msg import PerceptionIdentity
from sar_yolo_detector.srv import SelectFusionROIRequest, SelectSmartTrackRequest, SwitchTrackingModeRequest
from sar_yolo_detector.box_fusion import BoxFusion, Candidate
from sar_yolo_detector.async_pipeline import LatestMailbox, FlowDetections, InferencePacket
from sar_yolo_detector.camera_motion import CameraMotion
from sar_yolo_detector.capture_clock import CaptureClock
from sar_yolo_detector.manual_box_tracker import ManualTrackResult
from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection

SPEC = importlib.util.spec_from_file_location("fusion_node", str(Path(__file__).resolve().parents[1] / "scripts/smart_tracker_node.py"))
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_num_connections(self):
        return 0


class FusionAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # __new__ fixtures skip the production constructor's OpenCV limit.
        # Exercise the same thread budget, without weakening latency bounds.
        cls._previous_cv_threads = cv2.getNumThreads()
        cv2.setNumThreads(1)

    @classmethod
    def tearDownClass(cls):
        cv2.setNumThreads(cls._previous_cv_threads)

    def setUp(self):
        rospy.rostime.set_rostime_initialized(True)
        self.node = NODE.SmartTrackerNode.__new__(NODE.SmartTrackerNode)
        node = self.node
        node._lock = threading.RLock()
        node._backend_lock = threading.Lock()
        node._commit_lock = threading.RLock()
        node._model_busy = False
        node._epoch = 0
        node._publication_context = threading.local()
        self.frame = np.full((160, 220, 3), 20, np.uint8)
        self.frame[50:90, 50:90] = np.random.RandomState(7).randint(0, 240, (40, 40, 3)).astype(np.uint8)
        self.detection = NormalizedDetection(5, 2, .9, (50, 50, 90, 90), (70, 70))
        self.detections = [self.detection]
        node._tracker = SimpleNamespace(
            backend=SimpleNamespace(detect_and_track=lambda *a, **k: (None, self.detections)),
            _should_run_track_api=lambda: True,
            conf_threshold=.1, iou_threshold=.3, max_det=30, tracker_type_str="botsort",
            tracker_args={}, allowed_class_ids=[0, 1, 2, 3], last_detections=[], labels={2: "tank"},
            get_runtime_info=lambda: {"backend": "fake", "device": "cpu", "model_path": "train7.pt"},
            clear_selection=lambda: None)
        node._fusion = BoxFusion(manual_config={"feature_tracker": {"tracker_algorithm": "kcf"}})
        node._fusion_snapshots = deque(maxlen=2)
        node._fusion_snapshot_max_bytes = 25165824
        node._fusion_snapshot_max_age = .75
        node._fusion_input_shape = None
        node._selected_target_id = -1
        node._selected_class_id = -1
        node._selected_bbox = None
        node._selected_target_is_stable = False
        node._selected_detections_publisher = Publisher()
        node._fusion_array_publisher = Publisher()
        node._observation_publisher = Publisher()
        node._last_observation_output_stamp = rospy.Time(0)
        node._fusion_next_id = 900000001
        node._future_tolerance = .02
        node._last_header = None
        node._publish_annotated = True
        node._tracking_mode = "fusion_tracker"
        node._fusion_mode_enabled = True
        node._identity = PerceptionIdentity()
        node._tracked_publisher = Publisher()
        node._state_publisher = Publisher()
        node._fusion_evidence_publisher = Publisher()
        node._manual_enabled = True
        node._manual_min_confidence = 0.
        node._manual_class_id = 2
        node._manual_track_id = 900000000
        node._last_processing_ms = 0.
        node._manual_trackers = {}
        node._manual_pending_roi = None
        node._manual_active = False
        node._flow = FlowDetections()
        self.image = Image(width=220, height=160)
        self.image.header.seq = 10
        self.image.header.frame_id = "camera_optical"
        self.image.header.stamp = rospy.Time.from_sec(time.time())

    def process(self, offset=0.):
        self.image = copy.deepcopy(self.image)
        self.image.header.stamp = rospy.Time.from_sec(time.time() + offset)
        self.image.header.seq += 1
        return self.node._process_fusion(self.image, self.frame)

    def test_loss_evidence_records_initial_detector_and_local_failure(self):
        self.process()
        self.select()
        self.node._fusion.tracker.reset()
        self.detections = []
        self.process(.1)
        self.process(.2)
        evidence = json.loads(self.node._fusion_evidence_publisher.messages[-1].data)
        self.assertTrue(evidence['diagnostics']['no_detector_candidates'])
        self.assertNotEqual(evidence['diagnostics']['local_tracker'], 'valid')
        self.assertEqual(evidence['loss_event']['loss_events'], 1)
        event = self.node._loss_journal.active
        self.assertTrue(event['first_diagnostics']['no_detector_candidates'])
        self.assertEqual(event['first_reason'], 'no_trusted_visual_observation')

    def select(self):
        request = SelectSmartTrackRequest(use_target_id=True, target_id=5)
        response = self.node._select(request)
        self.assertTrue(response.success, response.message)
        return response

    def test_no_selection_keeps_all_yolo_targets(self):
        output_image = self.process()
        self.assertEqual(output_image.shape, self.frame.shape)
        detection = self.node._tracked_publisher.messages[-1].detections[0]
        self.assertEqual(detection.track_id, 5)
        self.assertEqual(self.node._fusion_array_publisher.messages[-1].detections[0].measurement_source, "detector")
        self.assertFalse(self.node._state_publisher.messages[-1].has_selection)

    def test_no_selection_keeps_unstable_detector_boxes(self):
        self.detections = [NormalizedDetection(-1, 2, .9, (50, 50, 90, 90), (70, 70), track_id_is_stable=False)]
        self.process()
        output = self.node._tracked_publisher.messages[-1]
        self.assertEqual(len(output.detections), 1)
        self.assertFalse(output.detections[0].track_id_is_stable)

    def test_conflicting_classes_are_removed_from_output_and_roi_cache(self):
        self.detections.append(NormalizedDetection(7, 3, .8, (51, 50, 91, 90), (71, 70)))
        self.process()
        output = self.node._tracked_publisher.messages[-1]
        self.assertEqual([d.track_id for d in output.detections], [5])
        self.assertEqual(len(self.node._fusion_snapshots[-1][2]), 1)
        request = SelectFusionROIRequest(header=copy.deepcopy(self.image.header),
            roi=RegionOfInterest(x_offset=45, y_offset=45, width=55, height=55))
        response = self.node._select_fusion_roi(request)
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.selected_target_id, 5)

    def test_selected_id_survives_stronger_sibling_class_observation(self):
        self.process()
        self.select()
        self.detections.append(NormalizedDetection(7, 3, .99, (50, 50, 90, 90), (70, 70)))
        self.process()
        output = self.node._tracked_publisher.messages[-1]
        self.assertEqual([d.track_id for d in output.detections], [5])
        self.assertEqual(output.detections[0].detection.results[0].id, 2)
        typed = self.node._fusion_array_publisher.messages[-1]
        selected = self.node._selected_detections_publisher.messages[-1]
        self.assertEqual(len(typed.detections), 1)
        self.assertEqual(typed.detections[0].target, output.detections[0])
        self.assertEqual(selected.detections, output.detections)

    def test_visual_and_weak_new_class_share_one_final_output_box(self):
        self.process()
        self.select()
        self.detections = [NormalizedDetection(7, 3, .4, (50, 50, 90, 90), (70, 70))]
        self.process()
        output = self.node._tracked_publisher.messages[-1]
        self.assertEqual([d.track_id for d in output.detections], [5])
        typed = self.node._fusion_array_publisher.messages[-1]
        self.assertEqual(len(typed.detections), 1)
        self.assertEqual(typed.detections[0].measurement_source, "visual_tracker")
        self.assertEqual(self.node._selected_detections_publisher.messages[-1].detections, output.detections)

    def test_new_class_and_id_still_require_reacquisition_confirmation(self):
        self.process()
        self.select()
        self.detections = [NormalizedDetection(7, 3, .99, (50, 50, 90, 90), (70, 70))]
        for index in range(3):
            self.process()
            output = self.node._tracked_publisher.messages[-1]
            self.assertEqual(len(output.detections), 1)
            if index < 2:
                self.assertEqual(self.node._state_publisher.messages[-1].association_method, "fusion_visual")
                self.assertEqual(len(self.node._selected_detections_publisher.messages[-1].detections), 1)
                self.assertEqual(self.node._fusion.local_id, 5)
                self.assertEqual(self.node._fusion_array_publisher.messages[-1].detections[0].measurement_source, "visual_tracker")
            else:
                self.assertEqual(output.detections[0].track_id, 5)
                self.assertEqual(output.detections[0].detection.results[0].id, 2)

    def test_nearby_different_class_vehicle_remains_visible(self):
        self.detections.append(NormalizedDetection(7, 3, .9, (75, 50, 115, 90), (95, 70)))
        self.process()
        self.assertEqual({d.track_id for d in self.node._tracked_publisher.messages[-1].detections}, {5, 7})

    def test_same_class_pending_local_id_never_duplicates_selected_box(self):
        self.process()
        self.select()
        self.detections = [NormalizedDetection(7, 2, .99, (50, 50, 90, 90), (70, 70))]
        for _ in range(3):
            self.process()
            output = self.node._tracked_publisher.messages[-1]
            self.assertEqual([item.track_id for item in output.detections], [5])

    def test_snapshot_ttl_keeps_useful_history_without_unbounded_frames(self):
        self.node._fusion_snapshot_max_frames = 24
        self.node._fusion_snapshots = deque()
        for index in range(40):
            self.image = copy.deepcopy(self.image)
            self.image.header.stamp = rospy.Time.from_sec(10.+index*.03)
            self.node._cache_fusion_snapshot(self.image, self.frame, [])
        self.assertEqual(len(self.node._fusion_snapshots), 24)
        oldest = self.node._fusion_snapshots[0][0].stamp.to_sec()
        self.assertLessEqual(self.image.header.stamp.to_sec()-oldest, .75)
        self.image.header.stamp = rospy.Time.from_sec(15.)
        self.node._cache_fusion_snapshot(self.image, self.frame, [])
        self.assertEqual(len(self.node._fusion_snapshots), 1)

    def test_output_header_and_provenance_are_immutable_snapshots(self):
        self.process()
        output = self.node._fusion_array_publisher.messages[-1]
        stamp, seq = self.image.header.stamp, self.image.header.seq
        output.header.seq = 999
        self.node._identity.model_version = "changed"
        self.assertEqual(self.image.header.seq, seq)
        self.assertEqual(self.image.header.stamp, stamp)
        self.assertNotEqual(output.provenance.model_version, "changed")

    def test_repeated_same_mode_request_does_not_reset_selected_identity(self):
        self.process()
        self.select()
        epoch = self.node._epoch
        response = self.node._switch_tracking_mode(SwitchTrackingModeRequest(mode="fusion_tracker"))
        self.assertTrue(response.success)
        self.assertEqual(self.node._fusion.public_id, 5)
        self.assertEqual(self.node._epoch, epoch)

    def test_model_switch_releases_state_lock_and_invalidates_old_results(self):
        node = self.node
        node._package_root = Path(__file__).resolve().parents[1]
        entered, release = threading.Event(), threading.Event()
        def switch(*args):
            entered.set()
            release.wait(2.)
            return {"success": False, "message": "synthetic load failure; previous model retained"}
        node._tracker.switch_model = switch
        request = SimpleNamespace(model_path="models/xd_vehicle_train7/xd_vehicle_train7.pt",
                                  model_sha256="", device="auto")
        self.process()
        self.select()
        results = []
        worker = threading.Thread(target=lambda: results.append(node._switch_model(request)))
        worker.start()
        try:
            self.assertTrue(entered.wait(.5))
            started = time.monotonic()
            response = node._switch_tracking_mode(SwitchTrackingModeRequest(mode="smart_tracker"))
            self.assertLess(time.monotonic()-started, .05)
            self.assertFalse(response.success)
            self.assertIn("BUSY", response.message)
            self.assertGreater(node._epoch, 0)
            self.assertEqual(node._fusion.public_id, -1)
        finally:
            release.set()
            worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertFalse(results[0].success)
        self.assertFalse(node._model_busy)
        self.assertFalse(node._tracker.last_detections)

    def test_selected_target_is_single_box_with_original_id(self):
        self.process()
        response = self.select()
        self.process()
        output = self.node._tracked_publisher.messages[-1]
        self.assertEqual(len(output.detections), 1)
        self.assertEqual(output.detections[0].track_id, response.selected_target_id)
        self.assertEqual(response.selected_target_id, 5)
        self.assertEqual(self.node._state_publisher.messages[-1].association_method, "fusion_fused")

    def test_assistance_never_refreshes_detector_time(self):
        self.process()
        self.select()
        stamp = self.node._fusion.last_detection_stamp
        self.detections = []
        self.process()
        output = self.node._tracked_publisher.messages[-1]
        self.assertEqual(len(output.detections), 1)
        self.assertEqual(self.node._fusion_array_publisher.messages[-1].detections[0].measurement_source, "visual_tracker")
        self.assertEqual(self.node._fusion.last_detection_stamp, stamp)
        self.assertFalse(self.node._state_publisher.messages[-1].measurement_current)

    def test_authoritative_observation_keeps_original_capture_and_is_not_repeated(self):
        self.process()
        original = copy.deepcopy(self.image)
        packet = InferencePacket(original, self.frame, tuple(self.detections), self.node._epoch, 80.)
        self.process(.01)  # Presentation has already advanced to another capture.
        self.node._publish_detector_observations(packet)
        message = self.node._observation_publisher.messages[-1]
        self.assertEqual(message.header.stamp, original.header.stamp)
        self.assertEqual(message.detections[0].measurement_source, "detector")
        self.node._publish_detector_observations(packet)
        self.assertEqual(len(self.node._observation_publisher.messages), 1)

    def test_authoritative_selected_observation_uses_public_identity(self):
        self.process()
        self.select()
        self.process()
        packet = InferencePacket(copy.deepcopy(self.image), self.frame, tuple(self.detections), self.node._epoch, 80.)
        self.node._publish_detector_observations(packet)
        message = self.node._observation_publisher.messages[-1]
        self.assertEqual([entry.target.track_id for entry in message.detections], [5])

    def test_unselected_targets_remain_in_fusion(self):
        self.process()
        self.select()
        self.detections.append(NormalizedDetection(6, 2, .9, (150, 50, 180, 90), (165, 70)))
        self.process()
        self.assertEqual({d.track_id for d in self.node._tracked_publisher.messages[-1].detections}, {5, 6})

    def test_roi_original_frame_guard_before_matching(self):
        self.process()
        request = SelectFusionROIRequest()
        request.header = copy.deepcopy(self.image.header)
        request.header.seq += 100
        request.roi = RegionOfInterest(x_offset=45, y_offset=45, width=55, height=55)
        response = self.node._select_fusion_roi(request)
        self.assertFalse(response.success)
        self.assertEqual(self.node._fusion.public_id, -1)

    def test_coarse_roi_reuses_tight_detector_box(self):
        self.process()
        request = SelectFusionROIRequest(header=self.image.header,
                    roi=RegionOfInterest(x_offset=25, y_offset=25, width=100, height=100))
        response = self.node._select_fusion_roi(request)
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.node._fusion.box, (50., 50., 40., 40.))
        self.assertEqual(response.selected_target_id, 5)

    def test_unknown_roi_stays_unknown(self):
        self.detections = []
        self.process()
        request = SelectFusionROIRequest(header=self.image.header,
                    roi=RegionOfInterest(x_offset=50, y_offset=50, width=40, height=40))
        response = self.node._select_fusion_roi(request)
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.node._fusion.class_id, -1)
        self.assertGreaterEqual(response.selected_target_id, 900000001)

    def test_snapshot_byte_budget_and_frame_limit(self):
        for _ in range(8):
            self.process()
        self.assertEqual(len(self.node._fusion_snapshots), 2)
        self.node._fusion_snapshot_max_bytes = self.frame.nbytes
        self.process()
        self.assertEqual(len(self.node._fusion_snapshots), 1)

    def test_clear_selection(self):
        self.process()
        self.select()
        response = self.node._select(SelectSmartTrackRequest(clear_selection=True))
        self.assertTrue(response.success)
        self.assertEqual(self.node._fusion.public_id, -1)

    def test_selected_gimbal_stream_matches_final_fused_box(self):
        self.process()
        self.select()
        self.process()
        fused = self.node._tracked_publisher.messages[-1].detections[0]
        selected = self.node._selected_detections_publisher.messages[-1]
        self.assertEqual(len(selected.detections), 1)
        self.assertEqual(selected.detections[0], fused)

    def test_selected_stream_empty_after_loss(self):
        self.process()
        self.select()
        self.detections = []
        self.frame[:] = 20
        self.process(.1)
        self.assertEqual(self.node._selected_detections_publisher.messages[-1].detections, [])
        self.assertFalse(self.node._state_publisher.messages[-1].control_measurement_ready)

    def test_click_rejects_wrong_original_frame(self):
        self.process()
        click = PointStamped(header=copy.deepcopy(self.image.header))
        click.header.seq += 100
        click.point.x = click.point.y = 70
        self.node._select_pixel_callback(click)
        self.assertEqual(self.node._fusion.public_id, -1)

    def test_click_original_frame_initializes_selection(self):
        self.process()
        click = PointStamped(header=copy.deepcopy(self.image.header))
        click.point.x = click.point.y = 70
        self.node._select_pixel_callback(click)
        self.assertEqual(self.node._fusion.public_id, 5)
        click.point.x = -1
        self.node._select_pixel_callback(click)
        self.assertEqual(self.node._fusion.public_id, -1)
        self.assertEqual(self.node._selected_detections_publisher.messages[-1].detections, [])

    def test_parallel_message_keeps_actual_size_and_legacy_payload(self):
        self.process()
        legacy = self.node._tracked_publisher.messages[-1]
        typed = self.node._fusion_array_publisher.messages[-1]
        self.assertEqual((typed.image_width, typed.image_height), (220, 160))
        self.assertEqual(typed.detections[0].target, legacy.detections[0])
        self.assertFalse(hasattr(legacy.detections[0], "measurement_source"))

    def test_all_four_modes_switch_and_reset(self):
        for mode in ("smart_tracker", "feature_tracker", "color_tracker", "fusion_tracker"):
            response = self.node._switch_tracking_mode(SwitchTrackingModeRequest(mode=mode))
            self.assertTrue(response.success, response.message)
            self.assertEqual(response.active_mode, mode)
            self.assertEqual(len(self.node._fusion_snapshots), 0)

    def test_manual_modes_are_visual_and_publish_same_selected_box(self):
        for mode in ("feature_tracker", "color_tracker"):
            self.node._tracking_mode = mode
            self.node._manual_active = True
            result = ManualTrackResult((50., 50., 40., 40.), .8, True, "ok", "kcf", 0)
            self.node._publish_manual_result(self.image, result)
            typed = self.node._fusion_array_publisher.messages[-1].detections[0]
            self.assertEqual(typed.measurement_source, "visual_tracker")
            self.assertTrue(typed.last_detector_stamp.is_zero())
            self.assertEqual(self.node._tracked_publisher.messages[-1].detections,
                             self.node._selected_detections_publisher.messages[-1].detections)
            state = self.node._state_publisher.messages[-1]
            self.assertFalse(state.measurement_current)
            self.assertTrue(state.control_measurement_ready)
            self.assertFalse(state.data_is_stale)

    def test_manual_failure_neither_publishes_nor_draws_stale_box(self):
        self.node._tracking_mode = "feature_tracker"
        result = ManualTrackResult((50., 50., 40., 40.), 0., False, "failed", "kcf", 1)
        self.node._publish_manual_result(self.image, result)
        self.assertFalse(self.node._tracked_publisher.messages[-1].detections)
        self.assertFalse(self.node._selected_detections_publisher.messages[-1].detections)
        state = self.node._state_publisher.messages[-1]
        self.assertTrue(state.data_is_stale)
        self.assertFalse(state.control_measurement_ready)
        self.assertEqual(state.geometry_type, "none")
        annotated = self.node._annotate_manual(self.frame.copy(), result)
        # Status text may change, but the old target rectangle must not be drawn.
        self.assertTrue(np.array_equal(annotated[45:100, 45:100], self.frame[45:100, 45:100]))

    def test_legacy_entry_cannot_mislabel_fusion_as_detector(self):
        self.node._tracking_mode = "smart_tracker"
        self.node._fusion_mode_enabled = False
        response = self.node._switch_tracking_mode(SwitchTrackingModeRequest(mode="fusion_tracker"))
        self.assertFalse(response.success)
        self.assertEqual(response.active_mode, "smart_tracker")
        self.assertEqual(self.node._tracking_mode, "smart_tracker")

    def test_slow_inference_does_not_block_capture_or_local_tracking(self):
        node = self.node
        node._backend_lock = threading.Lock()
        node._commit_lock = threading.RLock()
        node._epoch = 0
        node._model_busy = False
        node._image_mailbox = LatestMailbox()
        node._inference_mailbox = LatestMailbox()
        node._result_mailbox = LatestMailbox()
        node._publication_context = threading.local()
        node._pipeline_current = None
        node._last_input_stamp = rospy.Time(0)
        node._last_capture_stamp = rospy.Time(0)
        node._maximum_age, node._reject_out_of_order = 2., True
        node._capture_clock = CaptureClock()
        node._maximum_image_bytes = 33554432
        node._flow = FlowDetections()
        node._camera_motion = CameraMotion()
        node._motion_header = node._motion_frame = node._camera_info = node._tf_buffer = None
        node._controller = NODE._SmartTrackerController()
        node._rejected_frames = node._processed_frames = 0
        node._updater = SimpleNamespace(update=lambda: None)
        node._annotated_publisher = Publisher()
        node._tracker.backend.reset_tracking = lambda: None
        node._tracker.close = lambda: None
        calls = []
        def slow(frame, **kwargs):
            calls.append(time.monotonic())
            time.sleep(.09)
            return None, tuple(self.detections)
        node._tracker.backend.detect_and_track = slow
        self.process()
        self.select()
        node._workers = [threading.Thread(target=node._frame_loop), threading.Thread(target=node._inference_loop)]
        for worker in node._workers:
            worker.start()
        try:
            times = []
            for index in range(20):
                message = NODE._bgr_to_image(self.frame, copy.deepcopy(self.image.header))
                message.header.seq += index+1
                message.header.stamp = rospy.Time.from_sec(time.time())
                started = time.monotonic()
                node._image_callback(message)
                times.append(time.monotonic()-started)
                time.sleep(.025)
            time.sleep(.12)
            self.assertLess(max(times), .02)
            self.assertGreater(node._processed_frames, len(calls)*2)
            self.assertGreater(node._inference_mailbox.replaced, 0)
            self.assertFalse(node._last_error)
            self.assertTrue(node._selected_detections_publisher.messages[-1].detections)
        finally:
            node.close()
        self.assertFalse(any(worker.is_alive() for worker in node._workers))


if __name__ == "__main__":
    unittest.main()
