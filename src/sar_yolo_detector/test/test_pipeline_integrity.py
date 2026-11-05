"""Exercise the ROS click boundary through the original selected-target algorithms."""
import copy
import threading
import unittest
from collections import deque
from unittest import mock

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, RegionOfInterest

from sar_yolo_detector.msg import PerceptionIdentity
from sar_yolo_detector.box_fusion import BoxFusion
from sar_yolo_detector.manual_box_tracker import ManualBoxTracker
from sar_yolo_detector.pixeagle.parameters import Parameters
from sar_yolo_detector.pixeagle.smart_tracker import SmartTracker
from sar_yolo_detector.pixeagle.tracking_state_manager import TrackingStateManager
from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection
from sar_yolo_detector.srv import SelectFusionROIRequest
import sar_yolo_detector.pixeagle.smart_tracker as smart_module
from test_pixeagle_portable import _FakeBackend, _Controller
from test_fusion_adapter import NODE, Publisher


class SelectedPipelineIntegrityTest(unittest.TestCase):
    def setUp(self):
        rospy.rostime.set_rostime_initialized(True)
        self.frame = np.random.RandomState(31).randint(20, 220, (120, 160, 3), dtype=np.uint8)
        self.detection = NormalizedDetection(5, 2, .9, (40, 30, 80, 70), (60, 50))
        self.backend = _FakeBackend([self.detection])
        Parameters.configure_smart_tracker({
            'SMART_TRACKER_USE_GPU': False, 'SMART_TRACKER_CPU_MODEL_PATH': '/trusted/fake.pt',
            'ENABLE_KALMAN_FILTER': True, 'ENABLE_PREDICTION_BUFFER': True,
            'TRACKING_STRATEGY': 'hybrid', 'CLASS_MATCH_FLEXIBLE': False,
        })
        self.controller = _Controller()
        self.controller.current_frame = self.frame
        with mock.patch.object(smart_module, 'create_backend', return_value=self.backend):
            self.tracker = SmartTracker(self.controller)
        self.tracker.track_and_draw(self.frame, detections=[self.detection], draw=False)
        node = self.node = NODE.SmartTrackerNode.__new__(NODE.SmartTrackerNode)
        node._lock = threading.RLock()
        node._model_busy = False
        node._tracker, node._controller = self.tracker, self.controller
        node._tracking_mode = 'smart_tracker'
        node._manual_trackers = {}
        node._selected_track_id = node._selected_class_id = node._selected_last_bbox = None
        node._selected_track_id_is_stable = False
        node._fusion = BoxFusion()
        node._identity = PerceptionIdentity()
        node._state_publisher = Publisher()
        node._selected_detections_publisher = Publisher()
        node._last_processing_ms = node._inference_ms = 0.
        node._future_tolerance = .02
        node._fusion_snapshot_max_age = .75
        self.image = Image(width=160, height=120)
        self.image.header.frame_id = 'test_optical'
        self.image.header.stamp = rospy.Time.now()
        node._last_header = copy.deepcopy(self.image.header)
        node._smart_last_measurement_stamp = self.image.header.stamp-rospy.Duration(.1)
        node._smart_prediction_hold_sec = .5
        node._smart_selection_snapshots = deque([
            (copy.deepcopy(self.image.header), tuple([self.detection]), self.frame.shape[:2])])

    def tearDown(self):
        self.tracker.close()

    def click(self, x=60., y=50.):
        message = PointStamped(header=copy.deepcopy(self.image.header))
        message.point.x, message.point.y = x, y
        self.node._select_pixel_callback(message)

    def test_web_click_initializes_original_kalman_and_state_manager(self):
        self.click()
        self.assertEqual(self.tracker.tracking_manager.selected_track_id, 5)
        self.assertIsNotNone(self.tracker.tracking_manager.kalman)
        self.assertEqual(self.tracker.selected_object_id, self.node._selected_track_id)

    def test_cancel_clears_internal_manager_as_well_as_web_id(self):
        self.tracker.select_object_by_click(60, 50)
        self.node._remember_selected_detection(self.detection)
        self.click(-1, -1)
        self.assertIsNone(self.tracker.tracking_manager.selected_track_id)
        self.assertIsNone(self.tracker.selected_object_id)
        self.assertIsNone(self.node._selected_track_id)

    def test_click_for_another_camera_cannot_select_latest_frame(self):
        self.image.header.frame_id = 'other_optical'
        self.click()
        self.assertIsNone(self.node._selected_track_id)

    def test_spatial_reassociation_reaches_selected_ros_output(self):
        self.tracker.select_object_by_click(60, 50)
        self.node._remember_selected_detection(self.detection)
        replacement = NormalizedDetection(9, 2, .9, (42, 30, 82, 70), (62, 50))
        self.tracker.track_and_draw(self.frame, detections=[replacement], draw=False)
        self.assertEqual(self.tracker.selected_object_id, 9)
        selected = self.node._selected_detection_for_frame([replacement])
        self.assertIsNotNone(selected, 'validated manager reassociation was discarded at ROS output')
        self.assertEqual(selected.track_id, 9)
        self.assertEqual(self.node._selected_track_id, 9)

    def test_raw_id_cannot_bypass_reacquisition_gate(self):
        self.tracker.select_object_by_click(60, 50)
        self.node._remember_selected_detection(self.detection)
        self.tracker._last_measurement_current = False
        self.assertIsNone(self.node._selected_detection_for_frame([self.detection]))

    def test_unstable_nearest_box_cannot_bypass_reacquisition_gate(self):
        self.tracker.select_object_by_click(60, 50)
        self.node._remember_selected_detection(self.detection)
        self.node._selected_track_id_is_stable = False
        self.tracker._last_measurement_current = False
        self.assertIsNone(self.node._selected_detection_for_frame([self.detection]))

    def test_short_prediction_is_visible_but_never_a_control_measurement(self):
        self.tracker.select_object_by_click(60, 50)
        self.node._remember_selected_detection(self.detection)
        self.tracker.track_and_draw(self.frame, detections=[self.detection], draw=False)
        self.tracker.track_and_draw(self.frame, detections=[], draw=False)
        self.node._publish_state(self.image, self.tracker.get_output(), None)
        state = self.node._state_publisher.messages[-1]
        self.assertTrue(state.prediction_only)
        self.assertGreater(state.selected_bbox.size_x, 0., 'Kalman display geometry was discarded')
        self.assertFalse(state.measurement_current)
        self.assertFalse(state.control_measurement_ready)

    def test_external_detections_do_not_trigger_detector_twice(self):
        with mock.patch.object(self.backend, 'detect_and_track', wraps=self.backend.detect_and_track) as infer:
            detections = self.node._infer(self.frame)
            self.tracker.track_and_draw(self.frame, detections=detections, draw=False)
        self.assertEqual(infer.call_count, 1)

    def test_long_gap_does_not_keep_showing_old_prediction(self):
        self.tracker.select_object_by_click(60, 50)
        self.node._remember_selected_detection(self.detection)
        self.tracker.track_and_draw(self.frame, detections=[], draw=False)
        self.node._smart_last_measurement_stamp = self.image.header.stamp-rospy.Duration(2.)
        self.node._publish_state(self.image, self.tracker.get_output(), None)
        state = self.node._state_publisher.messages[-1]
        self.assertEqual(state.selected_bbox.size_x, 0.)
        self.assertFalse(state.control_measurement_ready)


class ColourBackgroundIntegrityTest(unittest.TestCase):
    def scene(self, present=True):
        frame = np.full((180, 240, 3), (35, 110, 45), dtype=np.uint8)
        if present:
            frame[50:110, 90:120] = (28, 28, 28)
            frame[54:106, 97:113] = (20, 20, 170)
        return frame

    def test_roi_background_hue_cannot_expand_into_whole_field(self):
        tracker = ManualBoxTracker({'tracker_type': 'color_tracker'})
        frame = self.scene()
        self.assertTrue(tracker.initialize(frame, (86, 46, 38, 68)).valid)
        for _ in range(15):
            result = tracker.update(frame)
            self.assertTrue(result.valid, result.reason)
            x, y, width, height = result.bbox
            self.assertLess(width*height, 38*68*2.5)
            self.assertAlmostEqual(x+width/2., 105., delta=6.)
            self.assertAlmostEqual(y+height/2., 80., delta=6.)

    def test_background_only_cannot_be_reported_as_target_after_disappearance(self):
        tracker = ManualBoxTracker({'tracker_type': 'color_tracker'})
        self.assertTrue(tracker.initialize(self.scene(), (86, 46, 38, 68)).valid)
        result = tracker.update(self.scene(False))
        self.assertFalse(result.valid, 'background leakage was counted as an observed target')


class ClassRecoveryIntegrityTest(unittest.TestCase):
    def test_upstream_class_correction_survives_short_prediction(self):
        manager=TrackingStateManager({'SMART_TRACKER_CLASS_CONSENSUS_ENABLED':True,
            'CLASS_MATCH_FLEXIBLE':True,'CLASS_COMPATIBILITY_GROUPS':[[0,1],[2]],
            'ENABLE_KALMAN_FILTER':True,'ENABLE_PREDICTION_BUFFER':True})
        manager.start_tracking(5,0,(40,30,80,70),.9,(60,50))
        iou=lambda a,b:1. if a==b else 0.
        active,matched=manager.update_tracking([[40,30,80,70,5,.9,1,1]],iou)
        self.assertTrue(active)
        self.assertEqual(manager.selected_class_id,1)
        active,predicted=manager.update_tracking([],iou)
        self.assertTrue(active)
        self.assertEqual(predicted['class_id'],1)
        self.assertTrue(predicted['prediction_only'])

    def test_recycled_stable_id_cannot_teleport_selected_target(self):
        manager = TrackingStateManager({})
        manager.start_tracking(5,0,(40,30,80,70),.9,(60,50))
        self.assertIsNone(manager._match_by_id([[700,500,740,540,5,.99,0]]))
        self.assertIsNone(manager._match_by_id([[0,0,700,500,5,.99,0]]))
        self.assertIsNotNone(manager._match_by_id([[45,32,85,72,5,.8,0]]))

    def test_ground_selection_never_rebinds_to_drone(self):
        manager = TrackingStateManager({
            'CLASS_MATCH_FLEXIBLE': True,
            'CLASS_COMPATIBILITY_GROUPS': [[0, 1], [2]],
        })
        manager.start_tracking(5, 0, (40, 30, 80, 70), .9, (60, 50))
        drone_same_id = [40, 30, 80, 70, 5, .9, 2]
        drone_new_id = [40, 30, 80, 70, 9, .9, 2]
        tank_new_id = [40, 30, 80, 70, 9, .9, 1]
        iou = lambda a, b: 1. if a == b else 0.
        self.assertIsNone(manager._match_by_id([drone_same_id]))
        self.assertIsNone(manager._match_by_spatial([drone_new_id], iou))
        self.assertIsNotNone(manager._match_by_spatial([tank_new_id], iou))


class ManualFrameBindingTest(unittest.TestCase):
    def test_roi_uses_displayed_frame_and_follows_cached_frames(self):
        rospy.rostime.set_rostime_initialized(True)
        node = NODE.SmartTrackerNode.__new__(NODE.SmartTrackerNode)
        node._lock = threading.RLock()
        node._tracking_mode = 'color_tracker'
        node._model_busy = False
        node._manual_trackers = {'color_tracker': ManualBoxTracker({'tracker_type':'color_tracker'})}
        node._manual_snapshots = deque()
        node._manual_pending_roi = None
        node._manual_active = False
        node._manual_track_id = 900000000
        node._fusion_snapshot_max_age = .75
        node._fusion_snapshot_max_frames = 8
        node._fusion_snapshot_max_bytes = 10*1024*1024
        node._future_tolerance = .02
        first = Image(width=240, height=180)
        first.header.frame_id = 'down_optical'
        first.header.stamp = rospy.Time.now()
        frame = ColourBackgroundIntegrityTest().scene()
        node._cache_manual_snapshot(first, frame, None)
        second = copy.deepcopy(first)
        second.header.stamp = first.header.stamp+rospy.Duration(.1)
        shifted = np.roll(frame, 3, axis=1)
        node._cache_manual_snapshot(second, shifted, None)
        request = SelectFusionROIRequest()
        request.header = copy.deepcopy(first.header)
        request.roi = RegionOfInterest(x_offset=86, y_offset=46, width=38, height=68)
        result = node._select_manual_roi(request)
        self.assertTrue(result.success, result.message)
        self.assertTrue(node._manual_active)
        box = node._manual_trackers['color_tracker']._last_bbox
        self.assertAlmostEqual(box[0]+box[2]/2., 108., delta=6.)
        original_tracker = node._manual_trackers['color_tracker']
        request.header.frame_id = 'wrong_camera'
        rejected = node._select_manual_roi(request)
        self.assertFalse(rejected.success)
        self.assertIs(node._manual_trackers['color_tracker'], original_tracker)


if __name__ == '__main__':
    unittest.main()
