#!/usr/bin/env python3
"""Exercise actual mode manager callbacks with deterministic image evidence."""
from collections import OrderedDict, deque
from copy import deepcopy
import importlib.util
from pathlib import Path
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Header
from xd_uav_track.msg import (PerceptionTarget, TrackState, TrackStateArray,
                             SetPerceptionModeCommand, DetectionArray, DetectionCandidate)
from xd_uav_track.perception_policy import detector_rate, image_signature, evidence_gate
from xd_uav_track.trajectory import TargetTrajectory

path = Path(__file__).resolve().parents[1]/"scripts/perception_mode_manager.py"
spec = importlib.util.spec_from_file_location("perception_manager_test", str(path))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(deepcopy(message))


def manager_fixture():
    # Unit callbacks need a wall-time ROS clock, but never a ROS master.
    rospy.rostime.set_rostime_initialized(True)
    manager = module.PerceptionModeManager.__new__(module.PerceptionModeManager)
    manager._lock = threading.RLock()
    manager._bridge = CvBridge()
    manager._mode = "fusion"
    manager._session_id = "test-session"
    manager._version = 1
    manager._class_names = ["car", "ar-car", "tank", "m142"]
    manager._frame_cache_size = 6
    manager._source_clock_reset_jump = 5.0
    manager._frame_tolerance = .15
    manager._frames = {"fixed": deque(maxlen=6), "gimbal": deque(maxlen=6)}
    manager._frame_receipts = {}
    manager._presentation_ms = deque(maxlen=128)
    manager._pixel_history = {}
    manager._latest_targets = {}
    manager._latest_headers = {}
    manager._track_history = {}
    manager._sot_history = deque(maxlen=6)
    manager._selected_source = ""
    manager._selected_uid = ""
    manager._selected_track_id = -1
    manager._registration_number = 0
    manager._visual_targets = {}
    manager._background_visual = OrderedDict()
    manager._identity_aliases = OrderedDict()
    manager._image_evidence = OrderedDict()
    manager._prediction_horizon = 0.5
    manager._position_frame = "base_link_frd"
    manager._position_timeout = 1.0
    manager._stale_notified = set()
    manager._visual_ttl = 60.0
    manager._identity_client = lambda request: None
    manager._manual_tracker = None
    manager._manual_target = None
    manager._sot_manual = False
    manager._identity_attached = False
    manager._identity_update_stamp = None
    manager._identity_bind_stamp = None
    manager._manual_last_valid_wall = 0.0
    manager._reference_signature = None
    manager._fusion_reason = ""
    manager._fusion_conflict = False
    manager._last_error = ""
    manager._last_source_wall = {}
    manager._source_timeout = 1.0
    manager._model = {"epoch":1, "id":"train7"}
    manager._targets_pub = Publisher()
    manager._trajectory = TargetTrajectory()
    manager._trajectory_pub = Publisher()
    manager._trajectory_publish_interval = .5
    manager._last_trajectory_publish = 0.0
    manager._command_cache = OrderedDict()
    manager._model_transaction = None
    manager._transaction_event = threading.Event()
    manager._closing = False
    manager._result_pub = Publisher()
    manager._publish_state = lambda: None
    return manager


def frame(shift=0):
    image = np.zeros((120,180,3), np.uint8)
    pattern = np.random.RandomState(42).randint(20,220,(24,36,3)).astype(np.uint8)
    image[40:64,50+shift:86+shift] = pattern
    return image


def target(track_id=7, shift=0):
    item=PerceptionTarget()
    item.track_id=track_id
    item.target_uid="test-session:{}".format(track_id)
    item.bbox_xyxy=[50+shift,40,86+shift,64]
    item.bbox_cxcywh_norm=[(68+shift)/180,52/120,36/180,24/120]
    item.class_id=2; item.class_name="tank"; item.class_valid=True
    item.lifecycle_state="confirmed"; item.measurement_source="detector"
    item.detection_confidence=.9
    return item


class PerceptionModesTest(unittest.TestCase):
    def test_intermediate_pixel_selection_attaches_original_detector_capture(self):
        manager = manager_fixture()
        detected, displayed = rospy.Time(10), rospy.Time.from_sec(10.25)
        manager._frames['gimbal'].extend([(1, detected, frame()), (2, displayed, frame(3))])
        manager._latest_headers['gimbal'] = Header(seq=2, stamp=displayed)
        anchor = target()
        pixel = target(shift=3); pixel.measurement_source = 'visual_tracker'
        manager._cache_selection_snapshot('gimbal', detected, [anchor])
        replacement = deepcopy(anchor); replacement.measurement_source = 'visual_tracker'
        manager._cache_selection_snapshot('gimbal', detected, [replacement])
        self.assertEqual('detector', manager._track_history['gimbal'][0][1][0].measurement_source)
        requests = []
        def service(request):
            requests.append(deepcopy(request))
            return SimpleNamespace(success=request.observation.header.stamp == detected,
                                   message='confirmed detector capture missing', global_id=7)
        manager._identity_client = service
        self.assertTrue(manager._select_existing('gimbal', pixel, 'template'))
        self.assertTrue(manager._identity_attached)
        self.assertEqual(detected, manager._identity_update_stamp)
        self.assertEqual([displayed, detected], [r.observation.header.stamp for r in requests])
        self.assertEqual([50,40,86,64], requests[-1].observation.bbox)
        self.assertEqual([53,40,89,64], manager._manual_target.bbox_xyxy)
        self.assertEqual(7, manager._selected_track_id)

    def test_historical_attach_rejects_expired_and_other_source_evidence(self):
        manager = manager_fixture()
        old, current = rospy.Time(10), rospy.Time(12)
        manager._frames['gimbal'].extend([(1, old, frame()), (2, current, frame())])
        manager._frames['fixed'].append((1,current,frame()))
        manager._cache_selection_snapshot('gimbal', old, [target()])
        manager._cache_selection_snapshot('fixed', current, [target()])
        requests=[]
        manager._identity_client=lambda r: (requests.append(deepcopy(r)) or
            SimpleNamespace(success=False,message='confirmed detector capture missing'))
        pixel=target(); pixel.measurement_source='visual_tracker'
        manager._attach_selected_identity('gimbal',pixel,current)
        self.assertFalse(manager._identity_attached)
        self.assertEqual([current],[r.observation.header.stamp for r in requests])
        self.assertTrue(all(r.image_source=='gimbal' for r in requests))

    def test_missing_attach_is_retried_only_after_verified_same_id_detection(self):
        manager=manager_fixture(); initial=rospy.Time(10)
        manager._frames['gimbal'].append((1,initial,frame()))
        manager._latest_headers['gimbal']=Header(stamp=initial)
        requests=[]
        def service(request):
            requests.append(deepcopy(request))
            return SimpleNamespace(success=request.observation.header.stamp>initial,
                                   message='confirmed capture anchor missing',global_id=7)
        manager._identity_client=service
        with patch.object(module.time,'monotonic',return_value=200.):
            self.assertTrue(manager._select_existing('gimbal',target(),'template'))
        self.assertFalse(manager._identity_attached)
        stamp=initial+rospy.Duration(.25); header=Header(stamp=stamp)
        manager._frames['gimbal'].append((2,stamp,frame()))
        manager._update_sot(frame(),stamp)
        with patch.object(module.time,'monotonic',return_value=200.6):
            manager._validate_anchor(TrackStateArray(header=header,image_source='gimbal'),[target()])
        self.assertTrue(manager._identity_attached)
        self.assertEqual(['attach','attach'],[r.action for r in requests])
        self.assertEqual(stamp,manager._identity_update_stamp)

    def _selected_with_identity_owner(self):
        manager = manager_fixture()
        initial = rospy.Time(10)
        manager._frames['gimbal'].append((1, initial, frame()))
        manager._latest_headers['gimbal'] = Header(seq=1, stamp=initial)
        requests = []
        def service(request):
            requests.append(deepcopy(request))
            binds = sum(r.action == 'bind' for r in requests)
            return SimpleNamespace(success=True, global_id=7, message='',
                                   bound=request.action == 'bind' and binds >= 3)
        manager._identity_client = service
        self.assertTrue(manager._select_existing('gimbal', target(), 'template'))
        self.assertTrue(manager._identity_attached)
        self.assertFalse(manager._sot_manual)
        self.assertEqual('attach', requests[0].action)
        self.assertEqual(7, manager._selected_track_id)
        return manager, requests

    def test_id_fork_requires_owner_confirmation_then_preserves_public_id(self):
        manager, requests = self._selected_with_identity_owner()
        for index in range(1, 4):
            stamp = rospy.Time.from_sec(10.+index*.25)
            manager._frames['gimbal'].append((index+1, stamp, frame()))
            manager._update_sot(frame(), stamp)
            header = Header(seq=index+1, stamp=stamp)
            candidate = target(8)
            manager._latest_targets['gimbal'] = [candidate]
            manager._latest_headers['gimbal'] = header
            manager._validate_anchor(TrackStateArray(header=header, image_source='gimbal'), [candidate])
            if index < 3:
                # Associated raw boxes can arrive after TrackStateArray. The
                # second callback for this capture must not reset/increment
                # the service's consecutive-frame confirmation counter.
                manager._validate_anchor(TrackStateArray(header=header, image_source='gimbal'), [candidate])
            manager._publish_targets('gimbal', header, 180, 120)
            output = manager._targets_pub.messages[-1].targets
            confirmed = [t for t in output if t.lifecycle_state == 'confirmed']
            if index < 3:
                self.assertTrue(manager._fusion_conflict)
                self.assertEqual([8], [t.track_id for t in confirmed])
                self.assertEqual('lost', next(t for t in output if t.track_id == 7).lifecycle_state)
            else:
                self.assertFalse(manager._fusion_conflict)
                self.assertEqual([7], [t.track_id for t in confirmed])
                self.assertEqual('test-session:7', confirmed[0].target_uid)
                self.assertEqual([50, 40, 86, 64], confirmed[0].bbox_xyxy)
        self.assertEqual(3, sum(r.action == 'bind' for r in requests))

    def test_partial_box_and_similar_competitors_do_not_force_identity_merge(self):
        for partial in (False, True):
            manager, requests = self._selected_with_identity_owner()
            stamp = rospy.Time.from_sec(10.25)
            manager._frames['gimbal'].append((2, stamp, frame()))
            manager._update_sot(frame(), stamp)
            if partial:
                visual = manager._sot_at(stamp)
                visual.bbox_xyxy = [55, 42, 73, 57]
                visual.bbox_cxcywh_norm = [64/180, 49.5/120, 18/180, 15/120]
                manager._raw_sot = (stamp, deepcopy(visual))
            candidates = [target(8)] if partial else [target(8), target(9, 1)]
            header = Header(seq=2, stamp=stamp)
            manager._latest_targets['gimbal'] = candidates
            manager._latest_headers['gimbal'] = header
            manager._validate_anchor(TrackStateArray(header=header, image_source='gimbal'), candidates)
            self.assertTrue(manager._fusion_conflict)
            self.assertEqual('ambiguous_competitor', manager._fusion_reason)
            self.assertFalse(any(r.action == 'bind' for r in requests))
            manager._publish_targets('gimbal', header, 180, 120)
            self.assertFalse(any(t.selected and t.lifecycle_state == 'confirmed'
                                 for t in manager._targets_pub.messages[-1].targets))

    def test_owner_unavailable_cannot_be_bypassed_by_overlapping_new_id(self):
        manager, _ = self._selected_with_identity_owner()
        manager._identity_attached = False
        stamp = rospy.Time.from_sec(10.25)
        manager._frames['gimbal'].append((2, stamp, frame()))
        manager._update_sot(frame(), stamp)
        manager._validate_anchor(TrackStateArray(header=Header(stamp=stamp), image_source='gimbal'), [target(8)])
        self.assertEqual('identity_unavailable', manager._fusion_reason)
        self.assertEqual(7, manager._selected_track_id)
        self.assertEqual('lost', manager._manual_target.lifecycle_state)

    def test_stale_sot_box_cannot_be_promoted_to_rebinding_evidence(self):
        manager, requests = self._selected_with_identity_owner()
        stamp = rospy.Time.from_sec(10.25)
        manager._frames['gimbal'].append((2, stamp, frame()))
        stale = deepcopy(manager._manual_target)
        stale.lifecycle_state = 'lost'; stale.measurement_source = 'stale'
        manager._sot_history.append((stamp, stale))
        manager._raw_sot = None
        manager._validate_anchor(TrackStateArray(header=Header(stamp=stamp), image_source='gimbal'), [target(8)])
        self.assertEqual('identity_unavailable', manager._fusion_reason)
        self.assertFalse(any(r.action == 'bind' for r in requests))

    def test_selected_part_and_full_object_detections_are_ambiguous_not_two_locks(self):
        manager, requests = self._selected_with_identity_owner()
        stamp = rospy.Time.from_sec(10.25)
        manager._frames['gimbal'].append((2, stamp, frame()))
        part = target(7)
        part.bbox_xyxy = [55,42,73,57]
        part.bbox_cxcywh_norm = [64/180,49.5/120,18/180,15/120]
        visual = deepcopy(part); visual.measurement_source = 'visual_tracker'
        visual.tracking_quality = 1.
        manager._sot_history.append((stamp, visual)); manager._raw_sot=(stamp,deepcopy(visual))
        manager._validate_anchor(TrackStateArray(header=Header(stamp=stamp), image_source='gimbal'), [part,target(8)])
        self.assertEqual('ambiguous_competitor', manager._fusion_reason)
        self.assertEqual('lost', manager._manual_target.lifecycle_state)
        self.assertFalse(any(r.action == 'bind' for r in requests))

    def test_verified_full_object_anchor_repairs_gradual_sot_shrink(self):
        manager = manager_fixture()
        stamp = rospy.Time(10)
        manager._frames['gimbal'].append((1, stamp, frame()))
        manager._latest_headers['gimbal'] = Header(stamp=stamp)
        manager._select_existing('gimbal', target(), 'template')
        original = manager._manual_tracker
        visual = deepcopy(manager._manual_target)
        visual.bbox_xyxy = [55, 43, 82, 61]
        visual.bbox_cxcywh_norm = [68.5/180, 52/120, 27/180, 18/120]
        manager._sot_history.clear(); manager._sot_history.append((stamp, visual))
        peer = target(); peer.measurement_source = 'visual_tracker'
        peer.tracking_quality = .9; peer.last_detection_stamp = rospy.Time(9)
        manager._pixel_targets = {'gimbal': (stamp, [peer])}
        manager._validate_anchor(TrackStateArray(header=Header(stamp=stamp), image_source='gimbal'), [target()])
        self.assertIsNot(original, manager._manual_tracker)
        self.assertEqual([50, 40, 86, 64], manager._manual_target.bbox_xyxy)

    def test_authorized_remap_includes_pixel_and_selected_output_caches(self):
        manager = manager_fixture(); stamp = rospy.Time(10)
        manager._pixel_targets = {'gimbal': (stamp, [target(8)])}
        manager._pixel_history = {'gimbal': deque([(stamp, [target(8)])])}
        manager._raw_sot = (stamp, target(8))
        manager._selected_output = (time.monotonic(), target(8))
        manager._remap_identity(8, 7)
        self.assertEqual(7, manager._pixel_targets['gimbal'][1][0].track_id)
        self.assertEqual(7, manager._pixel_history['gimbal'][0][1][0].track_id)
        self.assertEqual(7, manager._raw_sot[1].track_id)
        self.assertEqual(7, manager._selected_output[1].track_id)

    def test_authorized_remap_prefers_measurement_over_dormant_fragment(self):
        manager = manager_fixture()
        stale = target(7); stale.lifecycle_state='lost'; stale.measurement_source='stale'
        manager._latest_targets={'gimbal': [target(8),stale]}
        manager._remap_identity(8,7)
        self.assertEqual(1,len(manager._latest_targets['gimbal']))
        self.assertEqual('detector',manager._latest_targets['gimbal'][0].measurement_source)

    def test_identity_evidence_at_image_edge_uses_same_frame_visible_pixels(self):
        manager = manager_fixture(); stamp = rospy.Time(10)
        manager._frames['gimbal'].append((1,stamp,frame()))
        edge = target(); edge.bbox_xyxy = [-2,40,35,64]
        request = manager._identity_request('update',edge,'gimbal',stamp)
        self.assertEqual([0,40,35,64],request.observation.bbox)
        self.assertAlmostEqual(35/180,request.observation.normalized_bbox[2])
        self.assertEqual([-2,40,35,64],edge.bbox_xyxy)
        expired = manager._identity_request('update',edge,'gimbal',rospy.Time(8))
        self.assertEqual([-2,40,35,64],expired.observation.bbox,
                         'an unrelated frame cannot supply evidence for clamping')

    def test_manual_roi_float32_image_edge_rounding_is_not_a_false_rejection(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames['gimbal'].append((1,stamp,frame()))
        manager._identity_client=lambda request: SimpleNamespace(success=True,global_id=201,message='')
        x=float(np.float32(1/3)); width=float(np.float32(2/3))
        self.assertGreater(x+width,1.)
        command=SimpleNamespace(image_source='gimbal',frame_seq=1,capture_stamp=stamp,
            roi_norm=[x,0.,width,1.],tracker_algorithm='template')
        self.assertTrue(manager._initialize_roi(command),manager._last_error)
        self.assertLessEqual(manager._manual_target.bbox_xyxy[2],180)
        command.roi_norm=[x,0.,.7,1.]
        self.assertFalse(manager._initialize_roi(command))

    def test_rejecting_detector_geometry_preserves_current_verified_sot(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames['fixed'].append((1,stamp,frame()))
        manager._latest_headers['fixed']=Header(stamp=stamp)
        manager._select_existing('fixed',target(),'template')
        current=target(); current.measurement_source='visual_tracker'; current.tracking_quality=1.
        manager._raw_sot=(stamp,current)
        manager._sot_history.clear(); manager._sot_history.append((stamp,current))
        manager._validate_anchor(TrackStateArray(header=Header(stamp=stamp),image_source='fixed'),[target(7,25)])
        self.assertFalse(manager._fusion_conflict)
        self.assertEqual('detector_geometry_correction_rejected',manager._fusion_reason)
        self.assertEqual('confirmed',manager._manual_target.lifecycle_state)

    def test_same_capture_gate_is_scale_aware_but_overlap_remains_strict(self):
        signature=np.ones(4,dtype=float)/2.
        self.assertFalse(evidence_gate([0,0,100,50],[16,0,116,50],0.,signature,signature)[0])
        self.assertTrue(evidence_gate([0,0,100,50],[16,0,116,50],0.,signature,signature,same_capture=True)[0])
        self.assertFalse(evidence_gate([0,0,100,50],[50,0,150,50],0.,signature,signature,same_capture=True)[0])
        self.assertFalse(evidence_gate([0,0,100,50],[16,0,116,50],.1,signature,signature,same_capture=True)[0])

    def test_geometry_continuation_cannot_override_hard_identity_conflict(self):
        for reason in ('ambiguous_competitor','appearance_conflict','appearance_unavailable'):
            with self.subTest(reason=reason):
                manager=manager_fixture(); stamp=rospy.Time(10)
                manager._frames['fixed'].append((1,stamp,frame()))
                manager._latest_headers['fixed']=Header(stamp=stamp)
                manager._select_existing('fixed',target(),'template')
                current=target(); current.measurement_source='visual_tracker'; current.tracking_quality=1.
                manager._raw_sot=(stamp,current)
                manager._sot_history.clear(); manager._sot_history.append((stamp,current))
                manager._fusion_conflict=True; manager._fusion_reason=reason
                peer=deepcopy(current); peer.last_detection_stamp=rospy.Time(9)
                manager._pixel_targets={'fixed':(stamp,[peer])}
                for _ in range(2):
                    manager._validate_anchor(TrackStateArray(header=Header(stamp=stamp),image_source='fixed'),[target(7,25)])
                    self.assertTrue(manager._fusion_conflict)
                    self.assertEqual(reason,manager._fusion_reason)
                    self.assertEqual('lost',manager._manual_target.lifecycle_state)

    def test_geometry_continuation_requires_actual_current_sot(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames['fixed'].append((1,stamp,frame()))
        manager._latest_headers['fixed']=Header(stamp=stamp)
        manager._select_existing('fixed',target(),'template')
        old=target(); old.measurement_source='visual_tracker'; old.tracking_quality=1.
        manager._raw_sot=(rospy.Time(9),old)
        stale=deepcopy(old); stale.lifecycle_state='lost'; stale.measurement_source='stale'
        manager._sot_history.clear(); manager._sot_history.append((stamp,stale))
        manager._validate_anchor(TrackStateArray(header=Header(stamp=stamp),image_source='fixed'),[target(7,25)])
        self.assertTrue(manager._fusion_conflict)
        self.assertEqual('lost',manager._manual_target.lifecycle_state)

    def test_two_fresh_metric_anchors_recover_same_entity_after_large_turn(self):
        manager=manager_fixture()
        initial=rospy.Time(10)
        manager._frames['fixed'].append((1,initial,frame()))
        manager._latest_headers['fixed']=Header(seq=1,stamp=initial)
        manager._select_existing('fixed',target(),'template')
        original=manager._manual_tracker
        for seconds in (10.5,10.5,10.75):
            stamp=rospy.Time.from_sec(seconds)
            if stamp != manager._frames['fixed'][-1][1]:
                manager._frames['fixed'].append((2,stamp,frame(25)))
                manager._sot_history.append((stamp,deepcopy(manager._manual_target)))
            candidate=target(7,25)
            candidate.position_valid=True
            candidate.position_frame=manager._position_frame
            candidate.position_stamp=stamp
            candidate.position_xyz=[10.,0.,30.]
            candidate.position_covariance=[.5,0.,0.,0.,.5,0.,0.,0.,.5]
            message=TrackStateArray(header=Header(stamp=stamp),image_source='fixed')
            manager._validate_anchor(message,[candidate])
            if seconds==10.5:
                self.assertIs(original,manager._manual_tracker)
                self.assertEqual(1,manager._metric_recovery[2])
        self.assertEqual('metric_identity_recovery',manager._fusion_reason)
        self.assertFalse(manager._fusion_conflict)
        self.assertIsNot(original,manager._manual_tracker)
        self.assertEqual(7,manager._manual_target.track_id)
        self.assertEqual([75,40,111,64],manager._manual_target.bbox_xyxy)

    def test_metric_recovery_rejects_wrong_frame_old_stamp_and_uncertainty(self):
        manager=manager_fixture()
        manager._selected_track_id=7; manager._selected_uid='test-session:7'
        candidate=target(7); candidate.position_valid=True
        candidate.position_stamp=rospy.Time(10); candidate.position_frame=manager._position_frame
        candidate.position_covariance=[.5,0.,0.,0.,.5,0.,0.,0.,.5]
        self.assertFalse(manager._metric_anchor_verified(candidate,rospy.Time(10)))
        candidate.position_frame='camera'
        self.assertFalse(manager._metric_anchor_verified(candidate,rospy.Time(10.2)))
        candidate.position_frame=manager._position_frame
        self.assertFalse(manager._metric_anchor_verified(candidate,rospy.Time(10.2)))
        candidate.position_stamp=rospy.Time(10.2)
        candidate.position_covariance=[9.,0.,0.,0.,9.,0.,0.,0.,9.]
        self.assertFalse(manager._metric_anchor_verified(candidate,rospy.Time(10.2)))
    def test_geometry_rejection_cannot_poison_future_independent_sot(self):
        manager=manager_fixture()
        stamp=rospy.Time.now()
        header=Header(seq=2,stamp=stamp)
        manager._frames['fixed'].append((2,stamp,frame()))
        manager._selected_source='fixed'; manager._selected_track_id=7
        manager._selected_uid='test-session:7'
        raw=target(); raw.measurement_source='visual_tracker'
        stale=deepcopy(raw); stale.lifecycle_state='lost'; stale.measurement_source='stale'
        manager._manual_target=stale
        manager._sot_history.append((stamp,stale))
        manager._raw_sot=(stamp,raw)
        manager._fusion_conflict=True; manager._fusion_reason='motion_conflict'
        manager._pixel_worker=SimpleNamespace(generation=1)
        manager._pixel_targets={}
        peer=deepcopy(raw); peer.tracking_quality=.9
        peer.last_detection_stamp=stamp-rospy.Duration(.5)
        manager._pixel_callback('fixed',header,180,120,[peer],1,1)
        self.assertFalse(manager._fusion_conflict)
        self.assertEqual('visual_consensus_recovered',manager._fusion_reason)
        self.assertEqual('confirmed',manager._targets_pub.messages[-1].targets[0].lifecycle_state)

    def test_identity_ambiguity_cannot_be_cleared_by_visual_consensus(self):
        manager=manager_fixture()
        stamp=rospy.Time.now(); header=Header(seq=2,stamp=stamp)
        manager._frames['fixed'].append((2,stamp,frame()))
        manager._selected_source='fixed'; manager._selected_track_id=7
        manager._selected_uid='test-session:7'
        raw=target(); raw.measurement_source='visual_tracker'
        stale=deepcopy(raw); stale.lifecycle_state='lost'; stale.measurement_source='stale'
        manager._manual_target=stale; manager._sot_history.append((stamp,stale))
        manager._raw_sot=(stamp,raw)
        manager._fusion_conflict=True; manager._fusion_reason='ambiguous_competitor'
        manager._pixel_worker=SimpleNamespace(generation=1); manager._pixel_targets={}
        peer=deepcopy(raw); peer.tracking_quality=.9
        peer.last_detection_stamp=stamp-rospy.Duration(.5)
        manager._pixel_callback('fixed',header,180,120,[peer],1,1)
        self.assertTrue(manager._fusion_conflict)
        self.assertEqual('stale',manager._targets_pub.messages[-1].targets[0].measurement_source)

    def test_detector_outlier_does_not_stop_two_agreeing_current_visual_observers(self):
        manager = manager_fixture()
        stamp = rospy.Time(10)
        manager._frames['fixed'].append((1,stamp,frame()))
        manager._latest_headers['fixed'] = Header(seq=1,stamp=stamp)
        manager._select_existing('fixed',target(),'template')
        peer = target(7)
        peer.measurement_source = 'visual_tracker'
        peer.tracking_quality = .9
        peer.last_detection_stamp = rospy.Time(9)
        manager._pixel_targets={'fixed':(stamp,[peer])}
        signature = manager._reference_signature.copy()
        original = manager._manual_tracker
        message = TrackStateArray(header=Header(stamp=stamp),image_source='fixed')
        manager._validate_anchor(message,[target(7,25)])
        self.assertFalse(manager._fusion_conflict)
        self.assertEqual('detector_outlier_geometry_conflict',manager._fusion_reason)
        self.assertEqual('confirmed',manager._manual_target.lifecycle_state)
        self.assertIs(original,manager._manual_tracker)
        np.testing.assert_array_equal(signature,manager._reference_signature)

    def test_anchor_validation_uses_raw_measurement_not_lagging_filter_box(self):
        manager = manager_fixture()
        initial = rospy.Time(10)
        manager._frames['fixed'].append((1,initial,frame()))
        manager._latest_headers['fixed'] = Header(seq=1,stamp=initial)
        manager._select_existing('fixed',target(),'template')
        stamp = rospy.Time(10.5)
        header = Header(seq=2,stamp=stamp)
        manager._frames['fixed'].append((2,stamp,frame(25)))
        manager._manual_target = target(7,25)
        manager._sot_history.append((stamp,target(7,25)))
        manager._pixel_anchors = {'fixed': deque([DetectionArray(header=header,
            image_source='fixed',image_width=180,image_height=120,model_epoch=1,
            candidates=[DetectionCandidate(track_id=7,has_bbox=True,bbox=[75,40,111,64],
                confidence=.9,measurement_source='detector')])])}
        original = manager._manual_tracker
        message = TrackStateArray(header=header,image_source='fixed')
        manager._validate_anchor(message,[target(7)])  # Kalman lags 25 pixels.
        self.assertEqual('accepted',manager._fusion_reason)
        self.assertFalse(manager._fusion_conflict)
        self.assertIs(original,manager._manual_tracker)
        self.assertEqual([75,40,111,64],manager._sot_at(stamp).bbox_xyxy)

    def test_failed_sot_cannot_erase_same_frame_global_observation(self):
        manager = manager_fixture()
        stamp = rospy.Time.now()
        header = Header(seq=1, stamp=stamp)
        manager._selected_source = 'fixed'
        manager._selected_track_id = 7
        manager._selected_uid = 'test-session:7'
        manager._latest_headers['fixed'] = header
        observed = target(7)
        observed.last_detection_stamp = stamp
        manager._latest_targets['fixed'] = [observed, target(8, 60)]
        failed = deepcopy(observed)
        failed.lifecycle_state = 'lost'
        failed.measurement_source = 'stale'
        manager._manual_target = failed
        manager._sot_history.append((stamp, failed))
        manager._publish_targets('fixed', header, 180, 120)
        output = manager._targets_pub.messages[-1]
        selected = next(t for t in output.targets if t.track_id == 7)
        self.assertEqual('detector', selected.measurement_source)
        self.assertTrue(selected.selected)
        self.assertEqual({7,8}, {t.track_id for t in output.targets})
        self.assertEqual('TRACKING', manager._operating_state())
        manager._selected_output = (time.monotonic()-2, selected)
        self.assertEqual('LOST', manager._operating_state())

    def test_same_capture_detector_miss_cannot_erase_independent_lk(self):
        for source in ('fixed','gimbal'):
            for mode in ('fusion','track'):
                manager=manager_fixture(); manager._mode=mode
                stamp=rospy.Time.now(); header=Header(seq=2,stamp=stamp)
                manager._selected_source=source; manager._selected_track_id=7
                manager._selected_uid='test-session:7'
                manager._latest_headers[source]=header
                missing=target(); missing.lifecycle_state='occluded'
                missing.measurement_source='prediction'
                failed=deepcopy(missing); failed.lifecycle_state='lost'; failed.measurement_source='stale'
                manager._manual_target=failed; manager._sot_history.append((stamp,failed))
                observed=target(shift=3); observed.measurement_source='visual_tracker'
                observed.tracking_quality=.8
                observed.last_detection_stamp=stamp-rospy.Duration(.25)
                manager._pixel_targets={source:(stamp,[observed])}
                # LK first, then a late YOLO miss: both revisions must keep
                # the actual visual observation, never a stationary old box.
                manager._publish_targets(source,header,180,120)
                manager._latest_targets[source]=[missing,target(8,60)]
                manager._publish_targets(source,header,180,120)
                for output in manager._targets_pub.messages:
                    selected=next(t for t in output.targets if t.track_id==7)
                    self.assertTrue(selected.selected)
                    self.assertEqual('confirmed',selected.lifecycle_state)
                    self.assertEqual('visual_tracker',selected.measurement_source)
                    self.assertEqual([53,40,89,64],selected.bbox_xyxy)
                    self.assertEqual(observed.last_detection_stamp,selected.last_detection_stamp)
                    self.assertFalse(selected.position_valid)
                manager._fusion_conflict=True; manager._fusion_reason='ambiguous_competitor'
                manager._publish_targets(source,header,180,120)
                selected=next(t for t in manager._targets_pub.messages[-1].targets if t.track_id==7)
                self.assertEqual('lost',selected.lifecycle_state)

    def test_weak_or_wrong_capture_pixels_cannot_replace_detector_miss(self):
        for issue in ('quality','prediction','wrong_capture','same_detection_capture'):
            manager=manager_fixture(); stamp=rospy.Time.now(); header=Header(stamp=stamp)
            manager._selected_source='gimbal'; manager._selected_track_id=7
            manager._selected_uid='test-session:7'; manager._latest_headers['gimbal']=header
            failed=target(); failed.lifecycle_state='lost'; failed.measurement_source='stale'
            manager._manual_target=failed; manager._sot_history.append((stamp,failed))
            manager._latest_targets['gimbal']=[deepcopy(failed)]
            pixel=target(); pixel.measurement_source='visual_tracker'; pixel.tracking_quality=.8
            pixel.last_detection_stamp=stamp-rospy.Duration(.25)
            captured=stamp
            if issue=='quality': pixel.tracking_quality=.59
            elif issue=='prediction': pixel.measurement_source='prediction'
            elif issue=='wrong_capture': captured=stamp-rospy.Duration(.25)
            else: pixel.last_detection_stamp=stamp
            manager._pixel_targets={'gimbal':(captured,[pixel])}
            manager._publish_targets('gimbal',header,180,120)
            selected=next(t for t in manager._targets_pub.messages[-1].targets if t.track_id==7)
            self.assertEqual('lost',selected.lifecycle_state,issue)

    def test_conflict_cannot_be_bypassed_by_output_fallback(self):
        manager = manager_fixture()
        stamp = rospy.Time.now()
        header = Header(seq=1, stamp=stamp)
        manager._selected_source = 'fixed'
        manager._selected_track_id = 7
        manager._selected_uid = 'test-session:7'
        manager._latest_headers['fixed'] = header
        manager._latest_targets['fixed'] = [target(7)]
        failed = target(7)
        failed.lifecycle_state = 'lost'
        failed.measurement_source = 'stale'
        manager._manual_target = failed
        manager._fusion_conflict = True
        manager._sot_history.append((stamp, failed))
        manager._publish_targets('fixed', header, 180, 120)
        self.assertEqual('stale',manager._targets_pub.messages[-1].targets[0].measurement_source)
        self.assertEqual('LOST', manager._operating_state())

    def test_current_independent_flow_recovers_stale_sot_without_new_selection(self):
        manager = manager_fixture()
        initial = rospy.Time(10)
        manager._frames['fixed'].append((1, initial, frame()))
        manager._latest_headers['fixed'] = Header(seq=1, stamp=initial)
        manager._select_existing('fixed', target(), 'template')
        old_tracker = manager._manual_tracker
        stamp = rospy.Time(10.5)
        manager._frames['fixed'].append((2, stamp, frame(25)))
        stale = target(7)
        stale.lifecycle_state = 'lost'
        stale.measurement_source = 'stale'
        manager._manual_target = deepcopy(stale)
        manager._sot_history.append((stamp, stale))
        peer = target(7,25)
        peer.measurement_source = 'visual_tracker'
        peer.tracking_quality = .9
        peer.last_detection_stamp = initial
        manager._pixel_targets = {'fixed': (stamp, [peer])}
        manager._fusion_conflict = True
        message = TrackStateArray(header=Header(seq=2,stamp=stamp), image_source='fixed')
        manager._validate_anchor(message,[target(7,25),target(8,70)])
        self.assertIsNot(old_tracker, manager._manual_tracker)
        self.assertFalse(manager._fusion_conflict)
        self.assertEqual(7,manager._selected_track_id)
        self.assertEqual('test-session:7',manager._selected_uid)
        self.assertEqual('confirmed',manager._manual_target.lifecycle_state)
        self.assertAlmostEqual(75,manager._manual_target.bbox_xyxy[0])

    def test_recovery_rejects_stale_pixel_capture(self):
        manager = manager_fixture()
        stamp = rospy.Time(10)
        manager._frames['fixed'].append((1, stamp, frame()))
        manager._latest_headers['fixed'] = Header(seq=1, stamp=stamp)
        manager._select_existing('fixed',target(),'template')
        peer = target(7,25)
        peer.measurement_source = 'visual_tracker'
        peer.tracking_quality = .9
        peer.last_detection_stamp = rospy.Time(9)
        manager._pixel_targets = {'fixed': (rospy.Time(9),[peer])}
        original = manager._manual_tracker
        message = TrackStateArray(header=Header(seq=1,stamp=stamp), image_source='fixed')
        manager._validate_anchor(message,[target(7,25)])
        self.assertIs(original,manager._manual_tracker)
        self.assertTrue(manager._fusion_conflict)

    def test_late_detector_uses_exact_pixel_history_not_newer_camera_frame(self):
        manager = manager_fixture()
        initial = rospy.Time(10)
        manager._frames['fixed'].append((1,initial,frame()))
        manager._latest_headers['fixed'] = Header(seq=1,stamp=initial)
        manager._select_existing('fixed',target(),'template')
        captured = rospy.Time(10.5)
        manager._frames['fixed'].append((2,captured,frame(25)))
        manager._manual_target.lifecycle_state = 'lost'
        stale = deepcopy(manager._manual_target)
        stale.measurement_source = 'stale'
        manager._sot_history.append((captured,stale))
        peer = target(7,25)
        peer.measurement_source = 'visual_tracker'
        peer.tracking_quality = .9
        peer.last_detection_stamp = initial
        manager._pixel_history = {'fixed': deque([(captured,[peer])],maxlen=6)}
        manager._pixel_targets = {'fixed': (rospy.Time(10.75),[target(7,40)])}
        message = TrackStateArray(header=Header(stamp=captured),image_source='fixed')
        manager._validate_anchor(message,[target(7,25)])
        self.assertFalse(manager._fusion_conflict)
        self.assertEqual('confirmed',manager._manual_target.lifecycle_state)
        self.assertEqual(75,manager._manual_target.bbox_xyxy[0])
        manager._clear_pixels()
        self.assertEqual({},manager._pixel_history)

    def test_verified_anchor_updates_reference_but_rejected_one_does_not(self):
        manager = manager_fixture()
        stamp = rospy.Time(10)
        manager._frames['fixed'].append((1,stamp,frame()))
        manager._latest_headers['fixed'] = Header(seq=1,stamp=stamp)
        manager._select_existing('fixed',target(),'template')
        signature = manager._reference_signature.copy()
        altered = signature.copy()
        altered[0] += .05
        altered /= np.linalg.norm(altered)
        with patch.object(module,'image_signature',return_value=altered):
            message=TrackStateArray(header=Header(stamp=stamp),image_source='fixed')
            manager._validate_anchor(message,[target(7)])
        adapted = manager._reference_signature.copy()
        self.assertFalse(np.array_equal(signature,adapted))
        self.assertAlmostEqual(1.,float(np.linalg.norm(adapted)),places=5)
        with patch.object(module,'image_signature',return_value=-altered):
            manager._validate_anchor(message,[target(7)])
        np.testing.assert_array_equal(adapted,manager._reference_signature)

    def test_healthy_but_drifted_sot_is_not_kept_when_peer_and_yolo_agree(self):
        manager = manager_fixture()
        stamp = rospy.Time(10)
        manager._frames['fixed'].append((1,stamp,frame()))
        manager._latest_headers['fixed'] = Header(seq=1,stamp=stamp)
        manager._select_existing('fixed',target(),'template')
        old_tracker = manager._manual_tracker
        captured = rospy.Time(10.5)
        manager._frames['fixed'].append((2,captured,frame(25)))
        manager._sot_history.append((captured,deepcopy(target(7))))
        peer = target(7,25)
        peer.measurement_source = 'visual_tracker'
        peer.tracking_quality = .9
        peer.last_detection_stamp = stamp
        manager._pixel_targets={'fixed': (captured,[peer])}
        message=TrackStateArray(header=Header(stamp=captured),image_source='fixed')
        manager._validate_anchor(message,[target(7,25)])
        self.assertIsNot(old_tracker,manager._manual_tracker)
        self.assertEqual(75,manager._manual_target.bbox_xyxy[0])

    def test_fusion_waits_for_complete_same_frame_scene(self):
        manager = manager_fixture()
        manager._selected_source = "fixed"
        manager._selected_track_id = 7
        manager._selected_uid = "test-session:7"
        selected = target(7)
        selected.selected = True
        selected.measurement_source = "visual_tracker"
        stamp = rospy.Time.now()
        header = Header(seq=20, stamp=stamp)
        manager._sot_history.append((stamp, selected))
        manager._latest_headers["fixed"] = Header(seq=19, stamp=stamp-rospy.Duration(.1))
        manager._latest_targets["fixed"] = [target(7), target(8)]
        manager._publish_targets("fixed", header, 180, 120)
        self.assertEqual([], manager._targets_pub.messages)
        manager._latest_headers["fixed"] = header
        manager._publish_targets("fixed", header, 180, 120)
        output = manager._targets_pub.messages[-1]
        self.assertEqual({7, 8}, {item.track_id for item in output.targets})
        self.assertEqual("visual_tracker", next(item for item in output.targets
                                               if item.track_id == 7).measurement_source)
        self.assertEqual("detector", next(item for item in output.targets
                                          if item.track_id == 8).measurement_source)

    def test_source_clock_restart_clears_only_affected_camera(self):
        manager = manager_fixture()
        manager._frames["fixed"].append((30, rospy.Time(30), frame()))
        manager._frames["gimbal"].append((40, rospy.Time(40), frame()))
        manager._latest_headers["fixed"] = Header(seq=30, stamp=rospy.Time(30))
        manager._latest_headers["gimbal"] = Header(seq=40, stamp=rospy.Time(40))
        manager._latest_targets["fixed"] = [target()]
        message = CvBridge().cv2_to_imgmsg(frame(), encoding="bgr8")
        message.header = Header(seq=1, stamp=rospy.Time(1))
        manager._image_callback(message, "fixed")
        self.assertEqual(rospy.Time(1), manager._frames["fixed"][-1][1])
        self.assertNotIn("fixed", manager._latest_headers)
        self.assertEqual(rospy.Time(40), manager._latest_headers["gimbal"].stamp)
        self.assertEqual(2, manager._version)
        message.header.stamp = rospy.Time(.9)
        manager._image_callback(message, "fixed")
        self.assertEqual(rospy.Time(1), manager._frames["fixed"][-1][1])

    def test_switch_initializes_transients_and_sot_without_changing_identity(self):
        manager = manager_fixture()
        for sequence in range(1, 4):
            manager._frames["fixed"].append((sequence, rospy.Time(7 + sequence), frame()))
        manager._latest_headers["fixed"] = Header(seq=3, stamp=rospy.Time(10))
        manager._select_existing("fixed", target(), "template")
        old_tracker = manager._manual_tracker
        last_valid = manager._manual_last_valid_wall
        manager._image_evidence["old"] = object()
        manager._track_history["fixed"] = [(rospy.Time(9), [target()])]
        command = SetPerceptionModeCommand(request_id="reset-mode",
            session_id=manager._session_id, expected_state_version=1, mode="track")
        manager._mode_callback(command)
        self.assertEqual("APPLIED", manager._result_pub.messages[-1].stage)
        self.assertEqual(7, manager._selected_track_id)
        self.assertEqual("test-session:7", manager._selected_uid)
        self.assertEqual(1, len(manager._frames["fixed"]))
        self.assertEqual({}, manager._image_evidence)
        self.assertEqual({}, manager._track_history)
        self.assertIsNot(old_tracker, manager._manual_tracker)
        self.assertIsNone(old_tracker._bbox)
        self.assertEqual(1, len(manager._sot_history))
        self.assertEqual(last_valid, manager._manual_last_valid_wall)
        fresh_tracker = manager._manual_tracker
        manager._mode_callback(command)
        self.assertIs(fresh_tracker, manager._manual_tracker, "dedupe must not initialize twice")

    def test_invalid_mode_does_not_clear_buffers(self):
        manager = manager_fixture()
        manager._frames["fixed"].extend([(1, rospy.Time(9), frame()), (2, rospy.Time(10), frame())])
        manager._mode_callback(SetPerceptionModeCommand(request_id="invalid-reset",
            session_id=manager._session_id, expected_state_version=1, mode="invalid"))
        self.assertEqual("REJECTED", manager._result_pub.messages[-1].stage)
        self.assertEqual(2, len(manager._frames["fixed"]))

    def test_track_reduces_detection_and_loss_restores_search(self):
        self.assertEqual(1.0, detector_rate("track", True, "fixed", "fixed", 4))
        self.assertEqual(.5, detector_rate("track", True, "fixed", "gimbal", 4))
        self.assertEqual(4, detector_rate("track", False, "fixed", "fixed", 4))
        for mode in ("detect","fusion"):
            self.assertEqual(4, detector_rate(mode, True, "fixed", "fixed", 4))

    def test_existing_id_initializes_local_tracker_and_keeps_identity(self):
        manager=manager_fixture()
        stamp=rospy.Time(10)
        manager._frames["fixed"].append((1,stamp,frame()))
        manager._latest_headers["fixed"]=Header(seq=1,stamp=stamp)
        self.assertTrue(manager._select_existing("fixed",target(),"template"))
        manager._update_sot(frame(3),rospy.Time.from_sec(10.1))
        self.assertEqual(7,manager._manual_target.track_id)
        self.assertEqual("visual_tracker",manager._manual_target.measurement_source)
        self.assertAlmostEqual(53,manager._manual_target.bbox_xyxy[0],delta=1)
        self.assertEqual("tank",manager._manual_target.class_name)

    def test_fusion_rejects_competitor_without_reassigning_selected_id(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames["fixed"].append((1,stamp,frame()))
        manager._latest_headers["fixed"]=Header(seq=1,stamp=stamp)
        manager._select_existing("fixed",target(),"template")
        array=TrackStateArray(header=Header(seq=1,stamp=stamp),image_source="fixed")
        manager._validate_anchor(array,[target(7),target(8,1)])
        self.assertEqual("ambiguous_competitor",manager._fusion_reason)
        self.assertEqual("lost",manager._manual_target.lifecycle_state)
        self.assertEqual(7,manager._selected_track_id)

    def test_fusion_validates_healthy_sot_without_reinitializing_or_replay(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames["fixed"].append((1,stamp,frame()))
        manager._latest_headers["fixed"]=Header(seq=1,stamp=stamp)
        manager._select_existing("fixed",target(),"template")
        array=TrackStateArray(header=Header(seq=1,stamp=stamp),image_source="fixed")
        tracker = manager._manual_tracker
        with patch.object(module, "SotTracker", side_effect=AssertionError("redundant initialization")), \
                patch.object(manager, "_replay_sot", side_effect=AssertionError("redundant replay")):
            manager._validate_anchor(array,[target(7)])
        self.assertEqual("accepted",manager._fusion_reason)
        self.assertEqual(7,manager._manual_target.track_id)
        self.assertIs(tracker, manager._manual_tracker)
        self.assertEqual("detector",manager._manual_target.measurement_source)
        self.assertEqual("detector",manager._sot_at(stamp).measurement_source)

    def test_fusion_failed_sot_still_reinitializes_from_verified_detector(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames["fixed"].append((1,stamp,frame()))
        manager._latest_headers["fixed"]=Header(seq=1,stamp=stamp)
        manager._select_existing("fixed",target(),"template")
        previous = manager._manual_tracker
        manager._manual_target.lifecycle_state = "lost"
        manager._manual_target.measurement_source = "stale"
        manager._fusion_conflict = True
        array=TrackStateArray(header=Header(seq=1,stamp=stamp),image_source="fixed")
        manager._validate_anchor(array,[target(7)])
        self.assertIsNot(previous, manager._manual_tracker)
        self.assertFalse(manager._fusion_conflict)
        self.assertEqual("confirmed", manager._manual_target.lifecycle_state)

    def test_nearby_different_appearance_is_rejected(self):
        first=image_signature(frame(),target().bbox_xyxy)
        other=frame(); other[40:64,50:86]=(0,0,255)
        accepted,reason=evidence_gate(target().bbox_xyxy,target().bbox_xyxy,0,
                                      first,image_signature(other,target().bbox_xyxy))
        self.assertFalse(accepted); self.assertEqual("appearance_conflict",reason)

    def test_lost_source_restores_search_and_no_foreign_stale_boxes(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames["fixed"].append((1,stamp,frame()))
        manager._latest_headers["fixed"]=Header(seq=1,stamp=stamp)
        manager._latest_targets["fixed"]=[target()]
        manager._select_existing("fixed",target(),"template")
        manager._manual_last_valid_wall=time.monotonic()-2
        self.assertEqual("LOST",manager._operating_state())
        manager._publish_targets("fixed",Header(seq=2,stamp=rospy.Time(11)),180,120)
        self.assertEqual([],manager._targets_pub.messages,
                         "fusion must not publish foreign-frame boxes")
        manager._last_source_wall["fixed"] = time.monotonic()-2
        manager._publish_targets("fixed",Header(seq=2,stamp=rospy.Time(11)),180,120)
        self.assertEqual([],manager._targets_pub.messages[-1].targets,
                         "a stopped camera must still clear the display")

    def test_manual_roi_uses_global_allocator_and_survives_detect(self):
        manager = manager_fixture(); stamp = rospy.Time(10)
        manager._frames["fixed"].append((1, stamp, frame()))
        requests = []
        def identity_service(request):
            requests.append(deepcopy(request))
            return SimpleNamespace(success=True, global_id=201, message="", bound=False)
        manager._identity_client = identity_service
        command = SimpleNamespace(image_source="fixed",frame_seq=1,capture_stamp=stamp,
            roi_norm=[50/180,40/120,36/180,24/120],tracker_algorithm="template")
        self.assertTrue(manager._initialize_roi(command))
        self.assertEqual(201, manager._selected_track_id)
        self.assertEqual("test-session:201", manager._selected_uid)
        self.assertEqual("register", requests[0].action)
        self.assertEqual(512, len(requests[0].image_signature))
        self.assertFalse(manager._manual_target.class_valid)
        manager._mode = "detect"
        manager._clear_selection()
        manager._advance_background("fixed", frame(3), rospy.Time.from_sec(10.1))
        self.assertEqual("update", requests[-1].action)
        self.assertEqual(201, requests[-1].global_id)
        self.assertEqual("confirmed", requests[-1].observation.lifecycle_state)

    def test_replacing_unclassified_roi_retires_observer_not_vehicle_tracks(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames['fixed'].append((1,stamp,frame()))
        manager._latest_headers['fixed']=Header(stamp=stamp)
        requests=[]
        def identity_service(request):
            requests.append(deepcopy(request))
            return SimpleNamespace(success=True,global_id=201,message='',bound=False)
        manager._identity_client=identity_service
        command=SimpleNamespace(image_source='fixed',frame_seq=1,capture_stamp=stamp,
            roi_norm=[.15,.15,.6,.6],tracker_algorithm='template')
        self.assertTrue(manager._initialize_roi(command))
        manager._visual_targets['fixed']=(stamp,[deepcopy(manager._manual_target)])
        self.assertTrue(manager._select_existing('fixed',target(7),'template'))
        self.assertTrue(any(r.action=='retire' and r.global_id==201 for r in requests))
        self.assertNotIn(201,manager._background_visual)
        self.assertEqual('lost',manager._visual_targets['fixed'][1][0].lifecycle_state)
        self.assertEqual([0,0,0,0],manager._visual_targets['fixed'][1][0].bbox_xyxy)
        self.assertEqual(7,manager._selected_track_id)

    def test_retiring_unknown_background_keeps_classified_manual_observer(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        unknown=target(201);unknown.class_valid=False
        known=target(202)
        first=SimpleNamespace(reset=lambda:None);second=SimpleNamespace(reset=lambda:None)
        manager._background_visual=OrderedDict([
            (201,dict(target=unknown,source='fixed',tracker=first)),
            (202,dict(target=known,source='fixed',tracker=second))])
        requests=[];manager._identity_client=lambda request:requests.append(request)
        manager._retire_unclassified_background()
        self.assertEqual([202],list(manager._background_visual))
        self.assertEqual('retire',requests[0].action)

    def test_failed_identity_registration_preserves_existing_selection(self):
        manager=manager_fixture(); stamp=rospy.Time(10)
        manager._frames["fixed"].append((1,stamp,frame()))
        manager._latest_headers["fixed"]=Header(seq=1,stamp=stamp)
        manager._select_existing("fixed",target(),"template")
        original = manager._manual_tracker
        manager._identity_client = lambda request: SimpleNamespace(success=False,message="capacity reached")
        command = SimpleNamespace(image_source="fixed",frame_seq=1,capture_stamp=stamp,
            roi_norm=[50/180,40/120,36/180,24/120],tracker_algorithm="template")
        self.assertFalse(manager._initialize_roi(command))
        self.assertIs(original, manager._manual_tracker)
        self.assertEqual(7,manager._selected_track_id)

    def test_missing_detector_evidence_resets_consecutive_manual_binding(self):
        manager = manager_fixture(); stamp = rospy.Time(10)
        manager._frames["fixed"].append((1, stamp, frame()))
        manager._visual_targets["fixed"] = (stamp, [target(201)])
        requests = []
        manager._identity_client = lambda request: requests.append(request)
        message = TrackStateArray(header=Header(stamp=stamp), image_source="fixed")
        manager._bind_registered_visuals(message, [])
        self.assertEqual("reset_bind", requests[0].action)
        self.assertEqual(201, requests[0].global_id)


if __name__=="__main__": unittest.main()
