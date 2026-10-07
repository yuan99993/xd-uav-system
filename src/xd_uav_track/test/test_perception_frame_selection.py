#!/usr/bin/env python3
"""Regression for delayed user selection accidentally binding a current target."""
from collections import deque
from copy import deepcopy
from types import SimpleNamespace
import unittest
import time
import rospy
from unittest.mock import patch
from xd_uav_track.msg import SelectPerceptionTargetCommand
from test_perception_modes import manager_fixture, target, frame


class FrameSelectionTest(unittest.TestCase):
    def setUp(self):
        self.manager=manager_fixture()
        self.manager._frame_tolerance=.001
        self.manager._frames["fixed"].append((11,rospy.Time(10),frame()))
        self.manager._frames["fixed"].append((12,rospy.Time(11),frame(3)))
        self.manager._latest_targets["fixed"]=[target(8)]
        self.manager._track_history["fixed"]=deque([
            (rospy.Time(10),[target(7)]),(rospy.Time(11),[target(8)])])
        self.command=SimpleNamespace(image_source="fixed",frame_seq=11,
            capture_stamp=rospy.Time(10),point_norm=[68/180,52/120],
            roi_norm=[50/180,40/120,36/180,24/120])

    def test_delayed_roi_and_click_match_historical_id_not_latest_id(self):
        entry,candidates=self.manager._selection_evidence(self.command,"roi")
        self.assertEqual(11,entry[0])
        self.assertEqual(7,self.manager._find_roi_target(self.command,candidates).track_id)
        self.assertEqual(7,self.manager._find_existing_target(self.command,"point",candidates).track_id)

    def test_padded_roi_selects_single_original_frame_identity_and_tight_box(self):
        self.command.roi_norm=[.15,.15,.6,.6]
        _,candidates=self.manager._selection_evidence(self.command,'roi')
        chosen=self.manager._find_roi_target(self.command,candidates)
        self.assertIsNotNone(chosen)
        self.assertEqual(7,chosen.track_id)
        self.assertEqual([50,40,86,64],chosen.bbox_xyxy)
        self.assertNotEqual(8,chosen.track_id,'latest-frame ID must not replace the referenced identity')

    def test_padded_roi_with_multiple_entities_is_not_a_unique_selection(self):
        self.command.roi_norm=[.15,.15,.8,.6]
        candidates=[target(7),target(8,60)]
        self.assertEqual(2,len(self.manager._roi_contained_targets(self.command,candidates)))
        self.assertIsNone(self.manager._find_roi_target(self.command,candidates))

    def test_padded_roi_command_reuses_identity_without_manual_allocation(self):
        command=SelectPerceptionTargetCommand(request_id='coarse-roi',session_id='test-session',
            expected_state_version=1,selection_type='roi',image_source='fixed',frame_seq=11,
            capture_stamp=rospy.Time(10),roi_norm=[.15,.15,.6,.6],tracker_algorithm='template')
        with patch.object(self.manager,'_initialize_roi',side_effect=AssertionError('duplicate allocation')):
            self.manager._selection_callback(command)
        self.assertEqual('APPLIED',self.manager._result_pub.messages[-1].stage)
        self.assertEqual(7,self.manager._selected_track_id)
        self.assertEqual([50,40,86,64],self.manager._sot_at(rospy.Time(10)).bbox_xyxy)
        self.assertEqual([53,40,89,64],self.manager._manual_target.bbox_xyxy,
                         'replay must advance the tight object box, not the padded user region')

    def test_ambiguous_padded_roi_command_cannot_allocate_manual_double(self):
        self.manager._track_history['fixed'][0][1].append(target(9,60))
        command=SelectPerceptionTargetCommand(request_id='ambiguous-roi',session_id='test-session',
            expected_state_version=1,selection_type='roi',image_source='fixed',frame_seq=11,
            capture_stamp=rospy.Time(10),roi_norm=[.15,.15,.8,.6],tracker_algorithm='template')
        with patch.object(self.manager,'_initialize_roi',side_effect=AssertionError('ambiguous allocation')):
            self.manager._selection_callback(command)
        self.assertEqual('REJECTED',self.manager._result_pub.messages[-1].stage)
        self.assertEqual('AMBIGUOUS_TARGET',self.manager._result_pub.messages[-1].error_code)
        self.assertEqual(-1,self.manager._selected_track_id)

    def test_roi_without_recognized_target_preserves_arbitrary_manual_tracking(self):
        self.command.roi_norm=[.15,.15,.6,.6]
        unknown=target(7);unknown.class_valid=False
        self.assertEqual([],self.manager._roi_contained_targets(self.command,[unknown]))
        self.assertIsNone(self.manager._find_roi_target(self.command,[unknown]))

    def test_confirmed_identity_can_be_selected_during_low_confidence_maintenance(self):
        self.command.roi_norm=[.15,.15,.6,.6]
        established=target(7); established.detection_confidence=.3
        self.assertEqual(7,self.manager._find_roi_target(self.command,[established]).track_id)
        established.lifecycle_state='lost'
        self.assertIsNone(self.manager._find_roi_target(self.command,[established]))

    def test_parked_unclassified_region_cannot_steal_coarse_vehicle_selection(self):
        self.command.roi_norm=[.15,.15,.6,.6]
        background=target(24);background.class_valid=False
        background.bbox_cxcywh_norm=[.45,.45,.6,.6]
        self.assertEqual(7,self.manager._find_roi_target(self.command,[background,target(7)]).track_id)

    def test_evicted_frame_cannot_fall_back_to_latest_overlapping_target(self):
        self.command.frame_seq=9
        entry,candidates=self.manager._selection_evidence(self.command,"roi")
        self.assertIsNone(entry);self.assertEqual([],candidates)

    def test_frame_sequence_and_stamp_must_agree(self):
        self.command.capture_stamp=rospy.Time(11)
        self.assertIsNone(self.manager._selection_evidence(self.command,"roi")[0])

    def test_frame_reference_is_required_even_when_latest_roi_matches(self):
        self.command.frame_seq=0;self.command.capture_stamp=rospy.Time(0)
        for kind in ("point","roi"):
            self.assertIsNone(self.manager._selection_evidence(self.command,kind)[0])

    def test_roi_without_detection_can_initialize_but_click_cannot(self):
        self.manager._track_history.clear()
        self.assertIsNotNone(self.manager._selection_evidence(self.command,"roi")[0])
        self.assertIsNone(self.manager._selection_evidence(self.command,"point")[0])

    def test_source_isolation(self):
        self.command.image_source="gimbal"
        self.assertIsNone(self.manager._selection_evidence(self.command,"roi")[0])

    def test_timestamp_only_does_not_select_adjacent_frame(self):
        self.command.frame_seq=0
        self.command.capture_stamp=rospy.Time.from_sec(10.03)
        self.assertIsNone(self.manager._selection_evidence(self.command,"roi")[0])

    def test_registered_visual_is_selectable_without_new_detector_frame(self):
        manual = target(31)
        self.manager._cache_selection_snapshot("fixed", rospy.Time(11), [manual])
        self.command.frame_seq = 0
        self.command.capture_stamp = rospy.Time(0)
        self.command.track_id = 31
        entry, candidates = self.manager._selection_evidence(self.command, "track_id")
        self.assertEqual(rospy.Time(11), entry[1])
        self.assertEqual(31, self.manager._find_existing_target(
            self.command, "track_id", candidates).track_id)

    def test_async_snapshot_cache_preserves_order_and_same_frame_entities(self):
        self.manager._cache_selection_snapshot("fixed", rospy.Time(10), [target(31)])
        history = self.manager._track_history["fixed"]
        self.assertEqual(rospy.Time(11), history[-1][0])
        self.assertEqual({7, 31}, {t.track_id for t in history[0][1]})

    def test_stopped_camera_does_not_turn_retained_image_into_fresh_selection(self):
        self.manager._last_source_wall["fixed"] = time.monotonic() - 5.0
        self.assertIsNone(self.manager._selection_evidence(self.command, "roi")[0])

    def test_bound_identity_remaps_delayed_snapshots_and_old_public_id(self):
        self.manager._remap_identity(7, 31)
        self.command.track_id = 7
        entry, candidates = self.manager._selection_evidence(self.command, "track_id")
        self.assertEqual(rospy.Time(10), entry[1])
        self.assertEqual(31, self.manager._find_existing_target(
            self.command, "track_id", candidates).track_id)


if __name__=="__main__":unittest.main()
