"""Historical unsafe takeover geometries, without simulator truth in production."""
import unittest
from dataclasses import replace
import numpy as np
from sar_yolo_detector.box_fusion import BoxFusion, Candidate
from sar_yolo_detector.takeover_safety import rejection_reason
from test_box_fusion import LocalTracker, image


class TakeoverSafetyTest(unittest.TestCase):
    def make(self, enabled=True):
        f=BoxFusion(dict(strong_detector_takeover_enabled=True,
            strong_takeover_safety_enabled=enabled,class_groups=[[0,1],[2]]),tracker=LocalTracker())
        f.initialize(image(),(60,60,40,40),10.,99,Candidate((60,60,40,40),0,.9,5,True,10.))
        return f

    def test_round5_old_packet_cannot_recover_same_capture_lost_or_update_history(self):
        t=1795161478.3615594;measured=1795161478.2774334
        for enabled in (False,True):
            f=self.make(enabled);f.stamp=t;f.last_detection_stamp=t-.6813716888427734
            f.last_observation_stamp=f.selection_stamp=f.last_detection_stamp
            f._detector_reference_stamp=f.last_detection_stamp
            f._result(None,'lost','no_trusted_visual_observation',0.);f.tracker.valid=False
            c=Candidate((60,60,40,40),0,.8320693,-1,False,measured,selected_public_id=99)
            result=f.step(image(),t,[c],same_frame_correction=True)
            if enabled:
                self.assertNotEqual('strong_detector_takeover',result.reason)
                self.assertLess(f.last_detection_stamp,measured)
                self.assertEqual(1,f.diagnostics['strong_takeover_vetoes']['stale_correction_after_loss'])
            else:self.assertEqual('strong_detector_takeover',result.reason)

    def test_round4_long_gap_neighbour_does_not_gain_authority_after_three_frames(self):
        frame=np.zeros((640,640,3),np.uint8)
        for enabled in (False,True):
            f=BoxFusion(dict(strong_detector_takeover_enabled=True,strong_takeover_safety_enabled=enabled,
                adaptive_detector_motion_gate=True),tracker=LocalTracker())
            last=1795156887.5399804;ref=(309.8474238433578,310.5871202141262,19.603433799650418,18.825083860706457)
            f.initialize(frame,ref,last,1,Candidate(ref,0,.9,1,True,last));f.tracker.valid=False
            f._detector_reference_box=ref;f._detector_reference_stamp=last
            f._detector_velocity=np.array([71.0120,29.0093]);f._detector_anchor_area=ref[2]*ref[3]
            for t in (1795156889.263539,1795156889.305539,1795156889.347539):
                c=Candidate((487,358,29,22),0,.8785547,-1,False,t,selected_public_id=1)
                result=f.step(frame,t,[c])
            if enabled:
                self.assertNotEqual('strong_detector_takeover',result.reason)
                self.assertEqual(last,f.last_detection_stamp)
            else:self.assertEqual('strong_detector_takeover',result.reason)

    def test_round4_crossing_expanded_box_is_rejected_but_target_proposal_survives(self):
        parent=Candidate((272,303,45,28),0,.63333535,-1,False,10.1,selected_public_id=99)
        neighbour=Candidate((294,304,23,13),0,.79540604,-1,False,10.1,selected_public_id=99)
        target=Candidate((272,315,26,16),0,.58399427,-1,False,10.1,selected_public_id=99)
        kw=dict(stamp=10.1,last_detection_stamp=10.,reference=(275,314,26,16),public_id=99,class_groups=[[0,1],[2]])
        self.assertEqual('abrupt_detector_scale_expansion',rejection_reason(parent,[parent,neighbour,target],**kw))
        self.assertIsNone(rejection_reason(target,[parent,neighbour,target],**kw))
        # Earlier failing frame: only one compact competitor is available.
        earlier=replace(parent,box=(273,303,39,27),confidence=.69801277)
        competitor=replace(neighbour,box=(289,304,22,14),confidence=.88105702)
        self.assertIsNotNone(rejection_reason(earlier,[earlier,competitor],**kw))

    def test_normal_fresh_single_vehicle_and_fast_handoff_are_unchanged(self):
        f=self.make();before=f.tracker.initializations
        c=Candidate((60,60,40,40),0,.55,-1,False,10.04,selected_public_id=99)
        r=f.step(image(),10.04,[c]);self.assertEqual('strong_detector_takeover',r.reason)
        self.assertTrue(r.observed);self.assertEqual(99,r.public_id)
        self.assertFalse(f.diagnostics['strong_takeover_vetoes'])

    def test_part_to_body_without_competing_vehicle_is_not_blanket_rejected(self):
        f=self.make();c=Candidate((60,60,60,40),0,.8,-1,False,10.1,selected_public_id=99)
        r=f.step(image(),10.1,[c]);self.assertEqual('strong_detector_takeover',r.reason)

    def test_wrong_family_inside_box_cannot_trigger_competitor_veto(self):
        parent=Candidate((60,60,60,40),0,.7,-1,False,10.1,selected_public_id=99)
        other=Candidate((90,60,20,20),2,.9,-1,False,10.1,selected_public_id=99)
        self.assertIsNone(rejection_reason(parent,[parent,other],stamp=10.1,last_detection_stamp=10.,reference=(60,60,40,40),public_id=99,class_groups=[[0,1],[2]]))

    def test_long_gap_same_position_uses_ordinary_confirmation_and_can_recover(self):
        f=self.make();f.tracker.valid=False;f._result(None,'lost','no_trusted_visual_observation',0.)
        for t in (11.1,11.14,11.18):
            c=Candidate((60,60,40,40),0,.9,-1,False,t,selected_public_id=99)
            r=f.step(image(),t,[c])
            self.assertNotEqual('strong_detector_takeover',r.reason)
            if t<11.18:self.assertEqual(10.,f.last_detection_stamp)
        self.assertTrue(r.observed);self.assertEqual(11.18,f.last_detection_stamp)

    def test_current_capture_can_correct_loss_without_using_an_old_packet(self):
        f=self.make();f.stamp=10.1;f._result(None,'lost','no_trusted_visual_observation',0.)
        c=Candidate((60,60,40,40),0,.8,-1,False,10.1,selected_public_id=99)
        r=f.step(image(),10.1,[c],same_frame_correction=True)
        self.assertEqual('strong_detector_takeover',r.reason)
        self.assertEqual(10.1,f.last_detection_stamp)

    def test_old_packet_on_new_frame_requires_current_independent_witnesses(self):
        f=self.make();f.tracker.valid=False
        c=Candidate((60,60,40,40),0,.95,-1,False,10.04,selected_public_id=99)
        r=f.step(image(),10.12,[c])
        self.assertNotEqual('strong_detector_takeover',r.reason)
        self.assertEqual(10.,f.last_detection_stamp)
        self.assertEqual(1,f.diagnostics['rejected']['camera_projection_lacks_current_witness'])

    def test_crossing_expansion_cannot_poison_history_for_next_neighbour(self):
        frame=np.zeros((640,640,3),np.uint8)
        f=BoxFusion(dict(strong_detector_takeover_enabled=True,strong_takeover_safety_enabled=True),tracker=LocalTracker())
        start=1795165789.6876116;ref=(273,317,24,16)
        f.initialize(frame,ref,start,99,Candidate(ref,0,.9,5,True,start))
        f._update_detector_reference(ref,start)
        for t,box in [(1795165789.7321544,(264,307,32,25)),
                      (1795165789.7778282,(271,306,26,25)),
                      (1795165789.7948122,(272,305,27,25))]:
            c=Candidate(box,0,.85,-1,False,t,selected_public_id=99)
            r=f.step(frame,t,[c])
            self.assertNotEqual('strong_detector_takeover',r.reason)
            self.assertEqual(start,f.last_detection_stamp)
            self.assertEqual(ref,f._detector_reference_box)
        c=Candidate((281,303,24,16),0,.7672587,-1,False,1795165789.8395114,selected_public_id=99)
        r=f.step(frame,1795165789.8395114,[c])
        self.assertNotEqual('strong_detector_takeover',r.reason)
        self.assertEqual(start,f.last_detection_stamp)

    def test_reference_size_is_not_compared_to_drifting_local_tracker(self):
        f=self.make();f._update_detector_reference((60,60,40,40),10.);f.box=(10,10,8,8)
        c=Candidate((60,60,40,40),0,.8,-1,False,10.04,selected_public_id=99)
        r=f.step(image(),10.04,[c])
        self.assertEqual('strong_detector_takeover',r.reason)

    def test_new_roi_without_detection_history_is_not_a_long_gap(self):
        f=BoxFusion(dict(strong_detector_takeover_enabled=True,strong_takeover_safety_enabled=True),tracker=LocalTracker())
        t=1795166315.748533
        f.initialize(image(),(60,60,40,40),t,900000002)
        self.assertEqual(0.,f.last_detection_stamp)
        c=Candidate((60,60,40,40),0,.9,-1,False,t+.08,selected_public_id=900000002)
        r=f.step(image(),t+.08,[c])
        self.assertEqual('strong_detector_takeover',r.reason)
        self.assertTrue(r.observed);self.assertEqual(t+.08,f.last_detection_stamp)

    def test_projected_marker_cannot_take_over_even_with_current_stamp(self):
        f=self.make();f.tracker.valid=False
        c=Candidate((60,60,40,40),0,.95,-1,False,10.04,camera_projected=True,selected_public_id=99)
        r=f.step(image(),10.04,[c])
        self.assertNotEqual('strong_detector_takeover',r.reason)
        self.assertEqual(10.,f.last_detection_stamp)

    def test_partial_box_can_return_to_body_without_poisoning_size_guard(self):
        frame=np.zeros((640,640,3),np.uint8)
        f=BoxFusion(dict(strong_detector_takeover_enabled=True,strong_takeover_safety_enabled=True),tracker=LocalTracker())
        t=1795167775.3849926;partial=(307,317,12,14)
        f.initialize(frame,partial,t,99,Candidate(partial,0,.7399,5,True,t))
        f._update_detector_reference(partial,t);f._detector_anchor_area=271.
        c=Candidate((305,316,19,17),0,.70521,-1,False,t+.0481,selected_public_id=99)
        r=f.step(frame,t+.0481,[c])
        self.assertEqual('strong_detector_takeover',r.reason)
        self.assertTrue(r.observed);self.assertEqual(t+.0481,f.last_detection_stamp)

    def test_long_gap_uses_independent_motion_prediction_not_old_position_iou(self):
        c=Candidate((84,60,40,40),0,.9,-1,False,11.2,selected_public_id=99)
        reason=rejection_reason(c,[c],stamp=11.2,last_detection_stamp=10.,
            reference=(60,60,40,40),predicted_reference=(84,60,40,40),
            public_id=99,class_groups=[[0,1],[2]])
        self.assertIsNone(reason)

    def test_same_size_long_gap_neighbour_still_fails_independent_reach(self):
        c=Candidate((130,60,40,40),0,.95,-1,False,11.2,selected_public_id=99)
        reason=rejection_reason(c,[c],stamp=11.2,last_detection_stamp=10.,
            reference=(60,60,40,40),predicted_reference=(84,60,40,40),
            public_id=99,class_groups=[[0,1],[2]])
        self.assertEqual('long_gap_identity_geometry_changed',reason)
