import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from sar_yolo_detector.box_fusion import BoxFusion, Candidate, class_duplicate_indices, iou


class LocalTracker:
    tracker_name = "fake_kcf"

    def __init__(self):
        self.box = None
        self.valid = True
        self.updates = 0
        self.initializations = 0

    def reset(self):
        self.box = None

    def initialize(self, frame, box):
        self.box = tuple(box)
        self.initializations += 1

    def update(self, frame):
        self.updates += 1
        return SimpleNamespace(bbox=self.box, valid=self.valid)


def image(box=(60, 60, 40, 40), background=24):
    frame = np.full((200, 260, 3), background, np.uint8)
    x, y, w, h = (int(v) for v in box)
    rng = np.random.RandomState(3)
    texture = rng.randint(0, 120, (h, w, 1)).astype(np.uint8)
    frame[y:y+h, x:x+w] = texture + np.array([20, 70, 10], np.uint8)
    return frame


class RawSelectedProposalTest(unittest.TestCase):
    def test_strong_frames_defer_kcf_cost_but_gap_initializes_exact_latest_seed(self):
        fusion=self.make();fusion.config.update(strong_detector_takeover_enabled=True,defer_strong_local_seed=True)
        count=fusion.tracker.initializations
        for stamp in (10.1,10.2,10.3):
            result=fusion.step(image(),stamp,[Candidate((60,60,40,40),0,.8,-1,False,stamp,selected_public_id=99)])
            self.assertEqual('strong_detector_takeover',result.reason)
            self.assertEqual(count,fusion.tracker.initializations)
            self.assertFalse(fusion.diagnostics['local_reinitialized'])
        fusion.step(image(),10.4,[],detector_updated=False)
        self.assertEqual(count+1,fusion.tracker.initializations)
        self.assertIsNone(fusion._deferred_local_seed)
        self.assertEqual(10.3,fusion.last_detection_stamp)

    def test_unique_high_confidence_short_dropout_recovers_on_first_fresh_frame(self):
        fusion=self.make();fusion.config.update(strong_detector_takeover_enabled=True,
            fast_reacquire_confidence=.60,fast_reacquire_max_gap_sec=1.25)
        fusion.step(image(),10.1,[Candidate((60,60,40,40),0,.8,-1,False,10.1,selected_public_id=99)])
        fusion.tracker.valid=False
        result=fusion.step(image(),11.0,[Candidate((60,60,40,40),0,.65,-1,False,11.,selected_public_id=99)])
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertTrue(fusion.diagnostics['fast_single_frame_recovery'])
        self.assertEqual(5,result.local_id)

    def test_speed_adapts_innovation_gate_and_motion_fit_beats_neighbour_confidence(self):
        fusion=self.make();fusion.config.update(strong_detector_takeover_enabled=True,adaptive_detector_motion_gate=True)
        fusion.step(image(),10.1,[Candidate((60,60,40,40),0,.8,-1,False,10.1,selected_public_id=99)])
        fusion._detector_velocity=np.array([100.,0.])
        result=fusion.step(image((77,60,40,40)),10.2,[Candidate((77,60,40,40),0,.7,-1,False,10.2,selected_public_id=99)])
        self.assertEqual('strong_detector_takeover',result.reason)
        fusion._detector_velocity=np.array([100.,0.])
        result=fusion.step(image((87,60,40,40)),10.3,[
            Candidate((87,60,40,40),0,.65,-1,False,10.3,selected_public_id=99),
            Candidate((67,60,40,40),0,.99,-1,False,10.3,selected_public_id=99)])
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual((87,60,40,40),result.box)

    def test_camera_rotation_round_trips_do_not_inflate_detector_scale(self):
        from sar_yolo_detector.camera_motion import transform_window
        original=(60,60,40,40)
        rotation=np.vstack([cv2.getRotationMatrix2D((80,80),2.,1.),[0,0,1]])
        inverse=np.linalg.inv(rotation)
        for detections_present in (False,True):
            with self.subTest(detections_present=detections_present):
                fusion=self.make();fusion.config['strong_detector_takeover_enabled']=True
                fusion.step(image(),10.1,[Candidate(original,0,.8,-1,False,10.1,selected_public_id=99)])
                initial=fusion._detector_anchor_area
                fusion.tracker.valid=False
                for index in range(100):
                    stamp=10.12+index*.02
                    matrix=rotation if index%2==0 else inverse
                    box=transform_window(original,rotation)[0] if index%2==0 else original
                    candidates=[Candidate(box,0,.8,-1,False,stamp,selected_public_id=99)] if detections_present else []
                    result=fusion.step(image(),stamp,candidates,camera_transform=matrix)
                    if detections_present:self.assertEqual('strong_detector_takeover',result.reason)
                self.assertAlmostEqual(initial,fusion._detector_anchor_area,places=3)
                if not detections_present:
                    np.testing.assert_allclose(fusion._detector_reference_box,original,atol=1e-5)

    def test_occlusion_uncertainty_grows_but_distinct_recovery_targets_stay_ambiguous(self):
        def create():
            fusion=BoxFusion({'strong_detector_takeover_enabled':True,'class_groups':[[0,1],[2]]},tracker=LocalTracker())
            fusion.initialize(image((180,80,12,13)),(180,80,12,13),10.,99,Candidate((180,80,12,13),0,.9,5,True,10.))
            fusion.step(image((180,80,12,13)),10.1,[Candidate((180,80,12,13),0,.8,-1,False,10.1,selected_public_id=99)])
            fusion._detector_velocity=np.array([-5.,0.]);fusion.velocity=np.array([-5.,0.])
            fusion.tracker.valid=False
            return fusion
        fusion=create()
        for stamp,x in [(13.8,105),(13.84,104),(13.88,103)]:
            result=fusion.step(image((x,80,12,13)),stamp,[Candidate((x,80,12,13),0,.8,-1,False,stamp,selected_public_id=99)])
        self.assertEqual('strong_detector_takeover',result.reason)
        ambiguous=create();frame=image((105,80,12,13))
        frame[80:93,165:177]=frame[80:93,105:117]
        for stamp in (13.8,13.84,13.88):
            result=ambiguous.step(frame,stamp,[
                Candidate((105,80,12,13),0,.55,-1,False,stamp,selected_public_id=99),
                Candidate((165,80,12,13),0,.95,-1,False,stamp,selected_public_id=99)])
        self.assertNotEqual('strong_detector_takeover',result.reason)
        self.assertTrue(ambiguous.diagnostics['strong_recovery_multiple_identities'])
        self.assertEqual(10.1,result.last_detection_stamp)

    def test_unique_strong_recovery_is_not_vetoed_by_old_appearance(self):
        from sar_yolo_detector.box_fusion import appearance
        fusion=self.make();fusion.config['strong_detector_takeover_enabled']=True
        fusion.step(image(),10.1,[Candidate((60,60,40,40),0,.8,-1,False,10.1,selected_public_id=99)])
        current=image()[:,:,[1,2,0]].copy()
        self.assertLess(float(np.dot(appearance(current,(60,60,40,40))[0],fusion.anchor)),fusion.appearance_gate)
        fusion.tracker.valid=False
        for stamp in (11.1,11.14,11.18):
            result=fusion.step(current,stamp,[Candidate((60,60,40,40),0,.8,-1,False,stamp,selected_public_id=99)])
            if stamp<11.18:self.assertEqual(10.1,result.last_detection_stamp)
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual(5,result.local_id)

    def test_recovery_uses_canonical_extent_instead_of_last_part_box(self):
        fusion=BoxFusion({'strong_detector_takeover_enabled':True,'class_groups':[[0,1],[2]]},tracker=LocalTracker())
        fusion.initialize(image((60,60,26,11)),(60,60,26,11),10.,99,Candidate((60,60,26,11),0,.9,5,True,10.))
        fusion.step(image((60,60,26,11)),10.1,[Candidate((68,60,10,13),0,.8,-1,False,10.1,selected_public_id=99)])
        fusion.tracker.valid=False
        for stamp in (11.1,11.14,11.18):
            result=fusion.step(image((62,57,23,20)),stamp,[Candidate((56,50,35,34),0,.8,-1,False,stamp,selected_public_id=99)])
            if stamp<11.18:self.assertEqual(10.1,result.last_detection_stamp)
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual(5,result.local_id)
        self.assertEqual(11.18,result.last_detection_stamp)

    def test_reassociation_window_is_independent_of_prediction_output_hold(self):
        fusion=BoxFusion({'strong_detector_takeover_enabled':True,'prediction_hold_sec':.65,
                          'class_groups':[[0,1],[2]]},tracker=LocalTracker())
        fusion.initialize(image((60,60,20,20)),(60,60,20,20),10.,99,Candidate((60,60,20,20),0,.9,5,True,10.))
        fusion.step(image((60,60,20,20)),10.1,[Candidate((60,60,20,20),0,.8,-1,False,10.1,selected_public_id=99)])
        fusion._detector_velocity=np.array([100.,0.]);fusion.velocity=np.array([100.,0.])
        fusion.tracker.valid=False
        for stamp,x in [(11.3,180),(11.34,184),(11.38,188)]:
            result=fusion.step(image((x,60,20,20)),stamp,[Candidate((x,60,20,20),0,.8,-1,False,stamp,selected_public_id=99)])
            if stamp<11.38:self.assertEqual(10.1,result.last_detection_stamp)
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual(5,result.local_id)
        self.assertEqual(.65,fusion.predict_hold)
        self.assertEqual(11.38,result.last_detection_stamp)

    def test_part_to_body_transition_does_not_create_target_velocity(self):
        fusion=BoxFusion({'strong_detector_takeover_enabled':True,'class_groups':[[0,1],[2]]},tracker=LocalTracker())
        fusion.initialize(image((60,60,20,20)),(60,60,20,20),10.,99,Candidate((60,60,20,20),0,.9,5,True,10.))
        fusion.step(image((60,60,20,20)),10.1,[Candidate((60,60,20,20),0,.8,-1,False,10.1,selected_public_id=99)])
        result=fusion.step(image((60,60,40,20)),10.2,[Candidate((60,60,40,20),0,.8,-1,False,10.2,selected_public_id=99)])
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual((60,60,40,20),result.box)
        np.testing.assert_allclose(fusion._detector_velocity,(0,0),atol=1e-6)

    def test_delayed_strong_detection_uses_capture_time_for_velocity_and_keeps_provenance(self):
        fusion=self.make();fusion.config['strong_detector_takeover_enabled']=True
        fusion.step(image(),10.1,[Candidate((60,60,40,40),0,.8,-1,False,10.1,selected_public_id=99)])
        fusion._detector_velocity=np.array([20.,0.])
        fusion.velocity=np.array([900.,700.])
        result=fusion.step(image((66,60,40,40)),10.4,[
            Candidate((62,60,40,40),0,.8,-1,False,10.2,selected_public_id=99,capture_box=(62,60,40,40))])
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual('visual',result.source)
        self.assertEqual(10.2,result.last_detection_stamp)
        self.assertEqual(10.2,fusion._detector_reference_stamp)
        np.testing.assert_allclose(result.box,(66,60,40,40))
        np.testing.assert_allclose(fusion._detector_velocity,(20,0))
        self.assertEqual((62,60,40,40),fusion.last_detector_candidate.capture_box)

    def test_unknown_selection_does_not_merge_overlapping_air_and_ground_candidates(self):
        fusion=self.make();fusion.config['strong_detector_takeover_enabled']=True
        fusion.class_id=-1
        result=fusion.step(image(),10.1,[
            Candidate((60,60,40,40),0,.8,-1,False,10.1,selected_public_id=99),
            Candidate((60,60,40,40),2,.8,-1,False,10.1,selected_public_id=99)])
        self.assertEqual(0,fusion.diagnostics['strong_takeover_duplicates'])
        self.assertEqual(0,fusion.diagnostics['local_duplicate_candidates'])
        self.assertEqual('ambiguous',result.source)
        self.assertEqual(-1,result.class_id)

    def test_visual_drift_cannot_move_strong_detector_history(self):
        fusion=self.make();fusion.config['strong_detector_takeover_enabled']=True
        fusion.step(image(),10.1,[Candidate((60,60,40,40),0,.8,-1,False,10.1,selected_public_id=99)])
        fusion.box=(140,100,40,40);fusion.velocity=np.array([900.,700.])
        fusion.tracker.valid=False
        result=fusion.step(image(),10.2,[Candidate((61,60,40,40),0,.8,-1,False,10.2,selected_public_id=99)])
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual((61,60,40,40),result.box)

    def test_strong_detection_resets_failed_local_tracking_without_lk_veto(self):
        fusion=self.make();fusion.config['strong_detector_takeover_enabled']=True
        fusion.tracker.valid=False
        result=fusion.step(image(),10.1,[Candidate((61,60,40,40),0,.8,-1,False,10.1,selected_public_id=99)])
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual((61,60,40,40),result.box)
        self.assertEqual(10.1,result.last_detection_stamp)
        self.assertEqual(5,result.local_id)
        self.assertEqual((61,60,40,40),fusion.tracker.box)

    def test_strong_takeover_rejects_cross_family_and_unreachable_boxes(self):
        for category,box in [(2,(60,60,40,40)),(0,(190,140,40,40))]:
            fusion=self.make();fusion.config['strong_detector_takeover_enabled']=True
            result=fusion.step(image(),10.1,[Candidate(box,category,.9,-1,False,10.1,selected_public_id=99)])
            self.assertNotEqual('strong_detector_takeover',result.reason)
            self.assertEqual(10.,result.last_detection_stamp)

    def test_contained_strong_duplicate_prefers_locked_geometry(self):
        fusion=self.make();fusion.config['strong_detector_takeover_enabled']=True
        result=fusion.step(image(),10.1,[
            Candidate((60,60,40,40),0,.7,-1,False,10.1,selected_public_id=99),
            Candidate((43,60,75,40),0,.9,-1,False,10.1,selected_public_id=99)])
        self.assertEqual('strong_detector_takeover',result.reason)
        self.assertEqual((60,60,40,40),result.box)
        self.assertEqual(1,fusion.diagnostics['strong_takeover_duplicates'])

    def test_strong_competitors_do_not_refresh_detector_time_or_override_current_pixels(self):
        fusion = self.make()
        fusion.config['continuous_selected_flow'] = True
        proposals = [Candidate(box, 0, .7, -1, False, 10.1, selected_public_id=99)
                     for box in [(60, 60, 40, 40), (62, 60, 40, 40)]]
        result = fusion.step(image(), 10.1, proposals)
        self.assertEqual('visual', result.source)
        self.assertEqual(10., result.last_detection_stamp)
        self.assertFalse(fusion.diagnostics['branches']['detector']['accepted'])

    def test_unique_current_pixel_match_wins_over_stronger_nearby_vehicle(self):
        fusion = self.make()
        fusion.config['continuous_selected_flow'] = True
        result = fusion.step(image(), 10.1, [
            Candidate((60, 60, 40, 40), 0, .3, -1, False, 10.1, selected_public_id=99),
            Candidate((74, 60, 40, 40), 0, .9, -1, False, 10.1, selected_public_id=99)])
        self.assertTrue(fusion.diagnostics['current_pixels_isolated_candidate'])
        self.assertEqual({}, fusion.diagnostics['rejected'])
        self.assertGreater(iou(result.box, (60, 60, 40, 40)), .95)
        self.assertEqual(5, result.local_id)
        self.assertEqual(10.1, result.last_detection_stamp)

    def test_weak_competitors_cannot_override_verified_current_pixels(self):
        fusion = self.make()
        fusion.config['continuous_selected_flow'] = True
        proposals = [Candidate(box, 0, .3, -1, False, 10.1, selected_public_id=99)
                     for box in [(60, 60, 40, 40), (62, 60, 40, 40)]]
        result = fusion.step(image(), 10.1, proposals)
        self.assertEqual('visual', result.source)
        self.assertTrue(fusion.diagnostics['detector_competitors_discarded'])
        self.assertEqual(10., result.last_detection_stamp)
        self.assertEqual(5, result.local_id)

    def test_competitors_still_require_ambiguity_without_independent_pixels(self):
        for confidence, continuous in [(.3, False), (.7, False)]:
            fusion = self.make()
            fusion.config['continuous_selected_flow'] = continuous
            proposals = [Candidate(box, 0, confidence, -1, False, 10.1, selected_public_id=99)
                         for box in [(60, 60, 40, 40), (62, 60, 40, 40)]]
            result = fusion.step(image(), 10.1, proposals)
            self.assertEqual('ambiguous', result.source)
            self.assertEqual(10., result.last_detection_stamp)

    def make(self):
        fusion = BoxFusion({'minimum_detector_confidence': .45,
                            'minimum_maintenance_confidence': .04,
                            'class_groups': [[0, 1], [2]]}, tracker=LocalTracker())
        fusion.initialize(image(), (60, 60, 40, 40), 10., 99,
                          Candidate((60, 60, 40, 40), 0, .9, 5, True, 10.))
        return fusion

    def test_raw_weak_proposal_can_only_maintain_its_existing_selection(self):
        fusion = self.make()
        raw = Candidate((60, 60, 40, 40), 0, .08, -1, False, 10.1,
                        selected_public_id=99, capture_box=(60, 60, 40, 40))
        result = fusion.step(image(), 10.1, [raw])
        self.assertEqual(10.1, result.last_detection_stamp)
        self.assertEqual(99, result.public_id)
        self.assertEqual(5, result.local_id)
        self.assertFalse(raw.stable)
        self.assertEqual(-1, raw.local_id)

    def test_old_selection_and_cross_family_raw_proposals_cannot_renew_identity(self):
        for identity, category in [(98, 0), (99, 2)]:
            fusion = self.make()
            raw = Candidate((60, 60, 40, 40), category, .9, -1, False, 10.1,
                            selected_public_id=identity)
            result = fusion.step(image(), 10.1, [raw])
            self.assertEqual(10., result.last_detection_stamp)
            self.assertEqual(0, result.class_id)

class ClassAgnosticLockTest(unittest.TestCase):
    def test_category_change_does_not_drop_identity_but_geometry_still_gates_it(self):
        for unrestricted in [False,True]:
            local=LocalTracker()
            fusion=BoxFusion({'class_agnostic_lock':unrestricted,'class_groups':[[0],[2]],
                'follow_upstream_class_consensus':True},tracker=local)
            fusion.initialize(image(),(60,60,40,40),10.,99,Candidate((60,60,40,40),0,.95,5,True,10.))
            result=fusion.step(image(),10.1,[Candidate((60,60,40,40),7,.95,5,True,10.1)])
            self.assertEqual(result.public_id,99)
            self.assertEqual(result.last_detection_stamp,10.1 if unrestricted else 10.)
            self.assertEqual(result.class_id,7 if unrestricted else 0)
            before=fusion.last_detection_stamp
            fusion.step(image(),10.2,[Candidate((200,140,40,40),42,.99,5,True,10.2)])
            self.assertEqual(fusion.last_detection_stamp,before)

class CalibratedDetectorHandoffTest(unittest.TestCase):
    def make(self):
        local=LocalTracker();fusion=BoxFusion(tracker=local)
        fusion.initialize(image(),(60,60,40,40),10.,99,Candidate((60,60,40,40),2,.95,5,True,10.))
        return fusion,local
    def projected(self,identity=5,box=(60,60,40,40)):
        return Candidate(box,2,.95,identity,True,10.1,camera_projected=True)
    def test_projected_real_detector_renews_original_time_but_uses_current_visual_pixels(self):
        fusion,local=self.make()
        result=fusion.step(image(),10.2,[self.projected()],camera_transform=np.eye(3),camera_motion_reliable=True)
        self.assertTrue(result.observed);self.assertEqual(result.source,'visual')
        self.assertTrue(fusion.diagnostics['camera_projection_verified'])
        self.assertEqual(result.public_id,99);self.assertEqual(result.last_detection_stamp,10.1)
        self.assertEqual(result.box,local.box);self.assertEqual(local.initializations,1)
    def test_projection_cannot_change_identity_recover_or_replace_failed_local_vision(self):
        for condition in ['new_id','lost','failed_local','spatial_disagreement','unreliable_motion']:
            fusion,local=self.make();candidate=self.projected()
            if condition=='new_id':candidate=self.projected(identity=6)
            elif condition=='lost':fusion._result(None,'lost','test_loss',0.)
            elif condition=='failed_local':local.valid=False
            elif condition=='spatial_disagreement':candidate=self.projected(box=(120,60,40,40))
            result=fusion.step(image(),10.2,[candidate],camera_transform=np.eye(3),
                camera_motion_reliable=condition!='unreliable_motion')
            self.assertEqual(fusion.last_detection_stamp,10.,condition)
            self.assertEqual(result.public_id,99)
            self.assertFalse(fusion.diagnostics.get('camera_projection_verified',False),condition)
    def test_same_capture_correction_does_not_manufacture_another_visual_update(self):
        fusion,local=self.make()
        fusion.step(image(),10.2,[],detector_updated=False,camera_transform=np.eye(3))
        before=local.updates
        fusion.step(image(),10.2,[self.projected()],same_frame_correction=True,
            camera_transform=np.eye(3),camera_motion_reliable=True)
        self.assertEqual(local.updates,before);self.assertEqual(fusion.last_detection_stamp,10.)



class ForegroundAppearanceTest(unittest.TestCase):
    def test_off_centre_vehicle_cannot_seed_its_identity_from_background_at_box_centre(self):
        from sar_yolo_detector.box_fusion import appearance
        frame=np.full((180,220,3),(25,95,30),np.uint8)
        frame[55:125,64:86]=[3,3,3]
        loose,_=appearance(frame,(60,50,80,80))
        tight,_=appearance(frame,(60,50,32,80))
        grass,_=appearance(frame,(145,50,40,80))
        self.assertGreater(float(np.dot(loose,tight)),.9)
        self.assertLess(float(np.dot(loose,grass)),.7)

    def test_black_vehicle_reference_does_not_learn_grass_or_match_white_vehicle(self):
        from sar_yolo_detector.box_fusion import appearance
        frame=np.full((160,200,3),(25,95,30),np.uint8)
        frame[63:97,75:125]=[4,4,4]
        selected,q=appearance(frame,(60,50,80,60))
        zoomed=cv2.resize(frame,(300,240))
        scaled,_=appearance(zoomed,(90,75,120,90))
        grass,_=appearance(frame,(5,5,40,40))
        white=frame.copy();white[63:97,75:125]=[230,230,230]
        other,_=appearance(white,(60,50,80,60))
        self.assertGreater(q,.35)
        self.assertGreater(float(np.dot(selected,scaled)),.9)
        self.assertLess(float(np.dot(selected,grass)),.7)
        self.assertLess(float(np.dot(selected,other)),.7)



class ClassDuplicateTest(unittest.TestCase):
    def candidate(self, box=(60, 60, 40, 40), category=2, confidence=.9, identity=5):
        return Candidate(box, category, confidence, identity)

    def test_conflicting_vehicle_classes_keep_stronger_box(self):
        candidates = [self.candidate(confidence=.8), self.candidate(category=3, identity=7)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]]), [1])

    def test_selected_id_wins_without_changing_class_or_box(self):
        candidates = [self.candidate(confidence=.6), self.candidate(category=3, identity=7)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]], 5), [0])

    def test_nearby_vehicles_are_not_duplicates(self):
        candidates = [self.candidate(), self.candidate((75, 60, 40, 40), category=3, identity=7)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]]), [0, 1])

    def test_same_class_competitors_remain_for_ambiguity_gate(self):
        candidates = [self.candidate(), self.candidate(identity=7)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]]), [0, 1])

    def test_different_parents_and_nested_background_boxes_remain(self):
        candidates = [self.candidate(), self.candidate(category=4),
                      self.candidate((20, 20, 120, 120), category=3)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3], [4]]), [0, 1, 2])

    def test_empty_input_and_stable_tie_break(self):
        self.assertEqual(class_duplicate_indices([], [[0, 1, 2, 3]]), [])
        candidates = [self.candidate(), self.candidate(category=3)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]]), [0])


class BoxFusionTest(unittest.TestCase):
    def setUp(self):
        self.local = LocalTracker()
        self.fusion = BoxFusion(tracker=self.local)
        self.frame = image()
        self.candidate = Candidate((60, 60, 40, 40), 2, .90, 5)
        self.fusion.initialize(self.frame, self.candidate.box, 10., 900000001, self.candidate)

    def test_agreement_returns_one_fused_box(self):
        result = self.fusion.step(self.frame, 10.1, [self.candidate])
        self.assertEqual(result.source, "fused")
        self.assertEqual(result.public_id, 900000001)
        self.assertTrue(result.observed)
        self.assertEqual(self.local.initializations, 1)

    def test_fresh_upstream_consensus_can_correct_class_without_rebinding(self):
        self.fusion.config['follow_upstream_class_consensus']=True
        revised=Candidate(self.candidate.box,3,.9,5,True,10.1)
        result=self.fusion.step(self.frame,10.1,[revised])
        self.assertEqual(result.class_id,3)
        self.assertEqual(result.public_id,900000001)
        self.assertEqual(result.local_id,5)

    def test_visual_hold_does_not_change_consensus_class(self):
        self.fusion.config['follow_upstream_class_consensus']=True
        result=self.fusion.step(self.frame,10.1,[])
        self.assertEqual(result.class_id,2)

    def test_strong_detector_not_pulled_back_by_valid_local_lag(self):
        candidate = Candidate((62, 60, 40, 40), 2, .9, 5)
        result = self.fusion.step(image(candidate.box), 10.1, [candidate])
        self.assertEqual(result.source, "fused")
        self.assertEqual(result.box, candidate.box)

    def test_moderate_detector_still_blends_compatible_witnesses(self):
        candidate = Candidate((62, 60, 40, 40), 2, .7, 5)
        result = self.fusion.step(image(candidate.box), 10.1, [candidate])
        self.assertEqual(result.source, "fused")
        self.assertGreater(result.box[0], 60.)
        self.assertLess(result.box[0], 62.)

    def test_missing_yolo_uses_visual_not_detector(self):
        result = self.fusion.step(self.frame, 10.1, [])
        self.assertEqual(result.source, "visual")
        self.assertEqual(result.last_detection_stamp, 10.)
        self.assertLessEqual(result.confidence, .65)

    def test_weak_detection_maintains_only_agreeing_current_identity(self):
        weak = Candidate(self.candidate.box, 2, .2, 5)
        result = self.fusion.step(self.frame, 10.1, [weak])
        self.assertEqual(result.source, "fused")
        self.assertEqual(result.last_detection_stamp, 10.1)
        self.assertEqual(result.public_id, 900000001)

    def test_weak_detection_cannot_recover_new_local_identity(self):
        self.local.valid = False
        weak = Candidate(self.candidate.box, 2, .2, 17)
        for stamp in (10.1, 10.2, 10.3):
            result = self.fusion.step(self.frame, stamp, [weak])
        self.assertFalse(result.observed)
        self.assertEqual(result.local_id, 5)
        self.assertEqual(result.last_detection_stamp, 10.)

    def test_unknown_manual_visual_hold_does_not_refresh_itself(self):
        self.fusion.initialize(self.frame, self.candidate.box, 10., 900000002)
        for stamp in np.arange(10.1, 12., .1):
            result = self.fusion.step(self.frame, float(stamp), [])
        self.assertFalse(result.observed)
        self.assertEqual(result.last_detection_stamp, 0.)

    def test_visual_expiry_not_indefinite(self):
        for stamp in np.arange(10.1, 12., .1):
            result = self.fusion.step(self.frame, float(stamp), [])
        self.assertEqual(result.source, "lost")
        self.assertIsNone(result.box)

    def test_failed_tracker_uses_yolo_and_reinitializes(self):
        self.local.valid = False
        result = self.fusion.step(self.frame, 10.1, [self.candidate])
        self.assertEqual(result.source, "yolo")
        self.assertEqual(self.local.initializations, 2)

    def test_one_failed_capture_does_not_require_same_id_rebinding(self):
        self.local.valid = False
        self.assertFalse(self.fusion.step(self.frame, 10.1, [], detector_updated=False).observed)
        result = self.fusion.step(self.frame, 10.1,
            [Candidate(self.candidate.box, 2, .9, 5, True, 10.1)], same_frame_correction=True)
        self.assertTrue(result.observed, result.reason)
        self.assertEqual(result.local_id, 5)
        self.assertEqual(self.fusion.pending_count, 0)

    def test_valid_same_id_flow_maintains_lock_when_kcf_fails(self):
        self.local.valid = False
        cue = Candidate((62, 60, 40, 40), 2, .65, 5, True, 10.)
        result = self.fusion.step(image(cue.box), 10.1, [], detector_updated=False,
            visual_observation=cue, visual_quality=.9)
        self.assertEqual(result.source, 'visual')
        self.assertEqual(result.box, cue.box)
        self.assertEqual(result.last_detection_stamp, 10.)

    def test_roi_colour_is_used_as_a_bounded_visual_witness(self):
        detector = Candidate((62, 60, 40, 40), 2, .75, 5)
        color = Candidate((62, 60, 40, 40), 2, .8, 5, True, None)
        result = self.fusion.step(image(detector.box), 10.1, [detector],
            color_observation=color, color_quality=.8)
        self.assertTrue(result.observed)
        self.assertEqual(result.source, "fused")
        self.assertTrue(self.fusion.diagnostics["color_selected_observation"])
        self.assertEqual(self.fusion.diagnostics["color_support"], "local_visual_agreement")

    def test_roi_colour_cannot_rebind_identity_or_renew_detector_time(self):
        self.local.valid = False
        impostor = Candidate((62, 60, 40, 40), 2, .9, 17, True, None)
        result = self.fusion.step(image(impostor.box), 10.1, [], detector_updated=False,
            color_observation=impostor, color_quality=.9)
        self.assertFalse(result.observed)
        self.assertEqual(result.local_id, 5)
        self.assertEqual(result.last_detection_stamp, 10.)
        self.assertFalse(self.fusion.diagnostics["color_selected_observation"])

    def test_flow_cannot_rebind_or_bypass_quality_expiry_or_bounds(self):
        self.local.valid = False
        for cue, quality in ((Candidate(self.candidate.box, 2, .65, 17, True, 10.), .9),
                             (Candidate(self.candidate.box, 2, .65, 5, True, 10.), .3),
                             (Candidate((170, 60, 40, 40), 2, .65, 5, True, 10.), .9),
                             (Candidate(self.candidate.box, 2, .65, 5, True, 8.), .9)):
            fusion = BoxFusion(tracker=LocalTracker())
            fusion.initialize(self.frame, self.candidate.box, 10., 99, self.candidate)
            fusion.tracker.valid = False
            result = fusion.step(self.frame, 10.1, [], detector_updated=False,
                visual_observation=cue, visual_quality=quality)
            self.assertFalse(result.observed)
            self.assertEqual(fusion.local_id, 5)

    def test_detector_refreshes_kcf_at_bounded_cadence(self):
        for stamp in (10.2, 10.4, 10.6):
            self.fusion.step(self.frame, stamp, [self.candidate])
        self.assertEqual(self.local.initializations, 1)
        self.fusion.step(self.frame, 10.8, [self.candidate])
        self.assertEqual(self.local.initializations, 2)

    def test_flow_scale_metadata_cannot_bypass_scale_gate(self):
        cue = Candidate(self.candidate.box, 2, .65, 5, True, 10.)
        for ratio in (0., .1, 3., float('nan'), float('inf')):
            fusion = BoxFusion(tracker=LocalTracker())
            fusion.initialize(self.frame, self.candidate.box, 10., 99, self.candidate)
            fusion.tracker.valid = False
            result = fusion.step(self.frame, 10.1, [], detector_updated=False,
                visual_observation=cue, visual_quality=.9, visual_scale_ratio=ratio)
            self.assertFalse(result.observed)

    def test_prediction_expires_and_is_not_observed(self):
        self.local.valid = False
        result = self.fusion.step(self.frame, 10.1, [])
        self.assertEqual(result.source, "prediction")
        self.assertFalse(result.observed)
        result = self.fusion.step(self.frame, 10.5, [])
        self.assertEqual(result.source, "lost")
        self.assertIsNone(result.box)

    def test_competing_identical_vehicles_fail_closed(self):
        competitor = Candidate((61, 60, 40, 40), 2, .9, 6)
        result = self.fusion.step(self.frame, 10.1, [self.candidate, competitor])
        self.assertEqual(result.source, "ambiguous")
        self.assertFalse(result.observed)

    def test_new_local_id_needs_consecutive_confirmation(self):
        new = Candidate(self.candidate.box, 2, .95, 17)
        for stamp in (10.1, 10.2):
            result = self.fusion.step(self.frame, stamp, [new])
            self.assertEqual(result.source, "visual")
            self.assertTrue(result.observed)
            self.assertEqual(result.local_id, 5)
            self.assertEqual(result.last_detection_stamp, 10.)
        result = self.fusion.step(self.frame, 10.3, [new])
        self.assertTrue(result.observed)
        self.assertEqual(result.local_id, 17)
        self.assertEqual(result.public_id, 900000001)

    def test_confirmation_without_valid_visual_still_fails_closed(self):
        self.local.valid = False
        new = Candidate(self.candidate.box, 2, .95, 17)
        for stamp in (10.1, 10.2):
            result = self.fusion.step(self.frame, stamp, [new])
            self.assertEqual(result.source, "ambiguous")
            self.assertFalse(result.observed)
        self.assertEqual(self.fusion.step(self.frame, 10.3, [new]).local_id, 17)

    def test_confirmation_does_not_extend_visual_hold_budget(self):
        new = Candidate(self.candidate.box, 2, .95, 17)
        for stamp in (10.5, 11., 11.5):
            self.fusion.step(self.frame, stamp, [])
        result = self.fusion.step(self.frame, 11.6, [new])
        self.assertFalse(result.observed)
        self.assertEqual(result.last_detection_stamp, 10.)

    def test_confirmation_visual_must_agree_with_candidate(self):
        # A motion-compatible box can still contradict the local observation.
        other = Candidate((85, 60, 40, 40), 2, .95, 17)
        frame = image()
        frame[60:100, 85:125] = self.frame[60:100, 60:100]
        result = self.fusion.step(frame, 10.1, [other])
        self.assertFalse(result.observed)

    def test_two_simultaneously_visible_tracks_never_stitched(self):
        new = Candidate(self.candidate.box, 2, .95, 17)
        far_old = Candidate((180, 120, 40, 40), 2, .9, 5)
        result = self.fusion.step(self.frame, 10.1, [new, far_old])
        self.assertEqual(result.reason, "old_and_new_tracks_simultaneously_visible")
        self.assertFalse(result.observed)

    def test_unstable_local_identity_not_rebound(self):
        self.local.valid = False
        new = Candidate(self.candidate.box, 2, .95, 17, False)
        self.assertFalse(self.fusion.step(self.frame, 10.1, [new]).observed)

    def test_duplicate_timestamp_does_not_confirm(self):
        new = Candidate(self.candidate.box, 2, .95, 17)
        self.fusion.step(self.frame, 10.1, [new])
        updates = self.local.updates
        for _ in range(4):
            self.assertFalse(self.fusion.step(self.frame, 10.1, [new]).observed)
        self.assertEqual(self.local.updates, updates)
        self.assertEqual(self.fusion.pending_count, 1)

    def test_switching_pending_candidate_resets_confirmation(self):
        self.fusion.step(self.frame, 10.1, [Candidate(self.candidate.box, 2, .95, 17)])
        self.fusion.step(self.frame, 10.2, [Candidate(self.candidate.box, 2, .95, 18)])
        self.assertEqual(self.fusion.pending_count, 1)

    def test_wrong_class_rejected(self):
        self.local.valid = False
        result = self.fusion.step(self.frame, 10.1, [Candidate(self.candidate.box, 8, .95, 5)])
        self.assertEqual(result.source, "prediction")

    def test_vehicle_class_flip_does_not_change_identity(self):
        result = self.fusion.step(self.frame, 10.1, [Candidate(self.candidate.box, 1, .95, 5)])
        self.assertTrue(result.observed)
        self.assertEqual(result.class_id, 2)
        self.assertEqual(result.public_id, 900000001)

    def test_manual_unknown_not_fabricated_as_tank(self):
        result = self.fusion.initialize(self.frame, self.candidate.box, 10., 900000002)
        self.assertEqual(result.class_id, -1)
        for stamp in (10.1, 10.2, 10.3):
            result = self.fusion.step(self.frame, stamp, [self.candidate])
        self.assertEqual(result.class_id, 2)
        self.assertEqual(result.public_id, 900000002)

    def test_low_quality_background_not_visual_lock(self):
        result = self.fusion.initialize(np.full_like(self.frame, 24), self.candidate.box, 10., 900000002)
        self.assertEqual(result.source, "lost")
        self.assertIsNone(result.box)

    def test_false_same_id_wrong_position_rejected(self):
        false = Candidate((170, 60, 40, 40), 2, .99, 5)
        self.local.valid = False
        self.assertFalse(self.fusion.step(image(false.box), 10.1, [false]).observed)

    def test_nan_candidate_ignored(self):
        false = Candidate((float("nan"), 60, 40, 40), 2, .99, 5)
        self.local.valid = False
        self.assertFalse(self.fusion.step(self.frame, 10.1, [false]).observed)

    def test_cumulative_scale_drift_rejected(self):
        self.local.box = (30, 30, 100, 100)
        result = self.fusion.step(self.frame, 10.1, [])
        self.assertFalse(result.observed)

    def test_long_gap_disables_local_tracker(self):
        result = self.fusion.step(self.frame, 11., [])
        self.assertFalse(result.observed)
        self.assertIsNone(self.local.box)

    def test_frame_dimensions_change_fails_closed(self):
        result = self.fusion.step(np.zeros((100, 100, 3), np.uint8), 10.1, [])
        self.assertEqual(result.reason, "image_dimensions_changed")

    def test_long_recovery_rejects_unknown_camera_motion(self):
        self.local.valid = False
        result = self.fusion.step(self.frame, 19., [self.candidate])
        self.assertEqual(result.reason, "identity_memory_waiting")
        self.assertFalse(result.observed)
        self.assertIsNotNone(self.fusion.identity_archive)

    def _long_absence(self):
        self.local.valid = False
        self.fusion.velocity[:] = 0.
        for stamp in np.arange(10.2, 19., .2):
            self.fusion.step(self.frame, float(stamp), [], camera_transform=np.eye(3))

    def test_long_recovery_rebinds_only_after_three_fresh_detections(self):
        self._long_absence()
        new = Candidate(self.candidate.box, 2, .95, 17)
        initializations = self.local.initializations
        for stamp in (19., 19.2):
            result = self.fusion.step(self.frame, stamp, [new], camera_transform=np.eye(3))
            self.assertEqual(result.source, "ambiguous")
            self.assertEqual(result.local_id, 5)
        result = self.fusion.step(self.frame, 19.4, [new], camera_transform=np.eye(3))
        self.assertTrue(result.observed)
        self.assertEqual(result.public_id, 900000001)
        self.assertEqual(result.local_id, 17)
        self.assertEqual(self.local.initializations, initializations + 1)
        self.assertIsNone(self.fusion.identity_archive)
        np.testing.assert_equal(self.fusion.velocity, [0., 0.])

    def test_long_recovery_same_local_id_also_requires_confirmation(self):
        self._long_absence()
        result = self.fusion.step(self.frame, 19., [self.candidate], camera_transform=np.eye(3))
        self.assertEqual(result.source, "ambiguous")

    def test_long_recovery_never_accepts_appearance_without_enough_motion_evidence(self):
        self._long_absence()
        far = Candidate((110, 60, 40, 40), 2, .99, 17)
        for stamp in (19., 19.2, 19.4):
            result = self.fusion.step(image(far.box), stamp, [far], camera_transform=np.eye(3))
        self.assertFalse(result.observed)
        self.assertIn('long_identity_score', self.fusion.diagnostics['rejected'])

    def test_rotation_only_or_untrusted_motion_is_not_long_identity_evidence(self):
        self.local.valid = False
        for stamp in np.arange(10.2, 19.6, .2):
            result = self.fusion.step(self.frame, float(stamp), [self.candidate] if stamp >= 19 else [],
                camera_transform=np.eye(3), camera_motion_reliable=False)
        self.assertFalse(result.observed)
        self.assertEqual(self.fusion.pending_count, 0)
        self.assertIn('camera_motion_chain_untrusted', self.fusion.diagnostics['rejected'])

    def test_long_recovery_competitors_do_not_restore_original_id(self):
        self._long_absence()
        candidates = [Candidate(self.candidate.box, 2, .95, identity) for identity in (17, 18)]
        for stamp in (19., 19.2, 19.4):
            result = self.fusion.step(self.frame, stamp, candidates, camera_transform=np.eye(3))
            self.assertEqual(result.reason, "competing_detections")
            self.assertFalse(result.observed)
        self.assertEqual(self.fusion.pending_count, 0)

    def test_identity_archive_ttl_is_not_refreshed_by_pending_matches(self):
        self._long_absence()
        expiry = self.fusion.identity_archive['expires_at']
        self.fusion.step(self.frame, 19., [Candidate(self.candidate.box, 2, .95, 17)], camera_transform=np.eye(3))
        self.assertEqual(self.fusion.identity_archive['expires_at'], expiry)
        result = self.fusion.step(self.frame, 70.1, [self.candidate])
        self.assertEqual(result.reason, "identity_memory_expired")
        self.assertFalse(result.observed)
        self.assertIsNone(self.fusion.identity_archive)
        self.assertIsNone(self.fusion.anchor)

    def test_rebound_clears_stale_velocity_even_for_short_absence(self):
        self.local.valid = False
        self.fusion.velocity[:] = (1., 0.)
        new = Candidate(self.candidate.box, 2, .95, 17)
        for stamp in (10.1, 10.2, 10.3):
            result = self.fusion.step(self.frame, stamp, [new])
        self.assertTrue(result.observed)
        np.testing.assert_equal(self.fusion.velocity, [0., 0.])

    def test_identity_memory_configuration_bounds(self):
        for ttl in (8., 301., float('inf')):
            with self.assertRaises(ValueError):
                BoxFusion({'identity_memory_ttl_sec': ttl})

    def test_pending_same_id_spatial_jump_cannot_confirm(self):
        self.local.valid = False
        for stamp, x in ((10.1, 60), (10.2, 90), (10.3, 60)):
            candidate = Candidate((x, 60, 40, 40), 2, .95, 17)
            result = self.fusion.step(image(candidate.box), stamp, [candidate])
        self.assertFalse(result.observed)
        self.assertEqual(self.fusion.pending_count, 1)
        self.assertTrue(self.fusion.diagnostics['pending_motion_rejected'])

    def test_actual_kcf_long_absence_reinitializes_and_recovers_public_id(self):
        if not hasattr(cv2, 'TrackerKCF_create') and not hasattr(getattr(cv2, 'legacy', object()), 'TrackerKCF_create'):
            self.skipTest('OpenCV contrib KCF not installed')
        fusion = BoxFusion(manual_config={'feature_tracker': {'tracker_algorithm': 'kcf'}})
        fusion.initialize(self.frame, self.candidate.box, 10., 99, self.candidate)
        blank = np.full_like(self.frame, 24)
        for index in range(1, 91):
            result = fusion.step(blank, 10.+index*.1, [], camera_transform=np.eye(3))
        self.assertFalse(result.observed)
        new = Candidate((70, 60, 40, 40), 2, .95, 17)
        for stamp in (19.1, 19.2, 19.3):
            result = fusion.step(image(new.box), stamp, [new], camera_transform=np.eye(3))
        self.assertTrue(result.observed)
        self.assertEqual(result.public_id, 99)
        self.assertEqual(result.local_id, 17)
        self.assertTrue(fusion.diagnostics['local_reinitialized'])
        self.assertGreater(iou(result.box, new.box), .99)

    def test_motion_uses_elapsed_time(self):
        second = Candidate((64, 60, 40, 40), 2, .9, 5)
        self.local.valid = False
        self.fusion.step(image(second.box), 10.2, [second])
        self.assertAlmostEqual(self.fusion.velocity[0], 8.)
        result = self.fusion.step(image(second.box), 10.3, [])
        self.assertEqual(result.source, "prediction")
        self.assertAlmostEqual(result.box[0], 64.8)

    def test_invalid_config_rejected(self):
        for config in ({"visual_hold_sec": -1}, {"detector_weight": 2}, {"max_frame_gap_sec": float("nan")}):
            with self.assertRaises(ValueError):
                BoxFusion(config)

    def test_actual_kcf_tracks_textured_moving_target(self):
        if not hasattr(cv2, "TrackerKCF_create") and not hasattr(getattr(cv2, "legacy", object()), "TrackerKCF_create"):
            self.skipTest("OpenCV contrib KCF not installed")
        fusion = BoxFusion(manual_config={"feature_tracker": {"tracker_algorithm": "kcf"}})
        fusion.initialize(self.frame, self.candidate.box, 10., 99, self.candidate)
        self.assertEqual(fusion.tracker.tracker_name, "kcf")
        for index in range(1, 11):
            box = (60 + index, 60, 40, 40)
            detection = Candidate(box, 2, .9, 5)
            result = fusion.step(image(box), 10 + index * .05, [] if index % 3 else [detection])
            self.assertTrue(result.observed, result.reason)
            self.assertGreater(iou(result.box, box), .70)
        self.assertEqual(result.public_id, 99)

    def test_actual_kcf_turn_gap_and_new_id_keep_all_120_observations(self):
        if not hasattr(cv2, "TrackerKCF_create") and not hasattr(getattr(cv2, "legacy", object()), "TrackerKCF_create"):
            self.skipTest("OpenCV contrib KCF not installed")
        texture = np.random.RandomState(31).randint(0, 200, (40, 40, 3), dtype=np.uint8)
        def scene(index):
            frame = np.full((200, 320, 3), 20, np.uint8)
            frame[70:110, 60+index:100+index] = cv2.warpAffine(texture,
                cv2.getRotationMatrix2D((20, 20), index*.7, 1.), (40, 40), borderMode=cv2.BORDER_REFLECT)
            return frame
        fusion = BoxFusion(manual_config={"feature_tracker": {"tracker_algorithm": "kcf"}})
        fusion.initialize(scene(0), (60, 70, 40, 40), 10., 99, Candidate((60, 70, 40, 40), 2, .9, 5))
        for index in range(1, 121):
            box, fresh = (60+index, 70, 40, 40), index % 5 == 0
            detections = [Candidate(box, 2, .9, 5 if index < 50 else 17)] if fresh and not 60 <= index <= 75 else []
            result = fusion.step(scene(index), 10.+index/30., detections, detector_updated=fresh)
            self.assertTrue(result.observed, "%d: %s" % (index, result.reason))
            self.assertEqual(result.public_id, 99)
            self.assertGreater(iou(result.box, box), .85)
            if detections and fusion.local_id == detections[0].local_id:
                self.assertEqual(result.box, box)





class BranchReliabilityTest(unittest.TestCase):
    def test_weak_colour_cannot_pull_feature_box_or_gain_confidence(self):
        tracker=LocalTracker()
        fusion=BoxFusion(tracker=tracker)
        frame=image()
        fusion.initialize(frame,(60,60,40,40),10.,1,Candidate((60,60,40,40),2,.9,5))
        color=Candidate((64,60,40,40),2,.08,5,True,None)
        result=fusion.step(frame,10.1,[],detector_updated=False,color_observation=color,color_quality=.08)
        branches=fusion.diagnostics['branches']
        self.assertEqual(set(branches),{'feature','flow','color','detector'})
        self.assertTrue(branches['feature']['accepted'])
        self.assertLessEqual(branches['color']['score'],.08)
        self.assertLessEqual(branches['color']['blend_weight'],.0161)
        self.assertLess(abs(result.box[0]-60),.1)
        self.assertFalse(branches['detector']['accepted'])

    def test_low_quality_feature_is_rejected(self):
        tracker=LocalTracker()
        tracker.update=lambda frame:SimpleNamespace(bbox=(60,60,40,40),valid=True,confidence=.05)
        fusion=BoxFusion(tracker=tracker)
        frame=image();fusion.initialize(frame,(60,60,40,40),10.,1,Candidate((60,60,40,40),2,.9,5))
        result=fusion.step(frame,10.1,[],detector_updated=False)
        self.assertFalse(result.observed)
        self.assertFalse(fusion.diagnostics['branches']['feature']['accepted'])


if __name__ == "__main__":
    unittest.main()
