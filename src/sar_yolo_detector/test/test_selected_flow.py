import unittest
from types import SimpleNamespace
import cv2
import numpy as np
from sar_yolo_detector.selected_flow import selected_flow
from sar_yolo_detector.box_fusion import BoxFusion,Candidate,iou

class Local:
    tracker_name='test_local'
    def initialize(self,frame,box):self.box=box
    def reset(self):pass
    def update(self,frame):return SimpleNamespace(valid=True,bbox=self.box,confidence=.95)

class SelectedFlowTest(unittest.TestCase):
    def test_short_reference_tracks_view_change_but_cannot_replace_identity_anchor(self):
        from unittest.mock import patch
        frame=self.image();box=(60,60,40,40);local=Local()
        f=BoxFusion({'continuous_selected_flow':True,'joint_current_visual_witnesses':True,
            'adaptive_joint_appearance':True,'visual_hold_sec':.2},tracker=local)
        witness=(box,dict(valid=True,points=24,inlier_ratio=1.,fb_error_median_px=.01))
        with patch('sar_yolo_detector.box_fusion.appearance',return_value=(np.array([1.,0.]),1.)):
            f.initialize(frame,box,10,20)
        anchor=f.anchor.copy()
        for n in range(1,41):
            feature=np.array([np.cos(n*.01),np.sin(n*.01)])
            with patch('sar_yolo_detector.box_fusion.appearance',return_value=(feature,1.)),patch('sar_yolo_detector.box_fusion.selected_flow',return_value=witness):
                result=f.step(frame,10+n*.1,[],detector_updated=False)
            self.assertTrue(result.observed,(n,f.diagnostics))
        np.testing.assert_array_equal(f.anchor,anchor)
        self.assertEqual(f.last_detection_stamp,0.)
        self.assertGreater(float(np.dot(feature,f._visual_reference)),.99)
        previous=f._visual_reference.copy()
        alien=np.array([np.cos(.6),np.sin(.6)])
        with patch('sar_yolo_detector.box_fusion.appearance',return_value=(alien,1.)),patch('sar_yolo_detector.box_fusion.selected_flow',return_value=witness):
            self.assertFalse(f.step(frame,14.1,[],detector_updated=False).observed)
        np.testing.assert_array_equal(f._visual_reference,previous)

    def test_confirmed_pixel_witness_cannot_be_bypassed_after_target_disappears(self):
        from unittest.mock import patch
        frame = self.image()
        box = (60, 60, 40, 40)
        fusion = BoxFusion({'continuous_selected_flow': True,
                            'joint_current_visual_witnesses': True,
                            'moving_small_pixel_witness': {'enabled': True}}, tracker=Local())
        fusion.initialize(frame, box, 10., 20)
        pixels = fusion.moving_pixel_lock
        self.assertTrue(pixels.applicable)
        flow = (box, dict(valid=True, points=24, inlier_ratio=1., fb_error_median_px=.01))
        with patch.object(pixels, 'update', return_value=(box, {'score': .9, 'valid': True})):
            self.assertTrue(fusion.step(frame, 10.05, [], detector_updated=False).observed)
        pixels.ever_confirmed = True
        with patch.object(pixels, 'update', return_value=(None, {'valid': False})), \
                patch('sar_yolo_detector.box_fusion.selected_flow', return_value=flow):
            # Even if KCF and LK both agree at the same patch, the independent
            # target verifier has rejected these pixels.
            result = fusion.step(frame, 10.10, [], detector_updated=False)
            self.assertTrue(fusion.diagnostics['joint_current_visual_witnesses']['valid'])
            self.assertFalse(result.observed)

    def test_weak_or_missing_flow_never_trains_short_reference(self):
        from unittest.mock import patch
        frame=self.image();box=(60,60,40,40)
        f=BoxFusion({'continuous_selected_flow':True,'joint_current_visual_witnesses':True,
            'adaptive_joint_appearance':True},tracker=Local())
        with patch('sar_yolo_detector.box_fusion.appearance',return_value=(np.array([1.,0.]),1.)):
            f.initialize(frame,box,10,20)
        original=f._visual_reference.copy()
        with patch('sar_yolo_detector.box_fusion.appearance',return_value=(np.array([.99,.1]),1.)),patch('sar_yolo_detector.box_fusion.selected_flow',return_value=(box,dict(valid=True,points=4,inlier_ratio=1.,fb_error_median_px=.01))):
            f.step(frame,10.1,[],detector_updated=False)
        np.testing.assert_array_equal(f._visual_reference,original)
        self.assertFalse(f.diagnostics['joint_current_visual_witnesses']['valid'])

    def image(self,offset=0,occluded=False):
        im=np.full((180,240,3),25,np.uint8)
        if not occluded:im[60:100,60+offset:100+offset]=np.random.RandomState(8).randint(40,210,(40,40,3))
        return im
    def test_unselected_fusion_reports_flow_ineligible_without_running_a_witness(self):
        f=BoxFusion({'continuous_selected_flow':True},tracker=Local())
        r=f.step(self.image(),10,[])
        self.assertFalse(r.observed)
        self.assertEqual(f.diagnostics['selected_target_flow']['reason'],'not_eligible')
        self.assertFalse(f.diagnostics['selected_target_flow']['valid'])

    def test_target_parallax_is_measured_without_ground_motion(self):
        box,e=selected_flow(self.image(),self.image(5),(60,60,40,40))
        self.assertTrue(e['valid']);self.assertGreater(iou(box,(65,60,40,40)),.9)

    def test_foreground_flow_follows_small_car_in_loose_roi_instead_of_noisy_road(self):
        old_threads=cv2.getNumThreads();cv2.setNumThreads(1)
        try:
            rng=np.random.RandomState(91)
            road=rng.randint(10,85,(240,360,1),dtype=np.uint8).repeat(3,2)
            car=np.zeros((20,44,3),np.uint8);car[:,:,1]=rng.randint(35,140,(20,44),dtype=np.uint8)
            before=road.copy();after=road.copy()
            before[90:110,100:144]=car;after[90:110,108:152]=car
            loose=(86,68,72,64)
            old,_=selected_flow(before,after,loose,local_crop=True)
            seed=np.eye(3);seed[0,2]=6.
            current,evidence=selected_flow(before,after,loose,local_crop=True,foreground_only=True,initial_transform=seed)
            self.assertLess(abs(old[0]-loose[0]),1.)
            self.assertTrue(evidence['valid']);self.assertTrue(evidence['foreground_only'])
            self.assertLess(abs(current[0]-(loose[0]+8)),.1)
            self.assertIsNone(selected_flow(before,road,loose,local_crop=True,foreground_only=True,initial_transform=seed)[0])
            empty=np.full_like(road,40)
            self.assertIsNone(selected_flow(empty,empty,loose,local_crop=True,foreground_only=True)[0])
        finally:cv2.setNumThreads(old_threads)

    def test_target_motion_hint_keeps_real_kcf_and_requires_current_local_response(self):
        from unittest.mock import patch
        from sar_yolo_detector.manual_box_tracker import ManualBoxTracker
        old_threads=cv2.getNumThreads();cv2.setNumThreads(1)
        try:
            texture=np.random.RandomState(21).randint(0,255,(48,60,3),dtype=np.uint8)
            def image(n,present=True):
                frame=np.full((240,1000,3),80,np.uint8)
                if present:frame[90:138,80+n*18:140+n*18]=texture
                return frame
            local=ManualBoxTracker({'feature_tracker':{'tracker_algorithm':'kcf',
                'require_opencv_tracker':True,'motion_local_warp':True}})
            fusion=BoxFusion({'continuous_selected_flow':True,'selected_flow_local_crop':True,
                'joint_current_visual_witnesses':True,'target_motion_search_hint':True},tracker=local)
            fusion.initialize(image(0),(80,90,60,48),10,20)
            for n in range(1,20):
                result=fusion.step(image(n),10+n*.05,[],detector_updated=False,
                    camera_transform=np.eye(3),camera_motion_reliable=True)
                self.assertTrue(result.observed)
                self.assertGreater(iou(result.box,(80+n*18,90,60,48)),.95)
                self.assertTrue(fusion.diagnostics['target_motion_search_hint']['used'])
                self.assertTrue(fusion.diagnostics['local_tracker_executed'])
                self.assertEqual(result.last_detection_stamp,0.)
            with patch.object(local,'update',return_value=SimpleNamespace(valid=False,bbox=None,confidence=0.,reason='failed')):
                result=fusion.step(image(20),11.,[],detector_updated=False,camera_transform=np.eye(3))
                self.assertTrue(fusion.diagnostics['selected_target_flow']['valid'])
                self.assertTrue(fusion.diagnostics['target_motion_search_hint']['used'])
                self.assertFalse(result.observed)
            self.assertFalse(fusion.step(image(21,False),11.05,[],detector_updated=False,camera_transform=np.eye(3)).observed)
        finally:cv2.setNumThreads(old_threads)

    def test_joint_current_witnesses_validate_low_contrast_target_without_detector_clock_renewal(self):
        texture=np.random.RandomState(81).randint(83,94,(30,40,3)).astype(np.uint8)
        def image(shift=0,present=True):
            frame=np.full((240,360,3),110,np.uint8)
            if present:frame[60:90,60+shift:100+shift]=texture
            return frame
        local=Local();f=BoxFusion({'continuous_selected_flow':True,'selected_flow_local_crop':True,
            'joint_current_visual_witnesses':True},tracker=local)
        f.initialize(image(),(60,60,40,30),10,20)
        for n in range(1,61):
            shift=n//3;local.box=(60+shift,60,40,30)
            result=f.step(image(shift),10+n*.1,[],detector_updated=False)
            self.assertEqual(result.observed,n>=3,(n,f.diagnostics))
            if n in (1,2):
                before=f._visual_bootstrap_count
                duplicate=f.step(image(shift),10+n*.1,[],detector_updated=True,same_frame_correction=True)
                self.assertFalse(duplicate.observed)
                self.assertEqual(f._visual_bootstrap_count,before)
        self.assertLess(f.diagnostics['branches']['feature']['texture_quality'],.35)
        self.assertEqual(result.last_detection_stamp,0.)
        self.assertEqual(result.class_id,-1)
        self.assertGreaterEqual(result.confidence,.5)
        self.assertFalse(f.step(image(present=False),16.1,[],detector_updated=False).observed)
        self.assertFalse(f.step(image(20),16.2,[],detector_updated=False).observed)
        self.assertFalse(f.step(image(20),16.3,[],detector_updated=False).observed)
        self.assertTrue(f.step(image(20),16.4,[],detector_updated=False).observed)
        for n in range(5,12):self.assertFalse(f.step(image(present=False),16+n*.1,[],detector_updated=False).observed)
        self.assertFalse(f.step(image(20),17.2,[],detector_updated=False).observed)
    def test_occlusion_and_textureless_fail(self):
        box,e=selected_flow(self.image(),self.image(0,True),(60,60,40,40));self.assertIsNone(box)
        box,e=selected_flow(self.image(0,True),self.image(0,True),(60,60,40,40));self.assertIsNone(box)

    def test_native_pixel_crop_preserves_high_resolution_target_motion_and_occlusion_gates(self):
        before=np.full((1080,1920,3),20,np.uint8);after=before.copy()
        texture=np.random.RandomState(88).randint(30,220,(40,40,3)).astype(np.uint8)
        before[600:640,1300:1340]=texture;after[600:640,1305:1345]=texture
        full,a=selected_flow(before,after,(1300,600,40,40))
        local,b=selected_flow(before,after,(1300,600,40,40),local_crop=True)
        self.assertTrue(a['valid']);self.assertTrue(b['valid'])
        self.assertGreater(iou(full,local),.98)
        self.assertGreater(iou(local,(1305,600,40,40)),.98)
        after[:]=20
        self.assertIsNone(selected_flow(before,after,(1300,600,40,40),local_crop=True)[0])
    def test_two_current_visual_witnesses_continue_both_domains_without_detector_clock_renewal(self):
        for cls in [0,2]:
            local=Local();f=BoxFusion({'continuous_selected_flow':True},tracker=local)
            f.initialize(self.image(),(60,60,40,40),10,20,Candidate((60,60,40,40),cls,.95,3,True,10))
            for n in range(1,41):
                r=f.step(self.image(),10+n*.1,[],detector_updated=False)
                self.assertTrue(r.observed,(cls,n,f.diagnostics))
            self.assertTrue(f.diagnostics['continuous_selected_flow_verified'])
            self.assertEqual(r.last_detection_stamp,10);self.assertEqual(r.local_id,3)
    def test_no_recovery_no_large_gap_no_same_frame_renewal(self):
        for condition in ['lost','gap','disagree','occluded']:
            local=Local();f=BoxFusion({'continuous_selected_flow':True},tracker=local)
            f.initialize(self.image(),(60,60,40,40),10,20,Candidate((60,60,40,40),2,.95,3,True,8))
            if condition=='lost':f._result(None,'lost','test_loss')
            if condition=='disagree':local.box=(120,60,40,40)
            frame=self.image(0,condition=='occluded')
            r=f.step(frame,11 if condition=='gap' else 10.1,[],detector_updated=False)
            self.assertFalse(r.observed,condition)
            self.assertEqual(r.last_detection_stamp,8)
    def test_unclassified_manual_roi_can_follow_pixels_without_becoming_detection(self):
        f=BoxFusion({'continuous_selected_flow':True},tracker=Local())
        f.initialize(self.image(),(60,60,40,40),10,20)
        for n in range(1,41):
            r=f.step(self.image(),10+n*.1,[],detector_updated=False)
            self.assertTrue(r.observed)
        self.assertEqual(r.local_id,-1);self.assertEqual(r.class_id,-1)
        self.assertEqual(r.last_detection_stamp,0);self.assertEqual(r.public_id,20)

    def test_weak_same_capture_packet_cannot_erase_independent_current_visual(self):
        local=Local();f=BoxFusion({'continuous_selected_flow':True},tracker=local)
        f.initialize(self.image(),(60,60,40,40),10,20,Candidate((60,60,40,40),2,.95,3,True,8))
        current=f.step(self.image(),10.1,[],detector_updated=False)
        self.assertTrue(current.observed)
        correction=f.step(self.image(),10.1,[Candidate((60,60,40,40),2,.05,3,True,9.9)],
            detector_updated=True,same_frame_correction=True)
        self.assertEqual(correction,current)
        self.assertEqual(f.last_detection_stamp,8)
        self.assertEqual(f.last_observation_stamp,10.1)
        self.assertTrue(f.diagnostics['kept_current_visual_on_unconfirmed_correction'])
        self.assertTrue(f.step(self.image(),10.2,[],detector_updated=False).observed)

    def test_option_disabled_preserves_existing_short_hold(self):
        f=BoxFusion(tracker=Local());f.initialize(self.image(),(60,60,40,40),10,20,Candidate((60,60,40,40),2,.95,3,True,8))
        self.assertFalse(f.step(self.image(),10.1,[],detector_updated=False).observed)

if __name__=='__main__':unittest.main()
