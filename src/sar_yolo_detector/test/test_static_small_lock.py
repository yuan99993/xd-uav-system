import unittest
from types import SimpleNamespace
import numpy as np
import cv2
from sar_yolo_detector.static_small_lock import StaticSmallLock
from sar_yolo_detector.box_fusion import BoxFusion,Candidate
from sar_yolo_detector.camera_motion import CameraMotion

class StaticSmallTests(unittest.TestCase):
    box=(70.,50.,14.,10.)
    def frame(self, present=True, shift=0):
        f=np.full((140,200,3),110,np.uint8)
        if present:
            cv2.line(f,(72+shift,52),(81+shift,57),(15,15,15),1)
            cv2.line(f,(72+shift,57),(81+shift,52),(15,15,15),1)
            f[53:57,75+shift:79+shift]=20
        return f
    def lock(self):
        s=StaticSmallLock({'enabled':True});s.initialize(self.frame(),self.box,10);return s
    def test_disconnected_object_uses_complete_silhouette_and_rejects_a_matching_fragment(self):
        box=(60.,40.,40.,32.);frame=np.full((120,180,3),110,np.uint8)
        for point in [(66,46),(94,46),(66,66),(94,66),(80,56)]:cv2.circle(frame,point,2,(20,20,20),-1)
        lock=StaticSmallLock({'enabled':True,'photometric_blur_sigma':.7,'multi_component_foreground':True,
            'sparse_foreground_costs':True,'contrast_normalized_search':True})
        lock.initialize(frame,box,10)
        ys,xs=np.nonzero(lock.mask)
        self.assertGreater(xs.max()-xs.min(),25)
        for n in range(1,6):result,status=lock.update(frame,box,10+n*.05,image_reference_transform=np.eye(3))
        self.assertIsNotNone(result,status)
        fragment=np.full_like(frame,110);cv2.circle(fragment,(80,56),2,(20,20,20),-1)
        self.assertIsNone(lock.update(fragment,box,10.3,image_reference_transform=np.eye(3))[0])

    def test_sparse_foreground_costs_preserve_dense_weighted_error(self):
        rng=np.random.RandomState(35)
        dense=StaticSmallLock();sparse=StaticSmallLock({'sparse_foreground_costs':True})
        for shape in [(10,14),(31,45)]:
            anchor=rng.randint(0,256,shape).astype(np.float32)
            search=rng.randint(0,256,(shape[0]+12,shape[1]+12)).astype(np.uint8)
            weights=rng.rand(*shape).astype(np.float32);weights[weights<.85]=0
            np.testing.assert_allclose(dense.costs(search,anchor,weights),sparse.costs(search,anchor,weights),rtol=2e-6,atol=2e-5)
            backgrounds=rng.uniform(80,140,(13,13)).astype(np.float32)
            np.testing.assert_allclose(dense.costs(search,anchor,weights,backgrounds,110),sparse.costs(search,anchor,weights,backgrounds,110),rtol=2e-6,atol=2e-5)

    def test_moving_image_template_scales_without_claiming_stationary_geometry(self):
        s=StaticSmallLock({"enabled":True,"photometric_blur_sigma":.7,"sparse_foreground_costs":True,"contrast_normalized_search":True});s.initialize(self.frame(),self.box,10);x,y,w,h=self.box
        for n in range(1,21):
            scale=1.+n*.01;dx=n*1.5;dy=n*.2
            matrix=np.array([[scale,0.,dx],[0.,scale,dy],[0.,0.,1.]])
            current=cv2.warpPerspective(self.frame(),matrix,(200,140),borderValue=(110,110,110))
            proposal=(scale*x+dx,scale*y+dy,w*scale,h*scale)
            box,status=s.update(current,proposal,10+n*.05,image_reference_transform=matrix)
            self.assertEqual(box is not None,n>=5,(n,status))
            self.assertFalse(status.get('stationary_geometry_supported',False))
            if box is not None:self.assertTrue(status['image_template_scaled'])
        self.assertIsNone(s.update(self.frame(False),proposal,11.05,image_reference_transform=matrix)[0])
        s=self.lock()
        self.assertIsNone(s.update(self.frame(),self.box,10.1,image_reference_transform=np.zeros((3,3)))[0])

    def test_moving_dense_pixels_can_recover_a_local_failure_within_capture_gap_budget(self):
        from unittest.mock import patch
        from sar_yolo_detector.manual_box_tracker import ManualBoxTracker
        old_threads=cv2.getNumThreads();cv2.setNumThreads(1)
        try:
            for present in [True,False]:
                local=ManualBoxTracker({'feature_tracker':{'tracker_algorithm':'kcf','require_opencv_tracker':True}})
                fusion=BoxFusion({'moving_small_pixel_witness':{'enabled':True,'photometric_blur_sigma':.7,'sparse_foreground_costs':True,'contrast_normalized_search':True}},tracker=local)
                fusion.initialize(self.frame(),self.box,10,99)
                for n in range(1,21):fusion.step(self.frame(shift=n//2),10+n*.05,[],detector_updated=False)
                with patch.object(local,'update',return_value=SimpleNamespace(valid=False,bbox=None,confidence=0.,reason='response_lost')):
                    result=fusion.step(self.frame(present,13),11.3,[],detector_updated=False)
                self.assertEqual(result.observed,present,fusion.diagnostics)
                self.assertTrue(fusion.diagnostics['local_tracker_executed'])
                self.assertEqual(result.last_detection_stamp,0.)
                if present:self.assertTrue(fusion.diagnostics['moving_small_pixel_reseeded_local'])
        finally:cv2.setNumThreads(old_threads)

    def test_moving_pixel_fusion_runs_real_kcf_without_texture_or_detector_clock_fabrication(self):
        from sar_yolo_detector.manual_box_tracker import ManualBoxTracker
        old_threads=cv2.getNumThreads();cv2.setNumThreads(1)
        try:
            local=ManualBoxTracker({'feature_tracker':{'tracker_algorithm':'kcf','require_opencv_tracker':True}})
            fusion=BoxFusion({'moving_small_pixel_witness':{'enabled':True,'photometric_blur_sigma':.7,'sparse_foreground_costs':True,'contrast_normalized_search':True}},tracker=local)
            fusion.initialize(self.frame(),self.box,10,99)
            self.assertFalse(fusion.result.observed)
            for n in range(1,41):
                shift=n//2
                result=fusion.step(self.frame(shift=shift),10+n*.05,[],detector_updated=False)
                self.assertEqual(result.observed,n>=5,(n,fusion.diagnostics))
                self.assertTrue(fusion.diagnostics['local_tracker_executed'])
                self.assertEqual(result.last_detection_stamp,0.)
                self.assertEqual(result.class_id,-1);self.assertEqual(result.public_id,99)
                if result.observed:self.assertLess(abs(result.box[0]-(self.box[0]+shift)),1.1)
            result=fusion.step(self.frame(False),12.05,[],detector_updated=False)
            self.assertFalse(result.observed)
            self.assertFalse(fusion.diagnostics['moving_small_pixel_witness']['valid'])
        finally:cv2.setNumThreads(old_threads)

    def test_five_fresh_captures_required_and_duplicates_cannot_confirm(self):
        s=self.lock()
        self.assertIsNone(s.update(self.frame(),self.box,10.03)[0])
        for _ in range(5):self.assertIsNone(s.update(self.frame(),self.box,10.03)[0])
        self.assertIsNone(s.update(self.frame(),self.box,10.06)[0])
        self.assertIsNone(s.update(self.frame(),self.box,10.1)[0])
        self.assertIsNone(s.update(self.frame(),self.box,10.13)[0])
        self.assertIsNotNone(s.update(self.frame(),self.box,10.17)[0])
    def test_current_pixels_are_required_after_disappearance(self):
        s=self.lock()
        for t in [10.03,10.06,10.1,10.13,10.17]:s.update(self.frame(),self.box,t)
        box,d=s.update(self.frame(False),self.box,10.2);self.assertIsNone(box);self.assertFalse(d['valid'])
        self.assertIsNone(s.update(self.frame(),self.box,10.23)[0])
    def test_flat_background_cannot_initialize(self):
        s=StaticSmallLock({'enabled':True});r=s.initialize(self.frame(False),self.box,10)
        self.assertEqual(r['reason'],'no_distinct_foreground');self.assertFalse(s.armed)
    def test_exposure_change_preserves_silhouette_but_not_empty_background(self):
        s=self.lock()
        for n in range(1,7):
            frame=np.clip(self.frame().astype(np.int16)+18,0,255).astype(np.uint8)
            box,status=s.update(frame,self.box,10+n*.04,True,np.eye(3))
        self.assertIsNotNone(box)
        self.assertGreater(status['absolute_foreground_mae'],8)
        self.assertLess(status['foreground_mae'],1)
        empty=np.clip(self.frame(False).astype(np.int16)+18,0,255).astype(np.uint8)
        self.assertIsNone(s.update(empty,self.box,10.3,True,np.eye(3))[0])

    def test_brief_bad_pixels_require_two_fresh_passes_and_expire_when_absent(self):
        s=self.lock()
        for n in range(1,7):s.update(self.frame(),self.box,10+n*.04,True,np.eye(3))
        self.assertIsNone(s.update(self.frame(False),self.box,10.28,True,np.eye(3))[0])
        self.assertIsNone(s.update(self.frame(),self.box,10.32,True,np.eye(3))[0])
        self.assertIsNotNone(s.update(self.frame(),self.box,10.36,True,np.eye(3))[0])
        for n in range(10,33):
            self.assertIsNone(s.update(self.frame(False),self.box,10+n*.04,True,np.eye(3))[0])
        self.assertFalse(s.armed)
        self.assertIsNone(s.update(self.frame(),self.box,11.4,True,np.eye(3))[0])

    def test_delayed_selection_is_pending_until_five_current_pose_validated_frames(self):
        s=self.lock()
        for n in range(5):
            box,status=s.update(self.frame(),self.box,11.5+n*.04,True,np.eye(3))
            self.assertEqual(box is not None,n==4)
        self.assertIsNone(s.update(self.frame(),self.box,12.3,True,np.eye(3))[0])
        self.assertFalse(s.armed)

    def test_unconfirmed_startup_stall_restarts_five_fresh_frames_but_locked_gap_cannot(self):
        s=self.lock()
        self.assertIsNone(s.update(self.frame(),self.box,10.1,True,np.eye(3),True)[0])
        for n in range(5):
            box,status=s.update(self.frame(),self.box,11.1+n*.1,True,np.eye(3),True)
            self.assertEqual(box is not None,n==4)
        self.assertIsNone(s.update(self.frame(),self.box,12.1,True,np.eye(3),True)[0])
        self.assertFalse(s.armed)
        s=self.lock()
        _,status=s.update(self.frame(),self.box,15.1,True,np.eye(3),True)
        self.assertEqual(status['reason'],'initial_confirmation_timeout')

    def test_large_padded_roi_keeps_independent_negative_support(self):
        box=(52.,37.,51.,35.)
        s=StaticSmallLock({'enabled':True});s.initialize(self.frame(),box,10)
        for n in range(1,7):result,status=s.update(self.frame(),box,10+n*.04,True,np.eye(3))
        self.assertIsNotNone(result)
        self.assertTrue(np.isfinite(status['negative_mae_margin']))
        self.assertIsNone(s.update(self.frame(False),box,10.3,True,np.eye(3))[0])

    def test_absolute_camera_reference_does_not_accumulate_background_translation(self):
        class Local:
            tracker_name='kcf'
            def reset(self):pass
            def initialize(self,frame,box):self.box=box
            def update(self,frame):return SimpleNamespace(valid=False,bbox=self.box,confidence=0.)
        f=BoxFusion({'civil_static_small_lock':{'enabled':True}},tracker=Local())
        f.initialize(self.frame(),self.box,10,20)
        background=np.array([[1.,0.,1.],[0.,1.,0.],[0.,0.,1.]])
        reference=dict(matrix=np.eye(3),translation_m=.05,maximum_translation_m=.25)
        for n in range(1,50):
            result=f.step(self.frame(),10+n*.04,[],detector_updated=False,
                camera_transform=background,camera_motion_reliable=True,static_reference=reference)
        self.assertTrue(result.observed)
        np.testing.assert_allclose(result.box,self.box,atol=1e-6)
        self.assertEqual(result.last_detection_stamp,0)
        self.assertFalse(f.step(self.frame(False),12,[],detector_updated=False,
            camera_transform=background,camera_motion_reliable=True,static_reference=reference).observed)
        reference['translation_m']=.3
        self.assertFalse(f.step(self.frame(),12.04,[],detector_updated=False,
            camera_transform=background,camera_motion_reliable=True,static_reference=reference).observed)
        self.assertEqual(f.diagnostics['civil_static_small_lock']['reason'],'observer_left_hover_scope')

    def test_accumulated_pan_validates_target_roi_instead_of_per_frame_full_image_limits(self):
        class Local:
            tracker_name='kcf'
            def reset(self):pass
            def initialize(self,frame,box):self.box=box
            def update(self,frame):return SimpleNamespace(valid=False,bbox=self.box,confidence=0.)
        original=np.full((1080,1920,3),110,np.uint8);box=(1282.,530.,24.,14.)
        cv2.line(original,(1285,532),(1302,541),(15,15,15),2)
        cv2.line(original,(1285,541),(1302,532),(15,15,15),2)
        f=BoxFusion({'civil_static_small_lock':{'enabled':True}},tracker=Local())
        f.initialize(original,box,10,20)
        k=np.array([[960.,0.,960.],[0.,960.,540.],[0.,0.,1.]])
        prior=np.eye(3)
        for n in range(1,29):
            a=-max(0,n-6)*np.pi/180
            r=np.array([[np.cos(a),0.,np.sin(a)],[0.,1.,0.],[-np.sin(a),0.,np.cos(a)]])
            absolute=k@r@np.linalg.inv(k);absolute/=absolute[2,2]
            current=cv2.warpPerspective(original,absolute,(1920,1080))
            result=f.step(current,10+n*.04,[],detector_updated=False,
                camera_transform=absolute@np.linalg.inv(prior),camera_motion_reliable=True,
                static_reference=dict(matrix=absolute,translation_m=.02,maximum_translation_m=.25))
            if n>=5:self.assertTrue(result.observed,(n,f.diagnostics['civil_static_small_lock']))
            prior=absolute
        self.assertFalse(CameraMotion.validate(absolute,original.shape))

    def test_camera_reprojection_uses_original_roi_without_accumulating_local_jitter(self):
        s=self.lock();translation=np.array([[1.,0.,1.],[0.,1.,0.],[0.,0.,1.]])
        for n in range(1,8):
            _,status=s.update(self.frame(shift=n),self.box,10+n*.04,True,translation)
            np.testing.assert_allclose(status['projected_reference_box'],(70+n,50,14,10),atol=1e-6)
        self.assertTrue(status['valid'])
    def test_no_recovery_after_large_frame_gap(self):
        s=self.lock();self.assertIsNone(s.update(self.frame(),self.box,11)[0]);self.assertFalse(s.armed)
    def test_ambiguous_duplicate_in_local_search_is_rejected(self):
        s=self.lock();f=self.frame();f=np.tile(f[:,70:77],(1,29,1))[:,:200]
        self.assertIsNone(s.update(f,self.box,10.1)[0])
    def test_negative_class_cannot_activate_civilian_extension(self):
        class Local:
            tracker_name='kcf'
            def reset(self):pass
            def initialize(self,*args):pass
        f=BoxFusion({'civil_static_small_lock':{'enabled':True}},tracker=Local())
        f.initialize(self.frame(),self.box,10,20,Candidate(self.box,1,.9,3,True,10))
        self.assertFalse(f.small_lock.applicable)

    def test_real_drone_detector_association_cannot_bypass_initial_pixel_confirmation(self):
        class Local:
            tracker_name='kcf'
            def reset(self):pass
            def initialize(self,*args):pass
        f=BoxFusion({'civil_static_small_lock':{'enabled':True}},tracker=Local())
        result=f.initialize(self.frame(),self.box,10,20,Candidate(self.box,2,.95,3,True,10))
        self.assertTrue(f.small_lock.applicable)
        self.assertFalse(result.observed)
        self.assertEqual(result.class_id,2)
        self.assertEqual(result.last_detection_stamp,10)
    def test_pixel_rescue_requires_calibrated_motion_and_reseeds_real_tracker(self):
        class Local:
            tracker_name='kcf'
            def __init__(self):self.seeds=0
            def reset(self):pass
            def initialize(self,frame,box):self.box=box;self.seeds+=1
            def update(self,frame):return SimpleNamespace(valid=False,bbox=self.box,confidence=0.)
        for pose in [False,True]:
            local=Local();f=BoxFusion({'civil_static_small_lock':{'enabled':True}},tracker=local)
            f.initialize(self.frame(),self.box,10,20)
            for n in range(1,12):r=f.step(self.frame(),10+n/30,[],detector_updated=False,
                camera_transform=np.eye(3) if pose else None,camera_motion_reliable=pose)
            self.assertEqual(r.observed,pose)
            if pose:self.assertGreater(local.seeds,1)
            self.assertEqual(r.last_detection_stamp,0)

    def test_fusion_preserves_identity_and_detector_clock(self):
        class Local:
            tracker_name='kcf'
            def reset(self):pass
            def initialize(self,frame,box):self.box=box
            def update(self,frame):return SimpleNamespace(valid=True,bbox=self.box,confidence=.95)
        f=BoxFusion({'continuous_selected_flow':True,'civil_static_small_lock':{'enabled':True}},tracker=Local())
        f.initialize(self.frame(),self.box,10,20)
        for n in range(1,101):r=f.step(self.frame(),10+n/30,[],detector_updated=False)
        self.assertTrue(r.observed);self.assertEqual(r.public_id,20);self.assertEqual(r.last_detection_stamp,0);self.assertEqual(r.class_id,-1)
        self.assertFalse(f.step(self.frame(False),13.4,[],detector_updated=False).observed)

if __name__=='__main__':unittest.main()
