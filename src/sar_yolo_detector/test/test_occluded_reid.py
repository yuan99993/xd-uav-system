import unittest
import numpy as np
from sar_yolo_detector.box_fusion import BoxFusion,Candidate
from test_box_fusion import LocalTracker,image
class OccludedReidTest(unittest.TestCase):
    def make(self):
        tracker=LocalTracker();f=BoxFusion(dict(occlusion_freeze_enabled=True,occluded_reid_enabled=True,prediction_hold_sec=1.2,class_groups=[[0,1],[2]]),tracker=tracker)
        frame=image();f.initialize(frame,(60,60,40,40),10.,77,Candidate((60,60,40,40),0,.9,3,detector_stamp=10.));tracker.valid=False
        f.velocity=np.array([10.,0.])
        for snapshot in f._clean_history:snapshot['velocity']=f.velocity.copy()
        return f,frame
    def test_wait_has_no_box_and_recovery_preserves_public_id(self):
        f,frame=self.make()
        coast=f.step(frame,10.2,[]);self.assertEqual(coast.source,'prediction')
        waiting=f.step(frame,12.,[]);self.assertIsNone(waiting.box);self.assertFalse(waiting.observed)
        result=f.step(frame,13.,[Candidate((60,60,40,40),0,.9,9,detector_stamp=13.)])
        self.assertFalse(result.observed)
        result=f.step(image((61,60,40,40)),13.1,[Candidate((61,60,40,40),0,.9,9,detector_stamp=13.1)])
        self.assertTrue(result.observed);self.assertEqual(result.public_id,77);self.assertEqual(result.local_id,9)
        self.assertEqual(result.reason,'occluded_reid_same_public_id')
    def test_wrong_class_stale_and_expired_do_not_wake(self):
        for label,measured,stamp in [(2,13.,13.),(0,10.,13.),(0,16.,16.)]:
            f,frame=self.make();r=f.step(frame,stamp,[Candidate((60,60,40,40),label,.9,9,detector_stamp=measured)])
            self.assertFalse(r.observed);self.assertIsNone(r.box)

class LagSnapshotTest(unittest.TestCase):
    def test_five_capture_cache_freezes_pre_occlusion_appearance_and_velocity(self):
        f,frame=OccludedReidTest().make()
        good=f.anchor.copy()
        for index in range(1,6):
            f.velocity=np.array([10.,0.]) if index<=2 else np.array([999.,0.])
            current=frame.copy() if index<=2 else np.zeros_like(frame)
            f._cache_clean_snapshot(current,(60,60,40,40),10.+index*.04)
        self.assertEqual(len(f._clean_history),5)
        f._freeze_occlusion(10.24)
        self.assertAlmostEqual(f._frozen_snapshot['stamp'],10.08)
        np.testing.assert_allclose(f.anchor,good)
        np.testing.assert_allclose(f.velocity,[10.,0.])
