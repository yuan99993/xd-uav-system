import unittest
import numpy as np
from sar_yolo_detector.box_fusion import BoxFusion, clip_visible_box
from test_box_fusion import LocalTracker, image

class EdgeFusionTest(unittest.TestCase):
    def test_visible_geometry_excludes_missing_area(self):
        tracker=LocalTracker()
        fusion=BoxFusion(dict(edge_aware_geometry=True),tracker=tracker)
        fusion.initialize(image(),(60,60,40,40),10.,3)
        self.assertEqual(clip_visible_box((-30,50,40,40),(200,260)),(0.,50.,10.,40.))
        self.assertTrue(fusion._compatible((0,50,10,40),(-30,50,40,40)))
        self.assertFalse(fusion._compatible((0,50,4,40),(-38,50,40,40)))

    def test_partial_prediction_keeps_identity_and_is_never_observed(self):
        tracker=LocalTracker()
        fusion=BoxFusion(dict(edge_aware_geometry=True,occlusion_freeze_enabled=True,
                             prediction_hold_sec=1.2),tracker=tracker)
        frame=image()
        fusion.initialize(frame,(220,60,40,40),10.,3)
        tracker.valid=False
        fusion.velocity=np.array([200.,0.])
        result=fusion.step(frame,10.1,[])
        self.assertEqual(result.source,'prediction')
        self.assertFalse(result.observed)
        self.assertEqual(result.public_id,3)
        self.assertAlmostEqual(result.box[2],20.)
        gone=fusion.step(frame,10.3,[])
        self.assertEqual(gone.source,'lost')
