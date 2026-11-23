import unittest
from sar_yolo_detector.box_fusion import Candidate,align_delayed_candidate,BoxFusion
class DelayAlignmentTest(unittest.TestCase):
    def test_stable_motion_expands_uncertainty_without_moving_pixels(self):
        c=Candidate((10.,20.,16.,16.),0,.8,7,detector_stamp=10.)
        a=align_delayed_candidate(c,10.05,(200.,0.),stable_motion=True)
        self.assertEqual(a.box,c.box);self.assertEqual(a.detector_stamp,c.detector_stamp)
        self.assertAlmostEqual(a.association_uncertainty_px,10.)
        f=BoxFusion();self.assertFalse(f._compatible((30,30,16,16),(75,30,16,16)))
        self.assertTrue(f._compatible((30,30,16,16),(75,30,16,16),16.))
    def test_unstable_future_expired_and_already_aligned_are_not_expanded(self):
        from dataclasses import replace
        c=Candidate((10.,20.,16.,16.),0,.8,7,detector_stamp=10.)
        self.assertIs(align_delayed_candidate(c,10.05,(200.,0.)),c)
        for stamp,velocity in [(9.9,(1,1)),(10.09,(1,1)),(10.01,(float('nan'),0))]:
            self.assertIs(align_delayed_candidate(c,stamp,velocity,stable_motion=True),c)
        c=replace(c,motion_aligned=True)
        self.assertIs(align_delayed_candidate(c,10.05,(200.,0.),stable_motion=True),c)
