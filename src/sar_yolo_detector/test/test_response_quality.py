import unittest
import numpy as np
from sar_yolo_detector.response_quality import response_quality
class ResponseQualityTest(unittest.TestCase):
    def test_flat_and_nonfinite_responses_fail(self):
        for x in [np.ones((16,16)),np.full((16,16),np.nan)]:
            self.assertEqual(response_quality(x),dict(apce=0.,psr=0.))
    def test_unique_peak_beats_background_and_broad_plateau(self):
        x=np.full((16,16),.1);x[8,8]=1.
        sharp=response_quality(x);self.assertGreater(sharp['apce'],3);self.assertGreater(sharp['psr'],3)
        broad=np.ones((16,16));broad[0,0]=0.
        self.assertLess(response_quality(broad)['apce'],3.)

class HysteresisTest(unittest.TestCase):
    def test_three_bad_frames_trip_and_probe_can_recover(self):
        from sar_yolo_detector.response_quality import ResponseHysteresis
        h=ResponseHysteresis()
        self.assertTrue(h.update(1.,1.));self.assertTrue(h.update(1.,1.))
        self.assertFalse(h.update(1.,1.));self.assertEqual(h.search_multiplier,1.5)
        self.assertFalse(h.update(3.,2.7));self.assertTrue(h.update(2.6,2.9))
        self.assertEqual(h.search_multiplier,1.)
    def test_one_metric_or_nonconsecutive_low_does_not_trip(self):
        from sar_yolo_detector.response_quality import ResponseHysteresis
        h=ResponseHysteresis()
        for pair in [(1.,1.),(1.,1.),(1.,3.),(1.,1.),(1.,1.),(3.,1.)]:self.assertTrue(h.update(*pair))
