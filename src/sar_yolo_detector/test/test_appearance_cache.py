import unittest
from unittest.mock import patch
import numpy as np
import sar_yolo_detector.box_fusion as module
from test_box_fusion import LocalTracker,image
class AppearanceCacheTest(unittest.TestCase):
    def run_case(self,enabled):
        tracker=LocalTracker();f=module.BoxFusion(dict(appearance_cache_enabled=enabled),tracker=tracker)
        frame=image();f.initialize(frame,(60,60,40,40),10.,7,module.Candidate((60,60,40,40),0,.9,3,detector_stamp=10.))
        candidates=[module.Candidate((60,60,40,40),0,.9,3,detector_stamp=10.1) for _ in range(3)]
        real=module.appearance
        with patch.object(module,'appearance',wraps=real) as calls:r=f.step(frame,10.1,candidates);count=calls.call_count
        return r,count,f,frame
    def test_outputs_identical_and_repeated_feature_computations_removed(self):
        raw,raw_count,_,_=self.run_case(False);cached,count,f,_=self.run_case(True)
        self.assertEqual(raw,cached);self.assertLess(count,raw_count)
        self.assertIsNone(f._appearance_cache)
    def test_changed_next_capture_cannot_reuse_old_foreground(self):
        _,_,f,frame=self.run_case(True)
        frame[:]=0
        r=f.step(frame,10.2,[])
        self.assertFalse(r.observed)
        self.assertIsNone(f._appearance_cache)
