import unittest
from types import SimpleNamespace as NS
import numpy as np
from sar_yolo_detector.camera_calibration import CalibrationCache
from sar_yolo_detector.camera_motion import rotation_homography


def header(seq):return NS(frame_id='camera',stamp=NS(secs=100,nsecs=seq))
def info(seq,fx=480):return NS(header=header(seq),width=960,height=540,
    K=[fx,0,480,0,fx,270,0,0,1],D=[0]*5)


class CalibrationTest(unittest.TestCase):
    def test_delayed_frame_uses_own_intrinsics_and_missing_does_not_use_latest(self):
        cache=CalibrationCache();cache.add(info(1));cache.add(info(2,830))
        self.assertEqual(cache.get(header(1),960,540).K[0],480)
        self.assertEqual(cache.get(header(2),960,540).K[0],830)
        self.assertIsNone(cache.get(header(3),960,540))
        self.assertIsNone(cache.get(header(2),640,360))
    def test_static_fallback_stops_once_dynamic_stream_is_seen(self):
        cache=CalibrationCache();message=info(0);message.header.stamp.secs=0;cache.add(message)
        self.assertEqual(cache.get(header(1),960,540).K[0],480)
        cache.add(info(2,830));self.assertIsNone(cache.get(header(3),960,540))
    def test_invalid_conflicting_or_distorted_calibration_is_not_reused(self):
        cache=CalibrationCache();cache.add(info(1));cache.add(info(1,830))
        self.assertIsNone(cache.get(header(1),960,540))
        for seq in [2,3,4]:
            message=info(seq)
            if seq==2:message.K[0]=float('nan')
            elif seq==3:message.K[4]=0
            else:message.D[0]=.1
            self.assertFalse(cache.add(message));self.assertIsNone(cache.get(header(seq),960,540))
    def test_cache_is_immutable_and_bounded(self):
        cache=CalibrationCache(4);message=info(1);cache.add(message);message.K[0]=1
        self.assertEqual(cache.get(header(1),960,540).K[0],480)
        for seq in range(2,20):cache.add(info(seq))
        self.assertEqual(len(cache.values),4);self.assertIsNone(cache.get(header(1),960,540))
    def test_zoom_projection_uses_new_K_and_old_inverse_K(self):
        old=np.array(info(1).K).reshape(3,3);new=old.copy();new[0,0]*=1.7;new[1,1]*=1.7
        h=rotation_homography(old,np.eye(3),np.eye(3),current_intrinsics=new)
        ray=np.array([.2,.1,1.]);np.testing.assert_allclose(h@old@ray,new@ray,atol=1e-8)
        np.testing.assert_allclose(h@np.array([480,270,1]),[480,270,1],atol=1e-8)

if __name__=='__main__':unittest.main()
