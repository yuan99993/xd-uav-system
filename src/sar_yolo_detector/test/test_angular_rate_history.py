import unittest
import numpy as np
from sar_yolo_detector.camera_motion import AngularRateHistory, CameraMotion

class AngularRateHistoryTest(unittest.TestCase):
    def test_stationary_scene_rotates_against_camera_yaw(self):
        history = AngularRateHistory()
        for stamp in (1., 1.02, 1.04, 1.06):
            self.assertTrue(history.add(stamp, (0,.1,0), 'camera_optical'))
        k = np.array([[500.,0,320.],[0,500.,240.],[0,0,1.]])
        h = history.homography(1.01,1.05,k,'camera_optical')
        point = h @ [320.,240.,1.]
        self.assertAlmostEqual(point[0]/point[2],318.,places=3)
        self.assertIsNone(history.homography(1.01,1.05,k,'body'))
        self.assertIsNone(history.homography(1.01,1.2,k,'camera_optical'))

    def test_invalid_and_reordered_samples_rejected(self):
        history = AngularRateHistory()
        self.assertFalse(history.add(1.,(np.nan,0,0),'camera'))
        self.assertTrue(history.add(1.,(0,0,0),'camera'))
        self.assertFalse(history.add(.9,(0,0,0),'camera'))

    def test_rate_fallback_only_without_background_or_pose(self):
        frame=np.zeros((240,320,3),np.uint8)
        motion=CameraMotion()
        motion.update(frame,1.)
        rate=np.array([[1.,0,-2],[0,1.,0],[0,0,1.]])
        np.testing.assert_allclose(motion.update(frame,1.04,rate_rotation=rate),rate)
        self.assertEqual(motion.source,'optical_imu_rate')
