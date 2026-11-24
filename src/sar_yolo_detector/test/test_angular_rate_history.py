import unittest
import cv2
import numpy as np
from sar_yolo_detector.camera_motion import AngularRateHistory, CameraMotion, transform_box

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

    def test_100hz_simulated_rate_removes_large_roll_search_error(self):
        width,height=640,384;fx=800.;cx,cy=width*.5,height*.5
        k=np.array([[fx,0,cx],[0,fx,cy],[0,0,1.]])
        amplitude=np.deg2rad(20.);frequency=1.
        history=AngularRateHistory(maximum_gap=.02)
        for index in range(1,25):
            stamp=index*.01
            rate=amplitude*2*np.pi*frequency*np.cos(2*np.pi*frequency*stamp)
            self.assertTrue(history.add(stamp,(0.,0.,rate),'fixed_camera_optical'))
        earlier,later=.20,.24
        correction=history.homography(earlier,later,k,'fixed_camera_optical')
        self.assertIsNotNone(correction)
        theta0=amplitude*np.sin(2*np.pi*frequency*earlier)
        theta1=amplitude*np.sin(2*np.pi*frequency*later)
        m0=np.vstack((cv2.getRotationMatrix2D((cx,cy),np.rad2deg(theta0),1.),[0,0,1.]))
        m1=np.vstack((cv2.getRotationMatrix2D((cx,cy),np.rad2deg(theta1),1.),[0,0,1.]))
        exact=m1@np.linalg.inv(m0)
        old=(80.,60.,50.,30.)
        corrected=transform_box(old,correction)
        expected=transform_box(old,exact)
        corrected_center=np.array([corrected[0]+corrected[2]/2,corrected[1]+corrected[3]/2])
        expected_center=np.array([expected[0]+expected[2]/2,expected[1]+expected[3]/2])
        self.assertLess(np.linalg.norm(corrected_center-expected_center),3.)
        uncorrected_center=np.array([old[0]+old[2]/2,old[1]+old[3]/2])
        self.assertGreater(np.linalg.norm(expected_center-uncorrected_center),3.)
