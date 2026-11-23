import unittest
from unittest.mock import patch
import cv2,numpy as np
from sar_yolo_detector.camera_motion import CameraMotion
from sar_yolo_detector.selected_flow import selected_flow
from sar_yolo_detector.box_fusion import appearance
class CpuBudgetTest(unittest.TestCase):
    def test_small_flow_budget_measures_current_target_motion(self):
        rng=np.random.RandomState(19);before=rng.randint(0,256,(240,320,3),np.uint8)
        after=cv2.warpAffine(before,np.float32([[1,0,3],[0,1,2]]),(320,240))
        box,status=selected_flow(before,after,(100.,80.,60.,45.),local_crop=True,window_size=9,pyramid_levels=1)
        self.assertTrue(status['valid']);np.testing.assert_allclose(box,(103.,82.,60.,45.),atol=.3)
    def test_background_budget_preserves_native_pixel_transform(self):
        rng=np.random.RandomState(42);before=rng.randint(0,256,(384,640,3),np.uint8)
        after=cv2.warpAffine(before,np.float32([[1,0,4],[0,1,2]]),(640,384))
        camera=CameraMotion({'processing_width':240,'lk_window_size':15,'lk_pyramid_levels':2,'max_features':80})
        camera.update(before,1.);matrix=camera.update(after,1.04)
        self.assertIsNotNone(matrix);np.testing.assert_allclose(matrix[:2,2],[4,2],atol=.4)
    def test_saturated_structure_skips_zero_weight_gradient_descriptor(self):
        frame=np.zeros((100,100,3),np.uint8);frame[:,:,1]=90
        frame[20:60,20:60]=(0,0,255);frame[20:40,20:40]=(0,0,50)
        plain,_=appearance(frame,(20.,20.,40.,40.),False)
        with patch('numpy.arctan2',side_effect=AssertionError('zero weight descriptor')):
            structural,quality=appearance(frame,(20.,20.,40.,40.),True)
        np.testing.assert_allclose(plain,structural,atol=1e-12);self.assertGreater(quality,.9)
    def test_invalid_camera_budget_rejected(self):
        for cfg in [{'processing_width':10},{'lk_window_size':8},{'lk_pyramid_levels':5}]:
            with self.assertRaises(ValueError):CameraMotion(cfg)
