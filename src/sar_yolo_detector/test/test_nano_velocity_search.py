import unittest
from types import SimpleNamespace

import numpy as np
import torch

from sar_yolo_detector.nano_tensorrt import NanoTensorRT


class NanoVelocitySearchTest(unittest.TestCase):
    class Engine:
        def __init__(self, output): self.output=output
        def run_tensors(self, inputs): return self.output

    def tracker(self):
        tracker=NanoTensorRT.__new__(NanoTensorRT)
        tracker.device=torch.device('cpu')
        tracker.template=torch.zeros((1,))
        tracker.center=np.array([100.,100.])
        tracker.size=np.array([20.,10.])
        tracker.guard_anchor_center=tracker.center.copy()
        tracker.guard_anchor_size=tracker.size.copy()
        tracker.minimum_score=.1
        tracker.occlusion_guard_enabled=False
        tracker.response_guard=SimpleNamespace(search_multiplier=1.,bad_frames=0,occluded=False)
        tracker.scale_ema_alpha=.25
        tracker.tracking_score=0.
        coordinates=(torch.arange(16,dtype=torch.float32)-8)*16+127
        tracker.grid_y,tracker.grid_x=torch.meshgrid(coordinates,coordinates,indexing='ij')
        tracker.window=torch.ones((16,16))
        score_logits=torch.zeros((1,2,16,16));score_logits[0,1,8,8]=10.
        distances=torch.full((1,4,16,16),20.)
        tracker.search_engine=self.Engine({'output':torch.zeros((1,3,255,255))})
        tracker.head_engine=self.Engine({'output1':score_logits,'output2':distances})
        tracker.crop_centers=[]
        def crop(frame,side,output_size,center=None):
            tracker.crop_centers.append(np.asarray(center).copy())
            return torch.zeros((1,3,output_size,output_size))
        tracker._crop=crop
        return tracker

    def test_velocity_hint_centers_search_crop_and_output(self):
        tracker=self.tracker()
        ok,box=tracker.update(np.zeros((300,300,3),np.uint8),motion_hint=(np.array([30.,0.]),.1))
        self.assertTrue(ok)
        np.testing.assert_allclose(tracker.crop_centers[-1],[103.,100.])
        self.assertAlmostEqual((box[0]+box[2]/2),103.,places=4)
        np.testing.assert_allclose(tracker.response_diagnostics['motion_shift_px'],[3.,0.],atol=1e-4)

    def test_size_ema_damps_single_frame_scale_change(self):
        tracker=self.tracker();previous=tracker.size.copy()
        ok,_=tracker.update(np.zeros((300,300,3),np.uint8))
        self.assertTrue(ok)
        raw=np.asarray(tracker.response_diagnostics['raw_size_px'])
        smoothed=np.asarray(tracker.response_diagnostics['smoothed_size_px'])
        np.testing.assert_allclose(smoothed,.75*previous+.25*raw)

    def test_camera_motion_and_target_velocity_both_center_the_search(self):
        tracker=self.tracker()
        transform=np.array([[1.,0.,5.],[0.,1.,-2.],[0.,0.,1.]])
        ok,_=tracker.update(np.zeros((300,300,3),np.uint8),
            motion_hint=(np.array([30.,0.]),.1),camera_motion_matrix=transform)
        self.assertTrue(ok)
        np.testing.assert_allclose(tracker.crop_centers[-1],[108.,98.],atol=1e-4)
        np.testing.assert_allclose(tracker.response_diagnostics['camera_motion_shift_px'],[5.,-2.])
        np.testing.assert_allclose(tracker.response_diagnostics['motion_shift_px'],[3.,0.],atol=1e-4)

    def test_invalid_scale_alpha_is_rejected_before_loading_engine(self):
        with self.assertRaises(ValueError):
            NanoTensorRT('/missing','',scale_ema_alpha=float('nan'))


if __name__=='__main__':
    unittest.main()
