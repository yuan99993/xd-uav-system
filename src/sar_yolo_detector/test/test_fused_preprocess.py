import unittest
import numpy as np
class FusedPreprocessTest(unittest.TestCase):
    def test_antialias_rgb_rounding_and_reused_pinned_upload(self):
        try:import torch
        except ImportError:self.skipTest('Optional CUDA runtime unavailable')
        if not torch.cuda.is_available():self.skipTest('CUDA unavailable')
        from sar_yolo_detector.pixeagle.backends.fused_preprocess import FusedPreprocessor
        for shape,target in [((384,640,3),(512,512)),((320,320,3),(320,320))]:
            output=torch.empty((1,3,*target),device='cuda')
            pre=FusedPreprocessor(torch.device('cuda'),output)
            for seed in [13,17]:
                # Negative stride and read-only sources must also be safe.
                frame=np.random.RandomState(seed).randint(0,256,shape,np.uint8)[::-1]
                frame.setflags(write=False)
                p=torch.from_numpy(frame.copy()).cuda().permute(2,0,1).flip(0)[None].float()
                reference=torch.nn.functional.interpolate(p,size=target,mode='bilinear',align_corners=False,antialias=True).round_().div_(255.)
                actual=pre(frame);torch.cuda.synchronize()
                delta=(actual-reference).abs()
                self.assertLessEqual(float(delta.max()),1./255.+1e-6)
                self.assertLess(float((delta>1e-6).float().mean()),.01)
                self.assertTrue(pre.host.is_pinned())
                self.assertEqual(actual.data_ptr(),output.data_ptr())
