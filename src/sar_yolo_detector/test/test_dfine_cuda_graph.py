"""Actual V3 graph replay must preserve tensors and consume each new input."""
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
try:
    import torch
    import cv2
except ImportError:
    torch=None


@unittest.skipIf(torch is None,'Requires model runtime')
class GraphExecutionTest(unittest.TestCase):
    @unittest.skipUnless(torch is not None and torch.cuda.is_available(),'Requires CUDA')
    def test_official_postprocess_graph_consumes_new_logits_and_sizes_exactly(self):
        from sar_yolo_detector.pixeagle.backends.dfine_runtime import config,graph_postprocess
        post=config('dfine').postprocessor.cuda().eval()
        output=dict(pred_logits=torch.randn(1,300,3,device='cuda'),
                    pred_boxes=torch.rand(1,300,4,device='cuda'))
        model=SimpleNamespace(use_cuda_graph=True,_sar_graph=(None,None,None,output))
        with torch.inference_mode():
            for side in (640,256,384,512,640):
                output['pred_logits'].normal_()
                output['pred_boxes'].uniform_()
                sizes=torch.tensor([[side,side]],device='cuda')
                expected=post(output,sizes)[0]
                actual=graph_postprocess(model,post,output,sizes,side,side)
                for key in ('labels','boxes','scores'):
                    torch.testing.assert_close(actual[key],expected[key],rtol=0,atol=0)
        self.assertEqual(4,len(model._sar_postprocess_graphs))

    @unittest.skipUnless(torch is not None and torch.cuda.is_available(),'Requires CUDA')
    def test_preprocess_graph_is_exact_for_new_and_noncontiguous_frames(self):
        from sar_yolo_detector.pixeagle.backends.dfine_runtime import preprocess,graph_preprocess
        model=SimpleNamespace(use_cuda_graph=True)
        rng=np.random.default_rng(20261120)
        with torch.inference_mode():
            for side in (256,384,512,640,320):
                frame=rng.integers(0,256,(side,side,3),dtype=np.uint8)
                for current in (frame,frame[:,::-1],np.full_like(frame,83)):
                    expected=preprocess(current,640,'cuda')
                    actual=graph_preprocess(model,current,640,'cuda')
                    torch.testing.assert_close(actual,expected,rtol=0,atol=0)
        self.assertEqual(4,len(model._sar_preprocess_graphs))

    def test_cpu_keeps_normal_network_execution(self):
        from sar_yolo_detector.pixeagle.backends.dfine_runtime import graph_forward
        model=torch.nn.Linear(4,3).eval();model.use_cuda_graph=True
        image=torch.ones(2,4)
        torch.testing.assert_close(graph_forward(model,image),model(image))
        self.assertFalse(hasattr(model,'_sar_graph'))

    @unittest.skipUnless(torch is not None and torch.cuda.is_available(),'Requires CUDA')
    def test_actual_v3_new_inputs_match_full_fp32_network(self):
        from sar_yolo_detector.pixeagle.backends.dfine_runtime import load_bundle,preprocess,graph_forward
        package=Path(__file__).resolve().parents[1]
        model,post,_=load_bundle(package/'models/dfine_s_v3_16e/manifest.json','cuda')
        model.deploy()
        source=cv2.imread(str(package/'test/fixtures/dfine_v3_ue.jpg'))
        seen=[]
        with torch.inference_mode():
            for frame in (source,source[:,::-1].copy(),np.full_like(source,83)):
                image=preprocess(frame,640,'cuda')
                expected=model(image)
                model.use_cuda_graph=True
                actual=graph_forward(model,image)
                self.assertTrue(model.use_cuda_graph,getattr(model,'cuda_graph_error',''))
                self.assertIsNotNone(model._sar_graph)
                for key in ('pred_logits','pred_boxes'):
                    torch.testing.assert_close(actual[key],expected[key],rtol=1e-4,atol=1e-5)
                sizes=torch.tensor([[frame.shape[1],frame.shape[0]]],device='cuda')
                expected_rows=post(expected,sizes)[0];actual_rows=post(actual,sizes)[0]
                for key in ('labels','scores','boxes'):
                    torch.testing.assert_close(actual_rows[key],expected_rows[key],rtol=1e-4,atol=1e-4)
                seen.append(actual['pred_logits'].detach().cpu().clone())
        self.assertFalse(torch.equal(seen[0],seen[-1]),'Graph must consume new image contents')


if __name__=='__main__':unittest.main()
