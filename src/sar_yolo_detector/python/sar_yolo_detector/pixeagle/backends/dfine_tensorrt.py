"""Explicit, hash-verified fixed-shape D-FINE TensorRT execution."""
import threading
import numpy as np
import torch
from .tensorrt_engine import StaticTensorRTEngine

class DFineTensorRT(StaticTensorRTEngine):
    def __init__(self, path, expected_sha256, device, fused_preprocess=False):
        super().__init__(path,expected_sha256,device)
        if set(self.inputs) != {'images'} or set(self.outputs) != {'logits','boxes'}:
            raise ValueError("Unexpected D-FINE TensorRT tensor contract")
        self._infer_lock=threading.Lock()
        self.fused_preprocessor=None
        self._fused_pending=False
        self.shape = tuple(self.inputs['images'].shape)
        if len(self.shape) != 4 or self.shape[:2] != (1,3):
            raise ValueError("Expected a fixed batch-one RGB D-FINE engine")
        for name,last in (('logits',3),('boxes',4)):
            if tuple(self.outputs[name].shape) != (1,300,last):
                raise ValueError("Unexpected D-FINE output shape")

        if fused_preprocess:
            from .fused_preprocess import FusedPreprocessor
            self.fused_preprocessor=FusedPreprocessor(self.device,self.inputs['images'])

    def run(self, pixels):
        output = self.run_tensors({'images':pixels})
        return dict(pred_logits=output['logits'],pred_boxes=output['boxes'])

    def infer(self, frame, post):
        with self._infer_lock, torch.inference_mode():
            h,w = frame.shape[:2]
            if self.fused_preprocessor is not None:
                # Direct writes into TRT input must wait for its prior consumer.
                if self._fused_pending:torch.cuda.current_stream(self.device).wait_event(self.completion)
                pixels=self.fused_preprocessor(frame)
                self._fused_pending=True
            else:
                pixels = torch.from_numpy(np.require(frame,requirements=['C','W'])).to(self.device)
                pixels = pixels.permute(2,0,1).flip(0).unsqueeze(0).to(torch.float32)
                pixels = torch.nn.functional.interpolate(pixels,size=self.shape[2:],mode='bilinear',
                    align_corners=False,antialias=True).round_().div_(255.).contiguous()
            output = self.run(pixels)
            return post(output,torch.tensor([[w,h]],device=self.device))[0]
