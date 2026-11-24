"""Explicit, hash-verified fixed-shape D-FINE TensorRT execution."""
import threading
import time
import torch
from .tensorrt_engine import StaticTensorRTEngine

class DFineTensorRT(StaticTensorRTEngine):
    def __init__(self, path, expected_sha256, device, fused_preprocess=False, resize_mode='stretch'):
        super().__init__(path,expected_sha256,device)
        if resize_mode not in ('stretch','letterbox'):raise ValueError('resize_mode must be stretch or letterbox')
        self.resize_mode=resize_mode
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
            self.fused_preprocessor=FusedPreprocessor(self.device,self.inputs['images'],resize_mode)

    def run(self, pixels):
        output = self.run_tensors({'images':pixels})
        return dict(pred_logits=output['logits'],pred_boxes=output['boxes'])

    @staticmethod
    def _focal_candidates(output, canvas_width, canvas_height, confidence_threshold, max_det):
        """Filter/rank class-query pairs on GPU before the small result reaches host."""
        logits, raw_boxes = output['pred_logits'], output['pred_boxes']
        if logits.ndim != 3 or raw_boxes.ndim != 3 or logits.shape[:2] != raw_boxes.shape[:2]:
            raise ValueError('Unexpected D-FINE output tensor shapes')
        classes=int(logits.shape[-1])
        if classes<=0:raise ValueError('D-FINE output has no classes')
        scores=logits.float().sigmoid().flatten(1)
        # Keep a small reserve for class duplicates while honoring the backend
        # output budget. The default selected-target path transfers at most 20
        # candidates instead of all 300 queries x 3 classes.
        candidate_count=min(scores.shape[1],max(16,int(max_det)))
        if candidate_count==0:
            return [dict(boxes=raw_boxes.new_empty((0,4)),scores=scores.new_empty((0,)),
                labels=torch.empty((0,),device=logits.device,dtype=torch.long))]
        scores=scores.masked_fill((~torch.isfinite(scores))|(scores<float(confidence_threshold)),-float('inf'))
        top_scores,indices=torch.topk(scores,candidate_count,dim=1,sorted=True)
        labels=torch.remainder(indices,classes)
        query_indices=torch.div(indices,classes,rounding_mode='floor')
        selected=raw_boxes.gather(1,query_indices.unsqueeze(-1).expand(-1,-1,4)).float()
        cx,cy,width,height=selected.unbind(dim=-1)
        boxes=torch.stack((cx-width*.5,cy-height*.5,cx+width*.5,cy+height*.5),dim=-1)
        scale=boxes.new_tensor([canvas_width,canvas_height,canvas_width,canvas_height])
        boxes=boxes*scale
        return [dict(boxes=boxes[0],scores=top_scores[0],labels=labels[0])]

    def infer(self, frame, post, resize_mode=None, confidence_threshold=.3, max_det=20):
        with self._infer_lock, torch.inference_mode():
            h,w = frame.shape[:2]
            mode=resize_mode or self.resize_mode
            if mode!=self.resize_mode:raise ValueError('TensorRT preprocessing mode does not match model profile')
            stages={};started=time.perf_counter()
            if self.fused_preprocessor is not None:
                # Direct writes into TRT input must wait for its prior consumer.
                if self._fused_pending:torch.cuda.current_stream(self.device).wait_event(self.completion)
                pixels=self.fused_preprocessor(frame)
                self._fused_pending=True
            else:
                from .dfine_runtime import preprocess
                pixels=preprocess(frame,self.shape[2:],self.device,mode)
            stages['preprocess_host_ms']=(time.perf_counter()-started)*1000.
            started=time.perf_counter()
            output = self.run(pixels)
            stages['engine_host_ms']=(time.perf_counter()-started)*1000.
            started=time.perf_counter()
            canvas_h,canvas_w=self.shape[2:]
            if bool(getattr(post,'use_focal_loss',False)):
                target_width,target_height=(canvas_w,canvas_h) if mode=='letterbox' else (w,h)
                result=self._focal_candidates(output,target_width,target_height,confidence_threshold,max_det)[0]
            else:
                target_sizes=(torch.tensor([[canvas_w,canvas_h]],device=self.device) if mode=='letterbox'
                    else torch.tensor([[w,h]],device=self.device))
                result=post(output,target_sizes)[0]
            if mode=='letterbox':
                from .dfine_runtime import _restore_letterbox_result
                result=_restore_letterbox_result(result,w,h,canvas_h,canvas_w)
            stages['postprocess_host_ms']=(time.perf_counter()-started)*1000.
            self.last_stage_timings=stages
            self.last_postprocess_diagnostics=dict(candidate_count=int(result['scores'].shape[0]),
                prefilter_topk=min(int(output['pred_logits'].shape[1]*output['pred_logits'].shape[2]),
                    max(16,int(max_det))) if bool(getattr(post,'use_focal_loss',False)) else None,
                confidence_threshold=float(confidence_threshold))
            return result
