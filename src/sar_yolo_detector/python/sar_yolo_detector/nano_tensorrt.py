"""NanoTrack v2 Siamese inference using three fixed-shape CUDA engines."""
import hashlib
import json
import math
import time
from pathlib import Path
import cv2
import numpy as np
import torch
from .pixeagle.backends.tensorrt_engine import StaticTensorRTEngine
from .response_quality import response_quality,ResponseHysteresis

class NanoTensorRT:
    def __init__(self, directory, manifest_sha256, minimum_score=.35, minimum_apce=1.8, minimum_psr=2.2, minimum_flow_ratio=.35, occlusion_guard=False, recovery_apce=2.5, recovery_psr=2.8):
        directory=Path(directory)
        manifest_path=directory/'manifest.json'
        raw=manifest_path.read_bytes()
        if not manifest_sha256 or hashlib.sha256(raw).hexdigest()!=manifest_sha256:
            raise ValueError("NanoTrack engine manifest integrity mismatch")
        manifest=json.loads(raw)
        if set(manifest['files'])!={'template.engine','search.engine','head.engine'}:
            raise ValueError("Incomplete NanoTrack engine bundle")
        self.template_engine=StaticTensorRTEngine(directory/'template.engine',manifest['files']['template.engine'])
        self.search_engine=StaticTensorRTEngine(directory/'search.engine',manifest['files']['search.engine'])
        self.head_engine=StaticTensorRTEngine(directory/'head.engine',manifest['files']['head.engine'])
        self.device=self.search_engine.device
        self.minimum_score=float(minimum_score)
        self.occlusion_guard_enabled=bool(occlusion_guard)
        self.minimum_apce=float(minimum_apce)
        self.minimum_psr=float(minimum_psr)
        self.minimum_flow_ratio=float(minimum_flow_ratio)
        if (not all(math.isfinite(v) for v in (self.minimum_score,self.minimum_apce,self.minimum_psr,self.minimum_flow_ratio))
                or not 0<=self.minimum_score<=1 or not 0<=self.minimum_flow_ratio<=1
                or min(self.minimum_apce,self.minimum_psr)<0):
            raise ValueError("Invalid NanoTrack response/flow thresholds")
        self.response_guard=ResponseHysteresis(self.minimum_apce,self.minimum_psr,recovery_apce,recovery_psr)
        self.last_failure_reason="not_initialized"
        self.previous_frame=None
        self.template=None
        self.tracking_score=0.
        coordinates=(torch.arange(16,device=self.device,dtype=torch.float32)-8)*16+127
        self.grid_y,self.grid_x=torch.meshgrid(coordinates,coordinates,indexing='ij')
        window=torch.hann_window(16,periodic=False,device=self.device)
        self.window=torch.sqrt(window[:,None]*window[None,:])

    def _crop(self, frame, side, output_size):
        side=max(2,int(side))
        left=int(self.center[0])-(side+1)//2
        top=int(self.center[1])-(side+1)//2
        right,bottom=left+side,top+side
        height,width=frame.shape[:2]
        if right<=0 or bottom<=0 or left>=width or top>=height:
            raise ValueError("NanoTrack search window is outside the image")
        patch=frame[max(0,top):min(height,bottom),max(0,left):min(width,right)]
        patch=cv2.copyMakeBorder(patch,max(0,-top),max(0,bottom-height),
            max(0,-left),max(0,right-width),cv2.BORDER_CONSTANT,value=cv2.mean(frame)[:3])
        patch=cv2.resize(patch,(output_size,output_size))
        pixels=np.ascontiguousarray(patch[:,:,::-1].transpose(2,0,1)[None])
        return torch.from_numpy(pixels).to(device=self.device,dtype=torch.float32)

    @staticmethod
    def _size(width,height):
        padding=(width+height)/2
        return ((width+padding)*(height+padding))**.5

    def init(self, frame, bbox):
        x,y,width,height=map(float,bbox)
        if min(width,height)<=0:
            return False
        self.center=np.array([x+width/2,y+height/2])
        self.size=np.array([width,height])
        context=(width+height)*.5
        side=math.sqrt((width+context)*(height+context))
        with torch.inference_mode():
            self.template=self.template_engine.run_tensors({'input':self._crop(frame,side,127)})['output']
        self.response_guard.reset()
        self.guard_anchor_center=self.center.copy();self.guard_anchor_size=self.size.copy()
        return True

    def reposition_search(self, bbox, matrix):
        """Carry camera motion into search and last-good anchor, not template."""
        from .camera_motion import transform_window
        x,y,width,height=map(float,bbox)
        anchor_box=tuple(np.r_[self.guard_anchor_center-self.guard_anchor_size/2,self.guard_anchor_size])
        anchor,_=transform_window(anchor_box,matrix)
        if anchor is None or min(width,height)<=0:
            return False
        ax,ay,aw,ah=anchor
        self.center=np.array([x+width/2,y+height/2]);self.size=np.array([width,height])
        self.guard_anchor_center=np.array([ax+aw/2,ay+ah/2]);self.guard_anchor_size=np.array([aw,ah])
        return True

    def update(self, frame):
        if self.template is None:
            return False,None
        context=float(self.size.sum())*.5
        side=math.sqrt((self.size[0]+context)*(self.size[1]+context))
        search_factor=self.response_guard.search_multiplier if self.occlusion_guard_enabled else 1.
        scale=127./(side*search_factor)
        with torch.inference_mode():
            search=self.search_engine.run_tensors({'input':self._crop(frame,side*255./127.*search_factor,255)})['output']
            output=self.head_engine.run_tensors({'input1':self.template,'input2':search})
            score=output['output1'][0].float().softmax(dim=0)[1]
            distances=output['output2'][0].float()
            x1,y1=self.grid_x-distances[0],self.grid_y-distances[1]
            x2,y2=self.grid_x+distances[2],self.grid_y+distances[3]
            width,height=(x2-x1).clamp_min(1e-6),(y2-y1).clamp_min(1e-6)
            size_ratio=self._size(width,height)/self._size(*(self.size*scale))
            size_ratio=torch.maximum(size_ratio,1./size_ratio)
            aspect_ratio=float(self.size[0]/self.size[1])/(width/height)
            aspect_ratio=torch.maximum(aspect_ratio,1./aspect_ratio)
            penalty=torch.exp(-(size_ratio*aspect_ratio-1)*.055)
            ranked=penalty*score*(1.-.455)+self.window*.455
            index=ranked.flatten().argmax()
            vector=torch.stack([(x1+x2).flatten()[index]/2,(y1+y2).flatten()[index]/2,
                width.flatten()[index],height.flatten()[index],
                (penalty*score).flatten()[index],ranked.flatten()[index]])
            packed=torch.cat((vector,score.flatten()))
            packed=packed.cpu().numpy()
            values=packed[:6]
        quality_started=time.perf_counter()
        self.response_diagnostics=response_quality(packed[6:].reshape(16,16))
        guard_valid=(self.response_guard.update(self.response_diagnostics['apce'],self.response_diagnostics['psr']) if self.occlusion_guard_enabled else True)
        self.response_diagnostics.update(quality_ms=(time.perf_counter()-quality_started)*1000.,
            bad_frames=self.response_guard.bad_frames,occluded=self.response_guard.occluded,
            search_multiplier=search_factor)
        if not guard_valid:
            self.center=self.guard_anchor_center.copy();self.size=self.guard_anchor_size.copy()
            self.tracking_score=0.
            self.last_failure_reason='nano_response_occluded_searching'
            return False,None
        self.tracking_score=float(values[5])
        if not np.isfinite(values).all() or self.tracking_score<self.minimum_score:
            return False,None
        center=self.center+(values[:2]-127)/scale
        rate=float(values[4])*.37
        size=values[2:4]/scale*rate+self.size*(1-rate)
        frame_height,frame_width=frame.shape[:2]
        center=np.clip(center,[0,0],[frame_width,frame_height])
        size=np.clip(size,[4,4],[frame_width,frame_height])
        self.last_failure_reason="ok"
        self.center,self.size=center,size
        if not self.occlusion_guard_enabled or self.response_guard.bad_frames==0:
            self.guard_anchor_center=center.copy();self.guard_anchor_size=size.copy()
        return True,tuple(np.r_[center-size/2,size])

    def getTrackingScore(self):
        return self.tracking_score
