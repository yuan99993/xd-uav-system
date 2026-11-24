"""NanoTrack v2 Siamese inference using three fixed-shape CUDA engines."""
import hashlib
import json
import math
import time
from contextlib import nullcontext
from pathlib import Path
import cv2
import numpy as np
import torch
from .pixeagle.backends.tensorrt_engine import StaticTensorRTEngine
from .response_quality import response_quality,ResponseHysteresis

class NanoTensorRT:
    def _compute_context(self):
        stream=getattr(self,'compute_stream',None)
        return torch.cuda.stream(stream) if stream is not None else nullcontext()

    def __init__(self, directory, manifest_sha256, minimum_score=.35, minimum_apce=1.8, minimum_psr=2.2, minimum_flow_ratio=.35, occlusion_guard=False, recovery_apce=2.5, recovery_psr=2.8, scale_ema_alpha=.25):
        scale_ema_alpha=float(scale_ema_alpha)
        if not math.isfinite(scale_ema_alpha) or not 0<scale_ema_alpha<=1:
            raise ValueError("Invalid NanoTrack scale EMA alpha")
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
        # Nano response decode runs on a distinct non-blocking stream so it
        # cannot inherit D-FINE's queued work on the caller's default stream.
        self.compute_stream=torch.cuda.Stream(device=self.device)
        self.minimum_score=float(minimum_score)
        self.occlusion_guard_enabled=bool(occlusion_guard)
        self.minimum_apce=float(minimum_apce)
        self.minimum_psr=float(minimum_psr)
        self.minimum_flow_ratio=float(minimum_flow_ratio)
        self.scale_ema_alpha=scale_ema_alpha
        if (not all(math.isfinite(v) for v in (self.minimum_score,self.minimum_apce,self.minimum_psr,self.minimum_flow_ratio))
                or not 0<=self.minimum_score<=1 or not 0<=self.minimum_flow_ratio<=1
                or min(self.minimum_apce,self.minimum_psr)<0):
            raise ValueError("Invalid NanoTrack response/flow thresholds")
        self.response_guard=ResponseHysteresis(self.minimum_apce,self.minimum_psr,recovery_apce,recovery_psr)
        self.last_failure_reason="not_initialized"
        self.previous_frame=None
        self.template=None
        self.tracking_score=0.
        with torch.cuda.stream(self.compute_stream):
            coordinates=(torch.arange(16,device=self.device,dtype=torch.float32)-8)*16+127
            self.grid_y,self.grid_x=torch.meshgrid(coordinates,coordinates,indexing='ij')
            window=torch.hann_window(16,periodic=False,device=self.device)
            self.window=torch.sqrt(window[:,None]*window[None,:])

    def _crop(self, frame, side, output_size, center=None):
        side=max(2,int(side))
        crop_center=self.center if center is None else np.asarray(center,dtype=float)
        left=int(crop_center[0])-(side+1)//2
        top=int(crop_center[1])-(side+1)//2
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
        with self._compute_context(),torch.inference_mode():
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

    def update(self, frame, motion_hint=None, camera_motion_matrix=None):
        if self.template is None:
            return False,None
        camera_motion_shift=np.zeros(2,dtype=float)
        velocity=np.zeros(2,dtype=float)
        delta_time=0.
        original_center=self.center.copy()
        if motion_hint is not None:
            try:
                velocity,delta_time=motion_hint
                velocity=np.asarray(velocity,dtype=float).reshape(2)
                delta_time=float(delta_time)
                if not np.isfinite(velocity).all() or not math.isfinite(delta_time) or delta_time<=0:
                    velocity=np.zeros(2,dtype=float);delta_time=0.
            except (TypeError,ValueError,OverflowError):
                velocity=np.zeros(2,dtype=float);delta_time=0.
        if camera_motion_matrix is not None:
            from .camera_motion import CameraMotion,transform_window
            if CameraMotion.validate(camera_motion_matrix,frame.shape):
                old_box=tuple(np.r_[self.center-self.size/2,self.size])
                moved,_=transform_window(old_box,camera_motion_matrix)
                if moved is not None:
                    self.center=np.asarray([moved[0]+moved[2]/2,moved[1]+moved[3]/2])
                    self.size=np.asarray(moved[2:4],dtype=float)
                    camera_motion_shift=self.center-original_center
                    points=np.float32([original_center,original_center+velocity*.01])
                    mapped=cv2.perspectiveTransform(points.reshape(-1,1,2),camera_motion_matrix).reshape(-1,2)
                    velocity=(mapped[1]-mapped[0])/.01
                    anchor_box=tuple(np.r_[self.guard_anchor_center-self.guard_anchor_size/2,
                        self.guard_anchor_size])
                    moved_anchor,_=transform_window(anchor_box,camera_motion_matrix)
                    if moved_anchor is not None:
                        self.guard_anchor_center=np.asarray([moved_anchor[0]+moved_anchor[2]/2,
                            moved_anchor[1]+moved_anchor[3]/2])
                        self.guard_anchor_size=np.asarray(moved_anchor[2:4],dtype=float)
        timing={}
        context=float(self.size.sum())*.5
        side=math.sqrt((self.size[0]+context)*(self.size[1]+context))
        search_factor=self.response_guard.search_multiplier if self.occlusion_guard_enabled else 1.
        scale=127./(side*search_factor)
        crop_center=self.center.copy()
        motion_shift=np.zeros(2,dtype=float)
        if delta_time>0:
            motion_shift=velocity*min(delta_time,.2)
            # Keep the target comfortably inside the search crop if a stale
            # or noisy velocity estimate requests an extreme jump.
            max_shift=max(1.,side*search_factor*.35)
            magnitude=float(np.linalg.norm(motion_shift))
            if magnitude>max_shift:motion_shift*=max_shift/magnitude
            crop_center+=motion_shift
        with self._compute_context(),torch.inference_mode():
            started=time.perf_counter()
            search_input=self._crop(frame,side*255./127.*search_factor,255,center=crop_center)
            timing['search_crop_h2d_ms']=(time.perf_counter()-started)*1000.
            started=time.perf_counter()
            search=self.search_engine.run_tensors({'input':search_input})['output']
            timing['search_engine_host_ms']=(time.perf_counter()-started)*1000.
            started=time.perf_counter()
            output=self.head_engine.run_tensors({'input1':self.template,'input2':search})
            timing['head_engine_host_ms']=(time.perf_counter()-started)*1000.
            started=time.perf_counter()
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
            timing['decode_sync_ms']=(time.perf_counter()-started)*1000.
            values=packed[:6]
        quality_started=time.perf_counter()
        self.response_diagnostics=response_quality(packed[6:].reshape(16,16))
        guard_valid=(self.response_guard.update(self.response_diagnostics['apce'],self.response_diagnostics['psr']) if self.occlusion_guard_enabled else True)
        self.response_diagnostics.update(quality_ms=(time.perf_counter()-quality_started)*1000.,
            bad_frames=self.response_guard.bad_frames,occluded=self.response_guard.occluded,
            search_multiplier=search_factor,stage_timings_ms=timing)
        if not guard_valid:
            self.center=self.guard_anchor_center.copy();self.size=self.guard_anchor_size.copy()
            self.tracking_score=0.
            self.last_failure_reason='nano_response_occluded_searching'
            return False,None
        self.tracking_score=float(values[5])
        if not np.isfinite(values).all() or self.tracking_score<self.minimum_score:
            return False,None
        center=crop_center+(values[:2]-127)/scale
        rate=float(values[4])*.37
        raw_size=values[2:4]/scale*rate+self.size*(1-rate)
        size=self.size*(1.-self.scale_ema_alpha)+raw_size*self.scale_ema_alpha
        frame_height,frame_width=frame.shape[:2]
        center=np.clip(center,[0,0],[frame_width,frame_height])
        size=np.clip(size,[4,4],[frame_width,frame_height])
        self.last_failure_reason="ok"
        self.center,self.size=center,size
        self.response_diagnostics.update(motion_shift_px=motion_shift.tolist(),
            camera_motion_shift_px=camera_motion_shift.tolist(),
            scale_ema_alpha=self.scale_ema_alpha,raw_size_px=raw_size.tolist(),smoothed_size_px=size.tolist())
        if not self.occlusion_guard_enabled or self.response_guard.bad_frames==0:
            self.guard_anchor_center=center.copy();self.guard_anchor_size=size.copy()
        return True,tuple(np.r_[center-size/2,size])

    def getTrackingScore(self):
        return self.tracking_score
