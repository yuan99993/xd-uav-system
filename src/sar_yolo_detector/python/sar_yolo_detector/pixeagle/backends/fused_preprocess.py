"""Pinned BGR8 upload + one antialiased RGB/resize/round/normalize CUDA kernel."""
import math
import threading
import numpy as np
import torch
import triton
import triton.language as tl
from triton.language.extra.cuda import libdevice

@triton.jit
def _resize_rgb(Input,Output,H:tl.constexpr,W:tl.constexpr,OH:tl.constexpr,OW:tl.constexpr,
                NX:tl.constexpr,NY:tl.constexpr,BLOCK:tl.constexpr):
    index=tl.program_id(0)*BLOCK+tl.arange(0,BLOCK)
    valid=index<3*OH*OW
    channel=index//(OH*OW);y=(index//OW)%OH;x=index%OW
    sx:tl.constexpr=W/OW;sy:tl.constexpr=H/OH
    supportx:tl.constexpr=max(sx,1.);supporty:tl.constexpr=max(sy,1.)
    cx=(x+.5)*sx;cy=(y+.5)*sy
    ix0=tl.floor(cx-supportx+.5).to(tl.int32)
    iy0=tl.floor(cy-supporty+.5).to(tl.int32)
    value=tl.full((BLOCK,),0,tl.float32);normal=tl.full((BLOCK,),0,tl.float32)
    for dy in range(NY):
        iy=iy0+dy
        wy=tl.maximum(0.,1.-tl.abs((iy+.5-cy)/supporty))
        for dx in range(NX):
            ix=ix0+dx
            wx=tl.maximum(0.,1.-tl.abs((ix+.5-cx)/supportx))
            inside=(ix>=0)&(ix<W)&(iy>=0)&(iy<H)
            weight=tl.where(inside,wx*wy,0.)
            pixel=tl.load(Input+(iy*W+ix)*3+(2-channel),valid&inside,other=0).to(tl.float32)
            value+=pixel*weight;normal+=weight
    result=libdevice.nearbyint(value/tl.maximum(normal,1.e-12))*(1./255.)
    tl.store(Output+index,result,valid)

class FusedPreprocessor:
    def __init__(self,device,output):
        self.device=device;self.output=output;self.shape=None;self.lock=threading.Lock()
        self.done=torch.cuda.Event();self.pending=False
    def __call__(self,frame):
        if frame.dtype!=np.uint8 or frame.ndim!=3 or frame.shape[2]!=3:raise ValueError('Expected BGR8 image')
        with self.lock, torch.cuda.device(self.device):
            # Host staging cannot be reused until the previous async DMA ended.
            if self.pending:self.done.synchronize()
            if self.shape!=frame.shape:
                self.host=torch.empty(frame.shape,dtype=torch.uint8,pin_memory=True)
                self.host_array=self.host.numpy()
                self.gpu=torch.empty(frame.shape,dtype=torch.uint8,device=self.device)
                self.shape=frame.shape
            np.copyto(self.host_array,frame)
            self.gpu.copy_(self.host,non_blocking=True)
            self.done.record();self.pending=True
            h,w=frame.shape[:2];oh,ow=self.output.shape[2:]
            _resize_rgb[(triton.cdiv(3*oh*ow,256),)](self.gpu,self.output,h,w,oh,ow,
                math.ceil(2*max(w/ow,1.))+1,math.ceil(2*max(h/oh,1.))+1,256,
                enable_fp_fusion=False)
            return self.output
