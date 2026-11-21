"""Offline-only OSTrack comparison using an explicit upstream source/checkpoint.

Postprocessing adapted from OSTrack, Copyright (c) 2022 Botao Ye (MIT).
See LICENSES/OSTrack-MIT.txt.

No imports or model loads occur in the default four-mode pipeline. The network,
config, crop sampler, CE mask, Hann window and box mapping follow upstream
OSTrack-256. Extra research dependencies belong to the comparison environment.
"""
import importlib.util
import math
from pathlib import Path
import sys


def _file_module(name,path):
    spec=importlib.util.spec_from_file_location(name,str(path))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


class OSTrackCandidate:
    tracker_name='ostrack_256'
    def __init__(self,source,checkpoint):
        import torch
        from .manual_box_tracker import ManualTrackResult
        self.torch,self.result_type=torch,ManualTrackResult
        source=Path(source).resolve();checkpoint=Path(checkpoint).resolve()
        if not source.is_dir() or not checkpoint.is_file():raise ValueError('OSTrack source/checkpoint missing')
        sys.path.insert(0,str(source))
        from lib.config.ostrack.config import cfg,update_config_from_file
        from lib.models.ostrack import build_ostrack
        from lib.utils.ce_utils import generate_mask_cond
        from lib.utils.box_ops import clip_box
        update_config_from_file(str(source/'experiments/ostrack/vitb_256_mae_ce_32x4_ep300.yaml'))
        self.cfg=cfg;self.mask_function=generate_mask_cond;self.clip_box=clip_box
        self.processing=_file_module('sar_ostrack_crop',source/'lib/train/data/processing_utils.py')
        hann=_file_module('sar_ostrack_hann',source/'lib/test/utils/hann.py')
        if not torch.cuda.is_available():raise ValueError('This offline OSTrack comparison requires CUDA')
        self.network=build_ostrack(cfg,training=False)
        self.network.load_state_dict(torch.load(str(checkpoint),map_location='cpu',weights_only=True)['net'],strict=True)
        self.network.cuda().eval()
        self.mean=torch.tensor(cfg.DATA.MEAN,device='cuda').view(1,3,1,1)
        self.std=torch.tensor(cfg.DATA.STD,device='cuda').view(1,3,1,1)
        feature_size=cfg.TEST.SEARCH_SIZE//cfg.MODEL.BACKBONE.STRIDE
        self.window=hann.hann2d(torch.tensor([feature_size,feature_size]).long(),centered=True).cuda()
        self.state=None

    def _tensor(self,image):
        import cv2
        rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        tensor=self.torch.from_numpy(rgb).to('cuda').float().permute(2,0,1).unsqueeze(0)/255.
        return (tensor-self.mean)/self.std

    def initialize(self,frame,box):
        t=self.torch;self.state=list(box)
        patch,factor,_=self.processing.sample_target(frame,list(box),self.cfg.TEST.TEMPLATE_FACTOR,
                                                    output_sz=self.cfg.TEST.TEMPLATE_SIZE)
        self.template=self._tensor(patch)
        normalized=self.processing.transform_image_to_crop(t.tensor(box),t.tensor(box),factor,
            t.tensor([self.cfg.TEST.TEMPLATE_SIZE,self.cfg.TEST.TEMPLATE_SIZE]),normalize=True).view(1,1,4).cuda()
        self.mask=self.mask_function(self.cfg,1,self.template.device,normalized) if self.cfg.MODEL.BACKBONE.CE_LOC else None
        t.cuda.synchronize()
        return self.result_type(tuple(box),1.,True,'initialized',self.tracker_name,0)

    def update(self,frame):
        import numpy as np
        t=self.torch
        patch,factor,_=self.processing.sample_target(frame,self.state,self.cfg.TEST.SEARCH_FACTOR,
                                                    output_sz=self.cfg.TEST.SEARCH_SIZE)
        with t.inference_mode():
            output=self.network.forward(template=self.template,search=self._tensor(patch),ce_template_mask=self.mask)
            response=self.window*output['score_map']
            boxes=self.network.box_head.cal_bbox(response,output['size_map'],output['offset_map']).view(-1,4)
            cx,cy,w,h=(boxes.mean(dim=0)*self.cfg.TEST.SEARCH_SIZE/factor).tolist()
            old_x,old_y,old_w,old_h=self.state
            half=.5*self.cfg.TEST.SEARCH_SIZE/factor
            mapped=[cx+old_x+old_w/2-half-w/2,cy+old_y+old_h/2-half-h/2,w,h]
            self.state=self.clip_box(mapped,*frame.shape[:2],margin=10)
            score=float(output['score_map'].max().item())
        t.cuda.synchronize()
        valid=bool(np.isfinite(self.state).all() and math.isfinite(score) and score>=.35)
        return self.result_type(tuple(self.state),score,valid,'ok' if valid else 'low_response',self.tracker_name,0)
