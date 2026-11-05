"""Pinned official D-FINE / DEIM code, with safe tensor-only model bundles."""
import hashlib
import importlib
import json
import math
from pathlib import Path
import subprocess
import sys
import numpy as np
from PIL import Image
import torch

WORKSPACE=Path(__file__).resolve().parents[6]
REPOS={'dfine':'D-FINE','deim_dfine':'DEIM'}


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config(variant,path=None):
    repo=WORKSPACE/'third_party'/REPOS[variant]
    if str(repo) not in sys.path:sys.path.insert(0,str(repo))
    module=importlib.import_module('src.core' if variant=='dfine' else 'engine.core')
    if not Path(module.__file__).resolve().is_relative_to(repo):raise ValueError('Native framework namespace collision')
    default=repo/('configs/dfine/dfine_hgnetv2_s_coco.yml' if variant=='dfine' else 'configs/deim_dfine/deim_hgnetv2_s_coco.yml')
    if variant=='deim_dfine':
        # DEIM's custom transforms use torchvision<=0.20 hook names.
        # Alias only those local classes, preserving upstream algorithms/source.
        import torchvision.transforms.v2 as transforms
        custom=importlib.import_module('engine.data.transforms._transforms')
        if hasattr(transforms.Transform,'transform'):
            for name in ('PadToSize','ConvertBoxes','ConvertPILImage'):
                cls=getattr(custom,name)
                if '_transform' in cls.__dict__:cls.transform=cls.__dict__['_transform']
                if '_get_params' in cls.__dict__:cls.make_params=cls.__dict__['_get_params']
    cfg=module.YAMLConfig(str(default))
    if path is not None:
        # Upstream YAMLConfig only accepts .yml; our resolved JSON is data only.
        cfg.yaml_cfg=json.loads(Path(path).read_text())
        if "__include__" in cfg.yaml_cfg:raise ValueError("Bundle config must be self-contained")
    cfg.yaml_cfg['HGNetv2']['pretrained']=False
    cfg.yaml_cfg['num_classes']=3
    cfg.yaml_cfg['remap_mscoco_category']=False
    return cfg


def preprocess(frame,size,device):
    rgb=Image.fromarray(np.ascontiguousarray(frame[:,:,::-1])).resize((size,size),Image.Resampling.BILINEAR)
    array=np.asarray(rgb,dtype=np.float32)/255.
    return torch.from_numpy(np.ascontiguousarray(array.transpose(2,0,1))).unsqueeze(0).to(device)


def infer(model,post,frame,size,device):
    h,w=frame.shape[:2]
    with torch.inference_mode():
        output=model(preprocess(frame,size,device))
        result=post(output,torch.tensor([[w,h]],device=device))[0]
    return result


def normalized_rows(result,width,height,threshold,max_det):
    records=[]
    # Filter/rank on the inference device and transfer one compact table. Three
    # independent .tolist() calls used to synchronize CUDA for every frame.
    boxes,scores,labels=result['boxes'],result['scores'],result['labels']
    valid=torch.isfinite(boxes).all(dim=1) & torch.isfinite(scores) & (scores>=threshold)
    table=torch.cat((boxes[valid].float(), labels[valid,None].float(), scores[valid,None].float()),dim=1)
    table=table[torch.argsort(table[:,5],descending=True)[:max(0,int(max_det))]]
    for *box,label,score in table.detach().cpu().tolist():
        if score<threshold or not all(math.isfinite(v) for v in (*box,score)):continue
        x1,y1,x2,y2=box
        bbox=(max(0,min(width,math.floor(x1))),max(0,min(height,math.floor(y1))),
              max(0,min(width,math.ceil(x2))),max(0,min(height,math.ceil(y2))))
        if bbox[2]>bbox[0] and bbox[3]>bbox[1]:records.append((int(label),float(score),bbox))
    return sorted(records,key=lambda v:-v[1])[:max_det]


def export_bundle(model,cfg,variant,destination,names):
    from safetensors.torch import save_file
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    save_file({k:v.detach().cpu().contiguous().clone() for k,v in model.state_dict().items()},str(destination/'model.safetensors'))
    resolved=dict(cfg.yaml_cfg);resolved.pop('__include__',None)
    resolved['HGNetv2']=dict(resolved['HGNetv2'],pretrained=False)
    (destination/'config.json').write_text(json.dumps(resolved,indent=2)+'\n')
    repo=WORKSPACE/'third_party'/REPOS[variant]
    sources={str(p.relative_to(repo)):digest(p) for p in sorted(repo.rglob('*.py')) if '.git' not in p.parts}
    manifest={'architecture':'dfine','variant':variant,'class_names':names,
        'files':{name:digest(destination/name) for name in ('model.safetensors','config.json')},
        'source_repository':{'root':str(repo),'commit':subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),'files':sources}}
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return destination/'manifest.json'


def load_bundle(path,device):
    from safetensors.torch import load_file
    path=Path(path);manifest=json.loads(path.read_text())
    if manifest.get('class_names')!=['car','tank','drone']:raise ValueError('Unsupported D-FINE class contract')
    if manifest.get('architecture')!='dfine' or manifest.get('variant') not in REPOS:raise ValueError('Invalid D-FINE manifest')
    if set(manifest.get('files',{}))!={'config.json','model.safetensors'}:raise ValueError('Incomplete tensor bundle')
    for name,expected in manifest['files'].items():
        p=path.parent/name
        if p.is_symlink() or not p.is_file() or digest(p)!=expected:raise ValueError('D-FINE artifact integrity mismatch: '+name)
    source=manifest['source_repository'];repo=Path(source['root']).resolve()
    if repo!=WORKSPACE/'third_party'/REPOS[manifest['variant']]:raise ValueError('Unapproved native framework root')
    for name,expected in source['files'].items():
        p=repo/name
        if not p.resolve().is_relative_to(repo) or p.is_symlink() or not p.is_file() or digest(p)!=expected:
            raise ValueError('Native source integrity mismatch: '+name)
    cfg=config(manifest['variant'],path.parent/'config.json')
    model=cfg.model
    model.load_state_dict(load_file(str(path.parent/'model.safetensors'),device='cpu'),strict=True)
    model.to(device).eval();post=cfg.postprocessor.to(device).eval()
    return model,post,manifest
