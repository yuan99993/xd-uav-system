"""Pinned official D-FINE / DEIM code, with safe tensor-only model bundles."""
import hashlib
import importlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from PIL import Image
import torch

WORKSPACE=Path(__file__).resolve().parents[6]
REPOS={'dfine':'D-FINE','deim_dfine':'DEIM'}


def package_root():
    source_package = Path(__file__).resolve().parents[4]
    if (source_package/'package.xml').is_file():
        return source_package
    import rospkg
    return Path(rospkg.RosPack().get_path('sar_yolo_detector')).resolve()


def native_repository(variant, declared_root=None):
    packaged = package_root()/'third_party'/REPOS[variant]
    workspace = WORKSPACE/'third_party'/REPOS[variant]
    if declared_root is None:
        return packaged if packaged.is_dir() else workspace
    if declared_root == 'package://sar_yolo_detector/third_party/'+REPOS[variant]:
        return packaged
    # Keep historical training bundles usable, without accepting arbitrary
    # manifest-supplied Python code locations.
    if Path(declared_root).is_absolute() and Path(declared_root).resolve() == workspace.resolve():
        return workspace
    raise ValueError('Unapproved native framework root')


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config(variant,path=None,repository=None):
    repo=Path(repository) if repository is not None else native_repository(variant)
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


def _letterbox_geometry(height,width,out_height,out_width):
    scale=min(out_width/float(width),out_height/float(height))
    content_width=max(1,min(out_width,int(math.floor(width*scale+.5))))
    content_height=max(1,min(out_height,int(math.floor(height*scale+.5))))
    return content_height,content_width,(out_width-content_width)//2,(out_height-content_height)//2


def _gpu_rgb_image(pixels,size,height,width,resize_mode='stretch',output_width=None):
    out_height,out_width=int(size),int(output_width if output_width is not None else size)
    pixels=pixels.permute(2,0,1).flip(0).unsqueeze(0).to(torch.float32)
    if resize_mode=='letterbox':
        content_height,content_width,pad_x,pad_y=_letterbox_geometry(height,width,out_height,out_width)
        resized=torch.nn.functional.interpolate(pixels,size=(content_height,content_width),mode='bilinear',
            align_corners=False,antialias=True).round_().div_(255.)
        canvas=torch.full((1,3,out_height,out_width),114./255.,dtype=resized.dtype,device=resized.device)
        canvas[:,:,pad_y:pad_y+content_height,pad_x:pad_x+content_width]=resized
        return canvas.contiguous()
    if resize_mode!='stretch':raise ValueError('resize_mode must be stretch or letterbox')
    if (height,width)!=(out_height,out_width):
        pixels=torch.nn.functional.interpolate(pixels,size=(out_height,out_width),mode='bilinear',
            align_corners=False,antialias=True).round_()
    return pixels.div_(255.).contiguous()


def preprocess(frame,size,device,resize_mode='stretch'):
    if resize_mode not in ('stretch','letterbox'):raise ValueError('resize_mode must be stretch or letterbox')
    out_height,out_width=(tuple(map(int,size)) if isinstance(size,(tuple,list)) else (int(size),int(size)))
    if str(device).startswith('cuda'):
        pixels=torch.from_numpy(np.require(frame,requirements=['C','W'])).to(device)
        return _gpu_rgb_image(pixels,out_height,*frame.shape[:2],resize_mode=resize_mode,output_width=out_width)
    if resize_mode=='letterbox':
        height,width=frame.shape[:2]
        content_height,content_width,pad_x,pad_y=_letterbox_geometry(height,width,out_height,out_width)
        rgb=Image.fromarray(np.ascontiguousarray(frame[:,:,::-1])).resize(
            (content_width,content_height),Image.Resampling.BILINEAR)
        canvas=Image.new('RGB',(out_width,out_height),(114,114,114))
        canvas.paste(rgb,(pad_x,pad_y))
        array=np.asarray(canvas,dtype=np.float32)/255.
        return torch.from_numpy(np.ascontiguousarray(array.transpose(2,0,1))).unsqueeze(0).to(device)
    if frame.shape[:2] == (out_height, out_width):
        pixels = torch.from_numpy(np.require(frame, requirements=['C', 'W'])).to(device)
        return pixels.permute(2, 0, 1).flip(0).unsqueeze(0).to(torch.float32).div_(255.).contiguous()
    rgb=Image.fromarray(np.ascontiguousarray(frame[:,:,::-1])).resize((out_width,out_height),Image.Resampling.BILINEAR)
    array=np.asarray(rgb,dtype=np.float32)/255.
    return torch.from_numpy(np.ascontiguousarray(array.transpose(2,0,1))).unsqueeze(0).to(device)


def graph_preprocess(model,frame,size,device,resize_mode='stretch'):
    # Only the two common fixed shapes are cached. Arbitrary ROI shapes keep
    # the ordinary path instead of capturing a new graph during live tracking.
    height,width=frame.shape[:2]
    if (not getattr(model,'use_cuda_graph',False) or not str(device).startswith('cuda')
            or (height,width) not in ((256,256),(384,384),(512,512),(size,size))):
        return preprocess(frame,size,device,resize_mode)
    key=(height,width,size,str(device),resize_mode)
    cache=getattr(model,'_sar_preprocess_graphs',None)
    if cache is None:
        cache={};model._sar_preprocess_graphs=cache
    if key not in cache:
        staging=torch.empty((height,width,3),dtype=torch.uint8,pin_memory=True)
        pixels=torch.zeros((height,width,3),dtype=torch.uint8,device=device)
        warm_stream=torch.cuda.Stream(device=device)
        warm_stream.wait_stream(torch.cuda.current_stream(device))
        with torch.cuda.stream(warm_stream):
            for _ in range(3):_gpu_rgb_image(pixels,size,height,width,resize_mode)
        torch.cuda.current_stream(device).wait_stream(warm_stream)
        graph=torch.cuda.CUDAGraph()
        with torch.cuda.graph(graph):
            image=_gpu_rgb_image(pixels,size,height,width,resize_mode)
        cache[key]=(staging,pixels,graph,image,torch.cuda.Event())
    staging,pixels,graph,image,transferred=cache[key]
    # The host must not overwrite pinned memory while an earlier H2D copy
    # still reads it. Wait only for that copy, never the model forward pass.
    transferred.synchronize()
    staging.copy_(torch.from_numpy(np.require(frame,requirements=['C','W'])))
    pixels.copy_(staging,non_blocking=True)
    transferred.record()
    graph.replay()
    return image


def graph_forward(model,image):
    """Replay the identical deployed FP32 network in one CUDA submission.

    One graph belongs to one model/shape/device. No detection, postprocessing,
    association or image operation is omitted. Capture failure uses the normal
    network and is exposed to deployment diagnostics.
    """
    if not getattr(model,'use_cuda_graph',False) or image.device.type!='cuda':
        return model(image)
    key=(tuple(image.shape),image.dtype,image.device)
    cached=getattr(model,'_sar_graph',None)
    if cached is None or cached[0]!=key:
        try:
            # Upstream positional encodings are plain attributes, so .to()
            # does not move them with the model. Cache these constants on the
            # inference device; CPU->GPU copies cannot occur inside capture.
            encoder=getattr(model,'encoder',None)
            for index in getattr(encoder,'use_encoder_idx',[]):
                name='pos_embed'+str(index)
                position=getattr(encoder,name,None)
                if isinstance(position,torch.Tensor):setattr(encoder,name,position.to(image.device))
            static=image.clone()
            warm=torch.cuda.Stream(device=image.device)
            warm.wait_stream(torch.cuda.current_stream(image.device))
            with torch.cuda.stream(warm):
                for _ in range(3):model(static)
            torch.cuda.current_stream(image.device).wait_stream(warm)
            graph=torch.cuda.CUDAGraph()
            with torch.cuda.graph(graph):output=model(static)
            cached=(key,static,graph,output)
            model._sar_graph=cached
            model.cuda_graph_error=''
        except (RuntimeError,torch.cuda.OutOfMemoryError) as error:
            model.use_cuda_graph=False
            model.cuda_graph_error=str(error)
            return model(image)
    cached[1].copy_(image)
    cached[2].replay()
    return cached[3]


def _restore_letterbox_result(result,width,height,out_height,out_width):
    content_height,content_width,pad_x,pad_y=_letterbox_geometry(height,width,out_height,out_width)
    boxes=result['boxes'].clone()
    boxes[:,0::2]=(boxes[:,0::2]-pad_x)*(width/float(content_width))
    boxes[:,1::2]=(boxes[:,1::2]-pad_y)*(height/float(content_height))
    boxes[:,0::2].clamp_(0,width);boxes[:,1::2].clamp_(0,height)
    restored=dict(result);restored['boxes']=boxes
    return restored


def infer(model,post,frame,size,device,resize_mode='stretch'):
    h,w=frame.shape[:2]
    if resize_mode not in ('stretch','letterbox'):raise ValueError('resize_mode must be stretch or letterbox')
    with torch.inference_mode():
        start=time.perf_counter()
        target_key = (w, h, str(device))
        cached_sizes = getattr(model, '_sar_target_sizes', None)
        if cached_sizes is None or cached_sizes[0] != target_key:
            cached_sizes = (target_key, torch.tensor([[w, h]], device=device))
            model._sar_target_sizes = cached_sizes
        image=graph_preprocess(model,frame,size,device,resize_mode)
        prepared=time.perf_counter()
        output=graph_forward(model,image)
        forwarded=time.perf_counter()
        target_sizes=(torch.tensor([[size,size]],device=device) if resize_mode=='letterbox'
            else cached_sizes[1])
        result=graph_postprocess(model,post,output,target_sizes,size if resize_mode=='letterbox' else w,
            size if resize_mode=='letterbox' else h)
        if resize_mode=='letterbox':result=_restore_letterbox_result(result,w,h,size,size)
        model.runtime_host_timings=dict(preprocess_ms=(prepared-start)*1000.,forward_enqueue_ms=(forwarded-prepared)*1000.,post_enqueue_ms=(time.perf_counter()-forwarded)*1000.)
    return result


def graph_postprocess(model,post,output,target_sizes,width,height):
    # A forward graph owns stable output tensors. Replay the unchanged
    # official postprocessor against those tensors, with a bounded cache for
    # the full image and the common ROI. Other shapes keep normal execution.
    cached_forward=getattr(model,'_sar_graph',None)
    if (not getattr(model,'use_cuda_graph',False) or cached_forward is None
            or output is not cached_forward[3] or target_sizes.device.type!='cuda'
            or (width,height) not in ((256,256),(384,384),(512,512),(640,640))):
        return post(output,target_sizes)[0]
    cache=getattr(model,'_sar_postprocess_graphs',None)
    if cache is None:
        cache={};model._sar_postprocess_graphs=cache
    key=(width,height)
    signature=(id(output),id(post))
    cached=cache.get(key)
    if cached is None or cached[0]!=signature:
        sizes=target_sizes.clone()
        warm=torch.cuda.Stream(device=target_sizes.device)
        warm.wait_stream(torch.cuda.current_stream(target_sizes.device))
        with torch.cuda.stream(warm):
            for _ in range(3):post(output,sizes)
        torch.cuda.current_stream(target_sizes.device).wait_stream(warm)
        graph=torch.cuda.CUDAGraph()
        with torch.cuda.graph(graph):result=post(output,sizes)[0]
        cached=(signature,graph,result,sizes)
        cache[key]=cached
    cached[1].replay()
    return cached[2]


def normalized_rows(result,width,height,threshold,max_det,allowed_class_ids=None,nms_iou=None,agnostic=False,class_groups=None):
    records=[]
    # Filter/rank on the inference device and transfer one compact table. Three
    # independent .tolist() calls used to synchronize CUDA for every frame.
    boxes,scores,labels=result['boxes'],result['scores'],result['labels']
    valid=torch.isfinite(boxes).all(dim=1) & torch.isfinite(scores) & (scores>=threshold)
    boxes = boxes.float().clone()
    boxes[:, 0::2].clamp_(0, width)
    boxes[:, 1::2].clamp_(0, height)
    valid &= (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    if allowed_class_ids is not None:
        allowed = torch.tensor(sorted(allowed_class_ids), device=labels.device, dtype=labels.dtype)
        valid &= torch.isin(labels, allowed)
    # Keep a fixed query pool until NMS so boolean indexing does not repeatedly
    # synchronize CUDA merely to discover intermediate tensor lengths.
    boxes = torch.where(valid[:, None], boxes, torch.zeros_like(boxes))
    scores = torch.where(valid, scores, torch.full_like(scores, -float('inf')))
    if nms_iou is not None:
        from torchvision.ops import batched_nms
        groups = torch.zeros_like(labels) if agnostic else labels.clone()
        if class_groups:
            for index, family in enumerate(class_groups):
                members=torch.zeros_like(labels,dtype=torch.bool)
                for label in family:members |= labels==label
                groups.masked_fill_(members,-(index+1))
        order = batched_nms(boxes, scores, groups, float(nms_iou))
    else:
        order = torch.argsort(scores, descending=True)
    order = order[:max(0, int(max_det))]
    table = torch.cat((boxes[order], labels[order, None].float(), scores[order, None].float()), dim=1)
    # At most max_det rows cross the device boundary. The loop below already
    # rejects non-finite/low scores; boolean GPU indexing here would force an
    # extra device-to-host length synchronisation before the actual transfer.
    for *box,label,score in table.detach().cpu().tolist():
        if score<threshold or not all(math.isfinite(v) for v in (*box,score)):continue
        x1,y1,x2,y2=box
        bbox=(max(0,min(width,math.floor(x1))),max(0,min(height,math.floor(y1))),
              max(0,min(width,math.ceil(x2))),max(0,min(height,math.ceil(y2))))
        if bbox[2]>bbox[0] and bbox[3]>bbox[1]:records.append((int(label),float(score),bbox))
    return sorted(records,key=lambda v:-v[1])[:max_det]


def export_bundle(model,cfg,variant,destination,names,preprocess_mode='stretch'):
    if preprocess_mode not in ('stretch','letterbox'):raise ValueError('preprocess_mode must be stretch or letterbox')
    from safetensors.torch import save_file
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    save_file({k:v.detach().cpu().contiguous().clone() for k,v in model.state_dict().items()},str(destination/'model.safetensors'))
    resolved=dict(cfg.yaml_cfg);resolved.pop('__include__',None)
    resolved['HGNetv2']=dict(resolved['HGNetv2'],pretrained=False)
    (destination/'config.json').write_text(json.dumps(resolved,indent=2)+'\n')
    repo=WORKSPACE/'third_party'/REPOS[variant]
    sources={str(p.relative_to(repo)):digest(p) for p in sorted(repo.rglob('*.py')) if '.git' not in p.parts}
    manifest={'architecture':'dfine','variant':variant,'class_names':names,
        'preprocess_mode':preprocess_mode,
        'input_preprocessing':dict(mode=preprocess_mode,channel_order='BGR_to_RGB',
            normalization='uint8_to_float32_div_255',letterbox_fill=114 if preprocess_mode=='letterbox' else None),
        'files':{name:digest(destination/name) for name in ('model.safetensors','config.json')},
        'source_repository':{'root':str(repo),'commit':subprocess.check_output(['git','-c','safe.directory='+str(repo),'-C',str(repo),'rev-parse','HEAD'],text=True).strip(),'files':sources}}
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return destination/'manifest.json'


def load_bundle(path,device):
    from safetensors.torch import load_file
    path=Path(path);manifest=json.loads(path.read_text())
    if manifest.get('class_names')!=['car','tank','drone']:raise ValueError('Unsupported D-FINE class contract')
    if manifest.get('architecture')!='dfine' or manifest.get('variant') not in REPOS:raise ValueError('Invalid D-FINE manifest')
    if manifest.get('preprocess_mode','stretch') not in ('stretch','letterbox'):raise ValueError('Unsupported D-FINE preprocessing mode')
    if set(manifest.get('files',{}))!={'config.json','model.safetensors'}:raise ValueError('Incomplete tensor bundle')
    for name,expected in manifest['files'].items():
        p=path.parent/name
        if p.is_symlink() or not p.is_file() or digest(p)!=expected:raise ValueError('D-FINE artifact integrity mismatch: '+name)
    source=manifest['source_repository']
    repo=native_repository(manifest['variant'],source['root']).resolve()
    for name,expected in source['files'].items():
        p=repo/name
        if not p.resolve().is_relative_to(repo) or p.is_symlink() or not p.is_file() or digest(p)!=expected:
            raise ValueError('Native source integrity mismatch: '+name)
    cfg=config(manifest['variant'],path.parent/'config.json',repository=repo)
    model=cfg.model
    model.load_state_dict(load_file(str(path.parent/'model.safetensors'),device='cpu'),strict=True)
    model.to(device).eval();post=cfg.postprocessor.to(device).eval()
    return model,post,manifest
