"""Official D-FINE and DEIM+D-FINE models; detector IDs are never track IDs."""
import importlib.util
import math
import threading
import time
from pathlib import Path
from contextlib import nullcontext
from collections import Counter
from types import SimpleNamespace
import numpy as np
from .rtdetr_backend import RTDetrBackend
from .ultralytics_backend import UltralyticsBackend
from ..detection_adapter import NormalizedDetection
from ...temporal_detection_evidence import TemporalDetectionEvidence


class DFineBackend(RTDetrBackend):
    def __init__(self, config, **kwargs):
        super().__init__(config, **kwargs)
        self._external_tracker=None
        self._external_tracker_type=None
        self._external_tracker_shape=None
        self._inference_stream=None
        self._inference_stream_device=None
        self._stage_lock=threading.Lock()
        self._stage_counts=Counter()
        self._stage_last={}
        self._class_consensus_enabled=bool(config.get('SMART_TRACKER_CLASS_CONSENSUS_ENABLED', False))
        self._class_consensus=TemporalDetectionEvidence(birth_confirmation=False, class_consensus=True)
        self._last_tracking_capture_stamp=None
        self.last_classification_evidence=[]
        self.last_timings={}

    def stage_trace(self):
        """Actual detector/NMS/tracker call counts for the live pipeline."""
        with self._stage_lock:
            model=getattr(getattr(self,'_model',None),'model',None)
            return {'counts': dict(self._stage_counts), 'last': dict(self._stage_last), 'timings':dict(getattr(self,'last_timings',{})),
                    'tensorrt_active':getattr(getattr(self,'_model',None),'trt_engine',None) is not None,
                    'cuda_graph_active':bool(getattr(model,'_sar_graph',None)) and getattr(getattr(self,'_model',None),'trt_engine',None) is None,
                    'cuda_graph_error':str(getattr(model,'cuda_graph_error',''))}

    def _record_stage(self, stage, **detail):
        # Some small unit fixtures construct this backend through __new__.
        lock=getattr(self, '_stage_lock', None)
        if lock is None:
            return
        with lock:
            self._stage_counts[stage]+=1
            self._stage_last[stage]=detail

    @property
    def backend_name(self):return 'dfine'

    @property
    def is_available(self):
        return all(importlib.util.find_spec(m) is not None for m in ('torch','torchvision','safetensors','faster_coco_eval'))

    def _load_candidate(self,path,target_device):
        from .dfine_runtime import load_bundle
        provenance=self._verify_file(path)
        model,post,manifest=load_bundle(path,target_device)
        declared_mode=manifest.get('preprocess_mode','stretch')
        resize_mode=str(self._config.get('SMART_TRACKER_DFINE_RESIZE_MODE',declared_mode)).strip().lower()
        if resize_mode not in ('stretch','letterbox'):
            raise ValueError('SMART_TRACKER_DFINE_RESIZE_MODE must be stretch or letterbox')
        if resize_mode!=declared_mode and not bool(self._config.get(
                'SMART_TRACKER_DFINE_ALLOW_PREPROCESS_OVERRIDE',False)):
            raise ValueError('D-FINE resize mode differs from its model manifest; retrain/export with the selected preprocessing mode')
        native_deploy=bool(self._config.get('SMART_TRACKER_DFINE_NATIVE_DEPLOY', True))
        if native_deploy:
            # Official inference conversion: fuse reparameterizable branches
            # and discard unused decoder heads; checkpoint/precision unchanged.
            model.deploy()
        package_root_value=str(self._config.get('SMART_TRACKER_MODELS_ROOT','')).strip()
        package_root=Path(package_root_value).expanduser() if package_root_value else None
        def resolve_engine_path(value):
            value=str(value or '').strip()
            if not value:return ''
            engine=Path(value).expanduser()
            if not engine.is_absolute() and package_root is not None:
                engine=package_root/engine
            return str(engine.resolve())
        engine_path = resolve_engine_path(self._config.get('SMART_TRACKER_DFINE_TENSORRT_ENGINE', ''))
        model.use_cuda_graph=bool(self._config.get('SMART_TRACKER_DFINE_CUDA_GRAPH',False)) and not engine_path
        if model.use_cuda_graph and str(target_device).startswith('cuda'):
            # Capture the graph and finish CUDA lazy initialization before
            # image workers expose frames for operator selection.
            from .dfine_runtime import infer, normalized_rows
            size=self._resolve_image_size(None)
            warm_frame=np.zeros((size,size,3),dtype=np.uint8)
            warm_output=infer(model,post,warm_frame,size,target_device)
            normalized_rows(warm_output,size,size,.04,20)
            for roi_size in (256,384,512):
                warm_output=infer(model,post,np.zeros((roi_size,roi_size,3),dtype=np.uint8),size,target_device)
                normalized_rows(warm_output,roi_size,roi_size,.04,20)
        trt_engine = None
        roi_engine = None
        if engine_path:
            from .dfine_tensorrt import DFineTensorRT
            from .dfine_runtime import digest
            if self._config.get('SMART_TRACKER_DFINE_TENSORRT_MANIFEST_SHA256') != digest(path):
                raise ValueError('TensorRT engine was exported from a different model manifest')
            trt_engine = DFineTensorRT(engine_path,
                self._config.get('SMART_TRACKER_DFINE_TENSORRT_SHA256',''),target_device,
                fused_preprocess=bool(self._config.get('SMART_TRACKER_DFINE_FUSED_PREPROCESS',False)),
                resize_mode=resize_mode)
            roi_path = resolve_engine_path(self._config.get('SMART_TRACKER_DFINE_TENSORRT_ROI_ENGINE',''))
            if roi_path:
                roi_engine = DFineTensorRT(roi_path,
                    self._config.get('SMART_TRACKER_DFINE_TENSORRT_ROI_SHA256',''),target_device,
                    fused_preprocess=bool(self._config.get('SMART_TRACKER_DFINE_FUSED_PREPROCESS',False)),
                    resize_mode=resize_mode)
        candidate=SimpleNamespace(trt_engine=trt_engine,trt_roi_engine=roi_engine,model=model,post=post,device=target_device,task='detect',
            names=dict(enumerate(manifest['class_names'])),variant=manifest['variant'],
            resize_mode=resize_mode,
            native_deploy=native_deploy,
            encoder_eval_spatial_size=model.encoder.eval_spatial_size,
            decoder_eval_spatial_size=model.decoder.eval_spatial_size)
        return candidate,provenance

    def load_model(self,*args,**kwargs):
        UltralyticsBackend.load_model(self,*args,**kwargs)
        self._runtime_info.update(backend=self._model.variant+'_'+self._runtime_info['effective_device'],model_format='safetensors_manifest')
        self._runtime_info['resize_mode']=self._model.resize_mode
        self._runtime_info['native_deploy']=self._model.native_deploy
        self._runtime_info['tensorrt_active']=getattr(self._model,'trt_engine',None) is not None
        if self._runtime_info['tensorrt_active']:
            self._runtime_info['backend']='dfine_tensorrt_cuda'
            self._runtime_info['tensorrt_engine_path']=str(self._config['SMART_TRACKER_DFINE_TENSORRT_ENGINE'])
            self._runtime_info['tensorrt_engine_sha256']=str(self._config['SMART_TRACKER_DFINE_TENSORRT_SHA256'])
        return dict(self._runtime_info)

    def _configured_nms(self,detections,iou):
        ranked=sorted(detections,key=lambda detection:detection.confidence,reverse=True)
        kept=[]
        for candidate in ranked:
            x1,y1,x2,y2=candidate.aabb_xyxy
            area=max(0,x2-x1)*max(0,y2-y1)
            duplicate=False
            for selected in kept:
                if not self._agnostic_nms and candidate.class_id!=selected.class_id and not {candidate.class_id, selected.class_id} <= {0, 1}:continue
                sx1,sy1,sx2,sy2=selected.aabb_xyxy
                intersection=max(0,min(x2,sx2)-max(x1,sx1))*max(0,min(y2,sy2)-max(y1,sy1))
                union=area+max(0,sx2-sx1)*max(0,sy2-sy1)-intersection
                if union>0 and intersection/union>float(iou):
                    duplicate=True
                    break
            if not duplicate:kept.append(candidate)
        return kept

    def detect(self,frame,conf=.3,iou=.3,max_det=20,image_size=None):
        if self._model is None:raise RuntimeError('D-FINE model not loaded')
        runtime=self._model
        context=nullcontext()
        if str(runtime.device).startswith('cuda'):
            import torch
            device=str(torch.device(runtime.device))
            if self._inference_stream is None or self._inference_stream_device!=device:
                self._inference_stream=torch.cuda.Stream(device=runtime.device)
                self._inference_stream.wait_stream(torch.cuda.current_stream(runtime.device))
                self._inference_stream_device=device
            context=torch.cuda.stream(self._inference_stream)
        with context:
            return self._detect_on_current_stream(frame,conf,iou,max_det,image_size)

    def _detect_on_current_stream(self,frame,conf=.3,iou=.3,max_det=20,image_size=None):
        if self._model is None:raise RuntimeError('D-FINE model not loaded')
        if frame.ndim!=3 or frame.shape[2]!=3 or frame.dtype!=np.uint8:raise ValueError('Expected uint8 BGR image')
        from .dfine_runtime import infer,normalized_rows
        h,w=frame.shape[:2];runtime=self._model
        size=self._resolve_image_size(image_size)
        # The bundled 640 model caches sinusoidal positions and encoder anchors
        # for its training canvas. For an explicitly requested alternate size,
        # switch those two modules to their upstream dynamic-grid path. The
        # default 640 inference remains on its cached path.
        runtime.model.encoder.eval_spatial_size=(runtime.encoder_eval_spatial_size if size==640 else None)
        runtime.model.decoder.eval_spatial_size=(runtime.decoder_eval_spatial_size if size==640 else None)
        measured=time.perf_counter()
        engine = getattr(runtime,'trt_engine',None)
        roi_engine = getattr(runtime,'trt_roi_engine',None)
        # Older CPU-only fixtures construct a minimal runtime object without
        # the preprocessing manifest field. Keep those on the historical
        # stretch path; production runtimes always carry their declared mode.
        resize_mode=getattr(runtime,'resize_mode','stretch')
        if roi_engine is not None and image_size is not None and size <= max(roi_engine.shape[2:]):
            engine = roi_engine
        result = (engine.infer(frame,runtime.post,resize_mode,conf,max_det) if engine is not None else
                  infer(runtime.model,runtime.post,frame,size,runtime.device,resize_mode))
        inference_queued=time.perf_counter()
        self._record_stage('dfine_infer', image_width=w, image_height=h, image_size=size,
            network_input_shape=list(engine.shape) if engine is not None else [1,3,size,size])
        records=[]
        # Decode the bounded postprocessor pool before limiting output. Taking
        # the top max_det first lets duplicate/class-filtered queries consume
        # the budget and erase otherwise valid small targets.
        for label,score,bbox in normalized_rows(result,w,h,conf,max_det,self._allowed_class_ids,iou,self._agnostic_nms,
                [[0, 1], [2]] if not self._agnostic_nms else None):
            if self._allowed_class_ids is not None and label not in self._allowed_class_ids:continue
            records.append(NormalizedDetection(track_id=-1,class_id=label,confidence=score,aabb_xyxy=bbox,
                center_xy=((bbox[0]+bbox[2])//2,(bbox[1]+bbox[3])//2),track_id_is_stable=False))
        # Keep D-FINE's output contract aligned with the replaced YOLO
        # backend: run the configured NMS before boxes reach BoT-SORT. The
        # transformer postprocessor can emit the same object under multiple
        # classes; without agnostic NMS these become separate tracked IDs.
        transferred=time.perf_counter()
        kept=records
        self.last_timings=dict(infer_host_ms=(inference_queued-measured)*1000.,transfer_decode_ms=(transferred-inference_queued)*1000.,nms_ms=(time.perf_counter()-transferred)*1000.)
        if engine is not None:
            self.last_timings.update(getattr(engine,'last_stage_timings',{}))
            postprocess_diagnostics=dict(getattr(engine,'last_postprocess_diagnostics',{}))
            if postprocess_diagnostics:
                self.last_timings['postprocess_diagnostics']=postprocess_diagnostics
                self.last_timings['prefilter_topk']=postprocess_diagnostics.get('prefilter_topk')
        self.last_timings.update(getattr(runtime.model,'runtime_host_timings',{}))
        self._record_stage('configured_nms', input_queries=len(result['scores']), output_boxes=len(kept),
            class_agnostic=bool(self._agnostic_nms), iou_threshold=float(iou))
        return 'detect',kept

    def _get_external_tracker(self,tracker_type,shape):
        from ultralytics.cfg import IterableSimpleNamespace, YAML
        from ultralytics.trackers.track import TRACKER_MAP
        key=str(tracker_type or self.tracker_type_str).strip().lower()
        if key not in {'botsort','bytetrack'}:
            raise ValueError(f'Unsupported external tracker: {key}')
        if self._tracker_config_path:
            config=YAML.load(self._tracker_config_path)
            configured=str(config.get('tracker_type','')).strip().lower()
            if configured!=key:raise ValueError(f'Tracker config uses {configured}, requested {key}')
        else:
            raise RuntimeError('D-FINE tracking requires the same explicit tracker YAML as the YOLO profile')
        if (self._external_tracker is None or self._external_tracker_type!=key or
                self._external_tracker_shape!=tuple(shape)):
            args=IterableSimpleNamespace(**config)
            args.device=str(self._model.device)
            args.model='auto'
            self._external_tracker=TRACKER_MAP[key](args=args)
            self._external_tracker_type=key
            self._external_tracker_shape=tuple(shape)
            if hasattr(self,'_class_consensus'):self._class_consensus.reset()
        return self._external_tracker

    def detect_and_track(self,frame,conf=.3,iou=.3,max_det=20,tracker_type='botsort',tracker_args=None):
        """Run D-FINE detection and pass its boxes through the configured legacy tracker."""
        from ultralytics.engine.results import Boxes
        import torch

        capture_stamp=(tracker_args or {}).get('_capture_stamp')
        stamp=float(capture_stamp) if capture_stamp is not None else time.monotonic()
        self.last_classification_evidence=[]
        if self._class_consensus_enabled and (not math.isfinite(stamp) or
                (self._last_tracking_capture_stamp is not None and stamp<=self._last_tracking_capture_stamp)):
            self._record_stage('classification_capture_rejected')
            return 'track',[]
        _,detections=self.detect(frame,conf,iou,max_det)
        h,w=frame.shape[:2]
        rows=np.asarray([[*d.aabb_xyxy,d.confidence,d.class_id] for d in detections],dtype=np.float32).reshape(-1,6)
        tracker=self._get_external_tracker(tracker_type,(h,w))
        association_started=time.perf_counter()
        tracked=tracker.update(Boxes(torch.from_numpy(rows),orig_shape=(h,w)),frame)
        self.last_timings["botsort_ms"]=(time.perf_counter()-association_started)*1000.
        self._record_stage('external_tracker_update', tracker_type=str(tracker_type),
            input_boxes=len(detections), tracked_rows=len(tracked))
        output=[]
        for row in tracked:
            if len(row)<8 or not np.isfinite(row[:8]).all():continue
            x1,y1,x2,y2,track_id,score,label,index=row[:8]
            index=int(index)
            if index<0 or index>=len(detections):continue
            x1=max(0,min(w,int(math.floor(x1))));y1=max(0,min(h,int(math.floor(y1))))
            x2=max(0,min(w,int(math.ceil(x2))));y2=max(0,min(h,int(math.ceil(y2))))
            if x2<=x1 or y2<=y1:continue
            output.append(NormalizedDetection(track_id=int(track_id),class_id=int(label),
                confidence=float(score),aabb_xyxy=(x1,y1,x2,y2),
                center_xy=((x1+x2)//2,(y1+y2)//2),track_id_is_stable=True))
        self.last_classification_evidence=[]
        if self._class_consensus_enabled:
            original={d.track_id:d for d in output}
            rows=self._class_consensus.process([dict(track_id=d.track_id, stable=d.track_id_is_stable,
                label=d.class_id, score=d.confidence, box=d.aabb_xyxy) for d in output],stamp)
            output=[]
            for row in rows:
                detection=original[row['track_id']]
                detection.raw_class_id=detection.class_id
                detection.class_id=row['label']
                detection.class_label_source=row.get('label_source','detector')
                output.append(detection)
            self._record_stage('temporal_class_consensus', input_boxes=len(original), output_boxes=len(output),
                capture_stamp=stamp, counters=dict(self._class_consensus.stats))
            self._last_tracking_capture_stamp=stamp
        for detection in output:
            self.last_classification_evidence.append(dict(id=detection.track_id,
                raw_class_id=detection.raw_class_id if detection.raw_class_id is not None else detection.class_id,
                stable_class_id=detection.class_id, label_source=detection.class_label_source,
                proposal_confidence=detection.confidence))
        return 'track',output

    def supports_tracking(self):return True

    def reset_tracking(self):
        if self._external_tracker is not None:self._external_tracker.reset()
        self._external_tracker=None
        self._external_tracker_type=None
        self._external_tracker_shape=None
        if hasattr(self,'_class_consensus'):self._class_consensus.reset()
        self._last_tracking_capture_stamp=None
        self.last_classification_evidence=[]

    def unload_model(self):
        self.reset_tracking()
        super().unload_model()
