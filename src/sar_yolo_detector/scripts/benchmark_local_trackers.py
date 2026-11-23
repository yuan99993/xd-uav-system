#!/usr/bin/env python3
"""Identical controlled image sequences for KCF/CSRT/NanoTrack comparison.

The target texture comes from the package's own UE fixture. Geometry is known
by construction. This is a synthetic stress test, not real-map accuracy.
"""
import argparse
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
import rospkg
from sar_yolo_detector.manual_box_tracker import ManualBoxTracker
from sar_yolo_detector.box_fusion import iou


def sequence(fixture, small=False):
    texture=fixture[232:339,372:424]
    rng=np.random.RandomState(48)
    noise=rng.randint(-8,9,(360,640,1)).astype(np.int16)
    for index in range(180):
        scale=(.45 if small else .8)*(1+.15*math.sin(index/19.))
        w,h=max(12,int(52*scale)),max(20,int(107*scale))
        x=int(160+index*.9+25*math.sin(index/15.))
        y=int(140+40*math.sin(index/26.))
        colour=([48,85,125] if index<60 else [40,110,40] if index<120 else [100,95,90])
        frame=np.uint8(np.clip(np.array(colour)[None,None,:]+noise,0,255))
        patch=cv2.resize(texture,(w,h))
        frame[y:y+h,x:x+w]=patch
        visible=not 95<=index<105
        if not visible:frame[y:y+h,x:x+w]=colour
        yield frame,(float(x),float(y),float(w),float(h)),visible


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ostrack-source',type=Path)
    parser.add_argument('--ostrack-checkpoint',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--nano-occlusion-guard',action='store_true')
    parser.add_argument('--nano-engine-directory',type=Path)
    parser.add_argument('--nano-engine-manifest-sha256',default='')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if bool(args.ostrack_source)!=bool(args.ostrack_checkpoint):parser.error('Supply both OSTrack source and checkpoint')
    cv2.setNumThreads(1)
    if args.ostrack_source:
        import torch
        torch.set_num_threads(1)
    package=Path(rospkg.RosPack().get_path('sar_yolo_detector'))
    fixture=cv2.imread(str(package/'test/fixtures/dfine_v3_ue.jpg'))
    report=dict(kind='controlled synthetic UE-texture tracking stress; not real-map accuracy',results={})
    for scenario,small in [('background_change',False),('small_target',True)]:
        frames=list(sequence(fixture,small));rows={}
        video=cv2.VideoWriter(str(args.output/(scenario+'.avi')),cv2.VideoWriter_fourcc(*'MJPG'),30,(640,360))
        for frame,_,_ in frames:video.write(frame)
        video.release()
        (args.output/(scenario+'_truth.json')).write_text(json.dumps([dict(box=b,visible=v) for _,b,v in frames]))
        algorithms=['kcf','csrt','nano']
        if args.ostrack_source:
            algorithms.append('ostrack')
        if args.nano_engine_directory:
            algorithms.append('nano_trt')
        for algorithm in algorithms:
            if algorithm=='ostrack':
                from sar_yolo_detector.ostrack_candidate import OSTrackCandidate
                tracker=OSTrackCandidate(args.ostrack_source,args.ostrack_checkpoint)
            else:
                tracker=ManualBoxTracker(dict(feature_tracker=dict(tracker_algorithm=algorithm,require_opencv_tracker=True,
                    nano_occlusion_guard_enabled=args.nano_occlusion_guard,
                    nano_engine_directory=str(args.nano_engine_directory),
                    nano_engine_manifest_sha256=args.nano_engine_manifest_sha256)))
            tracker.initialize(frames[0][0],frames[0][1])
            timings=[];overlaps=[];false_visible=0;valid=0;response_rows=[]
            for frame,truth,visible in frames[1:]:
                started=time.perf_counter();result=tracker.update(frame);timings.append((time.perf_counter()-started)*1000)
                if algorithm=='nano_trt':
                    response_rows.append(dict(index=len(response_rows)+1,visible=visible,valid=result.valid,**getattr(tracker._opencv_tracker,'response_diagnostics',{})))
                if visible:
                    overlaps.append(iou(result.bbox,truth) if result.valid else 0.)
                    valid+=int(result.valid)
                else:false_visible+=int(result.valid)
            if response_rows:(args.output/(scenario+'_nano_responses.json')).write_text(json.dumps(response_rows,indent=2))
            rows[algorithm]=dict(p50_ms=float(np.percentile(timings,50)),p95_ms=float(np.percentile(timings,95)),
                mean_iou=float(np.mean(overlaps)),success_iou50=float(np.mean(np.array(overlaps)>=.5)),
                valid_visible_frames=valid,visible_frames=len(overlaps),false_valid_occluded_frames=false_visible,
                active_algorithm=tracker.tracker_name,device='cuda' if algorithm in ('ostrack','nano_trt') else 'cpu')
            report['results'][scenario]=rows
            (args.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
        report['results'][scenario]=rows
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
