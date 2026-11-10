#!/usr/bin/env python3
"""Bounded FP16/PT numerical and speed comparison, not a field-accuracy claim."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from sar_yolo_detector.model_registry import load_model_registry
from sar_yolo_detector.pixeagle.backends.tensorrt_compat import import_tensorrt


def overlap(a, b):
    w, h = max(0., min(a[2], b[2])-max(a[0], b[0])), max(0., min(a[3], b[3])-max(a[1], b[1]))
    inter=w*h
    return inter/max(1e-9,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--images", nargs="+",required=True)
    args=parser.parse_args()
    import_tensorrt()
    import cv2
    import numpy as np
    import torch
    from ultralytics import YOLO
    import rospkg
    root=Path(rospkg.RosPack().get_path("sar_yolo_detector"))
    registry=load_model_registry(root/"config/model_registry.yaml",root)
    frames=[cv2.imread(path) for path in args.images]
    if any(frame is None for frame in frames):raise ValueError("unreadable input image")
    report={}
    for key,item in registry.items():
        pt,engine=YOLO(item["pt_path"]),YOLO(item["engine_path"],task="detect")
        kwargs=dict(imgsz=item["input_size"],rect=False,conf=.1,iou=.7,agnostic_nms=True,max_det=100,device=0,verbose=False)
        timings={"pt":[],"engine":[]};references=[];candidates=[]
        for name,model,outputs in (("pt",pt,references),("engine",engine,candidates)):
            for _ in range(2):model.predict(frames[0],**kwargs)
            for frame in frames:
                for _ in range(3):
                    torch.cuda.synchronize();start=time.perf_counter()
                    result=model.predict(frame,**kwargs)[0]
                    torch.cuda.synchronize();timings[name].append((time.perf_counter()-start)*1000)
                outputs.append(result.boxes.data.cpu().numpy())
        tested=matched=0;max_delta=0.
        for reference,candidate in zip(references,candidates):
            used=set()
            for box in reference:
                if box[4]<.3:continue
                tested+=1
                choices=[(overlap(box,x),i) for i,x in enumerate(candidate) if i not in used and int(box[5])==int(x[5])]
                if choices:
                    score,index=max(choices)
                    if score>=.9:
                        matched+=1;used.add(index);max_delta=max(max_delta,abs(float(box[4]-candidate[index][4])))
        ratio=matched/tested if tested else 1.
        speed={name:float(np.median(values)) for name,values in timings.items()}
        report[key]={"confident_boxes":tested,"matched":matched,"match_fraction":ratio,
            "max_confidence_delta":max_delta,"median_ms":speed}
        print("ENGINE_COMPARE "+json.dumps({key:report[key]},sort_keys=True),flush=True)
        if tested==0 or ratio<.95 or max_delta>.05 or speed["engine"]>1.25*speed["pt"]:
            raise RuntimeError("engine numerical/speed criterion failed: "+key)
        del pt,engine
        torch.cuda.empty_cache()
    print("ENGINE_COMPARE_PASS "+json.dumps(report,sort_keys=True),flush=True)


if __name__=="__main__":main()
