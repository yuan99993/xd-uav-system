#!/usr/bin/env python3
"""Evaluate automatic RGB detection on independently annotated video frames."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml
from faster_coco_eval import COCO, COCOeval_faster

from sar_yolo_detector.pixeagle.backends.dfine_backend import DFineBackend
from sar_yolo_detector.pixeagle.backends.detection_backend import DevicePreference


def overlap(a, b):
    width = max(0, min(a[0]+a[2], b[0]+b[2])-max(a[0], b[0]))
    height = max(0, min(a[1]+a[3], b[1]+b[3])-max(a[1], b[1]))
    intersection = width*height
    return intersection/max(1., a[2]*a[3]+b[2]*b[3]-intersection)


def update_confusion(confusion, predictions, truth, names, confidence):
    remaining = {i for i, box in enumerate(truth) if not box.get('iscrowd', 0)}
    crowds = [box for box in truth if box.get('iscrowd', 0)]
    for prediction in sorted(predictions, key=lambda row: -row['score']):
        if prediction['score'] < confidence:
            continue
        index = max(sorted(remaining), key=lambda i: overlap(prediction['bbox'], truth[i]['bbox']),
                    default=None)
        predicted = names[prediction['category_id']]
        if index is not None and overlap(prediction['bbox'], truth[index]['bbox']) >= .5:
            confusion[names[truth[index]['category_id']], predicted] += 1
            remaining.remove(index)
        elif not any(overlap(prediction['bbox'], box['bbox']) >= .5 for box in crowds):
            confusion['background', predicted] += 1
    for index in remaining:
        confusion[names[truth[index]['category_id']], 'background'] += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--annotations', type=Path, required=True, help='COCO boxes for raw RGB frames')
    parser.add_argument('--images', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', choices=['cuda', 'cpu'], default='cuda')
    parser.add_argument('--confidence', type=float, default=.1, help='Operating threshold for confusion matrix')
    args = parser.parse_args()
    if not 0 < args.confidence <= 1:
        parser.error('confidence must be in (0, 1]')
    if args.output.exists():
        parser.error('Use a new output file')
    package = Path(__file__).resolve().parents[1]
    manifest_path = package/'models/dfine_s_v3_16e/manifest.json'
    manifest = json.loads(manifest_path.read_text())
    labels = manifest['class_names']
    gt = COCO(str(args.annotations))
    images = sorted(gt.imgs.values(), key=lambda row: row['id'])
    names = {key: row['name'] for key, row in gt.cats.items()}
    if not images or not gt.anns or set(names.values()) != set(labels) or len(names) != len(labels):
        parser.error('Annotations must include the exact model category names, annotated targets and RGB frames')
    by_label = {name: key for key, name in names.items()}
    cfg = yaml.safe_load((package/'config/dfine_v3.yaml').read_text())['SmartTracker']
    cfg.update(SMART_TRACKER_MODELS_ROOT=str(package), SMART_TRACKER_FALLBACK_TO_CPU=False,
               SMART_TRACKER_USE_GPU=args.device == 'cuda', SMART_TRACKER_ALLOWED_CLASS_IDS=None,
               SMART_TRACKER_TRACKER_CONFIG_PATH=str(package/'config/vehicle_botsort.yaml'))
    cv2.setNumThreads(1)
    backend = DFineBackend(cfg)
    backend.load_model(str(manifest_path), DevicePreference(args.device), fallback_enabled=False)
    detections, timings, confusion = [], [], Counter()
    image_root = args.images.resolve()
    for row in images:
        path = (image_root / row['file_name']).resolve()
        if image_root not in path.parents:
            raise ValueError('Image must be inside --images: '+row['file_name'])
        frame = cv2.imread(str(path))
        if frame is None or frame.shape[:2] != (row['height'], row['width']):
            raise ValueError('Image missing or dimensions differ from annotations: '+row['file_name'])
        start = time.perf_counter()
        _, boxes = backend.detect(frame, conf=min(.001, args.confidence),
                                  iou=cfg['SMART_TRACKER_IOU_THRESHOLD'], max_det=100)
        timings.append((time.perf_counter()-start)*1000.)
        predictions = []
        for box in boxes:
            x1, y1, x2, y2 = box.aabb_xyxy
            predictions.append(dict(image_id=row['id'], category_id=by_label[labels[box.class_id]],
                                    bbox=[x1,y1,x2-x1,y2-y1], score=box.confidence))
        detections.extend(predictions)
        update_confusion(confusion, predictions, gt.imgToAnns[row['id']], names, args.confidence)
    if detections:
        predictions = gt.loadRes(detections)
    else:
        predictions = COCO()
        predictions.dataset = dict(images=list(gt.imgs.values()), categories=list(gt.cats.values()), annotations=[])
        predictions.createIndex()
    evaluator = COCOeval_faster(gt, predictions, 'bbox')
    evaluator.evaluate()
    evaluator.accumulate()
    evaluator.summarize()
    metrics = dict(zip(('ap','ap50','ap75','ap_small','ap_medium','ap_large'), map(float, evaluator.stats[:6])))
    precision = evaluator.eval['precision']
    class_ap = {}
    for offset, category in enumerate(evaluator.params.catIds):
        values = precision[:, :, offset, 0, -1]
        valid = values[values >= 0]
        class_ap[names[category]] = float(np.mean(valid)) if valid.size else None
    counts = Counter(names[a['category_id']] for a in gt.anns.values() if not a.get('iscrowd', 0))
    args.output.write_text(json.dumps(dict(
        frames=len(images), annotations_sha256=hashlib.sha256(args.annotations.read_bytes()).hexdigest(),
        weights_sha256=manifest['files']['model.safetensors'], device=args.device,
        coco=metrics, ap_by_class=class_ap, annotated_by_class=counts, confusion_threshold=args.confidence,
        confusion=[dict(actual=a,predicted=p,count=n) for (a,p),n in sorted(confusion.items())],
        inference_p50_ms=float(np.percentile(timings,50)), inference_p95_ms=float(np.percentile(timings,95))),
        indent=2)+'\n')


if __name__ == '__main__':
    main()
