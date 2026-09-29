#!/usr/bin/env python3
"""Evaluate two Ultralytics YOLO vehicle models on a local YOLO validation split.

The evaluator intentionally compares semantically equivalent class IDs from
different label spaces (for example COCO ``car=2`` versus a custom ``car=0``).
It computes class-specific AP and steady-state per-frame inference latency, so
the output can be used to assess a SmartTracker model replacement.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
from ultralytics import YOLO


@dataclass
class Sample:
    image: Path
    ground_truth: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--baseline-class", type=int, required=True)
    parser.add_argument("--candidate-class", type=int, required=True)
    parser.add_argument("--ground-truth-class", type=int, required=True)
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.001)
    parser.add_argument("--deployment-conf", type=float, default=0.60)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--max-images", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def xywhn_to_xyxy(row: list[float], width: int, height: int) -> list[float]:
    _, center_x, center_y, box_width, box_height = row
    return [
        (center_x - box_width / 2.0) * width,
        (center_y - box_height / 2.0) * height,
        (center_x + box_width / 2.0) * width,
        (center_y + box_height / 2.0) * height,
    ]


def load_samples(images_dir: Path, labels_dir: Path, target_class: int,
                 max_images: int) -> list[Sample]:
    samples: list[Sample] = []
    image_paths = sorted(
        path for path in images_dir.iterdir()
        if path.suffix.lower() in {".jpg", ".jpeg", ".png"} and path.exists()
    )
    if max_images:
        image_paths = image_paths[:max_images]
    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        height, width = image.shape[:2]
        label_path = labels_dir / f"{image_path.stem}.txt"
        boxes: list[list[float]] = []
        if label_path.is_file():
            for line in label_path.read_text(encoding="utf-8").splitlines():
                values = line.split()
                if len(values) != 5:
                    continue
                row = [float(value) for value in values]
                if int(row[0]) == target_class:
                    boxes.append(xywhn_to_xyxy(row, width, height))
        samples.append(Sample(image_path, np.asarray(boxes, dtype=np.float32).reshape(-1, 4)))
    if not samples:
        raise RuntimeError(f"No readable images found in {images_dir}")
    return samples


def box_iou(box: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    if not len(boxes):
        return np.empty((0,), dtype=np.float32)
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    intersection = np.maximum(x2 - x1, 0) * np.maximum(y2 - y1, 0)
    area_box = np.maximum(box[2] - box[0], 0) * np.maximum(box[3] - box[1], 0)
    area_boxes = np.maximum(boxes[:, 2] - boxes[:, 0], 0) * np.maximum(boxes[:, 3] - boxes[:, 1], 0)
    return intersection / np.maximum(area_box + area_boxes - intersection, 1e-9)


def average_precision(predictions: list[tuple[int, float, np.ndarray]],
                      ground_truth: list[np.ndarray], iou_threshold: float) -> float:
    positives = sum(len(boxes) for boxes in ground_truth)
    if not positives:
        return 0.0
    matched = [np.zeros(len(boxes), dtype=bool) for boxes in ground_truth]
    predictions = sorted(predictions, key=lambda item: item[1], reverse=True)
    true_positive = np.zeros(len(predictions), dtype=np.float32)
    false_positive = np.zeros(len(predictions), dtype=np.float32)
    for index, (image_index, _, box) in enumerate(predictions):
        ious = box_iou(box, ground_truth[image_index])
        if len(ious):
            best_index = int(np.argmax(ious))
            if ious[best_index] >= iou_threshold and not matched[image_index][best_index]:
                true_positive[index] = 1.0
                matched[image_index][best_index] = True
                continue
        false_positive[index] = 1.0
    recall = np.cumsum(true_positive) / positives
    precision = np.cumsum(true_positive) / np.maximum(np.cumsum(true_positive + false_positive), 1e-9)
    levels = np.linspace(0.0, 1.0, 101)
    return float(np.mean([precision[recall >= level].max() if np.any(recall >= level) else 0.0 for level in levels]))


def threshold_metrics(predictions: list[tuple[int, float, np.ndarray]],
                      ground_truth: list[np.ndarray], score_threshold: float) -> dict[str, float]:
    filtered = [item for item in predictions if item[1] >= score_threshold]
    matched = [np.zeros(len(boxes), dtype=bool) for boxes in ground_truth]
    tp = fp = 0
    for image_index, _, box in sorted(filtered, key=lambda item: item[1], reverse=True):
        ious = box_iou(box, ground_truth[image_index])
        if len(ious):
            best_index = int(np.argmax(ious))
            if ious[best_index] >= 0.5 and not matched[image_index][best_index]:
                matched[image_index][best_index] = True
                tp += 1
                continue
        fp += 1
    total_ground_truth = sum(len(boxes) for boxes in ground_truth)
    fn = total_ground_truth - tp
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-9)
    return {"precision": precision, "recall": recall, "f1": f1, "tp": tp, "fp": fp, "fn": fn}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def evaluate(model_path: Path, model_class: int, samples: list[Sample], args: argparse.Namespace) -> dict[str, object]:
    model = YOLO(str(model_path))
    warmup_image = cv2.imread(str(samples[0].image), cv2.IMREAD_COLOR)
    for _ in range(args.warmup):
        model.predict(warmup_image, classes=[model_class], conf=args.conf,
                      imgsz=args.imgsz, device=args.device, verbose=False)

    predictions: list[tuple[int, float, np.ndarray]] = []
    latencies: list[float] = []
    for image_index, sample in enumerate(samples):
        image = cv2.imread(str(sample.image), cv2.IMREAD_COLOR)
        started = time.perf_counter()
        result = model.predict(image, classes=[model_class], conf=args.conf,
                               imgsz=args.imgsz, device=args.device, verbose=False)[0]
        latencies.append((time.perf_counter() - started) * 1000.0)
        if result.boxes is not None and len(result.boxes):
            boxes = result.boxes.xyxy.detach().cpu().numpy()
            confidences = result.boxes.conf.detach().cpu().numpy()
            predictions.extend((image_index, float(confidence), box.astype(float))
                               for box, confidence in zip(boxes, confidences))

    ground_truth = [sample.ground_truth for sample in samples]
    ap50 = average_precision(predictions, ground_truth, 0.50)
    map5095 = float(np.mean([average_precision(predictions, ground_truth, threshold)
                             for threshold in np.arange(0.50, 0.96, 0.05)]))
    latency_sorted = sorted(latencies)
    p95_index = min(len(latency_sorted) - 1, int(np.ceil(len(latency_sorted) * 0.95)) - 1)
    return {
        "path": str(model_path),
        "sha256": sha256(model_path),
        "names": {str(key): str(value) for key, value in model.names.items()},
        "samples": len(samples),
        "ground_truth_boxes": int(sum(len(boxes) for boxes in ground_truth)),
        "predictions": len(predictions),
        "ap50": ap50,
        "map50_95": map5095,
        "at_deployment_confidence": threshold_metrics(predictions, ground_truth, args.deployment_conf),
        "latency_ms": {
            "mean": float(statistics.mean(latencies)),
            "median": float(statistics.median(latencies)),
            "p95": float(latency_sorted[p95_index]),
        },
    }


def main() -> None:
    args = parse_args()
    samples = load_samples(args.images, args.labels, args.ground_truth_class, args.max_images)
    results = {
        "protocol": {
            "dataset_images": str(args.images),
            "dataset_labels": str(args.labels),
            "ground_truth_class": args.ground_truth_class,
            "baseline_class": args.baseline_class,
            "candidate_class": args.candidate_class,
            "imgsz": args.imgsz,
            "prediction_confidence_floor": args.conf,
            "deployment_confidence": args.deployment_conf,
            "device": args.device,
        },
        "baseline": evaluate(args.baseline, args.baseline_class, samples, args),
        "candidate": evaluate(args.candidate, args.candidate_class, samples, args),
    }
    results["delta"] = {
        "ap50": results["candidate"]["ap50"] - results["baseline"]["ap50"],
        "map50_95": results["candidate"]["map50_95"] - results["baseline"]["map50_95"],
        "latency_mean_ms": results["candidate"]["latency_ms"]["mean"] - results["baseline"]["latency_ms"]["mean"],
        "f1_at_deployment_conf": (
            results["candidate"]["at_deployment_confidence"]["f1"]
            - results["baseline"]["at_deployment_confidence"]["f1"]
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
