#!/usr/bin/env python3
"""Evaluate detection models with UAV-relevant temporal metrics.

Ultralytics mAP scores boxes independently. This evaluator consumes the
manifest written by ``prepare_fireman_multiclass.py`` and also reports whether
each continuous fire/smoke event was detected, false-alarm events per observed
hour, time to first correct detection, and recall by supplied altitude band.
"""

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Box = Tuple[int, float, float, float, float, float]  # class, confidence, x1, y1, x2, y2


def read_manifest(path: Path, split: str) -> List[dict]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
    rows = [row for row in rows if row["split"] == split]
    if not rows:
        raise ValueError(f"manifest has no {split} samples")
    return sorted(rows, key=lambda row: (row["group_id"], row.get("timestamp_sec") is None,
                                         row.get("timestamp_sec", 0.0), row["frame_index"]))


def read_names(dataset_yaml: Path) -> List[str]:
    names, in_names = [], False
    for line in dataset_yaml.read_text(encoding="utf-8").splitlines():
        if line.strip() == "names:":
            in_names = True
            continue
        if in_names and line.startswith("  ") and ":" in line:
            names.append(line.split(":", 1)[1].strip().strip("'\""))
        elif in_names and line and not line.startswith(" "):
            break
    if not names:
        raise ValueError(f"cannot read class names from {dataset_yaml}")
    return names


def iou(left: Box, right: Box) -> float:
    x1 = max(left[2], right[2])
    y1 = max(left[3], right[3])
    x2 = min(left[4], right[4])
    y2 = min(left[5], right[5])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = ((left[4] - left[2]) * (left[5] - left[3]) +
             (right[4] - right[2]) * (right[5] - right[3]) - intersection)
    return intersection / union if union > 0.0 else 0.0


def image_shape(path: Path) -> Tuple[int, int]:
    import cv2
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"cannot decode {path}")
    return image.shape[1], image.shape[0]


def read_ground_truth(dataset_root: Path, row: dict) -> List[Box]:
    width, height = image_shape(dataset_root / row["image_key"])
    labels: List[Box] = []
    label_path = dataset_root / row["label_key"]
    for line in label_path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) != 5:
            continue
        class_id = int(fields[0])
        xc, yc, bw, bh = (float(value) for value in fields[1:])
        labels.append((class_id, 1.0, (xc - bw / 2.0) * width, (yc - bh / 2.0) * height,
                       (xc + bw / 2.0) * width, (yc + bh / 2.0) * height))
    return labels


def infer(model, image_path: Path, confidence: float) -> List[Box]:
    result = model.predict(str(image_path), conf=confidence, verbose=False)[0]
    if result.boxes is None:
        return []
    xyxy = result.boxes.xyxy.cpu().tolist()
    scores = result.boxes.conf.cpu().tolist()
    classes = result.boxes.cls.cpu().tolist()
    return [(int(class_id), float(score), *map(float, box))
            for box, score, class_id in zip(xyxy, scores, classes)]


def match_predictions(predictions: Sequence[Box], ground_truth: Sequence[Box],
                      minimum_iou: float) -> Tuple[List[Box], List[Box], List[Box]]:
    """Return true positives, unmatched predictions, and unmatched GT boxes."""
    used_gt, true_positives = set(), []
    for prediction in sorted(predictions, key=lambda box: box[1], reverse=True):
        candidates = [(iou(prediction, truth), index) for index, truth in enumerate(ground_truth)
                      if index not in used_gt and truth[0] == prediction[0]]
        if candidates and max(candidates)[0] >= minimum_iou:
            _, index = max(candidates)
            used_gt.add(index)
            true_positives.append(prediction)
    unmatched_predictions = [box for box in predictions if box not in true_positives]
    unmatched_truth = [box for index, box in enumerate(ground_truth) if index not in used_gt]
    return true_positives, unmatched_predictions, unmatched_truth


def contiguous_events(rows: Sequence[dict], class_id: int, class_name: str,
                      gap_seconds: float) -> List[dict]:
    events, active = [], None
    for row in rows:
        timestamp = row.get("timestamp_sec")
        has_class = class_name in row["classes"]
        same_group = active is not None and active["group_id"] == row["group_id"]
        close = (same_group and timestamp is not None and active["end"] is not None and
                 timestamp - active["end"] <= gap_seconds)
        if has_class and (active is None or not close):
            if active is not None:
                events.append(active)
            active = {"group_id": row["group_id"], "class_id": class_id,
                      "class_name": class_name, "start": timestamp, "end": timestamp,
                      "altitude_m": row.get("altitude_m"), "detected_at": None}
        elif has_class:
            active["end"] = timestamp
            if active["altitude_m"] is None:
                active["altitude_m"] = row.get("altitude_m")
        elif active is not None and not same_group:
            events.append(active)
            active = None
    if active is not None:
        events.append(active)
    return events


def coverage_seconds(rows: Sequence[dict]) -> Optional[float]:
    by_group: Dict[str, List[float]] = defaultdict(list)
    for row in rows:
        value = row.get("timestamp_sec")
        if value is not None:
            by_group[row["group_id"]].append(float(value))
    if not by_group:
        return None
    total = sum(max(values) - min(values) for values in by_group.values() if len(values) > 1)
    return total if total > 0.0 else None


def alarm_count(false_positives: Sequence[dict], gap_seconds: float) -> int:
    """Count temporally grouped false alarms, never raw per-frame FP boxes."""
    last: Dict[Tuple[str, int], float] = {}
    count = 0
    for item in sorted(false_positives, key=lambda value: (value["group_id"], value["class_id"],
                                                           value["timestamp_sec"] is None,
                                                           value["timestamp_sec"] or 0.0)):
        timestamp = item["timestamp_sec"]
        if timestamp is None:
            continue
        key = item["group_id"], item["class_id"]
        if key not in last or timestamp - last[key] > gap_seconds:
            count += 1
        last[key] = timestamp
    return count


def altitude_bin(value: Optional[float], edges: Sequence[float]) -> Optional[str]:
    if value is None:
        return None
    for low, high in zip(edges, edges[1:]):
        if low <= value < high:
            return f"[{low:g},{high:g})m"
    return f"[{edges[-1]:g},inf)m" if value >= edges[-1] else None


def evaluate(rows: Sequence[dict], dataset_root: Path, model, names: Sequence[str],
             confidence: float, minimum_iou: float, event_gap_seconds: float,
             false_alarm_gap_seconds: float, altitude_edges: Sequence[float]) -> dict:
    true_positive_time: Dict[Tuple[str, int], List[float]] = defaultdict(list)
    false_positives: List[dict] = []
    for index, row in enumerate(rows, start=1):
        truth = read_ground_truth(dataset_root, row)
        predictions = infer(model, dataset_root / row["image_key"], confidence)
        positives, unmatched, _ = match_predictions(predictions, truth, minimum_iou)
        timestamp = row.get("timestamp_sec")
        for box in positives:
            if timestamp is not None:
                true_positive_time[(row["group_id"], box[0])].append(float(timestamp))
        for box in unmatched:
            false_positives.append({"group_id": row["group_id"], "class_id": box[0],
                                    "timestamp_sec": timestamp, "confidence": box[1]})
        if index % 100 == 0 or index == len(rows):
            print(f"evaluated {index}/{len(rows)} frames", flush=True)

    events = []
    for class_id, class_name in enumerate(names):
        events.extend(contiguous_events(rows, class_id, class_name, event_gap_seconds))
    for event in events:
        detections = true_positive_time.get((event["group_id"], event["class_id"]), [])
        in_event = [value for value in detections if event["start"] is not None and
                    event["end"] is not None and event["start"] <= value <= event["end"]]
        event["detected_at"] = min(in_event) if in_event else None

    detected = [event for event in events if event["detected_at"] is not None]
    latency = [event["detected_at"] - event["start"] for event in detected
               if event["start"] is not None]
    coverage = coverage_seconds(rows)
    alarms = alarm_count(false_positives, false_alarm_gap_seconds)
    altitude_metrics = {}
    for event in events:
        label = altitude_bin(event["altitude_m"], altitude_edges)
        if label is None:
            continue
        item = altitude_metrics.setdefault(label, {"events": 0, "detected_events": 0})
        item["events"] += 1
        item["detected_events"] += event["detected_at"] is not None
    for item in altitude_metrics.values():
        item["event_recall"] = item["detected_events"] / item["events"]
    per_class = {}
    for class_id, class_name in enumerate(names):
        subset = [event for event in events if event["class_id"] == class_id]
        per_class[class_name] = {
            "events": len(subset), "detected_events": sum(event["detected_at"] is not None for event in subset),
            "event_recall": (sum(event["detected_at"] is not None for event in subset) / len(subset)
                             if subset else None),
        }
    return {
        "frames": len(rows), "confidence_threshold": confidence, "match_iou_threshold": minimum_iou,
        "event_gap_seconds": event_gap_seconds, "false_alarm_gap_seconds": false_alarm_gap_seconds,
        "events": len(events), "detected_events": len(detected),
        "event_recall": len(detected) / len(events) if events else None,
        "first_detection_latency_seconds": {
            "count": len(latency), "median": median(latency) if latency else None,
            "max": max(latency) if latency else None,
        },
        "false_alarm_events": alarms, "observed_duration_seconds": coverage,
        "false_alarm_events_per_hour": alarms / (coverage / 3600.0) if coverage else None,
        "per_class": per_class, "altitude_bands": altitude_metrics,
        "altitude_metrics_available": bool(altitude_metrics),
        "events_detail": events,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True,
                        help="converted data-set root containing manifest.jsonl and dataset.yaml")
    parser.add_argument("--split", choices=("train", "val"), default="val")
    parser.add_argument("--device", default="0")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.50)
    parser.add_argument("--event-gap-seconds", type=float, default=3.0)
    parser.add_argument("--false-alarm-gap-seconds", type=float, default=5.0)
    parser.add_argument("--altitude-bins", default="0,30,60,120",
                        help="lower edges in metres, comma separated")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 0.0 <= args.confidence <= 1.0 or not 0.0 < args.iou <= 1.0:
        parser.error("invalid confidence or IoU threshold")
    edges = [float(value) for value in args.altitude_bins.split(",") if value.strip()]
    if len(edges) < 2 or edges != sorted(set(edges)):
        parser.error("--altitude-bins needs at least two increasing edges")
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit("Install training/requirements.txt in the training environment") from error
    rows = read_manifest(args.dataset / "manifest.jsonl", args.split)
    names = read_names(args.dataset / "dataset.yaml")
    model = YOLO(str(args.model))
    summary = evaluate(rows, args.dataset, model, names, args.confidence, args.iou,
                       args.event_gap_seconds, args.false_alarm_gap_seconds, edges)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in summary.items() if key != "events_detail"}, indent=2))


if __name__ == "__main__":
    main()
