#!/usr/bin/env python3
"""Compare the train7 and pre-train7 XD vehicle checkpoints on predict7.

predict7 contains images but no independent YOLO/COCO sidecar annotations, so
the report intentionally measures positive-set coverage and runtime regression
metrics rather than precision, recall, IoU, or mAP.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import time
from collections import Counter
from pathlib import Path
from typing import Any

from ultralytics import YOLO


CLASS_IDS = [0, 1, 2]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def image_hash(path: Path) -> str:
    return sha256(path)


def duplicate_summary(images: list[Path]) -> dict[str, Any]:
    groups: dict[str, list[int]] = {}
    for index, image in enumerate(images):
        groups.setdefault(image_hash(image), []).append(index)
    duplicate_groups = [indices for indices in groups.values() if len(indices) > 1]
    unique_indices = [indices[0] for indices in groups.values()]
    return {
        "unique_image_count": len(groups),
        "duplicate_group_count": len(duplicate_groups),
        "duplicate_file_count": sum(len(indices) - 1 for indices in duplicate_groups),
        "unique_indices": sorted(unique_indices),
    }


def run_predictions(model_path: Path, images: list[Path], device: str, imgsz: int) -> dict[str, Any]:
    load_start = time.perf_counter()
    model = YOLO(str(model_path))
    load_seconds = time.perf_counter() - load_start
    predictions: list[list[dict[str, Any]]] = []
    inference_ms: list[float] = []
    predict_start = time.perf_counter()
    for index, image in enumerate(images, start=1):
        result = model.predict(
            source=str(image),
            imgsz=imgsz,
            conf=0.30,
            iou=0.7,
            max_det=100,
            device=device,
            batch=1,
            verbose=False,
        )[0]
        boxes: list[dict[str, Any]] = []
        if result.boxes is not None:
            for class_id, confidence, box in zip(
                result.boxes.cls.detach().cpu().tolist(),
                result.boxes.conf.detach().cpu().tolist(),
                result.boxes.xyxy.detach().cpu().tolist(),
            ):
                boxes.append({
                    "class_id": int(class_id),
                    "confidence": float(confidence),
                    "xyxy": [round(float(value), 3) for value in box],
                })
        predictions.append(boxes)
        if result.speed and result.speed.get("inference") is not None:
            inference_ms.append(float(result.speed["inference"]))
        if index == 1 or index % 10 == 0 or index == len(images):
            print(f"{model_path.name}: {index}/{len(images)}", flush=True)

    names = model.names
    if isinstance(names, list):
        names = {str(index): value for index, value in enumerate(names)}
    else:
        names = {str(key): value for key, value in names.items()}
    return {
        "model_path": str(model_path),
        "sha256": sha256(model_path),
        "model_names": names,
        "load_seconds": load_seconds,
        "predict_seconds": time.perf_counter() - predict_start,
        "reported_inference_ms": inference_ms,
        "predictions": predictions,
    }


def metrics(raw: dict[str, Any], indices: list[int], threshold: float) -> dict[str, Any]:
    names = raw["model_names"]
    predictions = raw["predictions"]
    accepted = set(CLASS_IDS)
    per_image = [
        [box for box in predictions[index] if box["class_id"] in accepted and box["confidence"] >= threshold]
        for index in indices
    ]
    max_confidences = [max((box["confidence"] for box in boxes), default=0.0) for boxes in per_image]
    class_counts = Counter(box["class_id"] for boxes in per_image for box in boxes)
    return {
        "images": len(indices),
        "hit_images": sum(bool(boxes) for boxes in per_image),
        "miss_images": sum(not boxes for boxes in per_image),
        "hit_rate": sum(bool(boxes) for boxes in per_image) / len(indices) if indices else 0.0,
        "mean_max_confidence_all_images": statistics.mean(max_confidences) if max_confidences else 0.0,
        "accepted_box_count": sum(len(boxes) for boxes in per_image),
        "mean_boxes_per_image": statistics.mean(len(boxes) for boxes in per_image) if per_image else 0.0,
        "class_distribution": {
            names.get(str(class_id), str(class_id)): count
            for class_id, count in sorted(class_counts.items())
        },
    }


def evaluate(raw: dict[str, Any], indices: list[int]) -> dict[str, Any]:
    return {
        "conf_0.30": metrics(raw, indices, 0.30),
        "conf_0.60": metrics(raw, indices, 0.60),
    }


def pct(value: float) -> str:
    return f"{value * 100:.2f}%"


def make_report(data: dict[str, Any]) -> str:
    models = data["models"]
    results = data["results"]
    dataset_name = Path(data["dataset"]["root"]).name
    new = results["new_model"]["all_images"]
    old = results["old_model"]["all_images"]
    new_unique = results["new_model"]["unique_images"]
    old_unique = results["old_model"]["unique_images"]

    lines = [
        f"# {dataset_name} 新旧 YOLO 识别模型性能对比报告",
        "",
        f"- 测试时间：{data['generated_at']}",
        f"- 数据目录：`{data['dataset']['root']}`",
        f"- 图片数量：{data['dataset']['image_count']} 张；SHA-256 去重后 {data['dataset']['unique_image_count']} 个画面；重复组 {data['dataset']['duplicate_group_count']} 组",
        "- 标注情况：未发现独立 YOLO/COCO sidecar 标注；以下是正样本检测覆盖率与运行性能对比，不是严格准确率或 mAP",
        "",
        "## 测试模型",
        "",
        f"- 新模型（train7）：`{models['new_model']['model_path']}`，SHA-256 `{models['new_model']['sha256']}`",
        f"- 旧模型（train7 切换前备份）：`{models['old_model']['model_path']}`，SHA-256 `{models['old_model']['sha256']}`",
        "- 两个模型均按自定义车辆类别 ID `0/1/2`（car/ar-car/tank）统计。",
        "",
        "## 全部图片：车辆检测覆盖率",
        "",
        "命中率定义：一张图片中至少输出一个类别 ID 为 0、1 或 2 且置信度达到阈值的框。",
        "",
        "| 模型 | conf=0.30 | conf=0.60 | conf=0.60 全图平均最高置信度 | 平均框数/图（conf=0.60） |",
        "|---|---:|---:|---:|---:|",
    ]
    for label, item in [("新模型 train7", new), ("旧模型", old)]:
        low = item["conf_0.30"]
        high = item["conf_0.60"]
        lines.append(
            f"| {label} | {pct(low['hit_rate'])} ({low['hit_images']}/{low['images']}) | "
            f"{pct(high['hit_rate'])} ({high['hit_images']}/{high['images']}) | "
            f"{high['mean_max_confidence_all_images']:.4f} | {high['mean_boxes_per_image']:.2f} |"
        )

    lines += [
        "",
        "## 去重后画面：车辆检测覆盖率",
        "",
        "| 模型 | conf=0.30 | conf=0.60 |",
        "|---|---:|---:|",
    ]
    for label, item in [("新模型 train7", new_unique), ("旧模型", old_unique)]:
        low = item["conf_0.30"]
        high = item["conf_0.60"]
        lines.append(
            f"| {label} | {pct(low['hit_rate'])} ({low['hit_images']}/{low['images']}) | "
            f"{pct(high['hit_rate'])} ({high['hit_images']}/{high['images']}) |"
        )

    lines += ["", "## 类别分布（conf=0.60，全量图片）", ""]
    lines.append(f"- 新模型 train7：{json.dumps(new['conf_0.60']['class_distribution'], ensure_ascii=False)}")
    lines.append(f"- 旧模型：{json.dumps(old['conf_0.60']['class_distribution'], ensure_ascii=False)}")

    new_rate = new["conf_0.60"]["hit_rate"]
    old_rate = old["conf_0.60"]["hit_rate"]
    new_conf = new["conf_0.60"]["mean_max_confidence_all_images"]
    old_conf = old["conf_0.60"]["mean_max_confidence_all_images"]
    lines += [
        "",
        "## GPU 推理耗时",
        "",
        f"- 新模型 train7：Ultralytics 上报平均 inference {statistics.mean(models['new_model']['reported_inference_ms']):.2f} ms/图；逐图总耗时 {models['new_model']['predict_seconds']:.2f}s。",
        f"- 旧模型：Ultralytics 上报平均 inference {statistics.mean(models['old_model']['reported_inference_ms']):.2f} ms/图；逐图总耗时 {models['old_model']['predict_seconds']:.2f}s。",
        "",
        "## 结论与限制",
        "",
        f"- conf=0.60 全量覆盖率：新模型 {pct(new_rate)}，旧模型 {pct(old_rate)}，新模型相差 {(new_rate - old_rate) * 100:.2f} 个百分点。",
        f"- conf=0.60 平均最高置信度：新模型 {new_conf:.4f}，旧模型 {old_conf:.4f}，差值 {(new_conf - old_conf):.4f}。",
        "- predict7 没有独立框标注，且图片可能包含场景叠加信息；因此本报告不能证明真实 precision、recall、IoU 或 mAP，只能用于同一数据集上的回归对比。",
        "- 新模型已部署到 `sar_yolo_detector/models/xd_vehicle_latest/xd_vehicle_latest.pt`；切换前旧模型保存在 `models/xd_vehicle_previous/xd_vehicle_before_train7_20260909.pt`。",
        "",
        f"原始 JSON 明细：`{data['json_output']}`",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/home/promise/mrs_test/src/predict7"))
    parser.add_argument("--new-model", type=Path, default=Path("/home/promise/mrs_test/src/train7/weights/best.pt"))
    parser.add_argument("--old-model", type=Path, default=Path("/home/promise/mrs_test/src/sar_yolo_detector/models/xd_vehicle_previous/xd_vehicle_before_train7_20260909.pt"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--json-output", type=Path, default=Path("/home/promise/mrs_test/model-evaluations/predict7_two_model_comparison.json"))
    parser.add_argument("--report-output", type=Path, default=Path("/home/promise/mrs_test/model-evaluations/PREDICT7_TWO_MODEL_COMPARISON.md"))
    args = parser.parse_args()

    images = sorted(path for path in args.root.rglob("*") if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        raise SystemExit(f"no images found under {args.root}")
    duplicates = duplicate_summary(images)
    raw_models = {
        "new_model": run_predictions(args.new_model, images, args.device, args.imgsz),
        "old_model": run_predictions(args.old_model, images, args.device, args.imgsz),
    }
    unique_indices = duplicates["unique_indices"]
    result_data: dict[str, Any] = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "dataset": {
            "root": str(args.root),
            "image_count": len(images),
            "unique_image_count": duplicates["unique_image_count"],
            "duplicate_group_count": duplicates["duplicate_group_count"],
            "duplicate_file_count": duplicates["duplicate_file_count"],
            "labels_found": False,
        },
        "models": raw_models,
        "results": {
            "new_model": {
                "all_images": evaluate(raw_models["new_model"], list(range(len(images)))),
                "unique_images": evaluate(raw_models["new_model"], unique_indices),
            },
            "old_model": {
                "all_images": evaluate(raw_models["old_model"], list(range(len(images)))),
                "unique_images": evaluate(raw_models["old_model"], unique_indices),
            },
        },
        "json_output": str(args.json_output),
    }
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.report_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(result_data, ensure_ascii=False, indent=2), encoding="utf-8")
    args.report_output.write_text(make_report(result_data), encoding="utf-8")
    print(json.dumps({
        "json_output": str(args.json_output),
        "report_output": str(args.report_output),
        "images": len(images),
        "unique_images": duplicates["unique_image_count"],
        "new_conf060": result_data["results"]["new_model"]["all_images"]["conf_0.60"]["hit_rate"],
        "old_conf060": result_data["results"]["old_model"]["all_images"]["conf_0.60"]["hit_rate"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
