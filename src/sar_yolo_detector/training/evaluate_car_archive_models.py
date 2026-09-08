#!/usr/bin/env python3
"""Compare the old and new detector on the extracted car image archive.

The archive contains rendered positive images but no ground-truth annotations.
Consequently this script reports positive-set detection hit rate and confidence
statistics; it deliberately does not present those values as precision, recall,
or mAP.
"""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
import statistics
import time
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

from ultralytics import YOLO
import torch


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    values = sorted(values)
    position = (len(values) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def run_predictions(model_path: Path, images: list[Path], device: str, imgsz: int, batch: int, source_conf: float) -> dict[str, Any]:
    load_start = time.perf_counter()
    model = YOLO(str(model_path))
    load_seconds = time.perf_counter() - load_start

    predict_start = time.perf_counter()
    # Process one image per predict call.  On this 4GB laptop GPU, passing the
    # whole variable-resolution archive as one source makes Ultralytics build a
    # large warmup tensor and intermittently trips a CUDA driver error.  The
    # model remains loaded on GPU, so subsequent calls are still fast.
    results = []
    for index, image in enumerate(images, start=1):
        result = model.predict(
            source=str(image),
            imgsz=imgsz,
            conf=source_conf,
            iou=0.7,
            max_det=100,
            device=device,
            batch=1,
            verbose=False,
        )[0]
        results.append(result)
        if index == 1 or index % 25 == 0 or index == len(images):
            print(f"{model_path.name}: {index}/{len(images)}", flush=True)
    predict_seconds = time.perf_counter() - predict_start

    predictions: list[list[dict[str, Any]]] = []
    reported_inference_ms: list[float] = []
    for result in results:
        boxes: list[dict[str, Any]] = []
        if result.boxes is not None:
            classes = result.boxes.cls.detach().cpu().tolist()
            confidences = result.boxes.conf.detach().cpu().tolist()
            xyxy = result.boxes.xyxy.detach().cpu().tolist()
            for class_id, confidence, box in zip(classes, confidences, xyxy):
                boxes.append(
                    {
                        "class_id": int(class_id),
                        "confidence": float(confidence),
                        "xyxy": [round(float(value), 3) for value in box],
                    }
                )
        predictions.append(boxes)
        if result.speed and result.speed.get("inference") is not None:
            reported_inference_ms.append(float(result.speed["inference"]))

    names = model.names
    if isinstance(names, list):
        names = {str(index): value for index, value in enumerate(names)}
    else:
        names = {str(key): value for key, value in names.items()}

    # The two models are evaluated in one process.  Release the first model's
    # CUDA allocations before the second model is loaded on a 4GB GPU.
    del model
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    return {
        "model_path": str(model_path),
        "model_names": names,
        "load_seconds": load_seconds,
        "predict_seconds": predict_seconds,
        "predictions": predictions,
        "reported_inference_ms": reported_inference_ms,
    }


def threshold_metrics(
    predictions: list[list[dict[str, Any]]],
    class_ids: Iterable[int],
    threshold: float,
    names: dict[str, Any],
) -> dict[str, Any]:
    accepted_ids = set(class_ids)
    per_image: list[list[dict[str, Any]]] = []
    for image_predictions in predictions:
        per_image.append(
            [
                box
                for box in image_predictions
                if box["class_id"] in accepted_ids and box["confidence"] >= threshold
            ]
        )

    max_confidences = [max((box["confidence"] for box in boxes), default=0.0) for boxes in per_image]
    class_counts = Counter(box["class_id"] for boxes in per_image for box in boxes)
    return {
        "threshold": threshold,
        "accepted_class_ids": sorted(accepted_ids),
        "accepted_class_names": [names.get(str(class_id), str(class_id)) for class_id in sorted(accepted_ids)],
        "images": len(per_image),
        "hit_images": sum(bool(boxes) for boxes in per_image),
        "miss_images": sum(not boxes for boxes in per_image),
        "hit_rate": sum(bool(boxes) for boxes in per_image) / len(per_image) if per_image else 0.0,
        "mean_max_confidence": statistics.mean(max_confidences) if max_confidences else 0.0,
        "median_max_confidence": statistics.median(max_confidences) if max_confidences else 0.0,
        "p10_max_confidence": percentile(max_confidences, 0.10),
        "mean_accepted_boxes_per_image": (
            statistics.mean(len(boxes) for boxes in per_image) if per_image else 0.0
        ),
        "accepted_box_count": sum(len(boxes) for boxes in per_image),
        "class_distribution": {
            names.get(str(class_id), str(class_id)): count
            for class_id, count in sorted(class_counts.items())
        },
    }


def evaluate_set(
    predictions: list[list[dict[str, Any]]],
    names: dict[str, Any],
    image_indices: list[int],
) -> dict[str, Any]:
    subset = [predictions[index] for index in image_indices]
    definitions = {
        "strict_car": [0],
        "custom_vehicle": [0, 1, 2],
        "old_coco_car": [2],
        "old_coco_broad_vehicle": [2, 5, 7],
        "all_model_classes": sorted({box["class_id"] for image in subset for box in image}),
    }
    return {
        key: {
            "at_conf_0.30": threshold_metrics(subset, ids, 0.30, names),
            "at_conf_0.60": threshold_metrics(subset, ids, 0.60, names),
        }
        for key, ids in definitions.items()
    }


def fmt_percent(value: float) -> str:
    return f"{value * 100:.2f}%"


def fmt_float(value: float) -> str:
    return f"{value:.4f}"


def make_report(data: dict[str, Any]) -> str:
    old = data["models"]["old"]
    new = data["models"]["new"]
    lines = [
        "# car.rar 新旧识别模型测试报告",
        "",
        f"- 测试时间：{data['generated_at']}",
        f"- 数据目录：`{data['dataset']['root']}`",
        f"- 图片文件：{data['dataset']['file_count']} 张；按 SHA-256 去重后 {data['dataset']['unique_image_count']} 个画面",
        f"- 重复组：{data['dataset']['duplicate_group_count']} 组（重复帧仍保留在归档中）",
        "- 标注情况：未发现 YOLO/COCO 标注文件；PNG 全部为不透明 RGBA 图像",
        "",
        "## 测试方法",
        "",
        "对归档内图片使用相同的 `imgsz=640`、GPU 推理和 `conf=0.30` 候选框输出，再在置信度 0.30 和 0.60 处统计。由于数据集没有人工框标注，本报告的‘命中率’定义为：含车正样本中，模型在指定类别集合内至少输出一个达到阈值的框。它可作为正样本检测覆盖率参考，但不能等同于 precision、recall 或 mAP，也无法判断框的 IoU 是否正确。",
        "",
        "类别口径：旧模型使用 COCO `car=2`；新模型使用 `car=0`，同时额外报告新模型自定义车辆类 `car/ar-car/tank=0/1/2`，以避免装甲车辆被新模型判为 `tank` 时被误算为漏检。",
        "",
        "## 结果（全部 200 个文件）",
        "",
        "| 口径 | conf=0.30 命中率 | conf=0.60 命中率 | conf=0.60 全图平均最高置信度（未命中=0） | conf=0.60 平均框数/图 |",
        "|---|---:|---:|---:|---:|",
    ]
    all_results = data["results"]["all_files"]
    rows = [
        ("旧模型：COCO car", "old_coco_car"),
        ("旧模型：COCO car/bus/truck", "old_coco_broad_vehicle"),
        ("新模型：car/ar-car/tank", "custom_vehicle"),
    ]
    for label, key in rows:
        metrics = all_results[key]
        low = metrics["at_conf_0.30"]
        high = metrics["at_conf_0.60"]
        lines.append(
            f"| {label} | {fmt_percent(low['hit_rate'])} ({low['hit_images']}/{low['images']}) | "
            f"{fmt_percent(high['hit_rate'])} ({high['hit_images']}/{high['images']}) | "
            f"{fmt_float(high['mean_max_confidence'])} | {fmt_float(high['mean_accepted_boxes_per_image'])} |"
        )

    lines += [
        "",
        "## 去重画面结果（102 个不同画面）",
        "",
        "| 口径 | conf=0.30 命中率 | conf=0.60 命中率 | conf=0.60 全图平均最高置信度（未命中=0） |",
        "|---|---:|---:|---:|",
    ]
    unique_results = data["results"]["unique_images"]
    for label, key in rows:
        low = unique_results[key]["at_conf_0.30"]
        high = unique_results[key]["at_conf_0.60"]
        lines.append(
            f"| {label} | {fmt_percent(low['hit_rate'])} ({low['hit_images']}/{low['images']}) | "
            f"{fmt_percent(high['hit_rate'])} ({high['hit_images']}/{high['images']}) | "
            f"{fmt_float(high['mean_max_confidence'])} |"
        )

    lines += [
        "",
        "## 类别分布（conf=0.60，全部文件）",
        "",
        f"- 旧模型（COCO car）：{json.dumps(all_results['old_coco_car']['at_conf_0.60']['class_distribution'], ensure_ascii=False)}",
        f"- 新模型（自定义车辆类）：{json.dumps(all_results['custom_vehicle']['at_conf_0.60']['class_distribution'], ensure_ascii=False)}",
        "",
        "## 推理耗时",
        "",
        f"- 旧模型：加载 {old['load_seconds']:.2f}s；逐图预测墙钟时间 {old['predict_seconds']:.2f}s，约 {old['predict_seconds'] / data['dataset']['file_count'] * 1000:.2f}ms/文件；Ultralytics 上报平均 inference {statistics.mean(old['reported_inference_ms']):.2f}ms/图。",
        f"- 新模型：加载 {new['load_seconds']:.2f}s；逐图预测墙钟时间 {new['predict_seconds']:.2f}s，约 {new['predict_seconds'] / data['dataset']['file_count'] * 1000:.2f}ms/文件；Ultralytics 上报平均 inference {statistics.mean(new['reported_inference_ms']):.2f}ms/图。",
        "",
        "## 结论与限制",
        "",
        "1. 该压缩包适合做无标注正样本覆盖率/可视化回归测试；当前不能据此给出严格的检测准确率、误检率、召回率或 mAP。",
        "2. 判断新模型时应优先看 `car/ar-car/tank` 汇总，而不能只看严格 `car`，因为新模型的类别体系包含装甲车和坦克类别。",
        "3. 若要得到可发表或可用于模型替换决策的 precision/recall/mAP，需要为这 102 个不同画面补充目标框标注，或从 Gazebo 生成同步的目标位姿/投影框作为自动真值。",
        "",
        f"原始 JSON 明细：`{data['json_output']}`",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/home/promise/mrs_test/data/sar_yolo_detector/car"))
    parser.add_argument("--old-model", type=Path, default=Path("/home/promise/mrs_test/src/sar_yolo_detector/models/aircraft_coco/yolo11n.pt"))
    parser.add_argument("--new-model", type=Path, default=Path("/home/promise/mrs_test/src/train/weights/best.pt"))
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--source-conf", type=float, default=0.30)
    parser.add_argument("--only", choices=["both", "old", "new"], default="both")
    parser.add_argument("--raw-output", type=Path, help="write one-model predictions for a separate-process run")
    parser.add_argument("--old-raw", type=Path, help="read old-model predictions from a separate process")
    parser.add_argument("--new-raw", type=Path, help="read new-model predictions from a separate process")
    parser.add_argument("--json-output", type=Path, default=Path("/home/promise/mrs_test/model-evaluations/car_archive_model_comparison.json"))
    parser.add_argument("--report-output", type=Path, default=Path("/home/promise/mrs_test/model-evaluations/CAR_ARCHIVE_MODEL_TEST_REPORT.md"))
    args = parser.parse_args()

    images = sorted(args.root.rglob("*.png"))
    if not images:
        raise SystemExit(f"no PNG images found under {args.root}")
    hashes = [sha256(path) for path in images]
    first_for_hash: dict[str, int] = {}
    unique_indices: list[int] = []
    for index, image_hash in enumerate(hashes):
        if image_hash not in first_for_hash:
            first_for_hash[image_hash] = index
            unique_indices.append(index)

    if args.only != "both":
        model_path = args.old_model if args.only == "old" else args.new_model
        raw = run_predictions(model_path, images, args.device, args.imgsz, args.batch, args.source_conf)
        raw_output = args.raw_output or Path(f"/home/promise/mrs_test/model-evaluations/car_archive_{args.only}_raw.json")
        raw_output.parent.mkdir(parents=True, exist_ok=True)
        raw_output.write_text(json.dumps({
            "dataset": {
                "root": str(args.root),
                "file_count": len(images),
                "unique_image_count": len(unique_indices),
                "hashes": hashes,
                "unique_indices": unique_indices,
            },
            "raw": raw,
        }, ensure_ascii=False), encoding="utf-8")
        print(json.dumps({"raw_output": str(raw_output), "model": args.only, "images": len(images)}, ensure_ascii=False, indent=2))
        return

    def load_raw(path: Path) -> dict[str, Any]:
        return json.loads(path.read_text(encoding="utf-8"))["raw"]

    old_raw = load_raw(args.old_raw) if args.old_raw else run_predictions(args.old_model, images, args.device, args.imgsz, args.batch, args.source_conf)
    new_raw = load_raw(args.new_raw) if args.new_raw else run_predictions(args.new_model, images, args.device, args.imgsz, args.batch, args.source_conf)

    old_results = evaluate_set(old_raw["predictions"], old_raw["model_names"], list(range(len(images))))
    new_results = evaluate_set(new_raw["predictions"], new_raw["model_names"], list(range(len(images))))
    old_unique = evaluate_set(old_raw["predictions"], old_raw["model_names"], unique_indices)
    new_unique = evaluate_set(new_raw["predictions"], new_raw["model_names"], unique_indices)

    # Keep the report-oriented aliases explicit so that readers can see which
    # class set belongs to which model.
    data: dict[str, Any] = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "dataset": {
            "root": str(args.root),
            "file_count": len(images),
            "unique_image_count": len(unique_indices),
            "duplicate_group_count": sum(1 for count in Counter(hashes).values() if count > 1),
            "duplicate_file_count": len(images) - len(unique_indices),
            "labels_found": False,
            "all_png_alpha_opaque": True,
        },
        "models": {
            "old": {
                "model_path": old_raw["model_path"],
                "results": old_results,
                "reported_inference_ms": old_raw["reported_inference_ms"],
                "load_seconds": old_raw["load_seconds"],
                "predict_seconds": old_raw["predict_seconds"],
            },
            "new": {
                "model_path": new_raw["model_path"],
                "results": new_results,
                "reported_inference_ms": new_raw["reported_inference_ms"],
                "load_seconds": new_raw["load_seconds"],
                "predict_seconds": new_raw["predict_seconds"],
            },
        },
        "results": {
            "all_files": {
                "old_coco_car": old_results["old_coco_car"],
                "old_coco_broad_vehicle": old_results["old_coco_broad_vehicle"],
                "strict_car": new_results["strict_car"],
                "custom_vehicle": new_results["custom_vehicle"],
            },
            "unique_images": {
                "old_coco_car": old_unique["old_coco_car"],
                "old_coco_broad_vehicle": old_unique["old_coco_broad_vehicle"],
                "strict_car": new_unique["strict_car"],
                "custom_vehicle": new_unique["custom_vehicle"],
            },
        },
        "json_output": str(args.json_output),
    }
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.report_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    args.report_output.write_text(make_report(data), encoding="utf-8")
    print(json.dumps({
        "json_output": str(args.json_output),
        "report_output": str(args.report_output),
        "file_count": len(images),
        "unique_image_count": len(unique_indices),
        "old_coco_car_conf030": old_results["old_coco_car"]["at_conf_0.30"]["hit_rate"],
        "old_coco_car_conf060": old_results["old_coco_car"]["at_conf_0.60"]["hit_rate"],
        "new_custom_vehicle_conf030": new_results["custom_vehicle"]["at_conf_0.30"]["hit_rate"],
        "new_custom_vehicle_conf060": new_results["custom_vehicle"]["at_conf_0.60"]["hit_rate"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
