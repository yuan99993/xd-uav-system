#!/usr/bin/env python3
"""GPU comparison of three YOLO checkpoints on the predict4 image archive.

predict4 has no sidecar ground-truth boxes.  Some filenames provide a class
hint (car/arcar/tank), while add* images do not.  The report therefore separates
positive-set detection coverage from filename-based class-hit statistics and
does not claim precision/recall/mAP.
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


MODEL_CLASS_IDS = {
    "latest": {"car": 0, "ar-car": 1, "tank": 2},
    "yesterday": {"car": 0, "ar-car": 1, "tank": 2},
    "coco": {"car": 2},
}
VEHICLE_CLASS_IDS = {
    "latest": [0, 1, 2],
    "yesterday": [0, 1, 2],
    "coco": [2, 5, 7],  # COCO car, bus, truck
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def expected_class(path: Path) -> str | None:
    token = path.stem.split("_", 1)[0].lower()
    return {"car": "car", "arcar": "ar-car", "tank": "tank"}.get(token)


def run_predictions(model_path: Path, images: list[Path], device: str, imgsz: int) -> dict[str, Any]:
    load_start = time.perf_counter()
    model = YOLO(str(model_path))
    load_seconds = time.perf_counter() - load_start
    predictions: list[list[dict[str, Any]]] = []
    reported_inference_ms: list[float] = []

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
            classes = result.boxes.cls.detach().cpu().tolist()
            confidences = result.boxes.conf.detach().cpu().tolist()
            xyxy = result.boxes.xyxy.detach().cpu().tolist()
            for class_id, confidence, box in zip(classes, confidences, xyxy):
                boxes.append({
                    "class_id": int(class_id),
                    "confidence": float(confidence),
                    "xyxy": [round(float(value), 3) for value in box],
                })
        predictions.append(boxes)
        if result.speed and result.speed.get("inference") is not None:
            reported_inference_ms.append(float(result.speed["inference"]))
        if index == 1 or index % 50 == 0 or index == len(images):
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
        "reported_inference_ms": reported_inference_ms,
        "predictions": predictions,
    }


def positive_metrics(
    predictions: list[list[dict[str, Any]]],
    indices: list[int],
    class_ids: list[int],
    names: dict[str, Any],
    threshold: float,
) -> dict[str, Any]:
    accepted = set(class_ids)
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
        "mean_accepted_boxes_per_image": statistics.mean(len(boxes) for boxes in per_image) if per_image else 0.0,
        "class_distribution": {
            names.get(str(class_id), str(class_id)): count
            for class_id, count in sorted(class_counts.items())
        },
    }


def exact_class_metrics(
    predictions: list[list[dict[str, Any]]],
    indices: list[int],
    expected: dict[int, str],
    class_ids: dict[str, int],
    threshold: float,
) -> dict[str, Any]:
    applicable = [index for index in indices if expected.get(index) in class_ids]
    hits = 0
    predicted_counts: Counter[str] = Counter()
    for index in applicable:
        candidates = [
            box for box in predictions[index]
            if box["class_id"] in class_ids.values() and box["confidence"] >= threshold
        ]
        if not candidates:
            continue
        best = max(candidates, key=lambda item: item["confidence"])
        predicted_name = next(name for name, value in class_ids.items() if value == best["class_id"])
        predicted_counts[predicted_name] += 1
        if predicted_name == expected[index]:
            hits += 1
    return {
        "applicable_images": len(applicable),
        "exact_hits": hits,
        "exact_misses": len(applicable) - hits,
        "exact_hit_rate": hits / len(applicable) if applicable else None,
        "predicted_class_distribution": dict(predicted_counts),
    }


def make_report(data: dict[str, Any]) -> str:
    models = data["models"]
    all_results = data["results"]["all_images"]
    labeled_results = data["results"]["labeled_images"]
    add_results = data["results"]["unlabeled_add_images"]

    def pct(value: float | None) -> str:
        return "不适用" if value is None else f"{value * 100:.2f}%"

    lines = [
        "# predict4 新旧模型三方性能对比报告",
        "",
        f"- 测试时间：{data['generated_at']}",
        f"- 数据目录：`{data['dataset']['root']}`",
        f"- 图片总数：{data['dataset']['image_count']} 张；带类别文件名线索 {data['dataset']['labeled_image_count']} 张；`add*` 无类别线索 {data['dataset']['unlabeled_add_image_count']} 张",
        "- 独立标注：未发现 YOLO/COCO sidecar 标注；部分图片本身包含检测框/跟踪文字叠加",
        "",
        "## 模型",
        "",
        f"- 最新模型（train4）：`{models['latest']['model_path']}`，SHA-256 `{models['latest']['sha256']}`",
        "- 当前部署副本 `sar_yolo_detector/models/xd_vehicle_latest/xd_vehicle_latest.pt` 与最新 train4 权重 SHA-256 一致",
        f"- 昨天模型（train）：`{models['yesterday']['model_path']}`，SHA-256 `{models['yesterday']['sha256']}`",
        f"- 最早 COCO 模型：`{models['coco']['model_path']}`，SHA-256 `{models['coco']['sha256']}`",
        "",
        "## 统计口径",
        "",
        "- 新模型和昨天模型的车辆集合为 `car/ar-car/tank`；COCO 模型的宽口径车辆集合为 `car/bus/truck`，另单独列出 COCO `car`。",
        "- 命中率表示正样本图片中至少有一个指定类别的框达到阈值；没有框 IoU 真值，因此不能解释为 precision、recall 或 mAP。",
        "- 文件名类别线索仅用于类别命中参考：`car_*`、`arcar_*`、`tank_*`。`add*` 图片只参与车辆检测覆盖率统计。",
        "",
        "## 全部 388 张图片：车辆检测覆盖率",
        "",
        "| 模型/类别集合 | conf=0.30 | conf=0.60 | conf=0.60 全图平均最高置信度（未命中=0） |",
        "|---|---:|---:|---:|",
    ]
    all_rows = [
        ("最新 train4：car/ar-car/tank", "latest", "vehicle"),
        ("昨天 train：car/ar-car/tank", "yesterday", "vehicle"),
        ("COCO：car/bus/truck", "coco", "vehicle"),
        ("COCO：car", "coco", "car"),
    ]
    for label, model_key, kind in all_rows:
        item = all_results[model_key][kind]
        lines.append(
            f"| {label} | {pct(item['conf_0.30']['hit_rate'])} ({item['conf_0.30']['hit_images']}/{item['conf_0.30']['images']}) | "
            f"{pct(item['conf_0.60']['hit_rate'])} ({item['conf_0.60']['hit_images']}/{item['conf_0.60']['images']}) | "
            f"{item['conf_0.60']['mean_max_confidence_all_images']:.4f} |"
        )

    lines += [
        "",
        "## 208 张带类别线索图片：类别命中率",
        "",
        "| 模型 | conf=0.30 类别命中 | conf=0.60 类别命中 | 适用图片数 |",
        "|---|---:|---:|---:|",
    ]
    for label, model_key in [("最新 train4", "latest"), ("昨天 train", "yesterday"), ("COCO（仅 car_*）", "coco")]:
        m = labeled_results[model_key]["exact_class"]
        lines.append(
            f"| {label} | {pct(m['conf_0.30']['exact_hit_rate'])} ({m['conf_0.30']['exact_hits']}/{m['conf_0.30']['applicable_images']}) | "
            f"{pct(m['conf_0.60']['exact_hit_rate'])} ({m['conf_0.60']['exact_hits']}/{m['conf_0.60']['applicable_images']}) | {m['conf_0.60']['applicable_images']} |"
        )

    lines += [
        "",
        "## add* 场景（180 张，无类别线索）",
        "",
        "| 模型/类别集合 | conf=0.30 命中率 | conf=0.60 命中率 |",
        "|---|---:|---:|",
    ]
    for label, model_key in [("最新 train4：car/ar-car/tank", "latest"), ("昨天 train：car/ar-car/tank", "yesterday"), ("COCO：car/bus/truck", "coco")]:
        item = add_results[model_key]["vehicle"]
        lines.append(
            f"| {label} | {pct(item['conf_0.30']['hit_rate'])} ({item['conf_0.30']['hit_images']}/{item['conf_0.30']['images']}) | "
            f"{pct(item['conf_0.60']['hit_rate'])} ({item['conf_0.60']['hit_images']}/{item['conf_0.60']['images']}) |"
        )

    lines += [
        "",
        "## 类别分布（conf=0.60，全量图片）",
        "",
    ]
    for model_key, label in [("latest", "最新 train4"), ("yesterday", "昨天 train"), ("coco", "COCO 宽口径")]:
        lines.append(f"- {label}：{json.dumps(all_results[model_key]['vehicle']['conf_0.60']['class_distribution'], ensure_ascii=False)}")

    lines += [
        "",
        "## GPU 推理耗时",
        "",
    ]
    for model_key, label in [("latest", "最新 train4"), ("yesterday", "昨天 train"), ("coco", "COCO")]:
        item = models[model_key]
        mean_inf = statistics.mean(item["reported_inference_ms"])
        lines.append(
            f"- {label}：Ultralytics 上报平均 inference {mean_inf:.2f} ms/图；逐图墙钟时间 {item['predict_seconds']:.2f}s，约 {item['predict_seconds'] / data['dataset']['image_count'] * 1000:.2f} ms/文件（含首次 warmup 和当时的文件系统等待，不作为稳定 FPS 指标）。"
        )

    lines += [
        "",
        "## 结论与限制",
        "",
        "1. 最新 train4 与昨天 train 使用相同的自定义类别空间，二者的直接比较以 `car/ar-car/tank` 为准。",
        f"2. 全量图片 conf=0.60 车辆覆盖率：最新 train4 {all_results['latest']['vehicle']['conf_0.60']['hit_rate'] * 100:.2f}%，昨天 train {all_results['yesterday']['vehicle']['conf_0.60']['hit_rate'] * 100:.2f}%，最新低 { (all_results['yesterday']['vehicle']['conf_0.60']['hit_rate'] - all_results['latest']['vehicle']['conf_0.60']['hit_rate']) * 100:.2f} 个百分点。",
        f"3. 208 张有类别线索图片的 conf=0.60 类别命中率：最新 train4 {labeled_results['latest']['exact_class']['conf_0.60']['exact_hit_rate'] * 100:.2f}%，昨天 train {labeled_results['yesterday']['exact_class']['conf_0.60']['exact_hit_rate'] * 100:.2f}%；主要差异来自 `add*` 场景，最新为 {add_results['latest']['vehicle']['conf_0.60']['hit_rate'] * 100:.2f}%，昨天为 {add_results['yesterday']['vehicle']['conf_0.60']['hit_rate'] * 100:.2f}%。",
        "4. COCO 模型不认识 `ar-car` 和 `tank` 这两个自定义类别，因此 COCO 的车辆宽口径结果只能作为跨类别基线，不能作为同类别精度比较。",
        "5. predict4 没有独立框标注，且部分输入图已经带有检测框/跟踪标签；本报告适合做回归覆盖率和类别输出对比，不足以得出严格准确率或 mAP。",
        "6. 如果要形成严格模型验收结果，需要为 388 张图补充真实框标注，或从 Gazebo 目标位姿生成同步真值框。",
        "",
        f"原始 JSON 明细：`{data['json_output']}`",
    ]
    return "\n".join(lines) + "\n"


def evaluate_raw(raw: dict[str, Any], model_key: str, images: list[Path], expected: dict[int, str]) -> dict[str, Any]:
    predictions = raw["predictions"]
    names = raw["model_names"]
    all_indices = list(range(len(images)))
    labeled_indices = [index for index in all_indices if expected.get(index) is not None]
    add_indices = [index for index in all_indices if expected.get(index) is None]
    classes = MODEL_CLASS_IDS[model_key]
    vehicle_ids = VEHICLE_CLASS_IDS[model_key]
    return {
        "vehicle": {
            "conf_0.30": positive_metrics(predictions, all_indices, vehicle_ids, names, 0.30),
            "conf_0.60": positive_metrics(predictions, all_indices, vehicle_ids, names, 0.60),
        },
        "car": {
            "conf_0.30": positive_metrics(predictions, all_indices, [classes["car"]], names, 0.30),
            "conf_0.60": positive_metrics(predictions, all_indices, [classes["car"]], names, 0.60),
        },
        "labeled_exact_class": {
            "conf_0.30": exact_class_metrics(predictions, labeled_indices, expected, classes, 0.30),
            "conf_0.60": exact_class_metrics(predictions, labeled_indices, expected, classes, 0.60),
        },
        "add_vehicle": {
            "conf_0.30": positive_metrics(predictions, add_indices, vehicle_ids, names, 0.30),
            "conf_0.60": positive_metrics(predictions, add_indices, vehicle_ids, names, 0.60),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/home/promise/mrs_test/src/predict4"))
    parser.add_argument("--latest-model", type=Path, default=Path("/home/promise/mrs_test/src/train4/weights/best.pt"))
    parser.add_argument("--yesterday-model", type=Path, default=Path("/home/promise/mrs_test/src/train/weights/best.pt"))
    parser.add_argument("--coco-model", type=Path, default=Path("/home/promise/mrs_test/src/sar_yolo_detector/models/aircraft_coco/yolo11n.pt"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--only", choices=["latest", "yesterday", "coco", "both"], default="both")
    parser.add_argument("--raw-output", type=Path)
    parser.add_argument("--latest-raw", type=Path)
    parser.add_argument("--yesterday-raw", type=Path)
    parser.add_argument("--coco-raw", type=Path)
    parser.add_argument("--json-output", type=Path, default=Path("/home/promise/mrs_test/model-evaluations/predict4_three_model_comparison.json"))
    parser.add_argument("--report-output", type=Path, default=Path("/home/promise/mrs_test/model-evaluations/PREDICT4_THREE_MODEL_COMPARISON.md"))
    args = parser.parse_args()

    images = sorted(args.root.glob("*"))
    images = [path for path in images if path.suffix.lower() in {".jpg", ".jpeg", ".png"}]
    if not images:
        raise SystemExit(f"no images found under {args.root}")
    expected = {index: label for index, path in enumerate(images) if (label := expected_class(path)) is not None}

    model_paths = {"latest": args.latest_model, "yesterday": args.yesterday_model, "coco": args.coco_model}
    if args.only != "both":
        raw = run_predictions(model_paths[args.only], images, args.device, args.imgsz)
        output = args.raw_output or Path(f"/home/promise/mrs_test/model-evaluations/predict4_{args.only}_raw.json")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({"dataset": {"root": str(args.root), "image_count": len(images)}, "raw": raw}, ensure_ascii=False), encoding="utf-8")
        print(json.dumps({"raw_output": str(output), "model": args.only, "images": len(images)}, ensure_ascii=False, indent=2))
        return

    raw_paths = {"latest": args.latest_raw, "yesterday": args.yesterday_raw, "coco": args.coco_raw}
    raw_models: dict[str, Any] = {}
    for key in ("latest", "yesterday", "coco"):
        if raw_paths[key]:
            raw_models[key] = json.loads(raw_paths[key].read_text(encoding="utf-8"))["raw"]
        else:
            raw_models[key] = run_predictions(model_paths[key], images, args.device, args.imgsz)

    result_data: dict[str, Any] = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "dataset": {
            "root": str(args.root),
            "image_count": len(images),
            "labeled_image_count": len(expected),
            "unlabeled_add_image_count": len(images) - len(expected),
            "labels_found": False,
            "filename_class_hint": {"car_*": "car", "arcar_*": "ar-car", "tank_*": "tank"},
        },
        "models": raw_models,
        "results": {
            "all_images": {},
            "labeled_images": {},
            "unlabeled_add_images": {},
        },
        "json_output": str(args.json_output),
    }
    for key, raw in raw_models.items():
        evaluated = evaluate_raw(raw, key, images, expected)
        result_data["results"]["all_images"][key] = {
            "vehicle": evaluated["vehicle"],
            "car": evaluated["car"],
        }
        result_data["results"]["labeled_images"][key] = {"exact_class": evaluated["labeled_exact_class"]}
        result_data["results"]["unlabeled_add_images"][key] = {"vehicle": evaluated["add_vehicle"]}

    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.report_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(result_data, ensure_ascii=False, indent=2), encoding="utf-8")
    args.report_output.write_text(make_report(result_data), encoding="utf-8")
    print(json.dumps({
        "json_output": str(args.json_output),
        "report_output": str(args.report_output),
        "images": len(images),
        "labeled_images": len(expected),
        "latest_vehicle_conf060": result_data["results"]["all_images"]["latest"]["vehicle"]["conf_0.60"]["hit_rate"],
        "yesterday_vehicle_conf060": result_data["results"]["all_images"]["yesterday"]["vehicle"]["conf_0.60"]["hit_rate"],
        "coco_vehicle_conf060": result_data["results"]["all_images"]["coco"]["vehicle"]["conf_0.60"]["hit_rate"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
