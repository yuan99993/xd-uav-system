#!/usr/bin/env python3
"""GPU comparison of the latest, previous, and COCO models on car.rar."""

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


MODEL_CLASSES = {
    "latest": [0, 1, 2],
    "yesterday": [0, 1, 2],
    "coco": [2, 5, 7],
}
MODEL_CAR_CLASS = {"latest": [0], "yesterday": [0], "coco": [2]}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_hash(path: Path) -> str:
    return sha256(path)


def run_predictions(model_path: Path, images: list[Path], device: str, imgsz: int) -> dict[str, Any]:
    load_start = time.perf_counter()
    model = YOLO(str(model_path))
    load_seconds = time.perf_counter() - load_start
    predictions: list[list[dict[str, Any]]] = []
    inference_ms: list[float] = []
    predict_start = time.perf_counter()

    # One-image calls are intentional: car.rar contains variable-resolution
    # images and the 4GB GPU is more stable without a large variable batch.
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
        "reported_inference_ms": inference_ms,
        "predictions": predictions,
    }


def metrics(
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
    max_conf = [max((box["confidence"] for box in boxes), default=0.0) for boxes in per_image]
    counts = Counter(box["class_id"] for boxes in per_image for box in boxes)
    return {
        "images": len(indices),
        "hit_images": sum(bool(boxes) for boxes in per_image),
        "miss_images": sum(not boxes for boxes in per_image),
        "hit_rate": sum(bool(boxes) for boxes in per_image) / len(indices) if indices else 0.0,
        "mean_max_confidence_all_images": statistics.mean(max_conf) if max_conf else 0.0,
        "accepted_box_count": sum(len(boxes) for boxes in per_image),
        "mean_boxes_per_image": statistics.mean(len(boxes) for boxes in per_image) if per_image else 0.0,
        "class_distribution": {
            names.get(str(class_id), str(class_id)): count
            for class_id, count in sorted(counts.items())
        },
    }


def evaluate(raw: dict[str, Any], model_key: str, all_indices: list[int], unique_indices: list[int]) -> dict[str, Any]:
    names = raw["model_names"]
    predictions = raw["predictions"]
    output: dict[str, Any] = {}
    for label, indices in [("all_files", all_indices), ("unique_images", unique_indices)]:
        output[label] = {
            "vehicle": {
                "conf_0.30": metrics(predictions, indices, MODEL_CLASSES[model_key], names, 0.30),
                "conf_0.60": metrics(predictions, indices, MODEL_CLASSES[model_key], names, 0.60),
            },
            "car": {
                "conf_0.30": metrics(predictions, indices, MODEL_CAR_CLASS[model_key], names, 0.30),
                "conf_0.60": metrics(predictions, indices, MODEL_CAR_CLASS[model_key], names, 0.60),
            },
        }
    return output


def make_report(data: dict[str, Any]) -> str:
    results = data["results"]
    models = data["models"]

    def pct(value: float) -> str:
        return f"{value * 100:.2f}%"

    lines = [
        "# car.rar 三个识别模型性能对比报告",
        "",
        f"- 测试时间：{data['generated_at']}",
        f"- 数据目录：`{data['dataset']['root']}`",
        f"- 文件数：{data['dataset']['file_count']} 张；SHA-256 去重后 {data['dataset']['unique_image_count']} 个画面；重复组 {data['dataset']['duplicate_group_count']} 组",
        "- 标注情况：没有 YOLO/COCO 独立标注文件；部分图片本身带有检测框/跟踪文字，因此本报告统计的是正样本检测覆盖率，不是严格 mAP",
        "",
        "## 测试模型",
        "",
        f"- 最新 train4（当前部署模型）：`{models['latest']['model_path']}`，SHA-256 `{models['latest']['sha256']}`",
        f"- 昨天 train（回退备份模型）：`{models['yesterday']['model_path']}`，SHA-256 `{models['yesterday']['sha256']}`",
        f"- 最早 COCO：`{models['coco']['model_path']}`，SHA-256 `{models['coco']['sha256']}`",
        "",
        "## 全部 200 个文件",
        "",
        "命中率定义为：一张正样本图中，在对应类别集合内至少输出一个 conf 达到阈值的框。新模型使用 `car/ar-car/tank`；COCO 使用 `car/bus/truck`，并单独列出 COCO `car`。",
        "",
        "| 模型/类别集合 | conf=0.30 命中率 | conf=0.60 命中率 | conf=0.60 全图平均最高置信度（未命中=0） |",
        "|---|---:|---:|---:|",
    ]
    for label, key, kind in [
        ("最新 train4：car/ar-car/tank", "latest", "vehicle"),
        ("昨天 train：car/ar-car/tank", "yesterday", "vehicle"),
        ("COCO：car/bus/truck", "coco", "vehicle"),
        ("COCO：car", "coco", "car"),
    ]:
        low = results["all_files"][key][kind]["conf_0.30"]
        high = results["all_files"][key][kind]["conf_0.60"]
        lines.append(
            f"| {label} | {pct(low['hit_rate'])} ({low['hit_images']}/{low['images']}) | "
            f"{pct(high['hit_rate'])} ({high['hit_images']}/{high['images']}) | "
            f"{high['mean_max_confidence_all_images']:.4f} |"
        )

    lines += [
        "",
        "## 去重后 102 个画面",
        "",
        "| 模型/类别集合 | conf=0.30 命中率 | conf=0.60 命中率 |",
        "|---|---:|---:|",
    ]
    for label, key, kind in [
        ("最新 train4：car/ar-car/tank", "latest", "vehicle"),
        ("昨天 train：car/ar-car/tank", "yesterday", "vehicle"),
        ("COCO：car/bus/truck", "coco", "vehicle"),
        ("COCO：car", "coco", "car"),
    ]:
        low = results["unique_images"][key][kind]["conf_0.30"]
        high = results["unique_images"][key][kind]["conf_0.60"]
        lines.append(
            f"| {label} | {pct(low['hit_rate'])} ({low['hit_images']}/{low['images']}) | "
            f"{pct(high['hit_rate'])} ({high['hit_images']}/{high['images']}) |"
        )

    lines += ["", "## 类别分布（conf=0.60，全部文件）", ""]
    for key, label in [("latest", "最新 train4"), ("yesterday", "昨天 train"), ("coco", "COCO 宽口径")]:
        distribution = results["all_files"][key]["vehicle"]["conf_0.60"]["class_distribution"]
        lines.append(f"- {label}：{json.dumps(distribution, ensure_ascii=False)}")

    latest = results["all_files"]["latest"]["vehicle"]["conf_0.60"]["hit_rate"]
    yesterday = results["all_files"]["yesterday"]["vehicle"]["conf_0.60"]["hit_rate"]
    coco = results["all_files"]["coco"]["vehicle"]["conf_0.60"]["hit_rate"]
    lines += [
        "",
        "## 结论",
        "",
        f"- 最新 train4 相比昨天 train 的全量 conf=0.60 车辆覆盖率变化：{pct(latest)} 对 {pct(yesterday)}，相差 {(latest - yesterday) * 100:.2f} 个百分点。",
        f"- 最新 train4 相比 COCO 宽口径的全量 conf=0.60 车辆覆盖率高 {(latest - coco) * 100:.2f} 个百分点。",
        "- COCO 模型类别空间不同，不能与自定义 `ar-car/tank` 做严格同类精度比较。",
        "- 没有独立框标注，不能据此计算严格 precision、recall、IoU 或 mAP；图片中的已有叠加框也可能影响输入结果。",
        "",
        "## GPU 推理耗时",
        "",
    ]
    for key, label in [("latest", "最新 train4"), ("yesterday", "昨天 train"), ("coco", "COCO")]:
        item = models[key]
        lines.append(
            f"- {label}：平均 inference {statistics.mean(item['reported_inference_ms']):.2f} ms/图；逐图总耗时 {item['predict_seconds']:.2f}s（含首次 warmup）。"
        )
    lines += ["", f"原始 JSON 明细：`{data['json_output']}`", ""]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/home/promise/mrs_test/data/sar_yolo_detector/car"))
    parser.add_argument("--latest-model", type=Path, default=Path("/home/promise/mrs_test/src/sar_yolo_detector/models/xd_vehicle_latest/xd_vehicle_latest.pt"))
    parser.add_argument("--yesterday-model", type=Path, default=Path("/home/promise/mrs_test/src/sar_yolo_detector/models/xd_vehicle_previous/xd_vehicle_previous.pt"))
    parser.add_argument("--coco-model", type=Path, default=Path("/home/promise/mrs_test/src/sar_yolo_detector/models/aircraft_coco/yolo11n.pt"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--only", choices=["latest", "yesterday", "coco", "both"], default="both")
    parser.add_argument("--raw-output", type=Path)
    parser.add_argument("--latest-raw", type=Path)
    parser.add_argument("--yesterday-raw", type=Path)
    parser.add_argument("--coco-raw", type=Path)
    parser.add_argument("--json-output", type=Path, default=Path("/home/promise/mrs_test/model-evaluations/car_archive_three_model_comparison.json"))
    parser.add_argument("--report-output", type=Path, default=Path("/home/promise/mrs_test/model-evaluations/CAR_ARCHIVE_THREE_MODEL_COMPARISON.md"))
    args = parser.parse_args()

    images = sorted(path for path in args.root.rglob("*.png"))
    if not images:
        raise SystemExit(f"no PNG images found under {args.root}")
    hashes = [file_hash(path) for path in images]
    first_by_hash: dict[str, int] = {}
    unique_indices: list[int] = []
    for index, image_hash in enumerate(hashes):
        if image_hash not in first_by_hash:
            first_by_hash[image_hash] = index
            unique_indices.append(index)
    all_indices = list(range(len(images)))
    model_paths = {"latest": args.latest_model, "yesterday": args.yesterday_model, "coco": args.coco_model}

    if args.only != "both":
        raw = run_predictions(model_paths[args.only], images, args.device, args.imgsz)
        output = args.raw_output or Path(f"/home/promise/mrs_test/model-evaluations/car_archive_{args.only}_raw.json")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({"raw": raw}, ensure_ascii=False), encoding="utf-8")
        print(json.dumps({"raw_output": str(output), "model": args.only, "images": len(images)}, ensure_ascii=False, indent=2))
        return

    raw_paths = {"latest": args.latest_raw, "yesterday": args.yesterday_raw, "coco": args.coco_raw}
    raw_models = {}
    for key in ("latest", "yesterday", "coco"):
        if raw_paths[key]:
            raw_models[key] = json.loads(raw_paths[key].read_text(encoding="utf-8"))["raw"]
        else:
            raw_models[key] = run_predictions(model_paths[key], images, args.device, args.imgsz)

    data: dict[str, Any] = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "dataset": {
            "root": str(args.root),
            "file_count": len(images),
            "unique_image_count": len(unique_indices),
            "duplicate_group_count": sum(count > 1 for count in Counter(hashes).values()),
            "labels_found": False,
        },
        "models": raw_models,
        "results": {"all_files": {}, "unique_images": {}},
        "json_output": str(args.json_output),
    }
    for key, raw in raw_models.items():
        evaluated = evaluate(raw, key, all_indices, unique_indices)
        data["results"]["all_files"][key] = evaluated["all_files"]
        data["results"]["unique_images"][key] = evaluated["unique_images"]

    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.report_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    args.report_output.write_text(make_report(data), encoding="utf-8")
    print(json.dumps({
        "json_output": str(args.json_output),
        "report_output": str(args.report_output),
        "images": len(images),
        "unique_images": len(unique_indices),
        "latest_conf060": data["results"]["all_files"]["latest"]["vehicle"]["conf_0.60"]["hit_rate"],
        "yesterday_conf060": data["results"]["all_files"]["yesterday"]["vehicle"]["conf_0.60"]["hit_rate"],
        "coco_conf060": data["results"]["all_files"]["coco"]["vehicle"]["conf_0.60"]["hit_rate"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
