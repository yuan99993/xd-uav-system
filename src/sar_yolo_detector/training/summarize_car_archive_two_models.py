#!/usr/bin/env python3
"""Summarize two already-computed car archive YOLO prediction files."""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import time
from collections import Counter
from pathlib import Path
from typing import Any


CLASS_IDS = [0, 1, 2]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
    new = data["results"]["new_model"]
    old = data["results"]["old_model"]
    new_all = new["all_images"]
    old_all = old["all_images"]
    new_unique = new["unique_images"]
    old_unique = old["unique_images"]
    new_rate = new_all["conf_0.60"]["hit_rate"]
    old_rate = old_all["conf_0.60"]["hit_rate"]
    lines = [
        "# car.rar train7 新旧模型性能对比报告",
        "",
        f"- 测试时间：{data['generated_at']}",
        f"- 数据目录：`{data['dataset']['root']}`；共 {data['dataset']['image_count']} 张 PNG，去重后 {data['dataset']['unique_image_count']} 个画面",
        "- 测试设备：NVIDIA GeForce RTX 3050 Laptop GPU，`cuda:0`；每次只加载一个模型、逐图推理",
        "- 标注情况：没有独立 YOLO/COCO 标注；结果是正样本检测覆盖率，不等同于严格 precision、recall 或 mAP",
        "",
        "## 模型",
        "",
        f"- 新模型 train7：`{data['models']['new_model']['model_path']}`，SHA-256 `{data['models']['new_model']['sha256']}`",
        f"- 旧模型：`{data['models']['old_model']['model_path']}`，SHA-256 `{data['models']['old_model']['sha256']}`",
        "- 车辆类别集合：`car/ar-car/tank`（类别 ID `0/1/2`）。",
        "",
        "## 全部 200 张图片",
        "",
        "| 模型 | conf=0.30 覆盖率 | conf=0.60 覆盖率 | conf=0.60 平均最高置信度 | conf=0.60 平均框数/图 |",
        "|---|---:|---:|---:|---:|",
    ]
    for label, item in [("新模型 train7", new_all), ("旧模型", old_all)]:
        low = item["conf_0.30"]
        high = item["conf_0.60"]
        lines.append(
            f"| {label} | {pct(low['hit_rate'])} ({low['hit_images']}/{low['images']}) | "
            f"{pct(high['hit_rate'])} ({high['hit_images']}/{high['images']}) | "
            f"{high['mean_max_confidence_all_images']:.4f} | {high['mean_boxes_per_image']:.2f} |"
        )
    lines += [
        "",
        "## 去重后画面",
        "",
        "| 模型 | conf=0.30 覆盖率 | conf=0.60 覆盖率 |",
        "|---|---:|---:|",
    ]
    for label, item in [("新模型 train7", new_unique), ("旧模型", old_unique)]:
        low = item["conf_0.30"]
        high = item["conf_0.60"]
        lines.append(
            f"| {label} | {pct(low['hit_rate'])} ({low['hit_images']}/{low['images']}) | "
            f"{pct(high['hit_rate'])} ({high['hit_images']}/{high['images']}) |"
        )
    lines += [
        "",
        "## GPU 推理耗时",
        "",
        f"- 新模型 train7：平均 inference {statistics.mean(data['models']['new_model']['reported_inference_ms']):.2f} ms/图；逐图总耗时 {data['models']['new_model']['predict_seconds']:.2f}s。",
        f"- 旧模型：平均 inference {statistics.mean(data['models']['old_model']['reported_inference_ms']):.2f} ms/图；逐图总耗时 {data['models']['old_model']['predict_seconds']:.2f}s。",
        "",
        "## 类别分布（conf=0.60，全量图片）",
        "",
        f"- 新模型 train7：{json.dumps(new_all['conf_0.60']['class_distribution'], ensure_ascii=False)}",
        f"- 旧模型：{json.dumps(old_all['conf_0.60']['class_distribution'], ensure_ascii=False)}",
        "",
        "## 结论",
        "",
        f"- conf=0.60 全量覆盖率：新模型 {pct(new_rate)}，旧模型 {pct(old_rate)}，新模型相差 {(new_rate - old_rate) * 100:.2f} 个百分点。",
        "- car.rar 无独立框标注，结论表示本批正样本上的检测覆盖差异，不是严格精度结论。",
        f"- 原始预测 JSON：`{data['raw_new']}`、`{data['raw_old']}`",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/home/promise/mrs_test/data/sar_yolo_detector/car"))
    parser.add_argument("--new-raw", type=Path, required=True)
    parser.add_argument("--old-raw", type=Path, required=True)
    parser.add_argument("--json-output", type=Path, required=True)
    parser.add_argument("--report-output", type=Path, required=True)
    args = parser.parse_args()

    images = sorted(args.root.rglob("*.png"))
    if not images:
        raise SystemExit(f"no PNG images found under {args.root}")
    hashes = [sha256(image) for image in images]
    first_by_hash: dict[str, int] = {}
    unique_indices: list[int] = []
    for index, image_hash in enumerate(hashes):
        if image_hash not in first_by_hash:
            first_by_hash[image_hash] = index
            unique_indices.append(index)
    new_raw = json.loads(args.new_raw.read_text(encoding="utf-8"))["raw"]
    old_raw = json.loads(args.old_raw.read_text(encoding="utf-8"))["raw"]
    all_indices = list(range(len(images)))
    data: dict[str, Any] = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "dataset": {
            "root": str(args.root),
            "image_count": len(images),
            "unique_image_count": len(unique_indices),
            "duplicate_group_count": sum(count > 1 for count in Counter(hashes).values()),
            "duplicate_file_count": len(images) - len(unique_indices),
            "labels_found": False,
        },
        "models": {"new_model": new_raw, "old_model": old_raw},
        "results": {
            "new_model": {"all_images": evaluate(new_raw, all_indices), "unique_images": evaluate(new_raw, unique_indices)},
            "old_model": {"all_images": evaluate(old_raw, all_indices), "unique_images": evaluate(old_raw, unique_indices)},
        },
        "raw_new": str(args.new_raw),
        "raw_old": str(args.old_raw),
    }
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.report_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    args.report_output.write_text(make_report(data), encoding="utf-8")
    print(json.dumps({
        "json_output": str(args.json_output),
        "report_output": str(args.report_output),
        "images": len(images),
        "unique_images": len(unique_indices),
        "new_conf060": data["results"]["new_model"]["all_images"]["conf_0.60"]["hit_rate"],
        "old_conf060": data["results"]["old_model"]["all_images"]["conf_0.60"]["hit_rate"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
