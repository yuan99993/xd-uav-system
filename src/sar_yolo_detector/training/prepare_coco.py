#!/usr/bin/env python3
"""Convert selected COCO categories to a YOLO detection data set.

This converter is used for SeaDronesSee v2 and HIT-UAV JSON annotations.
Mappings are explicit JSON files so class order cannot silently drift between
training, TensorRT export, and the C++ detector configuration.
"""

import argparse
import json
import os
import re
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Tuple


def canonical(value: str) -> str:
    return re.sub(r"_+", "_", re.sub(r"[^a-z0-9]+", "_", value.lower())).strip("_")


def key_value_path(value: str) -> Tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("Expected split=/absolute/or/relative/path")
    key, path = value.split("=", 1)
    if not key or not path:
        raise argparse.ArgumentTypeError("Expected split=/absolute/or/relative/path")
    return key, Path(path)


def link_or_copy(source: Path, destination: Path, copy_images: bool) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        if destination.resolve() != source.resolve():
            raise FileExistsError(f"Refusing to replace existing image: {destination}")
        return
    if copy_images:
        shutil.copy2(source, destination)
    else:
        os.symlink(source.resolve(), destination)


def convert_split(split: str, images_root: Path, annotation_path: Path,
                  output_root: Path, mapping: Dict[str, int],
                  copy_images: bool) -> Counter:
    data = json.loads(annotation_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("images"), list):
        raise ValueError(f"{annotation_path} is not a COCO annotation object")
    categories = {int(item["id"]): canonical(str(item["name"]))
                  for item in data.get("categories", [])}
    # HIT-UAV v1.2 follows the COCO layout but names these fields
    # ``filename`` and ``annotation``.  Keep accepting canonical COCO so
    # the same converter can be used for SeaDronesSee and other datasets.
    raw_annotations = data.get("annotations", data.get("annotation", []))
    if not isinstance(raw_annotations, list):
        raise ValueError(f"{annotation_path} annotations field is not a list")
    annotations = defaultdict(list)
    for annotation in raw_annotations:
        category = categories.get(int(annotation.get("category_id", -1)))
        if category in mapping:
            annotations[int(annotation["image_id"])].append(annotation)
    statistics = Counter()
    for image in data["images"]:
        image_id = int(image["id"])
        width, height = int(image["width"]), int(image["height"])
        if width <= 0 or height <= 0:
            statistics["invalid_images"] += 1
            continue
        file_name = image.get("file_name", image.get("filename"))
        if not file_name:
            raise ValueError(f"image {image_id} in {annotation_path} has no file name")
        source = images_root / str(file_name)
        if not source.is_file():
            raise FileNotFoundError(f"COCO image does not exist: {source}")
        relative_name = Path(str(file_name)); label_lines = []
        for annotation in annotations[image_id]:
            bbox = annotation.get("bbox", [])
            if len(bbox) != 4:
                statistics["malformed_annotations"] += 1
                continue
            x, y, box_width, box_height = (float(value) for value in bbox)
            x1, y1 = max(0.0, x), max(0.0, y)
            x2, y2 = min(float(width), x + box_width), min(float(height), y + box_height)
            if x2 <= x1 or y2 <= y1:
                statistics["invalid_boxes"] += 1
                continue
            category_name = categories[int(annotation["category_id"])]
            class_id = mapping[category_name]
            label_lines.append(
                f"{class_id} {(x1 + x2) * 0.5 / width:.8f} {(y1 + y2) * 0.5 / height:.8f} "
                f"{(x2 - x1) / width:.8f} {(y2 - y1) / height:.8f}")
            statistics[f"class_{class_id}"] += 1
        link_or_copy(source, output_root / "images" / split / relative_name, copy_images)
        label_path = output_root / "labels" / split / relative_name.with_suffix(".txt")
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text("\n".join(label_lines) + ("\n" if label_lines else ""),
                              encoding="utf-8")
        statistics["images"] += 1; statistics["labels"] += len(label_lines)
    return statistics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, action="append", type=key_value_path,
                        help="repeat for each split, e.g. train=/data/images/train")
    parser.add_argument("--annotations", required=True, action="append", type=key_value_path,
                        help="repeat for each split, e.g. train=/data/instances_train.json")
    parser.add_argument("--class-map", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--copy-images", action="store_true")
    arguments = parser.parse_args()
    image_roots, annotation_paths = dict(arguments.images), dict(arguments.annotations)
    if set(image_roots) != set(annotation_paths) or not {"train", "val"}.issubset(image_roots):
        raise ValueError("--images and --annotations need matching train and val entries")
    map_data = json.loads(arguments.class_map.read_text(encoding="utf-8"))
    names = list(map_data.get("names", []))
    mapping = {canonical(key): int(value) for key, value in map_data.get("categories", {}).items()}
    if not names or not mapping or any(value < 0 or value >= len(names) for value in mapping.values()):
        raise ValueError("class map needs contiguous named output IDs in range")
    output = arguments.output.resolve(); report = {}
    for split in sorted(image_roots):
        report[split] = convert_split(split, image_roots[split], annotation_paths[split],
                                      output, mapping, arguments.copy_images)
    (output / "dataset.yaml").write_text(
        "path: " + str(output) + "\ntrain: images/train\nval: images/val\n"
        "names:\n" + "\n".join(f"  {index}: {name}" for index, name in enumerate(names)) + "\n",
        encoding="utf-8")
    (output / "conversion_report.json").write_text(
        json.dumps({key: dict(value) for key, value in report.items()}, indent=2) + "\n",
        encoding="utf-8")
    print(f"Wrote {output / 'dataset.yaml'} and {output / 'conversion_report.json'}")


if __name__ == "__main__":
    main()
