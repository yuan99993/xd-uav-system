#!/usr/bin/env python3
"""Prepare the official WIT-UAS split archive for the SAR YOLO pipeline.

The upstream WIT-UAS repository stores labels beside each image as
``x_min y_min x_max y_max token`` in ``.label`` files.  This converter keeps
the existing wildfire profile class contract and writes a standard Ultralytics
dataset.yaml.  Images are symlinked by default so the large archive is not
duplicated; pass --copy-images when a self-contained dataset is required.
"""

import argparse
import json
import os
import re
import shutil
from collections import Counter
from pathlib import Path
from typing import Optional


NAMES = ["person", "car", "bicycle", "other_vehicle"]


def token_to_class(token: str) -> Optional[int]:
    """Map WIT's h/v tokens and explicit class names to package IDs.

    The upstream WIT loader defines ``h`` as human and ``v`` as vehicle/car.
    Numeric tokens follow ``dataset.classes`` (0=noobject, 1=person,
    2=car, 3=bicycle, 4=othervehicle, 5=dontcare).
    """
    value = re.sub(r"[^a-z0-9]+", "", token.lower())
    if value in {"h", "human", "person", "people"}:
        return 0
    if value in {"v", "vehicle", "car"}:
        return 1
    if value in {"bicycle", "bike"}:
        return 2
    if value in {"othervehicle", "othercar", "truck", "bus"}:
        return 3
    if value.isdigit():
        upstream_id = int(value)
        return {1: 0, 2: 1, 3: 2, 4: 3}.get(upstream_id)
    return None


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


def image_sensor(path: Path) -> str:
    parts = {part.lower() for part in path.parts}
    if "flir" in parts:
        return "flir"
    if "seek" in parts:
        return "seek"
    return "unknown"


def convert_split(split: str, split_root: Path, output: Path, sensor: str,
                  copy_images: bool) -> Counter:
    stats = Counter()
    image_paths = sorted(
        path for path in split_root.rglob("*")
        if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"}
        and (sensor == "both" or image_sensor(path) == sensor)
    )
    for source in image_paths:
        relative = source.relative_to(split_root)
        label_source = source.with_suffix(".label")
        if not label_source.is_file():
            stats["images_without_labels"] += 1
            continue
        try:
            from PIL import Image
            width, height = Image.open(source).size
        except Exception as error:
            raise RuntimeError(f"Cannot read image size for {source}: {error}") from error
        if width <= 0 or height <= 0:
            stats["invalid_images"] += 1
            continue
        lines = []
        for line_number, raw in enumerate(label_source.read_text(errors="replace").splitlines(), 1):
            fields = raw.split()
            if len(fields) < 5:
                if raw.strip():
                    stats["malformed_annotations"] += 1
                continue
            try:
                x1, y1, x2, y2 = (float(value) for value in fields[:4])
            except ValueError:
                stats["malformed_annotations"] += 1
                continue
            class_id = token_to_class(fields[-1])
            if class_id is None:
                stats["ignored_annotations"] += 1
                continue
            x1, x2 = max(0.0, min(x1, width)), max(0.0, min(x2, width))
            y1, y2 = max(0.0, min(y1, height)), max(0.0, min(y2, height))
            if x2 <= x1 or y2 <= y1:
                stats["invalid_boxes"] += 1
                continue
            lines.append(
                f"{class_id} {(x1 + x2) * 0.5 / width:.8f} "
                f"{(y1 + y2) * 0.5 / height:.8f} "
                f"{(x2 - x1) / width:.8f} {(y2 - y1) / height:.8f}"
            )
            stats[f"class_{class_id}"] += 1
        destination = output / "images" / split / relative
        link_or_copy(source, destination, copy_images)
        label_destination = output / "labels" / split / relative.with_suffix(".txt")
        label_destination.parent.mkdir(parents=True, exist_ok=True)
        label_destination.write_text("\n".join(lines) + ("\n" if lines else ""))
        stats["images"] += 1
        stats["labels"] += len(lines)
    return stats


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", type=Path, required=True,
                        help="WIT-UAS-Dataset_split directory")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sensor", choices=["flir", "seek", "both"], default="flir")
    parser.add_argument("--copy-images", action="store_true")
    args = parser.parse_args()
    root = args.input_root.resolve()
    output = args.output.resolve()
    if not root.is_dir():
        raise SystemExit(f"Input root does not exist: {root}")
    reports = {}
    for split in ("train", "val", "test"):
        split_root = root / split
        if split_root.is_dir():
            reports[split] = dict(convert_split(split, split_root, output, args.sensor,
                                                args.copy_images))
    if not {"train", "val"}.issubset(reports):
        raise SystemExit("WIT-UAS split must contain train/ and val/ directories")
    output.mkdir(parents=True, exist_ok=True)
    (output / "dataset.yaml").write_text(
        "path: " + str(output) + "\ntrain: images/train\nval: images/val\n"
        "names:\n" + "\n".join(f"  {i}: {name}" for i, name in enumerate(NAMES)) + "\n"
    )
    (output / "conversion_report.json").write_text(
        json.dumps({"sensor": args.sensor, "class_names": NAMES, "splits": reports},
                   indent=2) + "\n"
    )
    print(f"Wrote {output / 'dataset.yaml'} and {output / 'conversion_report.json'}")


if __name__ == "__main__":
    main()
