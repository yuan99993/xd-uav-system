#!/usr/bin/env python3
"""Convert VisDrone2019-DET annotations to a self-contained YOLO layout.

The script retains all ten valid VisDrone detection classes. It excludes
ignored regions (0) and `others` (11), clips boxes to the image boundary, and
records every decision in conversion_report.json for review.
"""

import argparse
import json
import os
import shutil
from collections import Counter
from pathlib import Path


NAMES = [
    "pedestrian", "people", "bicycle", "car", "van", "truck", "tricycle",
    "awning_tricycle", "bus", "motor",
]


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


def convert_split(source_root: Path, output_root: Path, split: str,
                  copy_images: bool) -> Counter:
    split_root = source_root / split
    if not split_root.is_dir():
        split_root = source_root / f"VisDrone2019-DET-{split}"
    image_directory = split_root / "images"
    annotation_directory = split_root / "annotations"
    if not image_directory.is_dir() or not annotation_directory.is_dir():
        raise FileNotFoundError(
            f"Expected {image_directory} and {annotation_directory}; "
            "use extracted VisDrone2019-DET-{train,val} directories")
    statistics = Counter()
    for image_path in sorted(image_directory.iterdir()):
        if image_path.suffix.lower() not in {".jpg", ".jpeg", ".png"}:
            continue
        annotation_path = annotation_directory / f"{image_path.stem}.txt"
        if not annotation_path.is_file():
            raise FileNotFoundError(f"Missing annotation for {image_path.name}")
        # OpenCV/Pillow are deliberately not needed: VisDrone annotations give
        # pixels, and this basic image header reader handles JPEG/PNG only when
        # conversion is requested. Ultralytics validates dimensions at train.
        from PIL import Image  # pylint: disable=import-outside-toplevel
        with Image.open(image_path) as image:
            width, height = image.size
        label_lines = []
        for line in annotation_path.read_text(encoding="utf-8").splitlines():
            fields = [field.strip() for field in line.split(",")]
            if len(fields) < 8:
                statistics["malformed_annotations"] += 1
                continue
            x, y, box_width, box_height = map(float, fields[:4])
            score, category = float(fields[4]), int(float(fields[5]))
            if score <= 0 or category not in range(1, 11):
                statistics["ignored_annotations"] += 1
                continue
            x1, y1 = max(0.0, x), max(0.0, y)
            x2, y2 = min(float(width), x + box_width), min(float(height), y + box_height)
            if x2 <= x1 or y2 <= y1:
                statistics["invalid_boxes"] += 1
                continue
            class_id = category - 1
            center_x, center_y = (x1 + x2) * 0.5 / width, (y1 + y2) * 0.5 / height
            normalized_width, normalized_height = (x2 - x1) / width, (y2 - y1) / height
            label_lines.append(
                f"{class_id} {center_x:.8f} {center_y:.8f} "
                f"{normalized_width:.8f} {normalized_height:.8f}")
            statistics[f"class_{class_id}"] += 1
        image_destination = output_root / "images" / split / image_path.name
        label_destination = output_root / "labels" / split / f"{image_path.stem}.txt"
        link_or_copy(image_path, image_destination, copy_images)
        label_destination.parent.mkdir(parents=True, exist_ok=True)
        label_destination.write_text("\n".join(label_lines) + ("\n" if label_lines else ""),
                                     encoding="utf-8")
        statistics["images"] += 1
        statistics["labels"] += len(label_lines)
    return statistics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path,
                        help="directory containing train/ and val/ VisDrone folders")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--copy-images", action="store_true",
                        help="copy instead of making absolute source-image symlinks")
    arguments = parser.parse_args()
    output = arguments.output.resolve()
    report = {split: convert_split(arguments.source, output, split,
                                   arguments.copy_images)
              for split in ("train", "val")}
    dataset_yaml = output / "dataset.yaml"
    dataset_yaml.write_text(
        "path: " + str(output) + "\ntrain: images/train\nval: images/val\n"
        "names:\n" + "\n".join(f"  {index}: {name}" for index, name in enumerate(NAMES)) + "\n",
        encoding="utf-8")
    (output / "conversion_report.json").write_text(
        json.dumps({key: dict(value) for key, value in report.items()}, indent=2) + "\n",
        encoding="utf-8")
    print(f"Wrote {dataset_yaml} and {output / 'conversion_report.json'}")


if __name__ == "__main__":
    main()
