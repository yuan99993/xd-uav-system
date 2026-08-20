#!/usr/bin/env python3
"""Prepare the NITC Person Rescue RGB data as a video-disjoint YOLO set.

The downloaded Google Drive material can contain images and labels in separate
folders (and Windows ``:Zone.Identifier`` sidecars).  This tool merges those
trees by file stem, never treats an unpaired image as a negative, and creates a
video-disjoint train/validation/test split.  NITC is visible-light data and is
therefore kept as a one-class EO model rather than mixed into the thermal
profile.
"""

import argparse
import json
import os
import re
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def valid_file(path: Path, suffixes: set[str]) -> bool:
    return path.is_file() and ":Zone.Identifier" not in path.name and path.suffix.lower() in suffixes


def collect_images(roots: Iterable[Path]) -> Dict[str, Path]:
    images: Dict[str, Path] = {}
    for root in roots:
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*")):
            if valid_file(path, IMAGE_SUFFIXES):
                images.setdefault(path.stem, path)
    return images


def valid_label_rows(path: Path) -> List[str]:
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        fields = line.split()
        if len(fields) != 5:
            continue
        try:
            values = [float(value) for value in fields[1:]]
        except ValueError:
            continue
        # NITC is a person-only YOLO set.  Keep class 0 and reject malformed
        # boxes rather than silently training on corrupt coordinates.
        if fields[0] != "0" or any(value < 0.0 or value > 1.0 for value in values):
            continue
        if values[2] <= 0.0 or values[3] <= 0.0:
            continue
        rows.append("0 " + " ".join(f"{value:.8f}" for value in values))
    return rows


def collect_labels(roots: Iterable[Path]) -> Dict[str, Tuple[Path, List[str]]]:
    labels: Dict[str, Tuple[Path, List[str]]] = {}
    for root in roots:
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*.txt")):
            if ":Zone.Identifier" in path.name:
                continue
            rows = valid_label_rows(path)
            if not rows:
                continue
            previous = labels.get(path.stem)
            if previous is None or len(rows) > len(previous[1]):
                labels[path.stem] = (path, rows)
    return labels


def video_id(stem: str) -> str:
    match = re.match(r"([^_]+)_frame_", stem)
    return match.group(1) if match else stem.split("_", 1)[0]


def provided_split(path: Path) -> str:
    parts = {part.lower() for part in path.parts}
    if "test" in parts:
        return "test"
    if "val" in parts or "valid" in parts or "validation" in parts:
        return "val"
    if "train" in parts:
        return "train"
    return "unknown"


def link_or_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(str(destination)):
        raise FileExistsError(destination)
    try:
        destination.symlink_to(source.resolve())
    except OSError:
        shutil.copy2(source, destination)


def write_yaml(output: Path) -> None:
    (output / "dataset.yaml").write_text(
        f"path: {output}\ntrain: images/train\nval: images/val\ntest: images/test\n"
        "names:\n  0: person\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True,
                        help="canonical extracted NITC root")
    parser.add_argument("--extra-root", type=Path, action="append", default=[],
                        help="additional extracted tree containing missing labels/images")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--val-videos", default="719")
    parser.add_argument("--test-videos", default="718,720")
    args = parser.parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit(f"refusing to use non-empty output: {args.out}")
    args.out.mkdir(parents=True, exist_ok=True)
    roots = [args.root, *args.extra_root]
    images = collect_images(roots)
    labels = collect_labels(roots)
    val_videos = {value.strip() for value in args.val_videos.split(",") if value.strip()}
    test_videos = {value.strip() for value in args.test_videos.split(",") if value.strip()}
    if val_videos & test_videos:
        raise SystemExit("validation and test video sets overlap")

    manifest: List[dict] = []
    stats = Counter()
    video_stats = defaultdict(Counter)
    for stem, source_image in sorted(images.items()):
        label_entry = labels.get(stem)
        if label_entry is None:
            stats["unpaired_images"] += 1
            continue
        label_source, rows = label_entry
        vid = video_id(stem)
        split = "test" if vid in test_videos else ("val" if vid in val_videos else "train")
        name = f"nitc_{vid}_{stem}{source_image.suffix.lower()}"
        image_key = str(Path("images") / split / name)
        label_key = str(Path("labels") / split / (Path(name).stem + ".txt"))
        link_or_copy(source_image, args.out / image_key)
        label_path = args.out / label_key
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
        row = {
            "split": split, "source": "NITC-Person-Rescue", "video_id": vid,
            "frame_stem": stem, "provided_split": provided_split(source_image),
            "image_key": image_key, "label_key": label_key,
            "label_source": str(label_source), "boxes": len(rows),
        }
        manifest.append(row)
        stats[f"{split}_images"] += 1
        stats[f"{split}_boxes"] += len(rows)
        video_stats[vid][split] += 1
        video_stats[vid]["boxes"] += len(rows)

    write_yaml(args.out)
    (args.out / "manifest.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in manifest),
        encoding="utf-8")
    report = {
        "dataset": str(args.out), "class_names": ["person"],
        "source_roots": [str(root) for root in roots],
        "video_disjoint_split": {"train": "all other videos",
                                  "val": sorted(val_videos),
                                  "test": sorted(test_videos)},
        "stats": dict(stats), "videos": {key: dict(value)
                                           for key, value in sorted(video_stats.items())},
        "available_images": len(images), "available_labels": len(labels),
    }
    (args.out / "preparation_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
