#!/usr/bin/env python3
"""Build an auditable thermal rescue detector data set.

The current workspace contains aerial thermal HIT-UAV positives and FireMan
thermal imagery without person annotations.  This builder keeps the runtime
four-class contract (person/bicycle/car/other_vehicle), preserves HIT-UAV's
native train/val/test split, and adds sampled FireMan frames as explicit
person-free hard negatives.  It never changes the packaged model.

Images are symlinked; generated labels and manifest metadata are local to the
experiment output.  The manifest is intentionally simple so later temporal
and altitude evaluations can be added without reparsing source archives.
"""

import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


NAMES = ["person", "bicycle", "car", "other_vehicle"]
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def image_files(root: Path) -> List[Path]:
    # Check symlinks before following them.  The existing converted datasets
    # intentionally use thousands of links, and stat-following each one makes
    # preparation unnecessarily slow on networked filesystems.
    return sorted(path for path in root.iterdir()
                  if path.suffix.lower() in IMAGE_SUFFIXES
                  if path.is_symlink() or path.is_file())


def link_or_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(str(destination)):
        raise FileExistsError(f"refusing to overwrite {destination}")
    try:
        destination.symlink_to(source.resolve())
    except OSError:
        shutil.copy2(source, destination)


def group_id(stem: str) -> str:
    if "_frame_" in stem:
        return stem.split("_frame_", 1)[0]
    return stem.rsplit("_", 1)[0] if "_" in stem else stem


def altitude_m(stem: str) -> Optional[float]:
    fields = stem.split("_")
    if len(fields) >= 3:
        try:
            value = float(fields[1])
            return value if value > 0.0 else None
        except ValueError:
            pass
    return None


def read_labels(path: Path) -> List[str]:
    if not path.is_file():
        return []
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines()
            if len(line.split()) == 5]


def stable_sample(paths: Sequence[Path], limit: int, seed: str) -> List[Path]:
    if limit <= 0 or len(paths) <= limit:
        return list(paths)
    ranked = sorted(paths, key=lambda path: hashlib.sha256(
        f"{seed}:{path.name}".encode("utf-8")).hexdigest())
    return sorted(ranked[:limit])


def write_yaml(output: Path) -> None:
    (output / "dataset.yaml").write_text(
        "path: " + str(output) + "\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        "names:\n" + "".join(f"  {index}: {name}\n"
                             for index, name in enumerate(NAMES)),
        encoding="utf-8")


def add_hit_split(hit_root: Path, output: Path, source_split: str,
                  target_split: str, manifest: List[dict], stats: Dict[str, int]) -> None:
    images = image_files(hit_root / "images" / source_split)
    labels_root = hit_root / "labels" / source_split
    for index, source in enumerate(images):
        stem = source.stem
        name = f"hit_{source_split}_{stem}{source.suffix.lower()}"
        image_key = str(Path("images") / target_split / name)
        label_key = str(Path("labels") / target_split / (Path(name).stem + ".txt"))
        destination_image = output / image_key
        link_or_copy(source, destination_image)
        source_label = labels_root / f"{stem}.txt"
        labels = read_labels(source_label)
        destination_label = output / label_key
        destination_label.parent.mkdir(parents=True, exist_ok=True)
        destination_label.write_text("\n".join(labels) + ("\n" if labels else ""),
                                     encoding="utf-8")
        classes = sorted({int(line.split()[0]) for line in labels})
        manifest.append({
            "split": target_split, "group_id": f"hit:{group_id(stem)}",
            "source": "HIT-UAV", "image_key": image_key, "label_key": label_key,
            "frame_index": index, "timestamp_sec": None,
            "altitude_m": altitude_m(stem), "classes": classes,
        })
        stats[f"{target_split}_images"] = stats.get(f"{target_split}_images", 0) + 1
        stats[f"{target_split}_boxes"] = stats.get(f"{target_split}_boxes", 0) + len(labels)


def add_hard_negatives(negative_root: Path, output: Path, source_split: str,
                       target_split: str, limit: int, manifest: List[dict],
                       stats: Dict[str, int], seed: str) -> None:
    images = stable_sample(image_files(negative_root / "images" / source_split), limit,
                           f"{seed}:{source_split}")
    for index, source in enumerate(images):
        name = f"fireman_negative_{source_split}_{source.stem}{source.suffix.lower()}"
        image_key = str(Path("images") / target_split / name)
        label_key = str(Path("labels") / target_split / (Path(name).stem + ".txt"))
        link_or_copy(source, output / image_key)
        label_path = output / label_key
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text("", encoding="utf-8")
        manifest.append({
            "split": target_split, "group_id": f"fireman:{group_id(source.stem)}",
            "source": "FireMan-hard-negative", "image_key": image_key,
            "label_key": label_key, "frame_index": index, "timestamp_sec": None,
            "altitude_m": None, "classes": [],
        })
        stats[f"{target_split}_hard_negative_images"] = stats.get(
            f"{target_split}_hard_negative_images", 0) + 1


def locate_coco_image(root: Path, annotation_file: Path, file_name: str) -> Optional[Path]:
    """Resolve common COCO archive layouts without guessing across datasets."""
    relative = Path(file_name)
    candidates = [annotation_file.parent / relative, root / relative,
                  root / "images" / relative, root / "Images" / relative]
    for candidate in candidates:
        if candidate.is_file() and candidate.suffix.lower() in IMAGE_SUFFIXES:
            return candidate
    basename_matches = list(root.rglob(relative.name))
    return next((candidate for candidate in basename_matches if candidate.is_file()), None)


def coco_split(file_name: str, image_id: int) -> str:
    parts = {part.lower() for part in Path(file_name).parts}
    if "train" in parts:
        return "train"
    if "val" in parts or "valid" in parts or "validation" in parts:
        return "val"
    if "test" in parts or "test-dev" in parts:
        return "test"
    # A deterministic split is only a fallback for archives without native splits.
    bucket = int(hashlib.sha256(f"zenodo:{image_id}".encode()).hexdigest()[:8], 16) % 10
    return "test" if bucket == 0 else ("val" if bucket == 1 else "train")


def add_coco_person_dataset(coco_root: Path, output: Path, manifest: List[dict],
                            stats: Dict[str, int], limit: int = 0) -> None:
    """Import person-only COCO annotations into the existing four-class contract.

    Images without a valid person annotation are intentionally skipped.  An
    unlabeled person image must never silently become a negative sample.
    """
    candidates = []
    for path in sorted(coco_root.rglob("*.json")):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if isinstance(payload, dict) and {"images", "annotations", "categories"} <= set(payload):
            candidates.append((path, payload))
    if not candidates:
        raise FileNotFoundError(f"no COCO annotation file found below {coco_root}")
    imported = 0
    for annotation_file, payload in candidates:
        categories = {int(category["id"]): str(category.get("name", "")).lower()
                      for category in payload.get("categories", [])}
        person_ids = {category_id for category_id, name in categories.items()
                      if any(token in name for token in ("person", "human", "pedestrian", "people"))}
        if not person_ids:
            continue
        by_image: Dict[int, List[dict]] = {}
        for annotation in payload.get("annotations", []):
            if int(annotation.get("category_id", -1)) in person_ids:
                by_image.setdefault(int(annotation.get("image_id", -1)), []).append(annotation)
        for image in payload.get("images", []):
            image_id = int(image.get("id", -1))
            annotations = by_image.get(image_id, [])
            width, height = float(image.get("width", 0)), float(image.get("height", 0))
            source = locate_coco_image(coco_root, annotation_file, str(image.get("file_name", "")))
            if source is None or width <= 0 or height <= 0 or not annotations:
                continue
            split = coco_split(str(image.get("file_name", "")), image_id)
            stem = source.stem
            safe_stem = "".join(character if character.isalnum() else "_" for character in stem)
            name = f"tir_{imported:06d}_{safe_stem}{source.suffix.lower()}"
            image_key = str(Path("images") / split / name)
            label_key = str(Path("labels") / split / (Path(name).stem + ".txt"))
            rows = []
            for annotation in annotations:
                box = annotation.get("bbox", [])
                if len(box) != 4:
                    continue
                x, y, box_width, box_height = map(float, box)
                x = max(0.0, min(x, width)); y = max(0.0, min(y, height))
                box_width = max(0.0, min(box_width, width - x))
                box_height = max(0.0, min(box_height, height - y))
                if box_width <= 1.0 or box_height <= 1.0:
                    continue
                rows.append("0 %.6f %.6f %.6f %.6f" %
                            ((x + box_width / 2.0) / width,
                             (y + box_height / 2.0) / height,
                             box_width / width, box_height / height))
            if not rows:
                continue
            if limit and imported >= limit:
                return
            link_or_copy(source, output / image_key)
            label_path = output / label_key
            label_path.parent.mkdir(parents=True, exist_ok=True)
            label_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
            manifest.append({
                "split": split, "group_id": f"zenodo:{group_id(stem)}",
                "source": "UAV-TIR-Zenodo", "image_key": image_key,
                "label_key": label_key, "frame_index": imported,
                "timestamp_sec": None, "altitude_m": None, "classes": [0],
            })
            stats[f"{split}_external_images"] = stats.get(f"{split}_external_images", 0) + 1
            stats[f"{split}_external_boxes"] = stats.get(f"{split}_external_boxes", 0) + len(rows)
            imported += 1


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hit-root", type=Path, required=True,
                        help="converted HIT-UAV root containing images/labels")
    parser.add_argument("--hard-negative-root", type=Path,
                        help="converted FireMan thermal root")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--hard-negative-train", type=int, default=1500)
    parser.add_argument("--hard-negative-val", type=int, default=300)
    parser.add_argument("--hard-negative-test", type=int, default=300)
    parser.add_argument("--external-coco-root", type=Path,
                        help="optional extracted COCO UAV-person dataset")
    parser.add_argument("--external-limit", type=int, default=0,
                        help="optional cap for external images; 0 imports all")
    parser.add_argument("--seed", default="thermal_rescue_v1")
    args = parser.parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit(f"refusing to use non-empty output: {args.out}")
    args.out.mkdir(parents=True, exist_ok=True)
    manifest: List[dict] = []
    stats: Dict[str, int] = {}
    for split in ("train", "val", "test"):
        add_hit_split(args.hit_root, args.out, split, split, manifest, stats)
    if args.hard_negative_root:
        add_hard_negatives(args.hard_negative_root, args.out, "train", "train",
                           args.hard_negative_train, manifest, stats, args.seed)
        add_hard_negatives(args.hard_negative_root, args.out, "val", "val",
                           args.hard_negative_val, manifest, stats, args.seed)
        add_hard_negatives(args.hard_negative_root, args.out, "val", "test",
                           args.hard_negative_test, manifest, stats, args.seed)
    if args.external_coco_root:
        add_coco_person_dataset(args.external_coco_root, args.out, manifest, stats,
                                args.external_limit)
    write_yaml(args.out)
    manifest_path = args.out / "manifest.jsonl"
    manifest_path.write_text("".join(json.dumps(row, sort_keys=True) + "\n"
                                             for row in manifest), encoding="utf-8")
    report = {
        "dataset": str(args.out), "class_names": NAMES,
        "source_contract": {"hit_uav": str(args.hit_root),
                             "hard_negative": str(args.hard_negative_root or "")},
        "split_policy": "native_hit_uav_train_val_test_plus_sampled_fireman_negatives_plus_native_or_deterministic_external_coco_split",
        "hard_negative_limits": {"train": args.hard_negative_train,
                                  "val": args.hard_negative_val,
                                  "test": args.hard_negative_test},
        "stats": stats, "images": len(manifest),
    }
    (args.out / "preparation_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
