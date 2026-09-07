#!/usr/bin/env python3
"""Build a video-disjoint EO rescue-person data set from local sources.

This builder deliberately combines only sources with compatible *visible-light*
person annotations:

* NITC Person Rescue: people stranded on buildings, roofs and balconies.
* SeaDronesSee: the annotated ``swimmer`` category, normalized to ``person``.

SeaDronesSee's supplied train/val files contain frames from some of the same
videos.  It is therefore re-split by ``source.video`` across both JSON files
before any training data are selected.  FloodNet is intentionally not used as
a person-detector negative set: its segmentation labels do not annotate
people, so treating all flood imagery as person-free would teach false
negatives.  Thermal sets likewise remain separate from this RGB model.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
from collections import Counter, defaultdict
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
DEFAULT_SEA_VAL_VIDEOS = ("DJI_0063.MP4", "NorthSea_Fri_DJI_0117.MP4")
DEFAULT_SEA_TEST_VIDEOS = (
    "NorthSea_Sat_Z30_DJI003.MP4",
    "NorthSea_Fri_DJI_0114.MP4",
)


@dataclass(frozen=True)
class Record:
    source: str
    source_split: str
    video_id: str
    image_id: str
    image: Path
    rows: tuple[str, ...]


def parse_csv(value: str) -> set[str]:
    return {part.strip() for part in value.split(",") if part.strip()}


def valid_person_rows(label: Path) -> tuple[str, ...]:
    rows: list[str] = []
    for line in label.read_text(encoding="utf-8", errors="replace").splitlines():
        fields = line.split()
        if len(fields) != 5 or fields[0] != "0":
            continue
        try:
            values = [float(item) for item in fields[1:]]
        except ValueError:
            continue
        if any(item < 0.0 or item > 1.0 for item in values):
            continue
        if values[2] <= 0.0 or values[3] <= 0.0:
            continue
        rows.append("0 " + " ".join(f"{item:.8f}" for item in values))
    return tuple(rows)


def collect_nitc(root: Path) -> list[Record]:
    records: list[Record] = []
    for split in ("train", "val", "test"):
        image_dir = root / "images" / split
        label_dir = root / "labels" / split
        if not image_dir.is_dir() or not label_dir.is_dir():
            raise FileNotFoundError(f"NITC prepared split is missing: {split}")
        for image in sorted(image_dir.iterdir()):
            if image.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            label = label_dir / f"{image.stem}.txt"
            if not label.is_file():
                raise FileNotFoundError(f"NITC image has no label: {image}")
            rows = valid_person_rows(label)
            if not rows:
                raise ValueError(f"NITC image has no valid person box: {image}")
            # Files created by prepare_nitc_person.py begin with nitc_<video>_.
            parts = image.stem.split("_", 2)
            video = parts[1] if len(parts) >= 3 and parts[0] == "nitc" else "unknown"
            records.append(Record("NITC-Person-Rescue", split, video, image.stem,
                                  image.resolve(), rows))
    return records


def normalized_box(annotation: dict, width: int, height: int) -> str | None:
    raw = annotation.get("bbox", [])
    if len(raw) != 4 or width <= 0 or height <= 0:
        return None
    try:
        x, y, box_width, box_height = (float(value) for value in raw)
    except (TypeError, ValueError):
        return None
    x1, y1 = max(0.0, x), max(0.0, y)
    x2, y2 = min(float(width), x + box_width), min(float(height), y + box_height)
    if x2 <= x1 or y2 <= y1:
        return None
    return (f"0 {(x1 + x2) * 0.5 / width:.8f} {(y1 + y2) * 0.5 / height:.8f} "
            f"{(x2 - x1) / width:.8f} {(y2 - y1) / height:.8f}")


def collect_seadronessee(root: Path) -> list[Record]:
    raw = root / "raw"
    image_root = root / "images"
    records: list[Record] = []
    for split in ("train", "val"):
        annotation_path = raw / f"instances_{split}.json"
        if not annotation_path.is_file():
            raise FileNotFoundError(annotation_path)
        payload = json.loads(annotation_path.read_text(encoding="utf-8"))
        categories = {int(item["id"]): str(item["name"]).strip().lower()
                      for item in payload.get("categories", [])}
        swimmer_ids = {category_id for category_id, name in categories.items()
                       if name == "swimmer"}
        if not swimmer_ids:
            raise ValueError(f"{annotation_path} has no swimmer category")
        annotations: dict[int, list[dict]] = defaultdict(list)
        for annotation in payload.get("annotations", []):
            if int(annotation.get("category_id", -1)) in swimmer_ids:
                annotations[int(annotation["image_id"])].append(annotation)
        for image_info in payload.get("images", []):
            image_id = int(image_info["id"])
            candidate_annotations = annotations.get(image_id, [])
            if not candidate_annotations:
                continue
            width, height = int(image_info["width"]), int(image_info["height"])
            rows = tuple(row for annotation in candidate_annotations
                         if (row := normalized_box(annotation, width, height)) is not None)
            if not rows:
                continue
            file_name = str(image_info.get("file_name", ""))
            image = image_root / split / file_name
            if not image.is_file():
                raise FileNotFoundError(f"SeaDronesSee image is missing: {image}")
            source = image_info.get("source") or {}
            video = (str(source.get("video") or source.get("folder_name") or
                         f"unattributed_{split}"))
            records.append(Record("SeaDronesSee-swimmer-as-person", split, video,
                                  str(image_id), image.resolve(), rows))
    return records


def balanced_video_sample(records: Iterable[Record], maximum: int, seed: int) -> list[Record]:
    by_video: dict[str, list[Record]] = defaultdict(list)
    for record in records:
        by_video[record.video_id].append(record)
    randomizer = random.Random(seed)
    queues: dict[str, list[Record]] = {}
    for video, items in by_video.items():
        ordered = sorted(items, key=lambda item: (item.source_split, int(item.image_id)))
        randomizer.shuffle(ordered)
        queues[video] = ordered
    selected: list[Record] = []
    positions = Counter()
    while len(selected) < maximum:
        added = False
        for video in sorted(queues):
            position = positions[video]
            if position >= len(queues[video]):
                continue
            selected.append(queues[video][position])
            positions[video] += 1
            added = True
            if len(selected) >= maximum:
                break
        if not added:
            break
    return selected


def link_or_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(destination):
        raise FileExistsError(destination)
    try:
        destination.symlink_to(source.resolve())
    except OSError:
        shutil.copy2(source, destination)


def destination_name(record: Record) -> str:
    if record.source == "NITC-Person-Rescue":
        return f"nitc_{record.video_id}_{record.image_id}{record.image.suffix.lower()}"
    return f"seadronessee_{record.source_split}_{record.image_id}{record.image.suffix.lower()}"


def write_record(output: Path, split: str, record: Record) -> tuple[str, str]:
    name = destination_name(record)
    image_key = Path("images") / split / name
    label_key = Path("labels") / split / f"{Path(name).stem}.txt"
    link_or_copy(record.image, output / image_key)
    label_path = output / label_key
    label_path.parent.mkdir(parents=True, exist_ok=True)
    label_path.write_text("\n".join(record.rows) + "\n", encoding="utf-8")
    return str(image_key), str(label_key)


def write_yaml(output: Path, filename: str, test_path: str) -> None:
    (output / filename).write_text(
        f"path: {output}\ntrain: images/train\nval: images/val\ntest: {test_path}\n"
        "names:\n  0: person\n", encoding="utf-8")


def tally(records: Iterable[Record]) -> dict[str, int]:
    count = Counter()
    for record in records:
        count["images"] += 1
        count["boxes"] += len(record.rows)
        count[f"video::{record.video_id}"] += 1
    return dict(count)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nitc", type=Path, required=True,
                        help="prepared NITC one-class dataset root")
    parser.add_argument("--seadronessee", type=Path, required=True,
                        help="local SeaDronesSee root with raw JSON and images")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-sea-train", type=int, default=1600,
                        help="balanced upper bound for positive SeaDronesSee train frames")
    parser.add_argument("--nitc-train-repeats", type=int, default=1,
                        help=("repeat only the video-disjoint NITC training split to "
                              "avoid overwhelming land/roof victims when all sea frames "
                              "are used"))
    parser.add_argument("--sea-val-videos", default=",".join(DEFAULT_SEA_VAL_VIDEOS))
    parser.add_argument("--sea-test-videos", default=",".join(DEFAULT_SEA_TEST_VIDEOS))
    parser.add_argument("--seed", type=int, default=20260820)
    args = parser.parse_args()
    if args.max_sea_train <= 0 or args.nitc_train_repeats <= 0:
        raise ValueError("--max-sea-train and --nitc-train-repeats must be positive")
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit(f"refusing to use non-empty output: {args.out}")
    val_videos, test_videos = parse_csv(args.sea_val_videos), parse_csv(args.sea_test_videos)
    if not val_videos or not test_videos or val_videos & test_videos:
        raise ValueError("sea validation/test video sets must be non-empty and disjoint")

    nitc_records = collect_nitc(args.nitc)
    sea_records = collect_seadronessee(args.seadronessee)
    available_videos = {record.video_id for record in sea_records}
    unknown = (val_videos | test_videos) - available_videos
    if unknown:
        raise ValueError(f"unknown SeaDronesSee video(s): {sorted(unknown)}")

    nitc_by_split = {split: [record for record in nitc_records if record.source_split == split]
                     for split in ("train", "val", "test")}
    sea_val = [record for record in sea_records if record.video_id in val_videos]
    sea_test = [record for record in sea_records if record.video_id in test_videos]
    sea_train_pool = [record for record in sea_records
                      if record.video_id not in val_videos | test_videos]
    sea_train = balanced_video_sample(sea_train_pool, args.max_sea_train, args.seed)
    if not sea_val or not sea_test or not sea_train:
        raise ValueError("a requested SeaDronesSee partition is empty")

    # Repeating a record is deliberate sample weighting, not a split leak: each
    # copy points to the same *training* frame and receives a unique output
    # filename.  It keeps the much smaller roof/building rescue set visible to
    # the optimizer when SeaDronesSee is expanded to its full training pool.
    nitc_train = [
        replace(record, image_id=f"{record.image_id}_repeat{repeat_index}")
        for repeat_index in range(args.nitc_train_repeats)
        for record in nitc_by_split["train"]
    ]

    args.out.mkdir(parents=True, exist_ok=True)
    partitions = {
        "train": [*nitc_train, *sea_train],
        "val": [*nitc_by_split["val"], *sea_val],
        "test": [*nitc_by_split["test"], *sea_test],
    }
    manifest: list[dict] = []
    stats = Counter()
    for split, records in partitions.items():
        for record in sorted(records, key=lambda item: (item.source, item.video_id, item.image_id)):
            image_key, label_key = write_record(args.out, split, record)
            manifest.append({
                "split": split, "source": record.source, "source_split": record.source_split,
                "video_id": record.video_id, "image_id": record.image_id,
                "image_key": image_key, "label_key": label_key, "boxes": len(record.rows),
            })
            stats[f"{split}_images"] += 1
            stats[f"{split}_boxes"] += len(record.rows)
            stats[f"{split}_{record.source}_images"] += 1
            stats[f"{split}_{record.source}_boxes"] += len(record.rows)

    # Duplicate symlinks only for source-specific test metrics.  This keeps a
    # combined test set available while preventing a good sea score from
    # hiding poor roof/building performance (or vice versa).
    for suffix, source_records in (("test_nitc", nitc_by_split["test"]),
                                   ("test_seadronessee", sea_test)):
        for record in source_records:
            write_record(args.out, suffix, record)
    write_yaml(args.out, "dataset.yaml", "images/test")
    write_yaml(args.out, "dataset_nitc_test.yaml", "images/test_nitc")
    write_yaml(args.out, "dataset_seadronessee_test.yaml", "images/test_seadronessee")
    (args.out / "manifest.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in manifest), encoding="utf-8")
    report = {
        "dataset": str(args.out),
        "class_names": ["person"],
        "normalization": {"SeaDronesSee.swimmer": "person"},
        "nitc_source": str(args.nitc),
        "seadronessee_source": str(args.seadronessee),
        "sea_video_disjoint_split": {
            "train": "all SeaDronesSee videos except validation/test sets; balanced frame sampling",
            "val": sorted(val_videos), "test": sorted(test_videos),
        },
        "sea_train_sampling": {
            "seed": args.seed, "maximum_images": args.max_sea_train,
            "available_positive_images": len(sea_train_pool),
            "selected_positive_images": len(sea_train),
            "selected": tally(sea_train),
        },
        "nitc_train_sample_weight": {
            "repeats": args.nitc_train_repeats,
            "unique_images": len(nitc_by_split["train"]),
            "emitted_training_images": len(nitc_train),
            "rationale": ("keep NITC roof/building victims represented while "
                          "using the full SeaDronesSee training pool"),
        },
        "partitions": {key: tally(value) for key, value in partitions.items()},
        "stats": dict(stats),
        "excluded_local_sources": {
            "FloodNet": "semantic masks have no reliable person bounding boxes; not safe as person-negative training data",
            "FireMan-UAV-RGBT": "local converted labels are fire_region/smoke_region/building, not person",
            "HIT-UAV and UAV-TIR": "thermal modality; retained for a separate infrared person model rather than mixed into RGB",
        },
    }
    (args.out / "preparation_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
