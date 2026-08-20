#!/usr/bin/env python3
"""Prepare a video-disjoint FireMan thermal YOLO detection data set.

The original FireMan CVAT package is organised in short videos. Treating
individual frames as independently shuffled samples leaks nearly identical
views into validation. This tool assigns a whole video (or hard-negative
sequence) to exactly one split and records provenance for event evaluation.
"""

import argparse
import csv
import hashlib
import itertools
import json
import re
import shutil
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple


DEFAULT_CLASSES = ("fire_region", "smoke_region")
FRAME_NUMBER = re.compile(r"(?:frame_|^)(\d+)(?:\D|$)", re.IGNORECASE)
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


@dataclass
class Box:
    name: str
    xtl: float
    ytl: float
    xbr: float
    ybr: float


@dataclass
class Frame:
    source: Path
    group_id: str
    source_subset: str
    video_name: str
    frame_index: int
    timestamp_sec: Optional[float]
    width: int
    height: int
    boxes: List[Box] = field(default_factory=list)
    negative_category: str = ""
    altitude_m: Optional[float] = None

    @property
    def is_event(self) -> bool:
        return bool(self.boxes)


def parse_class_list(value: str) -> Tuple[str, ...]:
    classes = tuple(item.strip() for item in value.split(",") if item.strip())
    if not classes:
        raise argparse.ArgumentTypeError("--classes must contain at least one class")
    if len(set(classes)) != len(classes):
        raise argparse.ArgumentTypeError("--classes contains duplicates")
    return classes


def parse_optional_float(value: str) -> Optional[float]:
    value = value.strip()
    return None if not value else float(value)


def frame_number(image_name: str, fallback: int) -> int:
    match = FRAME_NUMBER.search(Path(image_name).stem)
    return int(match.group(1)) if match else fallback


def image_path_for(video_dir: Path, image_name: str) -> Path:
    candidate = video_dir / "images" / image_name
    if candidate.is_file():
        return candidate
    for extension in IMAGE_EXTENSIONS:
        alternative = candidate.with_suffix(extension)
        if alternative.is_file():
            return alternative
    return candidate


def parse_video(video_dir: Path, source_subset: str, classes: Set[str],
                source_fps: float) -> List[Frame]:
    root = ET.parse(video_dir / "annotations.xml").getroot()
    frames: List[Frame] = []
    group_id = f"{source_subset}/{video_dir.name}"
    for fallback_index, image in enumerate(root.findall("image")):
        name = image.attrib["name"]
        source = image_path_for(video_dir, name)
        if not source.is_file():
            continue
        width = int(float(image.attrib.get("width", 0)))
        height = int(float(image.attrib.get("height", 0)))
        if width <= 0 or height <= 0:
            continue
        index = frame_number(name, fallback_index)
        boxes: List[Box] = []
        for box in image.findall("box"):
            label = box.attrib.get("label", "")
            if label not in classes:
                continue
            xtl = max(0.0, min(width, float(box.attrib["xtl"])))
            ytl = max(0.0, min(height, float(box.attrib["ytl"])))
            xbr = max(0.0, min(width, float(box.attrib["xbr"])))
            ybr = max(0.0, min(height, float(box.attrib["ybr"])))
            if xbr > xtl and ybr > ytl:
                boxes.append(Box(label, xtl, ytl, xbr, ybr))
        frames.append(Frame(
            source=source, group_id=group_id, source_subset=source_subset,
            video_name=video_dir.name, frame_index=index,
            timestamp_sec=index / source_fps, width=width, height=height,
            boxes=boxes))
    return frames


def sample_video_frames(frames: Sequence[Frame], sample_stride: int) -> List[Frame]:
    if not frames:
        return []
    first_index = min(frame.frame_index for frame in frames)
    return [frame for frame in frames
            if (frame.frame_index - first_index) % sample_stride == 0]


def read_group_metadata(path: Optional[Path]) -> Dict[str, Optional[float]]:
    if path is None:
        return {}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or "group_id" not in reader.fieldnames:
            raise ValueError("group metadata CSV needs a group_id column")
        result: Dict[str, Optional[float]] = {}
        for row in reader:
            group = row.get("group_id", "").strip()
            if group:
                result[group] = parse_optional_float(row.get("altitude_m", ""))
    return result


def read_negative_manifest(path: Optional[Path]) -> List[Frame]:
    """Read curated empty-label hard negatives without assuming a data source.

    Required columns: image_path, group_id, category. group_id must identify a
    continuous sequence/flight, not one image; timestamp_sec and altitude_m
    are optional but required for their respective time/altitude metrics.
    """
    if path is None:
        return []
    required = {"image_path", "group_id", "category"}
    frames: List[Frame] = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("negative manifest requires image_path, group_id, category")
        for index, row in enumerate(reader):
            raw_path = Path(row["image_path"]).expanduser()
            source = raw_path if raw_path.is_absolute() else path.parent / raw_path
            if not source.is_file() or source.suffix.lower() not in IMAGE_EXTENSIONS:
                raise FileNotFoundError(f"negative manifest image is unavailable: {source}")
            group = row["group_id"].strip()
            category = row["category"].strip().lower()
            if not group or not category:
                raise ValueError(f"negative manifest row {index + 2} has empty group_id/category")
            import cv2  # Training-only dependency; ROS runtime remains C++.
            image = cv2.imread(str(source), cv2.IMREAD_UNCHANGED)
            if image is None:
                raise ValueError(f"cannot decode hard negative: {source}")
            height, width = image.shape[:2]
            frames.append(Frame(
                source=source, group_id=f"negative/{group}", source_subset="negative",
                video_name=group, frame_index=index,
                timestamp_sec=parse_optional_float(row.get("timestamp_sec", "")),
                width=width, height=height, boxes=[], negative_category=category,
                altitude_m=parse_optional_float(row.get("altitude_m", ""))))
    return frames


def group_statistics(frames: Sequence[Frame], classes: Sequence[str]) -> Dict[str, Counter]:
    stats: Dict[str, Counter] = defaultdict(Counter)
    for frame in frames:
        stats[frame.group_id]["images"] += 1
        stats[frame.group_id]["empty_images"] += not frame.boxes
        for box in frame.boxes:
            stats[frame.group_id][box.name] += 1
    for group in stats:
        for name in classes:
            stats[group][name] += 0
    return stats


def choose_validation_groups(group_stats: Dict[str, Counter], classes: Sequence[str],
                             val_ratio: float, require_class_complete: bool) -> Set[str]:
    """Select entire groups while preferring class-complete validation."""
    groups = sorted(group_stats)
    if len(groups) < 2:
        raise ValueError("at least two video/flight groups are required for a disjoint split")
    totals = Counter()
    for stats in group_stats.values():
        totals.update(stats)
    supporting_groups = {
        name: {group for group in groups if group_stats[group][name] > 0}
        for name in classes
    }
    impossible = [name for name, values in supporting_groups.items() if len(values) < 2]
    if require_class_complete and impossible:
        raise ValueError("cannot create class-complete video split; class appears in fewer "
                         f"than two groups: {', '.join(impossible)}")

    def score(candidate: Set[str]) -> Optional[float]:
        if not candidate or len(candidate) == len(groups):
            return None
        train = set(groups) - candidate
        if require_class_complete:
            for name in classes:
                if not supporting_groups[name].intersection(candidate):
                    return None
                if not supporting_groups[name].intersection(train):
                    return None
        validation = Counter()
        for group in candidate:
            validation.update(group_stats[group])
        value = 2.0 * abs(validation["images"] / max(1, totals["images"]) - val_ratio)
        for name in classes:
            if totals[name]:
                value += 4.0 * abs(validation[name] / totals[name] - val_ratio)
        return value

    best: Optional[Tuple[float, Tuple[str, ...]]] = None
    # FireMan has eight groups. Exhaustive assignment makes this stable and
    # avoids random frame-level leakage. Larger future data sets use a stable
    # greedy fallback.
    if len(groups) <= 16:
        for size in range(1, len(groups)):
            for combination in itertools.combinations(groups, size):
                value = score(set(combination))
                if value is not None and (best is None or (value, combination) < best):
                    best = (value, combination)
    else:
        selected: Set[str] = set()
        target = totals["images"] * val_ratio
        for group in sorted(groups, key=lambda item: (-group_stats[item]["images"], item)):
            if sum(group_stats[item]["images"] for item in selected) >= target:
                break
            trial = selected | {group}
            if score(trial) is not None:
                selected = trial
        value = score(selected)
        if value is not None:
            best = (value, tuple(sorted(selected)))
    if best is None:
        raise ValueError("unable to make a video-disjoint, class-complete validation split")
    return set(best[1])


def safe_stem(frame: Frame) -> str:
    group = re.sub(r"[^A-Za-z0-9_.-]+", "_", frame.group_id)
    digest = hashlib.sha1(str(frame.source).encode("utf-8")).hexdigest()[:10]
    return f"{group}_{frame.frame_index:07d}_{digest}"


def link_or_copy(source: Path, destination: Path, copy_images: bool) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if copy_images:
        shutil.copy2(source, destination)
    else:
        destination.symlink_to(source.resolve())


def reset_output(output: Path, overwrite: bool) -> None:
    directories = (output / "images", output / "labels")
    files = (output / "dataset.yaml", output / "conversion_report.json",
             output / "manifest.jsonl", output / "group_metadata_template.csv")
    existing = [path for path in (*directories, *files) if path.exists()]
    if existing and not overwrite:
        raise FileExistsError(f"output already contains converted data: {output}; use --overwrite")
    output.mkdir(parents=True, exist_ok=True)
    for directory in directories:
        if directory.exists():
            shutil.rmtree(directory)
    for file_path in files:
        if file_path.exists() or file_path.is_symlink():
            file_path.unlink()


def write_dataset(output: Path, frames: Sequence[Frame], classes: Sequence[str],
                  validation_groups: Set[str], copy_images: bool) -> dict:
    reports: Dict[str, Counter] = {"train": Counter(), "val": Counter()}
    manifest_rows = []
    for frame in sorted(frames, key=lambda item: (item.group_id, item.frame_index, str(item.source))):
        split = "val" if frame.group_id in validation_groups else "train"
        stem = safe_stem(frame)
        image_relative = Path("images") / split / f"{stem}{frame.source.suffix.lower()}"
        label_relative = Path("labels") / split / f"{stem}.txt"
        link_or_copy(frame.source, output / image_relative, copy_images)
        labels = []
        for box in frame.boxes:
            class_id = classes.index(box.name)
            xc = ((box.xtl + box.xbr) / 2.0) / frame.width
            yc = ((box.ytl + box.ybr) / 2.0) / frame.height
            bw = (box.xbr - box.xtl) / frame.width
            bh = (box.ybr - box.ytl) / frame.height
            labels.append(f"{class_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")
            reports[split][box.name] += 1
        (output / label_relative).parent.mkdir(parents=True, exist_ok=True)
        (output / label_relative).write_text("\n".join(labels) + ("\n" if labels else ""),
                                             encoding="utf-8")
        reports[split]["images"] += 1
        reports[split]["boxes"] += len(labels)
        reports[split]["empty_images"] += not labels
        if frame.negative_category:
            reports[split][f"negative:{frame.negative_category}"] += 1
        manifest_rows.append({
            "image_key": image_relative.as_posix(), "label_key": label_relative.as_posix(),
            "split": split, "group_id": frame.group_id,
            "source_subset": frame.source_subset, "video_name": frame.video_name,
            "source_path": str(frame.source), "frame_index": frame.frame_index,
            "timestamp_sec": frame.timestamp_sec, "altitude_m": frame.altitude_m,
            "is_event": frame.is_event, "classes": sorted({box.name for box in frame.boxes}),
            "negative_category": frame.negative_category,
        })
    (output / "manifest.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in manifest_rows), encoding="utf-8")
    (output / "dataset.yaml").write_text(
        "path: " + str(output) + "\ntrain: images/train\nval: images/val\nnames:\n" +
        "".join(f"  {index}: {name}\n" for index, name in enumerate(classes)), encoding="utf-8")
    return {split: dict(stats) for split, stats in reports.items()}


def write_group_template(output: Path, frames: Sequence[Frame], validation_groups: Set[str]) -> None:
    groups: Dict[str, Frame] = {}
    for frame in frames:
        groups.setdefault(frame.group_id, frame)
    with (output / "group_metadata_template.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["group_id", "split", "source_subset",
                                                    "video_name", "altitude_m"])
        writer.writeheader()
        for group, frame in sorted(groups.items()):
            writer.writerow({"group_id": group,
                             "split": "val" if group in validation_groups else "train",
                             "source_subset": frame.source_subset,
                             "video_name": frame.video_name,
                             "altitude_m": "" if frame.altitude_m is None else frame.altitude_m})


def build_report(frames: Sequence[Frame], classes: Sequence[str], validation_groups: Set[str],
                 reports: Optional[dict], sample_stride: int, source_fps: float,
                 sample_fps: float) -> dict:
    stats = group_statistics(frames, classes)
    return {
        "classes": list(classes), "split_policy": "video_disjoint",
        "source_fps": source_fps, "requested_sample_fps": sample_fps,
        "sample_stride_frames": sample_stride,
        "effective_sample_fps": source_fps / sample_stride,
        "groups": {group: {"split": "val" if group in validation_groups else "train",
                             **dict(values)} for group, values in sorted(stats.items())},
        "splits": reports or {}, "manifest": "manifest.jsonl",
        "altitude_metadata_template": "group_metadata_template.csv",
        "notes": [
            "A video/flight group never crosses training and validation splits.",
            "Hard negatives have empty labels and must be supplied through a curated manifest.",
            "Altitude-stratified metrics need altitude_m in metadata or the hard-negative manifest.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True,
                        help="extracted .../Unimodal/thermal directory containing train/val")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--classes", type=parse_class_list, default=DEFAULT_CLASSES,
                        help="output classes; default removes sparsely validated building")
    parser.add_argument("--source-fps", type=float, required=True,
                        help="verified source video frame rate for timestamp and sampling")
    parser.add_argument("--sample-fps", type=float, default=2.0,
                        help="target sampling in [1,3]; use 0 only for forensic conversion")
    parser.add_argument("--val-ratio", type=float, default=0.20)
    parser.add_argument("--negative-manifest", type=Path,
                        help="CSV: image_path,group_id,category[,timestamp_sec,altitude_m]")
    parser.add_argument("--group-metadata-csv", type=Path,
                        help="optional CSV: group_id,altitude_m")
    parser.add_argument("--copy-images", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-incomplete-class-split", action="store_true",
                        help="not for production: permit a class absent from a split")
    args = parser.parse_args()
    if args.source_fps <= 0:
        parser.error("--source-fps must be positive")
    if not 0.0 <= args.sample_fps <= 3.0 or 0.0 < args.sample_fps < 1.0:
        parser.error("--sample-fps must be 0 or between 1 and 3")
    if not 0.05 <= args.val_ratio <= 0.5:
        parser.error("--val-ratio must be in [0.05, 0.5]")
    sample_stride = 1 if args.sample_fps == 0 else max(1, round(args.source_fps / args.sample_fps))
    classes = tuple(args.classes)
    metadata = read_group_metadata(args.group_metadata_csv)
    frames: List[Frame] = []
    for subset in ("train", "val"):
        for video_dir in sorted((args.source / subset).glob("video_t_*")):
            if (video_dir / "annotations.xml").is_file():
                frames.extend(sample_video_frames(
                    parse_video(video_dir, subset, set(classes), args.source_fps), sample_stride))
    frames.extend(read_negative_manifest(args.negative_manifest))
    if not frames:
        raise ValueError("no usable frames found")
    for frame in frames:
        if frame.altitude_m is None and frame.group_id in metadata:
            frame.altitude_m = metadata[frame.group_id]
    validation_groups = choose_validation_groups(
        group_statistics(frames, classes), classes, args.val_ratio,
        not args.allow_incomplete_class_split)
    reports = None
    if not args.dry_run:
        reset_output(args.out, args.overwrite)
        reports = write_dataset(args.out, frames, classes, validation_groups, args.copy_images)
        write_group_template(args.out, frames, validation_groups)
        (args.out / "conversion_report.json").write_text(
            json.dumps(build_report(frames, classes, validation_groups, reports, sample_stride,
                                    args.source_fps, args.sample_fps), indent=2) + "\n",
            encoding="utf-8")
    print(json.dumps(build_report(frames, classes, validation_groups, reports, sample_stride,
                                  args.source_fps, args.sample_fps), indent=2))


if __name__ == "__main__":
    main()
