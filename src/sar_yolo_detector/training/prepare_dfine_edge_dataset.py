#!/usr/bin/env python3
"""Build an auditable, map-disjoint D-FINE edge-truncation COCO dataset.

Each source is a complete COCO annotation file plus its image root. COCO image
records marked ``verified_negative: true`` are the only eligible background
frames; an empty annotation list alone is not treated as proof of a negative.
"""
import argparse
import hashlib
import json
import random
import re
import shutil
from collections import Counter, defaultdict
from pathlib import Path


CLASSES = {"car": 0, "tank": 1, "drone": 2}
ALIASES = {"ar-car": "car", "military_tank": "tank", "uav": "drone",
           "vehicle": "car", "van": "car", "truck": "car", "bus": "car",
           "freight car": "car"}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def edge_box(box, width, height, margin):
    x, y, box_width, box_height = box
    return (x <= margin * width or y <= margin * height or
            x + box_width >= (1. - margin) * width or
            y + box_height >= (1. - margin) * height)


def load_source(map_id, image_root, annotation_path, margin):
    map_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", map_id.strip())
    if not map_id:
        raise ValueError("map_id must contain a safe filename character")
    image_root, annotation_path = Path(image_root).resolve(), Path(annotation_path).resolve()
    document = json.loads(annotation_path.read_text(encoding="utf-8"))
    info = document.get("info", {})
    coordinate_space = str(info.get("bbox_coordinate_space",
                                    info.get("bbox_format", ""))).strip().lower()
    description = str(info.get("description", "")).strip().lower()
    normalized_boxes = (bool(info.get("bbox_normalized", False)) or
                        coordinate_space in ("normalized", "normalized_xywh",
                                             "normalized_cxcywh") or
                        "normalized coco bbox" in description)
    categories = {int(item["id"]): ALIASES.get(str(item["name"]).strip().lower(),
                                                  str(item["name"]).strip().lower())
                  for item in document.get("categories", [])}
    category_ids = {source_id: CLASSES[name] for source_id, name in categories.items()
                    if name in CLASSES}
    if not category_ids:
        raise ValueError(f"{annotation_path}: no car/tank/drone categories")
    image_by_id = {int(image["id"]): image for image in document.get("images", [])}
    annotations = defaultdict(list)
    edge_counts = Counter()
    for annotation in document.get("annotations", []):
        image_id = int(annotation["image_id"])
        if image_id not in image_by_id:
            raise ValueError(f"{annotation_path}: annotation references missing image {image_id}")
        source_class = int(annotation["category_id"])
        if source_class not in category_ids:
            continue
        image = image_by_id[image_id]
        x, y, width, height = map(float, annotation["bbox"])
        image_width, image_height = float(image["width"]), float(image["height"])
        if normalized_boxes:
            x, width = x * image_width, width * image_width
            y, height = y * image_height, height * image_height
        if min(width, height, image_width, image_height) <= 0:
            raise ValueError(f"{annotation_path}: non-positive image or box size")
        converted = dict(category_id=category_ids[source_class],
                         bbox=[x, y, width, height],
                         area=(width * height if normalized_boxes else
                               float(annotation.get("area", width * height))),
                         iscrowd=int(annotation.get("iscrowd", 0)))
        annotations[image_id].append(converted)
        if edge_box((x, y, width, height), image_width, image_height, margin):
            edge_counts[image_id] += 1

    positives, baseline, negatives = [], [], []
    for source_id, image in image_by_id.items():
        target_annotations = annotations.get(source_id, [])
        if edge_counts[source_id] > 0:
            positives.append((map_id, source_id, image, target_annotations,
                              edge_counts[source_id], image_root))
        elif target_annotations:
            baseline.append((map_id, source_id, image, target_annotations, 0, image_root))
        elif not target_annotations and bool(image.get("verified_negative", False)):
            negatives.append((map_id, source_id, image, [], 0, image_root))
    return positives, baseline, negatives, annotation_path, (
        "normalized_xywh" if normalized_boxes else "pixel_xywh")


def copy_split(records, split, output, categories):
    image_dir = output / split / "images"
    image_dir.mkdir(parents=True, exist_ok=True)
    images, annotations, edge_boxes, maps = [], [], 0, set()
    next_image_id = 1
    next_annotation_id = 1
    source_manifest = []
    class_counts = Counter()
    for map_id, source_id, image, source_annotations, edge_count, root in records:
        source = (root / image["file_name"]).resolve()
        if not source.is_relative_to(root) or not source.is_file():
            raise FileNotFoundError(f"Image path escapes source root or is missing: {source}")
        basename = re.sub(r"[^A-Za-z0-9_.-]+", "_", Path(image["file_name"]).name)
        filename = f"{map_id}_{source_id}_{basename}"
        destination = image_dir / filename
        if destination.exists():
            raise FileExistsError(str(destination))
        shutil.copy2(source, destination)
        images.append(dict(id=next_image_id, file_name=filename,
            width=int(image["width"]), height=int(image["height"]), source_map=map_id,
            source_image_id=source_id, verified_negative=(len(source_annotations) == 0)))
        for item in source_annotations:
            annotations.append(dict(id=next_annotation_id, image_id=next_image_id,
                category_id=item["category_id"], bbox=item["bbox"], area=item["area"],
                iscrowd=item["iscrowd"]))
            next_annotation_id += 1
            class_counts[item["category_id"]] += 1
        edge_boxes += edge_count
        maps.add(map_id)
        source_manifest.append(dict(map_id=map_id, source_image_id=source_id,
            file_name=filename, sha256=sha256(destination), edge_box_count=edge_count,
            verified_negative=(len(source_annotations) == 0)))
        next_image_id += 1
    document = dict(images=images, annotations=annotations,
        categories=[dict(id=class_id, name=name, supercategory="target")
                    for name, class_id in CLASSES.items()])
    (output / split / "instances.json").write_text(
        json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return dict(images=len(images), annotations=len(annotations), edge_boxes=edge_boxes,
                maps=sorted(maps), class_counts={
                    name: class_counts[class_id] for name, class_id in categories.items()},
                records=source_manifest)


def cap_edge_records(records, maximum, rng):
    """Stratified deterministic cap which keeps every contributing map."""
    if maximum <= 0 or len(records) <= maximum:
        return records
    by_map = defaultdict(list)
    for record in records:
        by_map[record[0]].append(record)
    if maximum < len(by_map):
        raise SystemExit(f"edge-image cap {maximum} cannot retain all {len(by_map)} maps")
    for values in by_map.values():
        rng.shuffle(values)
    counts = {map_id: 1 for map_id in by_map}
    remaining = maximum - len(by_map)
    while remaining:
        eligible = [map_id for map_id, values in by_map.items()
                    if counts[map_id] < len(values)]
        if not eligible:
            break
        eligible.sort(key=lambda map_id: (counts[map_id] / len(by_map[map_id]), map_id))
        for map_id in eligible:
            if remaining == 0:
                break
            counts[map_id] += 1
            remaining -= 1
    return [record for map_id in sorted(by_map)
            for record in by_map[map_id][:counts[map_id]]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-id", action="append", required=True,
                        help="source map/scene label; repeat alongside --images and --annotations")
    parser.add_argument("--images", action="append", required=True, help="source image root")
    parser.add_argument("--annotations", action="append", required=True, help="source COCO JSON")
    parser.add_argument("--validation-map", required=True,
                        help="entire held-out map ID; it never appears in training")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--edge-margin", type=float, default=0.08)
    parser.add_argument("--negative-fraction", type=float, default=0.20)
    parser.add_argument("--minimum-edge-boxes", type=int, default=300)
    parser.add_argument("--minimum-edge-images", type=int, default=300)
    parser.add_argument("--max-edge-images-per-split", type=int, default=0,
                        help="Optional deterministic cap; 0 uses all edge-positive images")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if not (len(args.map_id) == len(args.images) == len(args.annotations)):
        parser.error("--map-id, --images and --annotations must be repeated equally")
    if not 0 < args.edge_margin <= 0.25:
        parser.error("--edge-margin must be within (0, 0.25]")
    if not 0 <= args.negative_fraction < 0.5:
        parser.error("--negative-fraction must be in [0, 0.5)")
    if args.minimum_edge_boxes < 1:
        parser.error("--minimum-edge-boxes must be positive")
    if args.minimum_edge_images < 1:
        parser.error("--minimum-edge-images must be positive")
    if args.max_edge_images_per_split < 0:
        parser.error("--max-edge-images-per-split cannot be negative")
    args.map_id = [re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()) for value in args.map_id]
    args.validation_map = re.sub(r"[^A-Za-z0-9_.-]+", "_", args.validation_map.strip())
    if len(set(args.map_id)) < 2 or args.validation_map not in args.map_id:
        parser.error("Use at least two distinct maps and designate one as --validation-map")

    all_positive, all_baseline, all_negative, source_paths = [], [], [], []
    for map_id, image_root, annotation_path in zip(args.map_id, args.images, args.annotations):
        positive, baseline, negative, source_path, bbox_format = load_source(
            map_id, image_root, annotation_path, args.edge_margin)
        all_positive.extend(positive)
        all_baseline.extend(baseline)
        all_negative.extend(negative)
        source_paths.append(dict(map_id=map_id, annotations=str(source_path),
                                 annotation_sha256=sha256(source_path),
                                 bbox_coordinate_space=bbox_format))
    edge_total = sum(item[4] for item in all_positive)
    edge_image_total = len(all_positive)
    edge_maps = {item[0] for item in all_positive}
    if edge_total < args.minimum_edge_boxes:
        raise SystemExit(f"Need at least {args.minimum_edge_boxes} edge boxes; found {edge_total}")
    if edge_image_total < args.minimum_edge_images:
        raise SystemExit(f"Need at least {args.minimum_edge_images} edge-positive images; found {edge_image_total}")
    if len(edge_maps) < 2:
        raise SystemExit("Edge-truncated boxes must come from at least two distinct maps")

    rng = random.Random(args.seed)
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"Output must be empty: {output}")
    for path in (output / "train", output / "val"):
        path.mkdir(parents=True, exist_ok=True)
    selected = {}
    for split in ("train", "val"):
        map_filter = args.validation_map if split == "val" else None
        positives = [r for r in all_positive if (r[0] == map_filter if map_filter else r[0] != args.validation_map)]
        baseline = [r for r in all_baseline if (r[0] == map_filter if map_filter else r[0] != args.validation_map)]
        negatives = [r for r in all_negative if (r[0] == map_filter if map_filter else r[0] != args.validation_map)]
        if not positives:
            raise SystemExit(f"No edge-positive images available for {split}; map-disjoint validation required")
        positives = cap_edge_records(positives, args.max_edge_images_per_split, rng)
        # Define the requested 3:1 joint mix explicitly as edge-positive
        # frames : ordinary positive baseline frames (by image count).
        baseline_count = int(round(len(positives) / 3.))
        if baseline_count > len(baseline):
            raise SystemExit(f"{split} needs {baseline_count} non-edge baseline frames for the 3:1 mix; "
                             f"only {len(baseline)} exist")
        baseline = rng.sample(baseline, baseline_count) if baseline_count else []
        positives = positives + baseline
        wanted = int(round(len(positives) * args.negative_fraction / (1. - args.negative_fraction)))
        if wanted > len(negatives):
            raise SystemExit(f"{split} needs {wanted} explicitly verified negatives but only {len(negatives)} exist")
        negatives = rng.sample(negatives, wanted) if wanted else []
        selected[split] = copy_split(positives + negatives, split, output, CLASSES)
    manifest = dict(dataset="dfine_edge_truncation_v1", edge_margin_fraction=args.edge_margin,
        edge_to_baseline_ratio=3.0,
        negative_fraction=args.negative_fraction, negative_positive_ratio=(
            args.negative_fraction / (1. - args.negative_fraction)), seed=args.seed,
        validation_policy="whole map held out", validation_map=args.validation_map,
        minimum_edge_boxes=args.minimum_edge_boxes, minimum_edge_images=args.minimum_edge_images,
        edge_boxes_total=edge_total, edge_images_total=edge_image_total,
        max_edge_images_per_split=args.max_edge_images_per_split,
        edge_source_maps=sorted(edge_maps), sources=source_paths, splits=selected,
        classes=CLASSES)
    (output / "dataset_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "dataset.yaml").write_text(
        f"path: {output}\ntrain: train/images\nval: val/images\nnames:\n"
        "  0: car\n  1: tank\n  2: drone\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in manifest.items()
                      if key != "splits"}, indent=2, ensure_ascii=False))
    for split, summary in selected.items():
        print(f"{split}: images={summary['images']} annotations={summary['annotations']} "
              f"edge_boxes={summary['edge_boxes']} maps={summary['maps']}")


if __name__ == "__main__":
    main()
