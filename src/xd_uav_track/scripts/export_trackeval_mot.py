#!/usr/bin/env python3
"""Export annotated XD tracking JSONL to the standard MOTChallenge layout.

This is deliberately an offline converter: it does not import ROS, load a
model, or alter flight-time tracking.  The resulting tree can be evaluated by
TrackEval for HOTA, IDF1, MOTA, ID switches and fragmentation.
"""
from __future__ import annotations

import argparse
import json
import pathlib
from collections import defaultdict
from typing import Dict, Iterable, List, Tuple


def load(path: pathlib.Path) -> List[dict]:
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                row["t"] = float(row["t"])
                row["bbox"] = [float(value) for value in row["bbox"]]
                if len(row["bbox"]) != 4:
                    raise ValueError("bbox must have four values")
                rows.append(row)
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                raise ValueError(f"{path}:{line_number}: invalid row: {error}")
    return rows


def frames(rows: Iterable[dict]) -> Dict[float, int]:
    stamps = sorted({float(row["t"]) for row in rows})
    return {stamp: index + 1 for index, stamp in enumerate(stamps)}


def mot_line(frame: int, identity: int, bbox: List[float], confidence: float,
             category: int = 1) -> str:
    x, y, width, height = bbox
    if width <= 0.0 or height <= 0.0:
        raise ValueError("bbox width and height must be positive")
    return f"{frame},{identity},{x:.3f},{y:.3f},{width:.3f},{height:.3f},{confidence:.6f},{category},1\n"


def write_lines(path: pathlib.Path, rows: Iterable[dict], identity_key: str,
                frame_map: Dict[float, int], ground_truth: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    output = []
    for row in rows:
        if not bool(row.get("visible", True)):
            continue
        value = row.get(identity_key)
        if value is None:
            continue
        try:
            identity = int(value)
        except (TypeError, ValueError):
            # MOT requires integer identities. Deterministic input ordering is
            # intentionally not used to conceal non-numeric annotation IDs.
            raise ValueError(f"{identity_key} must be an integer for MOT export")
        confidence = 1.0 if ground_truth else float(row.get("confidence", 1.0))
        output.append(mot_line(frame_map[float(row["t"])], identity, row["bbox"],
                               confidence, int(row.get("class_id", 1))))
    path.write_text("".join(output), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", required=True, type=pathlib.Path,
                        help="JSONL rows: t, ground_truth_id, bbox[, visible, class_id]")
    parser.add_argument("--tracker", required=True, type=pathlib.Path,
                        help="JSONL rows: t, track_id, bbox[, confidence, visible, class_id]")
    parser.add_argument("--output-root", required=True, type=pathlib.Path)
    parser.add_argument("--sequence", default="xd_uav")
    parser.add_argument("--tracker-name", default="xd_uav_track")
    parser.add_argument("--frame-rate", type=float, default=30.0)
    args = parser.parse_args()
    if args.frame_rate <= 0.0:
        parser.error("--frame-rate must be positive")
    truth, tracker = load(args.ground_truth), load(args.tracker)
    if not truth:
        parser.error("ground truth is empty")
    frame_map = frames([*truth, *tracker])
    root = args.output_root.resolve()
    gt_file = root / "gt" / "mot_challenge" / "MOT17-train" / args.sequence / "gt" / "gt.txt"
    tracker_file = root / "trackers" / "mot_challenge" / "MOT17-train" / args.tracker_name / "data" / f"{args.sequence}.txt"
    write_lines(gt_file, truth, "ground_truth_id", frame_map, True)
    write_lines(tracker_file, tracker, "track_id", frame_map, False)
    seqinfo = gt_file.parents[1] / "seqinfo.ini"
    seqinfo.write_text("[Sequence]\nname={0}\nframeRate={1:g}\nseqLength={2}\nimWidth=0\nimHeight=0\nimExt=.jpg\n".format(
        args.sequence, args.frame_rate, len(frame_map)), encoding="utf-8")
    (root / "gt" / "mot_challenge" / "seqmaps").mkdir(parents=True, exist_ok=True)
    (root / "gt" / "mot_challenge" / "seqmaps" / "MOT17-train.txt").write_text(
        "name\n" + args.sequence + "\n", encoding="utf-8")
    print(json.dumps({"gt": str(gt_file), "tracker": str(tracker_file),
                      "frames": len(frame_map), "sequence": args.sequence}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
