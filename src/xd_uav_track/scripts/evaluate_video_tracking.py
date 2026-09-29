#!/usr/bin/env python3
"""Small MOT-ground-truth evaluator for video_tracking_record JSONL.

Reports detector recall, observed ID switches and simultaneous cross-source ID
agreement. It deliberately does not infer identity from mere image proximity.
"""

import argparse
import csv
import json
from collections import defaultdict


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    overlap = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return overlap / max(1, area_a + area_b - overlap)


def read_gt(path, offset, scale):
    frames = defaultdict(list)
    with open(path, newline="") as handle:
        for row in csv.reader(handle):
            if len(row) < 6:
                continue
            if len(row) > 6 and float(row[6]) <= 0:
                continue
            frame = int(float(row[0])) - offset
            x, y, w, h = [float(value) * scale for value in row[2:6]]
            frames[frame].append({"id": int(float(row[1])),
                                  "bbox": [x, y, x + w, y + h]})
    return frames


def matches(truth, objects, threshold):
    pairs = sorted(((iou(a["bbox"], b["bbox"]), ai, bi)
                    for ai, a in enumerate(truth)
                    for bi, b in enumerate(objects)), reverse=True)
    used_a, used_b, output = set(), set(), []
    for score, ai, bi in pairs:
        if score < threshold:
            break
        if ai not in used_a and bi not in used_b:
            used_a.add(ai)
            used_b.add(bi)
            output.append((truth[ai]["id"], objects[bi].get("id")))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record")
    parser.add_argument("--gt-a", required=True, help="MOT CSV ground truth")
    parser.add_argument("--source-a", default="replay_fixed")
    parser.add_argument("--gt-b", help="paired camera MOT CSV; IDs must be global")
    parser.add_argument("--source-b", default="replay_gimbal")
    parser.add_argument("--frame-offset-a", type=int, default=0,
                        help="subtract this from MOT frame numbers; default first frame is 1")
    parser.add_argument("--frame-offset-b", type=int, default=0)
    parser.add_argument("--gt-scale-a", type=float, default=1.0,
                        help="replay width / original annotation width")
    parser.add_argument("--gt-scale-b", type=float, default=1.0)
    parser.add_argument("--iou", type=float, default=0.5)
    args = parser.parse_args()
    gt = {args.source_a: read_gt(args.gt_a, args.frame_offset_a, args.gt_scale_a)}
    if args.gt_b:
        gt[args.source_b] = read_gt(args.gt_b, args.frame_offset_b, args.gt_scale_b)
    observations = defaultdict(dict)
    with open(args.record) as handle:
        for line in handle:
            row = json.loads(line)
            if row.get("source") in gt and row.get("type") in ("frame", "tracks", "detections"):
                # TrackStateArray.seq is a tracker-local counter, not the
                # camera frame index. Its capture stamp is inherited from
                # DetectionArray, so pair snapshots by exact source + stamp.
                observations[(row["source"], row["t"])][row["type"]] = row
    total_gt = detection_hits = track_hits = switches = replay_frames = 0
    previous = {}
    paired = defaultdict(dict)
    for (source, stamp), row in sorted(observations.items(), key=lambda item: (item[0][1], item[0][0])):
        if "frame" not in row:
            continue
        replay_frames += 1
        frame = int(row["frame"]["frame"])
        truth = gt[source].get(frame, [])
        total_gt += len(truth)
        detection_hits += len(matches(truth, row.get("detections", {}).get("objects", []), args.iou))
        tracks = [obj for obj in row.get("tracks", {}).get("objects", []) if obj.get("detected") and
                  obj.get("state") == "confirmed"]
        for gt_id, track_id in matches(truth, tracks, args.iou):
            track_hits += 1
            key = (source, gt_id)
            if key in previous and previous[key] != track_id:
                switches += 1
            previous[key] = track_id
            paired[(frame, gt_id)][source] = track_id
    cross_pairs = [ids for ids in paired.values() if len(ids) == 2]
    cross_matches = sum(ids[args.source_a] == ids[args.source_b]
                        for ids in cross_pairs)
    print(json.dumps({"replayed_frames": replay_frames, "gt_boxes": total_gt,
                      "detection_recall": detection_hits / max(1, total_gt),
                      "confirmed_track_recall": track_hits / max(1, total_gt),
                      "id_switches_observed": switches,
                      "cross_source_pairs": len(cross_pairs),
                      "cross_source_id_agreement": cross_matches / max(1, len(cross_pairs))
                      if cross_pairs else None}, indent=2))


if __name__ == "__main__":
    main()
