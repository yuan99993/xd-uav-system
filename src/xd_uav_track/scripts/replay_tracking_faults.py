#!/usr/bin/env python3
"""Low-load JSONL tracking replay and acceptance metrics.

The tool deliberately has no ROS/Gazebo dependency.  Input records may contain
``t``, ``target_visible``, ``radius_m``, ``target_radius_m``, ``angle_rad`` and
``track_id``.  Missing input produces a deterministic circular synthetic run.
Faults are injected before metric calculation so the same file can be used for
5/15/30 FPS, drops, delay, reordering and intermittent occlusion regressions.
"""
import argparse
import json
import math
import random
import sys
import time
from collections import deque


def load_records(path, duration, fps):
    if path:
        records = []
        with open(path, "r", encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    records.append(json.loads(line))
        return records
    count = max(1, int(round(duration * fps)))
    records = []
    for index in range(count):
        t = index / float(fps)
        angle = 0.35 * t
        visible = not (6.0 <= t < 7.0 or 13.0 <= t < 13.6)
        records.append({
            "t": t,
            "target_visible": visible,
            "radius_m": 80.0 + 0.8 * math.sin(0.7 * t),
            "target_radius_m": 80.0,
            "angle_rad": angle,
            "track_id": 7,
        })
    return records


def inject_faults(records, args):
    selected = records[::max(1, int(round(30.0 / args.fps)))]
    delayed = []
    for index, record in enumerate(selected):
        item = dict(record)
        item["_order"] = index
        if args.drop_rate > 0.0 and random.random() < args.drop_rate:
            continue
        if args.occlusion_period > 0.0 and args.occlusion_duration > 0.0:
            phase = float(item.get("t", 0.0)) % args.occlusion_period
            if phase < args.occlusion_duration:
                item["target_visible"] = False
        item["t"] = float(item.get("t", index / args.fps)) + args.delay_sec
        delayed.append(item)
    if args.reorder_window > 1:
        chunks = []
        window = max(2, args.reorder_window)
        for start in range(0, len(delayed), window):
            block = delayed[start:start + window]
            block.reverse()
            chunks.extend(block)
        delayed = chunks
    # A tracker consumes arrival order, not capture order.  Keep the injected
    # order in ``_order`` and only use timestamps for freshness diagnostics.
    return delayed


def metric(records, args):
    if not records:
        return {"records": 0, "visual_coverage": 0.0, "radial_hit_rate": 0.0,
                "cumulative_angle_rad": 0.0, "signed_angle_rad": 0.0,
                "id_switches": 0, "source_switches": 0,
                "center_hold_samples": 0, "center_hold_drift_m": 0.0}
    visible = [r for r in records if bool(r.get("target_visible", False))]
    radial = []
    angles = []
    ids = []
    sources = []
    center_hold_points = []
    for record in records:
        radius = record.get("radius_m")
        target_radius = record.get("target_radius_m", args.target_radius)
        if radius is not None and target_radius is not None:
            radial.append(abs(float(radius) - float(target_radius)) <= args.radius_tolerance)
        if "angle_rad" in record:
            angles.append(float(record["angle_rad"]))
        if record.get("target_visible", False) and record.get("track_id") is not None:
            ids.append(record["track_id"])
        source = record.get("image_source", record.get("source"))
        if source is not None and source != "":
            sources.append(source)
        state = str(record.get("tracking_state", ""))
        if bool(record.get("center_hold", False)) or state == "center_hold":
            point = record.get("center_world")
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                try:
                    center_hold_points.append((float(point[0]), float(point[1])))
                except (TypeError, ValueError):
                    pass
            elif record.get("center_world_x_m") is not None and record.get("center_world_y_m") is not None:
                try:
                    center_hold_points.append((float(record["center_world_x_m"]),
                                               float(record["center_world_y_m"])))
                except (TypeError, ValueError):
                    pass
    cumulative_angle = 0.0
    signed_angle = 0.0
    if len(angles) > 1:
        previous = angles[0]
        for angle in angles[1:]:
            delta = (angle - previous + math.pi) % (2.0 * math.pi) - math.pi
            cumulative_angle += abs(delta)
            signed_angle += delta
            previous = angle
    switches = sum(1 for old, new in zip(ids, ids[1:]) if old != new)
    source_switches = sum(1 for old, new in zip(sources, sources[1:])
                          if old != new)
    center_hold_drift = 0.0
    if center_hold_points:
        origin_x, origin_y = center_hold_points[0]
        center_hold_drift = max(math.hypot(x - origin_x, y - origin_y)
                                for x, y in center_hold_points)
    capture_times = [float(r.get("t", 0.0)) for r in records]
    ages = [b - a for a, b in zip(capture_times, capture_times[1:])]
    return {
        "records": len(records),
        "visible_records": len(visible),
        "visual_coverage": len(visible) / float(len(records)),
        "radial_samples": len(radial),
        "radial_hit_rate": sum(radial) / float(len(radial)) if radial else 0.0,
        "cumulative_angle_rad": cumulative_angle,
        "signed_angle_rad": signed_angle,
        "id_switches": switches,
        "source_switches": source_switches,
        "center_hold_samples": len(center_hold_points),
        "center_hold_drift_m": center_hold_drift,
        "max_arrival_dt_sec": max(ages) if ages else 0.0,
        "out_of_order_timestamps": sum(1 for a, b in zip(capture_times, capture_times[1:]) if b < a),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", help="JSONL observations; omit for synthetic data")
    parser.add_argument("--fps", type=float, default=30.0, choices=(5.0, 15.0, 30.0))
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--drop-rate", type=float, default=0.0)
    parser.add_argument("--delay-sec", type=float, default=0.0)
    parser.add_argument("--reorder-window", type=int, default=0)
    parser.add_argument("--occlusion-period", type=float, default=0.0)
    parser.add_argument("--occlusion-duration", type=float, default=0.0)
    parser.add_argument("--radius-tolerance", type=float, default=8.0)
    parser.add_argument("--target-radius", type=float, default=80.0)
    parser.add_argument("--strict", action="store_true",
                        help="return non-zero when replay acceptance limits fail")
    parser.add_argument("--minimum-visual-coverage", type=float, default=0.60)
    parser.add_argument("--minimum-radial-hit-rate", type=float, default=0.90)
    parser.add_argument("--minimum-cumulative-angle-rad", type=float,
                        default=2.0 * math.pi)
    parser.add_argument("--maximum-id-switches", type=int, default=0)
    parser.add_argument("--maximum-source-switches", type=int, default=3)
    parser.add_argument("--maximum-center-hold-drift-m", type=float, default=5.0)
    parser.add_argument("--require-center-hold", action="store_true",
                        help="fail strict replay when no center_hold samples are present")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if (not 0.0 <= args.drop_rate <= 1.0 or args.delay_sec < 0.0 or
            args.maximum_center_hold_drift_m < 0.0):
        parser.error("drop-rate must be [0,1], delay-sec and center drift must be non-negative")
    random.seed(args.seed)
    started = time.perf_counter()
    records = inject_faults(load_records(args.input, args.duration, args.fps), args)
    result = metric(records, args)
    result["fps"] = args.fps
    result["faults"] = {"drop_rate": args.drop_rate, "delay_sec": args.delay_sec,
                         "reorder_window": args.reorder_window,
                         "occlusion_period": args.occlusion_period,
                         "occlusion_duration": args.occlusion_duration}
    result["checks"] = {
        "visual_coverage": result["visual_coverage"] >= args.minimum_visual_coverage,
        "radial_hit_rate": result["radial_hit_rate"] >= args.minimum_radial_hit_rate,
        "cumulative_angle": result["cumulative_angle_rad"] >= args.minimum_cumulative_angle_rad,
        "id_switches": result["id_switches"] <= max(0, args.maximum_id_switches),
        "source_switches": result["source_switches"] <= max(0, args.maximum_source_switches),
        "center_hold_drift": (result["center_hold_samples"] > 0 or
                              not args.require_center_hold) and
                             result["center_hold_drift_m"] <= args.maximum_center_hold_drift_m,
    }
    result["passed"] = all(result["checks"].values())
    result["cpu_time_sec"] = time.perf_counter() - started
    json.dump(result, sys.stdout, sort_keys=True)
    sys.stdout.write("\n")
    if args.strict and not result["passed"]:
        return 1
    return 0


if __name__ == "__main__":
    main()
