#!/usr/bin/env python3
"""Evaluate tracker identity quality from annotated JSONL recordings.

Each row represents one tracked detection at capture time. Required fields are
``t``, ``ground_truth_id`` and ``track_id``. Optional fields include
``visible`` (default true), ``image_source``, ``scenario`` and ``illumination``.
Rows with ``visible:false`` mark an annotated absence; they are used to measure
whether the next visible observation re-acquires the same public identity.

The tool intentionally does not start ROS, Gazebo, a detector, or a model. It
is suitable for a labelled rosbag export and produces a machine-readable report
for CI/release evidence.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import pathlib
import sys
from typing import Any, Dict, Iterable, List, Tuple


def records(path: pathlib.Path) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                row["t"] = float(row["t"])
                row["ground_truth_id"] = str(row["ground_truth_id"])
                if row.get("track_id") is not None:
                    row["track_id"] = int(row["track_id"])
                result.append(row)
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                raise ValueError(f"{path}:{line_number}: invalid annotation: {error}")
    return sorted(result, key=lambda item: (item["t"], item["ground_truth_id"]))


def summarize(items: Iterable[Dict[str, Any]], reid_gap_sec: float) -> Dict[str, Any]:
    by_truth: Dict[str, List[Dict[str, Any]]] = collections.defaultdict(list)
    by_track: Dict[int, collections.Counter] = collections.defaultdict(collections.Counter)
    scenario: Dict[str, List[Dict[str, Any]]] = collections.defaultdict(list)
    for row in items:
        by_truth[row["ground_truth_id"]].append(row)
        scenario[str(row.get("scenario", "all"))].append(row)
        if row.get("visible", True) and row.get("track_id") is not None:
            by_track[row["track_id"]][row["ground_truth_id"]] += 1

    id_switches = 0
    reid_attempts = 0
    reid_successes = 0
    visible_rows = 0
    assigned_rows = 0
    for truth_rows in by_truth.values():
        previous_id = None
        previous_visible_time = None
        absent_started = None
        for row in truth_rows:
            visible = bool(row.get("visible", True))
            track_id = row.get("track_id")
            if not visible:
                if previous_visible_time is not None and absent_started is None:
                    absent_started = row["t"]
                continue
            visible_rows += 1
            if track_id is not None:
                assigned_rows += 1
                if previous_id is not None and track_id != previous_id:
                    id_switches += 1
                if absent_started is not None and previous_id is not None and \
                        row["t"] - absent_started <= reid_gap_sec:
                    reid_attempts += 1
                    reid_successes += int(track_id == previous_id)
                previous_id = track_id
            previous_visible_time = row["t"]
            absent_started = None

    # A track is falsely associated when its majority identity differs from
    # the row's ground truth. This handles simultaneous targets without
    # requiring a fragile frame-by-frame ordering convention.
    dominant = {track: counts.most_common(1)[0][0]
                for track, counts in by_track.items() if counts}
    false_associations = sum(
        1 for truth_rows in by_truth.values() for row in truth_rows
        if row.get("visible", True) and row.get("track_id") is not None and
        dominant.get(row["track_id"]) != row["ground_truth_id"])
    report = {
        "annotated_rows": sum(len(value) for value in by_truth.values()),
        "visible_rows": visible_rows,
        "assigned_rows": assigned_rows,
        "assignment_coverage": assigned_rows / visible_rows if visible_rows else 0.0,
        "ground_truth_targets": len(by_truth),
        "public_tracks": len(by_track),
        "id_switches": id_switches,
        "id_switch_rate": id_switches / assigned_rows if assigned_rows else 0.0,
        "false_associations": false_associations,
        "false_association_rate": false_associations / assigned_rows if assigned_rows else 0.0,
        "reid_attempts": reid_attempts,
        "reid_successes": reid_successes,
        "reid_reacquisition_rate": reid_successes / reid_attempts if reid_attempts else None,
        "by_scenario": {},
    }
    # Recursion is intentionally avoided so a malformed scenario key cannot
    # alter global counters. Scenario reports omit nested scenario summaries.
    for name, rows in scenario.items():
        local = summarize_without_scenarios(rows, reid_gap_sec)
        report["by_scenario"][name] = local
    return report


def summarize_without_scenarios(items: List[Dict[str, Any]], gap: float) -> Dict[str, Any]:
    copied = [dict(item, scenario="all") for item in items]
    base = summarize_core(copied, gap)
    return base


def summarize_core(items: List[Dict[str, Any]], reid_gap_sec: float) -> Dict[str, Any]:
    # Reuse summarize while preventing nested scenario expansion.
    by_truth: Dict[str, List[Dict[str, Any]]] = collections.defaultdict(list)
    by_track: Dict[int, collections.Counter] = collections.defaultdict(collections.Counter)
    for row in items:
        by_truth[row["ground_truth_id"]].append(row)
        if row.get("visible", True) and row.get("track_id") is not None:
            by_track[row["track_id"]][row["ground_truth_id"]] += 1
    switches = attempts = successes = visible = assigned = 0
    for truth_rows in by_truth.values():
        prior_id = None
        absent = None
        for row in truth_rows:
            if not row.get("visible", True):
                absent = row["t"] if absent is None else absent
                continue
            visible += 1
            current = row.get("track_id")
            if current is not None:
                assigned += 1
                switches += int(prior_id is not None and current != prior_id)
                if absent is not None and prior_id is not None and row["t"] - absent <= reid_gap_sec:
                    attempts += 1
                    successes += int(current == prior_id)
                prior_id = current
            absent = None
    dominant = {track: counts.most_common(1)[0][0] for track, counts in by_track.items()}
    false = sum(1 for rows in by_truth.values() for row in rows
                if row.get("visible", True) and row.get("track_id") is not None and
                dominant.get(row["track_id"]) != row["ground_truth_id"])
    return {
        "visible_rows": visible, "assigned_rows": assigned,
        "assignment_coverage": assigned / visible if visible else 0.0,
        "id_switches": switches, "id_switch_rate": switches / assigned if assigned else 0.0,
        "false_associations": false,
        "false_association_rate": false / assigned if assigned else 0.0,
        "reid_attempts": attempts, "reid_successes": successes,
        "reid_reacquisition_rate": successes / attempts if attempts else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--reid-gap-sec", type=float, default=5.0)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--maximum-id-switch-rate", type=float, default=0.02)
    parser.add_argument("--maximum-false-association-rate", type=float, default=0.01)
    parser.add_argument("--minimum-reid-reacquisition-rate", type=float, default=0.80)
    parser.add_argument("--required-scenarios", default="",
                        help="comma-separated scenario labels required for strict evidence")
    args = parser.parse_args()
    if args.reid_gap_sec <= 0.0:
        parser.error("--reid-gap-sec must be positive")
    report = summarize(records(args.input), args.reid_gap_sec)
    checks = {
        "id_switch_rate": report["id_switch_rate"] <= args.maximum_id_switch_rate,
        "false_association_rate": report["false_association_rate"] <= args.maximum_false_association_rate,
        "reid_reacquisition_rate": report["reid_reacquisition_rate"] is not None and
            report["reid_reacquisition_rate"] >= args.minimum_reid_reacquisition_rate,
    }
    required = [item.strip() for item in args.required_scenarios.split(",") if item.strip()]
    if required:
        checks["required_scenarios"] = all(item in report["by_scenario"]
                                            for item in required)
    report["checks"] = checks
    report["passed"] = all(checks.values())
    encoded = json.dumps(report, sort_keys=True, indent=2) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    return 0 if not args.strict or report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
