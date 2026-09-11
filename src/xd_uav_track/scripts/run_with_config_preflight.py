#!/usr/bin/env python3
"""Validate raw deployment YAML, then replace this process with a ROS node.

`rosparam load` cannot expose duplicate YAML keys after parsing.  This wrapper
runs before the target executable, so a malformed configuration never reaches
the tracker or detector process.  ROS remapping arguments are preserved.
"""

import argparse
import importlib.util
import os
import sys

import roslib.packages


def _load_validator_module():
    """Load the sibling validator by file path, not ``sys.path``.

    Catkin exposes both scripts through generated Python relay files in
    ``devel/lib/<package>``.  A normal ``import validate_tracking_config`` can
    therefore import that relay instead of the source module and leave the
    validator symbols unavailable.  Loading the known sibling path keeps the
    preflight usable from source, devel and install spaces.
    """
    validator_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "validate_tracking_config.py")
    spec = importlib.util.spec_from_file_location(
        "xd_uav_track_config_validator", validator_path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load config validator from %s" % validator_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_validator = _load_validator_module()
Report = _validator.Report
_load = _validator._load
merge_mappings = _validator.merge_mappings
validate_detector = _validator.validate_detector
validate_tracker = _validator.validate_tracker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track-config", action="append", default=[], metavar="YAML")
    parser.add_argument("--detector-config", action="append", default=[], metavar="YAML")
    parser.add_argument("--strict-metric", action="store_true")
    parser.add_argument("--node-package", required=True)
    parser.add_argument("--node-type", required=True)
    args, ros_arguments = parser.parse_known_args()
    report = Report()
    tracker_config = {}
    for path in args.track_config:
        tracker_config = merge_mappings(tracker_config, _load(path, report))
    if args.track_config:
        validate_tracker(tracker_config, report, " + ".join(args.track_config))
    for path in args.detector_config:
        validate_detector(_load(path, report), report, path, args.strict_metric)
    for text in report.warnings:
        print("WARNING: " + text, file=sys.stderr)
    if report.errors:
        for text in report.errors:
            print("ERROR: " + text, file=sys.stderr)
        return 2
    nodes = roslib.packages.find_node(args.node_package, args.node_type)
    if not nodes:
        print("ERROR: cannot find %s/%s" %
              (args.node_package, args.node_type), file=sys.stderr)
        return 3
    executable = nodes[0]
    os.execv(executable, [executable] + ros_arguments)
    return 4


if __name__ == "__main__":
    sys.exit(main())
