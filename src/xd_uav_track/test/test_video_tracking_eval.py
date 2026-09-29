#!/usr/bin/env python3
"""Offline evaluator must align by capture stamp, not tracker-local seq."""

import importlib.util
import contextlib
import io
import json
import os
import sys
import tempfile
import unittest


SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts",
                      "evaluate_video_tracking.py")
SPEC = importlib.util.spec_from_file_location("video_tracking_eval", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class VideoTrackingEvalTest(unittest.TestCase):
    def test_mot_frame_offset_and_resize(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".txt") as handle:
            handle.write("7,42,100,200,40,20,1,1,1\n")
            handle.flush()
            row = MODULE.read_gt(handle.name, offset=6, scale=0.5)[1][0]
            self.assertEqual(row["id"], 42)
            self.assertEqual(row["bbox"], [50, 100, 70, 110])

    def test_unique_iou_assignment(self):
        truth = [{"id": 1, "bbox": [0, 0, 10, 10]},
                 {"id": 2, "bbox": [20, 0, 30, 10]}]
        tracks = [{"id": 8, "bbox": [0, 0, 10, 10]},
                  {"id": 9, "bbox": [20, 0, 30, 10]}]
        self.assertEqual(set(MODULE.matches(truth, tracks, 0.5)),
                         {(1, 8), (2, 9)})

    def test_full_report_uses_camera_frame_marker(self):
        with tempfile.TemporaryDirectory() as directory:
            gt_path = os.path.join(directory, "gt.txt")
            record_path = os.path.join(directory, "record.jsonl")
            with open(gt_path, "w") as handle:
                handle.write("1,42,0,0,10,10,1,1,1\n")
                handle.write("2,42,0,0,10,10,1,1,1\n")
            rows = []
            for frame, track_id in ((1, 7), (2, 9)):
                stamp = float(frame)
                rows.extend((
                    {"type": "frame", "t": stamp, "source": "replay_fixed",
                     "frame": frame},
                    {"type": "detections", "t": stamp, "source": "replay_fixed",
                     "frame": frame, "objects": [{"bbox": [0, 0, 10, 10]}]},
                    {"type": "tracks", "t": stamp, "source": "replay_fixed",
                     "frame": frame - 1, "objects": [{"id": track_id,
                     "bbox": [0, 0, 10, 10], "state": "confirmed",
                     "detected": True}]},
                ))
            with open(record_path, "w") as handle:
                for row in rows:
                    handle.write(json.dumps(row) + "\n")
            output = io.StringIO()
            old_argv = sys.argv
            try:
                sys.argv = [SCRIPT, record_path, "--gt-a", gt_path]
                with contextlib.redirect_stdout(output):
                    MODULE.main()
            finally:
                sys.argv = old_argv
            report = json.loads(output.getvalue())
            self.assertEqual(report["replayed_frames"], 2)
            self.assertEqual(report["detection_recall"], 1.0)
            self.assertEqual(report["id_switches_observed"], 1)


if __name__ == "__main__":
    unittest.main()
