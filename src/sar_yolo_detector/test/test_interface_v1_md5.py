#!/usr/bin/env python3
"""Fail CI when the frozen ROS1 v1 wire contract changes accidentally."""

import json
import os
import subprocess
import unittest


class InterfaceV1Md5Test(unittest.TestCase):
    def test_frozen_wire_hashes(self):
        directory = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(directory, "interface_v1_md5.json"), encoding="utf-8") as stream:
            expected = json.load(stream)
        for key, expected_hash in sorted(expected.items()):
            kind, name = key.split("/", 1)
            command = "rosmsg" if kind == "msg" else "rossrv"
            actual = subprocess.check_output(
                [command, "md5", "sar_yolo_detector/" + name],
                text=True,
            ).strip()
            self.assertEqual(
                actual,
                expected_hash,
                "{} changed: bump protocol/package major or add a v2 type".format(key),
            )


if __name__ == "__main__":
    unittest.main()
