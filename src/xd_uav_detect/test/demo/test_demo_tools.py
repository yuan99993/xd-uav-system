#!/usr/bin/env python3
"""Smoke tests for the optional target-generation and red-box demo tools."""

import os
import importlib.util
import subprocess
import sys
import unittest

import cv2
import numpy


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEMO_DIR = os.path.join(PACKAGE_ROOT, "scripts", "demo")


def load_demo_module(module_name):
    path = os.path.join(DEMO_DIR, module_name + ".py")
    specification = importlib.util.spec_from_file_location(
        "xd_uav_detect_test_" + module_name, path
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class DemoToolsTest(unittest.TestCase):

    def run_tool(self, script_name, *arguments):
        command = [sys.executable, os.path.join(DEMO_DIR, script_name)]
        command.extend(arguments)
        return subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=15,
        )

    def test_all_upstream_tools_have_working_help(self):
        for script_name in (
            "red_box_detector.py",
            "spawn_red_boxes.py",
            "spawn_random_vehicles.py",
            "spawn_yolo_vehicle_targets.py",
        ):
            with self.subTest(script=script_name):
                result = self.run_tool(script_name, "--help")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("usage:", result.stdout.lower())

    def test_red_box_explicit_dry_run_does_not_require_gazebo(self):
        result = self.run_tool(
            "spawn_red_boxes.py",
            "--position=5,6",
            "--size",
            "2",
            "2",
            "1",
            "--ground-z",
            "0",
            "--dry-run",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("red_box_1: x=5.000, y=6.000, z=0.500", result.stdout)

    def test_red_box_detector_extracts_a_red_region(self):
        module = load_demo_module("red_box_detector")
        detector = module.MultiUavRedBoxDetector.__new__(
            module.MultiUavRedBoxDetector
        )
        detector._hue_low_1 = 0
        detector._hue_high_1 = 12
        detector._hue_low_2 = 168
        detector._hue_high_2 = 179
        detector._saturation_min = 100
        detector._value_min = 60
        detector._minimum_area_px = 100.0
        detector._minimum_width_px = 5
        detector._minimum_height_px = 5
        detector._maximum_targets = 0
        detector._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        detector._morphology_iterations = 1

        image = numpy.zeros((100, 120, 3), dtype=numpy.uint8)
        cv2.rectangle(image, (20, 30), (70, 80), (0, 0, 255), -1)
        boxes = detector._valid_boxes(detector._make_red_mask(image))

        self.assertEqual(len(boxes), 1)
        x, y, width, height = boxes[0]
        self.assertLessEqual(abs(x - 20), 1)
        self.assertLessEqual(abs(y - 30), 1)
        self.assertGreaterEqual(width, 50)
        self.assertGreaterEqual(height, 50)


if __name__ == "__main__":
    unittest.main()
