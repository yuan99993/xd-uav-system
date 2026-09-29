#!/usr/bin/env python3
"""Check the production launch graph without starting Gazebo or a YOLO model."""

import subprocess
import unittest

import yaml


def launch_output(command, package, launch_file, *arguments):
    result = subprocess.run(
        ["roslaunch", command, package, launch_file, *arguments],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, timeout=20)
    return result.stdout


class VehiclePerceptionLaunchTest(unittest.TestCase):
    def test_single_production_entry_owns_one_detector(self):
        nodes = launch_output("--nodes", "xd_uav_track",
                              "vision_tracking_stack.launch").splitlines()
        self.assertEqual(1, sum("sar_yolo_multi_source_detection" in n
                                for n in nodes))
        self.assertIn("/uav1/detect_fixed", nodes)
        self.assertIn("/uav1/detect_gimbal_lrf", nodes)
        self.assertIn("/uav1/track", nodes)
        self.assertFalse(any("smart_tracker" in n for n in nodes))

    def test_detection_only_uses_same_runtime_without_tracker(self):
        nodes = launch_output("--nodes", "xd_uav_track",
                              "vision_tracking_stack.launch",
                              "enable_tracking:=false").splitlines()
        self.assertEqual(["/uav1/sar_yolo_multi_source_detection"], nodes)
        params = yaml.safe_load(launch_output(
            "--dump-params", "xd_uav_track", "vision_tracking_stack.launch",
            "enable_tracking:=false"))
        root = "/uav1/sar_yolo_multi_source_detection/"
        self.assertEqual("tensorrt_fp16", params[root + "inference_backend"])
        self.assertFalse(params[root + "pause_without_subscribers"])
        self.assertEqual("", params[root + "tracking_tracks_topic"])
        self.assertFalse(params[root + "sources"][0]["reid_enabled"])

    def test_dual_camera_still_shares_one_model_and_tracker(self):
        nodes = launch_output("--nodes", "xd_uav_track",
                              "tracking_fixedwing_dual_sensor.launch").splitlines()
        self.assertEqual(1, sum("sar_yolo_multi_source_detection" in n
                                for n in nodes))
        self.assertEqual(1, sum(n.endswith("/track") for n in nodes))
        self.assertIn("/uav1/detect_fixed", nodes)
        self.assertIn("/uav1/detect_gimbal_lrf", nodes)

    def test_camera_specific_compatibility_entries_keep_one_model(self):
        cases = (
            ("tracking_multirotor_fixed.launch", "detect_fixed", "detect_gimbal_lrf"),
            ("tracking_multirotor_gimbal_lrf.launch", "detect_gimbal_lrf", "detect_fixed"),
            ("tracking_fixedwing_fixed.launch", "detect_fixed", "detect_gimbal_lrf"),
        )
        for launch_file, wanted, unwanted in cases:
            with self.subTest(launch_file=launch_file):
                nodes = launch_output("--nodes", "xd_uav_track", launch_file).splitlines()
                self.assertEqual(1, sum("sar_yolo_multi_source_detection" in n
                                        for n in nodes))
                self.assertEqual(1, sum(n.endswith("/track") for n in nodes))
                self.assertIn("/uav1/" + wanted, nodes)
                self.assertNotIn("/uav1/" + unwanted, nodes)

    def test_standalone_vision_output_needs_no_xd_track_topic(self):
        params = yaml.safe_load(launch_output(
            "--dump-params", "sar_yolo_detector", "detection_only.launch"))
        root = "/sar_yolo_detection_only/"
        self.assertEqual("tensorrt_fp16", params[root + "inference_backend"])
        source, = params[root + "sources"]
        self.assertEqual("", source["xd_detections_topic"])
        self.assertTrue(source["publish_vision_detections"])
        self.assertFalse(source["reid_enabled"])


if __name__ == "__main__":
    unittest.main()
