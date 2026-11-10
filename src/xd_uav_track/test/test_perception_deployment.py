#!/usr/bin/env python3
"""Portable configuration validation; no ROS master, camera or GPU required."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("deployment_validator",
                                           ROOT / "scripts/validate_tracking_config.py")
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)


class DeploymentTest(unittest.TestCase):
    def setUp(self):
        self.report = validator.Report()
        self.config = validator._load(str(ROOT / "config/perception_deployment.yaml"), self.report)

    def errors(self, config):
        report = validator.Report()
        validator.validate_perception_deployment(config, report, "test")
        return report.errors

    def test_default_is_complete_and_has_no_host_specific_paths(self):
        self.assertEqual([], self.report.errors)
        self.assertEqual([], self.errors(self.config))
        self.assertNotIn("/home/promise", str(self.config))

    def test_source_and_original_image_contracts_fail_closed(self):
        for change in ("topic", "source", "duplicate", "allowlist", "overlay", "frame"):
            config = deepcopy(self.config)
            if change == "topic": config["perception_mode_manager"]["sources"][0]["image_topic"] = "wrong"
            if change == "source": config["perception_mode_manager"]["sources"][0]["image_source"] = "wrong"
            if change == "duplicate": config["sar_yolo_multi_source_detection"]["sources"].append(
                config["sar_yolo_multi_source_detection"]["sources"][0])
            if change == "allowlist": config["track"]["tracker"]["sources"]["allowed"] = []
            if change == "overlay": config["fixed_perception_overlay"]["input_image_topic"] = "resized"
            if change == "frame": config["perception_mode_manager"]["body_frd_frame"] = "camera"
            with self.subTest(change=change): self.assertTrue(self.errors(config))

    def test_flight_enable_invalid_mode_and_unbounded_cache_are_rejected(self):
        for node, path, value in (("track", ("runtime", "perception_only"), False),
                                 ("track", ("runtime", "publish_control_reference"), True),
                                 ("track", ("tracker", "sources", "maximum_count"), 99),
                                 ("perception_mode_manager", ("initial_mode",), "follow"),
                                 ("perception_mode_manager", ("frame_cache_size",), 999),
                                 ("perception_mode_manager", ("trajectory", "maximum_points"), 99999)):
            config = deepcopy(self.config)
            destination = config[node]
            for part in path[:-1]: destination = destination[part]
            destination[path[-1]] = value
            with self.subTest(path=path): self.assertTrue(self.errors(config))

    def test_duplicate_yaml_keys_are_not_silently_overwritten(self):
        with self.assertRaises(validator.DuplicateKeyError):
            validator.yaml.load("runtime: {x: 1}\nruntime: {x: 2}\n", Loader=validator.UniqueKeyLoader)

    def test_single_camera_configuration_uses_same_code(self):
        config = deepcopy(self.config)
        config["sar_yolo_multi_source_detection"]["sources"].pop()
        config["perception_mode_manager"]["sources"].pop()
        config.pop("gimbal_perception_overlay")
        config["track"]["tracker"]["sources"]["allowed"] = ["fixed_rgb"]
        config["track"]["tracker"]["sources"]["maximum_count"] = 1
        self.assertEqual([], self.errors(config))


if __name__ == "__main__":
    unittest.main()
