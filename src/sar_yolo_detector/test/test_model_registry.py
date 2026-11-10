#!/usr/bin/env python3
"""The web model list must be content-addressed and deterministic."""

from pathlib import Path
import tempfile
import unittest

import yaml

from sar_yolo_detector.model_registry import identify_model, load_model_registry


class ModelRegistryTest(unittest.TestCase):
    def test_workspace_registry_resolves_all_three_vehicle_models(self):
        package = Path(__file__).resolve().parents[1]
        models = load_model_registry(package / "config/model_registry.yaml", package)
        self.assertEqual({"train", "train4", "train7"}, set(models))
        self.assertEqual("train7", identify_model(
            models, "8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54"))
        self.assertTrue(models["train7"]["engine_prepared"])
        self.assertEqual(["car", "ar-car", "tank", "m142"],
                         models["train7"]["class_names"])

    def test_digest_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "model.pt").write_bytes(b"model")
            registry = {"schema_version": 1, "models": {"bad": {
                "pt_path": "model.pt", "pt_sha256": "0" * 64,
                "class_names": ["tank"]}}}
            path = root / "registry.yaml"
            path.write_text(yaml.safe_dump(registry), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                load_model_registry(path, root)


if __name__ == "__main__":
    unittest.main()
