#!/usr/bin/env python3
"""Model identity must describe the loaded artifact, including fallback."""

import copy
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

from diagnostic_msgs.msg import DiagnosticStatus
from genpy import Time

from sar_yolo_detector.model_runtime_status import build_model_status
from sar_yolo_detector.msg import DetectorModelInfo
from sar_yolo_detector.srv import ListDetectorModelsResponse


class ModelRuntimeStatusTest(unittest.TestCase):
    def test_public_registry_list_preserves_the_snapshot_active_model(self):
        # Compile the actual method without importing the GPU frontend runtime.
        script = Path(__file__).resolve().parents[1] / "scripts/multi_source_detection_node.py"
        tree = ast.parse(script.read_text(encoding="utf-8"))
        method = next(function for item in tree.body if isinstance(item, ast.ClassDef)
                      for function in item.body if isinstance(function, ast.FunctionDef)
                      and function.name == "_list_models")
        namespace = {"DetectorModelInfo": DetectorModelInfo,
                     "ListDetectorModelsResponse": ListDetectorModelsResponse}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(script), "exec"), namespace)
        registry = {model: {"pt_path": model + ".pt", "pt_sha256": "a" * 64,
                            "class_names": ["tank"], "input_size": 640,
                            "engine_prepared": True, "engine_path": model + ".engine",
                            "engine_sha256": "b" * 64}
                    for model in ("train", "train4", "train7")}
        for active in registry:
            instance = SimpleNamespace(_model_registry=registry,
                _model_snapshot=({"model_path": active + ".engine",
                                  "artifact_sha256": "b" * 64,
                                  "backend": "tensorrt_fp16"}, active, "a" * 64,
                                 11, ["tank"], active + ".pt"))
            response = namespace["_list_models"](instance, None)
            self.assertEqual(active, response.active_model_id)
            self.assertEqual(11, response.model_epoch)
            self.assertEqual(sorted(registry), [item.model_id for item in response.models])

    def snapshot(self, runtime, **kwargs):
        message = build_model_status(
            runtime, node_name="/demo/detector", stamp=Time(12, 34), **kwargs)
        self.assertEqual(Time(12, 34), message.header.stamp)
        self.assertEqual(1, len(message.status))
        status = message.status[0]
        self.assertEqual("/demo/detector: model_runtime", status.name)
        return status, {entry.key: entry.value for entry in status.values}

    def test_reports_actual_fallback_artifact_instead_of_requested_model(self):
        runtime = {
            "model_path": "/models/cpu_fallback.pt",
            "artifact_sha256": "A" * 64,
            "backend": "cpu_torch",
            "model_format": "ultralytics_pt",
            "requested_device": "cuda",
            "effective_device": "cpu",
            "fallback_occurred": True,
            "fallback_reason": "CUDA load failed",
        }
        before = copy.deepcopy(runtime)
        status, values = self.snapshot(runtime, startup_ready=True,
                                       configured_model_path="/models/requested.pt")
        self.assertEqual(DiagnosticStatus.OK, status.level)
        self.assertEqual("/models/cpu_fallback.pt", values["loaded_model_path"])
        self.assertEqual("/models/requested.pt", values["configured_model_path"])
        self.assertEqual("sha256:" + "a" * 64, values["artifact_id"])
        self.assertEqual("cpu", values["effective_device"])
        self.assertEqual("true", values["fallback_occurred"])
        self.assertEqual(before, runtime)

    def test_engine_digest_is_not_source_weight_digest(self):
        _, values = self.snapshot({
            "model_path": "/cache/host.engine", "artifact_sha256": "b" * 64,
            "model_format": "tensorrt_engine", "backend": "tensorrt_fp16",
        }, configured_model_path="/models/source.pt", startup_ready=True)
        self.assertEqual("host.engine", values["loaded_model_name"])
        self.assertEqual("b" * 64, values["loaded_artifact_sha256"])
        self.assertEqual("tensorrt_engine", values["model_format"])

    def test_missing_or_invalid_digest_does_not_use_expected_digest(self):
        for digest in (None, "", "bad", "a" * 63, "z" * 64):
            with self.subTest(digest=digest):
                status, values = self.snapshot({
                    "model_path": "/models/renamed.pt", "artifact_sha256": digest,
                    "expected_sha256": "c" * 64,
                }, startup_ready=True)
                self.assertEqual(DiagnosticStatus.WARN, status.level)
                self.assertEqual("", values["artifact_id"])
                self.assertEqual("", values["loaded_artifact_sha256"])
                self.assertEqual("false", values["artifact_digest_available"])

    def test_readiness_and_failure_are_not_inferred_from_filename(self):
        runtime = {"model_path": "/models/ready.pt", "artifact_sha256": "d" * 64}
        _, values = self.snapshot(runtime)
        self.assertEqual("WARMING_UP", values["state"])
        self.assertEqual("false", values["startup_ready"])
        status, values = self.snapshot(runtime, startup_ready=True,
                                       startup_error="warmup failed")
        self.assertEqual(DiagnosticStatus.ERROR, status.level)
        self.assertEqual("ERROR", values["state"])
        self.assertEqual("false", values["startup_ready"])

    def test_missing_runtime_path_is_not_reported_as_ready(self):
        status, values = self.snapshot({}, startup_ready=True)
        self.assertEqual(DiagnosticStatus.ERROR, status.level)
        self.assertEqual("UNAVAILABLE", values["state"])
        self.assertEqual("false", values["startup_ready"])


if __name__ == "__main__":
    unittest.main()
