#!/usr/bin/env python3
"""Unit coverage for the portable PixEagle/SmartTracker transplant."""

import hashlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

import sar_yolo_detector.pixeagle.smart_tracker as smart_tracker_module
from sar_yolo_detector.pixeagle.backends.ultralytics_backend import UltralyticsBackend
from sar_yolo_detector.pixeagle.deep_reid_model import DeepReIDModel
from sar_yolo_detector.pixeagle.appearance_model import AppearanceModel
from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection
from sar_yolo_detector.pixeagle.geometry_utils import (
    obb_xywhr_to_aabb,
    point_in_polygon,
)
from sar_yolo_detector.pixeagle.parameters import Parameters
from sar_yolo_detector.pixeagle.smart_tracker import SmartTracker
from sar_yolo_detector.pixeagle.tracking_roi import (
    TrackingROIError,
    tracking_point_to_pixels,
    tracking_roi_to_pixels,
)

try:
    import torch  # noqa: F401
    import torchreid  # noqa: F401
    DEEP_REID_TEST_AVAILABLE = True
except Exception:
    # ROS system-Python test jobs may intentionally install detector-only
    # dependencies.  The executable deep test runs in the SmartTracker GPU
    # environment, while those jobs should retain their existing coverage.
    DEEP_REID_TEST_AVAILABLE = False


class _FakeBackend:
    tracker_type_str = "botsort"
    use_custom_reid = False
    tracker_args = {"persist": True, "verbose": False}
    backend_name = "fake"
    is_available = True

    def __init__(self, detections):
        self.detections = list(detections)
        self.fail = False
        self.unloaded = False

    def load_model(self, **_kwargs):
        return {
            "model_path": "/trusted/fake.pt",
            "model_name": "fake.pt",
            "backend": "fake_cpu",
            "effective_device": "cpu",
            "artifact_sha256": "0" * 64,
            "fallback_occurred": False,
        }

    def unload_model(self):
        self.unloaded = True

    def get_model_labels(self):
        return {0: "person", 2: "car", 25: "umbrella"}

    def get_model_task(self):
        return "detect"

    def detect_and_track(self, *_args, **_kwargs):
        if self.fail:
            raise RuntimeError("synthetic inference failure")
        return "detect", list(self.detections)

    def detect(self, *_args, **_kwargs):
        return self.detect_and_track(*_args, **_kwargs)


class _Controller:
    def __init__(self):
        self.video_handler = SimpleNamespace(width=160, height=120)
        self.current_frame = None
        self.tracker = None
        self.tracking_started = False


class PixEaglePortableTest(unittest.TestCase):
    @unittest.skipUnless(DEEP_REID_TEST_AVAILABLE, "torchreid is not installed")
    def test_deep_reid_checkpoint_and_embedding_contract(self):
        """Exercise the bundled, verified OSNet checkpoint once (no download)."""
        checkpoint = (
            Path(__file__).resolve().parents[1]
            / "models"
            / "person_reid"
            / "osnet_x0_25_market1501.pt"
        )
        digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        model = DeepReIDModel(
            {
                "DEEP_REID_MODEL_PATH": str(checkpoint),
                "DEEP_REID_MODEL_SHA256": digest,
                "DEEP_REID_DEVICE": "cpu",
            }
        )
        frame = np.zeros((320, 240, 3), dtype=np.uint8)
        embedding = model.extract_features(frame, (20, 20, 180, 300))
        self.assertIsNotNone(embedding)
        self.assertEqual(embedding.shape, (model.embedding_dimension,))
        self.assertTrue(np.isfinite(embedding).all())
        self.assertAlmostEqual(float(np.linalg.norm(embedding)), 1.0, places=4)

    @unittest.skipUnless(DEEP_REID_TEST_AVAILABLE, "torchreid is not installed")
    def test_appearance_model_deep_feature_type_does_not_require_hog(self):
        checkpoint = (
            Path(__file__).resolve().parents[1]
            / "models"
            / "person_reid"
            / "osnet_x0_25_market1501.pt"
        )
        appearance = AppearanceModel(
            {
                "APPEARANCE_FEATURE_TYPE": "deep",
                "DEEP_REID_MODEL_PATH": str(checkpoint),
                "DEEP_REID_MODEL_SHA256": hashlib.sha256(
                    checkpoint.read_bytes()
                ).hexdigest(),
                "DEEP_REID_DEVICE": "cpu",
            }
        )
        feature = appearance.extract_features(
            np.zeros((160, 120, 3), dtype=np.uint8), (10, 10, 100, 150)
        )
        self.assertIsNotNone(feature)
        self.assertEqual(feature.shape, (appearance.deep_model.embedding_dimension,))

    def test_roi_and_geometry_contracts(self):
        self.assertEqual(
            tracking_roi_to_pixels(
                x=0.1,
                y=0.2,
                width=0.5,
                height=0.5,
                coordinate_space="normalized",
                frame_width=640,
                frame_height=480,
            ),
            {"x": 64, "y": 96, "width": 320, "height": 240},
        )
        self.assertEqual(
            tracking_point_to_pixels(
                x=1.0,
                y=1.0,
                coordinate_space="normalized",
                frame_width=640,
                frame_height=480,
            ),
            (639, 479),
        )
        with self.assertRaises(TrackingROIError):
            tracking_roi_to_pixels(
                x=0.99,
                y=0.99,
                width=0.1,
                height=0.1,
                coordinate_space="normalized",
                frame_width=640,
                frame_height=480,
            )
        self.assertEqual(obb_xywhr_to_aabb((50.0, 40.0, 20.0, 10.0, 0.0)), (40, 35, 60, 45))
        self.assertTrue(point_in_polygon((50.0, 40.0), [(40, 35), (60, 35), (60, 45), (40, 45)]))

    def test_ultralytics_normalization_preserves_class_zero(self):
        boxes = SimpleNamespace(
            data=[1],
            xyxy=[[10.0, 20.0, 30.0, 50.0]],
            conf=[0.8],
            cls=[0.0],
            id=None,
        )
        mode, detections = UltralyticsBackend._normalize_results(
            [SimpleNamespace(boxes=boxes, obb=None)]
        )
        self.assertEqual(mode, "detect")
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].class_id, 0)
        self.assertFalse(detections[0].track_id_is_stable)

    def test_model_integrity_accepts_only_matching_digest(self):
        payload = b"trusted-model-fixture"
        expected = hashlib.sha256(payload).hexdigest()
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / "fixture.pt"
            model.write_bytes(payload)
            model.chmod(0o600)
            backend = UltralyticsBackend(
                {
                    "SMART_TRACKER_MODELS_ROOT": directory,
                    "SMART_TRACKER_REQUIRE_MODEL_SHA256": True,
                    "SMART_TRACKER_MODEL_SHA256_BY_NAME": {"fixture.pt": expected},
                }
            )
            provenance = backend._verify_file(model)
            self.assertTrue(provenance["verified"])
            self.assertEqual(provenance["sha256"], expected)

            backend.authorize_model_digest(str(model), "f" * 64)
            with self.assertRaisesRegex(ValueError, "mismatch"):
                backend._verify_file(model)

    def test_smart_tracker_selection_and_fail_closed_output(self):
        detection = NormalizedDetection(
            track_id=7,
            class_id=0,
            confidence=0.9,
            aabb_xyxy=(40, 30, 80, 70),
            center_xy=(60, 50),
            track_id_is_stable=True,
        )
        backend = _FakeBackend([detection])
        Parameters.configure_smart_tracker(
            {
                "DETECTION_BACKEND": "ultralytics",
                "SMART_TRACKER_USE_GPU": False,
                "SMART_TRACKER_CPU_MODEL_PATH": "/trusted/fake.pt",
                "SMART_TRACKER_SHOW_PASSIVE_LABELS": False,
                "ENABLE_PREDICTION_BUFFER": False,
                "ENABLE_KALMAN_FILTER": False,
                "TRACKING_STRATEGY": "hybrid",
            }
        )
        controller = _Controller()
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        controller.current_frame = frame

        with mock.patch.object(
            smart_tracker_module, "create_backend", return_value=backend
        ):
            tracker = SmartTracker(controller)
        try:
            tracker.track_and_draw(frame.copy())
            self.assertTrue(tracker.select_object_by_click(60, 50))
            tracker.track_and_draw(frame.copy())
            output = tracker.get_output()
            self.assertTrue(output.tracking_active)
            self.assertEqual(output.target_id, 7)
            self.assertTrue(output.raw_data["usable_for_following"])
            self.assertTrue(output.raw_data["selected_detected_this_frame"])

            backend.fail = True
            tracker.track_and_draw(frame.copy())
            failed_output = tracker.get_output()
            self.assertFalse(failed_output.raw_data["usable_for_following"])
            self.assertTrue(failed_output.raw_data["data_is_stale"])
        finally:
            tracker.close()
        self.assertTrue(backend.unloaded)

    def test_smart_tracker_filters_unapproved_classes_before_publication(self):
        detections = [
            NormalizedDetection(
                track_id=2,
                class_id=2,
                confidence=0.8,
                aabb_xyxy=(10, 10, 50, 40),
                center_xy=(30, 25),
                track_id_is_stable=True,
            ),
            NormalizedDetection(
                track_id=25,
                class_id=25,
                confidence=0.9,
                aabb_xyxy=(60, 20, 100, 70),
                center_xy=(80, 45),
                track_id_is_stable=True,
            ),
        ]
        backend = _FakeBackend(detections)
        Parameters.configure_smart_tracker(
            {
                "DETECTION_BACKEND": "ultralytics",
                "SMART_TRACKER_USE_GPU": False,
                "SMART_TRACKER_CPU_MODEL_PATH": "/trusted/fake.pt",
                "SMART_TRACKER_ALLOWED_CLASS_IDS": [2, 3, 5, 7],
                "SMART_TRACKER_SHOW_PASSIVE_LABELS": False,
                "ENABLE_PREDICTION_BUFFER": False,
                "ENABLE_KALMAN_FILTER": False,
                "TRACKING_STRATEGY": "hybrid",
            }
        )
        controller = _Controller()
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        controller.current_frame = frame

        with mock.patch.object(
            smart_tracker_module, "create_backend", return_value=backend
        ):
            tracker = SmartTracker(controller)
        try:
            tracker.track_and_draw(frame.copy())
            self.assertEqual(
                [detection.class_id for detection in tracker.last_detections],
                [2],
            )
        finally:
            tracker.close()


if __name__ == "__main__":
    unittest.main()
