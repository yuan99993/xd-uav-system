#!/usr/bin/env python3
import unittest
from unittest import mock

import cv2
import numpy as np

from xd_uav_track.reid.encoder import AppearanceEncoder


class ReIDEncoderTest(unittest.TestCase):
    def setUp(self):
        self.encoder = AppearanceEncoder({
            "default_backend": "hybrid",
            "minimum_roi_width_px": 16,
            "minimum_roi_height_px": 16,
            "edge_margin_px": 2,
            "class_profiles": {
                "vehicle": {"class_ids": [0, 1, 2], "backend": "hybrid"},
            },
        })

    @staticmethod
    def vehicle(colour, reverse=False):
        image = np.zeros((96, 128, 3), dtype=np.uint8)
        cv2.rectangle(image, (24, 20), (104, 76), colour, -1)
        endpoints = ((25, 70), (100, 25)) if reverse else ((25, 25), (100, 70))
        cv2.line(image, endpoints[0], endpoints[1], (240, 240, 240), 4)
        return image

    def test_hybrid_descriptor_is_lighting_tolerant_and_discriminative(self):
        original = self.vehicle((30, 80, 210))
        darker = np.clip(original.astype(np.float32) * 0.45, 0, 255).astype(np.uint8)
        different = self.vehicle((210, 80, 30), reverse=True)
        first = self.encoder.encode(original, (20, 16, 108, 80), 0)
        second = self.encoder.encode(darker, (20, 16, 108, 80), 0)
        third = self.encoder.encode(different, (20, 16, 108, 80), 0)
        self.assertEqual(first.shape, (170,))
        self.assertGreater(float(np.dot(first, second)), 0.90)
        self.assertLess(float(np.dot(first, third)), 0.75)

    def test_small_and_edge_crops_do_not_pollute_gallery(self):
        image = self.vehicle((30, 80, 210))
        self.assertIsNone(self.encoder.encode(image, (0, 0, 10, 10), 0))
        self.assertIsNone(self.encoder.encode(image, (0, 10, 40, 60), 0))

    def test_batch_api_is_identical_to_individual_hybrid_encoding(self):
        image = self.vehicle((30, 80, 210))
        observations = [
            ((20, 16, 108, 80), 0),
            ((24, 20, 104, 76), 1),
            ((0, 0, 10, 10), 2),
        ]
        individual = [self.encoder.encode(image, box, class_id)
                      for box, class_id in observations]
        batched = self.encoder.encode_many(image, observations)
        self.assertEqual(len(individual), len(batched))
        for expected, actual in zip(individual, batched):
            if expected is None:
                self.assertIsNone(actual)
            else:
                np.testing.assert_array_equal(expected, actual)

    def test_onnx_profile_batches_compatible_rois_once(self):
        image = self.vehicle((30, 80, 210))

        class FakeOnnxModel:
            calls = 0

            def __init__(self, _profile):
                pass

            def encode_many(self, rois):
                FakeOnnxModel.calls += 1
                return [np.asarray([float(index + 1), 1.0], dtype=np.float32)
                        for index, _ in enumerate(rois)]

            def close(self):
                pass

        config = {
            "default_backend": "histogram",
            "minimum_roi_width_px": 8,
            "minimum_roi_height_px": 8,
            "class_profiles": {
                "vehicle": {"class_ids": [0, 1], "backend": "onnx"},
            },
        }
        with mock.patch(
                "xd_uav_track.reid.encoder.OnnxReIDModel", FakeOnnxModel):
            encoder = AppearanceEncoder(config)
            features = encoder.encode_many(image, [
                ((20, 16, 108, 80), 0),
                ((24, 20, 104, 76), 1),
            ])
            encoder.close()
        self.assertEqual(FakeOnnxModel.calls, 1)
        self.assertEqual(2, len(features))
        self.assertTrue(all(feature is not None for feature in features))

    def test_failed_vehicle_deep_profile_falls_back_to_hybrid(self):
        image = self.vehicle((30, 80, 210))
        config = {
            "minimum_roi_width_px": 8,
            "minimum_roi_height_px": 8,
            "class_profiles": {
                "vehicle": {
                    "class_ids": [0],
                    "backend": "onnx",
                    "fallback_backend": "hybrid",
                },
            },
        }
        with mock.patch(
                "xd_uav_track.reid.encoder.OnnxReIDModel",
                side_effect=RuntimeError("synthetic model failure")):
            encoder = AppearanceEncoder(config)
            first = encoder.encode(image, (20, 16, 108, 80), 0)
            second = encoder.encode(image, (20, 16, 108, 80), 0)
            encoder.close()
        self.assertEqual((170,), first.shape)
        np.testing.assert_array_equal(first, second)

    def test_class_profile_scales_appearance_evidence_quality(self):
        image = self.vehicle((30, 80, 210))
        config = {
            "minimum_roi_width_px": 8,
            "minimum_roi_height_px": 8,
            "class_profiles": {
                "vehicle": {
                    "class_ids": [0], "backend": "hybrid",
                    "association_weight": 1.0,
                },
                "person": {
                    "class_ids": [1], "backend": "hybrid",
                    "association_weight": 0.35,
                },
            },
        }
        encoder = AppearanceEncoder(config)
        observations = [
            ((20, 16, 108, 80), 0),
            ((20, 16, 108, 80), 1),
        ]
        _, qualities = encoder.encode_many_with_quality(image, observations)
        encoder.close()
        self.assertGreater(qualities[0], 0.0)
        self.assertAlmostEqual(qualities[1], qualities[0] * 0.35, places=6)


if __name__ == "__main__":
    unittest.main()
