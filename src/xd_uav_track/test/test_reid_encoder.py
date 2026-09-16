#!/usr/bin/env python3
import unittest

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


if __name__ == "__main__":
    unittest.main()
