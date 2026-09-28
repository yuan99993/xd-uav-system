#!/usr/bin/env python3

import math
import unittest

from xd_uav_planning.core import PointCloudSample, validate_pointcloud


class PointCloudCoreTest(unittest.TestCase):
    @staticmethod
    def sample(**updates):
        values = dict(stamp=10.0, frame_id="uav1/os", width=4, height=1,
                      point_step=16, data_size=64)
        values.update(updates)
        return PointCloudSample(**values)

    def test_accepts_fresh_well_formed_cloud(self):
        result = validate_pointcloud(
            self.sample(), 10.1, "uav1/os", 0.3, 0.02)
        self.assertTrue(result.valid)

    def test_rejects_frame_empty_stale_and_malformed(self):
        cases = [
            (self.sample(frame_id=""), "cloud_frame_empty"),
            (self.sample(frame_id="other"), "cloud_frame_mismatch"),
            (self.sample(width=0), "cloud_empty"),
            (self.sample(point_step=0), "cloud_layout_invalid"),
            (self.sample(data_size=63), "cloud_data_truncated"),
            (self.sample(stamp=9.0), "cloud_input_stale"),
            (self.sample(stamp=math.nan), "cloud_time_not_finite"),
        ]
        for sample, reason in cases:
            result = validate_pointcloud(
                sample, 10.1, "uav1/os", 0.3, 0.02)
            self.assertFalse(result.valid)
            self.assertEqual(result.reason, reason)


if __name__ == "__main__":
    unittest.main()
