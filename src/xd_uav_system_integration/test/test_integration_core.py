#!/usr/bin/env python3

import math
import unittest

from xd_uav_system_integration.core import (
    OdometrySample, ReferenceSample, health_conjunction,
    rigid_body_imu_to_body, switch_delta,
    validate_reference, validate_shadow_odometry)


class IntegrationCoreTest(unittest.TestCase):
    def test_rigid_body_imu_lever_arm_compensation(self):
        identity = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0)
        acceleration, omega = rigid_body_imu_to_body(
            (0.0, 1.0, -1.0), (0.0, 0.0, 2.0),
            (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), identity)
        self.assertEqual((0.0, 0.0, 2.0), omega)
        self.assertAlmostEqual(4.0, acceleration[0])
        self.assertAlmostEqual(0.0, acceleration[1])
        self.assertAlmostEqual(-1.0, acceleration[2])

    @staticmethod
    def odometry(**updates):
        values = dict(
            stamp=10.0, parent_frame="uav1/fastlio_origin",
            child_frame="uav1/mid360_imu", position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            linear_velocity=(0.1, 0.0, 0.0),
            pose_variance=(0.1, 0.1, 0.1),
            velocity_variance=(0.2, 0.2, 0.2))
        values.update(updates)
        return OdometrySample(**values)

    def test_fastlio_shadow_contract_fails_closed(self):
        validate = lambda sample: validate_shadow_odometry(
            sample, 10.1, "uav1/fastlio_origin", "uav1/mid360_imu",
            0.3, 0.02, 0.5)
        self.assertTrue(validate(self.odometry()).valid)
        cases = [
            (self.odometry(parent_frame="world"),
             "shadow_parent_frame_mismatch"),
            (self.odometry(child_frame="uav1/base_link"),
             "shadow_child_frame_mismatch"),
            (self.odometry(stamp=9.0), "shadow_stale"),
            (self.odometry(linear_velocity=(math.nan, 0.0, 0.0)),
             "shadow_not_finite"),
            (self.odometry(orientation=(0.0, 0.0, 0.0, 0.0)),
             "shadow_orientation_not_normalized"),
            (self.odometry(pose_variance=(0.0, 0.1, 0.1)),
             "shadow_covariance_not_positive"),
            (self.odometry(linear_velocity=(0.6, 0.0, 0.0)),
             "shadow_ground_speed_exceeded"),
        ]
        for sample, reason in cases:
            self.assertEqual(reason, validate(sample).reason)

    @staticmethod
    def reference(**updates):
        values = dict(
            stamp=10.0, frame_id="world", coordinate_frame=1, type_mask=0,
            position=(1.0, 2.0, 3.0), velocity=(0.1, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0), yaw=0.0, yaw_rate=0.0)
        values.update(updates)
        return ReferenceSample(**values)

    def test_reference_contract(self):
        self.assertTrue(validate_reference(
            self.reference(), 10.1, "world", 0.2, 0.02).valid)
        cases = [
            (self.reference(frame_id="odom"), "reference_frame_mismatch"),
            (self.reference(coordinate_frame=8), "coordinate_frame_unsupported"),
            (self.reference(type_mask=4096),
             "reference_type_mask_unknown_bits"),
            (self.reference(type_mask=512), "force_mode_unsupported"),
            (self.reference(type_mask=511),
             "reference_has_no_enabled_field"),
            (self.reference(yaw=math.nan), "reference_not_finite"),
            (self.reference(stamp=9.0), "reference_stale"),
        ]
        for sample, reason in cases:
            result = validate_reference(sample, 10.1, "world", 0.2, 0.02)
            self.assertFalse(result.valid)
            self.assertEqual(result.reason, reason)

        masked_nan = self.reference(type_mask=8, velocity=(math.nan, 0.0, 0.0))
        self.assertTrue(validate_reference(
            masked_nan, 10.1, "world", 0.2, 0.02).valid)
        enabled_nan = self.reference(position=(math.nan, 2.0, 3.0))
        self.assertEqual(validate_reference(
            enabled_nan, 10.1, "world", 0.2, 0.02).reason,
            "reference_not_finite")

    def test_switch_jump_and_health_gate(self):
        previous = self.reference()
        self.assertTrue(switch_delta(
            previous, self.reference(position=(1.1, 2.0, 3.0)),
            0.2, 0.3).valid)
        self.assertEqual(switch_delta(
            previous, self.reference(position=(1.3, 2.0, 3.0)),
            0.2, 0.3).reason, "switch_position_jump")
        self.assertTrue(health_conjunction([True, True], 2))
        self.assertFalse(health_conjunction([True, False], 2))
        self.assertFalse(health_conjunction([], 1))


if __name__ == "__main__":
    unittest.main()
