#!/usr/bin/env python3
"""Unit coverage for dynamic terrain-fleet allocation."""

import unittest

from xd_uav_planning.terrain_fleet import (
    FleetAllocator,
    slot_for_instance,
    validate_removal_state,
)


class TerrainFleetTest(unittest.TestCase):
    def test_repository_port_layout_is_unique_and_predictable(self):
        first = slot_for_instance(0)
        fourth = slot_for_instance(3)
        self.assertEqual(1, first.system_id)
        self.assertEqual(14560, first.mavlink_udp_port)
        self.assertEqual(4560, first.mavlink_tcp_port)
        self.assertEqual("udp://:14540@localhost:14580", first.mavros_fcu_url)
        self.assertEqual(4, fourth.system_id)
        self.assertEqual(14563, fourth.mavlink_udp_port)
        self.assertEqual("udp://:14543@localhost:14583", fourth.mavros_fcu_url)

    def test_allocator_reuses_released_lowest_slot(self):
        allocator = FleetAllocator(capacity=4)
        self.assertEqual(0, allocator.allocate("uav1").instance)
        self.assertEqual(1, allocator.allocate("uav2").instance)
        allocator.release("uav1")
        self.assertEqual(0, allocator.allocate("uav3").instance)

    def test_duplicate_invalid_and_full_requests_fail_closed(self):
        allocator = FleetAllocator(capacity=2)
        allocator.allocate("uav1")
        with self.assertRaisesRegex(ValueError, "already exists"):
            allocator.allocate("uav1")
        with self.assertRaisesRegex(ValueError, "lowercase ROS"):
            allocator.allocate("/uav2")
        allocator.allocate("uav2")
        with self.assertRaisesRegex(ValueError, "capacity"):
            allocator.allocate("uav3")

    def test_preferred_instance_is_range_and_collision_checked(self):
        allocator = FleetAllocator(capacity=4)
        self.assertEqual(3, allocator.allocate("uav4", 3).instance)
        with self.assertRaisesRegex(ValueError, "already allocated"):
            allocator.allocate("uav3", 3)
        with self.assertRaisesRegex(ValueError, "outside"):
            allocator.allocate("uav5", 4)

    def test_removal_requires_connected_disarmed_ground_state(self):
        validate_removal_state(True, False, True)
        with self.assertRaisesRegex(ValueError, "unavailable"):
            validate_removal_state(False, False, True)
        with self.assertRaisesRegex(ValueError, "armed"):
            validate_removal_state(True, True, True)
        with self.assertRaisesRegex(ValueError, "ground"):
            validate_removal_state(True, False, False)


if __name__ == "__main__":
    unittest.main()
