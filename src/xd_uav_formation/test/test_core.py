#!/usr/bin/env python3

import math
import unittest

from xd_uav_formation.core import (
    acquisition_slots, blocking_action, catchup_velocity,
    consensus_speed_targets, consensus_velocity, formation_offsets,
    path_reference, path_reference_state, ring_neighbors, rounded_path, shared_center,
    slot_target, slot_velocity,
    validate_offset, virtual_center)


class Actions:
    ACTION_ARM = 1
    ACTION_DISARM = 2
    ACTION_TAKEOFF = 3
    ACTION_HOLD = 4
    ACTION_NAVIGATE = 5
    ACTION_LAND = 6
    ACTION_RETURN_HOME = 7
    ACTION_EMERGENCY = 8


class FormationCoreTest(unittest.TestCase):
    def test_slot_rotates_with_leader_heading(self):
        x, y, z, yaw = slot_target(
            (10.0, 20.0, 30.0), math.pi / 2,
            (-4.0, 2.0, 3.0), math.pi)
        self.assertAlmostEqual(x, 8.0)
        self.assertAlmostEqual(y, 16.0)
        self.assertAlmostEqual(z, 33.0)
        self.assertAlmostEqual(yaw, -math.pi / 2)

    def test_slot_spacing_rejects_same_position_and_nonfinite(self):
        for offset in ([0, 0, 0], [1, 0, 0], [math.nan, 0, 0]):
            with self.assertRaises(ValueError):
                validate_offset(offset, 2.0)
        self.assertEqual(validate_offset([-4, 0, 0], 2.0), (-4.0, 0.0, 0.0))

    def test_swiftwing_layouts_keep_distinct_slots(self):
        circle = formation_offsets("circle", 5, 20.0)
        self.assertEqual(len(circle), 5)
        self.assertAlmostEqual(circle[0][1], 20.0)
        self.assertTrue(all(math.hypot(x, y) >= 20.0 - 1e-9
                            for x, y, _ in circle))
        rectangle = formation_offsets("rectangle_compact", 5, 20.0)
        self.assertEqual(rectangle[0], (0.0, -20.0, 0.0))
        self.assertEqual(len(set(rectangle)), 5)

    def test_turning_leader_changes_slot_velocity(self):
        # v=(15,0), lateral a=(0,1.5) -> omega=0.1 rad/s.
        velocity = slot_velocity((15.0, 0.0, 0.0),
                                 (0.0, 1.5, 0.0), 0.0,
                                 (-80.0, 0.0, 0.0), 0.15)
        self.assertAlmostEqual(velocity[0], 15.0)
        self.assertAlmostEqual(velocity[1], -8.0)
        bank_rate = slot_velocity((15.0, 0.0, 0.0),
                                  (0.0, 0.0, 0.0), 0.0,
                                  (-80.0, 0.0, 0.0), 0.15,
                                  turn_rate=0.1)
        self.assertAlmostEqual(bank_rate[1], -8.0)

    def test_slot_error_changes_speed_within_airframe_limits(self):
        self.assertEqual(catchup_velocity(
            (15.0, 0.0, 0.0), (10.0, 0.0, 0.0),
            (0.0, 0.0, 0.0), 0.25, 3.0, 24.0, 11.0),
            (17.5, 0.0, 0.0))
        self.assertEqual(catchup_velocity(
            (15.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0), 0.25, 3.0, 24.0, 11.0),
            (12.5, 0.0, 0.0))
        self.assertEqual(catchup_velocity(
            (15.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0), 0.25, 3.0, 24.0, 11.0),
            (15.0, 0.0, 0.0))
        self.assertEqual(catchup_velocity(
            (4.0, 0.0, 0.0), (100.0, 0.0, 0.0),
            (0.0, 0.0, 0.0), 0.25, 3.0, 5.0),
            (5.0, 0.0, 0.0))
        self.assertEqual(catchup_velocity(
            (12.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (100.0, 0.0, 0.0), 0.25, 3.0, 24.0, 11.0),
            (11.0, 0.0, 0.0))
        reversed_error = catchup_velocity(
            (12.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (100.0, 0.0, 0.0), 1.0, 100.0, 24.0, 11.0)
        self.assertEqual(reversed_error, (11.0, 0.0, 0.0))

    def test_consensus_ring_and_path(self):
        self.assertEqual(ring_neighbors(("uav1", "uav2", "uav3"), "uav2"),
                         ("uav1", "uav3"))
        self.assertEqual(ring_neighbors(("uav1", "uav2"), "uav1"),
                         ("uav2",))
        slot = (-10.0, 5.0, 2.0)
        self.assertEqual(virtual_center((90.0, 5.0, 102.0), 0.0, slot),
                         (100.0, 0.0, 100.0))
        position, velocity = path_reference(
            (100.0, 10.0, 100.0),
            ((0.0, 0.0, 100.0), (500.0, 0.0, 100.0)),
            20.0, 60.0, False)
        self.assertEqual(position, (160.0, 0.0, 100.0))
        self.assertEqual(velocity, (20.0, 0.0, 0.0))

    def test_consensus_velocity_never_reverses_route_motion(self):
        velocity = consensus_velocity(
            (20.0, 0.0, 0.0), (0.0, 0.0, 100.0),
            (100.0, 0.0, 100.0), ((100.0, 0.0, 100.0),),
            0.1, 2.5, 4.0)
        self.assertGreater(velocity[0], 0.0)

    def test_path_reference_reports_turn_rate_for_slot_feedforward(self):
        square = ((0.0, 0.0, 20.0), (100.0, 0.0, 20.0),
                  (100.0, 100.0, 20.0), (0.0, 100.0, 20.0))
        route = rounded_path(square, 20.0, True)
        samples = [path_reference_state(
            point, route, 10.0, 0.0, True) for point in route]
        turn_rates = [turn_rate for _position, _velocity, turn_rate in samples]
        self.assertTrue(any(rate > 0.0 for rate in turn_rates))
        self.assertLess(max(abs(rate) for rate in turn_rates), 1.0)

    def test_consensus_pacing_accelerates_lagger_and_slows_others(self):
        targets = consensus_speed_targets(
            20.0, (8.0, -1.6, -1.6, -1.6, -1.6, -1.6),
            (15.0, 20.0, 20.0, 20.0, 20.0, 20.0),
            0.4, 4.0, 0.1, deadband=1.0)
        self.assertGreater(targets[0], 15.0)
        self.assertTrue(all(speed < 20.0 for speed in targets[1:]))
        waiting = consensus_speed_targets(
            20.0, (8.0, -1.6, -1.6, -1.6, -1.6, -1.6),
            (15.0, 10.0, 10.0, 10.0, 10.0, 10.0),
            0.4, 4.0, 0.1, deadband=1.0)
        self.assertGreater(waiting[0], 15.0)
        self.assertTrue(all(speed <= 10.0 for speed in waiting[1:]))
        aligned = consensus_speed_targets(
            20.0, (0.0, 0.0, 0.0), (15.0, 15.0, 15.0),
            0.4, 4.0, 0.1, deadband=1.0)
        self.assertEqual(aligned, (20.0, 20.0, 20.0))

    def test_shared_path_center_and_start_spacing(self):
        slots = formation_offsets("circle", 4, 12.0)
        locations = [slot_target((0.0, 0.0, 20.0), 0.0, slot)[:3]
                     for slot in slots]
        center, error = shared_center(
            [virtual_center(location, 0.0, slot)
             for location, slot in zip(locations, slots)])
        for actual, expected in zip(center, (0.0, 0.0, 20.0)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(error, 0.0)
        scattered = list(locations)
        scattered[2] = (scattered[2][0] + 20.0,
                        scattered[2][1], scattered[2][2])
        _, error = shared_center(
            [virtual_center(location, 0.0, slot)
             for location, slot in zip(scattered, slots)])
        self.assertGreater(error, 4.0)

    def test_acquisition_preserves_team_center_for_sizes_and_shapes(self):
        for count in (2, 3, 4, 6):
            for shape in ("circle", "rectangle_compact"):
                starts = tuple((0.0, 3.0 * index, 20.0)
                               for index in range(count))
                offsets = formation_offsets(shape, count, 12.0)
                goals = acquisition_slots(starts, offsets, 0.4, 20.0)
                self.assertEqual(len(goals), count)
                for axis in (0, 1):
                    self.assertAlmostEqual(
                        sum(point[axis] for point in starts),
                        sum(point[axis] for point in goals))
                self.assertEqual(len(set((round(point[0], 6),
                                          round(point[1], 6))
                                         for point in goals)), count)

    def test_rounded_square_turns_without_a_right_angle_jump(self):
        square = ((0.0, 0.0, 20.0), (60.0, 0.0, 20.0),
                  (60.0, 60.0, 20.0), (0.0, 60.0, 20.0))
        route = rounded_path(square, 12.0, True)
        self.assertGreater(len(route), 40)
        segments = list(zip(route, route[1:] + route[:1]))
        headings = [math.atan2(end[1] - start[1],
                               end[0] - start[0]) for start, end in segments]
        turns = [abs(math.atan2(math.sin(b - a), math.cos(b - a)))
                 for a, b in zip(headings, headings[1:] + headings[:1])]
        self.assertLess(max(turns), 0.2)
        self.assertTrue(all(point[2] == 20.0 for point in route))

    def test_consensus_correction_is_bounded_and_directional(self):
        velocity = consensus_velocity(
            (20.0, 0.0, 0.0), (100.0, 0.0, 100.0),
            (100.0, 20.0, 100.0),
            ((100.0, 0.0, 100.0), (100.0, 0.0, 100.0)),
            0.1, 500.0, 4.0)
        self.assertLess(velocity[1], -3.9)
        self.assertLessEqual(math.hypot(velocity[0] - 20.0,
                                        velocity[1]), 4.05)
        behind = consensus_velocity(
            (3.0, 0.0, 0.0), (20.0, 0.0, 3.0),
            (0.0, 0.0, 3.0), ((20.0, 0.0, 3.0),),
            0.1, 500.0, 4.0)
        self.assertAlmostEqual(behind[0], 5.0)
        ahead = consensus_velocity(
            (3.0, 0.0, 0.0), (20.0, 0.0, 3.0),
            (20.0, 0.0, 3.0), ((0.0, 0.0, 3.0),),
            0.1, 500.0, 4.0)
        self.assertAlmostEqual(ahead[0], 1.0)

    def test_navigation_is_allowed_but_flight_actions_preempt(self):
        self.assertFalse(blocking_action(Actions.ACTION_NAVIGATE, Actions))
        self.assertFalse(blocking_action(Actions.ACTION_HOLD, Actions))
        for action in (Actions.ACTION_TAKEOFF, Actions.ACTION_LAND,
                       Actions.ACTION_RETURN_HOME, Actions.ACTION_EMERGENCY):
            self.assertTrue(blocking_action(action, Actions))


if __name__ == "__main__":
    unittest.main()
