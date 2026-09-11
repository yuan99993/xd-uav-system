#!/usr/bin/env python3

from types import SimpleNamespace
import unittest

from xd_uav_planning.nofly import (
    NoFlyConfig,
    NoFlyZoneStore,
    NoSafePathError,
    Zone,
    adjust_fixedwing_path,
    path_is_clear,
    path_min_clearance,
    remaining_path,
)


class _Time:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


def _message(operation=0, zone_id=7, frame="world", vertices=None,
             stamp=100.0):
    if vertices is None:
        vertices = ((80.0, -20.0), (110.0, -20.0),
                    (110.0, 20.0), (80.0, 20.0))
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=_Time(stamp)),
        schema_version=1,
        operation=operation,
        zone_id=zone_id,
        enabled=True,
        zone_type=0,
        min_altitude=0.0,
        max_altitude=100.0,
        valid_until=_Time(0.0),
        polygon=SimpleNamespace(points=[
            SimpleNamespace(x=x, y=y) for x, y in vertices]),
    )


class NoFlyZoneStoreTest(unittest.TestCase):
    def setUp(self):
        self.store = NoFlyZoneStore(NoFlyConfig(expected_frame="world"))

    def test_upsert_remove_clear_and_rejection(self):
        self.assertTrue(self.store.accept(_message(), 100.0))
        self.assertEqual(len(self.store.snapshot()), 1)
        self.assertTrue(self.store.accept(_message(operation=1), 100.0))
        self.assertFalse(self.store.snapshot())
        self.assertTrue(self.store.accept(_message(zone_id=8), 100.0))
        self.assertTrue(self.store.accept(
            _message(operation=2, zone_id=0, vertices=()), 100.0))
        self.assertFalse(self.store.snapshot())
        self.assertFalse(self.store.accept(_message(frame="map"), 100.0))
        self.assertEqual(self.store.last_reason, "zone_frame_mismatch")

    def test_zero_and_stale_header_stamps_are_stored(self):
        self.assertTrue(self.store.accept(_message(stamp=0.0), 100.0))
        self.assertTrue(self.store.accept(
            _message(zone_id=8, stamp=1.0), 100.0))
        self.assertEqual({zone.zone_id for zone in self.store.snapshot()},
                         {7, 8})


class StaticNoFlyPlannerTest(unittest.TestCase):
    def setUp(self):
        self.zone = Zone(
            7, True, 0.0, 100.0,
            ((80.0, -20.0), (110.0, -20.0),
             (110.0, 20.0), (80.0, 20.0)))
        self.path = [(0.0, 0.0, 30.0), (100.0, 0.0, 30.0),
                     (300.0, 0.0, 30.0)]

    def test_blocked_path_is_adjusted_with_clearance(self):
        result, adjusted = adjust_fixedwing_path(
            self.path, (0.0, 0.0, 30.0), 0.0, (self.zone,),
            turning_radius=35.0, clearance=10.0, sample_step=2.0)
        self.assertTrue(adjusted)
        self.assertGreater(len(result), len(self.path))
        self.assertTrue(path_is_clear(result, (self.zone,), 10.0, 1.0))
        self.assertGreaterEqual(path_min_clearance(result, (self.zone,), 1.0), 10.0)

    def test_unrelated_altitude_preserves_original_path(self):
        high_zone = Zone(self.zone.zone_id, True, 100.0, 200.0,
                         self.zone.vertices)
        result, adjusted = adjust_fixedwing_path(
            self.path, (0.0, 0.0, 30.0), 0.0, (high_zone,),
            turning_radius=35.0, clearance=10.0, sample_step=2.0)
        self.assertFalse(adjusted)
        self.assertEqual(result, self.path)

    def test_destination_inside_zone_is_rejected(self):
        with self.assertRaises(NoSafePathError):
            adjust_fixedwing_path(
                [(0.0, 0.0, 30.0), (90.0, 0.0, 30.0)],
                (0.0, 0.0, 30.0), 0.0, (self.zone,),
                turning_radius=35.0, clearance=10.0, sample_step=2.0)

    def test_dense_route_only_expands_the_blocked_edge(self):
        # Fixed-wing coverage routes contain many samples along their smooth
        # turns.  A no-fly replan must not create a new Dubins path for every
        # one of those already-safe samples.
        dense_path = [(float(x), 0.0, 30.0) for x in range(0, 301, 2)]
        result, adjusted = adjust_fixedwing_path(
            dense_path, (0.0, 0.0, 30.0), 0.0, (self.zone,),
            turning_radius=35.0, clearance=10.0, sample_step=2.0)
        self.assertTrue(adjusted)
        self.assertLess(len(result), 1000)
        self.assertTrue(path_is_clear(result, (self.zone,), 10.0, 1.0))


class RemainingPathTest(unittest.TestCase):
    def test_projects_to_forward_remaining_route(self):
        route = [(0.0, 0.0, 20.0), (100.0, 0.0, 20.0),
                 (200.0, 0.0, 20.0)]
        result, progress = remaining_path(route, (60.0, 15.0, 20.0))
        self.assertEqual(result[0], (60.0, 15.0, 20.0))
        self.assertEqual(result[1:], route[1:])
        self.assertAlmostEqual(progress, 60.0)

    def test_progress_never_moves_backwards_at_route_crossing(self):
        route = [(0.0, 0.0, 20.0), (100.0, 0.0, 20.0),
                 (0.0, 0.0, 20.0), (-100.0, 0.0, 20.0)]
        result, progress = remaining_path(
            route, (10.0, 2.0, 20.0), minimum_progress=150.0)
        self.assertGreaterEqual(progress, 150.0)
        self.assertEqual(result[-1], route[-1])


if __name__ == "__main__":
    unittest.main()
