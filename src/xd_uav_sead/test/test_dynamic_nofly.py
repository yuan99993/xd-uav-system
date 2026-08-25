#!/usr/bin/env python3

import math
from types import SimpleNamespace
import unittest

from xd_uav_sead.airspace.airspace_manager import AirspaceManager
from xd_uav_sead.airspace.dynamic_nofly import (
    DynamicNoFlyConfig,
    DynamicNoFlyZoneReceiver,
)
from xd_uav_sead.planning.GA_SEAD_process import plan_path_with_avoidance


class _Time:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def to_sec(self):
        return self.seconds


def _message(
    now=100.0,
    frame="uav1/odom",
    operation=0,
    zone_id=7,
    vertices=None,
    min_altitude=20.0,
    max_altitude=120.0,
    valid_until=110.0,
):
    if vertices is None:
        vertices = [(80.0, -30.0), (120.0, -30.0), (120.0, 30.0), (80.0, 30.0)]
    points = [SimpleNamespace(x=x, y=y, z=0.0) for x, y in vertices]
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=_Time(now)),
        schema_version=1,
        operation=operation,
        zone_id=zone_id,
        enabled=True,
        zone_type=0,
        min_altitude=min_altitude,
        max_altitude=max_altitude,
        valid_until=_Time(valid_until),
        polygon=SimpleNamespace(points=points),
    )


class DynamicNoFlyContractTest(unittest.TestCase):
    def setUp(self):
        self.airspace = AirspaceManager()
        self.receiver = DynamicNoFlyZoneReceiver(
            self.airspace,
            DynamicNoFlyConfig(expected_frame="uav1/odom"),
        )

    def test_upsert_remove_and_clear(self):
        self.assertTrue(self.receiver.accept(_message(), 100.0))
        self.assertIn(7, self.airspace.zones)
        self.assertTrue(self.airspace.is_in_nofly(100.0, 0.0, 60.0))
        self.assertFalse(self.airspace.is_in_nofly(100.0, 0.0, 10.0))

        self.assertTrue(
            self.receiver.accept(
                _message(operation=1, vertices=[], valid_until=110.0), 100.0
            )
        )
        self.assertNotIn(7, self.airspace.zones)

        self.assertTrue(self.receiver.accept(_message(zone_id=8), 100.0))
        self.assertTrue(
            self.receiver.accept(
                _message(operation=2, zone_id=0, vertices=[]), 100.0
            )
        )
        self.assertFalse(self.airspace.zones)

    def test_wrong_frame_stale_and_self_intersection_fail_closed(self):
        invalid = [
            _message(frame="map"),
            _message(now=90.0),
            _message(vertices=[(0.0, 0.0), (10.0, 10.0), (0.0, 10.0), (10.0, 0.0)]),
        ]
        for msg in invalid:
            with self.subTest(msg=msg):
                self.assertFalse(self.receiver.accept(msg, 100.0))
                event = self.receiver.pop_event()
                self.assertTrue(event.fault)
                self.assertFalse(event.accepted)

    def test_expiry_fault_retains_last_geometry(self):
        self.assertTrue(
            self.receiver.accept(_message(valid_until=101.0), 100.0)
        )
        self.receiver.pop_event()
        self.receiver.poll_expirations(101.1)
        event = self.receiver.pop_event()
        self.assertTrue(event.fault)
        self.assertIn("expired", event.reason)
        self.assertIn(7, self.airspace.zones)

    def test_zero_valid_until_is_permanent_until_remove(self):
        self.assertTrue(self.receiver.accept(_message(valid_until=0.0), 100.0))
        event = self.receiver.pop_event()
        self.assertEqual(event.reason, "permanent_zone_upserted")
        self.receiver.poll_expirations(100000.0)
        self.assertIsNone(self.receiver.pop_event())
        self.assertIn(7, self.airspace.zones)
        self.assertTrue(
            self.receiver.accept(
                _message(
                    now=100001.0,
                    operation=1,
                    vertices=[],
                    valid_until=0.0,
                ),
                100001.0,
            )
        )
        self.assertNotIn(7, self.airspace.zones)

    def test_negative_valid_until_is_rejected(self):
        self.assertFalse(self.receiver.accept(_message(valid_until=-1.0), 100.0))
        self.assertTrue(self.receiver.pop_event().fault)


class TurnConstrainedPlannerTest(unittest.TestCase):
    def setUp(self):
        self.zone = {
            "zone_id": 1,
            "minAlt": 20.0,
            "maxAlt": 120.0,
            "poly": [(80.0, -30.0), (120.0, -30.0), (120.0, 30.0), (80.0, 30.0)],
        }

    @staticmethod
    def _edge_clearance(point, poly):
        px, py = point[:2]
        best = float("inf")
        for i, a in enumerate(poly):
            b = poly[(i + 1) % len(poly)]
            vx, vy = b[0] - a[0], b[1] - a[1]
            denom = vx * vx + vy * vy
            t = max(0.0, min(1.0, ((px - a[0]) * vx + (py - a[1]) * vy) / denom))
            best = min(best, math.hypot(px - (a[0] + t * vx), py - (a[1] + t * vy)))
        return best

    def test_path_honors_turn_radius_clearance_and_altitude(self):
        path = plan_path_with_avoidance(
            (0.0, 0.0, 0.0),
            (220.0, 0.0, 0.0),
            20.0,
            [self.zone],
            15.0,
            sampling_step=2.0,
            clearance=20.0,
            flight_altitude=60.0,
        )
        self.assertGreater(len(path), 2)
        self.assertGreaterEqual(
            min(self._edge_clearance(point, self.zone["poly"]) for point in path),
            20.0 - 1e-6,
        )

        direct = plan_path_with_avoidance(
            (0.0, 0.0, 0.0),
            (220.0, 0.0, 0.0),
            20.0,
            [self.zone],
            15.0,
            sampling_step=2.0,
            clearance=20.0,
            flight_altitude=200.0,
        )
        self.assertGreater(len(direct), 2)
        self.assertLess(max(abs(point[1]) for point in direct), 1e-6)

    def test_too_late_to_turn_returns_no_path(self):
        path = plan_path_with_avoidance(
            (70.0, 0.0, 0.0),
            (220.0, 0.0, 0.0),
            20.0,
            [self.zone],
            15.0,
            sampling_step=2.0,
            clearance=20.0,
            flight_altitude=60.0,
        )
        self.assertEqual(path, [])


if __name__ == "__main__":
    unittest.main()
