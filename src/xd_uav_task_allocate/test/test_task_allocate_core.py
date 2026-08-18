#!/usr/bin/env python3

import math
import unittest

from xd_uav_task_allocate.core.allocation import (
    TASK_ASSIGNED,
    TASK_COMPLETED,
    TASK_PENDING,
    RescueTaskAllocator,
)
from xd_uav_task_allocate.core.coverage import (
    PlannedArea,
    SearchAreaDefinition,
    assign_areas,
    lawnmower_path,
)
from xd_uav_task_allocate.core.execution import (
    ArrivalDwellTracker,
    goal_distance,
    goal_heading,
    worker_approach_goal,
)
from xd_uav_task_allocate.core.geometry import (
    PoseHistory,
    PoseSample,
    RigidTransform,
    relative_frd_to_shared,
    rotate_covariance_frd_to_shared,
)
from xd_uav_task_allocate.core.registry import (
    TARGET_CANDIDATE,
    TARGET_CONFIRMED,
    TARGET_STALE,
    GlobalTargetRecord,
    GlobalTargetRegistry,
    TargetObservation,
)


class GeometryTest(unittest.TestCase):
    def test_relative_frd_uses_capture_pose_and_body_orientation(self):
        yaw = math.pi / 2.0
        pose = PoseSample(
            stamp=10.0,
            position_reference=(10.0, 20.0, 5.0),
            orientation_reference_body=(
                0.0,
                0.0,
                math.sin(yaw / 2.0),
                math.cos(yaw / 2.0),
            ),
        )
        target = relative_frd_to_shared((10.0, 0.0, 5.0), pose)
        self.assertAlmostEqual(target[0], 10.0)
        self.assertAlmostEqual(target[1], 30.0)
        self.assertAlmostEqual(target[2], 0.0)

    def test_pose_history_rejects_large_time_difference(self):
        history = PoseHistory()
        history.add(PoseSample(10.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
        self.assertIsNotNone(history.closest(10.1, 0.15))
        self.assertIsNone(history.closest(10.3, 0.15))

    def test_covariance_is_rotated_from_frd_to_shared(self):
        yaw = math.pi / 2.0
        covariance = (4.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 9.0)
        rotated = rotate_covariance_frd_to_shared(
            covariance,
            (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
        )
        self.assertAlmostEqual(rotated[0], 1.0)
        self.assertAlmostEqual(rotated[4], 4.0)
        self.assertAlmostEqual(rotated[8], 9.0)

    def test_full_tf_rotation_and_translation_are_applied(self):
        yaw = math.pi / 2.0
        world_from_odom = RigidTransform(
            translation=(100.0, 200.0, 5.0),
            rotation=(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
        )
        pose = PoseSample(
            stamp=10.0,
            position_reference=(1.0, 2.0, 3.0),
            orientation_reference_body=(0.0, 0.0, 0.0, 1.0),
            frame_id="uav1/odom",
        )
        target = relative_frd_to_shared((10.0, 0.0, 0.0), pose, world_from_odom)
        self.assertAlmostEqual(target[0], 98.0)
        self.assertAlmostEqual(target[1], 211.0)
        self.assertAlmostEqual(target[2], 8.0)


class CoverageTest(unittest.TestCase):
    def test_rectangle_generates_alternating_lanes(self):
        area = SearchAreaDefinition(
            area_id=1,
            boundary=((0.0, 0.0), (100.0, 0.0), (100.0, 40.0), (0.0, 40.0)),
            altitude=20.0,
            lane_spacing=10.0,
        )
        path = lawnmower_path(area)
        self.assertGreaterEqual(len(path), 6)
        self.assertLess(path[0][0], path[1][0])
        self.assertGreater(path[2][0], path[3][0])
        self.assertTrue(all(point[2] == 20.0 for point in path))

    def test_area_workload_is_distributed(self):
        areas = []
        for area_id, offset in ((1, 0.0), (2, 200.0)):
            definition = SearchAreaDefinition(
                area_id,
                ((offset, 0.0), (offset + 100.0, 0.0), (offset + 100.0, 40.0), (offset, 40.0)),
                20.0,
                10.0,
            )
            areas.append(PlannedArea(definition, lawnmower_path(definition)))
        assignment = assign_areas(areas, {"uav1": (0.0, 0.0), "uav2": (200.0, 0.0)})
        self.assertEqual(sum(len(value) for value in assignment.values()), 2)
        self.assertTrue(assignment["uav1"])
        self.assertTrue(assignment["uav2"])


class DirectExecutionTest(unittest.TestCase):
    def test_goal_heading_uses_world_enu_direction(self):
        self.assertAlmostEqual(goal_heading((0.0, 0.0), (5.0, 0.0)), 0.0)
        self.assertAlmostEqual(goal_heading((0.0, 0.0), (0.0, 5.0)), math.pi / 2.0)
        self.assertIsNone(goal_heading((1.0, 1.0), (1.05, 1.0), 0.1))

    def test_worker_arrival_checks_the_captured_approach_altitude(self):
        self.assertAlmostEqual(
            goal_distance((8.0, 0.0, 4.0), (8.0, 0.0, 5.0), use_z=True),
            1.0,
        )

    def test_worker_stops_before_target_and_holds_current_altitude(self):
        goal = worker_approach_goal(
            current=(0.0, 0.0, 5.0),
            target=(10.0, 0.0, 0.5),
            horizontal_standoff_m=2.0,
        )
        self.assertEqual(goal, (8.0, 0.0, 5.0))

    def test_worker_already_inside_standoff_holds_its_position(self):
        goal = worker_approach_goal(
            current=(9.0, 0.0, 5.0),
            target=(10.0, 0.0, 0.5),
            horizontal_standoff_m=2.0,
        )
        self.assertEqual(goal, (9.0, 0.0, 5.0))

    def test_arrival_requires_continuous_dwell(self):
        tracker = ArrivalDwellTracker(tolerance_m=1.0, dwell_sec=0.5)
        self.assertFalse(tracker.update("uav1", (0.5, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0))
        self.assertFalse(tracker.update("uav1", (2.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.2))
        self.assertFalse(tracker.update("uav1", (0.5, 0.0, 0.0), (0.0, 0.0, 0.0), 2.0))
        self.assertTrue(tracker.update("uav1", (0.5, 0.0, 0.0), (0.0, 0.0, 0.0), 2.5))


class RegistryAndAllocationTest(unittest.TestCase):
    @staticmethod
    def observation(source, stamp, x, y):
        return TargetObservation(source, stamp, 0, (x, y, 0.0), 1.0)

    def test_repeated_red_detections_create_one_global_target(self):
        registry = GlobalTargetRegistry(
            association_radius_m=4.0,
            confirmation_hits=5,
            confirmation_window_sec=2.0,
            confirmation_distinct_uavs=2,
        )
        updates = [
            registry.observe(self.observation("uav1", 10.0 + 0.1 * index, 100.0 + 0.1 * index, 50.0))
            for index in range(20)
        ]
        self.assertEqual(len(registry.targets), 1)
        self.assertEqual(updates[-1].target.status, TARGET_CONFIRMED)
        self.assertEqual(sum(1 for update in updates if update.newly_confirmed), 1)

    def test_two_scouts_confirm_without_detector_ids(self):
        registry = GlobalTargetRegistry(confirmation_hits=10, confirmation_distinct_uavs=2)
        registry.observe(self.observation("uav1", 10.0, 100.0, 50.0))
        update = registry.observe(self.observation("uav2", 10.1, 101.0, 50.5))
        self.assertTrue(update.newly_confirmed)
        self.assertEqual(len(registry.targets), 1)

    def test_duplicate_boxes_in_one_frame_are_one_confirmation_hit(self):
        registry = GlobalTargetRegistry(
            confirmation_hits=3,
            confirmation_minimum_span_sec=0.1,
            confirmation_distinct_uavs=2,
        )
        updates = [
            registry.observe(self.observation("uav1", 10.0, 100.0 + 0.01 * index, 50.0))
            for index in range(5)
        ]
        self.assertFalse(any(update.newly_confirmed for update in updates))
        self.assertEqual(len(updates[-1].target.recent_observations), 1)

    def test_unconfirmed_candidate_still_expires(self):
        registry = GlobalTargetRegistry(
            confirmation_hits=5,
            confirmation_distinct_uavs=2,
            stale_timeout_sec=1.0,
        )
        target = registry.observe(self.observation("uav1", 10.0, 5.0, 5.0)).target
        self.assertEqual(target.status, TARGET_CANDIDATE)
        self.assertEqual(registry.expire(11.1), [target.target_id])
        self.assertEqual(target.status, TARGET_STALE)

    def test_confirmed_target_waiting_for_worker_does_not_duplicate_after_timeout(self):
        registry = GlobalTargetRegistry(
            association_radius_m=4.0,
            confirmation_hits=10,
            confirmation_distinct_uavs=2,
            stale_timeout_sec=1.0,
        )
        allocator = RescueTaskAllocator()
        registry.observe(self.observation("uav1", 10.0, -15.8, -23.1))
        confirmation = registry.observe(
            self.observation("uav2", 10.1, -15.7, -23.4)
        )
        self.assertTrue(confirmation.newly_confirmed)
        task = allocator.ensure_task(confirmation.target)
        self.assertEqual(task.status, TASK_PENDING)

        # No worker is available, so this confirmed task waits much longer
        # than the candidate timeout before the same object is observed again.
        self.assertEqual(registry.expire(30.0), [])
        repeated = registry.observe(
            self.observation("uav1", 30.1, -15.75, -23.25)
        )
        self.assertFalse(repeated.created)
        self.assertEqual(repeated.target.target_id, confirmation.target.target_id)
        self.assertEqual(len(registry.targets), 1)
        self.assertIs(allocator.ensure_task(repeated.target), task)
        self.assertEqual(len(allocator.tasks), 1)

    def test_worker_finishes_then_receives_queued_task(self):
        allocator = RescueTaskAllocator()
        allocator.update_worker("uav3", (0.0, 0.0, 20.0), 1.0, True)
        first_target = GlobalTargetRecord(
            1, 0, [10.0, 0.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0, status=TARGET_CONFIRMED
        )
        second_target = GlobalTargetRecord(
            2, 0, [20.0, 0.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0, status=TARGET_CONFIRMED
        )
        first = allocator.ensure_task(first_target)
        second = allocator.ensure_task(second_target)
        allocator.assign_pending()
        self.assertEqual(first.status, TASK_ASSIGNED)
        self.assertEqual(second.status, TASK_PENDING)
        allocator.complete(first.task_id)
        allocator.assign_pending()
        self.assertEqual(first.status, TASK_COMPLETED)
        self.assertEqual(second.status, TASK_ASSIGNED)

    def test_worker_task_is_released_only_after_continuous_timeout(self):
        allocator = RescueTaskAllocator()
        allocator.update_worker("uav3", (0.0, 0.0, 5.0), 10.0, True)
        target = GlobalTargetRecord(
            1, 0, [10.0, 0.0, 0.5], [0.0] * 9, 1.0, 10.0, 10.0,
            status=TARGET_CONFIRMED,
        )
        task = allocator.ensure_task(target)
        allocator.assign_pending()
        allocator.workers["uav3"].online = False

        self.assertEqual(allocator.mark_offline_workers(10.5, 1.0), [])
        self.assertEqual(task.status, TASK_ASSIGNED)
        released = allocator.mark_offline_workers(11.1, 1.0)
        self.assertEqual([item.task_id for item in released], [task.task_id])
        self.assertEqual(task.status, TASK_PENDING)
        self.assertEqual(task.assigned_worker, "")


if __name__ == "__main__":
    unittest.main()
