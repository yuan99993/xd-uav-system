#!/usr/bin/env python3

import math
import threading
import unittest
from unittest.mock import patch

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from xd_uav_controller.msg import PathStatus

from xd_uav_task_allocate.core.allocation import (
    TASK_ASSIGNED,
    TASK_COMPLETED,
    TASK_FAILED,
    TASK_PENDING,
    RescueTaskAllocator,
)
from xd_uav_task_allocate.core.coverage import (
    PlannedArea,
    SearchAreaDefinition,
    assign_areas,
    connect_fixedwing_paths,
    dubins_path,
    fixedwing_entry_path,
    fixedwing_lawnmower_path,
    lawnmower_path,
    path_length,
    verification_search_area,
)
from xd_uav_task_allocate.core.execution import (
    ArrivalDwellTracker,
    fixedwing_trajectory_samples,
    fixedwing_waypoint_reached,
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
    TARGET_COMPLETED,
    TARGET_CONFIRMED,
    TARGET_STALE,
    TARGET_VERIFYING,
    GlobalTargetRecord,
    GlobalTargetRegistry,
    TargetObservation,
)
from xd_uav_task_allocate.msg import PlannerStatus
from xd_uav_task_allocate.ros.coordinator import (
    MISSION_ACTIVE,
    MISSION_PAUSED,
    TaskAllocateCoordinator,
)


class CoordinatorConfigurationTest(unittest.TestCase):
    def test_auto_mode_uses_single_stage_for_multirotor_only(self):
        self.assertFalse(
            TaskAllocateCoordinator._resolve_hierarchical_search_mode(
                "auto", {"multirotor"}
            )
        )

    def test_auto_mode_uses_hierarchy_for_mixed_scouts(self):
        self.assertTrue(
            TaskAllocateCoordinator._resolve_hierarchical_search_mode(
                "auto", {"fixedwing", "multirotor"}
            )
        )

    def test_verification_waits_until_all_fixedwing_routes_finish(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.hierarchical_search_enabled = True
        coordinator.mission_state = MISSION_ACTIVE
        coordinator.verification_dispatch_policy = "after_coarse_complete"
        coordinator.scout_configs = {"fw1": {}, "fw2": {}, "uav3": {}}
        coordinator.vehicle_types = {
            "fw1": "fixedwing",
            "fw2": "fixedwing",
            "uav3": "multirotor",
        }
        coordinator.routes = {
            "fw1": [(0.0, 0.0, 50.0)],
            "fw2": [(1.0, 0.0, 50.0)],
            "uav3": [],
        }
        coordinator.route_indices = {"fw1": 1, "fw2": 0, "uav3": 0}
        coordinator.verification_pending = [7]

        coordinator._assign_pending_verifications()

        self.assertEqual(coordinator.verification_pending, [7])

    def test_targets_inside_one_verification_region_share_one_route(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.verification_altitude = 20.0
        coordinator.verification_lane_spacing = 5.0
        coordinator.verification_minimum_radius = 20.0
        coordinator.verification_maximum_radius = 80.0
        coordinator.verification_covariance_sigma = 3.0
        coordinator.verification_radius_mode = "fixed"
        coordinator.verification_fixed_radius = 30.0
        coordinator.verification_pending = []
        coordinator.verification_plans = {}
        coordinator.verification_region_by_target = {}
        coordinator.verification_targets_by_region = {}
        coordinator.verification_region_states = {}
        coordinator.search_completed_at = 1.0
        coordinator._publish_verification_areas = lambda: None
        coordinator._assign_pending_verifications = lambda: None
        coordinator.registry = GlobalTargetRegistry()
        first = GlobalTargetRecord(
            1, 0, [100.0, 50.0, 0.0], [1.0] + [0.0] * 8,
            1.0, 0.0, 0.0,
        )
        second = GlobalTargetRecord(
            2, 0, [115.0, 55.0, 0.0], [1.0] + [0.0] * 8,
            1.0, 0.0, 0.0,
        )
        coordinator.registry.targets = {1: first, 2: second}

        coordinator._queue_target_verification(first)
        coordinator._queue_target_verification(second)

        self.assertEqual(coordinator.verification_pending, [1])
        self.assertEqual(coordinator.verification_region_by_target[2], 1)
        self.assertEqual(coordinator.verification_targets_by_region[1], {1, 2})

    def test_confirmed_member_does_not_cancel_pending_region_search(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.hierarchical_search_enabled = True
        coordinator.mission_state = MISSION_ACTIVE
        coordinator.verification_dispatch_policy = "immediate"
        coordinator.verification_maximum_concurrent_regions = 1
        coordinator.verification_pending = [1]
        coordinator.verification_target_by_scout = {}
        coordinator.verification_scout_by_target = {}
        coordinator.verification_targets_by_region = {1: {1}}
        coordinator.verification_region_states = {1: "pending"}
        coordinator.verification_plans = {
            1: PlannedArea(
                area=SearchAreaDefinition(
                    area_id=1,
                    boundary=((0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)),
                    altitude=20.0,
                    lane_spacing=5.0,
                ),
                path=[(0.0, 0.0, 20.0), (20.0, 0.0, 20.0)],
            )
        }
        coordinator.registry = GlobalTargetRegistry()
        coordinator.registry.targets = {
            1: GlobalTargetRecord(
                1, 0, [10.0, 10.0, 0.0], [1.0] + [0.0] * 8,
                1.0, 0.0, 0.0, status=TARGET_CONFIRMED,
            )
        }
        coordinator.scout_configs = {"uav2": {}}
        coordinator.vehicle_types = {"uav2": "multirotor"}
        coordinator.vehicle_world_positions = {"uav2": (1.0, (0.0, 0.0, 20.0))}
        coordinator.routes = {"uav2": []}
        coordinator.route_indices = {"uav2": 0}
        coordinator.active_goals = {}
        coordinator._vehicle_ready = lambda *_args: True
        coordinator._publish_path = lambda *_args: None
        coordinator._publish_verification_areas = lambda: None
        published = []
        coordinator._publish_next_scout_goal = lambda scout: published.append(scout)
        stamp = type("Stamp", (), {"to_sec": lambda self: 1.0})()

        with patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.Time.now",
            return_value=stamp,
        ):
            coordinator._assign_pending_verifications()

        self.assertEqual(coordinator.verification_pending, [])
        self.assertEqual(coordinator.verification_target_by_scout, {"uav2": 1})
        self.assertEqual(coordinator.verification_region_states[1], "active")
        self.assertEqual(published, ["uav2"])

    def test_region_completion_keeps_confirmed_members_and_rejects_only_unseen(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.verification_pending = []
        coordinator.verification_scout_by_target = {1: "uav2"}
        coordinator.verification_target_by_scout = {"uav2": 1}
        coordinator.verification_targets_by_region = {1: {1, 2}}
        coordinator.verification_region_states = {1: "active"}
        coordinator.active_goals = {}
        coordinator.active_goal_points = {}
        coordinator.active_goal_origins = {}
        coordinator.active_trajectory_end_times = {}
        coordinator.active_trajectory_route_progress = {}
        coordinator.routes = {"uav2": [(0.0, 0.0, 20.0)]}
        coordinator.route_indices = {"uav2": 1}
        coordinator.reject_after_verification_route = True
        coordinator.registry = GlobalTargetRegistry()
        confirmed = GlobalTargetRecord(
            1, 0, [0.0, 0.0, 0.0], [1.0] + [0.0] * 8,
            1.0, 0.0, 0.0, status=TARGET_CONFIRMED,
        )
        unseen = GlobalTargetRecord(
            2, 0, [1.0, 0.0, 0.0], [1.0] + [0.0] * 8,
            1.0, 0.0, 0.0, status=TARGET_VERIFYING,
        )
        coordinator.registry.targets = {1: confirmed, 2: unseen}
        coordinator.path_publishers = {}
        coordinator._publish_path = lambda *_args: None
        coordinator._publish_verification_areas = lambda: None
        coordinator._assign_pending_verifications = lambda: None
        coordinator._mark_search_complete_if_ready = lambda: None

        with patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.Time.now",
            return_value=type("Stamp", (), {})(),
        ):
            coordinator._finish_target_verification(1)

        self.assertEqual(confirmed.status, TARGET_CONFIRMED)
        self.assertEqual(unseen.status, TARGET_STALE)
        self.assertEqual(coordinator.verification_region_states[1], "completed")

    def test_fixedwing_setting_uses_vehicle_override(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.scout_configs = {
            "uav1": {"fixedwing": {"minimum_turn_radius_m": 75.0}}
        }
        coordinator.worker_configs = {}

        with patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.get_param",
            return_value=60.0,
        ):
            radius = coordinator._fixedwing_setting(
                "uav1", "minimum_turn_radius_m", 60.0
            )

        self.assertEqual(radius, 75.0)

    def test_fixedwing_setting_uses_mission_default(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.scout_configs = {"uav1": {}}
        coordinator.worker_configs = {}

        with patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.get_param",
            return_value=60.0,
        ):
            radius = coordinator._fixedwing_setting(
                "uav1", "minimum_turn_radius_m", 30.0
            )

        self.assertEqual(radius, 60.0)

    def test_fixedwing_route_velocity_uses_outgoing_path_tangent(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.active_goals = {"uav1": (4, "search", 1)}
        coordinator.routes = {
            "uav1": [(0.0, 0.0, 20.0), (10.0, 0.0, 20.0), (10.0, 5.0, 20.0)]
        }
        coordinator._coverage_speed = lambda _vehicle: 15.0

        velocity = coordinator._fixedwing_route_velocity("uav1")

        self.assertEqual(velocity, (0.0, 15.0))

    def test_fixedwing_direct_goal_carries_route_tangent(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.active_goals = {"uav1": (4, "search", 0)}
        coordinator.routes = {
            "uav1": [(0.0, 0.0, 20.0), (30.0, 40.0, 20.0)]
        }
        coordinator.vehicle_types = {"uav1": "fixedwing"}
        coordinator.vehicle_world_positions = {}
        coordinator.direct_face_goal = True
        coordinator._coverage_speed = lambda _vehicle: 15.0
        published = []
        coordinator.direct_goal_publishers = {
            "uav1": type("Publisher", (), {"publish": published.append})()
        }
        goal = PoseStamped()
        goal.pose.position.z = 20.0

        coordinator._publish_direct_controller_goal("uav1", goal)

        self.assertEqual(len(published), 1)
        setpoint = published[0]
        self.assertFalse(setpoint.type_mask & PositionTarget.IGNORE_VX)
        self.assertFalse(setpoint.type_mask & PositionTarget.IGNORE_VY)
        self.assertTrue(setpoint.type_mask & PositionTarget.IGNORE_VZ)
        self.assertAlmostEqual(setpoint.velocity.x, 9.0)
        self.assertAlmostEqual(setpoint.velocity.y, 12.0)

    def test_pause_and_resume_preserve_active_goal(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator._lock = threading.RLock()
        coordinator.mission_state = MISSION_ACTIVE
        coordinator.mission_detail = ""
        coordinator.active_goals = {"uav1": (7, "search", 3)}
        coordinator.routes = {"uav1": [(0.0, 0.0, 20.0)] * 5}
        coordinator.route_indices = {"uav1": 3}
        coordinator._pause_active_vehicles = lambda: (True, "")
        coordinator._publish_mission_state = lambda: None
        republished = []
        coordinator._republish_active_goal = republished.append
        coordinator._dispatch_assignments = lambda: None
        coordinator._publish_next_scout_goal = lambda _vehicle: None

        pause_response = coordinator._pause_callback(None)
        resume_response = coordinator._resume_callback(None)

        self.assertTrue(pause_response.success)
        self.assertTrue(resume_response.success)
        self.assertEqual(coordinator.mission_state, MISSION_ACTIVE)
        self.assertEqual(coordinator.active_goals["uav1"], (7, "search", 3))
        self.assertEqual(republished, ["uav1"])


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

    def test_area_assignment_balances_estimated_time_for_mixed_aircraft(self):
        areas = []
        for area_id, offset in ((1, 0.0), (2, 200.0)):
            definition = SearchAreaDefinition(
                area_id,
                ((offset, 0.0), (offset + 100.0, 0.0), (offset + 100.0, 40.0), (offset, 40.0)),
                50.0,
                20.0,
            )
            areas.append(PlannedArea(definition, lawnmower_path(definition)))
        assignment = assign_areas(
            areas,
            {"fw1": (0.0, 0.0), "uav2": (0.0, 0.0)},
            scout_speeds={"fw1": 20.0, "uav2": 2.0},
        )
        self.assertEqual(len(assignment["fw1"]), 2)
        self.assertEqual(assignment["uav2"], [])

    def test_dubins_connector_respects_requested_end_poses(self):
        path = dubins_path(
            (0.0, 0.0, 0.0),
            (100.0, 20.0, math.pi),
            minimum_turn_radius=30.0,
            waypoint_spacing=5.0,
        )
        self.assertGreater(len(path), 3)
        self.assertEqual(path[0], (0.0, 0.0, 0.0))
        self.assertEqual(path[-1], (100.0, 20.0, math.pi))

    def test_fixedwing_coverage_adds_turn_radius_connectors(self):
        area = SearchAreaDefinition(
            area_id=1,
            boundary=((0.0, 0.0), (100.0, 0.0), (100.0, 40.0), (0.0, 40.0)),
            altitude=50.0,
            lane_spacing=10.0,
        )
        multirotor = lawnmower_path(area)
        fixedwing = fixedwing_lawnmower_path(area, 30.0, 10.0)
        self.assertGreater(len(fixedwing), len(multirotor))
        self.assertTrue(all(point[2] == 50.0 for point in fixedwing))
        # Every in-polygon coverage chord is preserved, even though fixed-wing
        # lanes are visited in a different order.
        for point in multirotor:
            self.assertIn(point, fixedwing)
        self.assertTrue(
            any(
                point[0] < 0.0
                or point[0] > 100.0
                or point[1] < 0.0
                or point[1] > 40.0
                for point in fixedwing
            )
        )

    def test_fixedwing_spreads_lane_reversals_and_levels_before_boundary(self):
        area = SearchAreaDefinition(
            area_id=1,
            boundary=((0.0, 0.0), (200.0, 0.0), (200.0, 200.0), (0.0, 200.0)),
            altitude=40.0,
            lane_spacing=10.0,
        )
        route = fixedwing_lawnmower_path(
            area,
            minimum_turn_radius=30.0,
            turn_waypoint_spacing=10.0,
            straight_lead_distance=30.0,
        )

        # The first two covered lanes are 100 m apart (0 then 10), rather than
        # adjacent 10 m lanes.  Both polygon crossings have collinear lead
        # points, so the turn is completed outside x=[0, 200].
        first_start = route.index((0.0, 5.0, 40.0))
        second_start = route.index((200.0, 105.0, 40.0))
        self.assertLess(first_start, second_start)
        self.assertEqual(route[first_start - 1], (-30.0, 5.0, 40.0))
        self.assertEqual(route[first_start + 2], (230.0, 5.0, 40.0))
        self.assertEqual(route[second_start - 1], (230.0, 105.0, 40.0))

    def test_turn_sampling_density_does_not_change_fixedwing_path_length(self):
        area = SearchAreaDefinition(
            area_id=1,
            boundary=((0.0, 0.0), (200.0, 0.0), (200.0, 100.0), (0.0, 100.0)),
            altitude=40.0,
            lane_spacing=10.0,
        )
        dense = fixedwing_lawnmower_path(area, 30.0, 3.0, 30.0)
        sparse = fixedwing_lawnmower_path(area, 30.0, 10.0, 30.0)
        self.assertGreater(len(dense), len(sparse))
        self.assertAlmostEqual(
            path_length(dense),
            path_length(sparse),
            delta=0.002 * path_length(dense),
        )

    def test_coarse_lane_spacing_reduces_passes_without_losing_chords(self):
        area = SearchAreaDefinition(
            area_id=1,
            boundary=((0.0, 0.0), (300.0, 0.0), (300.0, 300.0), (0.0, 300.0)),
            altitude=40.0,
            lane_spacing=30.0,
        )
        chords = lawnmower_path(area)
        route = fixedwing_lawnmower_path(area, 30.0, 10.0, 30.0)
        self.assertEqual(len(chords) // 2, 10)
        for point in chords:
            self.assertIn(point, route)
        self.assertLess(path_length(route) / 15.0, 6.0 * 60.0)

    def test_separate_fixedwing_areas_are_joined_continuously(self):
        first = [(0.0, 0.0, 40.0), (100.0, 0.0, 40.0)]
        second = [(200.0, 50.0, 60.0), (100.0, 50.0, 60.0)]
        connected = connect_fixedwing_paths([first, second], 30.0, 10.0)
        self.assertGreater(len(connected), len(first) + len(second))
        self.assertEqual(connected[0], first[0])
        self.assertEqual(connected[-1], second[-1])
        self.assertTrue(any(40.0 < point[2] < 60.0 for point in connected))

    def test_fixedwing_route_has_a_constrained_entry_from_current_pose(self):
        route = [(100.0, 0.0, 60.0), (200.0, 0.0, 60.0)]
        entered = fixedwing_entry_path(
            current=(0.0, 50.0, 40.0),
            current_heading=math.pi / 2.0,
            route=route,
            minimum_turn_radius=30.0,
            turn_waypoint_spacing=10.0,
        )
        self.assertGreater(len(entered), len(route))
        self.assertEqual(entered[-1], route[-1])
        self.assertAlmostEqual(entered[-2][2], 60.0)
        self.assertTrue(any(40.0 < point[2] < 60.0 for point in entered))

    def test_verification_area_uses_coarse_xy_covariance_with_bounds(self):
        planned = verification_search_area(
            target_id=7,
            center=(100.0, 50.0, 0.0),
            covariance=(100.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0, 0.0, 4.0),
            altitude=20.0,
            lane_spacing=5.0,
            minimum_radius=10.0,
            maximum_radius=25.0,
            covariance_sigma=3.0,
        )
        self.assertEqual(planned.area.area_id, 7)
        self.assertEqual(planned.area.boundary[0], (75.0, 25.0))
        self.assertEqual(planned.area.boundary[2], (125.0, 75.0))
        self.assertTrue(all(point[2] == 20.0 for point in planned.path))

    def test_verification_area_can_use_an_explicit_fixed_radius(self):
        planned = verification_search_area(
            target_id=8,
            center=(100.0, 50.0, 0.0),
            covariance=(1.0,) + (0.0,) * 8,
            altitude=20.0,
            lane_spacing=5.0,
            minimum_radius=20.0,
            maximum_radius=80.0,
            covariance_sigma=3.0,
            fixed_radius=35.0,
        )
        self.assertEqual(planned.area.boundary[0], (65.0, 15.0))
        self.assertEqual(planned.area.boundary[2], (135.0, 85.0))


class DirectExecutionTest(unittest.TestCase):
    def test_fixedwing_route_is_published_as_geometry_only_path(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator._next_goal_id = 12
        coordinator.shared_frame = "world"
        coordinator.vehicle_world_positions = {
            "uav1": (1.0, (0.0, 0.0, 40.0))
        }
        coordinator.active_goals = {}
        coordinator.active_goal_points = {}
        coordinator.active_goal_origins = {}
        coordinator.active_controller_paths = set()
        published = []
        coordinator.direct_path_publishers = {
            "uav1": type(
                "Publisher", (), {"publish": lambda _self, message: published.append(message)}
            )()
        }
        coordinator._publish_direct_status = lambda *_args: None
        coordinator._handle_goal_status = lambda *_args: None
        coordinator._coverage_speed = lambda _vehicle: 15.0

        with patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.Time.now",
            return_value=rospy.Time.from_sec(1.0),
        ):
            goal_id = coordinator._publish_fixedwing_route_path(
                "uav1",
                [(20.0, 0.0, 40.0), (20.0, 20.0, 40.0)],
                0,
                "search",
            )

        self.assertEqual(goal_id, 12)
        self.assertEqual(published[0].header.seq, 12)
        self.assertEqual(published[0].header.frame_id, "world")
        self.assertEqual(len(published[0].poses), 3)
        self.assertEqual(published[0].poses[0].pose.position.x, 0.0)
        self.assertTrue(
            all(pose.header.seq == 12 for pose in published[0].poses)
        )
        self.assertIsNot(published[0].poses[0].header, published[0].header)
        self.assertIn("uav1", coordinator.active_controller_paths)

    def test_fixedwing_worker_uses_geometric_path_to_approach_goal(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator._next_goal_id = 9
        coordinator.shared_frame = "world"
        coordinator.vehicle_types = {"uav3": "fixedwing"}
        coordinator.vehicle_backends = {"uav3": "direct_controller_test"}
        coordinator.vehicle_world_positions = {
            "uav3": (1.0, (0.0, 0.0, 40.0))
        }
        coordinator.active_goals = {}
        coordinator.active_goal_points = {}
        coordinator.active_goal_origins = {}
        coordinator.active_trajectory_end_times = {}
        coordinator.active_trajectory_route_progress = {}
        coordinator.active_controller_paths = set()
        coordinator.arrival_tracker = ArrivalDwellTracker(1.0, 0.0)
        coordinator.worker_arrival_tracker = ArrivalDwellTracker(0.5, 0.0)
        paths = []
        points = []
        coordinator.direct_path_publishers = {
            "uav3": type(
                "Publisher", (), {"publish": lambda _self, message: paths.append(message)}
            )()
        }
        coordinator.direct_goal_publishers = {
            "uav3": type(
                "Publisher", (), {"publish": lambda _self, message: points.append(message)}
            )()
        }
        coordinator.direct_status_publishers = {}
        coordinator._publish_direct_status = lambda *_args: None
        coordinator._handle_goal_status = lambda *_args: None

        with patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.Time.now",
            return_value=rospy.Time.from_sec(2.0),
        ):
            goal_id = coordinator._publish_goal(
                "uav3", (100.0, 20.0, 40.0), "rescue", 4
            )

        self.assertEqual(goal_id, 9)
        self.assertEqual(points, [])
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[0].poses), 2)
        self.assertEqual(paths[0].poses[0].pose.position.x, 0.0)
        self.assertEqual(paths[0].poses[-1].pose.position.x, 100.0)
        self.assertTrue(all(pose.header.seq == 9 for pose in paths[0].poses))
        self.assertIn("uav3", coordinator.active_controller_paths)

    def test_fixedwing_worker_path_keeps_geometric_completion_fallback(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.direct_controller_test = True
        coordinator.mission_state = MISSION_ACTIVE
        coordinator.vehicle_backends = {"uav3": "direct_controller_test"}
        coordinator.vehicle_types = {"uav3": "fixedwing"}
        coordinator.active_goals = {"uav3": (9, "rescue", 4)}
        coordinator.active_goal_points = {"uav3": (100.0, 0.0, 40.0)}
        coordinator.active_goal_origins = {"uav3": (0.0, 0.0, 40.0)}
        coordinator.active_trajectory_route_progress = {}
        coordinator.active_trajectory_end_times = {}
        coordinator.active_controller_paths = {"uav3"}
        coordinator.vehicle_world_positions = {
            "uav3": (10.0, (101.0, 2.0, 40.0))
        }
        coordinator.arrival_tracker = ArrivalDwellTracker(1.0, 0.0)
        coordinator.worker_arrival_tracker = ArrivalDwellTracker(0.5, 0.0)
        coordinator._vehicle_ready = lambda *_args: True
        coordinator._fixedwing_setting = lambda _vehicle, _key, default: default
        reached = []
        coordinator._publish_direct_status = lambda *args: reached.append(args)
        coordinator._handle_goal_status = lambda *args: reached.append(args)

        coordinator._check_direct_goal_arrivals(10.0)

        self.assertEqual(len(reached), 2)
        self.assertEqual(reached[0][0:2], ("uav3", 9))
        self.assertEqual(reached[0][2], PlannerStatus.REACHED)

    def test_controller_path_completion_drives_planner_completion(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator._lock = threading.RLock()
        coordinator.active_controller_paths = {"uav1"}
        coordinator.active_goals = {"uav1": (7, "search", 3)}
        published = []
        handled = []
        coordinator._publish_direct_status = lambda *args: published.append(args)
        coordinator._handle_goal_status = lambda *args: handled.append(args)
        message = PathStatus()
        message.path_id = 7
        message.state = PathStatus.COMPLETED
        message.progress = 1.0
        message.detail = "done"

        coordinator._path_status_callback("uav1", message)

        self.assertEqual(published[0][2], PlannerStatus.REACHED)
        self.assertEqual(handled[0][2], PlannerStatus.REACHED)

    def test_fixedwing_path_is_time_parameterized_with_nonzero_speed(self):
        samples = fixedwing_trajectory_samples(
            [(0.0, 0.0, 20.0), (30.0, 0.0, 20.0), (30.0, 30.0, 20.0)],
            15.0,
        )
        self.assertEqual(len(samples), 3)
        self.assertAlmostEqual(samples[-1].time_from_start, 4.0)
        for sample in samples:
            speed = math.sqrt(sum(value * value for value in sample.velocity))
            self.assertAlmostEqual(speed, 15.0)
        self.assertNotEqual(samples[1].yaw_rate, 0.0)

    def test_fixedwing_trajectory_duration_does_not_complete_before_flythrough(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.direct_controller_test = True
        coordinator.mission_state = MISSION_ACTIVE
        coordinator.vehicle_backends = {"uav1": "direct_controller_test"}
        coordinator.vehicle_types = {"uav1": "fixedwing"}
        coordinator.scout_configs = {
            "uav1": {
                "fixedwing": {
                    "waypoint_acceptance_radius_m": 20.0,
                    "waypoint_altitude_tolerance_m": 10.0,
                    "pass_cross_track_limit_m": 40.0,
                }
            }
        }
        coordinator.worker_configs = {}
        coordinator.active_goals = {"uav1": (7, "search", 2)}
        coordinator.active_goal_points = {"uav1": (100.0, 0.0, 40.0)}
        coordinator.active_goal_origins = {"uav1": (0.0, 0.0, 40.0)}
        coordinator.active_trajectory_route_progress = {"uav1": []}
        coordinator.active_trajectory_end_times = {"uav1": 9.0}
        coordinator.routes = {
            "uav1": [(0.0, 0.0, 40.0), (80.0, 0.0, 40.0), (100.0, 0.0, 40.0)]
        }
        coordinator.route_indices = {"uav1": 0}
        coordinator.vehicle_world_positions = {
            "uav1": (10.0, (50.0, 50.0, 40.0))
        }
        coordinator.arrival_tracker = ArrivalDwellTracker(1.0, 0.0)
        coordinator.worker_arrival_tracker = ArrivalDwellTracker(0.5, 0.0)
        coordinator._vehicle_ready = lambda *_args: True
        coordinator._fixedwing_setting = (
            lambda _vehicle, key, default: {
                "waypoint_acceptance_radius_m": 20.0,
                "waypoint_altitude_tolerance_m": 10.0,
                "pass_cross_track_limit_m": 40.0,
            }.get(key, default)
        )
        reached = []
        coordinator._publish_direct_status = lambda *args: reached.append(args)
        coordinator._handle_goal_status = lambda *args: reached.append(args)

        coordinator._check_direct_goal_arrivals(10.0)
        self.assertEqual(reached, [])

        coordinator.vehicle_world_positions["uav1"] = (
            10.1,
            (105.0, 0.0, 40.0),
        )
        coordinator._check_direct_goal_arrivals(10.1)
        self.assertEqual(len(reached), 2)

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

    def test_worker_cannot_complete_approach_while_still_moving_fast(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.direct_controller_test = True
        coordinator.mission_state = MISSION_ACTIVE
        coordinator.vehicle_backends = {"uav3": "direct_controller_test"}
        coordinator.vehicle_types = {"uav3": "multirotor"}
        coordinator.active_goals = {"uav3": (7, "rescue", 1)}
        coordinator.active_goal_points = {"uav3": (5.0, 0.0, 4.0)}
        coordinator.active_trajectory_route_progress = {}
        coordinator.active_trajectory_end_times = {}
        coordinator.vehicle_world_positions = {"uav3": (10.0, (5.0, 0.0, 4.0))}
        coordinator.vehicle_world_speeds = {"uav3": (10.0, 2.0)}
        coordinator.odometry_timeout = 0.5
        coordinator.worker_maximum_arrival_speed = 0.35
        coordinator.arrival_tracker = ArrivalDwellTracker(1.0, 0.0)
        coordinator.worker_arrival_tracker = ArrivalDwellTracker(0.5, 0.0)
        coordinator._vehicle_ready = lambda *_args: True
        reached = []
        coordinator._publish_direct_status = lambda *args: reached.append(args)
        coordinator._handle_goal_status = lambda *args: reached.append(args)

        coordinator._check_direct_goal_arrivals(10.0)
        self.assertEqual(reached, [])

        coordinator.vehicle_world_speeds["uav3"] = (10.1, 0.1)
        coordinator._check_direct_goal_arrivals(10.1)
        self.assertEqual(len(reached), 2)

    def test_completed_direct_worker_is_commanded_to_hold_current_pose(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.registry = GlobalTargetRegistry()
        target = GlobalTargetRecord(
            1, 0, [10.0, 0.0, 0.0], [0.0] * 9,
            1.0, 1.0, 1.0, status=TARGET_CONFIRMED,
        )
        coordinator.registry.targets = {1: target}
        coordinator.allocator = RescueTaskAllocator()
        coordinator.allocator.update_worker("uav3", (0.0, 0.0, 4.0), 1.0, True)
        task = coordinator.allocator.ensure_task(target)
        coordinator.allocator.assign_pending()
        coordinator.active_goals = {"uav3": (7, "rescue", task.task_id)}
        coordinator.vehicle_backends = {"uav3": "direct_controller_test"}
        coordinator.vehicle_types = {"uav3": "multirotor"}
        holds = []
        coordinator._publish_direct_hold = lambda vehicle: holds.append(vehicle) or True
        coordinator._clear_active_goal = lambda _vehicle: None
        coordinator._dispatch_assignments = lambda: None
        coordinator._check_mission_completed = lambda _now: None
        coordinator._publish_state = lambda: None

        with patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.Time.now",
            return_value=type("Stamp", (), {"to_sec": lambda self: 1.0})(),
        ):
            coordinator._handle_goal_status(
                "uav3", 7, PlannerStatus.REACHED, "settled at standoff"
            )

        self.assertEqual(holds, ["uav3"])
        self.assertEqual(task.status, TASK_COMPLETED)
        self.assertEqual(target.status, TARGET_COMPLETED)

    def test_completed_fixedwing_worker_keeps_loiter_and_logs_completion(self):
        coordinator = TaskAllocateCoordinator.__new__(TaskAllocateCoordinator)
        coordinator.registry = GlobalTargetRegistry()
        target = GlobalTargetRecord(
            1, 0, [100.0, 0.0, 0.0], [0.0] * 9,
            1.0, 1.0, 1.0, status=TARGET_CONFIRMED,
        )
        coordinator.registry.targets = {1: target}
        coordinator.allocator = RescueTaskAllocator()
        coordinator.allocator.update_worker("uav3", (0.0, 0.0, 40.0), 1.0, True)
        task = coordinator.allocator.ensure_task(target)
        coordinator.allocator.assign_pending()
        coordinator.active_goals = {"uav3": (7, "rescue", task.task_id)}
        coordinator.vehicle_backends = {"uav3": "direct_controller_test"}
        coordinator.vehicle_types = {"uav3": "fixedwing"}
        holds = []
        coordinator._publish_direct_hold = lambda vehicle: holds.append(vehicle) or True
        coordinator._clear_active_goal = lambda _vehicle: None
        coordinator._dispatch_assignments = lambda: None
        coordinator._check_mission_completed = lambda _now: None
        coordinator._publish_state = lambda: None

        with patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.Time.now",
            return_value=type("Stamp", (), {"to_sec": lambda self: 1.0})(),
        ), patch(
            "xd_uav_task_allocate.ros.coordinator.rospy.loginfo"
        ) as loginfo:
            coordinator._handle_goal_status(
                "uav3", 7, PlannerStatus.REACHED, "controller path 100.0%"
            )

        self.assertEqual(holds, [])
        self.assertEqual(task.status, TASK_COMPLETED)
        self.assertEqual(target.status, TARGET_COMPLETED)
        self.assertEqual(loginfo.call_count, 1)
        self.assertIn("任务完成", loginfo.call_args.args[0])
        self.assertEqual(loginfo.call_args.args[3], "uav3")
        self.assertEqual(loginfo.call_args.args[4], "fixedwing")

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

    def test_fixedwing_accepts_fly_through_without_dwell(self):
        self.assertTrue(
            fixedwing_waypoint_reached(
                current=(105.0, 5.0, 52.0),
                segment_start=(0.0, 0.0, 50.0),
                goal=(100.0, 0.0, 50.0),
                acceptance_radius_m=3.0,
                altitude_tolerance_m=5.0,
                pass_cross_track_limit_m=10.0,
            )
        )

    def test_fixedwing_does_not_accept_a_distant_plane_crossing(self):
        self.assertFalse(
            fixedwing_waypoint_reached(
                current=(105.0, 50.0, 50.0),
                segment_start=(0.0, 0.0, 50.0),
                goal=(100.0, 0.0, 50.0),
                acceptance_radius_m=3.0,
                altitude_tolerance_m=5.0,
                pass_cross_track_limit_m=10.0,
            )
        )


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
        self.assertFalse(update.newly_confirmed)
        update = registry.observe(self.observation("uav1", 10.2, 100.5, 50.2))
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

    def test_stable_track_class_flicker_keeps_one_physical_target(self):
        registry = GlobalTargetRegistry(
            confirmation_hits=5,
            confirmation_minimum_span_sec=0.3,
            confirmation_distinct_uavs=2,
        )
        classes = [2, 2, 5, 2, 2]
        updates = []
        for index, class_id in enumerate(classes):
            updates.append(
                registry.observe(
                    TargetObservation(
                        source_uav="uav1",
                        stamp=10.0 + 0.1 * index,
                        class_id=class_id,
                        position=(20.0 + 0.1 * index, 5.0, 0.0),
                        confidence=0.9,
                        track_id=7,
                        track_id_is_stable=True,
                        sensor_id="front_camera",
                    )
                )
            )
        self.assertEqual(len(registry.targets), 1)
        self.assertEqual(updates[-1].target.class_id, 2)
        self.assertTrue(updates[-1].newly_confirmed)
        self.assertEqual(updates[-1].target.source_tracks, {("uav1", "front_camera", 7)})

    def test_changed_track_id_can_reassociate_by_world_position(self):
        registry = GlobalTargetRegistry(
            confirmation_hits=5,
            confirmation_minimum_span_sec=0.3,
        )
        updates = []
        for index, track_id in enumerate((7, 7, 21, 21, 21)):
            updates.append(
                registry.observe(
                    TargetObservation(
                        source_uav="uav1",
                        stamp=20.0 + 0.1 * index,
                        class_id=2,
                        position=(10.0 + 0.15 * index, -4.0, 0.0),
                        confidence=0.9,
                        track_id=track_id,
                        track_id_is_stable=True,
                    )
                )
            )
        self.assertEqual(len(registry.targets), 1)
        self.assertEqual(
            updates[-1].target.source_tracks,
            {("uav1", "", 7), ("uav1", "", 21)},
        )
        self.assertTrue(updates[-1].newly_confirmed)

    def test_same_frame_different_classes_remain_distinct_targets(self):
        registry = GlobalTargetRegistry(cross_class_association_radius_m=3.0)
        first = registry.observe(
            TargetObservation(
                "uav1", 10.0, 2, (0.0, 0.0, 0.0), 0.9,
                track_id=1, track_id_is_stable=True,
            )
        )
        second = registry.observe(
            TargetObservation(
                "uav1", 10.0, 5, (0.5, 0.0, 0.0), 0.9,
                track_id=2, track_id_is_stable=True,
            )
        )
        self.assertNotEqual(first.target.target_id, second.target.target_id)
        self.assertEqual(len(registry.targets), 2)

    def test_same_frame_stable_tracks_protect_near_same_class_targets(self):
        registry = GlobalTargetRegistry(association_radius_m=4.0)
        first = registry.observe(
            TargetObservation(
                "uav1", 10.0, 2, (0.0, 0.0, 0.0), 0.9,
                track_id=1, track_id_is_stable=True,
            )
        ).target
        second = registry.observe(
            TargetObservation(
                "uav1", 10.0, 2, (0.5, 0.0, 0.0), 0.9,
                track_id=2, track_id_is_stable=True,
            )
        ).target
        self.assertNotEqual(first.target_id, second.target_id)
        self.assertIn(second.target_id, first.known_distinct_target_ids)
        self.assertIn(first.target_id, second.known_distinct_target_ids)

        first.status = TARGET_CONFIRMED
        second.status = TARGET_CONFIRMED
        allocator = RescueTaskAllocator()
        first_task = allocator.ensure_task(first, duplicate_radius_m=3.0)
        second_task = allocator.ensure_task(second, duplicate_radius_m=3.0)
        self.assertNotEqual(first_task.task_id, second_task.task_id)

    def test_unstable_world_positions_do_not_confirm(self):
        registry = GlobalTargetRegistry(
            association_radius_m=5.0,
            confirmation_hits=5,
            confirmation_minimum_span_sec=0.3,
            maximum_confirmation_position_spread_m=3.0,
        )
        updates = [
            registry.observe(self.observation("uav1", 10.0 + 0.1 * index, x, 0.0))
            for index, x in enumerate((0.0, 4.0, 0.0, 4.0, 0.0))
        ]
        self.assertFalse(any(update.newly_confirmed for update in updates))
        self.assertEqual(updates[-1].target.status, TARGET_CANDIDATE)

    def test_ambiguous_class_votes_do_not_confirm(self):
        registry = GlobalTargetRegistry(
            confirmation_hits=5,
            confirmation_minimum_span_sec=0.3,
            class_confirmation_minimum_ratio=0.65,
        )
        updates = []
        for index, class_id in enumerate((2, 5, 2, 5, 2)):
            updates.append(
                registry.observe(
                    TargetObservation(
                        source_uav="uav1",
                        stamp=10.0 + 0.1 * index,
                        class_id=class_id,
                        position=(1.0, 1.0, 0.0),
                        confidence=1.0,
                        track_id=3,
                        track_id_is_stable=True,
                    )
                )
            )
        self.assertFalse(any(update.newly_confirmed for update in updates))
        self.assertEqual(len(registry.targets), 1)

    def test_fixedwing_evidence_requests_verification_but_cannot_confirm(self):
        registry = GlobalTargetRegistry(
            confirmation_hits=2,
            confirmation_minimum_span_sec=0.1,
            confirmation_distinct_uavs=2,
            confirmation_vehicle_types=("multirotor",),
        )
        first = TargetObservation(
            "fw1", 10.0, 0, (100.0, 50.0, 0.0), 1.0,
            (25.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0, 0.0, 4.0),
            "fixedwing",
        )
        second = TargetObservation(
            "fw1", 10.2, 0, (101.0, 50.0, 0.0), 1.0,
            (25.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0, 0.0, 4.0),
            "fixedwing",
        )
        registry.observe(first)
        update = registry.observe(second)
        self.assertTrue(update.newly_evidence_ready)
        self.assertFalse(update.newly_confirmed)
        self.assertEqual(update.target.status, TARGET_CANDIDATE)

    def test_multirotor_confirms_a_verifying_fixedwing_candidate(self):
        registry = GlobalTargetRegistry(
            confirmation_hits=2,
            confirmation_minimum_span_sec=0.1,
            confirmation_distinct_uavs=2,
            confirmation_vehicle_types=("multirotor",),
            association_radius_m=4.0,
            association_covariance_sigma=3.0,
            maximum_association_radius_m=50.0,
        )
        coarse = TargetObservation(
            "fw1", 10.0, 0, (100.0, 50.0, 0.0), 0.8,
            (100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 9.0),
            "fixedwing",
        )
        target = registry.observe(coarse).target
        registry.set_status(target.target_id, TARGET_VERIFYING)
        registry.observe(
            TargetObservation(
                "uav2", 20.0, 0, (112.0, 50.0, 0.0), 1.0,
                (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
                "multirotor",
            )
        )
        registry.observe(
            TargetObservation(
                "uav2", 20.2, 0, (112.2, 50.1, 0.0), 1.0,
                (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
                "multirotor",
            )
        )
        update = registry.observe(
            TargetObservation(
                "uav2", 20.4, 0, (112.1, 49.9, 0.0), 1.0,
                (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
                "multirotor",
            )
        )
        self.assertEqual(len(registry.targets), 1)
        self.assertTrue(update.newly_confirmed)
        self.assertEqual(update.target.status, TARGET_CONFIRMED)
        self.assertLess(abs(update.target.position[0] - 112.1), 1.0)

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
        self.assertFalse(confirmation.newly_confirmed)
        confirmation = registry.observe(
            self.observation("uav1", 10.2, -15.75, -23.25)
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

    def test_final_task_guard_suppresses_only_near_same_class_duplicate(self):
        allocator = RescueTaskAllocator()
        first_target = GlobalTargetRecord(
            1, 2, [10.0, 10.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0,
            status=TARGET_CONFIRMED,
        )
        duplicate_target = GlobalTargetRecord(
            2, 2, [12.0, 10.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0,
            status=TARGET_CONFIRMED,
        )
        different_class = GlobalTargetRecord(
            3, 5, [11.0, 10.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0,
            status=TARGET_CONFIRMED,
        )

        first = allocator.ensure_task(first_target, duplicate_radius_m=3.0)
        duplicate = allocator.ensure_task(duplicate_target, duplicate_radius_m=3.0)
        second = allocator.ensure_task(different_class, duplicate_radius_m=3.0)

        self.assertIs(duplicate, first)
        self.assertIs(
            allocator.ensure_task(duplicate_target, duplicate_radius_m=3.0),
            first,
        )
        self.assertEqual(first.target_position[:2], [10.0, 10.0])
        self.assertNotEqual(second.task_id, first.task_id)
        self.assertEqual(len(allocator.tasks), 2)
        self.assertIsNone(allocator.remove_target_task(duplicate_target.target_id))
        self.assertIn(first.task_id, allocator.tasks)

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

    def test_rescue_task_can_require_a_multirotor_worker(self):
        allocator = RescueTaskAllocator()
        allocator.update_worker(
            "fw1", (1.0, 0.0, 50.0), 1.0, True, vehicle_type="fixedwing"
        )
        allocator.update_worker(
            "uav3", (20.0, 0.0, 10.0), 1.0, True, vehicle_type="multirotor"
        )
        target = GlobalTargetRecord(
            1, 0, [0.0, 0.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0,
            status=TARGET_CONFIRMED,
        )
        task = allocator.ensure_task(
            target, allowed_vehicle_types=("multirotor",)
        )
        assignment = allocator.assign_pending()
        self.assertEqual(assignment[0][1].name, "uav3")
        self.assertEqual(task.allowed_vehicle_types, ("multirotor",))

    def test_incompatible_high_priority_task_does_not_block_later_task(self):
        allocator = RescueTaskAllocator()
        allocator.update_worker(
            "fw1", (0.0, 0.0, 50.0), 1.0, True, vehicle_type="fixedwing"
        )
        first_target = GlobalTargetRecord(
            1, 0, [10.0, 0.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0,
            status=TARGET_CONFIRMED,
        )
        second_target = GlobalTargetRecord(
            2, 1, [20.0, 0.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0,
            status=TARGET_CONFIRMED,
        )
        blocked = allocator.ensure_task(
            first_target,
            priority=10,
            allowed_vehicle_types=("multirotor",),
        )
        eligible = allocator.ensure_task(
            second_target,
            priority=1,
            allowed_vehicle_types=("fixedwing",),
        )
        assignments = allocator.assign_pending()
        self.assertEqual(blocked.status, TASK_PENDING)
        self.assertEqual(assignments[0][0].task_id, eligible.task_id)
        self.assertEqual(assignments[0][1].name, "fw1")

    def test_cancel_and_retry_task_release_worker(self):
        allocator = RescueTaskAllocator()
        allocator.update_worker("uav3", (0.0, 0.0, 10.0), 1.0, True)
        target = GlobalTargetRecord(
            1, 0, [10.0, 0.0, 0.0], [0.0] * 9, 1.0, 1.0, 1.0,
            status=TARGET_CONFIRMED,
        )
        task = allocator.ensure_task(target)
        allocator.assign_pending()

        allocator.cancel_task(task.task_id, "operator cancelled")

        self.assertEqual(task.status, TASK_FAILED)
        self.assertEqual(allocator.workers["uav3"].assigned_task, None)
        allocator.retry_task(task.task_id, "operator retry")
        self.assertEqual(task.status, TASK_PENDING)
        self.assertEqual(task.assigned_worker, "")

    def test_clear_retains_workers_but_resets_target_and_task_ids(self):
        registry = GlobalTargetRegistry(confirmation_hits=1)
        update = registry.observe(self.observation("uav1", 1.0, 0.0, 0.0))
        allocator = RescueTaskAllocator()
        allocator.register_worker("uav3")
        target = update.target
        target.status = TARGET_CONFIRMED
        allocator.ensure_task(target)

        registry.clear()
        allocator.clear_tasks()

        self.assertEqual(registry.targets, {})
        self.assertEqual(allocator.tasks, {})
        self.assertIn("uav3", allocator.workers)
        next_update = registry.observe(self.observation("uav1", 2.0, 1.0, 0.0))
        self.assertEqual(next_update.target.target_id, 1)


if __name__ == "__main__":
    unittest.main()
