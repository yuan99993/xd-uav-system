#!/usr/bin/env python3

import math
import os
import queue
import random
import sys
import unittest
from unittest import mock

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from xd_uav_sead.airspace.airspace_manager import AirspaceManager, ZoneDef
from xd_uav_sead.formation.formation_control import (
    FormationConfig,
    FormationController,
    FormationShape,
)
from xd_uav_sead.planning.GA_SEAD_process import (
    GA_SEAD,
    _point_in_poly,
    plan_path_with_avoidance,
)
from xd_uav_sead.comms.communication_info import FrameType, packet_processing
from xd_uav_sead.planning.pathFollowing import CraigReynolds_Path_Following
from xd_uav_sead.planning import DPGA
from xd_uav_sead.strike.simple_strike import SimpleStrikeManager
from xd_uav_sead.drone.drone import Drone

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from fixedwing_nofly_acceptance import FixedwingNoFlyAcceptance


class AirspaceFunctionTest(unittest.TestCase):
    def test_zone_lifecycle_altitude_and_export(self):
        manager = AirspaceManager()
        zone = ZoneDef(
            zone_id=7,
            enabled=True,
            zone_type=0,
            level2d=1,
            levelH=0,
            minAlt=20.0,
            maxAlt=80.0,
            vertices=[(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)],
        )
        manager.update_zone(zone)

        self.assertTrue(manager.is_in_nofly(0.0, 0.0, 50.0))
        self.assertFalse(manager.is_in_nofly(0.0, 0.0, 10.0))
        self.assertFalse(manager.is_in_nofly(20.0, 0.0, 50.0))
        self.assertEqual(manager.export_zones_for_planner()[0]["zone_id"], 7)

        manager.remove_zone(7)
        self.assertFalse(manager.is_in_nofly(0.0, 0.0, 50.0))


class SharedFrameFunctionTest(unittest.TestCase):
    def test_acceptance_waits_for_mavros_parameter_sync_before_setting(self):
        acceptance = object.__new__(FixedwingNoFlyAcceptance)
        acceptance.namespace = "/uav1"
        unsynchronized = mock.Mock(success=False)
        synchronized = mock.Mock(success=True)
        synchronized.value.integer = 1
        verified = mock.Mock(success=True)
        verified.value.integer = 4
        getter = mock.Mock(
            side_effect=[
                rospy.ServiceException("receiving not complete"),
                unsynchronized,
                synchronized,
                verified,
            ]
        )
        setter_response = mock.Mock(success=True)
        setter_response.value.integer = 4
        setter = mock.Mock(return_value=setter_response)

        def service_proxy(name, _service_type):
            return setter if name.endswith("/set") else getter

        with mock.patch(
            "fixedwing_nofly_acceptance.rospy.wait_for_service"
        ), mock.patch(
            "fixedwing_nofly_acceptance.rospy.ServiceProxy",
            side_effect=service_proxy,
        ), mock.patch(
            "fixedwing_nofly_acceptance.rospy.is_shutdown", return_value=False
        ), mock.patch(
            "fixedwing_nofly_acceptance.rospy.logwarn_throttle"
        ), mock.patch(
            "fixedwing_nofly_acceptance.time.monotonic",
            side_effect=[10.0, 10.1, 10.2],
        ), mock.patch(
            "fixedwing_nofly_acceptance.time.sleep"
        ) as sleep_mock:
            acceptance._set_and_verify_px4_int_param("COM_RC_IN_MODE", 4)

        self.assertEqual(getter.call_count, 4)
        setter.assert_called_once()
        self.assertEqual(setter.call_args.args[0].value.integer, 4)
        self.assertEqual(sleep_mock.call_count, 3)

    def test_headless_sitl_bypass_is_explicit_and_fail_closed(self):
        drone = object.__new__(Drone)
        drone.headless_sitl_failsafe_bypass = True
        calls = []
        drone._set_px4_param = lambda name, value_int=None, value_real=None: (
            calls.append((name, value_int, value_real)) or True
        )
        with mock.patch("xd_uav_sead.drone.drone.rospy.sleep"):
            self.assertTrue(drone._configure_fixedwing_takeoff_safety())
        self.assertEqual(
            calls,
            [
                ("COM_RC_IN_MODE", 4, None),
                ("COM_RCL_EXCEPT", 6, None),
                ("NAV_DLL_ACT", 0, None),
                ("SIM_BAT_DRAIN", None, 86400.0),
                ("SIM_BAT_MIN_PCT", None, 100.0),
            ],
        )

        drone._set_px4_param = mock.Mock(side_effect=[True, False])
        with mock.patch("xd_uav_sead.drone.drone.rospy.sleep"):
            self.assertFalse(drone._configure_fixedwing_takeoff_safety())

    def test_acceptance_verifies_float_px4_parameter_after_setting(self):
        acceptance = object.__new__(FixedwingNoFlyAcceptance)
        acceptance.namespace = "/uav1"
        setter = mock.Mock(return_value=mock.Mock(success=True))
        actual = mock.Mock(success=True)
        actual.value.real = 86400.0
        getter = mock.Mock(return_value=actual)

        def service_proxy(name, _service_type):
            return setter if name.endswith("/set") else getter

        with mock.patch(
            "fixedwing_nofly_acceptance.rospy.wait_for_service"
        ), mock.patch(
            "fixedwing_nofly_acceptance.rospy.ServiceProxy",
            side_effect=service_proxy,
        ), mock.patch("fixedwing_nofly_acceptance.time.sleep"):
            acceptance._set_and_verify_px4_float_param(
                "SIM_BAT_DRAIN", 86400.0
            )

        setter.assert_called_once()
        self.assertEqual(setter.call_args.args[0].value.real, 86400.0)
        getter.assert_called_once_with(param_id="SIM_BAT_DRAIN")

    def test_uav_classifier_uses_standard_mav_type(self):
        drone = object.__new__(Drone)
        drone.frame_type = None

        drone.get_param = mock.Mock(return_value=mock.Mock(integer=1))
        self.assertEqual(drone.uav_classifier(), FrameType.Fixed_wing)
        drone.get_param.assert_called_once_with("MAV_TYPE")

        drone.frame_type = None
        drone.get_param = mock.Mock(return_value=mock.Mock(integer=2))
        self.assertEqual(drone.uav_classifier(), FrameType.Quad)

        drone.frame_type = None
        drone.get_param = mock.Mock(return_value=mock.Mock(integer=10))
        self.assertIsNone(drone.uav_classifier())
        self.assertIsNone(drone.frame_type)

    def test_uav_classifier_waits_for_delayed_mavros_parameter_sync(self):
        drone = object.__new__(Drone)
        drone.frame_type = None
        attempts = []

        def classify():
            attempts.append(True)
            if len(attempts) == 3:
                drone.frame_type = FrameType.Fixed_wing
            return drone.frame_type

        drone.uav_classifier = classify
        with mock.patch(
            "xd_uav_sead.drone.drone.monotonic",
            side_effect=[10.0, 10.1, 10.2],
        ), mock.patch(
            "xd_uav_sead.drone.drone.wall_sleep"
        ) as sleep_mock, mock.patch(
            "xd_uav_sead.drone.drone.rospy.is_shutdown", return_value=False
        ):
            self.assertEqual(
                drone._wait_for_supported_frame_type(5.0),
                FrameType.Fixed_wing,
            )
        self.assertEqual(len(attempts), 3)
        self.assertEqual(sleep_mock.call_count, 2)

    def test_uav_classifier_timeout_remains_fail_closed(self):
        drone = object.__new__(Drone)
        drone.frame_type = None
        drone.uav_classifier = mock.Mock(return_value=None)
        with mock.patch(
            "xd_uav_sead.drone.drone.monotonic",
            side_effect=[20.0, 20.2, 21.1],
        ), mock.patch(
            "xd_uav_sead.drone.drone.wall_sleep"
        ), mock.patch(
            "xd_uav_sead.drone.drone.rospy.is_shutdown", return_value=False
        ):
            with self.assertRaisesRegex(RuntimeError, "within 1.0s"):
                drone._wait_for_supported_frame_type(1.0)

    def test_manager_fixedwing_takeoff_applies_explicit_sitl_safety_first(self):
        drone = object.__new__(Drone)
        drone.uav_name = "uav1"
        drone.uses_external_control_manager = True
        drone.frame_type = FrameType.Fixed_wing
        drone.headless_sitl_failsafe_bypass = True
        order = []
        drone._configure_fixedwing_takeoff_safety = lambda: (
            order.append("safety") or True
        )
        response = mock.Mock(success=True, message="accepted")

        def service_proxy(_name, _service_type):
            return lambda altitude: (order.append(("takeoff", altitude)) or response)

        with mock.patch(
            "xd_uav_sead.drone.drone.rospy.wait_for_service"
        ), mock.patch(
            "xd_uav_sead.drone.drone.rospy.ServiceProxy",
            side_effect=service_proxy,
        ):
            self.assertTrue(drone.takeoff(30.0))
        self.assertEqual(order, ["safety", ("takeoff", 30.0)])

    def test_shared_waypoint_is_converted_back_to_control_local_frame(self):
        drone = object.__new__(Drone)
        drone.shared_frame_enabled = True
        drone.shared_frame_offset = [4.0, -2.0, 0.25]
        self.assertEqual(
            drone.shared_to_control_waypoint([10.0, 5.0, 3.0]),
            [6.0, 7.0, 2.75],
        )
        drone.shared_frame_enabled = False
        self.assertEqual(
            drone.shared_to_control_waypoint([10.0, 5.0, 3.0]),
            [10.0, 5.0, 3.0],
        )

    def test_external_manager_is_cancelled_before_px4_loiter(self):
        drone = object.__new__(Drone)
        drone.uav_name = "uav1"
        drone.ns_mavros = "/uav1/mavros"
        drone.uses_external_control_manager = True
        drone.keepoffboard = [1.0, 2.0, 3.0]
        drone.defaultoffboard = [1.0, 2.0, 3.0]
        order = []

        def cancel(service):
            order.append(service)
            return True

        drone._call_manager_trigger = cancel
        response = mock.Mock(mode_sent=True)
        with mock.patch(
            "xd_uav_sead.drone.drone.rospy.wait_for_service"
        ), mock.patch(
            "xd_uav_sead.drone.drone.rospy.ServiceProxy",
            return_value=lambda **kwargs: (order.append("AUTO.LOITER") or response),
        ):
            self.assertTrue(drone.enter_fail_closed_loiter())
        self.assertEqual(order, ["cancel_offboard", "AUTO.LOITER"])
        self.assertIsNone(drone.keepoffboard)

    def test_external_loiter_mode_hands_control_back_to_px4(self):
        drone = object.__new__(Drone)
        drone.uses_external_control_manager = True
        drone._handoff_external_mode = mock.Mock(return_value=True)

        self.assertTrue(drone.set_mode("LOITER"))
        drone._handoff_external_mode.assert_called_once_with("AUTO.LOITER")


class PathFollowingFunctionTest(unittest.TestCase):
    def test_dpga_position_waypoint_keeps_flight_altitude_not_path_heading(self):
        process = object.__new__(DPGA.main_process)
        process.control_mode = "position_waypoint"
        process.path_update_flag = True
        process.path_following = mock.Mock()
        process.path_following.path = [[100.0, 20.0, 1.57]]
        process.path_following.get_fixed_wing_waypoint.return_value = [
            100.0,
            20.0,
            1.57,
        ]
        uav = mock.Mock()
        uav.local_pose = [10.0, 5.0, 24.0]

        process._follow_fixedwing_path(uav, height=24.0)

        uav.guide_to_waypoint.assert_called_once_with([100.0, 20.0, 24.0])

    def test_fixed_wing_waypoint_progresses_forward(self):
        follower = CraigReynolds_Path_Following(
            method=None,
            recedingHorizon=2.0,
            path=[[0.0, 0.0, 100.0], [100.0, 0.0, 100.0], [200.0, 0.0, 100.0]],
        )
        follower.Rmin = 20.0

        self.assertEqual(follower.get_fixed_wing_waypoint(0.0, 0.0), [100.0, 0.0, 100.0])
        self.assertEqual(follower.get_fixed_wing_waypoint(100.0, 0.0), [200.0, 0.0, 100.0])
        self.assertIsNone(follower.get_fixed_wing_waypoint(200.0, 0.0))

    def test_fixed_wing_waypoint_replan_anchors_to_nearest_path_point(self):
        follower = CraigReynolds_Path_Following(
            method=None,
            recedingHorizon=2.0,
            path=[[float(x), 0.0, 0.0] for x in range(0, 101, 5)],
        )
        follower.Rmin = 60.0

        waypoint = follower.get_fixed_wing_waypoint(
            52.0, 3.0, update=True, lookahead_dist=20.0
        )

        self.assertEqual(waypoint[:2], [70.0, 0.0])
        self.assertEqual(follower.fw_path_index, 10)

    def test_fixed_wing_waypoint_does_not_jump_to_return_leg_near_home(self):
        outbound = [[float(x), 0.0, 0.0] for x in range(0, 205, 5)]
        returning = [[float(x), 5.0, math.pi] for x in range(200, -5, -5)]
        follower = CraigReynolds_Path_Following(
            method=None,
            recedingHorizon=2.0,
            path=outbound + returning,
        )
        follower.Rmin = 60.0

        waypoint = follower.get_fixed_wing_waypoint(
            0.0, 4.9, update=True, lookahead_dist=30.0
        )

        self.assertEqual(waypoint[:2], [30.0, 0.0])
        self.assertEqual(follower.fw_path_index, 0)

    def test_v9_zone_selection_uses_outbound_arc_not_return_leg(self):
        acceptance = object.__new__(FixedwingNoFlyAcceptance)
        acceptance.turning_radius = 20.0
        acceptance.zone_ahead_distance = 75.0
        acceptance.zone_half_size = 5.0
        path = Path()
        for x, y in (
            (0.0, 0.0),
            (50.0, 0.0),
            (100.0, 0.0),
            (100.0, 50.0),
            (50.0, 50.0),
            (0.0, 1.0),
        ):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)

        rectangle = acceptance._choose_zone(path, 0.0, 0.0)

        self.assertEqual(rectangle, (95.0, 105.0, -5.0, 5.0))

    def test_v9_profiles_keep_larger_turn_radius_while_relaxing_zone(self):
        profiles = FixedwingNoFlyAcceptance.PROFILES
        self.assertGreaterEqual(min(p["turning_radius"] for p in profiles.values()), 60.0)
        self.assertGreater(
            profiles["nominal"]["planning_clearance"],
            profiles["relaxed1"]["planning_clearance"],
        )
        self.assertGreater(
            profiles["relaxed1"]["planning_clearance"],
            profiles["relaxed2"]["planning_clearance"],
        )


class FormationFunctionTest(unittest.TestCase):
    def test_all_shapes_produce_unique_safe_slots(self):
        team_ids = [1, 2, 3, 4, 5]
        spacing = 100.0
        for shape in FormationShape:
            with self.subTest(shape=shape.name):
                controller = FormationController(
                    3,
                    FormationConfig(
                        shape=shape,
                        spacing=spacing,
                        safe_separation=40.0,
                        leader_id=3,
                    ),
                )
                slots = controller._slot_map(team_ids)
                self.assertEqual(set(slots), set(team_ids))
                points = [slots[uid][:2] for uid in team_ids]
                for i, first in enumerate(points):
                    self.assertTrue(np.all(np.isfinite(first)))
                    for second in points[i + 1 :]:
                        self.assertGreaterEqual(
                            float(np.linalg.norm(first - second)),
                            40.0,
                        )

    def test_mission_transitions_from_assemble_to_hold(self):
        controller = FormationController(
            1,
            FormationConfig(
                shape=FormationShape.VEE,
                spacing=60.0,
                safe_separation=40.0,
                standoff_distance=100.0,
                hold_radius=30.0,
                leader_id=1,
            ),
        )
        self.assertTrue(
            controller.start_mission(
                targets=[[1000.0, 0.0]],
                base_config=[0.0, 0.0, 100.0],
                current_position=[0.0, 0.0, 100.0],
                cruise_speed=20.0,
                nominal_altitude=100.0,
                team_ids=[1],
                now=0.0,
            )
        )
        assembling = controller.step(
            local_position=[0.0, 0.0, 100.0],
            local_velocity=[0.0, 0.0, 0.0],
            local_yaw=0.0,
            now=1.0,
            team_ids=[1],
        )
        self.assertTrue(assembling["active"])
        self.assertFalse(assembling["released"])
        self.assertTrue(np.all(np.isfinite(assembling["waypoint"])))

        local_position = list(assembling["waypoint"])
        holding = assembling
        for tick in range(2, 60):
            holding = controller.step(
                local_position=local_position,
                local_velocity=[20.0, 0.0, 0.0],
                local_yaw=0.0,
                now=float(tick),
                team_ids=[1],
            )
            local_position = list(holding["waypoint"])
            if holding["phase"] == 2:
                break
        self.assertTrue(holding["active"])
        self.assertEqual(holding["phase"], 2)

    def test_multirotor_profile_preserves_low_formation_altitude(self):
        controller = FormationController(
            1,
            FormationConfig(
                shape=FormationShape.VEE,
                spacing=4.0,
                safe_separation=2.0,
                minimum_altitude=1.0,
            ),
        )
        self.assertTrue(
            controller.start_rally(
                [10.0, 0.0, 3.5], 8.0, [0.0, 0.0, 3.0], 3.0, 3.0,
                team_ids=[1], now=0.0,
            )
        )
        command = controller.step_rally([0.0, 0.0, 3.0], 0.0, 0.1, [1])
        self.assertLess(command["waypoint"][2], 5.0)
        self.assertAlmostEqual(command["waypoint"][2], 3.5)

    def test_multirotor_static_hold_keeps_fixed_slot(self):
        config = FormationConfig(
            shape=FormationShape.VEE,
            spacing=4.0,
            minimum_altitude=1.0,
            static_hold=True,
        )
        controller = FormationController(1, config)
        controller.rally_point = np.array([25.0, 3.0, 3.5])
        controller.rally_started_at = 1.0
        controller.nominal_altitude = 3.5
        controller.approach_dir = np.array([1.0, 0.0])
        slot_offset = np.array([-3.8, 3.28, 0.0])

        first = controller._hold_waypoint(1, 3, slot_offset, controller.rally_point, 5.0)
        later = controller._hold_waypoint(1, 3, slot_offset, controller.rally_point, 50.0)

        np.testing.assert_allclose(first, later)
        np.testing.assert_allclose(first, [21.2, 6.28, 3.5])

    def test_three_uav_trail_transitions_to_centered_vee(self):
        controller = FormationController(
            2,
            FormationConfig(
                shape=FormationShape.TRAIL,
                spacing=100.0,
                safe_separation=50.0,
                leader_id=1,
            ),
        )
        self.assertTrue(
            controller.start_rally(
                point=[1000.0, 0.0, 100.0],
                loiter_radius=200.0,
                current_position=[0.0, 0.0, 100.0],
                cruise_speed=20.0,
                nominal_altitude=100.0,
                team_ids=[1, 2, 3],
                now=10.0,
            )
        )
        controller.update_remote_state(
            {"uav_id": 1, "timestamp": 10.0, "position": [100.0, 0.0, 100.0],
             "velocity": [0.0, 0.0, 0.0], "yaw": 0.0, "phase": 1, "slot_id": 0}
        )
        controller.update_remote_state(
            {"uav_id": 3, "timestamp": 10.0, "position": [-100.0, 0.0, 100.0],
             "velocity": [0.0, 0.0, 0.0], "yaw": 0.0, "phase": 1, "slot_id": 2}
        )
        started = controller.start_trail_to_vee_centered(
            [0.0, 0.0, 100.0], 10.0, [1, 2, 3], entry_distance=1000.0
        )
        self.assertEqual(started["transition_event"], "start")
        self.assertEqual(started["center_id"], 2)

        completed = controller.step_rally(
            [0.0, 0.0, 100.0], 0.0, 14.0, [1, 2, 3]
        )
        self.assertEqual(completed["transition"]["transition_event"], "complete")
        self.assertEqual(controller.config.shape, FormationShape.VEE)
        self.assertTrue(controller.trail_to_vee_transition_completed)
        self.assertEqual(controller.frozen_slot_order[0], 2)


class PlannerFunctionTest(unittest.TestCase):
    def test_dubins_avoidance_path_stays_outside_zone(self):
        zone = {
            "zone_id": 1,
            "minAlt": 0.0,
            "maxAlt": 200.0,
            "poly": [(80.0, -30.0), (120.0, -30.0), (120.0, 30.0), (80.0, 30.0)],
        }
        path = plan_path_with_avoidance(
            (0.0, 0.0, 0.0),
            (200.0, 0.0, 0.0),
            20.0,
            [zone],
            20.0,
            sampling_step=3.0,
        )

        self.assertGreater(len(path), 2)
        self.assertTrue(all(math.isfinite(float(v)) for point in path for v in point[:2]))
        self.assertFalse(
            any(_point_in_poly(float(point[0]), float(point[1]), zone["poly"]) for point in path)
        )

    def test_real_ga_allocates_three_mission_stages(self):
        random.seed(7)
        np.random.seed(7)
        ga_input = [
            [1, 2, 3],
            [1, 2, 3],
            [15.0, 18.0, 16.0],
            [20.0, 20.0, 20.0],
            [[0.0, -50.0, 0.0], [0.0, 0.0, 0.0], [0.0, 50.0, 0.0]],
            [[0.0, -50.0, 0.0], [0.0, 0.0, 0.0], [0.0, 50.0, 0.0]],
            [None, None, None],
            [[], [], []],
            [],
            [],
            [],
        ]
        solution, fitness, population, curve = GA_SEAD(
            [[150.0, 0.0]], population_size=18
        ).run_GA(3, ga_input)
        self.assertTrue(math.isfinite(float(fitness)))
        self.assertGreater(float(fitness), 0.0)
        self.assertTrue(population)
        self.assertEqual(solution[1], [1, 1, 1])
        assigned_by_task = dict(zip(solution[2], solution[3]))
        self.assertEqual(assigned_by_task, {1: 1, 2: 2, 3: 3})
        self.assertGreaterEqual(len(curve), 2)


class DpgaFunctionTest(unittest.TestCase):
    def test_single_uav_route_uses_normal_airspace_planner_without_ga(self):
        airspace = AirspaceManager()
        with mock.patch.object(DPGA.rospy, "Publisher", return_value=mock.Mock()):
            process = DPGA.main_process(
                targets_sites=[[300.0, 0.0]],
                unknown_targets=[],
                base_config=[600.0, 0.0, 0.0],
                u2u_communication=None,
                ga2control_queue=queue.Queue(),
                control2ga_queue=queue.Queue(),
                airspace=airspace,
                uav_id=1,
                control_mode="position_waypoint",
                zone_clearance=20.0,
            )
        uav = mock.Mock(
            type=2,
            v=15.0,
            Rmin=50.0,
            local_pose=[0.0, 0.0, 30.0],
            yaw=0.0,
        )
        process.publish_rviz_path = mock.Mock()
        self.assertTrue(process.configure_single_uav_route(uav))
        self.assertEqual(process.best_solution[3], [1])
        self.assertGreater(len(process.path_following.path), 2)
        process.publish_rviz_path.assert_called_once()

    def test_fixedwing_periodic_broadcast_always_seeds_local_roster(self):
        with mock.patch.object(DPGA.rospy, "Publisher", return_value=mock.Mock()):
            process = DPGA.main_process(
                targets_sites=[[300.0, 0.0]],
                unknown_targets=[],
                base_config=[0.0, 0.0, 0.0],
                u2u_communication=None,
                ga2control_queue=queue.Queue(),
                control2ga_queue=queue.Queue(),
                uav_id=1,
                control_mode="position_waypoint",
            )
        protocol = packet_processing(1)
        radio = mock.Mock()
        uav = mock.Mock(
            type=2,
            v=15.0,
            Rmin=50.0,
            local_pose=[12.0, -3.0, 30.0],
            yaw=0.25,
        )

        class _SingleGateTimer:
            def __init__(self):
                self.calls = 0

            def check_timer(self, *_args):
                self.calls += 1
                if self.calls > 1:
                    raise AssertionError("periodic gate was evaluated twice")
                return True

            @staticmethod
            def check_period(*_args):
                return False

        timer = _SingleGateTimer()
        process.run_fixedWing(radio, protocol, uav, timer, 0, 30.0, 50.0)
        self.assertEqual(timer.calls, 1)
        self.assertIn(1, protocol.uavs_info)
        self.assertEqual(protocol.uavs_info[1]["pos"], [12.0, -3.0, 0.25])
        radio.send_data_broadcast.assert_called_once()

    def test_team_state_is_stably_flattened_for_ga(self):
        with mock.patch.object(DPGA.rospy, "Publisher", return_value=mock.Mock()):
            process = DPGA.main_process(
                targets_sites=[[100.0, 0.0]],
                unknown_targets=[],
                base_config=[0.0, 0.0, 0.0],
                u2u_communication=None,
                ga2control_queue=queue.Queue(),
                control2ga_queue=queue.Queue(),
                uav_id=1,
                control_mode="position_waypoint",
            )
        team = {
            2: {
                "type": 2, "v": 20.0, "Rmin": 30.0,
                "pos": [20.0, 0.0, 0.0], "base": [0.0, 0.0, 0.0],
                "cost": 12.0, "chromosome": [[1], [1], [2], [2], [0]],
                "completed_tasks": [[1, 1]], "new_targets": [[300.0, 0.0]],
            },
            1: {
                "type": 1, "v": 15.0, "Rmin": 25.0,
                "pos": [0.0, 0.0, 0.0], "base": [0.0, 0.0, 0.0],
                "cost": 10.0, "chromosome": [[1], [1], [1], [1], [0]],
                "completed_tasks": [[2, 2]], "new_targets": [[200.0, 0.0]],
            },
        }
        zones = [{"zone_id": 9, "poly": [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)]}]
        ga_input = process.build_ga_input_from_dict(team, zones)
        self.assertEqual(ga_input[0], [1, 2])
        self.assertEqual(ga_input[8], [[2, 2], [1, 1]])
        self.assertEqual(ga_input[9], [[200.0, 0.0], [300.0, 0.0]])
        self.assertEqual(ga_input[10], zones)

    def test_task_allocator_emits_result_and_honors_stop_sentinel(self):
        class _FakeGa:
            def __init__(self, targets, population_size):
                self.targets = targets

            def run_GA_time_period_version(self, interval, uavs, population, update):
                return "solution", 0.25, "population"

        ga_to_control = queue.Queue()
        control_to_ga = queue.Queue()
        control_to_ga.put(["initial team state"])
        control_to_ga.put([44])
        with mock.patch.object(DPGA, "GA_SEAD", _FakeGa):
            DPGA.task_allocation_process(
                [[100.0, 0.0]],
                0.01,
                4,
                ga_to_control,
                control_to_ga,
            )
        self.assertEqual(ga_to_control.get_nowait(), [0.25, "solution"])
        self.assertTrue(control_to_ga.empty())

    def test_dynamic_zone_replans_dpga_remaining_path(self):
        airspace = AirspaceManager()
        with mock.patch.object(DPGA.rospy, "Publisher", return_value=mock.Mock()):
            process = DPGA.main_process(
                targets_sites=[[220.0, 0.0]],
                unknown_targets=[],
                base_config=[300.0, 0.0, 0.0],
                u2u_communication=None,
                ga2control_queue=queue.Queue(),
                control2ga_queue=queue.Queue(),
                airspace=airspace,
                uav_id=1,
                control_mode="position_waypoint",
                zone_clearance=20.0,
            )
        process.target = [[220.0, 0.0, 0.0, 1, 1], [300.0, 0.0, 0.0]]
        process.path_following.path = process._plan_state_sequence(
            [0.0, 0.0, 0.0], process.target, 15.0, 20.0, flight_altitude=60.0
        )
        process.path_following.fw_path_index = 0
        process.publish_rviz_path = mock.Mock()
        airspace.update_zone(
            ZoneDef(
                zone_id=9,
                enabled=True,
                zone_type=0,
                level2d=0,
                levelH=0,
                minAlt=20.0,
                maxAlt=120.0,
                vertices=[(80.0, -30.0), (120.0, -30.0), (120.0, 30.0), (80.0, 30.0)],
            )
        )
        uav = mock.Mock(
            local_pose=[0.0, 0.0, 60.0], yaw=0.0, v=15.0, Rmin=20.0
        )
        self.assertEqual(process.handle_airspace_update(uav), "replanned")
        self.assertGreaterEqual(
            airspace.path_min_horizontal_clearance(process.path_following.path, 60.0),
            20.0 - 1e-6,
        )

    def test_fixedwing_base_completion_does_not_index_cleared_target(self):
        process = object.__new__(DPGA.main_process)
        process.ga2control_queue = queue.Queue()
        process.control2ga_queue = queue.Queue()
        process.previous_time_u2u = 0.0
        process.previous_time_control = 0.0
        process.T = 1.0
        process.T_comm = 1.0
        process.back_to_base = True
        process.packet = []
        process.path_following = mock.Mock(path=[[0.0, 0.0, 0.0]])
        process.mission_flag = False
        process.target = [[0.0, 0.0, 0.0]]
        process.into = False
        process.reset = mock.Mock(side_effect=lambda: process.target.clear())
        timer = mock.Mock()
        timer.check_timer.return_value = False
        timer.check_period.return_value = False
        timer.t.return_value = 1.0
        uav = mock.Mock(local_pose=[0.0, 0.0, 30.0], Rmin=20.0)

        process.run_fixedWing(
            mock.Mock(), mock.Mock(), uav, timer, mock.Mock(), 30.0, 20.0
        )

        self.assertTrue(process.mission_flag)
        uav.set_mode.assert_called_once_with("LOITER")


class SimpleStrikeFunctionTest(unittest.TestCase):
    def test_targets_and_assignment_are_deterministic_and_one_to_one(self):
        targets = SimpleStrikeManager.canonical_targets(
            [[100.0, 0.0], [0.0, 0.0], [100.0, 0.0], [200.0, 0.0]]
        )
        self.assertEqual([item["point"] for item in targets], [[0.0, 0.0], [100.0, 0.0], [200.0, 0.0]])
        assignment = SimpleStrikeManager.build_greedy_assignment(
            [
                {"uav_id": 1, "pos": [0.0, 0.0]},
                {"uav_id": 2, "pos": [100.0, 0.0]},
                {"uav_id": 3, "pos": [200.0, 0.0]},
            ],
            targets,
        )
        self.assertEqual(set(assignment), {1, 2, 3})
        self.assertEqual(
            {uid: item["point"] for uid, item in assignment.items()},
            {1: [0.0, 0.0], 2: [100.0, 0.0], 3: [200.0, 0.0]},
        )

    def test_three_managers_aggregate_assignment_ack_and_path_status(self):
        targets = [[100.0, 0.0], [200.0, 0.0], [300.0, 0.0]]
        managers = {
            uid: SimpleStrikeManager(
                targets, [], [0.0, 0.0, 0.0], uid, [2, 20.0, 30.0],
                control_mode="position_waypoint",
            )
            for uid in [1, 2, 3]
        }
        leader = managers[1]
        leader.assignment_map = SimpleStrikeManager.build_greedy_assignment(
            [
                {"uav_id": 1, "pos": [90.0, 0.0]},
                {"uav_id": 2, "pos": [190.0, 0.0]},
                {"uav_id": 3, "pos": [290.0, 0.0]},
            ],
            leader.targets,
        )
        payload = {
            "leader_id": 1,
            "target_count": 3,
            "assignment_map": leader.assignment_map,
        }
        self.assertTrue(managers[2].apply_remote_assignment(payload))
        self.assertTrue(managers[3].apply_remote_assignment(payload))
        leader.assignment_missing_uavs = {2, 3}
        leader.assignment_broadcast_count = leader.assignment_min_broadcast_count
        for uid in [2, 3]:
            self.assertTrue(
                leader.apply_assignment_ack(
                    {"leader_id": 1, "target_count": 3, "sender_uav_id": uid}
                )
            )
        self.assertTrue(leader.assignment_ack_complete)

        for uid, remaining in [(1, 300.0), (2, 360.0), (3, 420.0)]:
            self.assertTrue(
                leader.apply_path_status(
                    {
                        "target_count": 3,
                        "status": {
                            "uav_id": uid,
                            "target_id": uid,
                            "path_ready": True,
                            "path_length": remaining + 100.0,
                            "remaining_path_length": remaining,
                            "sync_remaining_path_length": remaining,
                            "actual_groundspeed": 20.0,
                            "predicted_arrival_time": 1000.0 + remaining / 20.0,
                        },
                    }
                )
            )
        terminal = leader._team_terminal_arrival_context(1000.0)
        self.assertEqual(terminal["latest_uid"], 3)
        self.assertAlmostEqual(terminal["spread"], 6.0)

        class _Radio:
            def __init__(self):
                self.packets = []

            def send_data_broadcast(self, packet):
                self.packets.append(packet)

        radio = _Radio()
        protocol = packet_processing(1)
        self.assertTrue(
            leader._leader_ensure_full_path_common_hit_time(
                radio, protocol, {}, now_abs=1000.0
            )
        )
        self.assertGreater(leader.full_path_common_hit_time, 1000.0)
        decoded, info = protocol.unpack_packet(radio.packets[-1])
        self.assertAlmostEqual(info["release_time"], leader.full_path_common_hit_time)

    def test_dynamic_zone_replans_simple_strike_or_clears_unsafe_path(self):
        airspace = AirspaceManager()
        manager = SimpleStrikeManager(
            [[220.0, 0.0], [300.0, 100.0], [300.0, -100.0]],
            [],
            [0.0, 0.0, 0.0],
            1,
            [1, 15.0, 20.0],
            airspace=airspace,
            control_mode="position_waypoint",
        )
        manager.assignment_map = {
            1: {"target_id": 1, "point": [220.0, 0.0]}
        }
        manager.assigned_target = manager.assignment_map[1]
        uav = mock.Mock(
            local_pose=[0.0, 0.0, 60.0], yaw=0.0, v=15.0, Rmin=20.0
        )
        with mock.patch(
            "xd_uav_sead.strike.simple_strike.rospy.logwarn_throttle"
        ):
            self.assertTrue(manager._build_full_path_for_uav(uav, 60.0))

        airspace.update_zone(
            ZoneDef(
                zone_id=10,
                enabled=True,
                zone_type=0,
                level2d=0,
                levelH=0,
                minAlt=20.0,
                maxAlt=120.0,
                vertices=[(80.0, -30.0), (120.0, -30.0), (120.0, 30.0), (80.0, 30.0)],
            )
        )
        with mock.patch(
            "xd_uav_sead.strike.simple_strike.rospy.logwarn_throttle"
        ):
            self.assertEqual(manager.handle_airspace_update(uav, 60.0), "replanned")
        self.assertGreaterEqual(
            airspace.path_min_horizontal_clearance(manager.path_following.path, 60.0),
            manager.final_zone_clearance - 2.5,
        )

        airspace.update_zone(
            ZoneDef(
                zone_id=11,
                enabled=True,
                zone_type=0,
                level2d=0,
                levelH=0,
                minAlt=20.0,
                maxAlt=120.0,
                vertices=[(10.0, -30.0), (50.0, -30.0), (50.0, 30.0), (10.0, 30.0)],
            )
        )
        with mock.patch(
            "xd_uav_sead.strike.simple_strike.rospy.logwarn_throttle"
        ):
            self.assertEqual(manager.handle_airspace_update(uav, 60.0), "failed")
        self.assertEqual(manager.path_following.path, [])


if __name__ == "__main__":
    unittest.main()
