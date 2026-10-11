"""M5 contract tests; no ROS connection and no network access."""

import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import MagicMock, patch


BACKEND = Path(__file__).resolve().parent
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))

from pydantic import ValidationError
from fastapi import HTTPException

from app.main import SAMPLE_TIF
from app.terrain_resource import CANONICAL_DAVOSDORF_DEM
from swarm_app.contracts import (
    FleetOperationRequest,
    FleetSpawnRequest,
    SwarmDispatchRequest,
    VehicleRoute,
    WorldPoint,
    build_path_message,
)
from swarm_app.gateway import MockSwarmGateway, RosbridgeSwarmGateway
from swarm_app.main import (
    GeoCoordinate,
    WorldElevationCoordinate,
    _decorate_snapshot,
    _spawn_example,
    swarm_capabilities,
    swarm_dispatch,
    swarm_health,
    swarm_snapshot,
    swarm_to_world,
    swarm_world_elevation,
)
from swarm_app.terrain_frame import DavosdorfWorldFrame, TerrainWorldFrame
from swarm_app.terrain_frame import CANONICAL_DEM, WEB_DEM


def route(name="uav1", goal_id=6201, y_offset=0.0):
    return VehicleRoute(
        uav_name=name,
        goal_id=goal_id,
        nominal_speed_mps=15.0,
        points=[
            WorldPoint(x=-100.0, y=y_offset, z=2000.0),
            WorldPoint(x=100.0, y=y_offset, z=2000.0),
        ],
    )


class SwarmContractsTest(unittest.TestCase):
    def test_nav_path_matches_m2_m3_contract(self):
        message = build_path_message(route(), timestamp=100.25)
        self.assertEqual("world", message["header"]["frame_id"])
        self.assertEqual({"secs": 100, "nsecs": 250000000},
                         message["header"]["stamp"])
        self.assertEqual(2, len(message["poses"]))
        for pose in message["poses"]:
            self.assertEqual(6201, pose["header"]["seq"])
            self.assertEqual("world", pose["header"]["frame_id"])
            self.assertEqual(message["header"]["stamp"], pose["header"]["stamp"])
            quaternion = pose["pose"]["orientation"]
            norm = math.sqrt(sum(quaternion[key] ** 2 for key in "xyzw"))
            self.assertAlmostEqual(1.0, norm)

    def test_invalid_frame_names_ids_and_segments_fail_closed(self):
        with self.assertRaises(ValidationError):
            SwarmDispatchRequest(
                frame_id="map", routes=[route(), route("uav2", 6202)])
        with self.assertRaises(ValidationError):
            route("/uav1", 6201)
        with self.assertRaises(ValidationError):
            route("uav1", 0)
        with self.assertRaises(ValidationError):
            VehicleRoute(
                uav_name="uav1", goal_id=1,
                points=[WorldPoint(x=0.0, y=0.0, z=0.0),
                        WorldPoint(x=0.5, y=0.0, z=0.0)])

    def test_duplicate_vehicle_or_goal_fails_closed(self):
        with self.assertRaises(ValidationError):
            SwarmDispatchRequest(routes=[route(), route("uav1", 6202)])
        with self.assertRaises(ValidationError):
            SwarmDispatchRequest(routes=[route(), route("uav2", 6201)])

class TerrainFrameTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # These assertions describe the tracked Davosdorf fallback and must
        # not silently switch to whichever arbitrary DEM happens to be active
        # in /tmp on the developer machine.
        cls.frame = DavosdorfWorldFrame({})

    def test_web_and_canonical_dems_match(self):
        self.assertTrue(self.frame.canonical_match)
        self.assertEqual(self.frame.web_dem_sha256,
                         self.frame.canonical_dem_sha256)

    def test_both_web_modes_use_the_tracked_canonical_dem(self):
        expected = BACKEND.parents[1] / "terrain-navigation" / "resources" / "davosdorf.tif"
        self.assertEqual(expected, CANONICAL_DAVOSDORF_DEM)
        self.assertEqual(expected, SAMPLE_TIF)
        self.assertEqual(expected, CANONICAL_DEM)
        self.assertEqual(expected, WEB_DEM)
        self.assertTrue(expected.is_file())

    def test_world_wgs84_round_trip(self):
        for point in ((0.0, 0.0), (-2012.4, -1188.0), (2012.4, 1188.0)):
            longitude, latitude = self.frame.world_to_geo(*point)
            x, y = self.frame.geo_to_world(longitude, latitude)
            # GDAL's CH1903/WGS84 round trip is millimetre-scale here; that is
            # far tighter than browser map resolution and does not alter the
            # original ROS world coordinates used for dispatch.
            self.assertAlmostEqual(point[0], x, delta=0.002)
            self.assertAlmostEqual(point[1], y, delta=0.002)

    def test_metadata_and_overlay_are_real_dem_outputs(self):
        metadata = self.frame.metadata()
        self.assertEqual("EPSG:21781", metadata["projection"])
        self.assertEqual("world", metadata["frame_id"])
        self.assertEqual(3, len(metadata["default_routes"]))
        first, second, third = metadata["default_routes"]
        self.assertEqual(300.0,
                         abs(first["points"][0]["y"] -
                             second["points"][0]["y"]))
        self.assertEqual(600.0,
                         first["points"][1]["x"] -
                         first["points"][0]["x"])
        self.assertGreaterEqual(
            abs(second["points"][0]["y"] - third["points"][0]["y"]),
            300.0)
        self.assertGreater(metadata["elevation_max_m"],
                           metadata["elevation_min_m"])
        self.assertTrue(self.frame.overlay_png().startswith(b"\x89PNG\r\n\x1a\n"))

    def test_world_raster_is_bounded_and_keeps_dem_edges(self):
        raster = self.frame.raster_payload(max_grid=48)
        self.assertEqual("world", raster["frame_id"])
        self.assertEqual("MSL", raster["vertical_datum"])
        self.assertLessEqual(raster["rows"], 48)
        self.assertLessEqual(raster["cols"], 48)
        self.assertEqual(raster["rows"], len(raster["elevation"]))
        self.assertEqual(raster["cols"], len(raster["elevation"][0]))
        self.assertAlmostEqual(-raster["width_m"] / 2.0,
                               raster["world_bounds"]["x_min"])
        self.assertAlmostEqual(raster["height_m"] / 2.0,
                               raster["world_bounds"]["y_max"])
        self.assertTrue(any(value is not None
                            for row in raster["elevation"] for value in row))

    def test_eight_vehicle_examples_use_sampled_dem_origins(self):
        examples = self.frame.vehicle_examples()
        self.assertEqual(8, len(examples))
        self.assertEqual(["uav{}".format(index) for index in range(1, 9)],
                         [item["uav_name"] for item in examples])
        expected = [1542.6483154296875, 1569.193603515625, 1570.616943359375]
        for item, elevation in zip(examples, expected):
            self.assertAlmostEqual(elevation, item["world_origin"]["z"])
            self.assertAlmostEqual(
                elevation + 80.0, item["route"]["points"][0]["z"])
            self.assertAlmostEqual(
                item["world_origin"]["x"], item["gazebo_spawn"]["x"])
            self.assertAlmostEqual(
                item["world_origin"]["y"], item["gazebo_spawn"]["y"])
            self.assertAlmostEqual(
                self.frame.gazebo_spawn_elevation_at_world(
                    item["world_origin"]["x"], item["world_origin"]["y"]),
                item["gazebo_spawn"]["z"])
            self.assertLessEqual(item["launch_site"]["slope_rad"], 0.10)
            self.assertLessEqual(item["launch_site"]["relief_3x3_m"], 3.0)

    def test_unsafe_ground_launch_is_rejected_instead_of_disabling_failsafes(self):
        metrics = self.frame.launch_site_metrics(600.0, -1700.0)
        self.assertGreater(metrics["slope_rad"], 0.10)
        ground = metrics["ground_elevation_msl_m"]
        with patch("swarm_app.main.terrain_frame", return_value=self.frame):
            with self.assertRaises(HTTPException) as context:
                _spawn_example(FleetSpawnRequest(
                    uav_name="uav2", use_mock=True,
                    world_origin={"x": 600.0, "y": -1700.0, "z": ground}))
        self.assertEqual(409, context.exception.status_code)

    def test_locally_flat_site_with_unsafe_rollout_is_rejected(self):
        # This point passes the local slope gate, but Gazebo's eastbound
        # landing surface rises sharply after the touchdown point.
        x, y = 600.0, -1390.0
        self.frame.validate_launch_site(x, y)
        runway = self.frame.fixedwing_runway_metrics(x, y)
        self.assertGreater(runway["rollout_relief_m"], 2.0)
        with self.assertRaisesRegex(
                ValueError, "Gazebo 实际地表坡度|进近方向|着陆滑跑"):
            self.frame.validate_fixedwing_site(x, y)

    def test_gazebo_cross_slope_is_rejected_even_when_source_cell_is_flat(self):
        frame = object.__new__(TerrainWorldFrame)
        frame.validate_launch_site = lambda _x, _y: {
            "ground_elevation_msl_m": 100.0,
            "slope_rad": 0.0,
            "relief_3x3_m": 0.0,
        }
        # Eastbound approach/rollout is level, but the exact continuous
        # Gazebo surface has a 0.09 north/south grade under the wings.
        frame.gazebo_spawn_elevation_at_world = (
            lambda _x, y: 100.0 + 0.09 * y)
        runway = frame.fixedwing_runway_metrics(0.0, 0.0)
        self.assertGreater(runway["local_surface_slope_rad"], 0.05)
        with self.assertRaisesRegex(ValueError, "Gazebo 实际地表坡度"):
            frame.validate_fixedwing_site(0.0, 0.0)

    def test_later_vehicle_examples_respect_endpoint_angle_budget(self):
        for item in self.frame.vehicle_examples()[3:]:
            start, end = item["route"]["points"]
            horizontal = math.hypot(end["x"] - start["x"],
                                    end["y"] - start["y"])
            angle = abs(math.atan2(end["z"] - start["z"], horizontal))
            self.assertLessEqual(angle, 0.15, item["uav_name"])

    def test_coordinate_selection_samples_dem_and_adds_clearance(self):
        longitude, latitude = self.frame.world_to_geo(600.0, -1100.0)
        with patch("swarm_app.main.terrain_frame", return_value=self.frame):
            converted = swarm_to_world(GeoCoordinate(
                longitude=longitude, latitude=latitude, clearance_m=80.0))
            sampled = swarm_world_elevation(WorldElevationCoordinate(
                x=converted["x"], y=converted["y"], clearance_m=80.0))
        self.assertAlmostEqual(600.0, converted["x"], delta=0.002)
        self.assertAlmostEqual(-1100.0, converted["y"], delta=0.002)
        self.assertAlmostEqual(
            80.0,
            converted["flight_z_m"] - converted["ground_elevation_m"])
        self.assertAlmostEqual(converted["ground_elevation_m"],
                               sampled["ground_elevation_m"], places=5)

    def test_custom_spawn_origin_is_resampled_from_canonical_dem(self):
        x, y = 600.0, -2020.0
        ground = self.frame.elevation_at_world(x, y)
        with patch("swarm_app.main.terrain_frame", return_value=self.frame):
            example = _spawn_example(FleetSpawnRequest(
                uav_name="uav1", use_mock=True,
                world_origin={"x": x, "y": y, "z": ground}))
        self.assertEqual(x, example["world_origin"]["x"])
        self.assertEqual(y, example["world_origin"]["y"])
        self.assertAlmostEqual(ground, example["world_origin"]["z"])
        self.assertEqual(x, example["gazebo_spawn"]["x"])
        self.assertEqual(y, example["gazebo_spawn"]["y"])
        self.assertAlmostEqual(
            self.frame.gazebo_spawn_elevation_at_world(x, y),
            example["gazebo_spawn"]["z"])
        self.assertNotEqual(ground, example["gazebo_spawn"]["z"])
        self.assertAlmostEqual(
            ground + 80.0, example["route"]["points"][0]["z"])


class MockFleetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.examples = DavosdorfWorldFrame().vehicle_examples()

    def test_operation_request_rejects_duplicate_names(self):
        with self.assertRaises(ValidationError):
            FleetOperationRequest(uav_names=["uav1", "uav1"])

    def test_spawn_requires_an_explicit_map_origin(self):
        with self.assertRaises(ValidationError):
            FleetSpawnRequest(uav_name="uav1", use_mock=True)

    def test_add_takeoff_land_and_safe_remove_are_separate(self):
        gateway = MockSwarmGateway()
        for example in self.examples[:2]:
            gateway.spawn_vehicle(example)
        self.assertEqual(2, len(gateway.fleet_snapshot()["vehicles"]))
        request = SwarmDispatchRequest(
            use_mock=True,
            routes=[route("uav1", 6201, -200.0),
                    route("uav2", 6202, 200.0)])
        gateway.dispatch(request)
        gateway.takeoff(["uav1", "uav2"], 80.0)
        with self.assertRaises(ValueError):
            gateway.remove_vehicle("uav1")
        gateway.land(["uav1", "uav2"])
        gateway.remove_vehicle("uav1")
        self.assertEqual(["uav2"], [
            item["uav_name"] for item in gateway.fleet_snapshot()["vehicles"]])

    def test_spawn_adds_a_real_example_to_mock_inventory(self):
        gateway = MockSwarmGateway()
        self.assertEqual([], gateway.fleet_snapshot()["vehicles"])
        response = gateway.spawn_vehicle(self.examples[0])
        self.assertTrue(response["accepted"])
        self.assertEqual(1, len(gateway.fleet_snapshot()["vehicles"]))

    def test_spatial_only_crossing_is_a_non_blocking_mock_warning(self):
        gateway = MockSwarmGateway()
        request = SwarmDispatchRequest(use_mock=True, routes=[
            VehicleRoute(
                uav_name="uav1", goal_id=6201, nominal_speed_mps=10.0,
                points=[WorldPoint(x=-100.0, y=0.0, z=50.0),
                        WorldPoint(x=100.0, y=0.0, z=50.0)]),
            VehicleRoute(
                uav_name="uav2", goal_id=6202, nominal_speed_mps=10.0,
                points=[WorldPoint(x=0.0, y=-300.0, z=50.0),
                        WorldPoint(x=0.0, y=100.0, z=50.0)]),
        ])
        result = gateway.dispatch(request)
        self.assertTrue(result["status"]["conflict_free"])
        self.assertEqual(1, len(result["status"]["conflicts"]))
        self.assertEqual(1, result["status"]["conflicts"][0]["conflict_type"])


class RosbridgeCacheTest(unittest.TestCase):
    def test_null_optional_sensor_does_not_break_fleet_snapshot(self):
        gateway = RosbridgeSwarmGateway()
        gateway._ensure_connected = lambda: None
        gateway._fleet = {"capacity": 1, "vehicles": [{
            "uav_name": "uav1",
            "lifecycle": 1,
            "connected": True,
            "world_origin": {"x": 100.0, "y": 200.0, "z": 300.0},
        }]}
        gateway._control_states = {"uav1": {
            "position_odom": {"x": 1.0, "y": 2.0, "z": 3.0},
            "airspeed": None,
            "groundspeed": None,
        }}
        gateway._control_received_at = {"uav1": time.monotonic()}
        live = gateway.fleet_snapshot()["runtime"]["uav1"]
        self.assertEqual(
            {"x": 101.0, "y": 202.0, "z": 303.0},
            live["world_position"])
        self.assertEqual(0.0, live["airspeed_mps"])
        self.assertEqual(0.0, live["groundspeed_mps"])

    def test_disconnected_vehicle_does_not_expose_cached_position_as_live(self):
        gateway = RosbridgeSwarmGateway()
        gateway._ensure_connected = lambda: None
        gateway._fleet = {"capacity": 1, "vehicles": [{
            "uav_name": "uav1",
            "lifecycle": 3,
            "connected": False,
            "world_origin": {"x": 100.0, "y": 200.0, "z": 300.0},
        }]}
        gateway._control_states = {"uav1": {
            "position_odom": {"x": 1.0, "y": 2.0, "z": 3.0},
            "state_valid": True,
        }}
        gateway._control_received_at = {"uav1": time.monotonic()}
        live = gateway.fleet_snapshot()["runtime"]["uav1"]
        self.assertFalse(live["telemetry_fresh"])
        self.assertIsNone(live["world_position"])

    def test_takeoff_precheck_rejects_stopped_clock_and_inverted_aircraft(self):
        gateway = RosbridgeSwarmGateway()
        now = time.monotonic()
        gateway._clock_value = 10.0
        gateway._clock_received_at = now
        gateway._clock_advanced_at = now - 6.0
        with self.assertRaisesRegex(RuntimeError, "仿真时钟"):
            gateway._takeoff_precheck(["uav1"])

        gateway._clock_advanced_at = now
        gateway._fleet = {"vehicles": [{
            "uav_name": "uav1", "lifecycle": 1, "connected": True,
            "armed": False, "on_ground": True,
        }]}
        gateway._control_states = {"uav1": {
            "state_valid": True,
            "orientation_odom_body": {"x": 1.0, "y": 0.0,
                                      "z": 0.0, "w": 0.0},
        }}
        gateway._control_received_at = {"uav1": now}
        gateway._last_dispatch = {"uav1": 7201}
        gateway._status = {
            "analysis_ready": True,
            "conflict_free": True,
            "vehicles": [{
                "uav_name": "uav1",
                "planner_status": {"goal_id": 7201},
            }],
        }
        with self.assertRaisesRegex(RuntimeError, "可能已侧翻"):
            gateway._takeoff_precheck(["uav1"])

    def test_landing_monitor_reports_completion_and_inverted_failure(self):
        gateway = RosbridgeSwarmGateway()
        gateway._release_generation = 3
        gateway._fleet = {"vehicles": [{
            "uav_name": "uav1", "lifecycle": 1, "connected": True,
            "armed": False, "on_ground": True,
        }]}
        gateway._wait_for_landing(["uav1"], 3)
        self.assertEqual("landed", gateway._takeoff_state["state"])

        gateway._release_generation = 4
        gateway._fleet["vehicles"][0].update(armed=True, on_ground=False)
        gateway._control_states = {"uav1": {
            "state_valid": False,
            "orientation_odom_body": {
                "x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
        }}
        gateway._control_received_at = {"uav1": 4.0}
        gateway._clock_value = 10.0
        gateway._clock_received_at = 4.0
        gateway._clock_advanced_at = 4.0
        with patch("swarm_app.gateway.time.monotonic",
                   side_effect=[0.0, 1.0, 1.0, 4.0, 4.0]), patch(
                       "swarm_app.gateway.time.sleep"):
            gateway._wait_for_landing(["uav1"], 4)
        self.assertEqual("failed", gateway._takeoff_state["state"])
        self.assertIn("可能已翻覆", gateway._takeoff_state["detail"])

    def test_unobserved_runtime_terrain_is_not_a_mismatch(self):
        frame = MagicMock(terrain_id="terrain-current")
        with patch("swarm_app.main._frame_or_503", return_value=frame), patch(
                "swarm_app.main.ROS_GATEWAY.capabilities",
                return_value={"terrain_id": None}):
            runtime = swarm_capabilities()["runtime"]
        self.assertIsNone(runtime["terrain_match"])

    def test_observed_runtime_terrain_is_compared(self):
        frame = MagicMock(terrain_id="terrain-current")
        with patch("swarm_app.main._frame_or_503", return_value=frame), patch(
                "swarm_app.main.ROS_GATEWAY.capabilities",
                return_value={"terrain_id": "terrain-current"}):
            matching = swarm_capabilities()["runtime"]
        with patch("swarm_app.main._frame_or_503", return_value=frame), patch(
                "swarm_app.main.ROS_GATEWAY.capabilities",
                return_value={"terrain_id": "terrain-old"}):
            stale = swarm_capabilities()["runtime"]
        self.assertTrue(matching["terrain_match"])
        self.assertFalse(stale["terrain_match"])

    def test_capabilities_probe_does_not_connect_or_dispatch(self):
        gateway = RosbridgeSwarmGateway()
        connection = MagicMock()
        connection.__enter__.return_value = connection
        with patch("swarm_app.gateway.socket.create_connection",
                   return_value=connection) as create_connection:
            result = gateway.capabilities()
        create_connection.assert_called_once_with(
            (gateway.host, gateway.port), timeout=0.2)
        self.assertTrue(result["rosbridge_reachable"])
        self.assertFalse(result["ros_connected"])
        self.assertFalse(result["execution_available"])
        self.assertEqual(0, result["aircraft_count"])

    def test_latched_path_from_previous_goal_is_ignored(self):
        gateway = RosbridgeSwarmGateway()
        gateway._ensure_connected = lambda: None
        gateway._last_dispatch = {"uav1": 7201}
        stale = build_path_message(route("uav1", 6201), timestamp=100.0)
        current = build_path_message(route("uav1", 7201), timestamp=101.0)

        gateway._path_callback("uav1", stale)
        self.assertEqual({}, gateway.snapshot()["paths"])
        gateway._path_callback("uav1", current)
        self.assertEqual(7201, gateway.snapshot()["paths"]["uav1"]
                         ["poses"][0]["header"]["seq"])

    def test_status_from_previous_batch_is_ignored(self):
        gateway = RosbridgeSwarmGateway()
        gateway._ensure_connected = lambda: None
        gateway._last_dispatch = {"uav1": 7201, "uav2": 7202}

        def status(first_goal, second_goal):
            return {"vehicles": [
                {"uav_name": "uav1",
                 "planner_status": {"goal_id": first_goal}},
                {"uav_name": "uav2",
                 "planner_status": {"goal_id": second_goal}},
            ]}

        gateway._status_callback(status(6201, 6202))
        self.assertIsNone(gateway.snapshot()["status"])
        gateway._status_callback(status(7201, 7202))
        self.assertEqual(7201, gateway.snapshot()["status"]["vehicles"][0]
                         ["planner_status"]["goal_id"])

    def test_restart_requires_every_vehicle_safe_on_ground(self):
        gateway = RosbridgeSwarmGateway()
        gateway._ensure_connected = lambda: None
        released = []
        gateway._publish_release = released.append
        gateway._fleet = {"vehicles": [{
            "uav_name": "uav1", "lifecycle": 1, "connected": True,
            "armed": False, "on_ground": True,
        }]}
        self.assertTrue(gateway.prepare_restart()["accepted"])
        self.assertEqual([False], released)

        gateway._fleet["vehicles"][0]["armed"] = True
        with self.assertRaises(RuntimeError):
            gateway.prepare_restart()


class MockApiTest(unittest.TestCase):
    def test_health_exposes_formal_ros_contract(self):
        health = swarm_health()
        self.assertEqual("nav_msgs/Path", health["ros_contract"]["input_type"])
        self.assertEqual("/<uav>/planning/task_path",
                         health["ros_contract"]["input_topic"])
        self.assertTrue(health["canonical_dem_match"])

    def test_mock_dispatch_uses_m4_conflict_engine_and_is_labelled(self):
        request = SwarmDispatchRequest(
            use_mock=True,
            routes=[route(), route("uav2", 6202)])
        result = swarm_dispatch(request)
        self.assertEqual("mock", result["mode"])
        self.assertEqual(2, len(result["dispatched"]))
        self.assertEqual("/uav1/planning/task_path",
                         result["dispatched"][0]["topic"])
        self.assertTrue(result["status"]["analysis_ready"])
        self.assertFalse(result["status"]["conflict_free"])
        self.assertIn("not_real_terrain_results", result["status"]["detail"])
        self.assertEqual(2, result["status"]["conflicts"][0]["conflict_type"])
        self.assertEqual({"uav1", "uav2"}, set(result["display_paths"]))
        self.assertEqual(result["display_paths"],
                         swarm_snapshot(use_mock=True)["display_paths"])

    def test_non_world_result_is_not_projected_for_display(self):
        message = build_path_message(route(), timestamp=100.25)
        message["header"]["frame_id"] = "map"
        result = _decorate_snapshot({"paths": {"uav1": message}})
        self.assertEqual([], result["display_paths"]["uav1"]["points"])
        self.assertIn("frame_id=world",
                      result["display_paths"]["uav1"]["error"])


if __name__ == "__main__":
    unittest.main()
