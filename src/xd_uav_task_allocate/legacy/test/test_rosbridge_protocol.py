#!/usr/bin/env python3

import base64
import json
import queue
import unittest

from xd_uav_sead.comms.communication_info import Message_ID, packet_processing
from xd_uav_sead.comms.rosbridge import SeadRosBridge


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


def _bridge(uav_id=1):
    bridge = object.__new__(SeadRosBridge)
    bridge.uav_id = uav_id
    bridge.gcs_address = 0
    bridge._pub_telemetry = _Publisher()
    bridge._pub_telemetry_raw = _Publisher()
    bridge._pub_u2u = _Publisher()
    bridge.publish_json_telemetry = True
    bridge._rx_queue = queue.Queue()
    return bridge


class RosBridgeProtocolTest(unittest.TestCase):
    def setUp(self):
        self.bridge = _bridge(1)
        self.protocol = packet_processing(1)

    def decode(self, msg_id, info):
        raw = self.bridge._encode(msg_id, info)
        return self.protocol.unpack_packet(raw)

    def test_basic_gcs_commands_round_trip(self):
        cases = [
            (1, {"name": "GUIDED"}, Message_ID.Mode_Change),
            (2, {"armed": True}, Message_ID.Arm),
            (3, {"alt": 120}, Message_ID.Takeoff),
            (5, {"method": 0, "radius": 50, "target": [100, 50, 80]}, Message_ID.Waypoints),
            (6, {"freq": 2.0}, Message_ID.Comm_u2gFreq),
            (9, {"origin_id": 1}, Message_ID.Origin_Correction),
            (19, {"point": [150, 75], "task_type": 0}, Message_ID.Task_Insert),
        ]
        for msg_id, info, expected in cases:
            with self.subTest(msg_id=msg_id):
                decoded, _ = self.decode(msg_id, info)
                self.assertEqual(decoded, expected)

    def test_sead_mission_round_trip(self):
        decoded, info = self.decode(
            18,
            {
                "targets": [[100, 0], [200, 50]],
                "unknown_targets": [[300, 80]],
                "uav_type": 2,
                "velocity": 20,
                "Rmin": 35,
                "waypoint_radius": 50,
                "init_pos": [0, 0, 0],
                "end": [400, 0, 0],
            },
        )
        self.assertEqual(decoded, Message_ID.SEAD_mission)
        self.assertEqual(info[0], [[100.0, 0.0], [200.0, 50.0]])
        self.assertEqual(info[1], [[300.0, 80.0]])

    def test_swarm_nested_params_round_trip(self):
        decoded, info = self.decode(
            24,
            {
                "command": "formation_config",
                "params": {
                    "shape": "TRAIL",
                    "enable": 1,
                    "leader_id": 2,
                    "spacing": 333.0,
                    "standoff": 5000.0,
                    "safe_sep": 140.0,
                    "alt_step": 20.0,
                },
            },
        )
        self.assertEqual(decoded, Message_ID.Swarm_Command)
        self.assertEqual(info["params"]["shape"], 4)
        self.assertEqual(info["params"]["leader_id"], 2)
        self.assertAlmostEqual(info["params"]["spacing"], 333.0)

    def test_airspace_zone_round_trip(self):
        decoded, info = self.decode(
            21,
            {
                "zone_id": 7,
                "enabled": True,
                "zone_type": 0,
                "level2d": 1,
                "levelH": 0,
                "minAlt": 10.0,
                "maxAlt": 200.0,
                "vertices": [[0, 0], [100, 0], [100, 100], [0, 100]],
            },
        )
        self.assertEqual(decoded, Message_ID.Airspace_ZoneFrag)
        self.assertTrue(info["ok"])
        self.assertEqual(info["zone_id"], 7)
        self.assertEqual(len(info["zonedef"]["vertices"]), 4)

    def test_u2u_filtering(self):
        blob = base64.b64encode(b"payload").decode("ascii")
        own = json.dumps({"src": 1, "dst": 2, "broadcast": False, "blob": blob})
        other = json.dumps({"src": 2, "dst": 1, "broadcast": False, "blob": blob})
        wrong_dst = json.dumps({"src": 2, "dst": 3, "broadcast": False, "blob": blob})
        broadcast = json.dumps({"src": 2, "dst": None, "broadcast": True, "blob": blob})
        self.assertIsNone(self.bridge._parse_u2u(own))
        self.assertEqual(self.bridge._parse_u2u(other), b"payload")
        self.assertIsNone(self.bridge._parse_u2u(wrong_dst))
        self.assertEqual(self.bridge._parse_u2u(broadcast), b"payload")

    def test_send_routes_gcs_and_uav_separately(self):
        self.bridge.send_data_async(0, b"gcs")
        self.bridge.send_data_async(2, b"peer")
        self.assertEqual(len(self.bridge._pub_telemetry.messages), 1)
        self.assertEqual(self.bridge._pub_telemetry_raw.messages, [[103, 99, 115]])
        self.assertEqual(len(self.bridge._pub_u2u.messages), 1)

    def test_raw_ros_command_is_not_reencoded(self):
        message = type("RawMessage", (), {"data": [18, 1, 0, 255]})()
        self.bridge._on_raw_command(message)
        self.assertEqual(self.bridge.read_data().data, b"\x12\x01\x00\xff")

    def test_simple_strike_assignment_and_ack_round_trip(self):
        assignment = {
            1: {"target_id": 2, "point": [100.125, -20.5]},
            2: {"target_id": 1, "point": [200.0, 30.25]},
        }
        decoded, info = self.protocol.unpack_packet(
            self.protocol.pack_simple_strike_assignment(1, 2, assignment)
        )
        self.assertEqual(decoded, Message_ID.SimpleStrike_Assignment)
        self.assertEqual(info["leader_id"], 1)
        self.assertEqual(info["assignment_map"], assignment)

        decoded, info = self.protocol.unpack_packet(
            self.protocol.pack_simple_strike_assignment_ack(1, 2)
        )
        self.assertEqual(decoded, Message_ID.SimpleStrike_Assignment_Ack)
        self.assertEqual(info["sender_uav_id"], 1)
        self.assertEqual(info["target_count"], 2)

    def test_simple_strike_release_and_path_status_round_trip(self):
        decoded, info = self.protocol.unpack_packet(
            self.protocol.pack_simple_strike_release_time(1, 3, 1234.5)
        )
        self.assertEqual(decoded, Message_ID.SimpleStrike_ReleaseTime)
        self.assertAlmostEqual(info["release_time"], 1234.5)

        status = {
            "uav_id": 1,
            "target_id": 2,
            "path_ready": True,
            "path_length": 450.0,
            "remaining_path_length": 300.0,
            "sync_remaining_path_length": 280.0,
            "arrival_radius_for_sync": 25.0,
            "dist_to_target": 275.0,
            "target_sync_remaining": 30.0,
            "eta": 15.0,
            "actual_groundspeed": 18.0,
            "predicted_arrival_time": 1235.0,
            "predicted_arrival_error": 0.5,
            "target_closure_speed": 17.0,
            "sync_progress_speed": 16.0,
            "target_closure_speed_ema": 16.5,
            "sync_progress_speed_ema": 15.5,
            "predicted_arrival_used_for_control": True,
            "stamp": 1220.0,
            "control_path_source": "full_path",
        }
        decoded, info = self.protocol.unpack_packet(
            self.protocol.pack_simple_strike_path_status(1, 3, status)
        )
        self.assertEqual(decoded, Message_ID.SimpleStrike_PathStatus)
        self.assertEqual(info["status"], status)


if __name__ == "__main__":
    unittest.main()
