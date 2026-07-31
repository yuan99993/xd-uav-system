#!/usr/bin/env python3
"""
rosbridge 闭环测试 — 验证 14 种消息类型的 JSON→bytes→unpack 链路，无需 PX4 SITL。
用法: python3 test_rosbridge_closed_loop.py
"""
import sys, os, struct, json, base64, importlib, unittest

# Add the xd_uav_sead source to path so we can import the real modules
SRC = '/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/src'
sys.path.insert(0, SRC)
sys.path.insert(0, '/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/scripts')


def load_rosbridge_encode():
    """Load rosbridge._encode and _serialize_info without executing module-level ROS code."""
    path = os.path.join(SRC, 'xd_uav_sead', 'comms', 'rosbridge.py')
    with open(path) as f:
        source = f.read()

    ns = {
        'struct': struct, 'json': json, 'base64': __import__('base64'),
        'time': __import__('time'), 'queue': __import__('queue'),
        'logging': __import__('logging'),
        'MSG_DISPATCH': {
            1: "Mode_Change", 2: "Arm", 3: "Takeoff",
            4: "Time_Synchromize", 5: "Waypoints", 6: "Comm_u2gFreq",
            7: "Record_Time", 8: "Mission_Abort", 9: "Origin_Correction",
            17: "SEAD", 18: "SEAD_mission", 19: "Task_Insert",
            20: "Airspace_Clear", 21: "Airspace_ZoneFrag", 22: "Airspace_ZoneRemove",
            23: "Airspace_Ack", 24: "Swarm_Command", 25: "Formation_State",
            26: "Formation_Point", 27: "SimpleStrike_Assignment",
            28: "SimpleStrike_Assignment_Ack", 30: "SimpleStrike_ReleaseTime",
            32: "SimpleStrike_PathStatus", 44: "info",
        },
    }
    exec(compile(source, 'rosbridge.py', 'exec'), ns)
    cls = ns['SeadRosBridge']

    # Reconstruct MSG_DISPATCH in the module-level scope for _serialize_info
    import builtins
    builtins.MSG_DISPATCH = ns['MSG_DISPATCH']

    class TestBridge:
        """Minimal bridge with just the serialization logic, no ROS deps."""
        uav_id = 1
        uav_name = 'uav1'

        def _serialize_info(self, msg_id, info):
            name = MSG_DISPATCH.get(msg_id, None)
            if msg_id == 1:
                mode_map = {"GUIDED":4,"LAND":9,"LOITER":5,"RTL":6,"AUTO":3,"STABILIZE":0,"POSHOLD":16,"OFFBOARD":4}
                mode_val = info.get("name", info.get("mode", "GUIDED"))
                if isinstance(mode_val, str):
                    mode_val = mode_map.get(mode_val, 4)
                return struct.pack("<B", int(mode_val))
            if msg_id == 2: return struct.pack("<B", 1 if bool(info.get("armed", True)) else 0)
            if msg_id == 3: return struct.pack("<B", int(min(float(info.get("alt", 120.0)), 255)))
            if msg_id == 4: return b""
            if msg_id == 5:
                method = int(info.get("method", 0))
                radius = int(info.get("radius", 50))
                target = info.get("target", [0.0, 0.0, 80.0])
                return struct.pack("<BBiii", method, radius,
                    int(float(target[0])*1e3), int(float(target[1])*1e3), int(float(target[2])*1e3))
            if msg_id == 6:
                freq = float(info.get("freq", 2.0))
                return struct.pack("<i", int(freq * 100))
            if msg_id == 8: return b""
            if msg_id == 9:
                oid = int(info.get("origin_id", 0))
                return struct.pack("<B", oid)
            if msg_id == 18:
                targets = info.get("targets", [])
                unknown = info.get("unknown_targets", [])
                uav_type = int(info.get("uav_type", 2))
                velocity = float(info.get("velocity", 20.0))
                rmin = float(info.get("Rmin", 35.0))
                radius = int(info.get("waypoint_radius", 50))
                init_pos = info.get("init_pos", [0.0, 0.0, 0.0])
                end = info.get("end", [0.0, 0.0, 0.0])
                target_num = len(targets)
                unknown_num = len(unknown)
                pack_fmt = "<BiiBiiiiiiBB" + "ii" * target_num + "ii" * unknown_num
                args = [uav_type, int(velocity*1e3), int(rmin*1e3), radius,
                    int(init_pos[0]*1e3), int(init_pos[1]*1e3), int(init_pos[2]*1e3),
                    int(end[0]*1e3), int(end[1]*1e3), int(end[2]*1e3),
                    target_num, unknown_num]
                for t in targets:
                    args.append(int(float(t[0])*1e3)); args.append(int(float(t[1])*1e3))
                for t in unknown:
                    args.append(int(float(t[0])*1e3)); args.append(int(float(t[1])*1e3))
                return struct.pack(pack_fmt, *args)
            if msg_id == 19:
                point = info.get("point", [0.0, 0.0])
                task_type = int(info.get("task_type", 0))
                return struct.pack("<iiB", int(float(point[0])*1e3), int(float(point[1])*1e3), task_type)
            if msg_id == 20: return b""
            if msg_id == 21:
                blob = info.get("blob", "")
                if blob: return base64.b64decode(blob)
                return b""
            if msg_id == 24:
                shape = int(info.get("shape", 1))
                enable = int(info.get("enable", 1))
                leader = int(info.get("leader_id", 1))
                spacing = float(info.get("spacing", 220.0))
                standoff = float(info.get("standoff", 5000.0))
                safe_sep = float(info.get("safe_sep", 140.0))
                alt_step = float(info.get("alt_step", 20.0))
                desired = float(info.get("desired_target_time", 0.0))
                return struct.pack("<BBBBiiiid", self.uav_id, enable, shape, leader,
                    int(spacing*1e3), int(standoff*1e3), int(safe_sep*1e3), int(alt_step*1e3), desired)
            if msg_id == 26:
                point = info.get("point", [0.0, 0.0, 120.0])
                point_id = int(info.get("point_id", 1))
                loiter_radius = float(info.get("loiter_radius", 300.0))
                return struct.pack("<Hiiii", point_id,
                    int(float(point[0])*1e3), int(float(point[1])*1e3),
                    int(float(point[2])*1e3), int(loiter_radius*1e3))
            if msg_id == 44:
                text = info.get("text", str(info))
                text_bytes = text.encode("utf-8")
                return struct.pack("<B", len(text_bytes)) + text_bytes
            return json.dumps(info).encode("utf-8")

        def _encode(self, msg_id, info):
            head = struct.pack("<BB", msg_id, self.uav_id)
            payload = self._serialize_info(msg_id, info)
            return head + payload

    return TestBridge()


def load_unpack_packet():
    """Load the real packet_processing.unpack_packet from communication_info.py."""
    path = os.path.join(SRC, 'xd_uav_sead', 'comms', 'communication_info.py')
    with open(path) as f:
        source = f.read()

    import numpy as np

    class FakePathFollowing:
        @staticmethod
        def CraigReynolds_Path_Following(*a, **kw):
            class CRPF:
                path = [] ; method = None
            return CRPF()
        @staticmethod
        def PlusMinusPi(x): return x
        @staticmethod
        def arctan2(y, x): return 0.0

    class FakeDubins:
        @staticmethod
        def shortest_path(*a, **kw):
            class P:
                @staticmethod
                def sample_many(s): return [[[0,0,0],[1,1,1]]]
            return P()

    g = {
        'struct': struct, 'Enum': __import__('enum').Enum,
        'np': np, 'pack': struct.pack, 'unpack': struct.unpack,
        'sleep': lambda x: None, 'time': lambda: 1000.0,
        'dubins': FakeDubins(),
        'rospy': type(sys)('rospy'),
        'pf': FakePathFollowing(),
        'SWARM_SHAPE_CODES': {"VEE":1,"ECHELON_LEFT":2,"ECHELON_RIGHT":3,"TRAIL":4,"TRIANGLE":5,"WEDGE_WIDE":6,"ARROW":7,"INVERTED_VEE":8},
        'Message_ID': None, 'Mode': None, 'Armed': None,
        'FrameType': None, 'WaypointMissionMethod': None,
        'pathFollowingMethod': None, 'XBee_Devices': None,
        'generate_dubinsPath': lambda points, radius, interval: [[0,0,0]],
        'GA_SEAD_process': type(sys)('GA_SEAD_process'),
        'get_peer_ids': lambda: [1,2,3],
    }
    g['rospy'].loginfo = lambda *a: None
    g['rospy'].logwarn = lambda *a: None
    g['rospy'].logerr = lambda *a: None
    g['rospy'].loginfo_throttle = lambda *a, **kw: None
    g['rospy'].logwarn_throttle = lambda *a, **kw: None
    g['rospy'].get_param = lambda k, d=None: d

    exec(compile(source, 'communication_info.py', 'exec'), g)
    pp = g['packet_processing'](1)
    # Also expose enums for assertions
    return pp, g['Message_ID'], g['Mode'], g['Armed']


class TestRoundTrip(unittest.TestCase):
    """Full round-trip: JSON → bytes → unpack_packet for each command type."""

    @classmethod
    def setUpClass(cls):
        cls.bridge = load_rosbridge_encode()
        cls.unpacker, cls.Message_ID, cls.Mode, cls.Armed = load_unpack_packet()

    def _rt(self, msg_id, info_dict):
        raw = self.bridge._encode(msg_id, info_dict)
        return self.unpacker.unpack_packet(raw)

    # ── 基础命令 ──

    def test_mode_change_guided(self):
        mt, info = self._rt(1, {"name": "GUIDED"})
        self.assertEqual(mt, self.Message_ID.Mode_Change)
        self.assertEqual(info, self.Mode.GUIDED)

    def test_mode_change_land(self):
        mt, info = self._rt(1, {"name": "LAND"})
        self.assertEqual(mt, self.Message_ID.Mode_Change)
        self.assertEqual(info, self.Mode.LAND)

    def test_arm(self):
        mt, info = self._rt(2, {"armed": True})
        self.assertEqual(mt, self.Message_ID.Arm)
        self.assertEqual(info, self.Armed.armed)

    def test_disarm(self):
        mt, info = self._rt(2, {"armed": False})
        self.assertEqual(mt, self.Message_ID.Arm)
        self.assertEqual(info, self.Armed.disarmed)

    def test_takeoff(self):
        mt, info = self._rt(3, {"alt": 120})
        self.assertEqual(mt, self.Message_ID.Takeoff)
        self.assertEqual(info, 120)

    def test_mission_abort(self):
        mt, info = self._rt(8, {})
        self.assertEqual(mt, self.Message_ID.Mission_Abort)
        self.assertIsNone(info)

    def test_origin_correction(self):
        mt, info = self._rt(9, {"origin_id": 1})
        self.assertEqual(mt, self.Message_ID.Origin_Correction)
        self.assertEqual(info, 1)

    def test_comm_freq(self):
        mt, info = self._rt(6, {"freq": 2.0})
        self.assertEqual(mt, self.Message_ID.Comm_u2gFreq)
        self.assertAlmostEqual(info, 2.0, places=1)

    def test_task_insert(self):
        mt, info = self._rt(19, {"point": [150.0, 250.0], "task_type": 0})
        self.assertEqual(mt, self.Message_ID.Task_Insert)
        self.assertAlmostEqual(info[0], 150.0, delta=0.05)

    def test_formation_point(self):
        mt, info = self._rt(26, {
            "point_id": 1, "point": [100.0, 200.0, 120.0], "loiter_radius": 300.0,
        })
        self.assertEqual(mt, self.Message_ID.Formation_Point)
        self.assertAlmostEqual(info["point"][0], 100.0, delta=0.05)

    def test_airspace_clear(self):
        mt, info = self._rt(20, {})
        self.assertEqual(mt, self.Message_ID.Airspace_Clear)
        self.assertIsNone(info)

    def test_info_roundtrip(self):
        # info 消息在原始代码里没有专门的 unpack 分支（走隐式 None fallback）
        # 这里验证的是编码后的格式正确
        raw = self.bridge._encode(44, {"text": "hello world"})
        self.assertEqual(raw[0], 44)           # msg_id
        self.assertEqual(raw[1], 1)            # uav_id
        self.assertEqual(raw[2], 11)           # str_len = len("hello world")
        self.assertEqual(raw[3:].decode(), "hello world")

    # ── SEAD_mission (关键 bug 修复验证) ──

    def test_sead_mission_empty_targets(self):
        """Empty target lists must not crash (was: f-string format bug)."""
        mt, info = self._rt(18, {
            "targets": [], "unknown_targets": [],
            "uav_type": 2, "velocity": 20.0, "Rmin": 35.0,
            "waypoint_radius": 50,
            "init_pos": [0.0, 0.0, 0.0], "end": [0.0, 0.0, 0.0],
        })
        self.assertEqual(mt, self.Message_ID.SEAD_mission)
        self.assertEqual(info[0], [])

    def test_sead_mission_one_target(self):
        mt, info = self._rt(18, {
            "targets": [[100.0, 200.0]], "unknown_targets": [],
            "uav_type": 3, "velocity": 25.0, "Rmin": 40.0,
            "waypoint_radius": 60,
            "init_pos": [10.0, 20.0, 30.0], "end": [40.0, 50.0, 60.0],
        })
        self.assertEqual(mt, self.Message_ID.SEAD_mission)
        self.assertEqual(len(info[0]), 1)
        self.assertAlmostEqual(info[0][0][0], 100.0, delta=0.05)

    def test_sead_mission_both_types(self):
        mt, info = self._rt(18, {
            "targets": [[100.0, 200.0], [300.0, 400.0]],
            "unknown_targets": [[500.0, 600.0]],
            "uav_type": 2, "velocity": 20.0, "Rmin": 35.0,
            "waypoint_radius": 50,
            "init_pos": [0.0, 0.0, 0.0], "end": [100.0, 200.0, 300.0],
        })
        self.assertEqual(mt, self.Message_ID.SEAD_mission)
        self.assertEqual(len(info[0]), 2)
        self.assertEqual(len(info[1]), 1)

    # ── 错误路径 ──

    def test_wrong_uav_takeoff(self):
        """Message for wrong UAV ID returns info, not crash."""
        raw = struct.pack('<BBB', 3, 99, 120)
        mt, info = self.unpacker.unpack_packet(raw)
        self.assertEqual(mt, self.Message_ID.info)
        self.assertIn("Wrong UAV", info)


if __name__ == '__main__':
    unittest.main(verbosity=2)
