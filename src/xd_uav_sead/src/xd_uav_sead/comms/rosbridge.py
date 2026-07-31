# SeadRosBridge — XBee → ROS 替换层
#
# 保留原始 XBee API 签名，内部用 ROS 话题实现。
# 这样 onboard node 里 68 个 XBee 调用可以直接替换，不用改上层逻辑。

import queue
import json
import time
import struct

import rospy
from std_msgs.msg import String

U2U_BUS_TOPIC = "/sead/u2u"

# === 消息类型分发 ===
# 跟 communication_info.Message_ID 保持同步
MSG_DISPATCH = {
    1: "Mode_Change",
    2: "Arm",
    3: "Takeoff",
    4: "Time_Synchromize",
    5: "Waypoints",
    6: "Comm_u2gFreq",
    7: "Record_Time",
    8: "Mission_Abort",
    9: "Origin_Correction",
    17: "SEAD",
    18: "SEAD_mission",
    19: "Task_Insert",
    20: "Airspace_Clear",
    21: "Airspace_ZoneFrag",
    22: "Airspace_ZoneRemove",
    23: "Airspace_Ack",
    24: "Swarm_Command",
    25: "Formation_State",
    26: "Formation_Point",
    27: "SimpleStrike_Assignment",
    28: "SimpleStrike_Assignment_Ack",
    30: "SimpleStrike_ReleaseTime",
    32: "SimpleStrike_PathStatus",
    44: "info",
}


class FakePacket:
    """模仿 digi.xbee 包的 data 属性。"""

    def __init__(self, data: bytes):
        self.data = data


class SeadRosBridge:
    """
    XBee 硬件通信的 ROS 替代。
    api 签名尽量保留原始 XBee 风格，使迁移成本最小。

    XBee               →    SeadRosBridge
    ──────────────────────────────────────────
    xbee.read_data()     →    bridge.read_data()
    xbee.send_data_async →    bridge.send_data_async()
    xbee.send_data_broadcast → bridge.send_data_broadcast()
    RemoteDigiMeshDevice  →    int(uav_id)
    XBee64BitAddress      →    不需要
    find_xbee_by_id()     →    rospy.get_param("~uav_id")
    """

    def __init__(self, uav_name="uav0", uav_id=0):
        self.uav_name = uav_name
        self.uav_id = uav_id
        self.ns = f"/{uav_name}/sead"

        # 接收队列 — 仿真 xbee.read_data() 的 drain 循环
        self._rx_queue = queue.Queue()

        # 订阅 GCS 命令
        self._sub_cmd = rospy.Subscriber(
            f"{self.ns}/command", String, self._on_command, queue_size=20
        )

        # 订阅 U2U 广播（多机 GA 协同）
        self._sub_u2u = rospy.Subscriber(
            U2U_BUS_TOPIC, String, self._on_u2u, queue_size=50
        )

        # 发布遥测 → GCS
        self._pub_telemetry = rospy.Publisher(
            f"{self.ns}/telemetry", String, queue_size=20
        )

        # 所有 UAV 共用一条总线；消息 envelope 中携带 src/dst/broadcast。
        self._pub_u2u = rospy.Publisher(
            U2U_BUS_TOPIC, String, queue_size=50
        )

        # GCS 和 UAV 的"地址"现在是整数 ID
        self.gcs_address = 0  # GCS 固定 ID
        self.u2u_address = []  # 其他 UAV ID 列表

        # 从配置加载
        try:
            peers = rospy.get_param("~sead_peer_ids", [1, 2, 3])
        except Exception:
            peers = [1, 2, 3]
        self.u2u_address = [pid for pid in peers if pid != self.uav_id]

        rospy.loginfo(
            f"[SeadRosBridge] uav={uav_name} id={uav_id} peers={self.u2u_address}"
        )

    # ── 命令接收 ────────────────────────────────

    def _on_command(self, msg: String):
        """GCS → UAV 命令。"""
        try:
            data = self._parse_command(msg.data)
            if data is not None:
                self._rx_queue.put(data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"[bridge] parse cmd failed: {exc}")

    def _on_u2u(self, msg: String):
        """其他 UAV 的消息。"""
        try:
            data = self._parse_u2u(msg.data)
            if data is not None:
                self._rx_queue.put(data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"[bridge] parse u2u failed: {exc}")

    def _parse_command(self, raw: str) -> bytes:
        """将 JSON 命令转为 bytes，维持 packet.unpack() 兼容。"""
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return None

        msg_id = int(obj.get("msg_id", 0))
        info = obj.get("info", obj.get("params", {}))

        return self._encode(msg_id, info)

    def _parse_u2u(self, raw: str) -> bytes:
        """接收共享总线中发给本机的单播或广播，并过滤自身回环。"""
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return None

        try:
            src = int(obj.get("src", -1))
        except (TypeError, ValueError):
            return None
        if src == self.uav_id:
            return None

        is_broadcast = bool(obj.get("broadcast", False))
        dst = obj.get("dst")
        if not is_broadcast:
            try:
                if int(dst) != self.uav_id:
                    return None
            except (TypeError, ValueError):
                return None

        # U2U 消息以 base64 bytes 方式传递
        import base64

        blob = obj.get("blob", "")
        if blob:
            return base64.b64decode(blob)
        return None

    def _encode(self, msg_id: int, info) -> bytes:
        """将 msg_id+info 包装成 communication_info.unpack_packet() 可解析的 bytes。

        这保持现有 msg_handler 分支不变。
        """
        # 1 byte msg_id, 1 byte uav_id (fill later by caller), 可变 payload
        head = struct.pack("<BB", msg_id, self.uav_id)
        payload = self._serialize_info(msg_id, info)
        return head + payload

    def _serialize_info(self, msg_id: int, info) -> bytes:
        """模仿 pack_* 函数生成二进制 payload。"""
        name = MSG_DISPATCH.get(msg_id, None)

        if msg_id == 1:  # Mode_Change
            mode_map = {
                "GUIDED": 4,
                "LAND": 9,
                "LOITER": 5,
                "RTL": 6,
                "AUTO": 3,
                "STABILIZE": 0,
                "POSHOLD": 16,
                "OFFBOARD": 4,
            }
            mode_val = info.get("name", info.get("mode", "GUIDED"))
            if isinstance(mode_val, str):
                mode_val = mode_map.get(mode_val, 4)
            return struct.pack("<B", int(mode_val))

        if msg_id == 2:  # Arm
            armed = bool(info.get("armed", True))
            return struct.pack("<B", 1 if armed else 0)

        if msg_id == 3:  # Takeoff
            alt = float(info.get("alt", 120.0))
            return struct.pack("<B", int(min(alt, 255)))

        if msg_id == 4:  # Time_Synchromize
            return b""

        if msg_id == 5:  # Waypoints
            method = int(info.get("method", 0))
            radius = int(info.get("radius", 50))
            target = info.get("target", [0.0, 0.0, 80.0])
            # pack method(1B) + radius(1B) + 3*int32
            return struct.pack(
                "<BBiii",
                method,
                radius,
                int(float(target[0]) * 1e3),
                int(float(target[1]) * 1e3),
                int(float(target[2]) * 1e3),
            )

        if msg_id == 6:  # Comm_u2gFreq
            freq = float(info.get("freq", 2.0))
            return struct.pack("<i", int(freq * 100))

        if msg_id == 8:  # Mission_Abort
            return b""

        if msg_id == 9:  # Origin_Correction
            oid = int(info.get("origin_id", 0))
            return struct.pack("<B", oid)

        if msg_id == 18:  # SEAD_mission
            # 保持与现有协议一致的 scale
            # 原始协议格式: B(uav_type) B(uav_type2?) → 参照 communication_info.py
            # packet[2]=uav_type(1B), [3:7]=velocity(4B), [7:11]=Rmin(4B), [11]=waypoint_radius(1B)
            # [12:24]=init_pos(3*4B), [24:36]=end(3*4B), [36]=target_num(1B), [37]=unknown_num(1B)
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
            args = [
                uav_type,
                int(velocity * 1e3),
                int(rmin * 1e3),
                radius,
                int(init_pos[0] * 1e3),
                int(init_pos[1] * 1e3),
                int(init_pos[2] * 1e3),
                int(end[0] * 1e3),
                int(end[1] * 1e3),
                int(end[2] * 1e3),
                target_num,
                unknown_num,
            ]
            for t in targets:
                args.append(int(float(t[0]) * 1e3))
                args.append(int(float(t[1]) * 1e3))
            for t in unknown:
                args.append(int(float(t[0]) * 1e3))
                args.append(int(float(t[1]) * 1e3))
            return struct.pack(pack_fmt, *args)

        if msg_id == 19:  # Task_Insert
            point = info.get("point", [0.0, 0.0])
            task_type = int(info.get("task_type", 0))
            return struct.pack(
                "<iiB",
                int(float(point[0]) * 1e3),
                int(float(point[1]) * 1e3),
                task_type,
            )

        if msg_id == 20:  # Airspace_Clear
            return b""

        if msg_id == 21:  # Airspace_ZoneFrag (passed through)
            zone_id = int(info.get("zone_id", 1))
            blob = self._encode_zonedef_v1(info)
            if len(blob) > 255:
                raise ValueError(
                    "Airspace ZoneDef exceeds one bridge packet; "
                    "send pre-fragmented packets instead"
                )
            return struct.pack("<HBBB", zone_id, 0, 1, len(blob)) + blob

        if msg_id == 24:  # Swarm_Command
            params = info.get("params", info)
            shape = params.get("shape", 1)
            if isinstance(shape, str):
                shape = {"TRAIL": 0, "VEE": 1, "ECHELON": 2}.get(
                    shape.upper(), 1
                )
            enable = int(params.get("enable", 1))
            leader = int(params.get("leader_id", 1))
            spacing = float(params.get("spacing", 220.0))
            standoff = float(
                params.get("standoff", params.get("standoff_distance", 5000.0))
            )
            safe_sep = float(
                params.get("safe_sep", params.get("safe_separation", 140.0))
            )
            alt_step = float(
                params.get("alt_step", params.get("altitude_step", 20.0))
            )
            desired = float(params.get("desired_target_time", 0.0))
            return struct.pack(
                "<BBBiiiid",
                enable,
                shape,
                leader,
                int(spacing * 1e3),
                int(standoff * 1e3),
                int(safe_sep * 1e3),
                int(alt_step * 1e3),
                desired,
            )

        if msg_id == 26:  # Formation_Point
            point = info.get("point", [0.0, 0.0, 120.0])
            point_id = int(info.get("point_id", 1))
            loiter_radius = float(info.get("loiter_radius", 300.0))
            return struct.pack(
                "<Hiiii",
                point_id,
                int(float(point[0]) * 1e3),
                int(float(point[1]) * 1e3),
                int(float(point[2]) * 1e3),
                int(loiter_radius * 1e3),
            )

        if msg_id == 44:  # info
            text = info.get("text", str(info))
            text_bytes = text.encode("utf-8")
            return struct.pack("<B", len(text_bytes)) + text_bytes

        # 其他消息: 直接传 JSON bytes
        return json.dumps(info).encode("utf-8")

    @staticmethod
    def _encode_zonedef_v1(info) -> bytes:
        """Encode the ZoneDef v1 payload expected by packet_processing."""
        zone = info.get("zonedef", info)
        if "blob" in info and not zone.get("vertices"):
            import base64

            decoded = base64.b64decode(info["blob"])
            zone = json.loads(decoded.decode("utf-8"))

        vertices = list(zone.get("vertices", []))
        blob = struct.pack(
            "<BBBBBiiH",
            1,
            int(zone.get("zone_type", 0)) & 0xFF,
            1 if bool(zone.get("enabled", True)) else 0,
            int(zone.get("level2d", 0)) & 0xFF,
            int(zone.get("levelH", 0)) & 0xFF,
            int(float(zone.get("minAlt", 0.0)) * 1e3),
            int(float(zone.get("maxAlt", 500.0)) * 1e3),
            len(vertices),
        )
        for east, north in vertices:
            blob += struct.pack("<ii", int(float(east) * 1e3), int(float(north) * 1e3))
        return blob

    # ── 发送 → ROS 话题 ──────────────────────────

    def send_data_async(self, address, raw_data: bytes):
        """仿真 xbee.send_data_async。address 现在是 int(uav_id) 或 GCS_ID。

        将二进制 raw_data 转为 JSON 发布到遥测话题。
        """
        try:
            msg_text = self._bytes_to_json(raw_data, address)
            if self._is_gcs_address(address):
                self._pub_telemetry.publish(String(data=msg_text))
            else:
                self._pub_u2u.publish(String(data=msg_text))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"[bridge] send_async failed: {exc}")

    def send_data_broadcast(self, raw_data: bytes):
        """仿真 xbee.send_data_broadcast。"""
        try:
            msg_text = self._bytes_to_json(raw_data, broadcast=True)
            self._pub_u2u.publish(String(data=msg_text))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"[bridge] broadcast failed: {exc}")

    def _bytes_to_json(self, raw_data: bytes, address=None, broadcast=False) -> str:
        """把二进制数据包装为 JSON 字符串，方便 GCS / 其他节点读取。"""
        import base64

        obj = {
            "ts": time.time(),
            "src": self.uav_id,
            "dst": address,
            "broadcast": broadcast,
            "blob": base64.b64encode(raw_data).decode("ascii"),
            "len": len(raw_data),
        }
        return json.dumps(obj, ensure_ascii=False)

    def _is_gcs_address(self, address) -> bool:
        try:
            return int(address) == int(self.gcs_address)
        except (TypeError, ValueError):
            return False

    # ── 接收 API ────────────────────────────────

    def read_data(self, timeout: float = 1e-5):
        """仿真 xbee.read_data() — 阻塞获取下一个包。

        保持原始 API: 有数据返回 FakePacket，无数据返回 None。
        """
        try:
            raw = self._rx_queue.get(timeout=max(timeout, 1e-6))
            return FakePacket(raw)
        except queue.Empty:
            return None

    # ── 时间同步 ────────────────────────────────

    def time_synchronize_process(self, gcs_addr, _unused_xbee, uav_id):
        """虚拟时间同步 — 直接设置 bias=0，不依赖硬件。

        保持 call-site 兼容:
            new_timer.time_synchronize_process(gcs_address, xbee, uav_id)
        →                .time_synchronize_process(gcs_address, comms, uav_id)
        """
        # 给 Timer 对象打补丁: bias=0 → 用 wall clock
        pass  # 调用方自己检查 self.bias 是否已设置
