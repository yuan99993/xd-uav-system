from struct import pack, unpack
from time import sleep
from enum import Enum
import numpy as np
import dubins
import rospy
import xd_uav_sead.planning.pathFollowing as pf
from xd_uav_sead.planning.GA_SEAD_process import *

SWARM_SHAPE_CODES = {
    "VEE": 1,
    "ECHELON_LEFT": 2,
    "ECHELON_RIGHT": 3,
    "TRAIL": 4,
    "TRIANGLE": 5,
    "WEDGE_WIDE": 6,
    "ARROW": 7,
    "INVERTED_VEE": 8,
}
from time import time  # 导入了模块里的 time 函数


class packet_processing(object):
    def __init__(self, uav_id):
        self.uav_id = uav_id
        ' information among UAVs (SEAD)'
        # self.uavs_info = [[] for _ in range(10)] # 10 info 注释掉，改为字典。因为索引问题导致程序运行错误
        self.uavs_info = {}   # 用 uav_id 作为 key
        # self.task_locking = []
        self.task_locking = {}   # key = uav_id
        self._zone_rx = {}  # zone_id -> {"total":int, "frags":{idx:bytes}, "t0":time()}

# 新增加的 sun
    def ensure_uav(self, uav_id):
        if uav_id not in self.uavs_info:
            self.uavs_info[uav_id] = {
                "id": uav_id,
                "type": None,
                "v": 0.0,
                "Rmin": 0.0,
                "pos": [0.0, 0.0, 0.0],
                "base": [0.0, 0.0, 0.0],
                "fix": False,
                "cost": 0.0,
                "chromosome": [],
                "completed_tasks": [],
                "new_targets": [],
            }
            self.task_locking[uav_id] = False


    def pack_u2g_packet_default(self, mission, frameType, mode, armed, battery, timestamp, position, roll, pitch, yaw, speed):
        if type(mission) == Message_ID:
            msg_id = mission.value
        else:
            msg_id = 0
        # print("\n" + "="*40)
        # print(f"   [Pack U2G Packet Debug Info]   ")

        # print(f"1. Mission   : {mission} (ID: {msg_id})")
        # print(f"2. FrameType : {frameType} (Val: {frameType.value})")
        # print(f"Mode changed to: {mode}")
        # print(f"4. Status    : Armed={armed}, Battery={battery}%")
        # print(f"5. Timestamp : {timestamp}")
        # print(f"6. Position  : Raw={position}")
        # print(f"   -> Scaled : E={int(position[0]*1e3)}, N={int(position[1]*1e3)}, U={int(position[2]*1e3)}")
        # print(f"7. Attitude  : R={roll:.2f}, P={pitch:.2f}, Y={yaw:.2f}")
        # print(f"   -> Scaled : R={int(roll*1e6)}, P={int(pitch*1e6)}, Y={int(yaw*1e6)}")
        # print(f"8. Speed     : {speed:.2f} (Scaled: {int(speed*1e3)})")
        # print("="*40 + "\n")
        return pack('<BBBBBBdiiiiiii', msg_id, self.uav_id, frameType.value, Mode[f"{mode}"].value, int(armed), int(battery), timestamp, 
                    int(position[0]*1e3), int(position[1]*1e3), int(position[2]*1e3), int(roll*1e6), int(pitch*1e6), int(yaw*1e6), int(speed*1e3))

    def pack_info_packet(self, statement):
        string_bytes = statement.encode()
        packet = bytearray([Message_ID.info.value, self.uav_id, len(string_bytes)])
        return packet + string_bytes

    def pack_record_time_packet(self, statement, time):
        string_bytes = statement.encode()
        packet = pack('<BBdB', Message_ID.Record_Time.value, self.uav_id, time, len(string_bytes))
        return packet + string_bytes
    
    def pack_SEAD_packet(self, uav_type, uav_velocity, uav_Rmin, UAV_pos, base_config, fix, cost, chromosome, tasks_completed, taregts_found):
        '''
            [id, type, v, Rmin, UAV pos, base config, Xtf, Nt, Nct, NnT, cost, chromosome, tasks completed, taregts found]
            UAV pos : (x,y,yaw)
            base config : (x_base, y_base, angle_base)
            Xtf : variable for task fixing 
            Nt : tasks number
            Nct : completed tasks number
            NnT : newly found targets number
            cost : objective value of the chromosome
            chromosome : current best solution
        '''
        if chromosome:
            Nt = len(chromosome[0])
        else:
            Nt = 0
        Nct = len(tasks_completed)
        NnT = len(taregts_found)
        packet = pack('<BBBiiiiiiiiBBBBi', Message_ID.SEAD.value, self.uav_id, uav_type, int(uav_velocity*1e3), int(uav_Rmin*1e3), 
                      int(UAV_pos[0]*1e3), int(UAV_pos[1]*1e3), int(UAV_pos[2]*1e3), 
                      int(base_config[0]*1e3), int(base_config[1]*1e3), int(base_config[2]*1e3), int(fix), Nt, Nct, NnT, int(cost*1e3))
        for gene in chromosome:
            packet += bytearray(gene)
        for task in tasks_completed:
            packet += bytearray(task)
        for target in taregts_found:
            packet += pack('<ii', int(target[0]*1e3), int(target[1]*1e3))
        # ' Add the information of the UAV itself '
        # self.uavs_info[0].append(self.uav_id)
        # self.uavs_info[1].append(uav_type)
        # self.uavs_info[2].append(uav_velocity)
        # self.uavs_info[3].append(uav_Rmin)
        # self.uavs_info[4].append(UAV_pos)
        # self.uavs_info[5].append(base_config)
        # self.uavs_info[6].append(cost)
        # self.uavs_info[7].append(chromosome)
        # self.uavs_info[8].extend(tasks_completed)
        # self.uavs_info[9].extend(taregts_found)
        # self.task_locking.append(fix)
        self.ensure_uav(self.uav_id)
        info = self.uavs_info[self.uav_id]

        info["type"] = uav_type
        info["v"] = uav_velocity
        info["Rmin"] = uav_Rmin
        info["pos"] = UAV_pos
        info["base"] = base_config
        info["fix"] = fix
        info["cost"] = cost
        info["chromosome"] = chromosome
        info["completed_tasks"] = list(tasks_completed)
        info["new_targets"] = list(taregts_found)

        self.task_locking[self.uav_id] = fix

        return packet, UAV_pos

    def pack_simple_strike_assignment(self, leader_id, target_count, assignment_map):
        count = min(int(target_count), len(assignment_map), 255)
        packet = pack(
            '<BBBB',
            Message_ID.SimpleStrike_Assignment.value,
            int(self.uav_id) & 0xFF,
            int(leader_id) & 0xFF,
            count & 0xFF,
        )
        for uav_id, target in sorted(assignment_map.items(), key=lambda item: int(item[0]))[:count]:
            target_id = int(target.get("target_id", 0)) if isinstance(target, dict) else 0
            point = target.get("point", [0.0, 0.0]) if isinstance(target, dict) else target
            packet += pack(
                '<BBii',
                int(uav_id) & 0xFF,
                target_id & 0xFF,
                int(float(point[0]) * 1e3),
                int(float(point[1]) * 1e3),
            )
        return packet

    def pack_simple_strike_assignment_ack(self, leader_id, target_count):
        return pack(
            '<BBBB',
            Message_ID.SimpleStrike_Assignment_Ack.value,
            int(self.uav_id) & 0xFF,
            int(leader_id) & 0xFF,
            int(target_count) & 0xFF,
        )

    def pack_simple_strike_release_time(self, leader_id, target_count, release_time):
        return pack(
            '<BBBBd',
            Message_ID.SimpleStrike_ReleaseTime.value,
            int(self.uav_id) & 0xFF,
            int(leader_id) & 0xFF,
            int(target_count) & 0xFF,
            float(release_time),
        )

    def pack_simple_strike_path_status(self, leader_id, target_count, status):
        status = dict(status or {})

        def _float_or_nan(value):
            if value is None:
                return float("nan")
            try:
                return float(value)
            except Exception:
                return float("nan")

        control_path_source = str(status.get("control_path_source", "unknown"))
        source_bytes = control_path_source.encode("utf-8")[:255]
        packet = pack(
            '<BBBBBBBdddddddddddddddBB',
            Message_ID.SimpleStrike_PathStatus.value,
            int(self.uav_id) & 0xFF,
            int(leader_id) & 0xFF,
            int(target_count) & 0xFF,
            int(status.get("uav_id", self.uav_id)) & 0xFF,
            int(status.get("target_id", 0) or 0) & 0xFF,
            1 if bool(status.get("path_ready", False)) else 0,
            _float_or_nan(status.get("path_length")),
            _float_or_nan(status.get("remaining_path_length")),
            _float_or_nan(status.get("sync_remaining_path_length")),
            _float_or_nan(status.get("arrival_radius_for_sync")),
            _float_or_nan(status.get("dist_to_target")),
            _float_or_nan(status.get("target_sync_remaining")),
            _float_or_nan(status.get("eta")),
            _float_or_nan(status.get("actual_groundspeed")),
            _float_or_nan(status.get("predicted_arrival_time")),
            _float_or_nan(status.get("predicted_arrival_error")),
            _float_or_nan(status.get("target_closure_speed")),
            _float_or_nan(status.get("sync_progress_speed")),
            _float_or_nan(status.get("target_closure_speed_ema")),
            _float_or_nan(status.get("sync_progress_speed_ema")),
            _float_or_nan(status.get("stamp", time())),
            1 if bool(status.get("predicted_arrival_used_for_control", False)) else 0,
            len(source_bytes) & 0xFF,
        )
        return packet + source_bytes

    # def SEAD_info_clear(self):
    #     self.uavs_info = [[] for _ in range(10)]
    #     self.task_locking = []
#修改原方法
    def SEAD_info_clear(self):
        for info in self.uavs_info.values():
            info["completed_tasks"].clear()
            info["new_targets"].clear()

    def pack_airspace_ack(self, zone_id: int, ok: int, err: int = 0):
    # [msg_id][uav_id][zone_id:u16][ok:u8][err:u8]
        return pack('<BBHBB', Message_ID.Airspace_Ack.value, self.uav_id, int(zone_id) & 0xFFFF, int(ok) & 0xFF, int(err) & 0xFF)

    def pack_swarm_command(
        self,
        uav_to: int,
        enable: int = 1,
        shape: int = 1,
        leader_id: int = 0,
        spacing: float = 220.0,
        standoff: float = 5000.0,
        safe_sep: float = 140.0,
        alt_step: float = 20.0,
        desired_target_time: float = 0.0,
    ):
        if isinstance(shape, str):
            shape = SWARM_SHAPE_CODES.get(shape.upper(), SWARM_SHAPE_CODES["VEE"])
        elif hasattr(shape, "value"):
            shape = int(shape.value)
        return pack(
            '<BBBBBiiiid',
            Message_ID.Swarm_Command.value,
            int(uav_to) & 0xFF,
            int(enable) & 0xFF,
            int(shape) & 0xFF,
            int(leader_id) & 0xFF,
            int(spacing * 1e3),
            int(standoff * 1e3),
            int(safe_sep * 1e3),
            int(alt_step * 1e3),
            float(desired_target_time),
        )

    def pack_formation_state_packet(
        self,
        timestamp,
        position,
        velocity,
        yaw,
        phase,
        slot_id,
        sync_eta=None,
        common_target_time: float = 0.0,
    ):
        eta_ms = -1 if sync_eta is None else int(float(sync_eta) * 1e3)
        return pack(
            '<BBdiiiiiiiBBid',
            Message_ID.Formation_State.value,
            self.uav_id,
            float(timestamp),
            int(position[0] * 1e3),
            int(position[1] * 1e3),
            int(position[2] * 1e3),
            int(velocity[0] * 1e3),
            int(velocity[1] * 1e3),
            int(velocity[2] * 1e3),
            int(yaw * 1e6),
            int(phase) & 0xFF,
            int(slot_id) & 0xFF,
            eta_ms,
            float(common_target_time or 0.0),
        )

    def pack_formation_point(
        self,
        uav_to: int,
        point_id: int,
        point,
        loiter_radius: float = 300.0,
    ):
        return pack(
            '<BBHiiii',
            Message_ID.Formation_Point.value,
            int(uav_to) & 0xFF,
            int(point_id) & 0xFFFF,
            int(point[0] * 1e3),
            int(point[1] * 1e3),
            int(point[2] * 1e3),
            int(loiter_radius * 1e3),
        )

    def _parse_zonedef_blob_v1(self, blob: bytes):
        # return dict: {version,type,enabled,level2d,levelH,minAlt,maxAlt,vertices[(E,N)...]}  # ENU meters
        if blob is None or len(blob) < 1 + 1 + 1 + 1 + 1 + 4 + 4 + 2:
            raise ValueError("ZoneDef blob too short")

        version = blob[0]
        if version != 1:
            raise ValueError(f"unsupported ZoneDef version {version}")

        zone_type = blob[1]
        enabled = blob[2]
        level2d = blob[3]
        levelH = blob[4]
        minAlt_mm = unpack('<i', blob[5:9])[0]
        maxAlt_mm = unpack('<i', blob[9:13])[0]
        n = unpack('<H', blob[13:15])[0]

        need = 15 + n * 8
        if len(blob) < need:
            raise ValueError("ZoneDef blob incomplete")


        verts = []
        off = 15
        for _ in range(n):
            e_mm = unpack('<i', blob[off:off+4])[0]
            n_mm = unpack('<i', blob[off+4:off+8])[0]
            off += 8
            verts.append((e_mm / 1000.0, n_mm / 1000.0))  # ENU meters


        return {
            "version": version,
            "zone_type": zone_type,
            "enabled": bool(enabled),
            "level2d": int(level2d),
            "levelH": int(levelH),
            "minAlt": minAlt_mm / 1000.0,
            "maxAlt": maxAlt_mm / 1000.0,
            "vertices": verts,
        }

    def unpack_packet(self, packet):
        try:
            msg_id = Message_ID(packet[0])
            # Formation state is periodic U2U telemetry.  Logging every packet
            # from every peer creates avoidable scheduler and rosout pressure.
            if msg_id == Message_ID.Formation_State:
                rospy.logdebug_throttle(
                    5.0, f"Received periodic message ID: {msg_id.name}"
                )
            else:
                rospy.loginfo(f"Received message ID: {msg_id.name}")
        except ValueError:
            return Message_ID.info, "invalid message ID"

        ' Commands (G2U) '
        if msg_id == Message_ID.Arm:
            if packet[1] == self.uav_id:
                return Message_ID.Arm, Armed(packet[2])
            else:
                return Message_ID.info, f"Wrong UAV delegation on {Armed(packet[2]).name} command"
        elif msg_id == Message_ID.Mode_Change:
            if packet[1] == self.uav_id:
                return Message_ID.Mode_Change, Mode(packet[2])
            else:
                return Message_ID.info, "Wrong UAV delegation on change mode command"
        elif msg_id == Message_ID.Time_Synchromize:
            if packet[1] == self.uav_id:
                return Message_ID.Time_Synchromize, None
            else:
                return Message_ID.info, "time sync fail"
        elif msg_id == Message_ID.Takeoff:
            if packet[1] == self.uav_id:
                return Message_ID.Takeoff, packet[2]
            else:
                return Message_ID.info, "Wrong UAV delegation on takeoff command"
        elif msg_id == Message_ID.Mission_Abort:
            if packet[1] == self.uav_id:
                return Message_ID.Mission_Abort, None
            else:
                return Message_ID.info, "Wrong UAV delegation on mission abort command"
        elif msg_id == Message_ID.Origin_Correction:
            if packet[1] == self.uav_id:
                return Message_ID.Origin_Correction, packet[2]
            else:
                return Message_ID.info, "Wrong UAV delegation on origin correction"
        elif msg_id == Message_ID.Waypoints:
            if packet[1] == self.uav_id:
                try:
                    type = WaypointMissionMethod(packet[2])
                except ValueError:
                    return Message_ID, "invalid waypoints mission method"

                waypoint_radius = packet[3]
                if type == WaypointMissionMethod.guide_waypoint:
                    waypoint = np.multiply(unpack('iii', packet[4:]), 1e-3)
                    return Message_ID.Waypoints, [WaypointMissionMethod.guide_waypoint, waypoint_radius, waypoint]
                elif type == WaypointMissionMethod.guide_WPwithHeading:
                    waypoint = np.multiply(unpack('iiii', packet[4:]), 1e-3)
                    return Message_ID.Waypoints, [WaypointMissionMethod.guide_WPwithHeading, waypoint_radius, waypoint]
                elif type == WaypointMissionMethod.guide_waypoints:
                    WPs_num = packet[4]
                    wps_index = 5
                    waypoints = []
                    for i in range(WPs_num):
                        waypoints.append(np.multiply(unpack('iii', packet[wps_index+12*i:wps_index+12+12*i]), 1e-3))
                    return Message_ID.Waypoints, [WaypointMissionMethod.guide_waypoints, waypoint_radius, waypoints]    
                elif type == WaypointMissionMethod.CraigReynolds_Path_Following:
                    WPs_num = packet[4]
                    try:
                        method = pathFollowingMethod(packet[5])
                    except ValueError:
                        return Message_ID.info, "invalid path following method"
                        
                    recedingHorizon = packet[6] / 10
                    velocity = unpack('i', packet[7:11])[0] * 1e-3
                    Rmin = unpack('i', packet[11:15])[0] * 1e-3
                    path = []    
                    if method == pathFollowingMethod.path_following_velocityBody_PID:
                        Kp = unpack('i', packet[15:19])[0] * 1e-3
                        Kd = unpack('i', packet[19:23])[0] * 1e-3
                        wps_index = 23
                        for i in range(WPs_num):
                            wp = np.multiply(unpack('iii', packet[wps_index+12*i:wps_index+12+12*i]), 1e-3)
                            path.append(wp)
                        return Message_ID.Waypoints, [WaypointMissionMethod.CraigReynolds_Path_Following, waypoint_radius, 
                                                    pf.CraigReynolds_Path_Following(pathFollowingMethod.dubinsPath_following_velocityBody_PID, recedingHorizon, path, Kp=Kp, Kd=Kd), [velocity, Rmin]]
                    elif method == pathFollowingMethod.dubinsPath_following_velocityBody_PID:
                        Kp = unpack('i', packet[15:19])[0] * 1e-3
                        Kd = unpack('i', packet[19:23])[0] * 1e-3
                        wps_index = 23
                        for i in range(WPs_num):
                            wp = np.multiply(unpack('iiii', packet[wps_index+16*i:wps_index+16+16*i]), 1e-3)
                            path.append(wp)
                        path = generate_dubinsPath([[p[0], p[1], p[3]*np.pi/180] for p in path], Rmin, velocity/10)
                        return Message_ID.Waypoints, [WaypointMissionMethod.CraigReynolds_Path_Following, waypoint_radius, 
                                                    pf.CraigReynolds_Path_Following(pathFollowingMethod.dubinsPath_following_velocityBody_PID, recedingHorizon, path, Kp=Kp, Kd=Kd), [velocity, Rmin]]
                    else:
                        wps_index = 15
                        for i in range(WPs_num+1):
                            wp = np.multiply(unpack('iii', packet[wps_index+12*i:wps_index+12+12*i]), 1e-3)
                            path.append(wp)
                        return Message_ID.Waypoints, [WaypointMissionMethod.CraigReynolds_Path_Following, waypoint_radius, 
                                                      pf.CraigReynolds_Path_Following(method, recedingHorizon, path), [velocity, Rmin]]
            else:
                return Message_ID.info, f"Wrong UAV delegate on {WaypointMissionMethod(packet[3])} command"
        
        elif msg_id == Message_ID.Comm_u2gFreq:
            if packet[1] == self.uav_id:
                frequency = unpack('i', packet[2:])[0] * 1e-2
                return Message_ID.Comm_u2gFreq, frequency

        elif msg_id == Message_ID.Swarm_Command:
            uav_to = packet[1]
            if uav_to == 0 or uav_to == self.uav_id:
                if len(packet) < 29:
                    return Message_ID.info, "invalid Swarm_Command packet"
                enable = bool(packet[2])
                shape = int(packet[3])
                leader_id = int(packet[4])
                spacing = unpack('<i', packet[5:9])[0] * 1e-3
                standoff = unpack('<i', packet[9:13])[0] * 1e-3
                safe_sep = unpack('<i', packet[13:17])[0] * 1e-3
                alt_step = unpack('<i', packet[17:21])[0] * 1e-3
                desired_target_time = unpack('<d', packet[21:29])[0]
                return Message_ID.Swarm_Command, {
                    "command": "formation_config",
                    "params": {
                        "enable": enable,
                        "shape": shape,
                        "leader_id": leader_id,
                        "spacing": spacing,
                        "standoff_distance": standoff,
                        "safe_separation": safe_sep,
                        "altitude_step": alt_step,
                        "desired_target_time": desired_target_time,
                    },
                }
            else:
                return Message_ID.info, "Wrong UAV delegation on Swarm_Command"

        elif msg_id == Message_ID.Formation_State:
            if len(packet) < 44:
                return Message_ID.info, "invalid Formation_State packet"
            peer_id = packet[1]
            timestamp = unpack('<d', packet[2:10])[0]
            position = list(np.multiply(unpack('<iii', packet[10:22]), 1e-3))
            velocity = list(np.multiply(unpack('<iii', packet[22:34]), 1e-3))
            yaw = unpack('<i', packet[34:38])[0] * 1e-6
            phase = int(packet[38])
            slot_id = int(packet[39])
            sync_eta_ms = unpack('<i', packet[40:44])[0]
            common_target_time = (
                unpack('<d', packet[44:52])[0] if len(packet) >= 52 else 0.0
            )
            return Message_ID.Formation_State, {
                "uav_id": peer_id,
                "timestamp": timestamp,
                "position": position,
                "velocity": velocity,
                "yaw": yaw,
                "phase": phase,
                "slot_id": slot_id,
                "sync_eta": None if sync_eta_ms < 0 else sync_eta_ms * 1e-3,
                "common_target_time": common_target_time,
            }

        elif msg_id == Message_ID.Formation_Point:
            uav_to = packet[1]
            if uav_to == 0 or uav_to == self.uav_id:
                if len(packet) < 20:
                    return Message_ID.info, "invalid Formation_Point packet"
                point_id = unpack('<H', packet[2:4])[0]
                e = unpack('<i', packet[4:8])[0] * 1e-3
                n = unpack('<i', packet[8:12])[0] * 1e-3
                z = unpack('<i', packet[12:16])[0] * 1e-3
                loiter_radius = unpack('<i', packet[16:20])[0] * 1e-3
                return Message_ID.Formation_Point, {
                    "point_id": point_id,
                    "point": [e, n, z],
                    "loiter_radius": loiter_radius,
                }
            else:
                return Message_ID.info, "Wrong UAV delegation on Formation_Point"
            

        elif msg_id == Message_ID.Airspace_Clear:
            uav_to = packet[1]
            if uav_to == 0 or uav_to == self.uav_id:
                return Message_ID.Airspace_Clear, None
            else:
                return Message_ID.info, "Wrong UAV delegation on Airspace_Clear"

        elif msg_id == Message_ID.Airspace_ZoneFrag:
            uav_to = packet[1]
            # if not (uav_to == 0 or uav_to == self.uav_id):
            #     return Message_ID.info, "Wrong UAV delegation on Airspace_ZoneFrag"
            zone_id = unpack('<H', packet[2:4])[0]
            frag_idx = packet[4]
            frag_total = packet[5]
            payload_len = packet[6]
            
            # 使用 loginfo 打印接收状态
            rospy.loginfo(f">>> [底层RX] 收到禁飞区包! ZoneID={zone_id}, 分片={frag_idx+1}/{frag_total}")
            payload = bytes(packet[7:7+payload_len])

            now = time()
            rec = self._zone_rx.get(zone_id)
            if rec is None or rec.get("total") != frag_total or (now - rec.get("t0", now)) > 10.0:
                # 修改：print -> rospy.loginfo
                rospy.loginfo(f"[RX][ZoneFrag][RESET] zid={zone_id} old_total={rec.get('total') if rec else None} new_total={frag_total}")
                rec = {"total": int(frag_total), "frags": {}, "t0": now}
                self._zone_rx[zone_id] = rec
            
            rec["frags"][int(frag_idx)] = payload
            # 修改：print -> rospy.loginfo
            rospy.loginfo(f"[RX][ZoneFrag] to={packet[1]} zid={zone_id} frag={frag_idx+1}/{frag_total} payload_len={payload_len} now_frags={len(rec['frags'])}分片进度")

            if len(rec["frags"]) == rec["total"]:
                blob = b''.join(rec["frags"][i] for i in range(rec["total"]))
                try:
                    zonedef = self._parse_zonedef_blob_v1(blob)
                    # 修改：print -> rospy.loginfo (解析成功用 info)
                    rospy.loginfo(f"[RX][ZoneFrag] COMPLETE zid={zone_id} verts={len(zonedef['vertices'])} alt=({zonedef['minAlt']},{zonedef['maxAlt']}) first={zonedef['vertices'][0] if zonedef['vertices'] else None}是否受到，是否拼齐，是否解析成功")
                except Exception as e:
                    del self._zone_rx[zone_id]
                    # 修改：print -> rospy.logerr (解析失败用 error)
                    rospy.logerr(f"[RX][ZoneFrag][PARSE_ERR] zid={zone_id} err={e}")
                    return Message_ID.Airspace_ZoneFrag, {"zone_id": zone_id, "ok": False, "error": str(e), "zonedef": None}

                del self._zone_rx[zone_id]
                return Message_ID.Airspace_ZoneFrag, {"zone_id": zone_id, "ok": True, "error": "", "zonedef": zonedef}
            else:
                return Message_ID.Airspace_ZoneFrag, {"zone_id": zone_id, "ok": None, "error": "", "zonedef": None}



        elif msg_id == Message_ID.SEAD_mission: 
            uav_to = packet[1]
            if uav_to == 0 or uav_to == self.uav_id:
                uav_type = packet[2]
                velocity = unpack('i', packet[3:7])[0] * 1e-3
                Rmin = unpack('i', packet[7:11])[0] * 1e-3
                waypoint_radius = packet[11]
                init_pos = list(np.multiply(unpack('iii', packet[12:24]), 1e-3))
                init_pos[2] = pf.PlusMinusPi(init_pos[2]*np.pi/180)
                end = list(np.multiply(unpack('iii', packet[24:36]), 1e-3))
                end[2] = end[2]*np.pi/180
                taregt_num, unknown_target_num = packet[36], packet[37]
                targets = [list(np.multiply(unpack('ii', packet[38+8*i:46+8*i]), 1e-3)) for i in range(taregt_num)]
                unknown_targets = [list(np.multiply(unpack('ii', packet[38+taregt_num*8+8*i:46+taregt_num*8+8*i]), 1e-3)) for i in range(unknown_target_num)]
                return Message_ID.SEAD_mission, [targets, unknown_targets, init_pos, end, [uav_type, velocity, Rmin], waypoint_radius]
            else:
                rospy.logwarn(
                    f"[SEAD][RX] Wrong UAV delegate on SEAD mission: uav_to={uav_to}, self_uav_id={self.uav_id}"
                )
                return Message_ID.info, "Wrong UAV delegate on SEAD mission"
        
        elif msg_id == Message_ID.SEAD:
            uav_id = packet[1]
            self.ensure_uav(uav_id)
            info = self.uavs_info[uav_id]

            info["type"] = packet[2]
            info["v"] = unpack('i', packet[3:7])[0] * 1e-3
            info["Rmin"] = unpack('i', packet[7:11])[0] * 1e-3
            info["pos"] = list(np.multiply(unpack('iii', packet[11:23]), 1e-3))
            info["base"] = list(np.multiply(unpack('iii', packet[23:35]), 1e-3))

            fix = bool(packet[35])
            Nt, Nct, NnT = packet[36], packet[37], packet[38]
            info["fix"] = fix
            info["cost"] = unpack('i', packet[39:43])[0] * 1e-3

            chromosome = []
            if Nt != 0:
                for i in range(5):
                    chromosome.append(list(packet[43+i*Nt:43+Nt*(i+1)]))
            info["chromosome"] = chromosome

            offset = 43 + Nt * 5

            completed = []
            for i in range(Nct):
                completed.append(list(packet[offset+i*2:offset+(i+1)*2]))
            info["completed_tasks"] = completed

            offset += Nct * 2

            new_targets = []
            for i in range(NnT):
                new_targets.append(list(np.multiply(unpack('ii', packet[offset+i*8:offset+(i+1)*8]), 1e-3)))
            info["new_targets"] = new_targets

            self.task_locking[uav_id] = fix
            return Message_ID.SEAD, None

        elif msg_id == Message_ID.SimpleStrike_Assignment:
            if len(packet) < 4:
                return Message_ID.info, "invalid SimpleStrike_Assignment packet"
            sender_id = packet[1]
            leader_id = packet[2]
            target_count = packet[3]
            expected_len = 4 + int(target_count) * 10
            if len(packet) < expected_len:
                return Message_ID.info, "invalid SimpleStrike_Assignment payload"
            assignment_map = {}
            offset = 4
            for _ in range(target_count):
                uav_id, target_id, x_mm, y_mm = unpack('<BBii', packet[offset:offset + 10])
                offset += 10
                assignment_map[int(uav_id)] = {
                    "target_id": int(target_id),
                    "point": [x_mm * 1e-3, y_mm * 1e-3],
                }
            return Message_ID.SimpleStrike_Assignment, {
                "sender_id": int(sender_id),
                "leader_id": int(leader_id),
                "target_count": int(target_count),
                "assignment_map": assignment_map,
            }

        elif msg_id == Message_ID.SimpleStrike_Assignment_Ack:
            if len(packet) < 4:
                return Message_ID.info, "invalid SimpleStrike_Assignment_Ack packet"
            return Message_ID.SimpleStrike_Assignment_Ack, {
                "sender_uav_id": int(packet[1]),
                "leader_id": int(packet[2]),
                "target_count": int(packet[3]),
            }

        elif msg_id == Message_ID.SimpleStrike_ReleaseTime:
            if len(packet) < 12:
                return Message_ID.info, "invalid SimpleStrike_ReleaseTime packet"
            return Message_ID.SimpleStrike_ReleaseTime, {
                "sender_uav_id": int(packet[1]),
                "leader_id": int(packet[2]),
                "target_count": int(packet[3]),
                "release_time": unpack('<d', packet[4:12])[0],
            }

        elif msg_id == Message_ID.SimpleStrike_PathStatus:
            min_len = 129
            if len(packet) < min_len:
                return Message_ID.info, "invalid SimpleStrike_PathStatus packet"
            (
                path_length,
                remaining_path_length,
                sync_remaining_path_length,
                arrival_radius_for_sync,
                dist_to_target,
                target_sync_remaining,
                eta,
                actual_groundspeed,
                predicted_arrival_time,
                predicted_arrival_error,
                target_closure_speed,
                sync_progress_speed,
                target_closure_speed_ema,
                sync_progress_speed_ema,
                stamp,
            ) = unpack('<ddddddddddddddd', packet[7:127])
            predicted_arrival_used_for_control = bool(packet[127])
            source_len = int(packet[128])
            source_start = 129
            source_end = min(len(packet), source_start + source_len)
            control_path_source = packet[source_start:source_end].decode(
                "utf-8",
                errors="replace",
            )

            def _none_if_nan(value):
                try:
                    return None if np.isnan(float(value)) else float(value)
                except Exception:
                    return None

            status = {
                "uav_id": int(packet[4]),
                "target_id": int(packet[5]),
                "path_ready": bool(packet[6]),
                "path_length": _none_if_nan(path_length),
                "remaining_path_length": _none_if_nan(remaining_path_length),
                "sync_remaining_path_length": _none_if_nan(
                    sync_remaining_path_length
                ),
                "arrival_radius_for_sync": _none_if_nan(arrival_radius_for_sync),
                "dist_to_target": _none_if_nan(dist_to_target),
                "target_sync_remaining": _none_if_nan(target_sync_remaining),
                "eta": _none_if_nan(eta),
                "control_path_source": control_path_source or "unknown",
                "actual_groundspeed": _none_if_nan(actual_groundspeed),
                "predicted_arrival_time": _none_if_nan(predicted_arrival_time),
                "predicted_arrival_error": _none_if_nan(predicted_arrival_error),
                "target_closure_speed": _none_if_nan(target_closure_speed),
                "sync_progress_speed": _none_if_nan(sync_progress_speed),
                "target_closure_speed_ema": _none_if_nan(
                    target_closure_speed_ema
                ),
                "sync_progress_speed_ema": _none_if_nan(sync_progress_speed_ema),
                "predicted_arrival_used_for_control": (
                    predicted_arrival_used_for_control
                ),
                "stamp": _none_if_nan(stamp),
            }
            return Message_ID.SimpleStrike_PathStatus, {
                "sender_uav_id": int(packet[1]),
                "leader_id": int(packet[2]),
                "target_count": int(packet[3]),
                "status": status,
            }
 
        
        
        elif msg_id == Message_ID.Task_Insert:
            uav_to = packet[1]
            if uav_to == 0 or uav_to == self.uav_id:
                E = unpack('i', packet[2:6])[0] * 1e-3
                N = unpack('i', packet[6:10])[0] * 1e-3
                task_type = packet[10] if len(packet) >= 11 else 0
                return Message_ID.Task_Insert, [E, N, task_type]
            else:
                return Message_ID.info, "Wrong UAV delegation on Task_Insert"
           


class Message_ID(Enum):
    Default = 0
    Mode_Change = 1
    Arm = 2
    Takeoff = 3
    Waypoints = 5
    Time_Synchromize = 4
    Comm_u2gFreq = 6
    Record_Time = 7
    Mission_Abort = 8
    Origin_Correction = 9
    info = 44
    SEAD = 17          # (U2U)
    SEAD_mission = 18  # (G2U)
    Task_Insert = 19  # (G2U)
    Airspace_Clear = 20
    Airspace_ZoneFrag = 21
    Airspace_ZoneRemove = 22
    Airspace_Ack = 23
    Swarm_Command = 24 #sun
    Formation_State = 25
    Formation_Point = 26
    SimpleStrike_Assignment = 27
    SimpleStrike_Assignment_Ack = 28
    SimpleStrike_ReleaseTime = 30
    SimpleStrike_PathStatus = 32



class Mode(Enum):
    STABILIZE = 0
    ARCO = 1
    ALT_HOD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    POSITION = 8
    LAND = 9
    OF_LOITER = 10
    DRIFT = 11
    SPORT = 12
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    AVOID_ADSB = 19
    GUIDED_NOGPS = 20


class Armed(Enum):
    armed = 1
    disarmed = 0


class FrameType(Enum):
    Quad = 0
    Fixed_wing = 1


class WaypointMissionMethod(Enum):
    guide_waypoint = 0
    guide_WPwithHeading = 3
    guide_waypoints = 1
    CraigReynolds_Path_Following = 2


class pathFollowingMethod(Enum):
    path_following_position = 0
    path_following_position_yaw = 1
    path_following_velocityLocal =  2
    path_following_velocityBody_PID = 3
    # Historical protocol name. Keep this value for GCS packet compatibility.
    dubinsPath_following_velocityBody_PID = 4


def generate_dubinsPath(points, radius, interval):
    dubins_path = [points[0]]
    for i in range(len(points)-1):
        if points[i][2] == points[i+1][2]:
            points[i+1][2] += 1e-3
        dubins_path.extend(dubins.shortest_path(points[i], points[i+1], radius).sample_many(interval)[0][1:])
    return dubins_path
