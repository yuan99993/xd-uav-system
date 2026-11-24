#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gimbal_attitude_bridge.py — 云台真实姿态桥接节点

从 mavros 原始 MAVLink 流解析 GIMBAL_DEVICE_ATTITUDE_STATUS(#285)，
发布成轻量标准话题，供其它控制程序订阅，无需各自解析 mavlink。

发布话题: <uav_ns>/gimbal/attitude   std_msgs/Float64MultiArray
   data = [yaw_deg, pitch_deg, roll_deg]   (欧拉角, 度)
   - yaw:   机体相对角(相对机头, ±180°)。#285 YAW_LOCK 置位时 yaw 为
            世界系, 本节点用飞机航向换算成机体相对: yaw_body = wrap180(yaw_world - heading)
   - pitch: 负值 = 低头(下视), 正值 = 抬头。沿用 #285 原始四元数约定
   - roll:  云台横滚(度), 稳像时≈0, |值|<1e-3 归零避免浮点尾渣
   yaw 为机体相对角(与 QGC 云台面板同基准)；飞机航向取
   <uav_ns>/mavros/global_position/compass_hdg；航向未就绪时 yaw 暂按
   原值输出并告警。

同时发布: <uav_ns>/track/gimbal_state   xd_uav_track/GimbalState
   yaw_rad/pitch_rad/roll_rad 使用 xd_uav_track 的约定：偏航向右为正、
   俯仰向下为正、横滚顺时针为正。由于本节点原始 pitch 是“低头为负”，
   输出给 track 时默认反转 pitch 符号。

参数:
   _uav_ns             mavros 命名空间前缀(默认 /uav1)
   _request_rate       启动时是否发 SET_MESSAGE_INTERVAL 请求 #285@10Hz(默认 True)
                       仿真器对 mavros 连接不一定持续推 #285(实测空闲时 30s 仅 1 条),
                       发一次速率请求可保证 9~10Hz, 与 QGC 共存无冲突
   _track_gimbal_state_topic track 姿态输出话题(默认 /uav1/track/gimbal_state)
   _track_yaw_sign     track 偏航符号(默认 +1)
   _track_pitch_sign   track 俯仰符号(默认 -1，将低头负值转为 track 的低头正值)
   _track_roll_sign    track 横滚符号(默认 +1)

运行:
   source /opt/ros/noetic/setup.bash
   source /home/nvidia/xd-uavsystem-test/devel/setup.bash
   source /home/nvidia/xd-uavsystem-test/.venv-sar/bin/activate
   python3 /home/nvidia/image_compress/gimbal_attitude_bridge.py _uav_ns:=/uav1
"""

import math
import struct

import rospy
from mavros_msgs.msg import Mavlink
from std_msgs.msg import Float64, Float64MultiArray
from xd_uav_track.msg import GimbalState as TrackGimbalState

# pymavlink 必须用 v20 common 方言(默认 v10 ardupilotmega 无 #285)
from pymavlink.dialects.v20 import common as mavlink2

MSG_GIMBAL_DEVICE_ATTITUDE_STATUS = 285
MAV_CMD_SET_MESSAGE_INTERVAL = 511
GIMBAL_SYSID = 1        # 自动驾驶仪(仿真器) sysid
GIMBAL_COMPID = 154     # MAV_COMP_ID_GIMBAL


def rebuild_frame(msg):
    """mavros_msgs/Mavlink → 完整 MAVLink 帧字节(同 gimbal_control.py)。"""
    payload = b''.join(struct.pack('<Q', w) for w in msg.payload64)
    payload = payload[:msg.len]
    msgid = msg.msgid & 0xFFFFFF
    header = struct.pack('<BBBBBBB', msg.magic, msg.len, msg.incompat_flags,
                         msg.compat_flags, msg.seq, msg.sysid, msg.compid)
    header += bytes([msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    return header + payload + struct.pack('<H', msg.checksum)


def buf_to_mavlink_msg(buf):
    """pymavlink pack() 出的完整 v2 帧 → mavros_msgs/Mavlink(同 gimbal_control.py)。"""
    plen = buf[1]
    payload = buf[10:10 + plen]
    checksum = struct.unpack('<H', buf[10 + plen:12 + plen])[0]
    pad = (8 - len(payload) % 8) % 8
    payload += b'\0' * pad
    msg = Mavlink()
    msg.header.stamp = rospy.Time.now()
    msg.framing_status = Mavlink.FRAMING_OK
    msg.magic = buf[0]
    msg.len = plen
    msg.incompat_flags = buf[2]
    msg.compat_flags = buf[3]
    msg.seq = buf[4]
    msg.sysid = buf[5]
    msg.compid = buf[6]
    msg.msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
    msg.checksum = checksum
    msg.payload64 = list(struct.unpack('<%dQ' % (len(payload) // 8), payload))
    return msg


class GimbalAttitudeBridge:

    def __init__(self):
        # Multiple aircraft run this bridge in the same ROS master.  The UAV
        # namespace is selected by the private ~uav_ns parameter, so without
        # an anonymous ROS node name every instance registers as
        # /gimbal_attitude_bridge and a later UAV evicts the earlier one.
        # Keep the topic namespace controlled by ~uav_ns, but give each
        # running process a unique node registration name.
        rospy.init_node('gimbal_attitude_bridge', anonymous=True)
        self.uav_ns = rospy.get_param('~uav_ns', '/uav1')
        self.request_rate = rospy.get_param('~request_rate', True)
        self.track_state_topic = rospy.get_param(
            '~track_gimbal_state_topic',
            f'{self.uav_ns.rstrip("/")}/track/gimbal_state')
        self.track_yaw_sign = float(rospy.get_param('~track_yaw_sign', 1.0))
        # This script's existing output uses pitch<0 for looking down, while
        # xd_uav_track uses pitch_rad>0 for looking down.
        self.track_pitch_sign = float(rospy.get_param('~track_pitch_sign', -1.0))
        self.track_roll_sign = float(rospy.get_param('~track_roll_sign', 1.0))

        self.parser = mavlink2.MAVLink(None)
        self.mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=190)
        self.pub = rospy.Publisher(
            f'{self.uav_ns}/gimbal/attitude', Float64MultiArray, queue_size=10)
        self.track_state_pub = rospy.Publisher(
            self.track_state_topic, TrackGimbalState, queue_size=10)
        rospy.Subscriber(f'{self.uav_ns}/mavlink/from', Mavlink,
                         self._cb, queue_size=100)
        # 飞机航向：YAW_LOCK 时 #285 yaw 是世界系，须减航向转机体相对
        self._hdg = None
        self._rate_request_started = False
        rospy.Subscriber(f'{self.uav_ns}/mavros/global_position/compass_hdg',
                         Float64, self._hdg_cb, queue_size=10)
        rospy.loginfo('[gimbal_att] MAVLink #285 -> %s (yaw/pitch/roll in rad)',
                      self.track_state_topic)

    def _hdg_cb(self, msg):
        self._hdg = float(msg.data)

        if self.request_rate and not self._rate_request_started:
            self._pub_rate_req = rospy.Publisher(
                f'{self.uav_ns}/mavlink/to', Mavlink, queue_size=10)
            rospy.Timer(rospy.Duration(2.0), self._request_stream_rate,
                        oneshot=True)
            self._rate_request_started = True

    def _request_stream_rate(self, _event):
        """发 SET_MESSAGE_INTERVAL 请求 #285@10Hz(100000us)，target 1/154。
        仿真器流控可能按连接生效，mavros 侧无 QGC 时不保证持续推 #285。"""
        try:
            m = self.mav.command_long_encode(
                GIMBAL_SYSID, GIMBAL_COMPID,
                MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                MSG_GIMBAL_DEVICE_ATTITUDE_STATUS,  # param1 = msg_id
                100000.0, 0, 0, 0, 0, 0)            # param2 = interval_us
            self._pub_rate_req.publish(buf_to_mavlink_msg(m.pack(self.mav)))
        except Exception as e:
            rospy.logwarn_throttle(10.0, '[gimbal_att] 速率请求编码失败: %s', e)

    def _cb(self, msg):
        if msg.msgid != MSG_GIMBAL_DEVICE_ATTITUDE_STATUS:
            return
        try:
            parsed = self.parser.parse_buffer(rebuild_frame(msg))
        except Exception:
            return
        if not parsed:
            return
        for m in parsed:
            if m.get_msgId() != MSG_GIMBAL_DEVICE_ATTITUDE_STATUS:
                continue
            q = list(m.q)  # [w, x, y, z]
            if not all(math.isfinite(float(v)) for v in q):
                continue
            norm = math.sqrt(sum(v * v for v in q))
            if not math.isfinite(norm) or norm <= 0.5:
                continue
            w, x, y, z = [v / norm for v in q]
            roll = math.degrees(math.atan2(2 * (w * x + y * z),
                                           1 - 2 * (x * x + y * y)))
            # pitch 沿用 #285 原始约定：负值 = 低头(下视)，正值 = 抬头
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
            yaw = math.degrees(math.atan2(2 * (w * z + x * y),
                                          1 - 2 * (y * y + z * z)))
            flags = int(m.flags)
            track_yaw_valid = True
            if flags & 16:  # YAW_LOCK：yaw 在世界系 → 减飞机航向转机体相对
                if self._hdg is None:
                    rospy.logwarn_throttle(
                        5.0, '[gimbal_att] 航向未就绪, yaw 暂按世界系输出')
                    # Track consumes body-relative gimbal yaw. Do not mark a
                    # world-relative value as a valid body-relative sample.
                    track_yaw_valid = False
                else:
                    yaw = (yaw - self._hdg + 180.0) % 360.0 - 180.0  # wrap ±180
            # 稳像时 roll≈0，浮点尾渣(e-05量级)归零
            if abs(roll) < 1e-3:
                roll = 0.0
            if abs(pitch) < 1e-3:
                pitch = 0.0
            arr = Float64MultiArray()
            arr.data = [yaw, pitch, roll]
            self.pub.publish(arr)

            track_state = TrackGimbalState()
            track_state.header.stamp = (
                msg.header.stamp if not msg.header.stamp.is_zero()
                else rospy.Time.now())
            track_state.header.frame_id = f'{self.uav_ns.strip("/")}/gimbal'
            track_state.yaw_rad = self.track_yaw_sign * math.radians(yaw)
            track_state.pitch_rad = self.track_pitch_sign * math.radians(pitch)
            track_state.roll_rad = self.track_roll_sign * math.radians(roll)
            track_state.valid = track_yaw_valid
            self.track_state_pub.publish(track_state)


if __name__ == '__main__':
    GimbalAttitudeBridge()
    rospy.spin()
