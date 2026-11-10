#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sim_target_sensor —— 在当前仿真系统里合成"共轴激光测距 + 云台指向目标"的数据。

为什么需要它
------------
本仿真(core_sim)里有云台姿态(#285)和飞机 GPS，但通常没有"目标被激光测距仪
照到、且云台正好指着目标"这种组合。为了在没有真机的情况下把整条链路跑通并
量化精度，这个节点：

  1. 订阅飞机 GPS，按参数在飞机附近生成一个**目标真值点**（可做圆周运动）；
  2. 由 飞机->目标 的 ENU 向量正算出"云台恰好指向目标"的姿态四元数
     （世界系, NED，等价于 YAW_LOCK 已置位的 #285），加一点角度噪声后发布；
  3. 斜距 = 三维距离 + 噪声/偏置，按 sensor_msgs/Range 发布；
  4. 目标真值同时以 NavSatFix 发布，供 check_accuracy.py 比对。

注意：这里用的是**正向模型**（由几何构造姿态），解算节点用的是**反向模型**
（由姿态还原几何），两者的符号约定互相独立，因此能真正暴露约定错误；如果是
同一个函数正反各调一次，符号错了也会"自洽"通过。

发布
----
    ~/range_topic      (sensor_msgs/Range)            斜距
    ~/gimbal_att_topic (geometry_msgs/QuaternionStamped) 云台姿态(世界系)
    ~/truth_topic      (sensor_msgs/NavSatFix)        目标真值
"""

import math
import random

import rospy
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import NavSatFix, Range

from target_geolocation import geodesy, solver


class SimTargetSensor(object):

    def __init__(self):
        rospy.init_node('sim_target_sensor')

        self.uav_ns = rospy.get_param('~uav_ns', '/uav1').rstrip('/')
        self.gps_topic = rospy.get_param(
            '~gps_topic', self.uav_ns + '/mavros/global_position/global')
        self.range_topic = rospy.get_param('~range_topic', '/sim/range')
        self.gimbal_att_topic = rospy.get_param('~gimbal_att_topic',
                                                '/sim/gimbal/attitude')
        self.truth_topic = rospy.get_param('~truth_topic', '/sim/target_truth')

        # 目标初始位置：以飞机为基准的方位/距离/相对高度，首次拿到 GPS 时固定下来
        self.truth_bearing_deg = float(rospy.get_param('~truth_bearing_deg', 60.0))
        self.truth_distance_m = float(rospy.get_param('~truth_distance_m', 120.0))
        self.truth_height_agl = float(rospy.get_param('~truth_height_agl', 100.0))

        # 目标圆周运动（0 = 静止）
        self.motion_radius_m = float(rospy.get_param('~motion_radius_m', 0.0))
        self.motion_period_s = float(rospy.get_param('~motion_period_s', 40.0))

        # 传感器噪声
        self.range_noise_std = float(rospy.get_param('~range_noise_std', 0.05))
        self.range_bias = float(rospy.get_param('~range_bias', 0.0))
        self.att_noise_deg = float(rospy.get_param('~att_noise_deg', 0.10))
        self.rate = float(rospy.get_param('~rate', 20.0))

        self._uav = None
        self._center = None       # 目标运动圆心 (lat, lon, alt)
        self._t0 = None

        rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb, queue_size=5)
        self.pub_range = rospy.Publisher(self.range_topic, Range, queue_size=10)
        self.pub_att = rospy.Publisher(self.gimbal_att_topic, QuaternionStamped,
                                       queue_size=10)
        self.pub_truth = rospy.Publisher(self.truth_topic, NavSatFix, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / max(self.rate, 1.0)), self._tick)
        rospy.loginfo('[SIM] 合成传感器: range=%s att=%s truth=%s',
                      self.range_topic, self.gimbal_att_topic, self.truth_topic)

    # ------------------------------------------------------------------ 回调
    def _gps_cb(self, msg):
        if not math.isfinite(msg.latitude) or (msg.latitude == 0.0 and msg.longitude == 0.0):
            return
        self._uav = (msg.latitude, msg.longitude, msg.altitude)
        if self._center is None:
            # 用第一帧飞机位置定义目标真值，这样换仿真地图/起飞点也不用改参数
            e = self.truth_distance_m * math.sin(math.radians(self.truth_bearing_deg))
            n = self.truth_distance_m * math.cos(math.radians(self.truth_bearing_deg))
            lat, lon, alt = geodesy.enu_to_geodetic(
                msg.latitude, msg.longitude, msg.altitude,
                e, n, -self.truth_height_agl)
            self._center = (lat, lon, alt)
            self._t0 = rospy.Time.now()
            rospy.loginfo('[SIM] 目标真值锚定在 %.7f, %.7f, %.1f（距飞机 %.0f m / %.0f°，'
                          '低 %.0f m）', lat, lon, alt,
                          self.truth_distance_m, self.truth_bearing_deg,
                          self.truth_height_agl)

    # ------------------------------------------------------------------ 主循环
    def _truth_now(self):
        lat0, lon0, alt0 = self._center
        if self.motion_radius_m <= 0.0:
            return lat0, lon0, alt0
        phase = 2.0 * math.pi * (rospy.Time.now() - self._t0).to_sec() / max(
            self.motion_period_s, 1e-3)
        e = self.motion_radius_m * math.sin(phase)
        n = self.motion_radius_m * math.cos(phase)
        return geodesy.enu_to_geodetic(lat0, lon0, alt0, e, n, 0.0)

    def _tick(self, _event):
        if self._uav is None or self._center is None:
            rospy.loginfo_throttle(3.0, '[SIM] 等待飞机 GPS…')
            return

        uav_lat, uav_lon, uav_alt = self._uav
        t_lat, t_lon, t_alt = self._truth_now()

        east, north = geodesy.geodetic_to_enu(uav_lat, uav_lon, t_lat, t_lon)
        up = t_alt - uav_alt
        slant = math.sqrt(east * east + north * north + up * up)
        if slant < 1.0:
            return

        # ---- 正向模型：构造"恰好指向目标"的世界系云台姿态（NED/FRD, ZYX） ----
        #   NED 方向 = (north, east, -up)；yaw = atan2(E, N)，pitch = asin(up/L)
        yaw_deg = math.degrees(math.atan2(east, north))
        pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, up / slant))))
        yaw_deg += random.gauss(0.0, self.att_noise_deg)
        pitch_deg += random.gauss(0.0, self.att_noise_deg)

        q = solver.quat_from_euler_zyx(0.0, math.radians(pitch_deg),
                                       math.radians(yaw_deg))
        stamp = rospy.Time.now()

        att = QuaternionStamped()
        att.header.stamp = stamp
        att.header.frame_id = 'world'
        att.quaternion.w, att.quaternion.x, att.quaternion.y, att.quaternion.z = q
        self.pub_att.publish(att)

        rng = Range()
        rng.header.stamp = stamp
        rng.header.frame_id = 'gimbal_laser'
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = 0.02
        rng.min_range = 0.2
        rng.max_range = 5000.0
        rng.range = max(0.2, slant + self.range_bias
                        + random.gauss(0.0, self.range_noise_std))
        self.pub_range.publish(rng)

        truth = NavSatFix()
        truth.header.stamp = stamp
        truth.header.frame_id = 'target'
        truth.latitude, truth.longitude, truth.altitude = t_lat, t_lon, t_alt
        truth.status.status = 0
        truth.status.service = 1
        self.pub_truth.publish(truth)

        rospy.loginfo_throttle(
            5.0, '[SIM] 目标真值 %.7f,%.7f,%.1f | 斜距 %.1f m 方位 %.1f° 仰角 %.1f°',
            t_lat, t_lon, t_alt, slant, yaw_deg, pitch_deg)


def main():
    try:
        SimTargetSensor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
