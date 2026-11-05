#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_accuracy —— 把解算结果和仿真真值比对，打印水平/垂直误差统计。

用法（配合 sim_target_sensor.py）：
    rosrun target_geolocation check_accuracy.py

误差定义：
    水平误差 = 估计点与真值点的 ENU 水平距离 (m)
    垂直误差 = |估计高度 - 真值高度| (m)
    三维误差 = 两者的合成
"""

import math

import rospy
from sensor_msgs.msg import NavSatFix

from target_geolocation import geodesy


class AccuracyChecker(object):

    def __init__(self):
        rospy.init_node('target_geolocation_accuracy')

        self.truth_topic = rospy.get_param('~truth_topic', '/sim/target_truth')
        self.est_topic = rospy.get_param('~est_topic', '/uav1/target/global_position')
        self.print_every = int(rospy.get_param('~print_every', 20))
        self.warmup = int(rospy.get_param('~warmup', 5))   # 跳过前几帧(缓存未填满)

        self._truth = None
        self._horiz = []
        self._vert = []

        rospy.Subscriber(self.truth_topic, NavSatFix, self._truth_cb, queue_size=5)
        rospy.Subscriber(self.est_topic, NavSatFix, self._est_cb, queue_size=5)
        rospy.on_shutdown(self._summary)
        rospy.loginfo('[ACC] 比对 %s (估计) vs %s (真值)', self.est_topic, self.truth_topic)

    def _truth_cb(self, msg):
        self._truth = (msg.latitude, msg.longitude, msg.altitude)

    def _est_cb(self, msg):
        if self._truth is None:
            return
        tlat, tlon, talt = self._truth
        h = geodesy.horizontal_distance(msg.latitude, msg.longitude, tlat, tlon)
        v = abs(msg.altitude - talt)
        self._horiz.append(h)
        self._vert.append(v)

        if len(self._horiz) < self.warmup:
            return
        if len(self._horiz) % self.print_every == 0:
            rospy.loginfo('[ACC] n=%d  水平 mean=%.2f p95=%.2f max=%.2f m | '
                          '垂直 mean=%.2f max=%.2f m',
                          len(self._horiz),
                          _mean(self._horiz), _p95(self._horiz), max(self._horiz),
                          _mean(self._vert), max(self._vert))

    def _summary(self):
        if len(self._horiz) < 2:
            rospy.logwarn('[ACC] 样本不足(%d)，无法统计', len(self._horiz))
            return
        h, v = self._horiz[self.warmup:], self._vert[self.warmup:]
        if not h:
            return
        rospy.loginfo('[ACC] ===== 汇总 n=%d =====', len(h))
        rospy.loginfo('[ACC] 水平误差: mean=%.2f m  p95=%.2f m  max=%.2f m',
                      _mean(h), _p95(h), max(h))
        rospy.loginfo('[ACC] 垂直误差: mean=%.2f m  p95=%.2f m  max=%.2f m',
                      _mean(v), _p95(v), max(v))
        rospy.loginfo('[ACC] 三维误差: mean=%.2f m  max=%.2f m',
                      _mean([math.hypot(a, b) for a, b in zip(h, v)]),
                      max([math.hypot(a, b) for a, b in zip(h, v)]))


def _mean(xs):
    return sum(xs) / float(len(xs))


def _p95(xs):
    s = sorted(xs)
    idx = min(len(s) - 1, int(0.95 * len(s)))
    return s[idx]


def main():
    try:
        AccuracyChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
