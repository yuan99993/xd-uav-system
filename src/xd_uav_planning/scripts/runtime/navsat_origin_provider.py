#!/usr/bin/env python3
"""Latch a stable vehicle-local geodetic origin when MAVROS omits gp_origin."""

from collections import deque
import math

import rospy
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus


class NavSatOriginProvider:
    def __init__(self):
        self._required_samples = int(rospy.get_param("~stable_samples", 5))
        self._horizontal_tolerance = float(rospy.get_param("~horizontal_tolerance_m", 1.0))
        self._vertical_tolerance = float(rospy.get_param("~vertical_tolerance_m", 1.0))
        if self._required_samples < 1:
            raise ValueError("stable_samples must be positive")
        self._samples = deque(maxlen=self._required_samples)
        self._origin = None
        self._publisher = rospy.Publisher(
            rospy.get_param("~output_topic", "integration/geodetic_origin"),
            GeoPointStamped, queue_size=1, latch=True)
        rospy.Subscriber(
            rospy.get_param("~input_topic", "mavros/global_position/global"),
            NavSatFix, self._callback, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(1.0), self._publish)

    @staticmethod
    def _valid(message):
        return (message.status.status >= NavSatStatus.STATUS_FIX and
                all(math.isfinite(value) for value in
                    (message.latitude, message.longitude, message.altitude)) and
                -90.0 <= message.latitude <= 90.0 and
                -180.0 <= message.longitude <= 180.0)

    def _stable(self):
        if len(self._samples) < self._required_samples:
            return False
        reference = self._samples[0]
        latitude_scale = 111320.0
        longitude_scale = latitude_scale * math.cos(math.radians(reference.latitude))
        for sample in self._samples:
            north = (sample.latitude - reference.latitude) * latitude_scale
            east = (sample.longitude - reference.longitude) * longitude_scale
            if (math.hypot(east, north) > self._horizontal_tolerance or
                    abs(sample.altitude - reference.altitude) > self._vertical_tolerance):
                return False
        return True

    def _callback(self, message):
        if self._origin is not None or not self._valid(message):
            return
        self._samples.append(message)
        if not self._stable():
            return
        self._origin = GeoPointStamped()
        self._origin.header.frame_id = message.header.frame_id
        self._origin.position.latitude = sum(sample.latitude for sample in self._samples) / len(self._samples)
        self._origin.position.longitude = sum(sample.longitude for sample in self._samples) / len(self._samples)
        self._origin.position.altitude = sum(sample.altitude for sample in self._samples) / len(self._samples)
        rospy.loginfo("latched vehicle geodetic origin from %d stable fixes", len(self._samples))
        self._publish(None)

    def _publish(self, _event):
        if self._origin is None:
            return
        self._origin.header.stamp = rospy.Time.now()
        self._publisher.publish(self._origin)


if __name__ == "__main__":
    rospy.init_node("navsat_origin_provider")
    NavSatOriginProvider()
    rospy.spin()
