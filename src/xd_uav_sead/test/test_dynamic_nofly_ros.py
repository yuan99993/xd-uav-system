#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point32

from xd_uav_sead.airspace.airspace_manager import AirspaceManager
from xd_uav_sead.airspace.dynamic_nofly import (
    DynamicNoFlyConfig,
    DynamicNoFlyZoneReceiver,
)
from xd_uav_sead.msg import NoFlyZone


class DynamicNoFlyRosInterfaceTest(unittest.TestCase):
    def test_ros_serialization_and_callback(self):
        airspace = AirspaceManager()
        receiver = DynamicNoFlyZoneReceiver(
            airspace, DynamicNoFlyConfig(expected_frame="uav1/odom")
        )
        callback_seen = threading.Event()

        def callback(message):
            receiver.accept(message, rospy.Time.now().to_sec())
            callback_seen.set()

        subscriber = rospy.Subscriber(
            "/test/dynamic_nofly_zone", NoFlyZone, callback, queue_size=1
        )
        publisher = rospy.Publisher(
            "/test/dynamic_nofly_zone", NoFlyZone, queue_size=1, latch=True
        )
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while publisher.get_num_connections() < 1 and rospy.Time.now() < deadline:
            rospy.sleep(0.02)
        self.assertGreaterEqual(publisher.get_num_connections(), 1)

        now = rospy.Time.now()
        message = NoFlyZone()
        message.header.stamp = now
        message.header.frame_id = "uav1/odom"
        message.schema_version = NoFlyZone.CURRENT_SCHEMA_VERSION
        message.operation = NoFlyZone.OP_UPSERT
        message.zone_id = 42
        message.enabled = True
        message.zone_type = NoFlyZone.TYPE_NO_FLY
        message.min_altitude = 20.0
        message.max_altitude = 120.0
        message.valid_until = now + rospy.Duration(10.0)
        message.polygon.points = [
            Point32(x=80.0, y=-30.0),
            Point32(x=120.0, y=-30.0),
            Point32(x=120.0, y=30.0),
            Point32(x=80.0, y=30.0),
        ]
        publisher.publish(message)
        self.assertTrue(callback_seen.wait(3.0))
        event = receiver.pop_event()
        self.assertIsNotNone(event)
        self.assertTrue(event.accepted)
        self.assertIn(42, airspace.zones)
        subscriber.unregister()
        publisher.unregister()


if __name__ == "__main__":
    rospy.init_node("test_dynamic_nofly_ros", anonymous=True)
    rostest.rosrun(
        "xd_uav_sead",
        "test_dynamic_nofly_ros",
        DynamicNoFlyRosInterfaceTest,
    )
