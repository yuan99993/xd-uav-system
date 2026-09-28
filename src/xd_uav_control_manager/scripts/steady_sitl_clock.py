#!/usr/bin/env python3

"""Publish a realtime-rate ROS clock backed by CLOCK_MONOTONIC.

This is only for ArduPilot's built-in SITL physics, which has no /clock topic.
The epoch is sampled once; subsequent clock progress is immune to host wall
clock steps (notably WSL synchronizing its clock with Windows).
"""

import time

import rospy
from rosgraph_msgs.msg import Clock


def main():
    rospy.init_node("steady_sitl_clock")
    publish_rate = float(rospy.get_param("~publish_rate", 100.0))
    if publish_rate <= 0.0:
        raise ValueError("~publish_rate must be greater than zero")

    publisher = rospy.Publisher("/clock", Clock, queue_size=1)
    epoch = time.time()
    monotonic_origin = time.monotonic()
    period = 1.0 / publish_rate
    next_publish = monotonic_origin

    while not rospy.is_shutdown():
        monotonic_now = time.monotonic()
        message = Clock()
        message.clock = rospy.Time.from_sec(
            epoch + monotonic_now - monotonic_origin)
        publisher.publish(message)

        next_publish += period
        sleep_duration = next_publish - time.monotonic()
        if sleep_duration > 0.0:
            time.sleep(sleep_duration)
        else:
            next_publish = time.monotonic()


if __name__ == "__main__":
    main()
