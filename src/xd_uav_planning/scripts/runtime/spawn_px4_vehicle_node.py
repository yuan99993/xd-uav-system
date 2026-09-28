#!/usr/bin/env python3
"""One-shot adapter from roslaunch lifecycle to the MRS spawn service."""

import sys

import rospy
from mrs_msgs.srv import String, StringRequest


def main():
    rospy.init_node("spawn_px4_vehicle")
    service_name = rospy.get_param("~service", "/mrs_drone_spawner/spawn")
    request_value = rospy.get_param("~request", "1 --x500")
    timeout = float(rospy.get_param("~timeout", 30.0))
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        response = rospy.ServiceProxy(service_name, String)(
            StringRequest(value=request_value))
    except (rospy.ROSException, rospy.ServiceException) as error:
        rospy.logfatal("PX4 vehicle spawn failed: %s", error)
        return 2
    if not response.success:
        rospy.logfatal("PX4 vehicle spawn rejected: %s", response.message)
        return 2
    rospy.loginfo("PX4 vehicle spawn queued: %s", response.message)
    return 0


if __name__ == "__main__":
    sys.exit(main())
