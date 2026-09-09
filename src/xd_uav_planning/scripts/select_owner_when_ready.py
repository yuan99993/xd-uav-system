#!/usr/bin/env python3
"""Request explicit EGO ownership at the first healthy candidate boundary."""

import sys
import time

import rospy
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def main():
    rospy.init_node("select_owner_when_ready", anonymous=True)
    healthy = {"value": False, "wall_stamp": 0.0}
    candidate = {"wall_stamp": 0.0}
    rospy.Subscriber(
        rospy.get_param("~health_topic", "/uav1/planning/healthy"),
        Bool,
        lambda message: healthy.update(
            value=bool(message.data), wall_stamp=time.monotonic()),
        queue_size=1)
    rospy.Subscriber(
        rospy.get_param("~candidate_topic", "/uav1/ego/reference_candidate"),
        PositionTarget,
        lambda _message: candidate.update(wall_stamp=time.monotonic()),
        queue_size=1)
    service_name = rospy.get_param(
        "~service", "/uav1/reference_mux/select_ego")
    timeout = float(rospy.get_param("~timeout", 15.0))
    if timeout > 0.0:
        rospy.wait_for_service(service_name, timeout=min(3.0, timeout))
    else:
        rospy.loginfo("waiting indefinitely for owner-selection service")
        rospy.wait_for_service(service_name)
    select = rospy.ServiceProxy(service_name, SetBool, persistent=True)
    deadline = time.monotonic() + timeout if timeout > 0.0 else None
    rate = rospy.Rate(100)
    last_message = "not ready"
    while (not rospy.is_shutdown() and
           (deadline is None or time.monotonic() < deadline)):
        now = time.monotonic()
        if (healthy["value"] and now - healthy["wall_stamp"] < 0.30 and
                now - candidate["wall_stamp"] < 0.20):
            try:
                response = select(True)
                last_message = response.message
                if response.success:
                    rospy.loginfo("explicit owner selection succeeded: %s",
                                  response.message)
                    return 0
            except rospy.ServiceException as error:
                last_message = str(error)
        rate.sleep()
    rospy.logerr("explicit owner selection timed out: %s", last_message)
    return 2


if __name__ == "__main__":
    sys.exit(main())
