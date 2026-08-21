#!/usr/bin/env python3
"""Issue one manager takeoff request after the guarded stack is ready."""

import time

import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from xd_uav_controller.srv import Takeoff


class AutoTakeoff:
    def __init__(self):
        self._state = None
        self._valid = False
        self._valid_wall = 0.0
        self._stable_since = None
        self._ready_duration = float(rospy.get_param("~ready_duration", 2.0))
        self._altitude = float(rospy.get_param("~altitude", 1.0))
        self._timeout = float(rospy.get_param("~timeout", 90.0))
        rospy.Subscriber(rospy.get_param("~mavros_state_topic", "mavros/state"),
                         State, self._state_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~state_valid_topic",
                                         "state_estimator/state_valid"),
                         Bool, self._valid_cb, queue_size=1)

    def _state_cb(self, message):
        self._state = message

    def _valid_cb(self, message):
        self._valid = bool(message.data)
        self._valid_wall = time.monotonic()

    def run(self):
        service_name = rospy.get_param("~takeoff_service", "control_manager/takeoff")
        started = time.monotonic()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            now = time.monotonic()
            fresh_valid = self._valid and now - self._valid_wall < 0.5
            ready = (self._state is not None and self._state.connected and
                     not self._state.armed and fresh_valid)
            if ready:
                if self._stable_since is None:
                    self._stable_since = now
                if now - self._stable_since >= self._ready_duration:
                    break
            else:
                self._stable_since = None
            if now - started >= self._timeout:
                rospy.logerr("auto takeoff timed out waiting for MAVROS and estimator")
                return 2
            rate.sleep()

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
            takeoff = rospy.ServiceProxy(service_name, Takeoff, persistent=True)
        except rospy.ROSException as error:
            rospy.logerr("auto takeoff service unavailable: %s", error)
            return 2
        while not rospy.is_shutdown() and time.monotonic() - started < self._timeout:
            try:
                response = takeoff(self._altitude)
            except rospy.ServiceException as error:
                rospy.logwarn("auto takeoff service retry: %s", error)
                rospy.sleep(1.0)
                continue
            if response.success:
                rospy.loginfo("auto takeoff accepted at %.2f m: %s",
                              self._altitude, response.message)
                return 0
            rospy.logwarn("auto takeoff not ready yet: %s", response.message)
            rospy.sleep(1.0)
        rospy.logerr("auto takeoff timed out after manager rejections")
        return 2


if __name__ == "__main__":
    rospy.init_node("auto_takeoff_when_ready")
    raise SystemExit(AutoTakeoff().run())
