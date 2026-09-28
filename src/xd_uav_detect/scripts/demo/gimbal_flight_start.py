#!/usr/bin/env python3
"""Bring the optional MRS flight demo from preflight checks to takeoff."""

import time
import threading

import rospy
from mrs_msgs.msg import ControlManagerDiagnostics, HwApiStatus
from std_srvs.srv import SetBool, Trigger


class FlightStarter:
    def __init__(self):
        self._arming_name = rospy.get_param(
            "~arming_service", "/uav1/hw_api/arming")
        self._offboard_name = rospy.get_param(
            "~offboard_service", "/uav1/hw_api/offboard")
        self._takeoff_name = rospy.get_param(
            "~takeoff_service", "/uav1/uav_manager/takeoff")
        self._timeout = float(rospy.get_param("~timeout", 90.0))
        self._lock = threading.Lock()
        self._hw_status = None
        self._control_status = None
        rospy.Subscriber("/uav1/hw_api/status", HwApiStatus,
                         self._hw_status_callback, queue_size=1)
        rospy.Subscriber("/uav1/control_manager/diagnostics",
                         ControlManagerDiagnostics,
                         self._control_status_callback, queue_size=1)

    def _hw_status_callback(self, message):
        with self._lock:
            self._hw_status = message

    def _control_status_callback(self, message):
        with self._lock:
            self._control_status = message

    def _wait_for(self, predicate, deadline, description):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate(self._hw_status, self._control_status):
                    return
            rospy.loginfo_throttle(5.0, "[gimbal_flight] waiting for %s",
                                   description)
            rate.sleep()
        raise RuntimeError("timed out waiting for " + description)

    @staticmethod
    def _call_until_accepted(proxy, request, deadline, operation):
        rate = rospy.Rate(2)
        last_message = "service not called"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                response = proxy(request) if request is not None else proxy()
                last_message = response.message
                if response.success:
                    rospy.loginfo("[gimbal_flight] %s accepted: %s",
                                  operation, response.message)
                    return
            except rospy.ServiceException as error:
                last_message = str(error)
            rospy.loginfo_throttle(
                5.0, "[gimbal_flight] waiting for %s: %s",
                operation, last_message)
            rate.sleep()
        raise RuntimeError("{} timed out: {}".format(operation, last_message))

    def run(self):
        deadline = time.monotonic() + self._timeout
        for service in (self._arming_name, self._offboard_name,
                        self._takeoff_name):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError("flight services did not become ready")
            rospy.wait_for_service(service, timeout=remaining)

        arm = rospy.ServiceProxy(self._arming_name, SetBool, persistent=True)
        offboard = rospy.ServiceProxy(
            self._offboard_name, Trigger, persistent=True)
        takeoff = rospy.ServiceProxy(
            self._takeoff_name, Trigger, persistent=True)
        self._wait_for(
            lambda hw, control: hw is not None and hw.connected,
            deadline, "connected HW API")
        self._call_until_accepted(arm, True, deadline, "arming")
        self._wait_for(
            lambda hw, control: hw is not None and hw.armed,
            deadline, "armed state")
        self._wait_for(
            lambda hw, control: (control is not None and
                                 control.output_enabled),
            deadline, "MRS control output")
        with self._lock:
            still_armed = self._hw_status is not None and self._hw_status.armed
        if not still_armed:
            self._call_until_accepted(arm, True, deadline, "re-arming")
            self._wait_for(
                lambda hw, control: hw is not None and hw.armed,
                deadline, "re-armed state")
        self._call_until_accepted(offboard, None, deadline, "offboard")
        self._wait_for(
            lambda hw, control: hw is not None and hw.offboard,
            deadline, "OFFBOARD state")
        self._call_until_accepted(takeoff, None, deadline, "takeoff")


if __name__ == "__main__":
    rospy.init_node("gimbal_flight_start")
    try:
        FlightStarter().run()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        rospy.logfatal("[gimbal_flight] %s", error)
        raise
