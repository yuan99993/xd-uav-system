#!/usr/bin/env python3
"""End-to-end PX4 fixed-wing acceptance for the planning Path backend."""

import json
import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamGet, ParamSet, ParamSetRequest
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from xd_uav_controller.msg import ControlState
from xd_uav_controller.srv import Takeoff
from xd_uav_task_allocate.msg import PlannerStatus


class AcceptanceFailure(RuntimeError):
    pass


class FixedwingPathAcceptance:
    def __init__(self):
        self.uav_name = rospy.get_param("~uav_name", "uav1")
        self.namespace = "/" + self.uav_name
        self.frame_id = self.uav_name + "/odom"
        self.takeoff_altitude = float(rospy.get_param("~takeoff_altitude", 30.0))
        self.path_length = float(rospy.get_param("~path_length", 180.0))
        self.goal_id = int(rospy.get_param("~goal_id", 9001))
        self.state = None
        self.control_state = None
        self.statuses = []
        self.start_position = None

        self.state_subscriber = rospy.Subscriber(
            self.namespace + "/mavros/state", State, self._state_callback)
        self.control_state_subscriber = rospy.Subscriber(
            self.namespace + "/control_manager/state", ControlState,
            self._control_state_callback)
        self.status_subscriber = rospy.Subscriber(
            self.namespace + "/planning/status", PlannerStatus,
            self.statuses.append)
        self.path_publisher = rospy.Publisher(
            self.namespace + "/planning/task_path", Path,
            queue_size=1, latch=True)
        self.result_publisher = rospy.Publisher(
            self.namespace + "/planning/fixedwing_acceptance/result",
            String, queue_size=1, latch=True)

    def _state_callback(self, message):
        self.state = message

    def _control_state_callback(self, message):
        self.control_state = message

    @staticmethod
    def _wait(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.1)
        raise AcceptanceFailure("timeout waiting for " + description)

    def _set_px4_param(self, name, value, integer):
        set_name = self.namespace + "/mavros/param/set"
        get_name = self.namespace + "/mavros/param/get"
        rospy.wait_for_service(set_name, timeout=15.0)
        rospy.wait_for_service(get_name, timeout=15.0)
        setter = rospy.ServiceProxy(set_name, ParamSet)
        getter = rospy.ServiceProxy(get_name, ParamGet)
        sync_deadline = time.monotonic() + 45.0
        while not rospy.is_shutdown() and time.monotonic() < sync_deadline:
            try:
                if getter(param_id=name).success:
                    break
            except rospy.ServiceException:
                pass
            time.sleep(1.0)
        else:
            raise AcceptanceFailure(
                "timeout waiting for PX4 parameter synchronization for " + name)
        for _ in range(10):
            try:
                request = ParamSetRequest()
                request.param_id = name
                if integer:
                    request.value.integer = int(value)
                else:
                    request.value.real = float(value)
                response = setter(request)
                time.sleep(0.25)
                actual = getter(param_id=name)
                actual_value = (actual.value.integer if integer
                                else actual.value.real)
                if (response.success and actual.success and
                        abs(float(actual_value) - float(value)) < 1e-3):
                    return
            except rospy.ServiceException as error:
                rospy.logwarn(
                    "[FIXEDWING_PLANNING_ACCEPTANCE] PX4 parameter %s is "
                    "not ready yet: %s", name, error)
            time.sleep(0.75)
        raise AcceptanceFailure("failed to configure PX4 parameter " + name)

    def _configure_headless_px4(self):
        self._set_px4_param("COM_RC_IN_MODE", 4, True)
        self._set_px4_param("COM_RCL_EXCEPT", 6, True)
        self._set_px4_param("NAV_DLL_ACT", 0, True)
        self._set_px4_param("SIM_BAT_DRAIN", 86400.0, False)
        self._set_px4_param("SIM_BAT_MIN_PCT", 100.0, False)

    def _path(self):
        state = self.control_state
        origin = state.position_odom
        course = float(state.course)
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        path.header.seq = self.goal_id
        for distance in (0.0, self.path_length * 0.5, self.path_length):
            pose = PoseStamped()
            pose.header.seq = self.goal_id
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = origin.x + distance * math.cos(course)
            pose.pose.position.y = origin.y + distance * math.sin(course)
            pose.pose.position.z = origin.z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.start_position = (origin.x, origin.y, origin.z)
        return path

    def _has_status(self, state):
        return any(message.goal_id == self.goal_id and message.state == state
                   for message in self.statuses)

    def _distance_flown(self):
        if self.start_position is None or self.control_state is None:
            return 0.0
        position = self.control_state.position_odom
        return math.sqrt(
            (position.x - self.start_position[0]) ** 2 +
            (position.y - self.start_position[1]) ** 2 +
            (position.z - self.start_position[2]) ** 2)

    def run(self):
        self._wait(
            lambda: self.state is not None and self.state.connected and
                    self.control_state is not None and
                    self.control_state.state_valid and
                    self.control_state.stable and
                    self.control_state.airspeed_valid,
            90.0, "MAVROS and stable fixed-wing control state")
        self._configure_headless_px4()

        takeoff_name = self.namespace + "/control_manager/takeoff"
        rospy.wait_for_service(takeoff_name, timeout=10.0)
        response = rospy.ServiceProxy(takeoff_name, Takeoff)(
            altitude=self.takeoff_altitude)
        if not response.success:
            raise AcceptanceFailure("takeoff rejected: " + response.message)
        self._wait(
            lambda: self.state.armed and self.state.mode == "OFFBOARD" and
                    self.control_state.position_odom.z >=
                    self.takeoff_altitude - 6.0 and
                    self.control_state.airspeed >= 8.0,
            90.0, "fixed-wing OFFBOARD takeoff")

        path = self._path()
        self._wait(lambda: self.path_publisher.get_num_connections() > 0,
                   10.0, "planning path subscriber")
        deadline = time.monotonic() + 10.0
        while (not self._has_status(PlannerStatus.ACTIVE) and
               time.monotonic() < deadline and not rospy.is_shutdown()):
            path.header.stamp = rospy.Time.now()
            for pose in path.poses:
                pose.header.stamp = path.header.stamp
            self.path_publisher.publish(path)
            time.sleep(0.2)
        if not self._has_status(PlannerStatus.ACTIVE):
            raise AcceptanceFailure("controller did not activate planning path")

        self._wait(lambda: self._has_status(PlannerStatus.REACHED), 60.0,
                   "fixed-wing path completion")
        distance = self._distance_flown()
        if distance < self.path_length * 0.60:
            raise AcceptanceFailure(
                "path completed without sufficient motion: %.1fm" % distance)

        land_name = self.namespace + "/control_manager/land"
        rospy.wait_for_service(land_name, timeout=10.0)
        landing = rospy.ServiceProxy(land_name, Trigger)()
        if not landing.success:
            raise AcceptanceFailure("landing rejected: " + landing.message)
        result = json.dumps({
            "success": True,
            "goal_id": self.goal_id,
            "distance_m": round(distance, 1),
            "landing_accepted": True,
        }, sort_keys=True)
        rospy.loginfo("[FIXEDWING_PLANNING_ACCEPTANCE] %s", result)
        self.result_publisher.publish(String(data=result))


def main():
    rospy.init_node("fixedwing_path_acceptance")
    acceptance = FixedwingPathAcceptance()
    try:
        acceptance.run()
        rospy.spin()
        return 0
    except Exception as error:  # noqa: broad exception reports acceptance failure
        rospy.logerr("[FIXEDWING_PLANNING_ACCEPTANCE] %s", error)
        acceptance.result_publisher.publish(String(data=json.dumps({
            "success": False, "error": str(error)}, sort_keys=True)))
        rospy.spin()
        return 1


if __name__ == "__main__":
    sys.exit(main())
