#!/usr/bin/env python3
"""M6 Web-to-rosbridge-to-terrain three-PX4 acceptance orchestration."""

import json
import math
import time
from urllib import request as http_request
from urllib.parse import urlparse

import rospy
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import ParamGet, ParamSet, ParamSetRequest
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from xd_uav_controller.msg import ControlState
from xd_uav_controller.srv import Takeoff
from xd_uav_planning.msg import TerrainSwarmStatus
from xd_uav_task_allocate.msg import PlannerStatus


class AcceptanceFailure(RuntimeError):
    pass


class TerrainFixedwingSwarmAcceptance:
    def __init__(self):
        self.routes = rospy.get_param("~routes")
        self.web_api_url = rospy.get_param(
            "~web_api_url", "http://127.0.0.1:8000/api/swarm/dispatch")
        self.states = {}
        self.extended_states = {}
        self.control_states = {}
        self.planner_statuses = {}
        self.swarm_status = None
        self.release_origins = {}
        self.release = rospy.Publisher(
            "/terrain_swarm/release", Bool, queue_size=1, latch=True)
        self.result = rospy.Publisher(
            "/terrain_swarm/sitl_acceptance/result", String,
            queue_size=1, latch=True)
        self.subscribers = [rospy.Subscriber(
            "/terrain_swarm/status", TerrainSwarmStatus,
            self._swarm_callback, queue_size=5)]
        names = set()
        for route in self.routes:
            name = str(route["uav_name"])
            if name in names:
                raise AcceptanceFailure("duplicate vehicle " + name)
            names.add(name)
            namespace = "/" + name
            self.subscribers.extend([
                rospy.Subscriber(
                    namespace + "/mavros/state", State,
                    lambda message, key=name: self.states.__setitem__(key, message)),
                rospy.Subscriber(
                    namespace + "/mavros/extended_state", ExtendedState,
                    lambda message, key=name: self.extended_states.__setitem__(
                        key, message)),
                rospy.Subscriber(
                    namespace + "/control_manager/state", ControlState,
                    lambda message, key=name: self.control_states.__setitem__(key, message)),
                rospy.Subscriber(
                    namespace + "/planning/status", PlannerStatus,
                    lambda message, key=name: self.planner_statuses.__setitem__(key, message)),
            ])
        self.names = sorted(names)
        self.release.publish(Bool(data=False))

    def _swarm_callback(self, message):
        self.swarm_status = message

    @staticmethod
    def _wait(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.1)
        raise AcceptanceFailure("timeout waiting for " + description)

    def _stable(self, name):
        state = self.states.get(name)
        control = self.control_states.get(name)
        return bool(state and state.connected and control and
                    control.state_valid and control.stable and
                    control.airspeed_valid)

    @staticmethod
    def _set_px4_param(namespace, name, value, integer):
        set_name = namespace + "/mavros/param/set"
        get_name = namespace + "/mavros/param/get"
        rospy.wait_for_service(set_name, timeout=20.0)
        rospy.wait_for_service(get_name, timeout=20.0)
        setter = rospy.ServiceProxy(set_name, ParamSet)
        getter = rospy.ServiceProxy(get_name, ParamGet)
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                if getter(param_id=name).success:
                    break
            except rospy.ServiceException:
                pass
            time.sleep(1.0)
        else:
            raise AcceptanceFailure("PX4 parameter sync timeout: " + name)
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
                        abs(float(actual_value) - float(value)) < 1.0e-3):
                    return
            except rospy.ServiceException as error:
                rospy.logwarn(
                    "[TERRAIN_SWARM_SITL_ACCEPTANCE] PX4 parameter %s is "
                    "not ready yet: %s", name, error)
            time.sleep(0.75)
        raise AcceptanceFailure("failed to set PX4 parameter " + name)

    def _configure_px4(self, name):
        namespace = "/" + name
        for param, value, integer in (
                ("COM_RC_IN_MODE", 4, True),
                ("COM_RCL_EXCEPT", 6, True),
                ("NAV_DLL_ACT", 0, True),
                ("SIM_BAT_DRAIN", 86400.0, False),
                ("SIM_BAT_MIN_PCT", 100.0, False)):
            self._set_px4_param(namespace, param, value, integer)

    def _dispatch_web(self):
        payload = {
            "frame_id": "world",
            "use_mock": False,
            "routes": [{
                "uav_name": route["uav_name"],
                "goal_id": int(route["goal_id"]),
                "nominal_speed_mps": float(route["nominal_speed_mps"]),
                "points": route["points"],
            } for route in self.routes],
        }
        body = json.dumps(payload).encode("utf-8")
        request = http_request.Request(
            self.web_api_url, data=body,
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            hostname = urlparse(self.web_api_url).hostname
            if hostname in ("127.0.0.1", "localhost", "::1"):
                opener = http_request.build_opener(http_request.ProxyHandler({}))
                response_context = opener.open(request, timeout=15.0)
            else:
                response_context = http_request.urlopen(request, timeout=15.0)
            with response_context as response:
                output = json.loads(response.read().decode("utf-8"))
        except Exception as error:
            raise AcceptanceFailure("Web dispatch failed: {}".format(error))
        if output.get("mode") != "ros" or len(output.get("dispatched", [])) != len(self.routes):
            raise AcceptanceFailure("Web did not confirm formal ROS dispatch")

    def _screen_ready(self):
        if not self.swarm_status:
            return False
        if not self.swarm_status.analysis_ready or not self.swarm_status.conflict_free:
            return False
        expected = {str(route["uav_name"]): int(route["goal_id"])
                    for route in self.routes}
        actual = {item.uav_name: int(item.planner_status.goal_id)
                  for item in self.swarm_status.vehicles
                  if item.path_available}
        return all(actual.get(name) == goal for name, goal in expected.items())

    def _airborne(self, route):
        name = str(route["uav_name"])
        state = self.states.get(name)
        control = self.control_states.get(name)
        altitude = float(route["takeoff_altitude_m"])
        return bool(state and state.armed and state.mode == "OFFBOARD" and
                    control and control.position_odom.z >= altitude - 6.0 and
                    control.airspeed >= 8.0)

    def _planner_state(self, name, state):
        status = self.planner_statuses.get(name)
        expected = next(int(route["goal_id"]) for route in self.routes
                        if route["uav_name"] == name)
        return bool(status and int(status.goal_id) == expected and
                    int(status.state) == state)

    def _motion(self, name):
        current = self.control_states[name].position_odom
        origin = self.release_origins[name]
        return math.sqrt((current.x - origin[0]) ** 2 +
                         (current.y - origin[1]) ** 2 +
                         (current.z - origin[2]) ** 2)

    def _land_all(self):
        accepted = {}
        for name in self.names:
            try:
                service = "/{}/control_manager/land".format(name)
                rospy.wait_for_service(service, timeout=10.0)
                response = rospy.ServiceProxy(service, Trigger)()
                accepted[name] = bool(response.success)
            except (rospy.ROSException, rospy.ServiceException):
                accepted[name] = False
        return accepted

    def _landed_and_connected(self, name):
        state = self.states.get(name)
        extended_state = self.extended_states.get(name)
        return bool(
            state and state.connected and not state.armed and
            extended_state and
            extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)

    def run(self):
        self._wait(lambda: all(self._stable(name) for name in self.names),
                   150.0, "all stable fixed-wing control states")
        for name in self.names:
            self._configure_px4(name)
        self._dispatch_web()
        self._wait(self._screen_ready, 45.0,
                   "conflict-free three-vehicle terrain planning result")

        for route in self.routes:
            service = "/{}/control_manager/takeoff".format(route["uav_name"])
            rospy.wait_for_service(service, timeout=10.0)
            response = rospy.ServiceProxy(service, Takeoff)(
                altitude=float(route["takeoff_altitude_m"]))
            if not response.success:
                raise AcceptanceFailure(
                    "{} takeoff rejected: {}".format(
                        route["uav_name"], response.message))
        self._wait(lambda: all(self._airborne(route) for route in self.routes),
                   150.0, "all OFFBOARD takeoffs")
        for name in self.names:
            position = self.control_states[name].position_odom
            self.release_origins[name] = (position.x, position.y, position.z)
        self._wait(lambda: self.release.get_num_connections() > 0,
                   10.0, "terrain release gate subscriber")
        self.release.publish(Bool(data=True))

        self._wait(lambda: all(self._planner_state(
                       name, PlannerStatus.ACTIVE) for name in self.names),
                   45.0, "all controllers accepting screened paths")
        self._wait(lambda: all(self._planner_state(
                       name, PlannerStatus.REACHED) for name in self.names),
                   210.0, "all terrain paths completing")
        motion = {name: round(self._motion(name), 1) for name in self.names}
        for route in self.routes:
            if motion[route["uav_name"]] < float(route["minimum_motion_m"]):
                raise AcceptanceFailure(
                    "{} insufficient motion: {:.1f} m".format(
                        route["uav_name"], motion[route["uav_name"]]))
        landing = self._land_all()
        if not all(landing.values()):
            raise AcceptanceFailure("a landing request was rejected")
        self._wait(lambda: all(
            self._landed_and_connected(name) for name in self.names),
            180.0, "all connected aircraft landing on ground and disarming")
        return {
            "success": True,
            "web_dispatch": True,
            "conflict_free": True,
            "motion_m": motion,
            "landing_accepted": landing,
            "landed_on_ground": {name: True for name in self.names},
        }

    def execute(self):
        try:
            result = self.run()
            rospy.loginfo("[TERRAIN_SWARM_SITL_ACCEPTANCE] %s",
                          json.dumps(result, sort_keys=True))
        except Exception as error:
            self.release.publish(Bool(data=False))
            self._land_all()
            result = {"success": False, "error": str(error)}
            rospy.logerr("[TERRAIN_SWARM_SITL_ACCEPTANCE] %s",
                         json.dumps(result, sort_keys=True))
        self.result.publish(String(data=json.dumps(result, sort_keys=True)))
        rospy.spin()


def main():
    rospy.init_node("terrain_fixedwing_swarm_acceptance")
    TerrainFixedwingSwarmAcceptance().execute()


if __name__ == "__main__":
    main()
