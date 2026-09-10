#!/usr/bin/env python3
"""End-to-end PX4 fixed-wing acceptance for the planning Path backend."""

import json
import math
import sys
import time

import rospy
from geometry_msgs.msg import Point32, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamGet, ParamSet, ParamSetRequest
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from xd_uav_controller.msg import ControlState
from xd_uav_planning.msg import NoFlyZone
from xd_uav_planning.nofly import Zone, path_min_clearance
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
        self.enable_nofly = bool(rospy.get_param("~enable_nofly", False))
        self.dynamic_nofly = bool(rospy.get_param("~dynamic_nofly", False))
        self.zone_id = int(rospy.get_param("~zone_id", 7101))
        self.zone_center_fraction = float(
            rospy.get_param("~zone_center_fraction", 0.25))
        self.zone_half_size = float(rospy.get_param("~zone_half_size", 10.0))
        self.zone_clearance = float(rospy.get_param("~zone_clearance", 10.0))
        self.dynamic_insert_fraction = float(
            rospy.get_param("~dynamic_insert_fraction", 0.08))
        self.dynamic_remove_fraction = float(
            rospy.get_param("~dynamic_remove_fraction", 0.20))
        self.dynamic_remove_margin = float(
            rospy.get_param("~dynamic_remove_margin", 5.0))
        self.state = None
        self.control_state = None
        self.statuses = []
        self.start_position = None
        self.forwarded_paths = []
        self.zone_model = None
        self.path_direction = None
        self.zone_pass_distance = None

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
        self.zone_publisher = rospy.Publisher(
            self.namespace + "/planning/no_fly_zone", NoFlyZone,
            queue_size=1, latch=True)
        self.forwarded_path_subscriber = rospy.Subscriber(
            self.namespace + "/control/reference/path", Path,
            self.forwarded_paths.append)
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
        self.path_direction = (math.cos(course), math.sin(course))
        return path

    def _zone(self, path):
        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        fraction = self.zone_center_fraction
        center_x = start.x + fraction * (end.x - start.x)
        center_y = start.y + fraction * (end.y - start.y)
        half = self.zone_half_size
        message = NoFlyZone()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.schema_version = NoFlyZone.CURRENT_SCHEMA_VERSION
        message.operation = NoFlyZone.OP_UPSERT
        message.zone_id = self.zone_id
        message.enabled = True
        message.zone_type = NoFlyZone.TYPE_NO_FLY
        message.min_altitude = 0.0
        message.max_altitude = max(100.0, self.takeoff_altitude + 30.0)
        message.valid_until = rospy.Time(0)
        vertices = ((center_x - half, center_y - half),
                    (center_x + half, center_y - half),
                    (center_x + half, center_y + half),
                    (center_x - half, center_y + half))
        for x_value, y_value in vertices:
            message.polygon.points.append(
                Point32(x=x_value, y=y_value, z=0.0))
        self.zone_model = Zone(
            self.zone_id, True, message.min_altitude,
            message.max_altitude, vertices)
        direction_x, direction_y = self.path_direction
        self.zone_pass_distance = max(
            (x_value - start.x) * direction_x +
            (y_value - start.y) * direction_y
            for x_value, y_value in vertices)
        self.zone_pass_distance += (
            self.zone_clearance + self.dynamic_remove_margin)
        return message

    def _zone_remove(self):
        message = NoFlyZone()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.schema_version = NoFlyZone.CURRENT_SCHEMA_VERSION
        message.operation = NoFlyZone.OP_REMOVE
        message.zone_id = self.zone_id
        message.enabled = False
        message.zone_type = NoFlyZone.TYPE_NO_FLY
        return message

    @staticmethod
    def _path_points(path):
        return [(pose.pose.position.x, pose.pose.position.y,
                 pose.pose.position.z) for pose in path.poses]

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

    def _along_track_distance(self):
        if (self.start_position is None or self.control_state is None or
                self.path_direction is None):
            return 0.0
        position = self.control_state.position_odom
        return ((position.x - self.start_position[0]) * self.path_direction[0] +
                (position.y - self.start_position[1]) * self.path_direction[1])

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
        if self.enable_nofly and self.dynamic_nofly:
            raise AcceptanceFailure(
                "enable_nofly and dynamic_nofly are mutually exclusive")
        if self.enable_nofly:
            if self.path_length < 240.0:
                raise AcceptanceFailure(
                    "no-fly demo requires path_length >= 240 m")
            zone = self._zone(path)
            self._wait(lambda: self.zone_publisher.get_num_connections() > 0,
                       10.0, "planning no-fly-zone subscriber")
            self.zone_publisher.publish(zone)
            rospy.sleep(0.3)
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
        minimum_clearance = None
        if self.enable_nofly:
            self._wait(lambda: bool(self.forwarded_paths), 5.0,
                       "adjusted controller path")
            forwarded = self.forwarded_paths[-1]
            if len(forwarded.poses) <= len(path.poses):
                raise AcceptanceFailure("planning did not expand the blocked path")
            minimum_clearance = path_min_clearance(
                self._path_points(forwarded), (self.zone_model,), 1.0)
            if minimum_clearance < self.zone_clearance:
                raise AcceptanceFailure(
                    "adjusted path violates zone clearance: %.2f m" %
                    minimum_clearance)
        elif self.dynamic_nofly:
            if self.path_length < 240.0:
                raise AcceptanceFailure(
                    "dynamic no-fly demo requires path_length >= 240 m")
            zone = self._zone(path)
            self._wait(
                lambda: self._distance_flown() >=
                        self.path_length * self.dynamic_insert_fraction,
                30.0, "dynamic no-fly insertion point")
            before_insert = len(self.forwarded_paths)
            status_before_insert = len(self.statuses)
            self.zone_publisher.publish(zone)
            self._wait(
                lambda: len(self.forwarded_paths) > before_insert and any(
                    message.goal_id == self.goal_id and
                    "dynamic no-fly upsert" in message.detail
                    for message in self.statuses[status_before_insert:]),
                15.0, "online detour replacement")
            detour = self.forwarded_paths[-1]
            minimum_clearance = path_min_clearance(
                self._path_points(detour), (self.zone_model,), 1.0)
            if minimum_clearance < self.zone_clearance:
                raise AcceptanceFailure(
                    "online detour violates zone clearance: %.2f m" %
                    minimum_clearance)
            self._wait(
                lambda: (self._distance_flown() >=
                         self.path_length * self.dynamic_remove_fraction and
                         self._along_track_distance() >=
                         self.zone_pass_distance),
                45.0, "aircraft passing the no-fly zone and clearance margin")
            before_remove = len(self.forwarded_paths)
            status_before_remove = len(self.statuses)
            self.zone_publisher.publish(self._zone_remove())
            self._wait(
                lambda: len(self.forwarded_paths) > before_remove and any(
                    message.goal_id == self.goal_id and
                    "dynamic no-fly remove" in message.detail
                    for message in self.statuses[status_before_remove:]),
                15.0, "restored remaining-path replacement")
            restored = self.forwarded_paths[-1]
            if len(restored.poses) >= len(detour.poses):
                raise AcceptanceFailure(
                    "zone removal did not restore a simpler remaining path")

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
            "no_fly_enabled": self.enable_nofly,
            "dynamic_no_fly": self.dynamic_nofly,
            "online_replacements": (2 if self.dynamic_nofly else 0),
            "zone_clearance_m": (round(minimum_clearance, 2)
                                 if minimum_clearance is not None else None),
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
