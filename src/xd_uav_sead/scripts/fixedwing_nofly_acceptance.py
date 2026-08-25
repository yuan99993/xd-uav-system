#!/usr/bin/env python3
"""Headless end-to-end acceptance for one PX4 fixed wing and dynamic NFZ."""

import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import Point32
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamGet, ParamSet, ParamSetRequest
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String

from xd_uav_controller.msg import ControlState
from xd_uav_controller.srv import Takeoff
from xd_uav_sead.msg import NoFlyZone


class AcceptanceFailure(RuntimeError):
    pass


def _distance_to_rectangle(x, y, rectangle):
    xmin, xmax, ymin, ymax = rectangle
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


class FixedwingNoFlyAcceptance:
    PROFILES = {
        # All profiles improve on the historical 35 m turn radius.  Relaxation
        # reduces obstacle size/clearance and mission length, never the real
        # trajectory collision check.
        "nominal": {
            "target_distance": 300.0,
            "turning_radius": 70.0,
            "planning_clearance": 10.0,
            "zone_half_size": 18.0,
            "zone_ahead_distance": 125.0,
        },
        "relaxed1": {
            "target_distance": 270.0,
            "turning_radius": 65.0,
            "planning_clearance": 8.0,
            "zone_half_size": 15.0,
            "zone_ahead_distance": 110.0,
        },
        "relaxed2": {
            "target_distance": 240.0,
            "turning_radius": 60.0,
            "planning_clearance": 5.0,
            "zone_half_size": 12.0,
            "zone_ahead_distance": 95.0,
        },
    }

    def __init__(self):
        self.uav_name = rospy.get_param("~uav_name", "uav1")
        self.namespace = "/" + self.uav_name
        self.profile = str(rospy.get_param("~profile", "nominal"))
        if self.profile not in self.PROFILES:
            raise AcceptanceFailure("unknown v9 profile: " + self.profile)
        profile = self.PROFILES[self.profile]
        self.takeoff_altitude = float(rospy.get_param("~takeoff_altitude", 30.0))
        self.target_distance = float(
            rospy.get_param("~target_distance", profile["target_distance"])
        )
        self.cruise_speed = float(rospy.get_param("~cruise_speed", 15.0))
        self.turning_radius = float(
            rospy.get_param("~turning_radius", profile["turning_radius"])
        )
        self.planning_clearance = float(
            rospy.get_param("~planning_clearance", profile["planning_clearance"])
        )
        self.total_timeout = float(rospy.get_param("~total_timeout", 210.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 80.0))
        self.sync_timeout = float(rospy.get_param("~sync_timeout", 30.0))
        configured_zone_half_size = float(rospy.get_param("~zone_half_size", -1.0))
        self.zone_half_size = (
            configured_zone_half_size
            if configured_zone_half_size > 0.0
            else float(profile["zone_half_size"])
        )
        self.zone_ahead_distance = float(
            rospy.get_param(
                "~zone_ahead_distance", profile["zone_ahead_distance"]
            )
        )
        self.zone_ttl = float(rospy.get_param("~zone_ttl", 45.0))
        self.configure_headless_failsafes = bool(
            rospy.get_param("~configure_headless_failsafes", True)
        )
        self.shared_frame = str(rospy.get_param("~shared_frame", ""))
        self.shared_offset = (
            float(rospy.get_param("~shared_offset_x", 0.0)),
            float(rospy.get_param("~shared_offset_y", 0.0)),
            float(rospy.get_param("~shared_offset_z", 0.0)),
        )
        self.zone_role = str(rospy.get_param("~zone_role", "standalone"))
        if self.zone_role not in ("standalone", "leader", "follower"):
            raise AcceptanceFailure("zone_role must be standalone, leader or follower")

        self._lock = threading.RLock()
        self.state = None
        self.control_state = None
        self.odometry = None
        self.path = None
        self.path_version = 0
        self.trajectory = []
        self.rectangle = None
        self.minimum_zone_clearance = math.inf
        self.ready_uavs = set()
        self.mission_ready_uavs = set()
        self.mission_start = self.zone_role == "standalone"

        rospy.Subscriber(self.namespace + "/mavros/state", State, self._state_cb)
        rospy.Subscriber(
            self.namespace + "/control_manager/state",
            ControlState,
            self._control_state_cb,
        )
        rospy.Subscriber(
            self.namespace + "/mavros/local_position/odom",
            Odometry,
            self._odometry_cb,
        )
        rospy.Subscriber(
            self.namespace + "/sead/planned_path", Path, self._path_cb
        )
        self.command_pub = rospy.Publisher(
            self.namespace + "/sead/command", String, queue_size=5, latch=True
        )
        self.zone_pub = rospy.Publisher(
            self.namespace + "/dynamic_nofly_zone",
            NoFlyZone,
            queue_size=5,
            latch=True,
        )
        self.shared_zone_pub = rospy.Publisher(
            "/sead/v10/dynamic_nofly_zone", NoFlyZone, queue_size=1, latch=True
        )
        self.zone_ready_pub = rospy.Publisher(
            "/sead/v10/%s/ready_for_zone" % self.uav_name,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.mission_ready_pub = rospy.Publisher(
            "/sead/v10/%s/ready_for_mission" % self.uav_name,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.mission_start_pub = rospy.Publisher(
            "/sead/v10/start_mission", Bool, queue_size=1, latch=True
        )
        if self.zone_role in ("leader", "follower"):
            rospy.Subscriber(
                "/sead/v10/dynamic_nofly_zone",
                NoFlyZone,
                self._shared_zone_cb,
                queue_size=5,
            )
            rospy.Subscriber(
                "/sead/v10/start_mission",
                Bool,
                self._mission_start_cb,
                queue_size=1,
            )
        if self.zone_role == "leader":
            for name in ("uav1", "uav2", "uav3"):
                rospy.Subscriber(
                    "/sead/v10/%s/ready_for_zone" % name,
                    Bool,
                    self._zone_ready_cb,
                    callback_args=name,
                    queue_size=1,
                )
                rospy.Subscriber(
                    "/sead/v10/%s/ready_for_mission" % name,
                    Bool,
                    self._mission_ready_cb,
                    callback_args=name,
                    queue_size=1,
                )
        self.result_pub = rospy.Publisher(
            self.namespace + "/sead/fixedwing_acceptance/result",
            String,
            queue_size=1,
            latch=True,
        )
        self.initial_path_pub = rospy.Publisher(
            self.namespace + "/sead/acceptance/initial_path",
            Path,
            queue_size=1,
            latch=True,
        )
        self.replanned_path_pub = rospy.Publisher(
            self.namespace + "/sead/acceptance/replanned_path",
            Path,
            queue_size=1,
            latch=True,
        )

    def _state_cb(self, message):
        with self._lock:
            self.state = message

    def _control_state_cb(self, message):
        with self._lock:
            self.control_state = message

    def _odometry_cb(self, message):
        position = message.pose.pose.position
        shared_x = float(position.x) + self.shared_offset[0]
        shared_y = float(position.y) + self.shared_offset[1]
        shared_z = float(position.z) + self.shared_offset[2]
        with self._lock:
            self.odometry = message
            if self.rectangle is not None:
                clearance = _distance_to_rectangle(
                    shared_x, shared_y, self.rectangle
                )
                self.minimum_zone_clearance = min(
                    self.minimum_zone_clearance, clearance
                )
            self.trajectory.append((
                rospy.Time.now().to_sec(),
                shared_x,
                shared_y,
                shared_z,
            ))

    def _shared_position(self):
        position = self.odometry.pose.pose.position
        return (
            float(position.x) + self.shared_offset[0],
            float(position.y) + self.shared_offset[1],
            float(position.z) + self.shared_offset[2],
        )

    def _zone_ready_cb(self, message, uav_name):
        with self._lock:
            if message.data:
                self.ready_uavs.add(uav_name)
            else:
                self.ready_uavs.discard(uav_name)

    def _mission_ready_cb(self, message, uav_name):
        with self._lock:
            if message.data:
                self.mission_ready_uavs.add(uav_name)
            else:
                self.mission_ready_uavs.discard(uav_name)

    def _mission_start_cb(self, message):
        with self._lock:
            self.mission_start = bool(message.data)

    def _shared_zone_cb(self, message):
        if self.shared_frame and message.header.frame_id != self.shared_frame:
            rospy.logerr(
                "[FW_ACCEPTANCE] shared NFZ frame mismatch: expected=%s got=%s",
                self.shared_frame,
                message.header.frame_id,
            )
            return
        vertices = [(float(p.x), float(p.y)) for p in message.polygon.points]
        if len(vertices) >= 3:
            xs = [p[0] for p in vertices]
            ys = [p[1] for p in vertices]
            with self._lock:
                self.rectangle = (min(xs), max(xs), min(ys), max(ys))
                self.minimum_zone_clearance = math.inf
        self.zone_pub.publish(message)

    def _path_cb(self, message):
        with self._lock:
            self.path = message
            self.path_version += 1

    def _wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return
            rate.sleep()
        raise AcceptanceFailure("timeout waiting for " + description)

    @staticmethod
    def _path_points(path):
        return [
            (float(p.pose.position.x), float(p.pose.position.y))
            for p in path.poses
        ]

    @staticmethod
    def _path_signature(path):
        return tuple(
            (round(p.pose.position.x, 1), round(p.pose.position.y, 1))
            for p in path.poses[:: max(1, len(path.poses) // 40)]
        )

    def _publish_command(self, msg_id, info):
        payload = {
            "msg_id": int(msg_id),
            "info": info,
            "ts": rospy.Time.now().to_sec(),
            "source": "fixedwing_nofly_acceptance",
        }
        self.command_pub.publish(String(data=json.dumps(payload)))

    def _set_and_verify_px4_int_param(self, param_id, value):
        set_service = self.namespace + "/mavros/param/set"
        get_service = self.namespace + "/mavros/param/get"
        rospy.wait_for_service(set_service, timeout=10.0)
        rospy.wait_for_service(get_service, timeout=10.0)
        setter = rospy.ServiceProxy(set_service, ParamSet)
        getter = rospy.ServiceProxy(get_service, ParamGet)
        sync_deadline = time.monotonic() + 45.0
        while not rospy.is_shutdown():
            try:
                current = getter(param_id=param_id)
            except rospy.ServiceException as exc:
                current = None
                rospy.logwarn_throttle(
                    5.0,
                    "[FW_ACCEPTANCE] MAVROS parameter pull still active for %s: %s",
                    param_id,
                    exc,
                )
            if current is not None and current.success:
                break
            if time.monotonic() >= sync_deadline:
                raise AcceptanceFailure(
                    "PX4 parameter %s was not synchronized by MAVROS" % param_id
                )
            rospy.logwarn_throttle(
                5.0,
                "[FW_ACCEPTANCE] waiting for MAVROS PX4 parameter sync: %s",
                param_id,
            )
            time.sleep(1.0)
        else:
            raise AcceptanceFailure("ROS shutdown during PX4 parameter sync")

        for attempt in range(1, 4):
            request = ParamSetRequest()
            request.param_id = param_id
            request.value.integer = int(value)
            try:
                response = setter(request)
                time.sleep(0.25)
                actual = getter(param_id=param_id)
            except rospy.ServiceException as exc:
                rospy.logwarn(
                    "[FW_ACCEPTANCE] PX4 %s attempt %d/3 had transient MAVROS error: %s",
                    param_id,
                    attempt,
                    exc,
                )
                time.sleep(1.0)
                continue
            if (
                response.success
                and actual.success
                and int(actual.value.integer) == int(value)
            ):
                rospy.logwarn(
                    "[FW_ACCEPTANCE] verified PX4 %s=%d",
                    param_id,
                    value,
                )
                return
            rospy.logwarn(
                "[FW_ACCEPTANCE] PX4 %s verification attempt %d/3 failed "
                "(set_success=%s get_success=%s actual_integer=%d)",
                param_id,
                attempt,
                response.success,
                actual.success,
                int(actual.value.integer),
            )
        raise AcceptanceFailure(
            "PX4 parameter %s did not verify as %d" % (param_id, value)
        )

    def _set_and_verify_px4_float_param(self, param_id, value):
        set_service = self.namespace + "/mavros/param/set"
        get_service = self.namespace + "/mavros/param/get"
        rospy.wait_for_service(set_service, timeout=10.0)
        rospy.wait_for_service(get_service, timeout=10.0)
        setter = rospy.ServiceProxy(set_service, ParamSet)
        getter = rospy.ServiceProxy(get_service, ParamGet)

        for attempt in range(1, 4):
            request = ParamSetRequest()
            request.param_id = param_id
            request.value.real = float(value)
            try:
                response = setter(request)
                time.sleep(0.25)
                actual = getter(param_id=param_id)
            except rospy.ServiceException as exc:
                rospy.logwarn(
                    "[FW_ACCEPTANCE] PX4 %s attempt %d/3 had transient MAVROS error: %s",
                    param_id,
                    attempt,
                    exc,
                )
                time.sleep(1.0)
                continue
            actual_value = float(actual.value.real)
            tolerance = max(1e-4, abs(float(value)) * 1e-6)
            if response.success and actual.success and abs(actual_value - value) <= tolerance:
                rospy.logwarn(
                    "[FW_ACCEPTANCE] verified PX4 %s=%.3f",
                    param_id,
                    value,
                )
                return
            rospy.logwarn(
                "[FW_ACCEPTANCE] PX4 %s verification attempt %d/3 failed "
                "(set_success=%s get_success=%s actual_real=%.3f)",
                param_id,
                attempt,
                response.success,
                actual.success,
                actual_value,
            )
        raise AcceptanceFailure(
            "PX4 parameter %s did not verify as %.3f" % (param_id, value)
        )

    def _configure_headless_px4(self):
        if not self.configure_headless_failsafes:
            return
        # SITL has neither RC nor a GCS heartbeat. Disable only those two loss
        # actions; retain the official simulated airspeed and all estimator
        # checks used by the real fixed-wing control gate.
        self._set_and_verify_px4_int_param("COM_RC_IN_MODE", 4)
        self._set_and_verify_px4_int_param("COM_RCL_EXCEPT", 6)
        self._set_and_verify_px4_int_param("NAV_DLL_ACT", 0)
        self._set_and_verify_px4_float_param("SIM_BAT_DRAIN", 86400.0)
        self._set_and_verify_px4_float_param("SIM_BAT_MIN_PCT", 100.0)

    def _publish_zone(self):
        xmin, xmax, ymin, ymax = self.rectangle
        now = rospy.Time.now()
        message = NoFlyZone()
        message.header.stamp = now
        message.header.frame_id = self.shared_frame or (
            self.namespace.lstrip("/") + "/odom"
        )
        message.schema_version = NoFlyZone.CURRENT_SCHEMA_VERSION
        message.operation = NoFlyZone.OP_UPSERT
        message.zone_id = 9001
        message.enabled = True
        message.zone_type = NoFlyZone.TYPE_NO_FLY
        message.min_altitude = 0.0
        message.max_altitude = max(100.0, self.takeoff_altitude + 30.0)
        message.valid_until = now + rospy.Duration(self.zone_ttl)
        for x, y in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)):
            point = Point32()
            point.x = x
            point.y = y
            message.polygon.points.append(point)
        if self.zone_role in ("leader", "follower"):
            if self.zone_role == "leader":
                self.shared_zone_pub.publish(message)
        else:
            self.zone_pub.publish(message)

    def _choose_zone(self, path, current_x, current_y):
        points = self._path_points(path)
        if len(points) < 2:
            raise AcceptanceFailure("initial path has no safe ahead point for NFZ insertion")

        # A closed route contains a geometrically nearby return leg.  Select
        # the obstacle by forward path arc length, not Euclidean distance, so
        # the acceptance always challenges the outbound leg.
        search_arc = max(
            60.0,
            2.0 * self.turning_radius,
            2.0 * self.zone_ahead_distance,
        )
        search_end = 0
        accumulated = 0.0
        while search_end < len(points) - 1 and accumulated < search_arc:
            accumulated += math.hypot(
                points[search_end + 1][0] - points[search_end][0],
                points[search_end + 1][1] - points[search_end][1],
            )
            search_end += 1
        anchor = min(
            range(search_end + 1),
            key=lambda index: math.hypot(
                points[index][0] - current_x,
                points[index][1] - current_y,
            ),
        )
        target = anchor
        travelled = 0.0
        while target < len(points) - 1 and travelled < self.zone_ahead_distance:
            travelled += math.hypot(
                points[target + 1][0] - points[target][0],
                points[target + 1][1] - points[target][1],
            )
            target += 1
        if target == anchor:
            raise AcceptanceFailure("initial path has no safe ahead point for NFZ insertion")
        center_x, center_y = points[target]
        half = self.zone_half_size
        return (
            center_x - half,
            center_x + half,
            center_y - half,
            center_y + half,
        )

    def _abort_to_loiter(self):
        with self._lock:
            airborne = bool(self.state and self.state.armed)
        if not airborne:
            return
        self._publish_command(8, {})
        try:
            self._wait(
                lambda: self.state is not None
                and self.state.mode == "AUTO.LOITER",
                12.0,
                "AUTO.LOITER fail-closed handoff",
            )
        except AcceptanceFailure as exc:
            rospy.logerr("[FW_ACCEPTANCE] %s", exc)

    def run(self):
        started = time.monotonic()
        rospy.loginfo("[FW_ACCEPTANCE] waiting for complete fixed-wing control state")
        self._wait(
            lambda: self.state is not None
            and self.state.connected
            and self.control_state is not None
            and self.control_state.state_valid
            and self.control_state.stable
            and self.control_state.airspeed_valid
            and self.odometry is not None,
            90.0,
            "MAVROS, estimator, manager and official airspeed sensor",
        )

        self._configure_headless_px4()

        rospy.wait_for_service(
            self.namespace + "/control_manager/takeoff", timeout=10.0
        )
        response = rospy.ServiceProxy(
            self.namespace + "/control_manager/takeoff", Takeoff
        )(altitude=self.takeoff_altitude)
        if not response.success:
            raise AcceptanceFailure("takeoff rejected: " + response.message)

        self._wait(
            lambda: self.state.armed
            and self.state.mode == "OFFBOARD"
            and self._shared_position()[2] >= self.takeoff_altitude - 6.0
            and self.control_state.airspeed >= 8.0,
            self.takeoff_timeout,
            "armed OFFBOARD climb",
        )
        with self._lock:
            origin_x, origin_y, origin_z = self._shared_position()
            initial_path_version = self.path_version

        if self.zone_role in ("leader", "follower"):
            self.mission_ready_pub.publish(Bool(data=True))
            if self.zone_role == "leader":
                self._wait(
                    lambda: self.mission_ready_uavs == {"uav1", "uav2", "uav3"},
                    self.sync_timeout,
                    "all v10 aircraft airborne",
                )
                self.mission_start_pub.publish(Bool(data=True))
            self._wait(
                lambda: self.mission_start,
                self.sync_timeout,
                "synchronized v10 mission start",
            )

        # Align the mission with measured ground track.  A fixed world +X
        # target caused an artificial near-90-degree command at handoff after
        # the runway climb, before the aircraft ever reached the NFZ.
        course = float(self.control_state.course)
        target_x = origin_x + self.target_distance * math.cos(course)
        target_y = origin_y + self.target_distance * math.sin(course)
        self._publish_command(
            18,
            {
                "targets": [[target_x, target_y]],
                "unknown_targets": [],
                "uav_type": 2,
                "velocity": self.cruise_speed,
                "Rmin": self.turning_radius,
                "waypoint_radius": 20,
                # The legacy GCS packet stores headings in degrees; the
                # onboard decoder converts them to radians.
                "init_pos": [origin_x, origin_y, math.degrees(course)],
                "end": [origin_x, origin_y, math.degrees(course + math.pi)],
            },
        )
        self._wait(
            lambda: self.path_version > initial_path_version
            and self.path is not None
            and len(self.path.poses) >= 10,
            20.0,
            "initial SEAD Dubins path",
        )

        with self._lock:
            initial_path = self.path
            initial_signature = self._path_signature(initial_path)
            current_x, current_y, _ = self._shared_position()
            before_replan_version = self.path_version
            if self.zone_role in ("standalone", "leader"):
                self.rectangle = self._choose_zone(initial_path, current_x, current_y)
            self.minimum_zone_clearance = math.inf
        self.initial_path_pub.publish(initial_path)

        self.zone_ready_pub.publish(Bool(data=True))
        if self.zone_role == "leader":
            self._wait(
                lambda: self.ready_uavs == {"uav1", "uav2", "uav3"},
                self.sync_timeout,
                "all v10 aircraft initial paths",
            )
            rospy.logwarn("[FW_ACCEPTANCE] inserting shared NFZ rectangle=%s", self.rectangle)
            self._publish_zone()
        elif self.zone_role == "follower":
            self._wait(
                lambda: self.rectangle is not None,
                self.sync_timeout,
                "shared v10 NFZ",
            )
        else:
            rospy.logwarn("[FW_ACCEPTANCE] inserting NFZ rectangle=%s", self.rectangle)
            self._publish_zone()
        initial_min_clearance = min(
            _distance_to_rectangle(x, y, self.rectangle)
            for x, y in self._path_points(initial_path)
        )
        replanned = False
        try:
            self._wait(
                lambda: self.path_version > before_replan_version
                and self.path is not None
                and self._path_signature(self.path) != initial_signature,
                20.0,
                "dynamic NFZ replan",
            )
            replanned = True
        except AcceptanceFailure:
            if initial_min_clearance + 1e-6 < self.planning_clearance:
                raise
            rospy.loginfo(
                "[FW_ACCEPTANCE] initial path unaffected by shared NFZ "
                "(clearance=%.1fm); retaining safe path",
                initial_min_clearance,
            )
        with self._lock:
            replanned_path = self.path if replanned else initial_path
        self.replanned_path_pub.publish(replanned_path)
        planned_min_clearance = min(
            _distance_to_rectangle(x, y, self.rectangle)
            for x, y in self._path_points(replanned_path)
        )
        if planned_min_clearance + 1e-6 < self.planning_clearance:
            raise AcceptanceFailure(
                "replanned path clearance %.1fm is below %.1fm"
                % (planned_min_clearance, self.planning_clearance)
            )

        reached_target = False
        last_zone_refresh = time.monotonic()
        deadline = started + self.total_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_zone_refresh >= self.zone_ttl / 3.0:
                self._publish_zone()
                last_zone_refresh = now
            with self._lock:
                position_x, position_y, position_z = self._shared_position()
                target_distance = math.hypot(position_x - target_x, position_y - target_y)
                home_distance = math.hypot(position_x - origin_x, position_y - origin_y)
                mode = self.state.mode
                armed = self.state.armed
                arrival_distance = max(20.0, 1.25 * self.turning_radius)
                if target_distance <= arrival_distance:
                    reached_target = True
                completed = (
                    reached_target
                    and home_distance <= arrival_distance
                    and mode == "AUTO.LOITER"
                    and armed
                )
                min_actual = self.minimum_zone_clearance
                altitude = position_z
            if not armed:
                raise AcceptanceFailure("aircraft disarmed before mission completion")
            if altitude < max(10.0, 0.5 * self.takeoff_altitude):
                raise AcceptanceFailure(
                    "aircraft descended below mission safety altitude: %.1fm"
                    % altitude
                )
            if min_actual <= 0.0:
                raise AcceptanceFailure("aircraft entered the dynamic no-fly polygon")
            if completed:
                return {
                    "success": True,
                    "phase": "complete",
                    "replanned": replanned,
                    "initial_path_clearance_m": round(initial_min_clearance, 2),
                    "profile": self.profile,
                    "planned_min_clearance_m": round(planned_min_clearance, 2),
                    "actual_min_clearance_m": round(min_actual, 2),
                    "reached_target": True,
                    "returned_home": True,
                    "final_mode": mode,
                    "elapsed_s": round(time.monotonic() - started, 1),
                }
            rate.sleep()
        raise AcceptanceFailure("mission did not reach target, return and enter AUTO.LOITER")

    def publish_result_forever(self, result):
        payload = json.dumps(result, sort_keys=True)
        level = rospy.loginfo if result.get("success") else rospy.logerr
        level("[FW_ACCEPTANCE_RESULT] %s", payload)
        rate = rospy.Rate(1)
        last_zone_refresh = 0.0
        while not rospy.is_shutdown():
            self.result_pub.publish(String(data=payload))
            now = time.monotonic()
            if (
                self.rectangle is not None
                and self.zone_role in ("standalone", "leader")
                and now - last_zone_refresh >= self.zone_ttl / 3.0
            ):
                self._publish_zone()
                last_zone_refresh = now
            rate.sleep()


def main():
    rospy.init_node("fixedwing_nofly_acceptance")
    acceptance = FixedwingNoFlyAcceptance()
    try:
        result = acceptance.run()
    except Exception as exc:
        rospy.logerr("[FW_ACCEPTANCE] failed: %s", exc)
        acceptance._abort_to_loiter()
        result = {
            "success": False,
            "phase": "failed",
            "reason": str(exc),
            "actual_min_clearance_m": (
                None
                if not math.isfinite(acceptance.minimum_zone_clearance)
                else round(acceptance.minimum_zone_clearance, 2)
            ),
        }
    acceptance.publish_result_forever(result)


if __name__ == "__main__":
    main()
