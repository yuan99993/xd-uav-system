#!/usr/bin/env python3
"""Team-level formation service and shared per-member runtime."""

import copy
import math
import os
import re
import threading
import time

import rospy
import tf2_ros
import yaml
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, TriggerResponse
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from xd_uav_formation.core import (
    formation_offsets, path_reference_state, rounded_path)
from xd_uav_formation.runtime import FormationRuntime
from xd_uav_formation.srv import FormationControl, FormationControlResponse


_SIMPLE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")
_FORMATION_SERVICE = re.compile(
    r"^/([A-Za-z0-9_-]+)/formation_(consensus|follower)/set_enabled$")
_MODES = ("leader_follow", "consensus")
_COMMON_NUMERIC_CONFIG_KEYS = (
    "minimum_slot_distance", "fixedwing_minimum_slot_distance",
    "fixedwing_min_speed", "fixedwing_max_catchup_speed",
    "fixedwing_min_groundspeed_for_course", "multirotor_max_speed",
    "catchup_gain", "catchup_ki", "catchup_kd", "catchup_deadband",
    "catchup_integral_limit", "catchup_derivative_filter_time_constant",
    "multirotor_max_catchup_speed", "yaw_offset", "rate",
    "state_timeout", "mavros_timeout", "tf_max_age",
    "future_stamp_tolerance", "tf_lookup_timeout",
    "tf_buffer_cache_time", "external_quiet_time")
_CONSENSUS_NUMERIC_CONFIG_KEYS = (
    "spacing", "corner_radius", "cruise_speed", "lookahead_distance",
    "consensus_gain", "convergence_time", "max_consensus_velocity",
    "acquire_consensus_gain", "acquire_max_consensus_velocity",
    "acquire_max_speed_adjustment", "formation_heading_rate_limit",
    "formation_speed_slew_rate", "formation_course_slew_rate",
    "route_correction_gain", "max_route_correction_velocity",
    "acquire_tolerance", "acquire_height_tolerance",
    "acquire_vertical_spacing", "acquire_timeout")


def _require_numeric_config(config, keys, mode):
    for key in keys:
        value = config.get(key)
        if isinstance(value, bool) or value is None:
            raise ValueError(
                "{} configuration is missing numeric parameter: {}".
                format(mode, key))
        try:
            number = float(value)
        except (TypeError, ValueError):
            raise ValueError(
                "{} configuration parameter must be numeric: {}".
                format(mode, key))
        if not math.isfinite(number):
            raise ValueError(
                "{} configuration parameter must be finite: {}".
                format(mode, key))


def _load_yaml(path):
    if not path:
        return {}
    with open(os.path.expanduser(path), "r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("formation YAML root must be a mapping")
    return value


class FormationManager:
    def __init__(self):
        self._lock = threading.RLock()
        self._active_group = None
        self._mode_runtimes = {}
        self._tf_buffer = None
        self._tf_listener = None
        self._configured_modes = {}

        config_path = rospy.get_param("~config", "")
        common_config_path = rospy.get_param("~common_config", "")
        self._profile = _load_yaml(config_path)
        common_parameters = _load_yaml(common_config_path)
        common_parameters.update(self._profile.get("common", {}) or {})
        if "service_timeout" in common_parameters:
            service_timeout = common_parameters["service_timeout"]
        elif rospy.has_param("~service_timeout"):
            service_timeout = rospy.get_param("~service_timeout")
        elif config_path or common_config_path:
            raise ValueError(
                "formation configuration is missing numeric parameter: "
                "service_timeout")
        else:
            service_timeout = 3.0
        self._service_timeout = float(service_timeout)
        if not math.isfinite(self._service_timeout) or \
                self._service_timeout <= 0:
            raise ValueError("service_timeout must be a positive finite number")
        self._load_mode_configs(self._profile, common_parameters)
        self._selection_subscribers = []
        if "consensus" in self._configured_modes:
            self._selection_subscribers.extend((
                rospy.Subscriber("/formation/shape", String,
                                 self._shape_command_callback,
                                 queue_size=1),
                rospy.Subscriber("/formation/path", Path,
                                 self._path_command_callback,
                                 queue_size=1)))

        self._spawn_members = bool(rospy.get_param("~spawn_members", False))
        if self._spawn_members:
            if not self._configured_modes:
                raise ValueError("formation config has no configured mode")

        self._service = rospy.Service(
            "/formation/set_enabled", FormationControl, self._handle)
        self._stop_service = rospy.Service(
            "/formation/stop", Trigger, self._handle_stop)
        rospy.on_shutdown(self._shutdown_children)
        launch_policy = ("single process on first enable" if self._spawn_members
                         else "external member nodes")
        rospy.loginfo(
            "[xd_uav_formation] unified services ready: "
            "/formation/set_enabled and /formation/stop; configured modes=%s; "
            "shared runtime policy=%s",
            ",".join(self._configured_modes) or "manual", launch_policy)

    @staticmethod
    def _valid_name(value):
        return isinstance(value, str) and bool(_SIMPLE_NAME.match(value))

    @staticmethod
    def _normalise_mode_config(mode_config, common_parameters, root_config):
        if not isinstance(mode_config, dict):
            raise ValueError("mode configuration must be a mapping")
        result = dict(common_parameters)
        result.update(copy.deepcopy(mode_config))
        result["team_id"] = mode_config.get(
            "team_id", root_config.get("team_id", "team1"))
        return result

    def _load_mode_configs(self, profile, common_parameters):
        nested_modes = {
            mode: profile[mode]
            for mode in _MODES
            if isinstance(profile.get(mode), dict)
        }
        if nested_modes:
            source_modes = nested_modes
        else:
            mode = profile.get("mode")
            source_modes = {mode: profile} if mode in _MODES else {}

        for mode, raw_config in source_modes.items():
            config = self._normalise_mode_config(
                raw_config, common_parameters, profile)
            members = config.get("members")
            if mode == "consensus":
                # The YAML supplies algorithm parameters only. The service
                # request supplies the members for each consensus run.
                config["members"] = []
                members = []
            else:
                if not isinstance(members, list) or not members or any(
                        not self._valid_name(member) for member in members):
                    raise ValueError(
                        "{} config must contain valid members".format(mode))
                if len(set(members)) != len(members):
                    raise ValueError("{} members contains duplicates".format(mode))
            if mode == "leader_follow":
                leader = config.get("leader_name", "")
                offsets = config.get("offsets")
                if (not self._valid_name(leader) or leader in members or
                        not isinstance(offsets, dict)):
                    raise ValueError(
                        "leader_follow requires leader_name and per-member offsets")
                for member in members:
                    offset = offsets.get(member)
                    if not isinstance(offset, (list, tuple)) or len(offset) != 3:
                        raise ValueError(
                            "leader_follow offset for {} must be [forward,left,up]".format(
                                member))
            team_id = config["team_id"]
            if not self._valid_name(team_id):
                raise ValueError("{} has invalid team_id".format(mode))
            config["mode"] = mode
            required_numeric = list(_COMMON_NUMERIC_CONFIG_KEYS)
            if mode == "consensus":
                required_numeric.extend(_CONSENSUS_NUMERIC_CONFIG_KEYS)
            _require_numeric_config(config, required_numeric, mode)
            if mode == "consensus":
                config["shape"] = str(config.get("shape", "circle")).strip().lower()
                config["spacing"] = float(config.get("spacing", 80.0))
                config["loop_path"] = bool(config.get("loop_path", True))
                config["waypoints"] = rounded_path(
                    config.get("waypoints", []),
                    float(config.get("corner_radius", 0.0)),
                    config["loop_path"])
                path_reference_state(
                    (0.0, 0.0, 0.0), config["waypoints"],
                    float(config.get("cruise_speed", 20.0)),
                    float(config.get("lookahead_distance", 60.0)),
                    config["loop_path"])
                config["waypoints_are_final"] = True
                formation_offsets(config["shape"], 2, config["spacing"])
            self._configured_modes[mode] = config

    def _shape_command_callback(self, message):
        shape = message.data.strip().lower()
        with self._lock:
            config = self._configured_modes.get("consensus")
            if config is None:
                return
            try:
                formation_offsets(shape, 2, config["spacing"])
                runtime = self._mode_runtimes.get("consensus")
                if runtime is not None:
                    runtime.update_selection(shape=shape)
                config["shape"] = shape
                rospy.loginfo(
                    "[xd_uav_formation] selected formation shape: %s", shape)
            except (TypeError, ValueError) as error:
                rospy.logwarn(
                    "[xd_uav_formation] rejected shape '%s': %s",
                    shape, error)

    def _path_command_callback(self, message):
        with self._lock:
            config = self._configured_modes.get("consensus")
            if config is None:
                return
            try:
                if len(message.poses) < 2:
                    raise ValueError("path requires at least two poses")
                target_frame = str(config.get("common_frame", "world")).strip("/")
                waypoints = []
                for source_pose in message.poses:
                    pose = copy.deepcopy(source_pose)
                    frame = (pose.header.frame_id or
                             message.header.frame_id).strip("/")
                    if not frame:
                        raise ValueError("path frame_id is empty")
                    pose.header.frame_id = frame
                    if pose.header.stamp == rospy.Time():
                        pose.header.stamp = message.header.stamp
                    if frame != target_frame:
                        transform = self._shared_tf_buffer().lookup_transform(
                            target_frame, frame, pose.header.stamp,
                            rospy.Duration(float(config.get(
                                "tf_lookup_timeout", 0.05))))
                        pose = do_transform_pose(pose, transform)
                    point = pose.pose.position
                    waypoints.append((point.x, point.y, point.z))
                # nav_msgs/Path has no loop flag. Keep the profile's
                # configured behavior instead of inferring an open path from
                # distinct endpoints.
                loop_path = bool(config.get("loop_path", True))
                path_reference_state(
                    (0.0, 0.0, 0.0), waypoints,
                    float(config.get("cruise_speed", 20.0)),
                    float(config.get("lookahead_distance", 60.0)),
                    loop_path)
                runtime = self._mode_runtimes.get("consensus")
                if runtime is not None:
                    runtime.update_selection(
                        waypoints=waypoints, loop_path=loop_path)
                config["waypoints"] = tuple(waypoints)
                config["loop_path"] = loop_path
                config["waypoints_are_final"] = True
                config.pop("_runtime_rounded_waypoints", None)
                rospy.loginfo(
                    "[xd_uav_formation] accepted %d route waypoint(s) in %s (%s)",
                    len(waypoints), target_frame,
                    "closed" if loop_path else "open")
            except (TypeError, ValueError, rospy.ROSException,
                    tf2_ros.TransformException) as error:
                rospy.logwarn(
                    "[xd_uav_formation] rejected route: %s", error)

    @staticmethod
    def _node_namespace(member, mode):
        node_name = "formation_consensus" if mode == "consensus" else \
            "formation_follower"
        return "/{}/{}".format(member, node_name)

    @staticmethod
    def _service_name(member, mode):
        node_name = "formation_consensus" if mode == "consensus" else \
            "formation_follower"
        return "/{}/{}/set_enabled".format(member, node_name)

    def _shared_tf_buffer(self):
        if self._tf_buffer is not None:
            return self._tf_buffer
        cache_times = [float(config.get("tf_buffer_cache_time", 5.0))
                       for config in self._configured_modes.values()]
        cache_time = max(cache_times or [5.0])
        if not math.isfinite(cache_time) or cache_time <= 0.0:
            raise ValueError("tf_buffer_cache_time must be positive and finite")
        self._tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(cache_time))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        return self._tf_buffer

    def _release_shared_tf(self):
        if self._tf_listener is not None:
            self._tf_listener.unregister()
        self._tf_listener = None
        self._tf_buffer = None

    def _start_configured_members(self, mode, members):
        """Create one shared runtime with per-member controllers."""
        template = self._configured_modes.get(mode)
        if template is None:
            raise RuntimeError("no configured {} mode to launch".format(mode))
        members = tuple(members)
        current = self._mode_runtimes.get(mode)
        if current is not None and current._members == members:
            return
        if current is not None:
            self._shutdown_mode_runtime(mode)
        config = copy.deepcopy(template)
        config["members"] = list(members)

        code, message, state = rospy.get_master().getSystemState()
        if code != 1:
            raise RuntimeError("cannot inspect ROS master: " + message)
        existing_services = {name for name, _nodes in state[2]}
        planned = []
        for member in config["members"]:
            service_name = self._service_name(member, mode)
            if service_name in existing_services:
                raise RuntimeError(
                    "{} already exists; stop the old formation nodes first".format(
                        service_name))
            planned.append((member, service_name))

        runtime = None
        try:
            runtime = FormationRuntime(
                mode, config, self._shared_tf_buffer())
            startup_deadline = time.monotonic() + self._service_timeout
            for _member, service_name in planned:
                remaining = startup_deadline - time.monotonic()
                if remaining <= 0.0:
                    raise rospy.ROSException(
                        "timed out waiting for {}".format(service_name))
                rospy.wait_for_service(service_name, timeout=remaining)
            rate = float(config.get("rate", 20.0))
            startup_settle = min(1.0, max(0.1, 2.0 / rate))
            if mode == "consensus":
                state_timeout = float(config.get("state_timeout", 0.5))
                center_period = min(1.0, 0.5 * state_timeout)
                startup_settle = max(startup_settle, center_period + 0.05)
            time.sleep(startup_settle)
        except Exception:
            if runtime is not None:
                runtime.shutdown()
            if not self._mode_runtimes:
                self._release_shared_tf()
            raise
        self._mode_runtimes[mode] = runtime

    def _shutdown_mode_runtime(self, mode):
        runtime = self._mode_runtimes.pop(mode, None)
        if runtime is None:
            return
        try:
            runtime.shutdown()
        except Exception as error:
            rospy.logwarn(
                "[xd_uav_formation] %s runtime shutdown error: %s",
                mode, error)
        if not self._mode_runtimes:
            try:
                self._release_shared_tf()
            except Exception as error:
                rospy.logwarn(
                    "[xd_uav_formation] TF shutdown error: %s", error)

    def _shutdown_children(self):
        for mode in tuple(self._mode_runtimes):
            self._shutdown_mode_runtime(mode)

    def _discover_members(self, mode):
        try:
            code, message, state = rospy.get_master().getSystemState()
            if code != 1:
                return None, message
        except Exception as error:
            return None, str(error)

        groups = {}
        for service_name, _nodes in state[2]:
            match = _FORMATION_SERVICE.match(service_name)
            if not match:
                continue
            member, node_mode = match.groups()
            if node_mode != mode:
                continue
            namespace = self._node_namespace(member, mode)
            team_id = rospy.get_param(namespace + "/team_id", None)
            if not self._valid_name(team_id):
                continue
            if mode == "leader_follow":
                leader = rospy.get_param(namespace + "/leader_name", "")
                if self._valid_name(leader):
                    groups.setdefault((team_id, leader), []).append(member)
            else:
                configured_members = rospy.get_param(
                    namespace + "/members", [])
                if isinstance(configured_members, list) and configured_members:
                    key = (team_id, tuple(configured_members))
                    groups.setdefault(key, list(configured_members))

        if not groups:
            return None, "no configured {} nodes are running".format(mode)
        if len(groups) != 1:
            return None, "multiple teams are running; use a profile YAML"
        members = next(iter(groups.values()))
        if mode == "leader_follow":
            members = sorted(set(members))
        return members, ""

    def _resolve_request(self, request):
        mode = (request.mode or "").strip().lower()
        requested_members = [
            name.strip().strip("/") for name in request.members]
        if mode not in _MODES:
            return None, "mode must be consensus or leader_follow"

        if mode == "consensus":
            members = requested_members
            if not members:
                return None, "consensus members must be specified in the service request"
            if self._spawn_members and mode not in self._configured_modes:
                return None, "loaded profile does not configure consensus parameters"
        elif self._configured_modes:
            config = self._configured_modes.get(mode)
            if config is None:
                return None, (
                    "loaded profile does not configure {}; select a matching YAML".
                    format(mode))
            members = list(config["members"])
            if requested_members and requested_members != members:
                return None, "members are defined by the loaded YAML profile"
        elif requested_members:
            members = requested_members
        else:
            members, error = self._discover_members(mode)
            if error:
                return None, error

        if not members or any(not self._valid_name(name) for name in members):
            return None, "members must contain valid UAV namespace names"
        if len(set(members)) != len(members):
            return None, "members contains duplicates"
        if mode == "consensus" and len(members) < 2:
            return None, "consensus requires at least two members"
        return (mode, members), ""

    def _validate_member_configuration(self, mode, members):
        if self._spawn_members and mode in self._configured_modes:
            config = self._configured_modes[mode]
            if mode != "consensus" and list(config["members"]) != list(members):
                return None, "members do not match the loaded YAML profile"
            return config["team_id"], ""

        team_id = None
        leaders = set()
        for member in members:
            namespace = self._node_namespace(member, mode)
            configured_team = rospy.get_param(namespace + "/team_id", None)
            configured_mode = rospy.get_param(
                namespace + "/mode",
                "leader_follow" if mode == "leader_follow" else None)
            if not self._valid_name(configured_team):
                return None, "{} has no valid configured team_id".format(member)
            if team_id is None:
                team_id = configured_team
            elif configured_team != team_id:
                return None, "members belong to different configured teams"
            if configured_mode != mode:
                return None, "{} is running mode={!r}, expected {!r}".format(
                    member, configured_mode, mode)
            if mode == "consensus":
                configured_members = rospy.get_param(
                    namespace + "/members", [])
                if list(configured_members) != members:
                    return None, "{} has a different consensus member order".format(
                        member)
            else:
                leader = rospy.get_param(namespace + "/leader_name", "")
                if not self._valid_name(leader):
                    return None, "{} has no valid configured leader".format(member)
                if member == leader:
                    return None, "leader {} must not be in the follower members list".format(
                        leader)
                leaders.add(leader)
        if mode == "leader_follow" and len(leaders) != 1:
            return None, "all leader-follow members must use the same leader"
        if mode in self._configured_modes and team_id != \
                self._configured_modes[mode]["team_id"]:
            return None, "running nodes do not match the loaded YAML team_id"
        return team_id, ""

    def _call_set_enabled(self, service_name, enabled):
        rospy.wait_for_service(service_name, timeout=self._service_timeout)
        return rospy.ServiceProxy(service_name, SetBool)(enabled)

    def _clear_acquisition_plan(self, team_id):
        for runtime in self._mode_runtimes.values():
            runtime.clear_acquisition_plan(team_id)
        path = "/formation/{}/acquisition_plan".format(team_id)
        if rospy.has_param(path):
            rospy.delete_param(path)

    def _set_members(self, team_id, mode, members, enabled):
        completed = []
        errors = []
        for member in members:
            service_name = self._service_name(member, mode)
            try:
                result = self._call_set_enabled(service_name, enabled)
                if result.success:
                    completed.append(member)
                else:
                    errors.append("{}: {}".format(member, result.message))
                    if enabled:
                        break
            except (rospy.ROSException, rospy.ServiceException) as error:
                errors.append("{}: {}".format(member, error))
                if enabled:
                    break

        if errors and enabled:
            rollback_errors = []
            for member in reversed(completed):
                try:
                    self._call_set_enabled(
                        self._service_name(member, mode), False)
                except (rospy.ROSException, rospy.ServiceException) as error:
                    rollback_errors.append("{}: {}".format(member, error))
            if rollback_errors:
                errors.extend("rollback " + item for item in rollback_errors)
            if mode == "consensus":
                self._clear_acquisition_plan(team_id)
        if errors:
            raise RuntimeError("; ".join(errors))
        if mode == "consensus" and not enabled:
            self._clear_acquisition_plan(team_id)
        return "team started" if enabled else "team stopped"

    def _handle(self, request):
        with self._lock:
            return self._handle_locked(request)

    def _stop_all(self):
        try:
            code, message, state = rospy.get_master().getSystemState()
            if code != 1:
                return False, message
        except Exception as error:
            return False, str(error)

        formation_services = []
        for service_name, _nodes in state[2]:
            match = _FORMATION_SERVICE.match(service_name)
            if match:
                member, node_mode = match.groups()
                formation_services.append((service_name, member, node_mode))

        teams = set()
        stopped = 0
        errors = []
        for service_name, member, node_mode in sorted(formation_services):
            mode = "consensus" if node_mode == "consensus" else "leader_follow"
            namespace = self._node_namespace(member, mode)
            if self._spawn_members and mode in self._configured_modes:
                team_id = self._configured_modes[mode]["team_id"]
            else:
                team_id = rospy.get_param(namespace + "/team_id", None)
            if self._valid_name(team_id):
                teams.add(team_id)
            try:
                response = self._call_set_enabled(service_name, False)
                if response.success:
                    stopped += 1
                else:
                    errors.append("{}: {}".format(member, response.message))
            except (rospy.ROSException, rospy.ServiceException) as error:
                errors.append("{}: {}".format(member, error))

        for team_id in teams:
            self._clear_acquisition_plan(team_id)
        self._active_group = None

        if not errors and self._spawn_members:
            for mode in tuple(self._mode_runtimes):
                self._shutdown_mode_runtime(mode)

        if errors:
            return False, "stopped {} node(s); {}".format(
                stopped, "; ".join(errors))
        if not formation_services:
            return True, "no formation nodes running"
        rospy.loginfo(
            "[xd_uav_formation] global stop: stopped %d formation node(s)",
            stopped)
        return True, "stopped {} formation node(s)".format(stopped)

    def _handle_stop(self, _request):
        with self._lock:
            success, message = self._stop_all()
            return TriggerResponse(success, message)

    def _handle_locked(self, request):
        if not request.enabled and not (request.mode or "").strip():
            success, message = self._stop_all()
            return FormationControlResponse(success, message)

        requested_mode = (request.mode or "").strip().lower()
        if not request.enabled and self._spawn_members and \
                requested_mode in _MODES and \
                (self._active_group is None or
                 self._active_group[1] != requested_mode):
            return FormationControlResponse(True, "formation already stopped")

        if not request.enabled and (request.mode or "").strip() and \
                not request.members and self._active_group is not None and \
                self._active_group[1] == requested_mode:
            request = copy.copy(request)
            request.members = list(self._active_group[2])

        validated, error = self._resolve_request(request)
        if error:
            return FormationControlResponse(False, error)
        mode, members = validated
        if not request.enabled and self._spawn_members and \
                (self._active_group is None or self._active_group[1] != mode):
            return FormationControlResponse(True, "formation already stopped")
        if not request.enabled and self._spawn_members and \
                self._active_group[2] != members:
            return FormationControlResponse(
                False, "members do not match the active formation")

        team_id, error = self._validate_member_configuration(mode, members)
        if error:
            active_mode = (self._active_group[1]
                           if self._active_group is not None else None)
            if request.enabled and self._spawn_members and \
                    mode != active_mode:
                self._shutdown_mode_runtime(mode)
            return FormationControlResponse(False, error)

        if request.enabled and self._active_group is not None:
            old_team, old_mode, old_members = self._active_group
            if (old_team, old_mode, old_members) != (team_id, mode, members):
                try:
                    self._set_members(old_team, old_mode, old_members, False)
                    self._active_group = None
                    self._shutdown_mode_runtime(old_mode)
                except (rospy.ROSException, rospy.ServiceException,
                        RuntimeError) as stop_error:
                    self._shutdown_mode_runtime(mode)
                    return FormationControlResponse(
                        False, "could not stop previous mode: {}".format(
                            stop_error))

        if request.enabled and self._spawn_members:
            try:
                self._start_configured_members(mode, members)
            except Exception as error:
                return FormationControlResponse(False, str(error))
        elif not request.enabled and self._spawn_members and \
                mode not in self._mode_runtimes:
            return FormationControlResponse(True, "formation already stopped")

        try:
            message = self._set_members(
                team_id, mode, members, request.enabled)
        except (rospy.ROSException, rospy.ServiceException,
                RuntimeError) as error:
            return FormationControlResponse(False, str(error))
        if request.enabled:
            self._active_group = (team_id, mode, members)
        elif self._active_group is not None and \
                self._active_group[1] == mode:
            self._active_group = None
        if not request.enabled and self._spawn_members:
            self._shutdown_mode_runtime(mode)
        action = "enabled" if request.enabled else "disabled"
        rospy.loginfo(
            "[xd_uav_formation] team %s %s (%s): %s",
            team_id, action, mode, message)
        return FormationControlResponse(True, message)


if __name__ == "__main__":
    rospy.init_node("formation_manager")
    FormationManager()
    rospy.spin()
