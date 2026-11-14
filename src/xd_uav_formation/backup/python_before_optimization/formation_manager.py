#!/usr/bin/env python3
"""Configured team-level formation service and member process launcher."""

import copy
import math
import os
import re
import threading

import roslaunch
import roslaunch.core
import roslaunch.rlutil
import roslaunch.scriptapi
import rospy
import yaml
from std_srvs.srv import SetBool, Trigger, TriggerResponse

from xd_uav_formation.srv import FormationControl, FormationControlResponse


_SIMPLE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")
_FORMATION_SERVICE = re.compile(
    r"^/([A-Za-z0-9_-]+)/formation_(consensus|follower)/set_enabled$")
_MODES = ("leader_follow", "consensus")


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
        self._child_launch = None
        self._child_processes = []
        self._configured_modes = {}

        config_path = rospy.get_param("~config", "")
        common_config_path = rospy.get_param("~common_config", "")
        self._profile = _load_yaml(config_path)
        common_parameters = _load_yaml(common_config_path)
        common_parameters.update(self._profile.get("common", {}) or {})
        self._service_timeout = float(rospy.get_param(
            "~service_timeout", common_parameters.get("service_timeout", 3.0)))
        if not math.isfinite(self._service_timeout) or \
                self._service_timeout <= 0:
            raise ValueError("service_timeout must be a positive finite number")
        self._load_mode_configs(self._profile, common_parameters)

        if rospy.get_param("~spawn_members", False):
            if not self._configured_modes:
                raise ValueError("formation config has no configured mode")
            self._start_configured_members()

        self._service = rospy.Service(
            "/formation/set_enabled", FormationControl, self._handle)
        self._stop_service = rospy.Service(
            "/formation/stop", Trigger, self._handle_stop)
        rospy.on_shutdown(self._shutdown_children)
        rospy.loginfo(
            "[xd_uav_formation] unified services ready: "
            "/formation/set_enabled and /formation/stop; configured modes=%s",
            ",".join(self._configured_modes) or "manual")

    @staticmethod
    def _valid_name(value):
        return isinstance(value, str) and bool(_SIMPLE_NAME.match(value))

    @staticmethod
    def _normalise_mode_config(mode_config, common_parameters, root_config):
        if not isinstance(mode_config, dict):
            raise ValueError("mode configuration must be a mapping")
        result = copy.deepcopy(common_parameters)
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
            if not isinstance(members, list) or not members or any(
                    not self._valid_name(member) for member in members):
                raise ValueError(
                    "{} config must contain valid members".format(mode))
            if len(set(members)) != len(members):
                raise ValueError("{} members contains duplicates".format(mode))
            if mode == "consensus" and len(members) < 2:
                raise ValueError("consensus requires at least two members")
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
            self._configured_modes[mode] = config

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

    def _start_configured_members(self):
        code, message, state = rospy.get_master().getSystemState()
        if code != 1:
            raise RuntimeError("cannot inspect ROS master: " + message)
        existing_services = {name for name, _nodes in state[2]}
        planned = []
        for mode, config in self._configured_modes.items():
            for member in config["members"]:
                service_name = self._service_name(member, mode)
                if service_name in existing_services:
                    raise RuntimeError(
                        "{} already exists; stop the old formation nodes first".format(
                            service_name))
                planned.append((mode, member, service_name))

        launch_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(launch_uuid)
        self._child_launch = roslaunch.scriptapi.ROSLaunch()
        self._child_launch.start()
        try:
            for mode, member, _service_name in planned:
                config = self._configured_modes[mode]
                node_name = "formation_consensus" if mode == "consensus" \
                    else "formation_follower"
                namespace = self._node_namespace(member, mode)
                parameters = copy.deepcopy(config)
                parameters.pop("offsets", None)
                parameters.pop("common", None)
                parameters.pop("service_timeout", None)
                if mode == "leader_follow":
                    parameters["offset"] = list(config["offsets"][member])
                    parameters["member_offsets"] = {
                        name: list(offset)
                        for name, offset in config["offsets"].items()}
                rospy.set_param(namespace, parameters)
                node = roslaunch.core.Node(
                    "xd_uav_formation", "formation_follower.py",
                    name=node_name, namespace="/" + member,
                    output="screen")
                self._child_processes.append(self._child_launch.launch(node))
                rospy.loginfo(
                    "[xd_uav_formation] launched %s in /%s",
                    node_name, member)
        except Exception:
            self._shutdown_children()
            raise

    def _shutdown_children(self):
        if self._child_launch is not None:
            try:
                self._child_launch.stop()
            except Exception as error:
                rospy.logwarn(
                    "[xd_uav_formation] child shutdown error: %s", error)
            self._child_launch = None
            self._child_processes = []

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

        if self._configured_modes:
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

    @staticmethod
    def _clear_acquisition_plan(team_id):
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

        validated, error = self._resolve_request(request)
        if error:
            return FormationControlResponse(False, error)
        mode, members = validated
        team_id, error = self._validate_member_configuration(mode, members)
        if error:
            return FormationControlResponse(False, error)

        if request.enabled and self._active_group is not None:
            old_team, old_mode, old_members = self._active_group
            if (old_team, old_mode, old_members) != (team_id, mode, members):
                try:
                    self._set_members(old_team, old_mode, old_members, False)
                    self._active_group = None
                except (rospy.ROSException, rospy.ServiceException,
                        RuntimeError) as stop_error:
                    return FormationControlResponse(
                        False, "could not stop previous mode: {}".format(
                            stop_error))

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
        action = "enabled" if request.enabled else "disabled"
        rospy.loginfo(
            "[xd_uav_formation] team %s %s (%s): %s",
            team_id, action, mode, message)
        return FormationControlResponse(True, message)


if __name__ == "__main__":
    rospy.init_node("formation_manager")
    FormationManager()
    rospy.spin()
