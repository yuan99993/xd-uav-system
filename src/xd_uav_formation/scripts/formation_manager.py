#!/usr/bin/env python3
"""Stable team-level formation service for consensus and leader-follow modes."""

import re
import threading

import rospy
from std_srvs.srv import SetBool, Trigger, TriggerResponse

from xd_uav_formation.srv import FormationControl, FormationControlResponse


_SIMPLE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")
_FORMATION_SERVICE = re.compile(
    r"^/([A-Za-z0-9_-]+)/formation_(consensus|follower)/set_enabled$")


class FormationManager:
    def __init__(self):
        self._lock = threading.RLock()
        self._service_timeout = max(
            0.1, float(rospy.get_param("~service_timeout", 3.0)))
        self._service = rospy.Service(
            "/formation/set_enabled", FormationControl, self._handle)
        self._stop_service = rospy.Service(
            "/formation/stop", Trigger, self._handle_stop)
        rospy.loginfo(
            "[xd_uav_formation] unified services ready: "
            "/formation/set_enabled and /formation/stop")

    @staticmethod
    def _valid_name(value):
        return isinstance(value, str) and bool(_SIMPLE_NAME.match(value))

    def _validate_request(self, request):
        mode = request.mode.strip().lower()
        members = [name.strip().strip("/") for name in request.members]
        if mode not in ("consensus", "leader_follow"):
            return None, "mode must be consensus or leader_follow"
        if not members or any(not self._valid_name(name) for name in members):
            return None, "members must contain valid UAV namespace names"
        if len(set(members)) != len(members):
            return None, "members contains duplicates"
        if mode == "consensus" and len(members) < 2:
            return None, "consensus requires at least two members"
        return (mode, members), ""

    @staticmethod
    def _node_namespace(member, mode):
        node_name = "formation_consensus" if mode == "consensus" else \
            "formation_follower"
        return "/{}/{}".format(member, node_name)

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
                        member)
                leaders.add(leader)
        if mode == "leader_follow" and len(leaders) != 1:
            return None, "all leader-follow members must use the same leader"
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
        node_name = "formation_consensus" if mode == "consensus" else \
            "formation_follower"
        completed = []
        errors = []
        for member in members:
            service_name = "/{}/{}/set_enabled".format(member, node_name)
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
                        "/{}/{}/set_enabled".format(member, node_name), False)
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

    def _handle_stop(self, _request):
        with self._lock:
            try:
                code, message, state = rospy.get_master().getSystemState()
                if code != 1:
                    return TriggerResponse(False, message)
            except Exception as error:
                return TriggerResponse(False, str(error))

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
                namespace = self._node_namespace(
                    member, "consensus" if node_mode == "consensus" else
                    "leader_follow")
                team_id = rospy.get_param(namespace + "/team_id", None)
                if self._valid_name(team_id):
                    teams.add(team_id)
                try:
                    response = self._call_set_enabled(service_name, False)
                    if response.success:
                        stopped += 1
                    else:
                        errors.append("{}: {}".format(
                            member, response.message))
                except (rospy.ROSException, rospy.ServiceException) as error:
                    errors.append("{}: {}".format(member, error))

            for team_id in teams:
                self._clear_acquisition_plan(team_id)

            if errors:
                return TriggerResponse(
                    False, "stopped {} node(s); {}".format(
                        stopped, "; ".join(errors)))
            if not formation_services:
                return TriggerResponse(True, "no formation nodes running")
            rospy.loginfo(
                "[xd_uav_formation] global stop: stopped %d formation node(s)",
                stopped)
            return TriggerResponse(
                True, "stopped {} formation node(s)".format(stopped))

    def _handle_locked(self, request):
        validated, error = self._validate_request(request)
        if error:
            return FormationControlResponse(False, error)
        mode, members = validated
        team_id, error = self._validate_member_configuration(mode, members)
        if error:
            return FormationControlResponse(False, error)
        try:
            message = self._set_members(
                team_id, mode, members, request.enabled)
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            return FormationControlResponse(False, str(error))
        action = "enabled" if request.enabled else "disabled"
        rospy.loginfo(
            "[xd_uav_formation] team %s %s (%s): %s",
            team_id, action, mode, message)
        return FormationControlResponse(True, message)


if __name__ == "__main__":
    rospy.init_node("formation_manager")
    FormationManager()
    rospy.spin()
