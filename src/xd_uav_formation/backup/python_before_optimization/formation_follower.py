#!/usr/bin/env python3
"""Leader-follow or path-consensus references with shared task preemption."""

import math
import threading

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
from tf.transformations import euler_from_quaternion
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import (
    do_transform_pose, do_transform_vector3)
from trajectory_msgs.msg import MultiDOFJointTrajectory
from xd_uav_controller.msg import ControlState
from xd_uav_task_execute.msg import ExecuteTaskActionGoal

from xd_uav_formation.core import (
    acquisition_slots, blocking_action, catchup_velocity,
    consensus_speed_targets, consensus_velocity,
    formation_offsets, path_reference, path_reference_state,
    rounded_path,
    shared_center, slot_target,
    slot_velocity, validate_offset, virtual_center)


class FormationFollower:
    def __init__(self):
        self._lock = threading.RLock()
        self._node_name = rospy.get_name()
        self._mode = rospy.get_param("~mode", "leader_follow")
        if self._mode not in ("leader_follow", "consensus"):
            raise rospy.ROSInitException("mode must be leader_follow or consensus")
        leader = rospy.get_param("~leader_name", "uav1").strip("/")
        self._leader_name = leader
        follower = rospy.get_namespace().strip("/")
        if not follower or (self._mode == "leader_follow" and
                            (not leader or leader == follower)):
            raise rospy.ROSInitException(
                "launch inside a UAV namespace and use a different leader")
        self._self_name = follower
        self._team_id = rospy.get_param(
            "~team_id", leader if self._mode == "leader_follow" else "team1")
        if not self._team_id or "/" in self._team_id:
            raise rospy.ROSInitException("team_id must be a simple name")
        if self._mode == "consensus":
            self._plan_param = "/formation/{}/acquisition_plan".format(
                self._team_id)
        minimum_slot_distance = float(rospy.get_param(
            "~minimum_slot_distance", 2.0))
        shape = rospy.get_param(
            "~shape", "manual" if self._mode == "leader_follow" else "circle")
        self._shape = shape
        self._members = tuple(rospy.get_param("~members", [follower]))
        if (not self._members or
                len(set(self._members)) != len(self._members) or
                follower not in self._members or
                any(not name or "/" in name for name in self._members)):
            raise rospy.ROSInitException(
                "formation members must be unique UAV namespace names")
        self._common_frame = rospy.get_param(
            "~common_frame", "/world").strip("/")
        if self._mode == "consensus":
            if len(self._members) < 2:
                raise rospy.ROSInitException(
                    "consensus members must be unique UAV namespace names")
            self._spacing = float(rospy.get_param("~spacing", 80.0))
            slots = formation_offsets(
                shape, len(self._members), self._spacing)
            self._slots = slots
            self._slot_offsets = dict(zip(self._members, slots))
            configured_offset = slots[self._members.index(follower)]
            self._waypoints = rospy.get_param("~waypoints", [])
            self._corner_radius = float(rospy.get_param("~corner_radius", 0.0))
            self._cruise_speed = float(rospy.get_param("~cruise_speed", 20.0))
            self._lookahead = float(rospy.get_param(
                "~lookahead_distance", 60.0))
            self._consensus_gain = float(rospy.get_param(
                "~consensus_gain", 0.1))
            self._convergence_time = float(rospy.get_param(
                "~convergence_time", 500.0))
            self._max_consensus_velocity = float(rospy.get_param(
                "~max_consensus_velocity", 4.0))
            self._acquire_consensus_gain = float(rospy.get_param(
                "~acquire_consensus_gain", self._consensus_gain))
            self._acquire_max_consensus_velocity = float(rospy.get_param(
                "~acquire_max_consensus_velocity",
                self._max_consensus_velocity))
            self._acquire_max_speed_adjustment = float(rospy.get_param(
                "~acquire_max_speed_adjustment",
                self._max_consensus_velocity))
            self._formation_heading_rate_limit = float(rospy.get_param(
                "~formation_heading_rate_limit", 0.10))
            self._formation_speed_slew_rate = float(rospy.get_param(
                "~formation_speed_slew_rate", 2.0))
            self._formation_course_slew_rate = float(rospy.get_param(
                "~formation_course_slew_rate", 0.25))
            self._route_correction_gain = float(rospy.get_param(
                "~route_correction_gain", self._consensus_gain))
            self._max_route_correction_velocity = float(rospy.get_param(
                "~max_route_correction_velocity",
                self._max_consensus_velocity))
            self._loop_path = bool(rospy.get_param("~loop_path", True))
            self._acquire_tolerance = float(rospy.get_param(
                "~acquire_tolerance", max(2.0, self._spacing * 0.1)))
            self._acquire_height_tolerance = float(rospy.get_param(
                "~acquire_height_tolerance",
                max(0.75, self._spacing * 0.025)))
            self._acquire_vertical_spacing = float(rospy.get_param(
                "~acquire_vertical_spacing", 8.0))
            self._acquire_timeout = float(rospy.get_param(
                "~acquire_timeout", 180.0))
            if not self._common_frame:
                raise rospy.ROSInitException("common_frame is empty")
            try:
                self._waypoints = rounded_path(
                    self._waypoints, self._corner_radius, self._loop_path)
                path_reference((0.0, 0.0, 0.0), self._waypoints,
                               self._cruise_speed, self._lookahead,
                               self._loop_path)
            except ValueError as error:
                raise rospy.ROSInitException("invalid consensus path: " + str(error))
        elif shape == "manual":
            configured_offset = rospy.get_param("~offset", [-4.0, 0.0, 0.0])
        else:
            slots = formation_offsets(
                shape, int(rospy.get_param("~follower_count", 1)),
                float(rospy.get_param("~spacing", 20.0)))
            slot_index = int(rospy.get_param("~slot_index", 1))
            if slot_index < 1 or slot_index > len(slots):
                raise rospy.ROSInitException("slot_index is outside formation")
            configured_offset = slots[slot_index - 1]
        self._offset = validate_offset(configured_offset,
                                       minimum_slot_distance)
        self._member_offsets = {self._self_name: self._offset}
        if self._mode == "leader_follow":
            configured_member_offsets = rospy.get_param(
                "~member_offsets", {})
            if not isinstance(configured_member_offsets, dict):
                raise rospy.ROSInitException(
                    "member_offsets must map UAV names to [forward,left,up]")
            for member in self._members:
                member_offset = configured_member_offsets.get(member)
                if member_offset is not None:
                    self._member_offsets[member] = validate_offset(
                        member_offset, minimum_slot_distance)
        self._fixedwing_minimum_slot_distance = float(rospy.get_param(
            "~fixedwing_minimum_slot_distance", 30.0))
        self._fixedwing_min_speed = float(rospy.get_param(
            "~fixedwing_min_speed", 11.0))
        self._fixedwing_max_catchup_speed = float(rospy.get_param(
            "~fixedwing_max_catchup_speed", 4.0))
        self._fixedwing_min_groundspeed_for_course = float(rospy.get_param(
            "~fixedwing_min_groundspeed_for_course", 2.0))
        self._multirotor_max_speed = float(rospy.get_param(
            "~multirotor_max_speed", 5.0))
        self._catchup_gain = float(rospy.get_param(
            "~catchup_gain", 0.25))
        self._catchup_ki = float(rospy.get_param("~catchup_ki", 0.01))
        self._catchup_kd = float(rospy.get_param("~catchup_kd", 0.20))
        self._catchup_deadband = float(rospy.get_param(
            "~catchup_deadband", 1.0))
        self._catchup_integral_limit = float(rospy.get_param(
            "~catchup_integral_limit", 50.0))
        self._catchup_derivative_filter_time_constant = float(
            rospy.get_param("~catchup_derivative_filter_time_constant", 0.4))
        self._multirotor_max_catchup_speed = float(rospy.get_param(
            "~multirotor_max_catchup_speed", 3.0))
        self._yaw_offset = float(rospy.get_param("~yaw_offset", 0.0))
        self._rate = float(rospy.get_param("~rate", 20.0))
        self._state_timeout = float(rospy.get_param("~state_timeout", 0.5))
        self._mavros_timeout = float(rospy.get_param("~mavros_timeout", 2.5))
        self._tf_max_age = float(rospy.get_param("~tf_max_age", 1.0))
        self._future_stamp_tolerance = float(rospy.get_param(
            "~future_stamp_tolerance", 0.5))
        self._tf_lookup_timeout = float(rospy.get_param(
            "~tf_lookup_timeout", 0.05))
        self._tf_buffer_cache_time = float(rospy.get_param(
            "~tf_buffer_cache_time", 5.0))
        self._external_quiet_time = float(
            rospy.get_param("~external_quiet_time", 0.5))
        timing_values = (
                self._rate, self._state_timeout, self._mavros_timeout,
                self._tf_max_age, self._future_stamp_tolerance,
                self._tf_lookup_timeout, self._tf_buffer_cache_time,
                self._external_quiet_time,
                self._fixedwing_minimum_slot_distance,
                self._fixedwing_min_speed,
                self._fixedwing_max_catchup_speed,
                self._fixedwing_min_groundspeed_for_course,
                self._multirotor_max_speed, self._catchup_gain,
                self._catchup_ki, self._catchup_kd,
                self._catchup_deadband, self._catchup_integral_limit,
                self._catchup_derivative_filter_time_constant,
                self._multirotor_max_catchup_speed)
        if self._mode == "consensus":
            timing_values += (self._consensus_gain, self._convergence_time,
                              self._max_consensus_velocity,
                              self._acquire_consensus_gain,
                              self._acquire_max_consensus_velocity,
                              self._acquire_max_speed_adjustment,
                              self._formation_heading_rate_limit,
                              self._formation_speed_slew_rate,
                              self._formation_course_slew_rate,
                              self._route_correction_gain,
                              self._max_route_correction_velocity,
                              self._acquire_tolerance,
                              self._acquire_height_tolerance,
                              self._acquire_vertical_spacing,
                              self._acquire_timeout,
                              self._corner_radius)
        if (not all(math.isfinite(value) for value in timing_values) or
                not math.isfinite(self._yaw_offset) or self._rate <= 0 or
                self._state_timeout <= 0 or self._mavros_timeout <= 0 or
                self._tf_max_age <= 0 or
                self._future_stamp_tolerance < 0 or
                self._tf_lookup_timeout < 0 or
                self._tf_buffer_cache_time <= 0 or
                self._external_quiet_time < 0 or
                self._fixedwing_minimum_slot_distance <= 0 or
                self._fixedwing_min_speed <= 0 or
                self._fixedwing_max_catchup_speed < 0 or
                self._fixedwing_min_groundspeed_for_course <= 0 or
                self._multirotor_max_speed <= 0 or
                self._catchup_gain < 0 or
                self._catchup_ki < 0 or self._catchup_kd < 0 or
                self._catchup_deadband < 0 or
                self._catchup_integral_limit < 0 or
                self._catchup_derivative_filter_time_constant < 0 or
                self._multirotor_max_catchup_speed < 0):
            raise rospy.ROSInitException("invalid formation timing or yaw parameters")
        if self._mode == "consensus" and (
                self._consensus_gain < 0 or self._convergence_time <= 0 or
                self._max_consensus_velocity < 0 or
                self._acquire_consensus_gain < 0 or
                self._acquire_max_consensus_velocity < 0 or
                self._acquire_max_speed_adjustment < 0 or
                self._formation_heading_rate_limit <= 0 or
                self._formation_speed_slew_rate <= 0 or
                self._formation_course_slew_rate <= 0 or
                self._route_correction_gain < 0 or
                self._max_route_correction_velocity < 0 or
                self._acquire_tolerance <= 0 or
                self._acquire_height_tolerance <= 0 or
                self._acquire_vertical_spacing <= 0 or
                self._acquire_timeout <= 0 or
                self._corner_radius < 0):
            raise rospy.ROSInitException("invalid consensus gains")

        rospy.loginfo(
            "[xd_uav_formation] mode=%s state_timeout=%.2fs "
            "future_stamp_tolerance=%.2fs tf_max_age=%.2fs "
            "tf_lookup_timeout=%.3fs tf_buffer_cache_time=%.2fs",
            self._mode, self._state_timeout,
            self._future_stamp_tolerance, self._tf_max_age,
            self._tf_lookup_timeout, self._tf_buffer_cache_time)

        self._enabled = False
        self._own_state = None
        self._leader_state = None
        self._own_state_received = None
        self._leader_state_received = None
        self._own_mavros = None
        self._leader_mavros = None
        self._own_mavros_received = None
        self._leader_mavros_received = None
        self._last_external_reference = None
        self._peer_states = {}
        self._peer_state_received = {}
        self._peer_mavros = {}
        self._peer_mavros_received = {}
        self._centers = {}
        self._center_received = {}
        self._peer_statuses = {}
        self._peer_status_received = {}
        self._dropped_members = set()
        self._progress_error_filter = {}
        self._filtered_course = None
        self._filtered_course_stamp = None
        self._filtered_speed = None
        self._paced_speed = None
        self._paced_speed_stamp = None
        self._common_state_cache_stamp = None
        self._common_state_cache = {}
        self._consensus_geometry_cache_key = None
        self._consensus_geometry_cache = None
        self._consensus_heading = None
        self._consensus_heading_stamp = None
        self._catchup_integral = [0.0, 0.0]
        self._catchup_previous_error = None
        self._catchup_filtered_derivative = [0.0, 0.0]
        self._catchup_pid_stamp = None
        self._acquire_stage = None
        self._acquire_started = None
        self._acquire_slots = None
        self._acquire_tiers = None
        self._acquire_initial_xy = None
        self._acquire_heading = None
        self._acquire_altitude = None
        self._reason = "waiting_for_start"
        self._tf = tf2_ros.Buffer(
            cache_time=rospy.Duration(self._tf_buffer_cache_time))
        self._tf_listener = tf2_ros.TransformListener(self._tf)

        self._reference_topic = rospy.get_param(
            "~reference_topic", "control/reference/setpoint")
        self._path_topic = rospy.get_param(
            "~path_topic", "control/reference/path")
        self._trajectory_topic = rospy.get_param(
            "~trajectory_topic", "control/reference/trajectory")
        self._reference_pub = rospy.Publisher(
            self._reference_topic, PositionTarget, queue_size=10)
        self._path_pub = rospy.Publisher(
            self._path_topic, Path, queue_size=1)
        self._trajectory_pub = rospy.Publisher(
            self._trajectory_topic, MultiDOFJointTrajectory, queue_size=1)
        team_cancel_template = rospy.get_param(
            "~team_cancel_topic", "/formation/{team_id}/cancel")
        self._team_cancel_topic = team_cancel_template.format(
            team_id=self._team_id)
        self._team_cancel_pub = rospy.Publisher(
            self._team_cancel_topic, String, queue_size=10)
        self._status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True)
        self._subscribers = [
            rospy.Subscriber("control_manager/state", ControlState,
                             self._own_state_callback, queue_size=10),
            rospy.Subscriber("mavros/state", State,
                             self._own_mavros_callback, queue_size=10),
            rospy.Subscriber(self._reference_topic, PositionTarget,
                             self._external_reference_callback, queue_size=20),
            rospy.Subscriber(self._path_topic, Path,
                             self._other_reference_callback,
                             callback_args="path", queue_size=10),
            rospy.Subscriber(self._trajectory_topic,
                             MultiDOFJointTrajectory,
                             self._other_reference_callback,
                             callback_args="trajectory", queue_size=10),
            rospy.Subscriber(self._team_cancel_topic, String,
                             self._team_cancel_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param(
                "~preempt_topic", "formation/preempt"), String,
                self._preempt_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param(
                "~task_goal_topic", "task_execute/execute/goal"),
                ExecuteTaskActionGoal, self._task_goal_callback,
                queue_size=10),
        ]
        if self._mode == "leader_follow":
            self._subscribers.extend((
                rospy.Subscriber("/{}/control_manager/state".format(leader),
                                 ControlState, self._leader_state_callback,
                                 queue_size=10),
                rospy.Subscriber("/{}/mavros/state".format(leader), State,
                                 self._leader_mavros_callback, queue_size=10),
                rospy.Subscriber(
                    "/{}/task_execute/execute/goal".format(leader),
                    ExecuteTaskActionGoal, self._leader_task_goal_callback,
                    queue_size=10),
                rospy.Subscriber(
                    "/{}/formation/preempt".format(leader), String,
                    self._leader_preempt_callback, queue_size=10)))
            for member in self._members:
                if member == self._self_name:
                    continue
                self._subscribers.extend((
                    rospy.Subscriber(
                        "/{}/control_manager/state".format(member),
                        ControlState, self._peer_state_callback,
                        callback_args=member, queue_size=10),
                    rospy.Subscriber(
                        "/{}/mavros/state".format(member), State,
                        self._peer_mavros_callback,
                        callback_args=member, queue_size=10),
                    rospy.Subscriber(
                        "/{}/formation_follower/status".format(member),
                        String, self._peer_status_callback,
                        callback_args=member, queue_size=10)))
        else:
            self._center_topic = "/formation/{}/{}/center".format(
                self._team_id, self._self_name)
            self._center_pub = rospy.Publisher(
                self._center_topic, PointStamped, queue_size=10)
            for member in self._members:
                if member == self._self_name:
                    continue
                self._subscribers.extend((
                    rospy.Subscriber(
                        "/{}/control_manager/state".format(member),
                        ControlState, self._peer_state_callback,
                        callback_args=member, queue_size=10),
                    rospy.Subscriber(
                        "/{}/mavros/state".format(member), State,
                        self._peer_mavros_callback,
                        callback_args=member, queue_size=10)))
            for member in self._members:
                if member == self._self_name:
                    continue
                self._subscribers.append(rospy.Subscriber(
                    "/formation/{}/{}/center".format(self._team_id, member),
                    PointStamped, self._center_callback,
                    callback_args=member, queue_size=10))
                self._subscribers.append(rospy.Subscriber(
                    "/{}/formation_consensus/status".format(member),
                    String, self._peer_status_callback,
                    callback_args=member, queue_size=10))
        self._service = rospy.Service("~set_enabled", SetBool,
                                      self._set_enabled)
        self._timer = rospy.Timer(rospy.Duration(1.0 / self._rate),
                                  self._tick)
        rospy.on_shutdown(self._timer.shutdown)
        self._publish_status()

    def _publish_status(self):
        self._status_pub.publish(String(
            data=("active" if self._enabled else "inactive") + ":" + self._reason))

    def _mark_peer_dropped(self, member, reason):
        if member == self._self_name or member in self._dropped_members:
            return
        self._dropped_members.add(member)
        rospy.logwarn(
            "[xd_uav_formation] excluding %s after %s",
            member, reason)
        if self._mode == "consensus" and self._enabled:
            # A member leaving during acquisition should not strand the
            # remaining aircraft in a stage whose shared plan still includes
            # that member. Keep each remaining aircraft's original slot and
            # continue on the route with the reduced team.
            if self._acquire_stage != "following":
                self._acquire_stage = "following"
                self._acquire_started = None
            self._reason = "following_degraded:{}:{}".format(member, reason)
            self._publish_status()

    def _active_members(self):
        return tuple(member for member in self._members
                     if member not in self._dropped_members)

    def _refresh_team_membership(self, now):
        if self._mode != "consensus" or not self._enabled:
            return
        for member in self._members:
            if member == self._self_name or member in self._dropped_members:
                continue
            status = self._peer_statuses.get(member, "")
            if status.startswith("inactive:dropout"):
                self._mark_peer_dropped(member, status)
                continue
            state = self._peer_states.get(member)
            received = self._peer_state_received.get(member)
            mavros = self._peer_mavros.get(member)
            mavros_received = self._peer_mavros_received.get(member)
            reason = None
            if not self._fresh(state, received, now):
                reason = "state_stale"
            elif not state.state_valid:
                reason = "state_invalid"
            elif not state.header.frame_id:
                reason = "state_frame_missing"
            elif state.transition_pending:
                reason = "transition_pending"
            elif blocking_action(state.vehicle_action, ControlState):
                reason = "flight_action"
            elif state.airframe_type != self._own_state.airframe_type:
                reason = "airframe_changed"
            elif (state.vehicle_type != self._own_state.vehicle_type or
                  state.flight_regime != self._own_state.flight_regime):
                reason = "flight_regime_changed"
            elif (mavros is None or mavros_received is None or
                  (now - mavros_received).to_sec() > self._mavros_timeout or
                  not mavros.armed or mavros.mode != "OFFBOARD"):
                reason = "left_offboard"
            elif not self._fresh(self._centers.get(member),
                                 self._center_received.get(member), now):
                reason = "center_stale"
            elif self._centers[member].header.frame_id.strip("/") != \
                    self._common_frame:
                reason = "center_frame_mismatch"
            if reason:
                self._mark_peer_dropped(member, reason)

    def _resume_degraded_acquisition(self):
        if self._mode == "consensus" and self._enabled and \
                self._dropped_members and self._acquire_stage != "following":
            self._acquire_stage = "following"
            self._acquire_started = None
            self._reason = "following_degraded"
            self._publish_status()

    def _begin_dropout(self, reason):
        was_enabled = self._enabled
        self._enabled = False
        self._acquire_stage = None
        self._acquire_started = None
        self._dropped_members.add(self._self_name)
        self._reason = "dropout:" + reason
        self._publish_status()
        if was_enabled:
            rospy.logwarn(
                "[xd_uav_formation] member %s left the formation: %s",
                self._self_name, reason)

    def _own_state_callback(self, message):
        with self._lock:
            self._own_state = message
            self._own_state_received = rospy.Time.now()
            if self._enabled and blocking_action(message.vehicle_action,
                                                 ControlState):
                self._begin_dropout("follower_flight_action")

    def _leader_state_callback(self, message):
        with self._lock:
            self._leader_state = message
            self._leader_state_received = rospy.Time.now()
            if self._enabled and blocking_action(message.vehicle_action,
                                                 ControlState):
                self._stop("leader_flight_action", hold=True,
                           broadcast=False)

    def _own_mavros_callback(self, message):
        with self._lock:
            self._own_mavros = message
            self._own_mavros_received = rospy.Time.now()
            if self._enabled and (not message.armed or message.mode != "OFFBOARD"):
                self._begin_dropout("follower_left_offboard")

    def _leader_mavros_callback(self, message):
        with self._lock:
            self._leader_mavros = message
            self._leader_mavros_received = rospy.Time.now()
            if self._enabled and not message.armed:
                self._stop("leader_disarmed", hold=True,
                           broadcast=False)

    def _leader_task_goal_callback(self, message):
        with self._lock:
            if self._enabled:
                self._stop(
                    "leader_task_goal:{}".format(message.goal.task_id),
                    hold=True, broadcast=False)

    def _leader_preempt_callback(self, message):
        with self._lock:
            if self._enabled:
                self._stop("leader_preempt:" + message.data[:128],
                           hold=True, broadcast=False)

    def _peer_state_callback(self, message, member):
        with self._lock:
            self._peer_states[member] = message
            self._peer_state_received[member] = rospy.Time.now()
            if self._enabled and self._mode == "consensus":
                self._refresh_team_membership(rospy.Time.now())

    def _peer_mavros_callback(self, message, member):
        with self._lock:
            self._peer_mavros[member] = message
            self._peer_mavros_received[member] = rospy.Time.now()
            if self._enabled and self._mode == "consensus":
                self._refresh_team_membership(rospy.Time.now())

    def _center_callback(self, message, member):
        with self._lock:
            self._centers[member] = message
            self._center_received[member] = rospy.Time.now()

    def _peer_status_callback(self, message, member):
        with self._lock:
            self._peer_statuses[member] = message.data
            self._peer_status_received[member] = rospy.Time.now()
            if self._enabled and message.data.startswith("inactive:dropout"):
                self._mark_peer_dropped(member, message.data)

    def _external_reference_callback(self, message):
        caller = (getattr(message, "_connection_header", None) or {}).get(
            "callerid", "")
        if caller == self._node_name:
            return
        with self._lock:
            self._last_external_reference = rospy.Time.now()
            if self._enabled:
                self._begin_dropout("external_setpoint_from:" + caller)

    def _other_reference_callback(self, message, source):
        with self._lock:
            self._last_external_reference = rospy.Time.now()
            if self._enabled:
                self._begin_dropout("external_" + source)

    def _team_cancel_callback(self, message):
        if message.data.startswith(self._node_name + ":"):
            return
        with self._lock:
            if self._enabled:
                now = rospy.Time.now()
                own_reference_recent = (
                    self._last_external_reference is not None and
                    (now - self._last_external_reference).to_sec() <
                    self._external_quiet_time)
                self._stop("team_cancel:" + message.data,
                           hold=not own_reference_recent,
                           broadcast=False)

    def _preempt_callback(self, message):
        with self._lock:
            self._last_external_reference = rospy.Time.now()
            if self._enabled:
                self._begin_dropout("preempt:" + message.data[:128])

    def _task_goal_callback(self, message):
        with self._lock:
            self._last_external_reference = rospy.Time.now()
            if self._enabled:
                self._begin_dropout(
                    "task_goal:{}".format(message.goal.task_id))

    def _fresh(self, message, received, now):
        if message is None or received is None or message.header.stamp.is_zero():
            return False
        stamp_age = (now - message.header.stamp).to_sec()
        return (0 <= (now - received).to_sec() <= self._state_timeout and
                -self._future_stamp_tolerance <= stamp_age <=
                self._state_timeout)

    def _readiness(self, now):
        if not self._fresh(self._own_state, self._own_state_received, now):
            return "follower_state_stale"
        own = self._own_state
        if not own.state_valid:
            return "state_invalid"
        if own.airframe_type not in (
                ControlState.AIRFRAME_MULTIROTOR,
                ControlState.AIRFRAME_FIXED_WING,
                ControlState.AIRFRAME_VTOL,
                ControlState.AIRFRAME_TILTROTOR):
            return "unsupported_airframe"
        if own.transition_pending:
            return "transition_pending"
        if own.vehicle_type == ControlState.VEHICLE_MULTIROTOR:
            if own.flight_regime != ControlState.REGIME_HOVER:
                return "hover_regime_required"
        elif own.vehicle_type == ControlState.VEHICLE_FIXEDWING:
            if own.flight_regime != ControlState.REGIME_FORWARD_FLIGHT:
                return "forward_flight_required"
            if math.sqrt(sum(value * value for value in self._offset)) < \
                    self._fixedwing_minimum_slot_distance:
                return "fixedwing_slot_too_close"
        else:
            return "unsupported_vehicle_type"
        if blocking_action(own.vehicle_action, ControlState):
            return "follower_flight_action"
        if not own.header.frame_id:
            return "state_frame_missing"
        if self._own_mavros is None or \
                self._own_mavros_received is None or \
                (now - self._own_mavros_received).to_sec() > \
                self._mavros_timeout or not self._own_mavros.armed or \
                self._own_mavros.mode != "OFFBOARD":
            return "follower_offboard_required"
        if self._mode == "leader_follow":
            peers = (("leader", self._leader_state,
                      self._leader_state_received, self._leader_mavros,
                      self._leader_mavros_received),)
        else:
            active_members = set(self._active_members())
            peers = tuple((member, self._peer_states.get(member),
                           self._peer_state_received.get(member),
                           self._peer_mavros.get(member),
                           self._peer_mavros_received.get(member))
                          for member in self._members
                          if member != self._self_name and
                          member in active_members)
        for name, state, received, mavros, mavros_received in peers:
            if not self._fresh(state, received, now):
                return "{}_state_stale".format(name)
            if not state.state_valid:
                return "state_invalid"
            if own.airframe_type != state.airframe_type:
                return "same_airframe_type_required"
            if state.transition_pending:
                return "transition_pending"
            if own.vehicle_type != state.vehicle_type or \
                    own.flight_regime != state.flight_regime:
                return "same_flight_regime_required"
            if blocking_action(state.vehicle_action, ControlState):
                return "{}_flight_action".format(name)
            if not state.header.frame_id:
                return "state_frame_missing"
            if mavros is None or mavros_received is None or \
                    (now - mavros_received).to_sec() > self._mavros_timeout or \
                    not mavros.armed or \
                    (self._mode == "consensus" and mavros.mode != "OFFBOARD"):
                return "leader_armed_required" if self._mode == \
                    "leader_follow" else "peer_offboard_required:" + name
        if self._mode == "consensus":
            for name in self._active_members():
                if name == self._self_name:
                    continue
                center = self._centers.get(name)
                received = self._center_received.get(name)
                if not self._fresh(center, received, now) or \
                        center.header.frame_id.strip("/") != self._common_frame:
                    return "neighbor_center_stale:" + name
        return None

    def _leader_pose_in_follower_frame(self, now):
        leader = self._leader_state
        pose = PoseStamped()
        pose.header = leader.header
        pose.pose.position = leader.position_odom
        pose.pose.orientation = leader.orientation_odom_body
        source = leader.header.frame_id.strip("/")
        target = self._own_state.header.frame_id.strip("/")
        velocity = Vector3Stamped()
        velocity.header = leader.header
        velocity.vector = leader.velocity_odom
        acceleration = Vector3Stamped()
        acceleration.header = leader.header
        acceleration.vector = leader.acceleration_odom
        if source == target:
            return pose, velocity, acceleration
        transform = self._tf.lookup_transform(
            target, source, rospy.Time(0),
            rospy.Duration(self._tf_lookup_timeout))
        if not transform.header.stamp.is_zero():
            age = (now - transform.header.stamp).to_sec()
            if (age < -self._future_stamp_tolerance or
                    age > self._tf_max_age):
                raise ValueError("leader_to_follower_tf_stale")
        return (do_transform_pose(pose, transform),
                do_transform_vector3(velocity, transform),
                do_transform_vector3(acceleration, transform))

    def _position_in_follower_frame(self, state, now):
        """Transform a team member position into this follower's frame."""
        pose = PoseStamped()
        pose.header = state.header
        pose.pose.position = state.position_odom
        source = state.header.frame_id.strip("/")
        target = self._own_state.header.frame_id.strip("/")
        if not source or not target:
            raise ValueError("formation_member_frame_missing")
        if source != target:
            transform = self._tf.lookup_transform(
                target, source, rospy.Time(0),
                rospy.Duration(0.0))
            if not transform.header.stamp.is_zero():
                age = (now - transform.header.stamp).to_sec()
                if (age < -self._future_stamp_tolerance or
                        age > self._tf_max_age):
                    raise ValueError("formation_member_tf_stale")
            pose = do_transform_pose(pose, transform)
        point = pose.pose.position
        result = (point.x, point.y, point.z)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("formation_member_position_invalid")
        return result

    def _leader_follow_pace_adjustment(self, now, leader_position, heading):
        """Return a shared speed adjustment from each member's slot error.

        A member behind its own slot speeds up. Members already at their
        slots slow relative to the leader while a laggard catches up, but keep
        publishing their slot references throughout the wait.
        """
        members = self._active_members()
        if len(members) < 2:
            return None
        tangent = (math.cos(heading), math.sin(heading))
        normal = (-tangent[1], tangent[0])
        errors = {}
        for member in members:
            offset = self._member_offsets.get(member)
            if offset is None:
                return None
            if member == self._self_name:
                state = self._own_state
            else:
                state = self._peer_states.get(member)
                received = self._peer_state_received.get(member)
                if not self._fresh(state, received, now):
                    return None
            try:
                position = self._position_in_follower_frame(state, now)
            except (ValueError, tf2_ros.TransformException):
                # A missing peer transform should leave the normal individual
                # slot controller in charge until a complete team snapshot is
                # available again.
                return None
            target = slot_target(leader_position, heading, offset)[:3]
            error_x, error_y = (target[0] - position[0],
                                target[1] - position[1])
            along_error = error_x * tangent[0] + error_y * tangent[1]
            cross_error = abs(error_x * normal[0] + error_y * normal[1])
            # Keep a clear ahead/behind sign. If progress is aligned but the
            # member is displaced sideways, still request catch-up speed.
            if abs(along_error) <= 1.0:
                errors[member] = (cross_error if cross_error > 1.0 else
                                  along_error)
            else:
                errors[member] = math.copysign(
                    math.hypot(along_error, cross_error), along_error)

        maximum_lag = max(0.0, max(errors.values()) - 1.0)
        own_error = errors[self._self_name]
        if maximum_lag <= 0.0:
            return None
        # Individual PID tracking speeds up a laggard. Let members already
        # at their slots ease off while that aircraft catches up.
        if own_error > 1.0:
            # None means "do not apply team pacing to this member". Returning
            # 0.0 still enters the caller's speed-rescaling branch and resets
            # a lagging member's PID catch-up speed back to the leader speed.
            return None
        adjustment = -self._catchup_gain * maximum_lag
        maximum_adjustment = (
            self._fixedwing_max_catchup_speed
            if self._own_state.vehicle_type ==
            ControlState.VEHICLE_FIXEDWING else
            self._multirotor_max_catchup_speed)
        return max(-maximum_adjustment,
                   min(maximum_adjustment, adjustment))

    def _reset_catchup_pid(self):
        self._catchup_integral = [0.0, 0.0]
        self._catchup_previous_error = None
        self._catchup_filtered_derivative = [0.0, 0.0]
        self._catchup_pid_stamp = None

    def _pid_catchup_velocity(self, base_velocity, slot_position,
                              own_position, now):
        """Apply PID slot tracking without adding lateral fixed-wing speed."""
        error_x = slot_position[0] - own_position[0]
        error_y = slot_position[1] - own_position[1]
        fixedwing = (self._own_state.vehicle_type ==
                     ControlState.VEHICLE_FIXEDWING)
        base_speed = math.hypot(base_velocity[0], base_velocity[1])
        if fixedwing:
            if base_speed < 1e-6:
                self._reset_catchup_pid()
                return base_velocity
            direction = (base_velocity[0] / base_speed,
                         base_velocity[1] / base_speed)
            errors = [error_x * direction[0] + error_y * direction[1], 0.0]
            max_correction = min(self._fixedwing_max_catchup_speed,
                                 0.5 * base_speed)
        else:
            errors = [error_x, error_y]
            max_correction = self._multirotor_max_catchup_speed

        for index, error in enumerate(errors):
            if abs(error) <= self._catchup_deadband:
                errors[index] = 0.0
            else:
                errors[index] = math.copysign(
                    abs(error) - self._catchup_deadband, error)

        dt = 0.0
        if self._catchup_pid_stamp is not None:
            measured_dt = (now - self._catchup_pid_stamp).to_sec()
            if 1e-3 < measured_dt <= 0.5:
                dt = measured_dt
        if self._catchup_previous_error is None or dt <= 0.0:
            derivative = [0.0, 0.0]
        else:
            raw_derivative = [
                (errors[index] - self._catchup_previous_error[index]) / dt
                for index in range(2)]
            tau = self._catchup_derivative_filter_time_constant
            alpha = 1.0 if tau <= 1e-6 else dt / (tau + dt)
            derivative = [
                self._catchup_filtered_derivative[index] + alpha *
                (raw_derivative[index] -
                 self._catchup_filtered_derivative[index])
                for index in range(2)]
        candidate_integral = list(self._catchup_integral)
        if dt > 0.0:
            for index, error in enumerate(errors):
                if error == 0.0:
                    # Let stored integral unwind after entering the deadband.
                    candidate_integral[index] *= max(0.0, 1.0 - dt / 5.0)
                else:
                    candidate_integral[index] = max(
                        -self._catchup_integral_limit,
                        min(self._catchup_integral_limit,
                            candidate_integral[index] + error * dt))

        def correction_for(integral):
            return [
                self._catchup_gain * errors[index] +
                self._catchup_ki * integral[index] +
                self._catchup_kd * derivative[index]
                for index in range(2)]

        raw_correction = correction_for(candidate_integral)
        raw_norm = math.hypot(*raw_correction)
        saturated = raw_norm > max_correction
        if fixedwing:
            target_speed = base_speed + raw_correction[0]
            saturated = saturated or (
                target_speed > base_speed +
                self._fixedwing_max_catchup_speed or
                target_speed < self._fixedwing_min_speed)
        else:
            target_vx = base_velocity[0] + raw_correction[0]
            target_vy = base_velocity[1] + raw_correction[1]
            saturated = saturated or (
                math.hypot(target_vx, target_vy) >
                self._multirotor_max_speed)
        integral_pushes_saturation = (
            saturated and
            sum(errors[index] * raw_correction[index]
                for index in range(2)) > 0.0)
        if integral_pushes_saturation:
            candidate_integral = list(self._catchup_integral)
            raw_correction = correction_for(candidate_integral)
            raw_norm = math.hypot(*raw_correction)
        self._catchup_integral = candidate_integral
        self._catchup_previous_error = list(errors)
        self._catchup_filtered_derivative = derivative
        self._catchup_pid_stamp = now

        if raw_norm > max_correction and raw_norm > 0.0:
            scale = max_correction / raw_norm
            raw_correction = [value * scale for value in raw_correction]
        if fixedwing:
            minimum_speed = self._fixedwing_min_speed
            maximum_speed = base_speed + self._fixedwing_max_catchup_speed
            target_speed = max(
                minimum_speed,
                min(maximum_speed, base_speed + raw_correction[0]))
            return (direction[0] * target_speed,
                    direction[1] * target_speed, base_velocity[2])

        velocity = (base_velocity[0] + raw_correction[0],
                    base_velocity[1] + raw_correction[1],
                    base_velocity[2])
        corrected_speed = math.hypot(velocity[0], velocity[1])
        if corrected_speed > self._multirotor_max_speed:
            scale = self._multirotor_max_speed / corrected_speed
            velocity = (velocity[0] * scale, velocity[1] * scale,
                        velocity[2])
        return velocity

    @staticmethod
    def _yaw(pose):
        rotation = pose.pose.orientation
        values = (rotation.x, rotation.y, rotation.z, rotation.w)
        if (not all(math.isfinite(value) for value in values) or
                sum(value * value for value in values) < 0.5):
            raise ValueError("invalid_orientation")
        return euler_from_quaternion(values)[2]

    def _position_target(self, position, yaw, now, velocity=None,
                         frame_id=None):
        target = PositionTarget()
        target.header.stamp = now
        target.header.frame_id = frame_id or self._own_state.header.frame_id
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW_RATE)
        if velocity is None:
            target.type_mask |= (PositionTarget.IGNORE_VX |
                                 PositionTarget.IGNORE_VY |
                                 PositionTarget.IGNORE_VZ)
        else:
            target.velocity.x, target.velocity.y, target.velocity.z = velocity
        target.position.x, target.position.y, target.position.z = position
        target.yaw = yaw
        return target

    def _state_in_common_frame(self, state, now):
        stamp = now.to_nsec()
        if self._common_state_cache_stamp != stamp:
            self._common_state_cache_stamp = stamp
            self._common_state_cache.clear()
        cached = self._common_state_cache.get(id(state))
        if cached is not None and cached[0] is state:
            return cached[1]
        pose = PoseStamped()
        pose.header = state.header
        pose.pose.position = state.position_odom
        pose.pose.orientation = state.orientation_odom_body
        velocity = Vector3Stamped()
        velocity.header = state.header
        velocity.vector = state.velocity_odom
        source = state.header.frame_id.strip("/")
        if source != self._common_frame:
            transform = self._tf.lookup_transform(
                self._common_frame, source, rospy.Time(0),
                rospy.Duration(self._tf_lookup_timeout))
            if not transform.header.stamp.is_zero():
                age = (now - transform.header.stamp).to_sec()
                if (age < -self._future_stamp_tolerance or
                        age > self._tf_max_age):
                    raise ValueError("state_to_common_tf_stale")
            pose = do_transform_pose(pose, transform)
            velocity = do_transform_vector3(velocity, transform)
        point = pose.pose.position
        result = (point.x, point.y, point.z)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("position_in_common_frame_invalid")
        transformed = (result, velocity.vector, pose)
        self._common_state_cache[id(state)] = (state, transformed)
        return transformed

    def _member_positions_in_common_frame(self, now):
        return tuple(self._state_in_common_frame(
            self._own_state if member == self._self_name else
            self._peer_states[member], now)[0]
            for member in self._members)

    def _uses_fixedwing_consensus_guidance(self):
        return (self._mode == "consensus" and
                self._own_state is not None and
                self._own_state.vehicle_type == ControlState.VEHICLE_FIXEDWING)

    def _fixedwing_consensus_geometry(self, now):
        """Use one state snapshot and one orientation for every member slot."""
        members = self._active_members()
        states = tuple(self._own_state if name == self._self_name else
                       self._peer_states[name] for name in members)
        key = (now.to_nsec(), members, tuple(id(state) for state in states),
               self._acquire_stage, self._acquire_heading)
        if self._consensus_geometry_cache_key == key:
            return self._consensus_geometry_cache
        transformed = tuple(self._state_in_common_frame(state, now)
                            for state in states)
        positions = tuple(state[0] for state in transformed)
        offsets = tuple(self._slot_offsets[name] for name in members)
        centroid, _ = shared_center(positions)

        capturing = (self._acquire_heading is not None and
                     self._acquire_stage in
                     ("waiting_for_team", "separating", "gathering", "merging"))
        if capturing:
            # Fixed-wings gather on parallel moving slots. Reprojecting the
            # scattered team onto the mission route here makes the capture
            # orientation change before the aircraft can reach their slots.
            heading = self._acquire_heading
            turn_rate = 0.0
            route_position = centroid
            route_vz = 0.0
        else:
            seed_heading = self._consensus_heading
            if seed_heading is None:
                _point, velocity, _rate = path_reference_state(
                    centroid, self._waypoints, self._cruise_speed,
                    self._lookahead, self._loop_path)
                seed_heading = math.atan2(velocity[1], velocity[0])
            estimated_centers = tuple(
                virtual_center(point, seed_heading, offset)
                for point, offset in zip(positions, offsets))
            estimate, _ = shared_center(estimated_centers)
            # Geometry uses the nominal preview for the entire team. Catch-up
            # speed must never change a member's preview point or orientation.
            route_position, route_velocity, _route_rate = path_reference_state(
                estimate, self._waypoints, self._cruise_speed,
                self._lookahead, self._loop_path)
            desired_heading = math.atan2(route_velocity[1], route_velocity[0])
            heading = seed_heading
            turn_rate = 0.0
            if self._consensus_heading_stamp is not None:
                dt = (now - self._consensus_heading_stamp).to_sec()
                if dt > 0.0:
                    dt = min(dt, 0.2)
                    error = math.atan2(math.sin(desired_heading - heading),
                                       math.cos(desired_heading - heading))
                    step = max(-self._formation_heading_rate_limit * dt,
                               min(self._formation_heading_rate_limit * dt,
                                   error))
                    heading += step
                    turn_rate = step / dt
            else:
                heading = desired_heading
            route_vz = route_velocity[2]
        self._consensus_heading = heading
        self._consensus_heading_stamp = now
        centers = tuple(virtual_center(point, heading, offset)
                        for point, offset in zip(positions, offsets))
        mean, largest_error = shared_center(centers)
        geometry = {
            "members": members, "centers": centers, "mean": mean,
            "largest_error": largest_error, "heading": heading,
            "turn_rate": turn_rate, "route_position": route_position,
            "route_vz": route_vz, "capturing": capturing,
        }
        self._consensus_geometry_cache_key = key
        self._consensus_geometry_cache = geometry
        return geometry

    def _own_center_in_common_frame(self, now):
        own = self._own_state
        position, velocity, pose = self._state_in_common_frame(own, now)
        if self._mode == "consensus":
            # Fixed-wing capture uses measured states instead of recursively
            # feeding asynchronous peer heading estimates back into itself.
            if self._uses_fixedwing_consensus_guidance() and all(
                    self._fresh(self._peer_states.get(member),
                                self._peer_state_received.get(member), now) and
                    self._peer_states[member].state_valid and
                    self._peer_states[member].header.frame_id
                    for member in self._active_members()
                    if member != self._self_name):
                geometry = self._fixedwing_consensus_geometry(now)
                return geometry["centers"][
                    geometry["members"].index(self._self_name)]

            # Preserve the existing center estimator for the other airframes.
            active = self._active_members()
            if (not self._uses_fixedwing_consensus_guidance() and
                    len(active) > 1):
                peers = [self._centers.get(name) for name in active
                         if name != self._self_name]
                if all(self._fresh(self._centers.get(name),
                                   self._center_received.get(name), now) and
                       self._centers[name].header.frame_id.strip("/") ==
                       self._common_frame
                       for name in active if name != self._self_name):
                    peer_points = tuple((peer.point.x, peer.point.y,
                                         peer.point.z) for peer in peers)
                    estimate, _ = shared_center(peer_points)
                    own_center = position
                    for _ in range(2):
                        _preview, path_velocity, _rate = path_reference_state(
                            estimate, self._waypoints, self._cruise_speed,
                            self._lookahead, self._loop_path)
                        heading = math.atan2(path_velocity[1], path_velocity[0])
                        own_center = virtual_center(position, heading,
                                                    self._offset)
                        estimate, _ = shared_center(peer_points + (own_center,))
                    return own_center

            # Publish a local estimate before all member states are available.
            center_estimate = position
            heading = None
            for _ in range(2):
                _preview, path_velocity, _turn_rate = path_reference_state(
                    center_estimate, self._waypoints, self._cruise_speed,
                    self._lookahead, self._loop_path)
                heading = math.atan2(path_velocity[1], path_velocity[0])
                center_estimate = virtual_center(
                    position, heading, self._offset)
            return center_estimate
        if own.vehicle_type == ControlState.VEHICLE_FIXEDWING:
            vx, vy = velocity.x, velocity.y
            ground_speed = math.hypot(vx, vy)
            if not all(math.isfinite(value) for value in (vx, vy)):
                raise ValueError("own_speed_too_low_for_fixedwing")
            if own.airspeed_valid:
                if (not math.isfinite(own.airspeed) or
                        own.airspeed < self._fixedwing_min_speed):
                    raise ValueError("own_airspeed_too_low_for_fixedwing")
                if ground_speed >= self._fixedwing_min_groundspeed_for_course:
                    heading = math.atan2(vy, vx)
                else:
                    # Course is ill-conditioned near zero ground speed; a
                    # safely flying fixed-wing can still use its body heading.
                    heading = self._yaw(pose)
            elif ground_speed >= self._fixedwing_min_speed:
                heading = math.atan2(vy, vx)
            else:
                raise ValueError("own_speed_too_low_for_fixedwing")
        else:
            heading = self._yaw(pose)
        return virtual_center(position, heading, self._offset)

    def _publish_center(self, now):
        if not self._fresh(self._own_state, self._own_state_received, now) or \
                not self._own_state.state_valid or \
                not self._own_state.header.frame_id:
            return
        try:
            center = self._own_center_in_common_frame(now)
        except (ValueError, tf2_ros.TransformException):
            return
        message = PointStamped()
        message.header.stamp = now
        message.header.frame_id = self._common_frame
        message.point.x, message.point.y, message.point.z = center
        try:
            self._center_pub.publish(message)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def _consensus_centers(self, now):
        if self._uses_fixedwing_consensus_guidance():
            geometry = self._fixedwing_consensus_geometry(now)
            return (geometry["centers"][
                        geometry["members"].index(self._self_name)],
                    geometry["mean"], geometry["largest_error"])
        center = self._own_center_in_common_frame(now)
        centers = self._team_center_points(center)
        mean, largest_error = shared_center(centers)
        return center, mean, largest_error

    def _team_center_points(self, own_center):
        return tuple(
            own_center if name == self._self_name else
            (self._centers[name].point.x,
             self._centers[name].point.y,
             self._centers[name].point.z)
            for name in self._active_members())

    def _effective_convergence_time(self):
        preview_time = self._lookahead / max(self._cruise_speed, 1e-3)
        return min(self._convergence_time, max(0.1, preview_time))

    def _active_consensus_correction(self):
        if self._acquire_stage in (
                "separating", "gathering", "merging"):
            return (self._acquire_consensus_gain,
                    self._acquire_max_consensus_velocity)
        return self._consensus_gain, self._max_consensus_velocity

    def _paced_path_reference(self, center, speed):
        speed = max(speed, 1e-3)
        lookahead = self._lookahead * speed / max(self._cruise_speed, 1e-3)
        return path_reference_state(
            center, self._waypoints, speed, lookahead, self._loop_path)

    def _team_paced_speed(self, now, center_points, mean, path_velocity,
                          heading, turn_rate):
        """Slow the group for lagging members and speed laggards up."""
        path_speed = math.hypot(path_velocity[0], path_velocity[1])
        active_members = self._active_members()
        if path_speed < 1e-6:
            return path_speed

        tangent = (path_velocity[0] / path_speed,
                   path_velocity[1] / path_speed)
        normal = (-tangent[1], tangent[0])
        raw_progress_errors = []
        for point in center_points:
            error_x = mean[0] - point[0]
            error_y = mean[1] - point[1]
            along_error = error_x * tangent[0] + error_y * tangent[1]
            cross_error = abs(error_x * normal[0] +
                              error_y * normal[1])
            # A member can be behind its formation slot sideways while its
            # along-track progress matches the team mean. In that case the
            # old pacing signal stayed at cruise speed, so it never caught
            # the slot. Preserve the leader/laggard sign when it is clear;
            # otherwise use the horizontal center offset as a positive
            # catch-up signal.
            if abs(along_error) <= 1.0:
                raw_progress_errors.append(cross_error)
            else:
                raw_progress_errors.append(math.copysign(
                    math.hypot(along_error, cross_error), along_error))
        raw_progress_errors = tuple(raw_progress_errors)
        progress_dt = (0.0 if self._paced_speed_stamp is None else
                       max(0.0, min(0.2,
                           (now - self._paced_speed_stamp).to_sec())))
        progress_alpha = progress_dt / (0.8 + progress_dt)
        progress_errors = []
        for name, raw_error in zip(active_members, raw_progress_errors):
            previous_error = self._progress_error_filter.get(name, raw_error)
            filtered_error = previous_error + progress_alpha * (
                raw_error - previous_error)
            self._progress_error_filter[name] = filtered_error
            progress_errors.append(filtered_error)
        progress_errors = tuple(progress_errors)
        member_speeds = []
        for name in active_members:
            state = (self._own_state if name == self._self_name else
                     self._peer_states[name])
            _position, velocity, _pose = self._state_in_common_frame(
                state, now)
            slot_turn = slot_velocity(
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), heading,
                self._slot_offsets[name], turn_rate=turn_rate)
            center_vx = velocity.x - slot_turn[0]
            center_vy = velocity.y - slot_turn[1]
            member_speeds.append(
                center_vx * tangent[0] + center_vy * tangent[1])

        is_fixedwing = (self._own_state.vehicle_type ==
                        ControlState.VEHICLE_FIXEDWING)
        minimum_speed = self._fixedwing_min_speed if is_fixedwing else 0.0
        consensus_gain, _ = self._active_consensus_correction()
        max_speed_adjustment = (
            self._acquire_max_speed_adjustment
            if self._acquire_stage in
            ("separating", "gathering", "merging") else
            self._max_consensus_velocity)
        targets = consensus_speed_targets(
            path_speed, progress_errors, member_speeds,
            consensus_gain, max_speed_adjustment,
            minimum_speed)
        target = targets[active_members.index(self._self_name)]

        # Smooth pace changes so waiting does not turn into repeated speed
        # jumps as noisy position estimates cross the arrival threshold.
        if self._paced_speed is None or self._paced_speed_stamp is None:
            self._paced_speed = target
        else:
            dt = max(0.0, min(0.2, (now - self._paced_speed_stamp).to_sec()))
            # Pace corrections react to sustained lag, while this slower
            # slew prevents noisy center estimates from making fixed-wings
            # repeatedly chase and brake on adjacent control cycles.
            maximum_change = 2.0 * dt
            self._paced_speed += max(
                -maximum_change,
                min(maximum_change, target - self._paced_speed))
        self._paced_speed_stamp = now
        return max(minimum_speed, self._paced_speed)

    def _smooth_course_velocity(self, velocity, now):
        if self._own_state.vehicle_type != ControlState.VEHICLE_FIXEDWING:
            return velocity
        vx, vy, vz = velocity
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            return velocity
        target_course = math.atan2(vy, vx)
        if self._filtered_course is None or \
                self._filtered_course_stamp is None:
            self._filtered_course = target_course
            self._filtered_course_stamp = now
            self._filtered_speed = speed
            return velocity
        dt = max(0.0, min(0.2,
                          (now - self._filtered_course_stamp).to_sec()))
        if self._filtered_speed is None:
            self._filtered_speed = speed
        else:
            speed_slew_rate = (self._formation_speed_slew_rate
                               if self._mode == "consensus" else 2.0)
            speed_step = speed_slew_rate * dt
            self._filtered_speed += max(
                -speed_step,
                min(speed_step, speed - self._filtered_speed))
        # Bound course and speed changes without changing vertical feedforward.
        course_slew_rate = (self._formation_course_slew_rate
                            if self._mode == "consensus" else 0.25)
        max_course_step = course_slew_rate * dt
        course_error = math.atan2(
            math.sin(target_course - self._filtered_course),
            math.cos(target_course - self._filtered_course))
        course_step = max(-max_course_step,
                          min(max_course_step, course_error))
        self._filtered_course += course_step
        self._filtered_course_stamp = now
        return (self._filtered_speed * math.cos(self._filtered_course),
                self._filtered_speed * math.sin(self._filtered_course), vz)

    @staticmethod
    def _cross_track_center(center, mean, heading):
        """Return the shared center with along-track error removed."""
        normal = (-math.sin(heading), math.cos(heading))
        cross_error = ((mean[0] - center[0]) * normal[0] +
                       (mean[1] - center[1]) * normal[1])
        return (center[0] + normal[0] * cross_error,
                center[1] + normal[1] * cross_error,
                mean[2])

    def _consensus_neighbors(self, center, mean, heading):
        active = self._active_members()
        if len(active) < 2:
            return ()
        # All active member states are already exchanged. Use the normalized
        # team mean at every size instead of switching larger teams to a ring.
        return (self._cross_track_center(center, mean, heading),)

    def _team_enabled(self, now):
        return all(
            self._peer_statuses.get(member, "").startswith("active:") and
            self._peer_status_received.get(member) is not None and
            0 <= (now - self._peer_status_received[member]).to_sec() <=
            self._mavros_timeout
            for member in self._active_members()
            if member != self._self_name)

    def _set_acquire_stage(self, stage):
        self._acquire_stage = stage
        self._reason = ("following" if stage == "following" else
                        "acquiring_" + stage)
        self._publish_status()

    def _build_acquisition_plan(self, now):
        positions = self._member_positions_in_common_frame(now)
        mean = tuple(sum(point[axis] for point in positions) /
                     len(positions) for axis in range(3))
        route_point, route_velocity = path_reference(
            mean, self._waypoints, self._cruise_speed,
            self._lookahead, self._loop_path)
        heading = math.atan2(route_velocity[1], route_velocity[0])
        slots = acquisition_slots(
            positions, self._slots, heading, route_point[2])
        base_altitude = math.ceil(max(
            route_point[2], *(point[2] for point in positions)))
        tiers = tuple(
            base_altitude + index * self._acquire_vertical_spacing
            for index in range(len(self._members)))
        return {
            "members": list(self._members),
            "shape": self._shape,
            "spacing": self._spacing,
            "stamp": now.to_sec(),
            "initial_positions": [list(point) for point in positions],
            "slots": [list(slot) for slot in slots],
            "tiers": list(tiers),
            "heading": heading,
            "altitude": route_point[2],
        }

    def _plan_acquisition(self, now):
        plan = rospy.get_param(self._plan_param, None)
        count = len(self._members)
        if (not isinstance(plan, dict) or
                plan.get("members") != list(self._members) or
                plan.get("shape") != self._shape or
                plan.get("spacing") != self._spacing or
                len(plan.get("initial_positions", [])) != count or
                len(plan.get("slots", [])) != count or
                len(plan.get("tiers", [])) != count):
            raise ValueError("shared acquisition plan is missing or mismatched")
        stamp = float(plan["stamp"])
        if not 0 <= (now - rospy.Time.from_sec(stamp)).to_sec() <= \
                self._acquire_timeout:
            raise ValueError("shared acquisition plan has expired")
        positions = tuple(tuple(float(value) for value in point)
                          for point in plan["initial_positions"])
        self._acquire_slots = tuple(tuple(float(value) for value in slot)
                                    for slot in plan["slots"])
        self._acquire_tiers = tuple(float(value) for value in plan["tiers"])
        if (any(len(point) != 3 for point in positions) or
                any(len(slot) != 3 for slot in self._acquire_slots) or
                not all(math.isfinite(value)
                        for point in positions + self._acquire_slots
                        for value in point) or
                not all(math.isfinite(value)
                        for value in self._acquire_tiers)):
            raise ValueError("shared acquisition plan is invalid")
        own_index = self._members.index(self._self_name)
        self._acquire_initial_xy = positions[own_index][:2]
        self._acquire_heading = float(plan["heading"])
        self._acquire_altitude = float(plan["altitude"])
        if not (math.isfinite(self._acquire_heading) and
                math.isfinite(self._acquire_altitude)):
            raise ValueError("shared acquisition heading or altitude is invalid")
        self._acquire_started = rospy.Time.from_sec(stamp)
        _, _, center_error = self._consensus_centers(now)
        if center_error <= self._acquire_tolerance:
            self._set_acquire_stage("following")
        else:
            self._set_acquire_stage("separating")

    def _fixedwing_wait_target(self, now):
        position, velocity, _ = self._state_in_common_frame(
            self._own_state, now)
        heading = math.atan2(velocity.y, velocity.x)
        speed = max(self._fixedwing_min_speed, self._cruise_speed)
        preview = max(self._lookahead, speed * 2.0)
        target = (position[0] + preview * math.cos(heading),
                  position[1] + preview * math.sin(heading),
                  position[2])
        return self._position_target(
            target, heading, now,
            (speed * math.cos(heading), speed * math.sin(heading), 0.0),
            self._common_frame)

    def _fixedwing_gathering_target(self, now, altitude):
        """Capture the team on a shared heading before mission tracking."""
        return self._fixedwing_consensus_target(now, altitude)

    def _acquisition_target(self, now):
        if self._acquire_stage == "waiting_for_team":
            if self._team_enabled(now):
                self._plan_acquisition(now)
            else:
                if self._own_state.vehicle_type == \
                        ControlState.VEHICLE_FIXEDWING:
                    return self._fixedwing_wait_target(now)
                point = self._own_state.position_odom
                pose = PoseStamped()
                pose.pose.orientation = self._own_state.orientation_odom_body
                return self._position_target(
                    (point.x, point.y, point.z), self._yaw(pose), now,
                    (0.0, 0.0, 0.0))
        if self._acquire_stage == "following":
            return self._consensus_target(now)
        positions = self._member_positions_in_common_frame(now)
        own_index = self._members.index(self._self_name)
        if self._acquire_stage == "separating" and all(
                abs(point[2] - tier) <= self._acquire_height_tolerance
                for point, tier in zip(positions, self._acquire_tiers)):
            self._set_acquire_stage("gathering")
        if self._own_state.vehicle_type == ControlState.VEHICLE_FIXEDWING:
            _, _, center_error = self._consensus_centers(now)
            if (self._acquire_stage == "gathering" and
                    center_error <= self._acquire_tolerance):
                self._set_acquire_stage("merging")
            if (self._acquire_stage == "merging" and
                    center_error <= self._acquire_tolerance and all(
                        abs(point[2] - self._acquire_altitude) <=
                        self._acquire_height_tolerance
                        for point in positions)):
                self._set_acquire_stage("following")
            if self._acquire_stage in ("separating", "gathering"):
                return self._fixedwing_gathering_target(
                    now, self._acquire_tiers[own_index])
            target = self._consensus_target(now)
            return target
        slot_error = max(math.hypot(point[0] - slot[0],
                                    point[1] - slot[1])
                         for point, slot in zip(positions,
                                                self._acquire_slots))
        if (self._acquire_stage == "gathering" and
                slot_error <= self._acquire_tolerance):
            self._set_acquire_stage("merging")
        if (self._acquire_stage == "merging" and
                slot_error <= self._acquire_tolerance and all(
                    abs(point[2] - slot[2]) <=
                    self._acquire_height_tolerance
                    for point, slot in zip(positions, self._acquire_slots))):
            self._set_acquire_stage("following")
            return self._consensus_target(now)
        slot = self._acquire_slots[own_index]
        if self._acquire_stage == "separating":
            target = (*self._acquire_initial_xy,
                      self._acquire_tiers[own_index])
        elif self._acquire_stage == "gathering":
            target = (slot[0], slot[1],
                      self._acquire_tiers[own_index])
        else:
            target = slot
        return self._position_target(
            target, self._acquire_heading + self._yaw_offset, now,
            (0.0, 0.0, 0.0), self._common_frame)

    def _fixedwing_consensus_target(self, now, altitude=None):
        """Capture moving slots using signed errors, without point chasing."""
        geometry = self._fixedwing_consensus_geometry(now)
        index = geometry["members"].index(self._self_name)
        center = geometry["centers"][index]
        mean = geometry["mean"]
        heading = geometry["heading"]
        tangent = (math.cos(heading), math.sin(heading))
        normal = (-tangent[1], tangent[0])
        error = (mean[0] - center[0], mean[1] - center[1])
        along_error = error[0] * tangent[0] + error[1] * tangent[1]
        cross_error = error[0] * normal[0] + error[1] * normal[1]
        gain, lateral_limit = self._active_consensus_correction()
        speed_limit = (self._acquire_max_speed_adjustment
                       if self._acquire_stage in
                       ("separating", "gathering", "merging") else
                       self._max_consensus_velocity)

        # The along-track error retains its sign. Turning lateral displacement
        # into a positive lag makes every displaced aircraft accelerate, and
        # using one maximum-lag adjustment for all members causes overshoot.
        along_error = math.copysign(
            max(0.0, abs(along_error) - self._catchup_deadband), along_error)
        speed_adjustment = max(-speed_limit,
                               min(speed_limit, gain * along_error))
        lateral_adjustment = max(-lateral_limit,
                                 min(lateral_limit, gain * cross_error))

        route_adjustment = 0.0
        if not geometry["capturing"]:
            route_point = geometry["route_position"]
            route_error = ((route_point[0] - mean[0]) * normal[0] +
                           (route_point[1] - mean[1]) * normal[1])
            route_adjustment = max(
                -self._max_route_correction_velocity,
                min(self._max_route_correction_velocity,
                    self._route_correction_gain * route_error))

        # Differentiate the shared, smoothly rotating offset before applying
        # individual capture corrections. Every aircraft uses the same heading
        # and angular rate, even when their commanded speeds differ.
        velocity = slot_velocity(
            (self._cruise_speed * tangent[0],
             self._cruise_speed * tangent[1], geometry["route_vz"]),
            (0.0, 0.0, 0.0), heading, self._offset,
            turn_rate=geometry["turn_rate"])
        along_speed = max(
            self._fixedwing_min_speed,
            velocity[0] * tangent[0] + velocity[1] * tangent[1] +
            speed_adjustment)
        cross_speed = (velocity[0] * normal[0] + velocity[1] * normal[1] +
                       lateral_adjustment + route_adjustment)
        velocity = self._smooth_course_velocity(
            (tangent[0] * along_speed + normal[0] * cross_speed,
             tangent[1] * along_speed + normal[1] * cross_speed,
             geometry["route_vz"]), now)

        slot = slot_target(mean, heading, self._offset, self._yaw_offset)
        if altitude is not None:
            target_altitude = altitude
        elif geometry["capturing"]:
            target_altitude = self._acquire_altitude + self._offset[2]
        else:
            target_altitude = geometry["route_position"][2] + self._offset[2]
        target = self._position_target(
            (slot[0], slot[1], target_altitude), slot[3], now,
            velocity, self._common_frame)
        # The downstream fixed-wing controller adds another lookahead to XY
        # position targets. A slot behind an ahead aircraft would then command
        # a U-turn despite the intended slowdown. Supply horizontal velocity
        # guidance only; keep altitude feedback active.
        target.type_mask |= PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY
        return target

    def _consensus_target(self, now):
        if self._uses_fixedwing_consensus_guidance():
            return self._fixedwing_consensus_target(now)
        center, mean, _ = self._consensus_centers(now)
        center_points = self._team_center_points(center)
        _nominal_position, nominal_velocity, nominal_turn_rate = \
            path_reference_state(
                mean, self._waypoints, self._cruise_speed,
                self._lookahead, self._loop_path)
        heading = math.atan2(nominal_velocity[1], nominal_velocity[0])
        paced_speed = self._team_paced_speed(
            now, center_points, mean, nominal_velocity, heading,
            nominal_turn_rate)
        position, paced_path_velocity, paced_turn_rate = \
            self._paced_path_reference(mean, paced_speed)
        heading = math.atan2(paced_path_velocity[1],
                             paced_path_velocity[0])
        x, y, z, yaw = slot_target(position, heading, self._offset,
                                   self._yaw_offset)
        neighbors = self._consensus_neighbors(center, mean, heading)
        fixedwing = (self._own_state.vehicle_type ==
                     ControlState.VEHICLE_FIXEDWING)
        consensus_gain, correction_limit = \
            self._active_consensus_correction()
        velocity = (consensus_velocity(
            # Position is a lookahead preview; it is not the consensus center.
            # Using it here makes the commanded course chase moving preview
            # points and can produce a weaving fixed-wing track.
            paced_path_velocity, center, center, neighbors,
            consensus_gain,
            self._effective_convergence_time(),
            correction_limit)
            if len(self._active_members()) > 1 else paced_path_velocity)
        velocity = slot_velocity(
            velocity, (0.0, 0.0, 0.0), heading, self._offset,
            turn_rate=paced_turn_rate)
        # Consensus already adds neighbor-center error, so avoid adding a
        # second correction toward the preview point.
        velocity = catchup_velocity(
            velocity, position, position, 0.0,
            None if fixedwing else 0.0,
            None if fixedwing else self._multirotor_max_speed)
        velocity = self._smooth_course_velocity(velocity, now)
        return self._position_target((x, y, z), yaw, now, velocity,
                                     self._common_frame)

    def _formation_target(self, now):
        if self._mode == "consensus":
            return self._consensus_target(now)
        pose, leader_velocity, leader_acceleration = \
            self._leader_pose_in_follower_frame(now)
        position = pose.pose.position
        vector = leader_velocity.vector
        leader_velocity_xyz = (vector.x, vector.y, vector.z)
        if not all(math.isfinite(value) for value in leader_velocity_xyz):
            raise ValueError("leader_velocity_invalid")
        fixedwing = (self._own_state.vehicle_type ==
                     ControlState.VEHICLE_FIXEDWING)
        speed = math.hypot(vector.x, vector.y)
        # Swiftwing rotates its slots using the leader's ground-track heading.
        # Hovering rotors can translate sideways, so their body yaw is the
        # meaningful orientation for a visual formation instead.
        if fixedwing and speed >= self._fixedwing_min_groundspeed_for_course:
            heading = math.atan2(vector.y, vector.x)
        else:
            # Ground course is undefined at very low speed. Use the leader's
            # nose direction so followers can start before it reaches its
            # first commanded point.
            heading = self._yaw(pose)
        x, y, z, yaw = slot_target(
            (position.x, position.y, position.z), heading,
            self._offset, self._yaw_offset)
        acceleration = leader_acceleration.vector
        leader_acceleration_xyz = (
            (acceleration.x, acceleration.y, acceleration.z)
            if self._leader_state.acceleration_fresh else (0.0, 0.0, 0.0))
        turn_rate = None
        if fixedwing and not self._leader_state.acceleration_fresh:
            # Swiftwing estimates curvature from the leader's bank angle.
            self._yaw(pose)
            rotation = pose.pose.orientation
            bank = euler_from_quaternion((rotation.x, rotation.y,
                                          rotation.z, rotation.w))[0]
            turn_rate = 9.8 * math.tan(bank) / max(
                speed, self._fixedwing_min_speed)
        elif not fixedwing and math.isfinite(self._leader_state.body_rate.z):
            turn_rate = self._leader_state.body_rate.z
        velocity = slot_velocity(
            leader_velocity_xyz, leader_acceleration_xyz,
            heading, self._offset,
            turn_rate=turn_rate)
        base_horizontal_speed = math.hypot(velocity[0], velocity[1])
        own_position = self._own_state.position_odom
        velocity = self._pid_catchup_velocity(
            velocity, (x, y, z),
            (own_position.x, own_position.y, own_position.z), now)
        pace_adjustment = self._leader_follow_pace_adjustment(
            now, (position.x, position.y, position.z), heading)
        if pace_adjustment is not None:
            minimum_speed = (self._fixedwing_min_speed if fixedwing else
                             0.0)
            pacing_base_speed = max(base_horizontal_speed, minimum_speed)
            maximum_speed = (pacing_base_speed +
                             self._fixedwing_max_catchup_speed
                             if fixedwing else
                             self._multirotor_max_speed)
            paced_speed = max(
                minimum_speed,
                min(maximum_speed,
                    pacing_base_speed + pace_adjustment))
            corrected_speed = math.hypot(velocity[0], velocity[1])
            if corrected_speed > 1e-3:
                scale = paced_speed / corrected_speed
                velocity = (velocity[0] * scale,
                            velocity[1] * scale, velocity[2])
        velocity = self._smooth_course_velocity(velocity, now)
        return self._position_target((x, y, z), yaw, now, velocity)

    def _publish_hold(self):
        now = rospy.Time.now()
        if (not self._fresh(self._own_state, self._own_state_received, now) or
                not self._own_state.state_valid or
                self._own_state.vehicle_type !=
                ControlState.VEHICLE_MULTIROTOR or
                self._own_mavros is None or not self._own_mavros.armed or
                self._own_mavros.mode != "OFFBOARD" or
                blocking_action(self._own_state.vehicle_action, ControlState)):
            return
        try:
            pose = PoseStamped()
            pose.pose.orientation = self._own_state.orientation_odom_body
            point = self._own_state.position_odom
            self._reference_pub.publish(self._position_target(
                (point.x, point.y, point.z), self._yaw(pose), now))
        except ValueError:
            pass

    def _stop(self, reason, hold, broadcast=True):
        was_enabled = self._enabled
        self._enabled = False
        self._acquire_stage = None
        self._acquire_started = None
        self._reason = reason
        if hold:
            self._publish_hold()
        self._publish_status()
        if was_enabled and broadcast:
            self._team_cancel_pub.publish(String(
                data=self._node_name + ":" + reason))
        rospy.loginfo("[xd_uav_formation] stopped: %s", reason)

    def _set_enabled(self, request):
        with self._lock:
            if not request.data:
                self._stop("stopped_by_service", hold=self._enabled)
                return SetBoolResponse(True, "formation stopped")
            if self._enabled:
                return SetBoolResponse(True, "formation already active")
            now = rospy.Time.now()
            if self._last_external_reference is not None and \
                    (now - self._last_external_reference).to_sec() < \
                    self._external_quiet_time:
                return SetBoolResponse(False, "another reference is active")
            reason = self._readiness(now)
            if reason:
                return SetBoolResponse(False, reason)
            try:
                if self._mode == "consensus":
                    self._member_positions_in_common_frame(now)
                    if self._self_name == self._members[0]:
                        rospy.set_param(
                            self._plan_param,
                            self._build_acquisition_plan(now))
                    elif not rospy.has_param(self._plan_param):
                        return SetBoolResponse(
                            False,
                            "start the team through /formation/set_enabled")
                else:
                    self._formation_target(now)
            except (ValueError, rospy.ROSException,
                    tf2_ros.TransformException) as error:
                return SetBoolResponse(False, "transform unavailable: " + str(error))
            self._dropped_members.clear()
            self._enabled = True
            if self._mode == "consensus":
                self._paced_speed = None
                self._paced_speed_stamp = None
                self._progress_error_filter.clear()
                self._consensus_geometry_cache_key = None
                self._consensus_geometry_cache = None
                self._consensus_heading = None
                self._consensus_heading_stamp = None
                self._acquire_heading = None
                self._acquire_stage = "waiting_for_team"
                self._acquire_started = now
                self._reason = "acquiring_waiting_for_team"
            else:
                self._reason = "following"
            self._filtered_course = None
            self._filtered_course_stamp = None
            self._filtered_speed = None
            self._reset_catchup_pid()
            self._publish_status()
            return SetBoolResponse(
                True, "formation gathering started" if
                self._mode == "consensus" else "formation started")

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        with self._lock:
            now = rospy.Time.now()
            if self._mode == "consensus":
                self._publish_center(now)
                self._refresh_team_membership(now)
            if not self._enabled:
                return
            self._publish_status()
            reason = self._readiness(now)
            if reason:
                if self._is_own_readiness_failure(reason):
                    self._begin_dropout(reason)
                else:
                    self._stop(reason, hold=True, broadcast=False)
                return
            if self._mode == "consensus" and \
                    self._acquire_stage != "waiting_for_team" and \
                    not self._team_enabled(now):
                self._begin_dropout("peer_formation_stopped")
                return
            if self._mode == "consensus" and \
                    self._acquire_stage != "following" and \
                    self._acquire_started is not None and \
                    (now - self._acquire_started).to_sec() > \
                    self._acquire_timeout:
                self._begin_dropout("acquisition_timeout")
                return
            try:
                target = (self._acquisition_target(now)
                          if self._mode == "consensus" else
                          self._formation_target(now))
            except tf2_ros.TransformException as error:
                self._begin_dropout("transform_unavailable:" + str(error))
                return
            except ValueError as error:
                self._begin_dropout("invalid_formation_state:" + str(error))
                return
            self._reference_pub.publish(target)

    def _is_own_readiness_failure(self, reason):
        if self._mode == "leader_follow" and reason in (
                "same_airframe_type_required",
                "same_flight_regime_required"):
            return False
        return reason in (
            "follower_state_stale", "state_invalid", "unsupported_airframe",
            "transition_pending", "hover_regime_required",
            "forward_flight_required", "fixedwing_slot_too_close",
            "unsupported_vehicle_type", "follower_flight_action",
            "state_frame_missing", "follower_offboard_required",
            "same_airframe_type_required", "same_flight_regime_required")


if __name__ == "__main__":
    rospy.init_node("formation_follower")
    FormationFollower()
    rospy.spin()
