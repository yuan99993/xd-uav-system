#!/usr/bin/env python3
"""Translate an EGO goal/health/odometry stream into PlannerStatus."""

import copy
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from xd_uav_controller.msg import ControlCommand
from xd_uav_task_allocate.msg import PlannerStatus
from xd_uav_task_allocate.srv import (
    CancelPlanning, CancelPlanningResponse, GetMissionState)

from xd_uav_planning.core import (
    PathSample, arrival_reached, path_length, project_path_progress,
    validate_path)


class EgoStatus:
    def __init__(self):
        self._common_frame = rospy.get_param("~common_frame", "world").strip("/")
        self._uav_name = rospy.get_param(
            "~uav_name", rospy.get_namespace()).strip("/")
        self._task_state_service = rospy.get_param(
            "~task_state_service", "/task_allocate/get_state")
        self._position_tolerance = float(
            rospy.get_param("~position_tolerance", 0.50))
        self._speed_tolerance = float(
            rospy.get_param("~speed_tolerance", 0.35))
        self._arrival_dwell = float(rospy.get_param("~arrival_dwell", 1.0))
        self._goal_timeout = float(rospy.get_param("~goal_timeout", 180.0))
        # A complete Path can legitimately take much longer than the legacy
        # single-goal timeout, especially while EGO replans around obstacles.
        # Route mode therefore uses a progress watchdog instead of a fixed
        # wall-clock deadline.  Single PoseStamped goals keep goal_timeout.
        self._route_stall_timeout = float(rospy.get_param(
            "~route_stall_timeout", 30.0))
        self._route_stall_recovery_enabled = bool(rospy.get_param(
            "~route_stall_recovery_enabled", True))
        # Zero means unlimited recoveries. The route remains owned by the
        # allocator and can always be reattached from current odometry.
        self._route_stall_max_recoveries = int(rospy.get_param(
            "~route_stall_max_recoveries", 0))
        self._route_progress_epsilon = float(rospy.get_param(
            "~route_progress_epsilon_m", 0.25))
        self._flight_type = int(rospy.get_param("~flight_type", 1))
        self._output_gate_service = rospy.get_param(
            "~output_gate_service", "").strip()
        self._owner_selection_service = rospy.get_param(
            "~owner_selection_service", "").strip()
        self._owner_acquisition_timeout = float(rospy.get_param(
            "~owner_acquisition_timeout", 5.0))
        # The controller owns takeoff. Acquiring EGO ownership before the
        # vehicle is airborne would cancel its internal takeoff reference.
        self._gate_owner_until_takeoff = bool(rospy.get_param(
            "~gate_owner_until_takeoff", False))
        self._startup_ground_height = float(rospy.get_param(
            "~startup_ground_height", -0.01))
        self._startup_min_altitude = float(rospy.get_param(
            "~startup_min_altitude", 0.60))
        self._startup_max_vertical_speed = float(rospy.get_param(
            "~startup_max_vertical_speed", 0.45))
        self._control_command_topic = rospy.get_param(
            "~control_command_topic", "").strip()
        self._odom_timeout = float(rospy.get_param("~odom_timeout", 0.30))
        self._path_timeout = float(rospy.get_param("~path_timeout", 2.0))
        self._path_progress_backtrack = float(rospy.get_param(
            "~path_progress_backtrack_m", 1.0))
        self._path_progress_search = float(rospy.get_param(
            "~path_progress_search_window_m", 25.0))
        self._goal = None
        self._goal_id = None
        self._route_points = None
        self._route_message = None
        self._route_recovery_attempts = 0
        self._route_progress = 0.0
        self._route_length = 0.0
        self._last_route_progress = 0.0
        self._last_route_progress_time = None
        self._goal_received = None
        self._healthy = False
        self._health_stamp = None
        self._last_odom_stamp = None
        self._last_position = None
        self._last_vertical_speed = None
        self._takeoff_active = False
        self._landing_active = False
        self._last_control_command_stamp = None
        self._arrival_since = None
        self._terminal = False
        self._last_state = None
        self._owner_selected_for_goal = not bool(self._owner_selection_service)

        self._publisher = rospy.Publisher(
            rospy.get_param("~status_topic", "planning/status"),
            PlannerStatus, queue_size=10, latch=True)
        self._ego_goal_publisher = rospy.Publisher(
            rospy.get_param("~ego_goal_topic", "ego/validated_goal"),
            PoseStamped, queue_size=1, latch=True)
        self._ego_path_publisher = rospy.Publisher(
            rospy.get_param("~ego_path_topic", "ego/validated_path"),
            Path, queue_size=1, latch=True)
        self._goal_subscriber = rospy.Subscriber(
            rospy.get_param("~goal_topic", "planning/goal"),
            PoseStamped, self._goal_callback, queue_size=2)
        self._path_subscriber = rospy.Subscriber(
            rospy.get_param("~path_topic", "planning/task_path"),
            Path, self._path_callback, queue_size=2)
        self._health_subscriber = rospy.Subscriber(
            rospy.get_param("~health_topic", "planning/healthy"),
            Bool, self._health_callback, queue_size=1)
        self._odom_subscriber = rospy.Subscriber(
            rospy.get_param("~odometry_topic", "ego/odometry"),
            Odometry, self._odometry_callback, queue_size=20)
        self._control_command_subscriber = None
        if self._control_command_topic:
            self._control_command_subscriber = rospy.Subscriber(
                self._control_command_topic, ControlCommand,
                self._control_command_callback, queue_size=5)
        self._candidate_subscriber = rospy.Subscriber(
            rospy.get_param("~candidate_topic", "ego/reference_candidate"),
            PositionTarget, self._candidate_callback, queue_size=20)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._timer_callback)
        self._cancel_service = rospy.Service(
            rospy.get_param("~cancel_service", "planning/cancel"),
            CancelPlanning, self._cancel_callback)

    @staticmethod
    def _finite(values):
        return all(math.isfinite(value) for value in values)

    def _goal_callback(self, message):
        frame = message.header.frame_id.strip("/")
        point = message.pose.position
        values = (float(point.x), float(point.y), float(point.z))
        goal_id = self._resolve_goal_id(message)
        if frame != self._common_frame or not self._finite(values):
            reason = ("goal_frame_mismatch" if frame != self._common_frame
                      else "goal_not_finite")
            self._publish(goal_id, PlannerStatus.FAILED, reason)
            return
        if self._flight_type != 1:
            self._publish(goal_id, PlannerStatus.FAILED,
                          "live_goal_requires_ego_manual_target_mode")
            return
        # Quarantine any previous trajectory before handing the new goal to
        # EGO. Output resumes only after this goal produces a fresh candidate.
        if not self._set_output_enabled(False):
            self._publish(goal_id, PlannerStatus.FAILED,
                          "planning_output_gate_unavailable")
            return
        self._goal = values
        self._goal_id = goal_id
        self._route_points = None
        self._route_message = None
        self._route_recovery_attempts = 0
        self._route_progress = 0.0
        self._route_length = 0.0
        self._goal_received = rospy.Time.now()
        self._arrival_since = None
        self._terminal = False
        self._owner_selected_for_goal = not bool(self._owner_selection_service)
        self._last_state = PlannerStatus.PLANNING
        self._publish(goal_id, PlannerStatus.PLANNING, "EGO goal accepted by planning layer")
        # EGO consumes only the planning layer's validated internal topic.
        self._ego_goal_publisher.publish(message)

    def _path_callback(self, message):
        """Accept one complete route and forward the validated copy to EGO."""
        goal_id = self._resolve_path_goal_id(message)
        points = tuple(
            (float(p.pose.position.x), float(p.pose.position.y),
             float(p.pose.position.z)) for p in message.poses)
        pose_frames = tuple(p.header.frame_id.strip("/")
                            for p in message.poses)
        sample = PathSample(
            stamp=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id.strip("/"),
            points=points,
            pose_frames=pose_frames)
        validation = validate_path(
            sample, rospy.Time.now().to_sec(), self._common_frame,
            self._path_timeout, 0.25)
        if not validation.valid:
            self._publish(goal_id, PlannerStatus.FAILED,
                          validation.reason)
            return
        if self._flight_type not in (1, 3):
            self._publish(goal_id, PlannerStatus.FAILED,
                          "path_requires_ego_route_target_mode")
            return
        if not self._set_output_enabled(False):
            self._publish(goal_id, PlannerStatus.FAILED,
                          "planning_output_gate_unavailable")
            return

        self._goal = points[-1]
        self._goal_id = goal_id
        self._route_points = points
        self._route_message = copy.deepcopy(message)
        self._route_recovery_attempts = 0
        self._route_progress = 0.0
        self._route_length = path_length(points)
        self._last_route_progress = 0.0
        self._last_route_progress_time = rospy.Time.now()
        self._goal_received = rospy.Time.now()
        self._arrival_since = None
        self._terminal = False
        self._owner_selected_for_goal = not bool(self._owner_selection_service)
        self._last_state = PlannerStatus.PLANNING
        self._publish(goal_id, PlannerStatus.PLANNING,
                      "EGO reference path accepted by planning layer")
        self._ego_path_publisher.publish(message)

    def _candidate_callback(self, message):
        if (self._goal is None or self._terminal or
                self._owner_selected_for_goal or
                not self._owner_selection_service):
            return
        if not self._takeoff_ready():
            rospy.logwarn_throttle(
                2.0,
                "EGO candidate is waiting for controller-owned takeoff "
                "to reach a stable airborne state")
            return
        if (self._goal_received is not None and
                message.header.stamp < self._goal_received):
            return
        try:
            rospy.wait_for_service(self._owner_selection_service, timeout=0.05)
            response = rospy.ServiceProxy(
                self._owner_selection_service, SetBool)(True)
            if response.success:
                self._owner_selected_for_goal = True
                if not self._set_output_enabled(True):
                    self._fail("planning_output_gate_unavailable")
                else:
                    rospy.loginfo("EGO reference owner acquired: %s",
                                  response.message)
        except (rospy.ROSException, rospy.ServiceException):
            pass

    def _set_output_enabled(self, enabled):
        if not self._output_gate_service:
            return True
        try:
            rospy.wait_for_service(self._output_gate_service, timeout=0.30)
            response = rospy.ServiceProxy(
                self._output_gate_service, SetBool)(bool(enabled))
            return bool(response.success)
        except (rospy.ROSException, rospy.ServiceException):
            return False

    def _cancel_callback(self, request):
        if self._goal_id is None:
            return CancelPlanningResponse(True, "planning already idle")
        if int(request.goal_id) not in (0, int(self._goal_id)):
            return CancelPlanningResponse(
                False, "active goal id is {}".format(self._goal_id))
        if not self._set_output_enabled(False):
            return CancelPlanningResponse(False,
                                          "failed to disable planning output")
        goal_id = self._goal_id
        self._goal = None
        self._goal_id = None
        self._route_points = None
        self._route_message = None
        self._route_recovery_attempts = 0
        self._route_progress = 0.0
        self._route_length = 0.0
        self._last_route_progress = 0.0
        self._last_route_progress_time = None
        self._goal_received = None
        self._arrival_since = None
        self._terminal = True
        self._last_state = PlannerStatus.IDLE
        self._ego_path_publisher.publish(Path())
        self._publish(goal_id, PlannerStatus.IDLE,
                      "planning cancelled: " + str(request.reason))
        return CancelPlanningResponse(True, "planning output released")

    def _resolve_goal_id(self, message):
        """Recover the allocator ID without trusting rospy's transport seq."""
        if self._task_state_service and self._uav_name:
            try:
                rospy.wait_for_service(self._task_state_service, timeout=0.10)
                response = rospy.ServiceProxy(
                    self._task_state_service, GetMissionState)()
                prefix = self._uav_name + ":goal="
                for entry in response.active_goals:
                    if entry.startswith(prefix):
                        value = entry[len(prefix):].split(",", 1)[0]
                        return int(value)
            except (rospy.ROSException, rospy.ServiceException, ValueError):
                pass
        # Standalone EGO demos do not run the allocator.  The fallback is
        # adequate for local status display, but allocator deployments use the
        # service path above because rospy rewrites top-level Header.seq.
        return int(message.header.seq)

    def _resolve_path_goal_id(self, message):
        """Use allocator state first, then the final pose/path sequence."""
        if self._task_state_service and self._uav_name:
            try:
                rospy.wait_for_service(self._task_state_service, timeout=0.10)
                response = rospy.ServiceProxy(
                    self._task_state_service, GetMissionState)()
                prefix = self._uav_name + ":goal="
                for entry in response.active_goals:
                    if entry.startswith(prefix):
                        value = entry[len(prefix):].split(",", 1)[0]
                        return int(value)
            except (rospy.ROSException, rospy.ServiceException, ValueError):
                pass
        if message.poses and message.poses[-1].header.seq:
            return int(message.poses[-1].header.seq)
        return int(message.header.seq)

    def _health_callback(self, message):
        self._healthy = bool(message.data)
        self._health_stamp = rospy.Time.now()

    def _control_command_callback(self, message):
        self._takeoff_active = bool(message.takeoff_active)
        self._landing_active = bool(message.landing_active)
        self._last_control_command_stamp = rospy.Time.now()

    def _odometry_callback(self, message):
        self._last_odom_stamp = message.header.stamp
        position = message.pose.pose.position
        self._last_position = (float(position.x), float(position.y),
                               float(position.z))
        self._last_vertical_speed = float(message.twist.twist.linear.z)
        if self._goal is None or self._terminal:
            return
        if message.header.frame_id.strip("/") != self._common_frame:
            self._fail("odometry_frame_mismatch")
            return
        now = rospy.Time.now()
        if not self._health_current(now):
            self._arrival_since = None
            return
        if not self._owner_selected_for_goal:
            return
        velocity = message.twist.twist.linear
        current_position = self._last_position
        current_velocity = (float(velocity.x), float(velocity.y),
                            float(velocity.z))
        if self._route_points is not None:
            previous_progress = self._route_progress
            self._route_progress = project_path_progress(
                self._route_points, current_position, self._route_progress,
                self._path_progress_backtrack, self._path_progress_search)
            if (self._route_progress >= previous_progress +
                    max(0.0, self._route_progress_epsilon)):
                self._last_route_progress = self._route_progress
                self._last_route_progress_time = now
                self._route_recovery_attempts = 0
            route_finished = (
                self._route_progress >= self._route_length -
                max(self._position_tolerance, 0.05))
            reached = route_finished and arrival_reached(
                current_position, current_velocity, self._goal,
                self._position_tolerance, self._speed_tolerance)
        else:
            reached = arrival_reached(
                current_position, current_velocity, self._goal,
                self._position_tolerance, self._speed_tolerance)
        if reached:
            if self._arrival_since is None:
                self._arrival_since = now
            if (now - self._arrival_since).to_sec() >= self._arrival_dwell:
                self._terminal = True
                self._last_state = PlannerStatus.REACHED
                detail = ("EGO reference path reached and settled"
                          if self._route_points is not None else
                          "EGO goal reached and settled")
                self._publish(self._goal_id, PlannerStatus.REACHED, detail)
        else:
            self._arrival_since = None
            if self._last_state != PlannerStatus.ACTIVE:
                self._last_state = PlannerStatus.ACTIVE
                self._publish(self._goal_id, PlannerStatus.ACTIVE,
                              "EGO trajectory active")

    def _health_current(self, now):
        return (self._healthy and self._health_stamp is not None and
                (now - self._health_stamp).to_sec() <= self._odom_timeout)

    def _takeoff_ready(self):
        """Return true only when controller-owned takeoff is out of the way."""
        if not self._gate_owner_until_takeoff:
            return True
        if self._last_position is None:
            return False
        x, y, z = self._last_position
        if (not self._finite((x, y, z)) or
                not math.isfinite(self._startup_ground_height)):
            return False
        if z <= self._startup_ground_height + max(
                0.10, self._startup_min_altitude):
            return False
        if (self._last_vertical_speed is not None and
                math.isfinite(self._last_vertical_speed) and
                abs(self._last_vertical_speed) > max(
                    0.05, self._startup_max_vertical_speed)):
            return False
        if self._control_command_topic:
            if (self._last_control_command_stamp is None or
                    (rospy.Time.now() -
                     self._last_control_command_stamp).to_sec() >
                    self._odom_timeout):
                return False
            if self._takeoff_active or self._landing_active:
                return False
        return True

    def _timer_callback(self, _event):
        if self._goal is None or self._terminal:
            return
        now = rospy.Time.now()
        if self._route_points is not None:
            if (self._route_stall_timeout > 0.0 and
                    self._last_route_progress_time is not None and
                    (now - self._last_route_progress_time).to_sec() >
                    self._route_stall_timeout):
                if self._recover_stalled_route(now):
                    return
                self._fail(
                    "EGO route recovery exhausted at {:.2f}/{:.2f} m".format(
                        self._route_progress, self._route_length))
                return
        elif (self._goal_received is not None and self._goal_timeout > 0.0 and
              (now - self._goal_received).to_sec() > self._goal_timeout):
            self._fail("EGO goal timeout")
            return
        if (not self._owner_selected_for_goal and
                self._owner_acquisition_timeout > 0.0 and
                self._goal_received is not None and
                self._takeoff_ready() and
                (now - self._goal_received).to_sec() >
                self._owner_acquisition_timeout):
            self._fail("EGO reference owner not acquired")
            return
        if self._last_odom_stamp is not None:
            age = (now - self._last_odom_stamp).to_sec()
            if age > self._odom_timeout:
                self._fail("EGO odometry stale")

    def _recover_stalled_route(self, now):
        """Reattach a stalled arbitrary Path to EGO from current odometry."""
        if (not self._route_stall_recovery_enabled or
                self._route_message is None or self._goal_id is None):
            return False
        if (self._route_stall_max_recoveries > 0 and
                self._route_recovery_attempts >=
                self._route_stall_max_recoveries):
            return False
        route = self._remaining_route_message(now)
        if route is None:
            return False
        # Stop forwarding the stale B-spline while EGO reconnects the same
        # route from measured state. A new reference candidate re-enables the
        # output through the normal ownership handshake.
        if not self._set_output_enabled(False):
            return False
        self._route_recovery_attempts += 1
        self._last_route_progress_time = now
        self._goal_received = now
        self._arrival_since = None
        self._terminal = False
        self._owner_selected_for_goal = not bool(self._owner_selection_service)
        self._last_state = PlannerStatus.PLANNING
        route.header.stamp = now
        for pose in route.poses:
            pose.header.stamp = now
        detail = (
            "EGO route stalled at {:.2f}/{:.2f} m; recovering from measured "
            "state (attempt {})".format(
                self._route_progress, self._route_length,
                self._route_recovery_attempts))
        rospy.logwarn(detail)
        self._publish(self._goal_id, PlannerStatus.PLANNING, detail)
        self._ego_path_publisher.publish(route)
        return True

    def _remaining_route_message(self, stamp):
        """Trim a recovery Path behind current monotonic route progress."""
        if (self._route_message is None or self._route_points is None or
                len(self._route_points) < 2):
            return None

        route = copy.deepcopy(self._route_message)
        progress = max(0.0, min(self._route_length, self._route_progress))
        distance = 0.0
        segment_index = len(self._route_points) - 2
        alpha = 1.0
        for index in range(len(self._route_points) - 1):
            start = self._route_points[index]
            end = self._route_points[index + 1]
            length = math.sqrt(sum(
                (end[axis] - start[axis]) ** 2 for axis in range(3)))
            if distance + length >= progress - 1.0e-9:
                segment_index = index
                alpha = (0.0 if length <= 1.0e-9 else
                         max(0.0, min(1.0,
                             (progress - distance) / length)))
                break
            distance += length

        anchor = copy.deepcopy(route.poses[segment_index + 1])
        start = self._route_points[segment_index]
        end = self._route_points[segment_index + 1]
        anchor.pose.position.x = start[0] + alpha * (end[0] - start[0])
        anchor.pose.position.y = start[1] + alpha * (end[1] - start[1])
        anchor.pose.position.z = start[2] + alpha * (end[2] - start[2])
        remaining = [anchor]
        remaining.extend(copy.deepcopy(
            route.poses[segment_index + 1:]))

        # Include measured position as the handoff point. EGO removes this
        # near-duplicate itself, but retaining it ensures the message still
        # contains two valid poses when only the final goal remains.
        if self._last_position is not None:
            measured = copy.deepcopy(anchor)
            measured.pose.position.x = self._last_position[0]
            measured.pose.position.y = self._last_position[1]
            measured.pose.position.z = self._last_position[2]
            anchor_position = (anchor.pose.position.x,
                               anchor.pose.position.y,
                               anchor.pose.position.z)
            separation = math.sqrt(sum(
                (self._last_position[axis] - anchor_position[axis]) ** 2
                for axis in range(3)))
            if separation >= 0.05:
                remaining.insert(0, measured)

        # Remove adjacent duplicates introduced when progress lies exactly
        # on an original waypoint.
        deduplicated = []
        for pose in remaining:
            point = pose.pose.position
            if deduplicated:
                previous = deduplicated[-1].pose.position
                if math.sqrt((point.x - previous.x) ** 2 +
                             (point.y - previous.y) ** 2 +
                             (point.z - previous.z) ** 2) < 0.05:
                    continue
            pose.header.stamp = stamp
            deduplicated.append(pose)
        if len(deduplicated) < 2:
            return None
        route.poses = deduplicated
        return route

    def _fail(self, reason):
        if self._terminal or self._goal_id is None:
            return
        # Do not leave a partially valid EGO trajectory connected to the
        # controller after a watchdog/health failure.
        self._set_output_enabled(False)
        self._terminal = True
        self._last_state = PlannerStatus.FAILED
        self._publish(self._goal_id, PlannerStatus.FAILED, reason)

    def _publish(self, goal_id, state, detail):
        message = PlannerStatus()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self._common_frame
        message.goal_id = int(goal_id)
        message.state = int(state)
        message.detail = str(detail)
        self._publisher.publish(message)


def main():
    rospy.init_node("ego_planner_status")
    EgoStatus()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
