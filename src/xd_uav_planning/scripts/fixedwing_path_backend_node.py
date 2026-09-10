#!/usr/bin/env python3
"""Fail-closed task-path adapter for the fixed-wing controller backend."""

import copy
import math
import sys
import threading

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from xd_uav_controller.msg import ControlState, PathStatus
from xd_uav_planning.msg import NoFlyZone
from xd_uav_task_allocate.msg import PlannerStatus
from xd_uav_task_allocate.srv import CancelPlanning, CancelPlanningResponse

from xd_uav_planning.core import (
    PathSample,
    VehicleStateSample,
    validate_path,
    validate_vehicle_state,
)
from xd_uav_planning.nofly import (
    NoFlyConfig,
    NoFlyZoneStore,
    NoSafePathError,
    adjust_fixedwing_path,
    path_is_clear,
    remaining_path,
)


class FixedwingPathBackend:
    def __init__(self):
        self._common_frame = rospy.get_param("~common_frame", "world").strip("/")
        self._state_timeout = float(rospy.get_param("~state_timeout", 0.30))
        self._path_timeout = float(rospy.get_param("~path_timeout", 1.0))
        self._future_tolerance = float(
            rospy.get_param("~future_tolerance", 0.02))
        self._minimum_segment_length = float(
            rospy.get_param("~minimum_segment_length", 0.05))
        self._maximum_points = int(rospy.get_param("~maximum_points", 10000))
        self._turning_radius = float(rospy.get_param("~turning_radius", 35.0))
        self._zone_clearance = float(rospy.get_param("~zone_clearance", 15.0))
        self._planning_sample_step = float(
            rospy.get_param("~planning_sample_step", 2.0))
        self._cancel_cruise_airspeed = float(
            rospy.get_param("~cancel_cruise_airspeed", 15.0))
        self._state = None
        self._state_message = None
        self._state_frame = ""
        self._state_position = None
        self._state_course = None
        self._active_goal_id = None
        self._controller_path_id = None
        self._awaiting_controller_acceptance = False
        self._active_terminal = False
        self._original_task_points = None
        self._original_progress = 0.0
        self._planning_lock = threading.RLock()
        self._state_reason = "state_not_received"
        self._path_reason = "path_not_received"
        self._zone_reason = "no_zone_received"
        self._zone_store = NoFlyZoneStore(NoFlyConfig(
            expected_frame=self._common_frame,
            max_message_age=float(rospy.get_param(
                "~zone_max_message_age", 1.0)),
            future_tolerance=self._future_tolerance,
            max_ttl=float(rospy.get_param("~zone_max_ttl", 3600.0)),
            min_altitude_limit=float(rospy.get_param(
                "~zone_min_altitude_limit", -1000.0)),
            max_altitude_limit=float(rospy.get_param(
                "~zone_max_altitude_limit", 10000.0)),
            max_abs_coordinate=float(rospy.get_param(
                "~zone_max_abs_coordinate", 100000.0)),
            min_polygon_area=float(rospy.get_param(
                "~zone_min_polygon_area", 1.0)),
            max_vertices=int(rospy.get_param("~zone_max_vertices", 64)),
        ))
        self._cancel_offboard_service = rospy.get_param(
            "~dynamic_failure_cancel_service",
            "control_manager/cancel_offboard")
        self._cancel_offboard = (rospy.ServiceProxy(
            self._cancel_offboard_service, Trigger)
            if self._cancel_offboard_service else None)

        self._path_publisher = rospy.Publisher(
            rospy.get_param("~controller_path_topic", "control/reference/path"),
            Path, queue_size=1)
        self._setpoint_publisher = rospy.Publisher(
            rospy.get_param("~controller_setpoint_topic",
                            "control/reference/setpoint"),
            PositionTarget, queue_size=1)
        self._status_publisher = rospy.Publisher(
            rospy.get_param("~planner_status_topic", "planning/status"),
            PlannerStatus, queue_size=10, latch=True)
        self._healthy_publisher = rospy.Publisher(
            rospy.get_param("~healthy_topic", "planning/healthy"),
            Bool, queue_size=1, latch=True)
        self._diagnostics_publisher = rospy.Publisher(
            rospy.get_param("~diagnostics_topic", "planning/diagnostics"),
            DiagnosticArray, queue_size=1)
        self._state_subscriber = rospy.Subscriber(
            rospy.get_param("~state_topic", "control_manager/state"),
            ControlState, self._state_callback, queue_size=20)
        self._path_subscriber = rospy.Subscriber(
            rospy.get_param("~task_path_topic", "planning/task_path"),
            Path, self._path_callback, queue_size=2)
        self._zone_subscriber = rospy.Subscriber(
            rospy.get_param("~no_fly_zone_topic", "planning/no_fly_zone"),
            NoFlyZone, self._zone_callback, queue_size=10)
        self._path_status_subscriber = rospy.Subscriber(
            rospy.get_param("~controller_path_status_topic",
                            "controller/path_status"),
            PathStatus, self._path_status_callback, queue_size=20)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._timer_callback)
        self._cancel_service = rospy.Service(
            rospy.get_param("~cancel_service", "planning/cancel"),
            CancelPlanning, self._cancel_callback)
        self._publish_health(False)

    @staticmethod
    def _vehicle_state(message):
        return VehicleStateSample(
            stamp=message.header.stamp.to_sec(),
            vehicle_type=int(message.vehicle_type),
            state_valid=bool(message.state_valid),
            localization_valid=bool(message.localization_valid),
            odometry_fresh=bool(message.odometry_fresh),
        )

    @staticmethod
    def _path_sample(message):
        points = tuple((float(pose.pose.position.x),
                        float(pose.pose.position.y),
                        float(pose.pose.position.z))
                       for pose in message.poses)
        frames = tuple(pose.header.frame_id.strip("/")
                       for pose in message.poses)
        return PathSample(
            stamp=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id.strip("/"),
            points=points,
            pose_frames=frames,
        )

    @staticmethod
    def _goal_id(message):
        # rospy overwrites the top-level Header.seq while publishing.  The
        # allocator deliberately mirrors its stable goal ID into every nested
        # PoseStamped header, which rospy leaves untouched.
        if message.poses and int(message.poses[0].header.seq) != 0:
            return int(message.poses[0].header.seq)
        return int(message.header.seq)

    def _state_validation(self):
        if self._state is None:
            return None
        return validate_vehicle_state(
            self._state, rospy.Time.now().to_sec(),
            ControlState.VEHICLE_FIXEDWING,
            self._state_timeout, self._future_tolerance)

    def _state_callback(self, message):
        self._state_message = copy.deepcopy(message)
        self._state = self._vehicle_state(message)
        self._state_frame = message.header.frame_id.strip("/")
        self._state_position = (
            float(message.position_odom.x), float(message.position_odom.y),
            float(message.position_odom.z))
        self._state_course = float(message.course)
        result = self._state_validation()
        self._state_reason = result.reason if result is not None else "state_not_received"

    def _zone_callback(self, message):
        with self._planning_lock:
            accepted = self._zone_store.accept(
                message, rospy.Time.now().to_sec())
            self._zone_reason = self._zone_store.last_reason
            if accepted:
                rospy.loginfo(
                    "fixed-wing no-fly update accepted: operation=%d zone=%d revision=%d",
                    message.operation, message.zone_id,
                    self._zone_store.revision)
                if (self._active_goal_id is not None and
                        self._original_task_points is not None and
                        not self._active_terminal):
                    self._replace_active_remaining_path(message.operation)
            else:
                rospy.logerr(
                    "fixed-wing no-fly update rejected: operation=%d zone=%d reason=%s",
                    message.operation, message.zone_id, self._zone_reason)

    def _adjust_points(self, input_points):
        zones = self._zone_store.snapshot()
        if not zones or path_is_clear(
                input_points, zones, self._zone_clearance,
                self._planning_sample_step):
            return list(input_points), False
        if self._state_frame != self._common_frame:
            raise NoSafePathError("state_frame_mismatch")
        output_points, adjusted = adjust_fixedwing_path(
            input_points, self._state_position, self._state_course,
            zones, self._turning_radius, self._zone_clearance,
            self._planning_sample_step)
        if len(output_points) > self._maximum_points:
            raise NoSafePathError("adjusted_path_too_long")
        return output_points, adjusted

    def _path_message(self, points, goal_id):
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self._common_frame
        message.header.seq = int(goal_id)
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = self._common_frame
            pose.header.seq = int(goal_id)
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            if index + 1 < len(points):
                neighbour = points[index + 1]
                yaw = math.atan2(neighbour[1] - point[1],
                                 neighbour[0] - point[0])
            else:
                neighbour = points[index - 1]
                yaw = math.atan2(point[1] - neighbour[1],
                                 point[0] - neighbour[0])
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        return message

    def _dispatch(self, points, detail):
        self._controller_path_id = None
        self._awaiting_controller_acceptance = True
        self._active_terminal = False
        self._path_reason = detail
        self._publish_status(
            self._active_goal_id, PlannerStatus.PLANNING, detail)
        self._path_publisher.publish(
            self._path_message(points, self._active_goal_id))

    def _fail_dynamic_replan(self, reason):
        detail = "fixedwing_dynamic_nofly_blocked:" + str(reason)
        self._path_reason = detail
        self._active_terminal = True
        self._controller_path_id = None
        self._awaiting_controller_acceptance = False
        self._publish_status(
            self._active_goal_id, PlannerStatus.BLOCKED, detail)
        if self._cancel_offboard is None:
            rospy.logfatal("%s; no cancel service configured", detail)
            return
        try:
            response = self._cancel_offboard()
            if response.success:
                rospy.logerr("%s; controller path execution cancelled: %s",
                             detail, response.message)
            else:
                rospy.logfatal("%s; cancel service rejected request: %s",
                               detail, response.message)
        except rospy.ServiceException as error:
            rospy.logfatal("%s; cancel service failed: %s", detail, error)

    def _cancel_callback(self, request):
        with self._planning_lock:
            if self._active_goal_id is None:
                return CancelPlanningResponse(True, "planning already idle")
            if int(request.goal_id) not in (0, int(self._active_goal_id)):
                return CancelPlanningResponse(
                    False, "active goal id is {}".format(self._active_goal_id))
            state = self._state_validation()
            message = self._state_message
            if state is None or not state.valid or message is None:
                return CancelPlanningResponse(
                    False, "cannot release path without valid vehicle state")

            # Replace the controller's geometric path with the previous
            # fixed-wing release reference, preserving altitude and direction.
            release = PositionTarget()
            release.header.stamp = rospy.Time.now()
            release.header.frame_id = message.header.frame_id
            release.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            release.type_mask = (
                PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
            release.position.z = message.position_odom.z
            horizontal_speed = math.hypot(message.velocity_odom.x,
                                          message.velocity_odom.y)
            if horizontal_speed > 0.5:
                release.velocity.x = message.velocity_odom.x
                release.velocity.y = message.velocity_odom.y
            else:
                measured_airspeed = float(message.airspeed)
                speed = (measured_airspeed
                         if math.isfinite(measured_airspeed) and
                         measured_airspeed > 1.0
                         else self._cancel_cruise_airspeed)
                release.velocity.x = speed * math.cos(float(message.course))
                release.velocity.y = speed * math.sin(float(message.course))
            self._setpoint_publisher.publish(release)

            goal_id = self._active_goal_id
            self._active_goal_id = None
            self._controller_path_id = None
            self._awaiting_controller_acceptance = False
            self._active_terminal = True
            self._original_task_points = None
            self._original_progress = 0.0
            self._path_reason = "cancelled:" + str(request.reason)
            self._publish_status(goal_id, PlannerStatus.IDLE, self._path_reason)
            return CancelPlanningResponse(True, "fixed-wing path released")

    def _replace_active_remaining_path(self, operation):
        state = self._state_validation()
        if state is None or not state.valid or self._state_position is None:
            self._fail_dynamic_replan(
                "vehicle_not_ready:" + (
                    state.reason if state is not None else "state_not_received"))
            return
        try:
            remaining, progress = remaining_path(
                self._original_task_points, self._state_position,
                self._original_progress)
            output_points, adjusted = self._adjust_points(remaining)
        except (NoSafePathError, ValueError) as error:
            self._fail_dynamic_replan(error)
            return
        self._original_progress = progress
        action = {
            NoFlyZone.OP_UPSERT: "upsert",
            NoFlyZone.OP_REMOVE: "remove",
            NoFlyZone.OP_CLEAR: "clear",
        }.get(int(operation), "update")
        detail = (
            "dynamic no-fly {} revision {}: {} remaining path; awaiting "
            "controller acceptance").format(
                action, self._zone_store.revision,
                "adjusted" if adjusted else "restored")
        self._dispatch(output_points, detail)

    def _path_callback(self, message):
        with self._planning_lock:
            goal_id = self._goal_id(message)
            state = self._state_validation()
            if state is None or not state.valid:
                reason = "vehicle_not_ready:" + (
                    state.reason if state is not None else "state_not_received")
                self._path_reason = reason
                self._publish_status(goal_id, PlannerStatus.FAILED, reason)
                return
            sample = self._path_sample(message)
            validation = validate_path(
                sample, rospy.Time.now().to_sec(), self._common_frame,
                self._path_timeout, self._future_tolerance,
                self._minimum_segment_length, self._maximum_points)
            self._path_reason = validation.reason
            if not validation.valid:
                self._publish_status(
                    goal_id, PlannerStatus.FAILED,
                    "fixedwing_path_rejected:" + validation.reason)
                return
            if (self._active_goal_id == goal_id and
                    not self._active_terminal):
                return
            try:
                output_points, adjusted = self._adjust_points(sample.points)
            except NoSafePathError as error:
                reason = "fixedwing_nofly_blocked:" + str(error)
                self._path_reason = reason
                self._publish_status(goal_id, PlannerStatus.BLOCKED, reason)
                return
            self._active_goal_id = goal_id
            self._original_task_points = sample.points
            self._original_progress = 0.0
            self._dispatch(
                output_points,
                ("fixed-wing no-fly path adjusted; awaiting controller acceptance"
                 if adjusted else
                 "fixed-wing path validated; awaiting controller acceptance"))

    def _path_status_callback(self, message):
        controller_path_id = int(message.path_id)
        if self._active_goal_id is None:
            return
        if (int(message.state) == PathStatus.ACCEPTED and
                self._awaiting_controller_acceptance):
            self._controller_path_id = controller_path_id
            self._awaiting_controller_acceptance = False
        elif (self._controller_path_id is None or
              controller_path_id != self._controller_path_id):
            return
        mapping = {
            PathStatus.ACCEPTED: PlannerStatus.PLANNING,
            PathStatus.ACTIVE: PlannerStatus.ACTIVE,
            PathStatus.REACQUIRING: PlannerStatus.ACTIVE,
            PathStatus.COMPLETED: PlannerStatus.REACHED,
            PathStatus.REJECTED: PlannerStatus.FAILED,
            PathStatus.FAILED: PlannerStatus.FAILED,
        }
        planner_state = mapping.get(int(message.state), PlannerStatus.FAILED)
        self._active_terminal = planner_state in (
            PlannerStatus.REACHED, PlannerStatus.FAILED)
        detail = "controller path {:.1f}%: {}".format(
            float(message.progress) * 100.0, message.detail)
        self._path_reason = detail
        self._publish_status(self._active_goal_id, planner_state, detail)

    def _publish_status(self, path_id, state, detail):
        message = PlannerStatus()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self._common_frame
        message.goal_id = int(path_id)
        message.state = int(state)
        message.detail = str(detail)
        self._status_publisher.publish(message)

    def _timer_callback(self, _event):
        validation = self._state_validation()
        healthy = validation is not None and validation.valid
        self._state_reason = (
            validation.reason if validation is not None else "state_not_received")
        if (not healthy and self._active_goal_id is not None and
                not self._active_terminal):
            self._active_terminal = True
            self._publish_status(
                self._active_goal_id, PlannerStatus.FAILED,
                "fixed-wing state became invalid:" + self._state_reason)
        self._publish_health(healthy)

    def _publish_health(self, healthy):
        self._healthy_publisher.publish(Bool(data=healthy))
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/fixedwing_path_backend"
        status.hardware_id = "xd_uav_planning"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = "healthy" if healthy else "fail_closed"
        status.values = [
            KeyValue(key="vehicle_type", value="fixedwing"),
            KeyValue(key="state", value=self._state_reason),
            KeyValue(key="path", value=self._path_reason),
            KeyValue(key="no_fly_zone", value=self._zone_reason),
            KeyValue(key="no_fly_zone_count",
                     value=str(len(self._zone_store.snapshot()))),
            KeyValue(key="common_frame", value=self._common_frame),
        ]
        array.status = [status]
        self._diagnostics_publisher.publish(array)


def main():
    rospy.init_node("fixedwing_path_backend")
    FixedwingPathBackend()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
