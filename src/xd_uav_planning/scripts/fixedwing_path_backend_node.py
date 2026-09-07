#!/usr/bin/env python3
"""Fail-closed task-path adapter for the fixed-wing controller backend."""

import copy
import sys

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from xd_uav_controller.msg import ControlState, PathStatus
from xd_uav_task_allocate.msg import PlannerStatus

from xd_uav_planning.core import (
    PathSample,
    VehicleStateSample,
    validate_path,
    validate_vehicle_state,
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
        self._state = None
        self._active_goal_id = None
        self._controller_path_id = None
        self._awaiting_controller_acceptance = False
        self._active_terminal = False
        self._state_reason = "state_not_received"
        self._path_reason = "path_not_received"

        self._path_publisher = rospy.Publisher(
            rospy.get_param("~controller_path_topic", "control/reference/path"),
            Path, queue_size=1)
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
            rospy.get_param("~mission_path_topic", "planning/mission_path"),
            Path, self._path_callback, queue_size=2)
        self._path_status_subscriber = rospy.Subscriber(
            rospy.get_param("~controller_path_status_topic",
                            "controller/path_status"),
            PathStatus, self._path_status_callback, queue_size=20)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._timer_callback)
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
        self._state = self._vehicle_state(message)
        result = self._state_validation()
        self._state_reason = result.reason if result is not None else "state_not_received"

    def _path_callback(self, message):
        goal_id = self._goal_id(message)
        state = self._state_validation()
        if state is None or not state.valid:
            reason = "vehicle_not_ready:" + (
                state.reason if state is not None else "state_not_received")
            self._path_reason = reason
            self._publish_status(goal_id, PlannerStatus.FAILED, reason)
            return
        validation = validate_path(
            self._path_sample(message), rospy.Time.now().to_sec(),
            self._common_frame, self._path_timeout, self._future_tolerance,
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
        forwarded = copy.deepcopy(message)
        forwarded.header.frame_id = self._common_frame
        for pose in forwarded.poses:
            if not pose.header.frame_id:
                pose.header.frame_id = self._common_frame
        self._active_goal_id = goal_id
        self._controller_path_id = None
        self._awaiting_controller_acceptance = True
        self._active_terminal = False
        self._publish_status(
            goal_id, PlannerStatus.PLANNING,
            "fixed-wing path validated; awaiting controller acceptance")
        self._path_publisher.publish(forwarded)

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
