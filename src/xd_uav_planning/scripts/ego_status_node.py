#!/usr/bin/env python3
"""Translate an EGO goal/health/odometry stream into PlannerStatus."""

import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from xd_uav_task_allocate.msg import PlannerStatus
from xd_uav_task_allocate.srv import GetMissionState

from xd_uav_planning.core import arrival_reached


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
        self._flight_type = int(rospy.get_param("~flight_type", 1))
        self._output_gate_service = rospy.get_param(
            "~output_gate_service", "").strip()
        self._owner_selection_service = rospy.get_param(
            "~owner_selection_service", "").strip()
        self._owner_acquisition_timeout = float(rospy.get_param(
            "~owner_acquisition_timeout", 5.0))
        self._odom_timeout = float(rospy.get_param("~odom_timeout", 0.30))
        self._goal = None
        self._goal_id = None
        self._goal_received = None
        self._healthy = False
        self._health_stamp = None
        self._last_odom_stamp = None
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
        self._goal_subscriber = rospy.Subscriber(
            rospy.get_param("~goal_topic", "planning/goal"),
            PoseStamped, self._goal_callback, queue_size=2)
        self._health_subscriber = rospy.Subscriber(
            rospy.get_param("~health_topic", "planning/healthy"),
            Bool, self._health_callback, queue_size=1)
        self._odom_subscriber = rospy.Subscriber(
            rospy.get_param("~odometry_topic", "ego/odometry"),
            Odometry, self._odometry_callback, queue_size=20)
        self._candidate_subscriber = rospy.Subscriber(
            rospy.get_param("~candidate_topic", "ego/reference_candidate"),
            PositionTarget, self._candidate_callback, queue_size=20)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._timer_callback)

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
        self._goal_received = rospy.Time.now()
        self._arrival_since = None
        self._terminal = False
        self._owner_selected_for_goal = not bool(self._owner_selection_service)
        self._last_state = PlannerStatus.PLANNING
        self._publish(goal_id, PlannerStatus.PLANNING, "EGO goal accepted by planning layer")
        # EGO consumes only the planning layer's validated internal topic.
        self._ego_goal_publisher.publish(message)

    def _candidate_callback(self, message):
        if (self._goal is None or self._terminal or
                self._owner_selected_for_goal or
                not self._owner_selection_service):
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

    def _health_callback(self, message):
        self._healthy = bool(message.data)
        self._health_stamp = rospy.Time.now()

    def _odometry_callback(self, message):
        self._last_odom_stamp = message.header.stamp
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
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        reached = arrival_reached(
            (float(position.x), float(position.y), float(position.z)),
            (float(velocity.x), float(velocity.y), float(velocity.z)),
            self._goal, self._position_tolerance, self._speed_tolerance)
        if reached:
            if self._arrival_since is None:
                self._arrival_since = now
            if (now - self._arrival_since).to_sec() >= self._arrival_dwell:
                self._terminal = True
                self._last_state = PlannerStatus.REACHED
                self._publish(self._goal_id, PlannerStatus.REACHED,
                              "EGO goal reached and settled")
        else:
            self._arrival_since = None
            if self._last_state != PlannerStatus.ACTIVE:
                self._last_state = PlannerStatus.ACTIVE
                self._publish(self._goal_id, PlannerStatus.ACTIVE,
                              "EGO trajectory active")

    def _health_current(self, now):
        return (self._healthy and self._health_stamp is not None and
                (now - self._health_stamp).to_sec() <= self._odom_timeout)

    def _timer_callback(self, _event):
        if self._goal is None or self._terminal:
            return
        now = rospy.Time.now()
        if (self._goal_received is not None and self._goal_timeout > 0.0 and
                (now - self._goal_received).to_sec() > self._goal_timeout):
            self._fail("EGO goal timeout")
            return
        if (not self._owner_selected_for_goal and
                self._owner_acquisition_timeout > 0.0 and
                self._goal_received is not None and
                (now - self._goal_received).to_sec() >
                self._owner_acquisition_timeout):
            self._fail("EGO reference owner not acquired")
            return
        if self._last_odom_stamp is not None:
            age = (now - self._last_odom_stamp).to_sec()
            if age > self._odom_timeout:
                self._fail("EGO odometry stale")

    def _fail(self, reason):
        if self._terminal or self._goal_id is None:
            return
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
