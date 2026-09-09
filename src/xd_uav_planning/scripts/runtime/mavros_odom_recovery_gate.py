#!/usr/bin/env python3
"""Pluggable MAVROS odometry guard for estimator quarantine recovery."""

import math
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class MavrosOdomRecoveryGate:
    def __init__(self):
        self._jump_threshold = float(rospy.get_param("~jump_threshold", 3.5))
        self._candidate_radius = float(rospy.get_param("~candidate_radius", 0.5))
        self._stable_time = float(rospy.get_param("~stable_time", 0.30))
        self._minimum_dropout = float(rospy.get_param("~minimum_dropout", 0.40))
        self._state_timeout = float(rospy.get_param("~state_timeout", 1.0))
        self._mode, self._reason = "pass", "ok"
        self._last_forwarded_position = None
        self._candidate_position = None
        self._candidate_started = None
        self._dropout_started = None
        self._latest_state = None
        self._latest_state_received = None
        self._output = rospy.Publisher(
            rospy.get_param("~output_topic", "integration/mavros_odom_guarded"),
            Odometry, queue_size=30)
        self._healthy = rospy.Publisher(
            rospy.get_param("~healthy_topic", "integration/mavros_odom_gate/healthy"),
            Bool, queue_size=1, latch=True)
        self._diagnostics = rospy.Publisher(
            rospy.get_param("~diagnostics_topic", "integration/mavros_odom_gate/diagnostics"),
            DiagnosticArray, queue_size=1)
        self._state_subscriber = rospy.Subscriber(
            rospy.get_param("~state_topic", "mavros/state"), State,
            self._state_callback, queue_size=5)
        self._odom_subscriber = rospy.Subscriber(
            rospy.get_param("~input_topic", "mavros/local_position/odom"), Odometry,
            self._odom_callback, queue_size=30)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._timer_callback)
        self._publish_status()

    @staticmethod
    def _position(message):
        p = message.pose.pose.position
        return float(p.x), float(p.y), float(p.z)

    @staticmethod
    def _distance(first, second):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))

    def _state_callback(self, message):
        self._latest_state = message
        self._latest_state_received = rospy.Time.now()

    def _confirmed_disarmed(self, now):
        return (self._latest_state is not None and
                self._latest_state_received is not None and
                (now - self._latest_state_received).to_sec() <= self._state_timeout and
                self._latest_state.connected and not self._latest_state.armed)

    def _forward(self, message, position):
        self._output.publish(message)
        self._last_forwarded_position = position
        self._mode, self._reason = "pass", "ok"
        self._candidate_position = None
        self._candidate_started = None
        self._dropout_started = None

    def _begin_candidate(self, position, now):
        self._mode, self._reason = "candidate", "position_jump_candidate"
        self._candidate_position = position
        self._candidate_started = now
        self._dropout_started = now
        rospy.logwarn("MAVROS odom jump detected; holding input for stability check")

    def _odom_callback(self, message):
        now = rospy.Time.now()
        position = self._position(message)
        if not all(math.isfinite(value) for value in position):
            self._mode, self._reason = "blocked", "non_finite_odometry"
            self._publish_status()
            return
        if self._last_forwarded_position is None:
            self._forward(message, position)
        elif self._mode == "pass":
            if self._distance(position, self._last_forwarded_position) >= self._jump_threshold:
                self._begin_candidate(position, now)
            else:
                self._forward(message, position)
        elif self._mode == "candidate":
            if self._distance(position, self._last_forwarded_position) < self._jump_threshold:
                rospy.loginfo("MAVROS odom jump candidate disappeared; resuming")
                self._forward(message, position)
            elif self._distance(position, self._candidate_position) > self._candidate_radius:
                self._candidate_position = position
                self._candidate_started = now
                self._reason = "position_jump_unstable"
            elif (now - self._candidate_started).to_sec() >= self._stable_time:
                if self._confirmed_disarmed(now):
                    self._mode, self._reason = "dropout", "disarmed_reseed_dropout"
                else:
                    self._mode, self._reason = "blocked", "armed_or_state_unknown_jump"
                    rospy.logerr("sustained odom jump while armed/state unknown; fail closed")
        elif self._mode == "blocked":
            if self._confirmed_disarmed(now):
                self._mode, self._reason = "dropout", "disarmed_reseed_dropout"
                self._dropout_started = now
                self._candidate_position = position
                rospy.logwarn("vehicle disarmed; preparing estimator reseed dropout")
        elif self._mode == "dropout":
            if not self._confirmed_disarmed(now):
                self._mode, self._reason = "blocked", "armed_or_state_unknown_jump"
            elif self._distance(position, self._candidate_position) > self._candidate_radius:
                self._candidate_position = position
                self._candidate_started = now
                self._reason = "position_jump_unstable"
            elif ((now - self._dropout_started).to_sec() >= self._minimum_dropout and
                  (now - self._candidate_started).to_sec() >= self._stable_time):
                rospy.loginfo("stable disarmed odometry reacquired after reseed dropout")
                self._forward(message, position)
        self._publish_status()

    def _timer_callback(self, _event):
        self._publish_status()

    def _publish_status(self):
        healthy = self._mode == "pass"
        self._healthy.publish(Bool(data=healthy))
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/mavros_odom_recovery_gate"
        status.hardware_id = "system_integration"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = self._mode
        status.values = [KeyValue(key="reason", value=self._reason),
                         KeyValue(key="removable", value="true")]
        array.status = [status]
        self._diagnostics.publish(array)


if __name__ == "__main__":
    rospy.init_node("mavros_odom_recovery_gate")
    MavrosOdomRecoveryGate()
    rospy.spin()
