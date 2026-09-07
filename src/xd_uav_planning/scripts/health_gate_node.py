#!/usr/bin/env python3
"""Publish the fail-closed conjunction of configured health inputs."""

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool

from xd_uav_planning.core import health_conjunction


class HealthGate:
    def __init__(self):
        topics = rospy.get_param("~inputs", [])
        if not isinstance(topics, list) or not topics:
            raise rospy.ROSInitException("~inputs must be a non-empty topic list")
        self._timeout = float(rospy.get_param("~timeout", 0.30))
        output = rospy.get_param("~output", "integration/healthy")
        diagnostics = rospy.get_param(
            "~diagnostics", "integration/health_diagnostics")
        self._states = {topic: (False, None) for topic in topics}
        self._subscribers = [
            rospy.Subscriber(topic, Bool, self._callback,
                             callback_args=topic, queue_size=1)
            for topic in topics
        ]
        self._publisher = rospy.Publisher(output, Bool, queue_size=1, latch=True)
        self._diagnostics = rospy.Publisher(
            diagnostics, DiagnosticArray, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(0.10), self._timer_callback)
        self._publish(False, ["waiting_for_inputs"])

    def _callback(self, message, topic):
        self._states[topic] = (bool(message.data), rospy.Time.now())

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        values = []
        reasons = []
        for topic, (value, stamp) in self._states.items():
            if stamp is None or (now - stamp).to_sec() > self._timeout:
                values.append(False)
                reasons.append(topic + ":stale")
            else:
                values.append(value)
                if not value:
                    reasons.append(topic + ":false")
        healthy = health_conjunction(values, len(self._states))
        self._publish(healthy, reasons)

    def _publish(self, healthy, reasons):
        self._publisher.publish(Bool(data=healthy))
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/health_gate"
        status.hardware_id = "system_integration"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = "healthy" if healthy else "fail_closed"
        status.values = [KeyValue(key="reason", value=reason)
                         for reason in (reasons or ["ok"])]
        array.status = [status]
        self._diagnostics.publish(array)


def main():
    rospy.init_node("health_gate")
    HealthGate()
    rospy.spin()


if __name__ == "__main__":
    main()
