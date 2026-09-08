#!/usr/bin/env python3
"""Derive fail-closed source health from a PositionTarget candidate stream."""

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool

from xd_uav_system_integration.core import ReferenceSample, validate_reference


class ReferenceHealth:
    def __init__(self):
        self._frame = rospy.get_param("~common_frame", "world").strip("/")
        self._timeout = float(rospy.get_param("~timeout", 0.20))
        self._future_tolerance = float(rospy.get_param("~future_tolerance", 0.02))
        self._last_stamp = None
        self._valid = False
        self._reason = "candidate_not_received"
        self._publisher = rospy.Publisher(
            rospy.get_param("~healthy_topic", "reference_healthy"),
            Bool, queue_size=1, latch=True)
        self._diagnostics = rospy.Publisher(
            rospy.get_param("~diagnostics_topic", "reference_health_diagnostics"),
            DiagnosticArray, queue_size=1)
        self._subscriber = rospy.Subscriber(
            rospy.get_param("~candidate_topic", "reference_candidate"),
            PositionTarget, self._callback, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(0.05), self._timer_callback)
        self._publish(False)

    @staticmethod
    def _vector(value):
        return (float(value.x), float(value.y), float(value.z))

    def _sample(self, message):
        return ReferenceSample(
            stamp=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id.strip("/"),
            coordinate_frame=int(message.coordinate_frame),
            type_mask=int(message.type_mask),
            position=self._vector(message.position),
            velocity=self._vector(message.velocity),
            acceleration=self._vector(message.acceleration_or_force),
            yaw=float(message.yaw), yaw_rate=float(message.yaw_rate))

    def _callback(self, message):
        sample = self._sample(message)
        result = validate_reference(
            sample, rospy.Time.now().to_sec(), self._frame, self._timeout,
            self._future_tolerance, PositionTarget.FRAME_LOCAL_NED,
            PositionTarget.FORCE)
        self._valid = result.valid
        self._reason = result.reason
        self._last_stamp = message.header.stamp if result.valid else None
        self._publish(result.valid)

    def _timer_callback(self, _event):
        healthy = False
        if self._valid and self._last_stamp is not None:
            age = (rospy.Time.now() - self._last_stamp).to_sec()
            healthy = -self._future_tolerance <= age <= self._timeout
            if not healthy:
                self._reason = "candidate_stream_stale"
        self._publish(healthy)

    def _publish(self, healthy):
        self._publisher.publish(Bool(data=healthy))
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/reference_health"
        status.hardware_id = "system_integration"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = "healthy" if healthy else "fail_closed"
        status.values = [KeyValue(key="reason", value=self._reason),
                         KeyValue(key="frame", value=self._frame)]
        array.status = [status]
        self._diagnostics.publish(array)


def main():
    rospy.init_node("reference_health")
    ReferenceHealth()
    rospy.spin()


if __name__ == "__main__":
    main()
