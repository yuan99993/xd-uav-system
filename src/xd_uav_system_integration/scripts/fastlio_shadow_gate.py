#!/usr/bin/env python3
"""Fail-closed Fast-LIO relay with explicit ground and flight lifecycle."""

from collections import deque
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from xd_uav_state_estimators.msg import EstimatorStatus
from xd_uav_state_estimators.srv import SwitchLocalizationSource

from xd_uav_system_integration.core import OdometrySample, validate_shadow_odometry


class FastlioShadowGate:
    def __init__(self):
        self._lock = threading.Lock()
        self._parent = rospy.get_param("~expected_parent_frame")
        self._child = rospy.get_param("~expected_child_frame")
        self._timeout = float(rospy.get_param("~odometry_timeout_s", 0.30))
        self._future = float(rospy.get_param("~future_tolerance_s", 0.02))
        self._state_timeout = float(rospy.get_param("~vehicle_state_timeout_s", 0.50))
        self._minimum_rate = float(rospy.get_param("~minimum_odometry_rate_hz", 5.0))
        self._maximum_ground_speed = float(
            rospy.get_param("~maximum_ground_speed_mps", 0.50))
        self._maximum_flight_speed = float(
            rospy.get_param("~maximum_flight_speed_mps", 1.50))
        self._maximum_horizontal_radius = float(
            rospy.get_param("~maximum_horizontal_radius_m", 2.0))
        self._minimum_height = float(rospy.get_param(
            "~minimum_height_m", -0.50))
        self._maximum_height = float(rospy.get_param(
            "~maximum_height_m", 2.0))
        self._switch_service = rospy.get_param(
            "~estimator_switch_service", "state_estimator/switch_source")
        self._switch_timeout = float(rospy.get_param(
            "~estimator_switch_timeout_s", 4.0))
        self._fastlio_source = rospy.get_param("~fastlio_source", "fastlio")
        self._fallback_source = rospy.get_param("~fallback_source", "mavros")
        self._sample = None
        self._reason = "shadow_not_received"
        self._receive_times = deque(maxlen=20)
        self._state = None
        self._state_received = None
        self._active_source = ""
        self._relay_authorized = False
        self._flight_authorized = False

        self._gated_odom_pub = rospy.Publisher(
            rospy.get_param("~gated_odometry_topic", "fastlio_gated/Odometry"),
            Odometry, queue_size=20)

        self._healthy_pub = rospy.Publisher(
            rospy.get_param("~healthy_topic", "fastlio_shadow/healthy"),
            Bool, queue_size=1, latch=True)
        self._diagnostics_pub = rospy.Publisher(
            rospy.get_param("~diagnostics_topic", "fastlio_shadow/diagnostics"),
            DiagnosticArray, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odometry_topic", "fastlio/Odometry"),
                         Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(rospy.get_param("~vehicle_state_topic", "mavros/state"),
                         State, self._state_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~estimator_status_topic", "state_estimator/status"),
                         EstimatorStatus, self._status_callback, queue_size=10)
        rospy.Service("~switch_to_fastlio", Trigger, self._switch_fastlio)
        rospy.Service("~authorize_flight", Trigger, self._authorize_flight)
        rospy.Service("~end_flight", Trigger, self._end_flight)
        rospy.Service("~restore_fallback", Trigger, self._restore_fallback)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)

    @staticmethod
    def _v3(value):
        return (float(value.x), float(value.y), float(value.z))

    def _odom_callback(self, message):
        pose_cov = message.pose.covariance
        twist_cov = message.twist.covariance
        sample = OdometrySample(
            stamp=message.header.stamp.to_sec(),
            parent_frame=message.header.frame_id.strip("/"),
            child_frame=message.child_frame_id.strip("/"),
            position=self._v3(message.pose.pose.position),
            orientation=(float(message.pose.pose.orientation.x),
                         float(message.pose.pose.orientation.y),
                         float(message.pose.pose.orientation.z),
                         float(message.pose.pose.orientation.w)),
            linear_velocity=self._v3(message.twist.twist.linear),
            pose_variance=(float(pose_cov[0]), float(pose_cov[7]),
                           float(pose_cov[14])),
            velocity_variance=(float(twist_cov[0]), float(twist_cov[7]),
                               float(twist_cov[14])))
        with self._lock:
            self._sample = sample
            self._receive_times.append(rospy.get_time())
            healthy, _reason, _rate = self._health_locked()
            vehicle_allowed, _state_reason = self._vehicle_allowed_locked()
            relay = self._relay_authorized and healthy and vehicle_allowed
        if relay:
            self._gated_odom_pub.publish(message)

    def _state_callback(self, message):
        with self._lock:
            self._state = message
            self._state_received = rospy.get_time()

    def _status_callback(self, message):
        with self._lock:
            self._active_source = message.active_source

    def _health_locked(self):
        if self._sample is None:
            return False, "shadow_not_received", 0.0
        armed = self._state is not None and self._state.armed
        maximum_speed = (self._maximum_flight_speed if armed and
                         self._flight_authorized else
                         self._maximum_ground_speed)
        result = validate_shadow_odometry(
            self._sample, rospy.Time.now().to_sec(), self._parent, self._child,
            self._timeout, self._future, maximum_speed)
        if not result.valid:
            return False, result.reason, 0.0
        horizontal_radius = (self._sample.position[0] ** 2 +
                             self._sample.position[1] ** 2) ** 0.5
        if armed and horizontal_radius > self._maximum_horizontal_radius:
            return False, "shadow_horizontal_envelope_exceeded", 0.0
        if armed and not (self._minimum_height <= self._sample.position[2] <=
                          self._maximum_height):
            return False, "shadow_height_envelope_exceeded", 0.0
        rate = 0.0
        if len(self._receive_times) >= 2:
            span = self._receive_times[-1] - self._receive_times[0]
            if span > 0.0:
                rate = (len(self._receive_times) - 1) / span
        if rate < self._minimum_rate:
            return False, "shadow_rate_low", rate
        return True, "ok", rate

    def _vehicle_disarmed_locked(self):
        if self._state is None or self._state_received is None:
            return False, "vehicle_state_not_received"
        if rospy.get_time() - self._state_received > self._state_timeout:
            return False, "vehicle_state_stale"
        if self._state.armed:
            return False, "vehicle_armed"
        return True, "ok"

    def _vehicle_allowed_locked(self):
        if self._state is None or self._state_received is None:
            return False, "vehicle_state_not_received"
        if rospy.get_time() - self._state_received > self._state_timeout:
            return False, "vehicle_state_stale"
        if self._state.armed and not self._flight_authorized:
            return False, "armed_without_flight_authorization"
        return True, "ok"

    def _call_switch(self, source):
        try:
            rospy.wait_for_service(self._switch_service, timeout=1.0)
            response = rospy.ServiceProxy(
                self._switch_service, SwitchLocalizationSource)(source)
            if response.success:
                with self._lock:
                    self._active_source = response.active_source
            return TriggerResponse(
                success=response.success,
                message=response.message + "; active=" + response.active_source)
        except (rospy.ROSException, rospy.ServiceException) as error:
            return TriggerResponse(False, "switch_service_failed: {}".format(error))

    def _call_switch_until(self, source):
        deadline = rospy.get_time() + self._switch_timeout
        last = TriggerResponse(False, "switch_not_attempted")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            with self._lock:
                disarmed, reason = self._vehicle_disarmed_locked()
                healthy, health_reason, _rate = self._health_locked()
                authorized = self._relay_authorized
            if not authorized:
                return TriggerResponse(False, "relay_not_authorized")
            if not disarmed:
                return TriggerResponse(False, reason)
            if not healthy:
                return TriggerResponse(False, health_reason)
            last = self._call_switch(source)
            if last.success:
                return last
            rate.sleep()
        return TriggerResponse(
            False, "switch_timeout: {}".format(last.message))

    def _switch_fastlio(self, _request):
        with self._lock:
            disarmed, reason = self._vehicle_disarmed_locked()
            healthy, health_reason, _rate = self._health_locked()
        if not disarmed:
            return TriggerResponse(False, reason)
        if not healthy:
            return TriggerResponse(False, health_reason)
        with self._lock:
            self._relay_authorized = True
        response = self._call_switch_until(self._fastlio_source)
        if not response.success:
            with self._lock:
                self._relay_authorized = False
        return response

    def _authorize_flight(self, _request):
        with self._lock:
            disarmed, reason = self._vehicle_disarmed_locked()
            healthy, health_reason, _rate = self._health_locked()
            relay = self._relay_authorized
            active = self._active_source
        if not disarmed:
            return TriggerResponse(False, reason)
        if not healthy:
            return TriggerResponse(False, health_reason)
        if not relay or active != self._fastlio_source:
            return TriggerResponse(False, "fastlio_not_active_and_authorized")
        with self._lock:
            self._flight_authorized = True
        return TriggerResponse(True, "flight_lifecycle_authorized")

    def _end_flight(self, _request):
        with self._lock:
            disarmed, reason = self._vehicle_disarmed_locked()
            if disarmed:
                self._flight_authorized = False
        if not disarmed:
            return TriggerResponse(False, reason)
        return TriggerResponse(True, "flight_lifecycle_closed")

    def _restore_fallback(self, _request):
        with self._lock:
            disarmed, reason = self._vehicle_disarmed_locked()
            self._relay_authorized = False
            self._flight_authorized = False
        if not disarmed:
            return TriggerResponse(False, reason)
        return self._call_switch(self._fallback_source)

    def _timer_callback(self, _event):
        with self._lock:
            healthy, reason, rate = self._health_locked()
            self._reason = reason
            active = self._active_source
            vehicle_allowed, state_reason = self._vehicle_allowed_locked()
            if self._relay_authorized and (not healthy or not vehicle_allowed):
                self._relay_authorized = False
            relay_authorized = self._relay_authorized
            flight_authorized = self._flight_authorized
        self._healthy_pub.publish(Bool(data=healthy))
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR,
            name=rospy.get_name() + "/health", hardware_id="fastlio_shadow",
            message="healthy" if healthy else "fail_closed")
        status.values = [
            KeyValue("reason", reason), KeyValue("rate_hz", "{:.3f}".format(rate)),
            KeyValue("expected_parent", self._parent),
            KeyValue("expected_child", self._child),
            KeyValue("maximum_ground_speed_mps", "{:.3f}".format(self._maximum_ground_speed)),
            KeyValue("maximum_flight_speed_mps", "{:.3f}".format(self._maximum_flight_speed)),
            KeyValue("relay_authorized", str(relay_authorized).lower()),
            KeyValue("flight_authorized", str(flight_authorized).lower()),
            KeyValue("vehicle_gate", "ok" if vehicle_allowed else state_reason),
            KeyValue("active_source", active)]
        array.status = [status]
        self._diagnostics_pub.publish(array)


if __name__ == "__main__":
    rospy.init_node("fastlio_shadow_gate")
    FastlioShadowGate()
    rospy.spin()
