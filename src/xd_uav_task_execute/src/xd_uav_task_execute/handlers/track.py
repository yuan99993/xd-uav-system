"""Adapter from ExecuteTask goals to xd_uav_track services and status."""

import threading
import time

import rospy

from xd_uav_task_execute.msg import ExecuteTaskGoal
from xd_uav_task_execute.core.track_monitor import (
    ERROR_ACQUISITION_TIMEOUT,
    ERROR_CONTROL_UNAVAILABLE,
    ERROR_EXECUTION_TIMEOUT,
    ERROR_GIMBAL_UNAVAILABLE,
    ERROR_PROFILE_MISMATCH,
    ERROR_STATUS_TIMEOUT,
    ERROR_TARGET_MISMATCH,
    ERROR_TARGET_LOST,
    ERROR_EMERGENCY_STOP,
    ERROR_TRACKER_STOPPED,
    STATE_FAILED,
    STATE_RUNNING,
    STATE_SUCCEEDED,
    TrackMonitor,
    TrackMonitorPolicy,
)
from xd_uav_track.msg import MetricTarget, TrackStatus
from xd_uav_track.srv import SelectTrack, SetProfile, StartTracker

from .base import HandlerResult


class TrackHandler:
    def __init__(self, config, interfaces, feedback_rate_hz=10.0):
        self._config = dict(config)
        self._feedback_rate_hz = max(1.0, float(feedback_rate_hz))
        self._start_service_name = str(
            interfaces.get("start_tracker_service", "track/start_tracker")
        )
        self._select_service_name = str(
            interfaces.get("select_track_service", "track/select_track")
        )
        self._profile_service_name = str(
            interfaces.get("set_profile_service", "track/set_profile")
        )
        self._metric_target_topic = str(
            interfaces.get("metric_target_topic", "track/metric_target")
        )
        status_topic = str(interfaces.get("status_topic", "track/status"))
        self._start_client = rospy.ServiceProxy(
            self._start_service_name, StartTracker, persistent=False
        )
        self._select_client = rospy.ServiceProxy(
            self._select_service_name, SelectTrack, persistent=False
        )
        self._profile_client = rospy.ServiceProxy(
            self._profile_service_name, SetProfile, persistent=False
        )
        self._metric_target_publisher = None
        if self._metric_target_topic:
            self._metric_target_publisher = rospy.Publisher(
                self._metric_target_topic,
                MetricTarget,
                queue_size=1,
                latch=True,
            )
        self._condition = threading.Condition()
        self._latest_status = None
        self._status_sequence = 0
        self._subscriber = rospy.Subscriber(
            status_topic, TrackStatus, self._status_callback, queue_size=20
        )

    def _status_callback(self, message):
        with self._condition:
            self._latest_status = message
            self._status_sequence += 1
            self._condition.notify_all()

    @staticmethod
    def _effective_profile(goal):
        profile = str(goal.follower_profile).strip()
        if profile:
            return profile
        # OBSERVE and INTERCEPT are task-level defaults; callers can still
        # override them with an explicit follower_profile.
        if int(goal.task_type) == ExecuteTaskGoal.OBSERVE:
            return "fw_metric_orbit"
        if int(goal.task_type) == ExecuteTaskGoal.INTERCEPT:
            return "fw_metric_pursuit"
        return ""

    @staticmethod
    def _is_metric_profile(profile):
        return str(profile).startswith("fw_metric_")

    @staticmethod
    def _metric_target_available(goal):
        pose = goal.target_pose
        return bool(pose.header.frame_id) or any(
            abs(float(value)) >= 1e-9
            for value in (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
        )

    def _publish_metric_target(self, goal, profile):
        if self._metric_target_publisher is None or not self._is_metric_profile(profile):
            return
        pose = goal.target_pose
        if not self._metric_target_available(goal):
            raise RuntimeError(
                "metric fixed-wing task requires target_pose in the shared world frame"
            )
        message = MetricTarget()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = str(pose.header.frame_id)
        message.target_id = int(goal.target_id)
        message.valid = True
        message.pose.pose = pose.pose
        message.has_velocity = False
        message.source = "task_execute"
        self._metric_target_publisher.publish(message)

    def _policy(self, goal):
        profile = self._effective_profile(goal)
        required = (
            float(goal.required_execution_sec)
            if float(goal.required_execution_sec) > 0.0
            else float(self._config.get("required_tracking_sec", 30.0))
        )
        maximum = (
            float(goal.maximum_duration_sec)
            if float(goal.maximum_duration_sec) > 0.0
            else float(self._config.get("maximum_duration_sec", 120.0))
        )
        return TrackMonitorPolicy(
            required_tracking_sec=required,
            acquisition_timeout_sec=float(
                self._config.get("acquisition_timeout_sec", 10.0)
            ),
            target_loss_timeout_sec=float(
                self._config.get("target_loss_timeout_sec", 3.0)
            ),
            status_timeout_sec=float(self._config.get("status_timeout_sec", 1.0)),
            maximum_duration_sec=maximum,
            minimum_tracking_quality=float(
                self._config.get("minimum_tracking_quality", 0.0)
            ),
            allow_predicted=bool(self._config.get("allow_predicted", False)),
            required_track_id=int(goal.local_track_id),
            required_profile=profile,
            require_control_reference=bool(
                self._config.get("require_control_reference", True)
            ),
        )

    def _wait_for_service(self, name):
        rospy.wait_for_service(
            name,
            timeout=max(0.1, float(self._config.get("service_wait_timeout_sec", 2.0))),
        )

    def _start(self, goal, should_stop):
        profile = self._effective_profile(goal)
        if self._is_metric_profile(profile) and not self._metric_target_available(goal):
            raise RuntimeError(
                "metric fixed-wing task requires target_pose in the shared world frame"
            )
        if profile:
            self._wait_for_service(self._profile_service_name)
            response = self._profile_client(profile)
            if not response.success:
                raise RuntimeError("track profile rejected: " + response.message)
        if int(goal.local_track_id) >= 0:
            self._wait_for_service(self._select_service_name)
            timeout = max(
                0.0, float(self._config.get("selection_wait_timeout_sec", 1.0))
            )
            retry_hz = max(
                1.0, float(self._config.get("selection_retry_rate_hz", 20.0))
            )
            deadline = time.monotonic() + timeout
            while True:
                response = self._select_client(
                    target_id=int(goal.local_track_id),
                    start_tracking=True,
                    use_normalized_roi=False,
                    normalized_roi=[0.0, 0.0, 0.0, 0.0],
                    image_source="",
                    capture_timestamp=rospy.Time(),
                )
                if response.success:
                    self._publish_metric_target(goal, profile)
                    return
                if should_stop():
                    raise RuntimeError("track selection interrupted by preemption")
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "track selection rejected after waiting for the detection: "
                        + response.message
                    )
                time.sleep(1.0 / retry_hz)
        self._wait_for_service(self._start_service_name)
        response = self._start_client(True)
        if not response.success or not response.active:
            raise RuntimeError("tracker start rejected: " + response.message)
        # Publish after StartTracker: that service resets the controller and
        # would otherwise discard the one-shot world target.
        self._publish_metric_target(goal, profile)

    def _stop(self):
        self._wait_for_service(self._start_service_name)
        response = self._start_client(False)
        if not response.success or response.active:
            raise RuntimeError("tracker stop rejected: " + response.message)

    @staticmethod
    def _monitor_error_code(error, result_type):
        return {
            ERROR_ACQUISITION_TIMEOUT: result_type.ACQUISITION_TIMEOUT,
            ERROR_TARGET_LOST: result_type.TARGET_LOST,
            ERROR_STATUS_TIMEOUT: result_type.STATUS_TIMEOUT,
            ERROR_EXECUTION_TIMEOUT: result_type.EXECUTION_TIMEOUT,
            ERROR_TARGET_MISMATCH: result_type.TARGET_LOST,
            ERROR_PROFILE_MISMATCH: result_type.START_REJECTED,
            ERROR_CONTROL_UNAVAILABLE: result_type.DEPENDENCY_UNAVAILABLE,
            ERROR_GIMBAL_UNAVAILABLE: result_type.DEPENDENCY_UNAVAILABLE,
            ERROR_TRACKER_STOPPED: result_type.START_REJECTED,
            ERROR_EMERGENCY_STOP: result_type.START_REJECTED,
        }.get(error, result_type.INTERNAL_ERROR)

    def execute(self, goal, should_stop, feedback, result_type):
        started_at = rospy.Time.now().to_sec()
        tracker_started = False
        result = None
        try:
            policy = self._policy(goal)
            # Ignore any latched/sample status from before this action. A
            # stopped status from the previous task must not fail a tracker
            # that has just accepted StartTracker(true).
            with self._condition:
                observed_sequence = self._status_sequence
            feedback("starting", 0.0, "starting xd_uav_track")
            self._start(goal, should_stop)
            tracker_started = True
            monitor = TrackMonitor(rospy.Time.now().to_sec(), policy)
            rate = rospy.Rate(self._feedback_rate_hz)
            while not rospy.is_shutdown():
                if should_stop():
                    result = HandlerResult(
                        False, result_type.PREEMPTED, "track task preempted", True
                    )
                    break
                now = rospy.Time.now().to_sec()
                with self._condition:
                    status = self._latest_status
                    sequence = self._status_sequence
                if status is not None and sequence != observed_sequence:
                    observed_sequence = sequence
                    monitor.observe(
                        now,
                        tracker_active=status.tracker_active,
                        target_visible=status.target_visible,
                        target_predicted=status.target_predicted,
                        metric_target_valid=status.metric_target_valid,
                        metric_active=status.metric_active,
                        orbit_active=status.orbit_active,
                        command_valid=status.command_valid,
                        state_valid=status.state_valid,
                        emergency_stop_active=status.emergency_stop_active,
                        tracking_state=status.tracking_state,
                        tracking_quality=status.tracking_quality,
                        track_id=status.track_id,
                        follower_profile=status.follower_profile,
                        requested_profile=status.requested_profile,
                        control_reference_published=(
                            status.control_reference_published
                        ),
                        gimbal_state_valid=status.gimbal_state_valid,
                        gimbal_fallback_active=status.gimbal_fallback_active,
                        invalid_reason=status.invalid_reason,
                    )
                else:
                    monitor.poll(now)
                phase = "running" if monitor.state == STATE_RUNNING else "acquiring"
                feedback(phase, monitor.progress, monitor.detail)
                if monitor.state == STATE_SUCCEEDED:
                    result = HandlerResult(True, result_type.OK, monitor.detail)
                    break
                if monitor.state == STATE_FAILED:
                    result = HandlerResult(
                        False,
                        self._monitor_error_code(monitor.error, result_type),
                        monitor.detail,
                    )
                    break
                rate.sleep()
            if result is None:
                result = HandlerResult(
                    False, result_type.INTERNAL_ERROR, "ROS shutdown during track task"
                )
        except (rospy.ROSException, rospy.ServiceException) as error:
            result = HandlerResult(
                False,
                result_type.DEPENDENCY_UNAVAILABLE,
                f"track dependency unavailable: {error}",
            )
        except (RuntimeError, ValueError) as error:
            result = HandlerResult(False, result_type.START_REJECTED, str(error))
        finally:
            if tracker_started:
                feedback("stopping", 1.0 if result and result.success else 0.0,
                         "stopping xd_uav_track")
                try:
                    self._stop()
                except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
                    result = HandlerResult(
                        False,
                        result_type.STOP_FAILED,
                        f"track action ended but tracker could not be stopped: {error}",
                    )
        return result, max(0.0, rospy.Time.now().to_sec() - started_at)
