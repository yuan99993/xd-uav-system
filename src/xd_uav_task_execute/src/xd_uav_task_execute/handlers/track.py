"""Adapter from ExecuteTask goals to xd_uav_track services and status."""

import threading

import rospy

from xd_uav_task_execute.core.track_monitor import (
    ERROR_ACQUISITION_TIMEOUT,
    ERROR_EXECUTION_TIMEOUT,
    ERROR_STATUS_TIMEOUT,
    ERROR_TARGET_LOST,
    STATE_FAILED,
    STATE_RUNNING,
    STATE_SUCCEEDED,
    TrackMonitor,
    TrackMonitorPolicy,
)
from xd_uav_track.msg import TrackStatus
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

    def _policy(self, goal):
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
        )

    def _wait_for_service(self, name):
        rospy.wait_for_service(
            name,
            timeout=max(0.1, float(self._config.get("service_wait_timeout_sec", 2.0))),
        )

    def _start(self, goal):
        if str(goal.follower_profile).strip():
            self._wait_for_service(self._profile_service_name)
            response = self._profile_client(str(goal.follower_profile).strip())
            if not response.success:
                raise RuntimeError("track profile rejected: " + response.message)
        if int(goal.local_track_id) >= 0:
            self._wait_for_service(self._select_service_name)
            response = self._select_client(
                target_id=int(goal.local_track_id),
                start_tracking=True,
                use_normalized_roi=False,
                normalized_roi=[0.0, 0.0, 0.0, 0.0],
                image_source="",
                capture_timestamp=rospy.Time(),
            )
            if not response.success:
                raise RuntimeError("track selection rejected: " + response.message)
            return
        self._wait_for_service(self._start_service_name)
        response = self._start_client(True)
        if not response.success or not response.active:
            raise RuntimeError("tracker start rejected: " + response.message)

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
            self._start(goal)
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
                        command_valid=status.command_valid,
                        state_valid=status.state_valid,
                        emergency_stop_active=status.emergency_stop_active,
                        tracking_state=status.tracking_state,
                        tracking_quality=status.tracking_quality,
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
