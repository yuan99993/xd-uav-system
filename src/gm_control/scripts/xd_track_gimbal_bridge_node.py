#!/usr/bin/env python3
"""Bridge xd_uav_track's selected target and gm_control's gimbal state."""

import math
import threading
import time

import rospy

from gm_control.msg import BoundingBox2D, GimbalState as GmGimbalState
from gm_control.srv import StartGimbalSearch, StartGimbalTracking
from nav_msgs.msg import Odometry
from xd_uav_task_allocate.msg import RescueTask, RescueTaskArray
from xd_uav_track.msg import GimbalState as TrackGimbalState
from xd_uav_track.msg import TrackStateArray, TrackStatus


class XdTrackGimbalBridgeNode:
    def __init__(self):
        tracks_topic = rospy.get_param("~tracks_topic", "track/tracks")
        bbox_topic = rospy.get_param("~bbox_topic", "gm_control/target_bbox")
        gm_state_topic = rospy.get_param(
            "~gm_gimbal_state_topic", "gm_control/gimbal_state"
        )
        track_state_topic = rospy.get_param(
            "~track_gimbal_state_topic", "track/gimbal_state"
        )
        track_status_topic = rospy.get_param("~track_status_topic", "track/status")
        gimbal_tracking_service = rospy.get_param(
            "~gimbal_tracking_service", "gm_control/start_tracking"
        )
        gimbal_search_service = rospy.get_param(
            "~gimbal_search_service", "gm_control/start_search"
        )
        worker_name = str(rospy.get_param("~worker_name", "")).strip()
        if not worker_name:
            worker_name = rospy.get_namespace().strip("/").split("/")[-1]
        rescue_tasks_topic = rospy.get_param(
            "~proximity_search/rescue_tasks_topic",
            "/task_allocate/rescue_tasks",
        )
        worker_odometry_topic = rospy.get_param(
            "~proximity_search/worker_odometry_topic",
            "state_estimator/main/odom",
        )

        self.track_timeout_s = max(0.02, float(rospy.get_param("~track_timeout_s", 0.35)))
        self.allow_predicted = bool(rospy.get_param("~allow_predicted", True))
        self.minimum_confidence = max(
            0.0, min(1.0, float(rospy.get_param("~minimum_confidence", 0.0)))
        )
        self.auto_gimbal_tracking = bool(
            rospy.get_param("~auto_gimbal_tracking", True)
        )
        # Search is an explicit operator action. Keep the parameter for
        # backwards-compatible launch files, but default it off so a lost
        # target or proximity event cannot start a scan by itself.
        self.auto_gimbal_search = bool(rospy.get_param("~auto_gimbal_search", False))
        self.proximity_search_enabled = bool(
            rospy.get_param("~proximity_search/enabled", True)
        )
        self.proximity_search_radius_m = max(
            0.0, float(rospy.get_param("~proximity_search/radius_m", 20.0))
        )
        self.worker_name = worker_name
        self.gimbal_service_retry_s = max(
            0.05, float(rospy.get_param("~gimbal_service_retry_s", 0.5))
        )
        self.yaw_sign = self._read_sign("~gimbal_state/yaw_sign")
        self.pitch_sign = self._read_sign("~gimbal_state/pitch_sign")
        self.roll_sign = self._read_sign("~gimbal_state/roll_sign")

        self.last_tracks_receive_time = None
        self.bbox_is_valid = False
        self.gimbal_tracking_active = False
        self.gimbal_search_active = False
        self.track_requests_gimbal_tracking = False
        self.proximity_search_requested = False
        self.worker_position = None
        self.assigned_task_positions = []
        self.last_gimbal_service_attempt = 0.0
        self.state_lock = threading.Lock()
        self.lifecycle_lock = threading.Lock()

        self.bbox_publisher = rospy.Publisher(
            bbox_topic, BoundingBox2D, queue_size=5
        )
        self.track_state_publisher = rospy.Publisher(
            track_state_topic, TrackGimbalState, queue_size=10
        )
        self.tracks_subscriber = rospy.Subscriber(
            tracks_topic, TrackStateArray, self._tracks_callback, queue_size=1
        )
        self.gm_state_subscriber = rospy.Subscriber(
            gm_state_topic, GmGimbalState, self._gimbal_state_callback, queue_size=10
        )
        self.gimbal_tracking_client = rospy.ServiceProxy(
            gimbal_tracking_service, StartGimbalTracking, persistent=False
        )
        self.gimbal_search_client = rospy.ServiceProxy(
            gimbal_search_service, StartGimbalSearch, persistent=False
        )
        self.track_status_subscriber = rospy.Subscriber(
            track_status_topic, TrackStatus, self._track_status_callback, queue_size=1
        )
        self.rescue_tasks_subscriber = rospy.Subscriber(
            rescue_tasks_topic,
            RescueTaskArray,
            self._rescue_tasks_callback,
            queue_size=1,
        )
        self.worker_odometry_subscriber = rospy.Subscriber(
            worker_odometry_topic,
            Odometry,
            self._worker_odometry_callback,
            queue_size=1,
        )
        self.shutdown_event = threading.Event()
        rospy.on_shutdown(self._shutdown)
        self.timeout_thread = threading.Thread(
            target=self._timeout_loop,
            name="xd-track-gimbal-timeout",
            daemon=True,
        )
        self.timeout_thread.start()

        rospy.loginfo(
            "xd track/gimbal bridge started: tracks=%s bbox=%s gm_state=%s "
            "track_state=%s track_status=%s tracking_service=%s "
            "search_service=%s auto_start=%s auto_search=%s "
            "proximity_search=%s worker=%s radius=%.1fm tasks=%s odometry=%s",
            tracks_topic,
            bbox_topic,
            gm_state_topic,
            track_state_topic,
            track_status_topic,
            rospy.resolve_name(gimbal_tracking_service),
            rospy.resolve_name(gimbal_search_service),
            self.auto_gimbal_tracking,
            self.auto_gimbal_search,
            self.proximity_search_enabled,
            self.worker_name,
            self.proximity_search_radius_m,
            rescue_tasks_topic,
            worker_odometry_topic,
        )

    @staticmethod
    def _status_requests_gimbal_tracking(message):
        return bool(
            message.tracker_active
            and not message.emergency_stop_active
            and str(message.requested_profile).startswith("gm_velocity_")
        )

    def _track_status_callback(self, message):
        desired = bool(
            self.auto_gimbal_tracking
            and self._status_requests_gimbal_tracking(message)
        )
        with self.state_lock:
            self.track_requests_gimbal_tracking = desired
        self._reconcile_gimbal_lifecycle(message.requested_profile)

    def _rescue_tasks_callback(self, message):
        active_statuses = (RescueTask.ASSIGNED, RescueTask.EXECUTING)
        positions = [
            (float(task.goal.x), float(task.goal.y))
            for task in message.tasks
            if task.assigned_worker == self.worker_name
            and int(task.status) in active_statuses
        ]
        with self.state_lock:
            self.assigned_task_positions = positions
        self._refresh_proximity_search()

    def _worker_odometry_callback(self, message):
        position = message.pose.pose.position
        with self.state_lock:
            self.worker_position = (float(position.x), float(position.y))
        self._refresh_proximity_search()

    def _refresh_proximity_search(self):
        with self.state_lock:
            worker_position = self.worker_position
            task_positions = list(self.assigned_task_positions)
            previous = self.proximity_search_requested
            desired = bool(
                self.proximity_search_enabled
                and worker_position is not None
                and any(
                    math.hypot(
                        worker_position[0] - task_position[0],
                        worker_position[1] - task_position[1],
                    )
                    <= self.proximity_search_radius_m
                    for task_position in task_positions
                )
            )
            self.proximity_search_requested = desired
        if desired != previous:
            rospy.loginfo(
                "gm_control proximity search %s for worker %s (radius %.1f m)",
                "requested" if desired else "released",
                self.worker_name,
                self.proximity_search_radius_m,
            )
        self._reconcile_gimbal_lifecycle()

    def _reconcile_gimbal_lifecycle(self, requested_profile=""):
        if not self.lifecycle_lock.acquire(False):
            return
        try:
            self._reconcile_gimbal_lifecycle_locked(requested_profile)
        finally:
            self.lifecycle_lock.release()

    def _reconcile_gimbal_lifecycle_locked(self, requested_profile=""):
        now = time.monotonic()
        with self.state_lock:
            desired_tracking = self.track_requests_gimbal_tracking
            desired_search = bool(
                self.auto_gimbal_search
                and (desired_tracking or self.proximity_search_requested)
            )
            tracking_change = desired_tracking != self.gimbal_tracking_active
            search_change = desired_search != self.gimbal_search_active
            if not tracking_change and not search_change:
                return
            if now - self.last_gimbal_service_attempt < self.gimbal_service_retry_s:
                return
            self.last_gimbal_service_attempt = now

        if tracking_change:
            if not self._call_gimbal_gate(
                self.gimbal_tracking_client, "tracking", desired_tracking
            ):
                return
            with self.state_lock:
                self.gimbal_tracking_active = desired_tracking
                if not desired_tracking:
                    # StartGimbalTracking(false) also disables search inside
                    # gm_control, so keep the local lifecycle mirror aligned.
                    self.gimbal_search_active = False
                else:
                    # Enabling tracking also resets search in gm_control.
                    self.gimbal_search_active = False
            rospy.loginfo(
                "gm_control tracking automatically %s for xd_uav_track profile %s",
                "enabled" if desired_tracking else "disabled",
                requested_profile or "<unset>",
            )

        with self.state_lock:
            desired_search = bool(
                self.auto_gimbal_search
                and (
                    self.track_requests_gimbal_tracking
                    or self.proximity_search_requested
                )
            )
            search_change = desired_search != self.gimbal_search_active
        if search_change:
            if self._call_gimbal_gate(
                self.gimbal_search_client, "search", desired_search
            ):
                with self.state_lock:
                    self.gimbal_search_active = desired_search
                rospy.loginfo(
                    "gm_control search automatically %s (%s)",
                    "enabled" if desired_search else "disabled",
                    "track active"
                    if self.track_requests_gimbal_tracking
                    else "worker proximity",
                )

    @staticmethod
    def _gate_response_matches(response, desired):
        return bool(response.success and bool(response.active) == bool(desired))

    def _call_gimbal_gate(self, client, label, desired):
        try:
            response = client(start=desired)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(
                2.0,
                "cannot switch gm_control %s to %s: %s",
                label,
                desired,
                error,
            )
            return False
        if not self._gate_response_matches(response, desired):
            rospy.logwarn_throttle(
                2.0,
                "gm_control rejected automatic %s=%s: %s",
                label,
                desired,
                response.message,
            )
            return False
        return True

    def _shutdown(self):
        self.shutdown_event.set()
        with self.state_lock:
            tracking_active = self.gimbal_tracking_active
            search_active = self.gimbal_search_active
        try:
            if tracking_active:
                self.gimbal_tracking_client(start=False)
            elif search_active:
                self.gimbal_search_client(start=False)
        except rospy.ServiceException:
            pass

    @staticmethod
    def _read_sign(name):
        value = float(rospy.get_param(name, 1.0))
        return -1.0 if value < 0.0 else 1.0

    @staticmethod
    def _stamp_or_now(stamp):
        return stamp if stamp != rospy.Time() else rospy.Time.now()

    def _invalid_bbox(self, header=None):
        output = BoundingBox2D()
        if header is not None:
            output.header = header
            output.header.stamp = self._stamp_or_now(output.header.stamp)
        else:
            output.header.stamp = rospy.Time.now()
        output.valid = False
        return output

    def _tracks_callback(self, message):
        with self.state_lock:
            self.last_tracks_receive_time = time.monotonic()
        selected = next((track for track in message.tracks if track.selected), None)

        lifecycle_valid = selected is not None and (
            selected.lifecycle_state == "confirmed"
            or (self.allow_predicted and selected.lifecycle_state == "occluded")
        )
        if not lifecycle_valid:
            self.bbox_publisher.publish(self._invalid_bbox(message.header))
            with self.state_lock:
                self.bbox_is_valid = False
            return

        x_min, y_min, x_max, y_max = selected.bbox
        width = x_max - x_min
        height = y_max - y_min
        values = (x_min, y_min, width, height, selected.confidence)
        valid = (
            all(math.isfinite(float(value)) for value in values)
            and width > 0
            and height > 0
            and selected.confidence >= self.minimum_confidence
        )
        if not valid:
            self.bbox_publisher.publish(self._invalid_bbox(message.header))
            with self.state_lock:
                self.bbox_is_valid = False
            return

        output = BoundingBox2D()
        output.header = message.header
        output.header.stamp = self._stamp_or_now(output.header.stamp)
        output.valid = True
        output.x = float(x_min)
        output.y = float(y_min)
        output.width = float(width)
        output.height = float(height)
        output.confidence = float(selected.confidence)
        output.target_id = str(selected.track_id)
        self.bbox_publisher.publish(output)
        with self.state_lock:
            self.bbox_is_valid = True

    def _gimbal_state_callback(self, message):
        angles_deg = (message.yaw_deg, message.pitch_deg, message.roll_deg)
        finite = all(math.isfinite(float(value)) for value in angles_deg)
        with self.state_lock:
            lifecycle_ready = (
                not self.auto_gimbal_tracking or self.gimbal_tracking_active
            )

        output = TrackGimbalState()
        output.header = message.header
        output.header.stamp = self._stamp_or_now(output.header.stamp)
        output.yaw_rad = self.yaw_sign * math.radians(message.yaw_deg) if finite else 0.0
        output.pitch_rad = (
            self.pitch_sign * math.radians(message.pitch_deg) if finite else 0.0
        )
        output.roll_rad = self.roll_sign * math.radians(message.roll_deg) if finite else 0.0
        # In automatic mode, a physical angle sample alone is insufficient:
        # track must not enter a GM follower until the controller's tracking
        # gate has acknowledged enablement.
        output.valid = bool(message.valid and finite and lifecycle_ready)
        self.track_state_publisher.publish(output)

    def _timeout_loop(self):
        period_s = max(0.02, min(0.10, self.track_timeout_s * 0.5))
        while not self.shutdown_event.wait(period_s):
            timed_out = False
            with self.state_lock:
                if (
                    self.bbox_is_valid
                    and self.last_tracks_receive_time is not None
                    and time.monotonic() - self.last_tracks_receive_time
                    > self.track_timeout_s
                ):
                    self.bbox_is_valid = False
                    timed_out = True
            if timed_out:
                self.bbox_publisher.publish(self._invalid_bbox())
                rospy.logwarn_throttle(
                    2.0,
                    "xd_uav_track selected-track stream timed out; "
                    "invalidating gimbal target",
                )


def main():
    rospy.init_node("xd_track_gimbal_bridge")
    XdTrackGimbalBridgeNode()
    rospy.spin()


if __name__ == "__main__":
    main()
