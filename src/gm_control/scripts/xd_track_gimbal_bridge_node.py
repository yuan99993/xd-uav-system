#!/usr/bin/env python3
"""Bridge xd_uav_track's selected target and gm_control's gimbal state."""

import math
import threading
import time

import rospy

from gm_control.msg import BoundingBox2D, GimbalState as GmGimbalState
from xd_uav_track.msg import GimbalState as TrackGimbalState
from xd_uav_track.msg import TrackStateArray


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

        self.track_timeout_s = max(0.02, float(rospy.get_param("~track_timeout_s", 0.35)))
        self.allow_predicted = bool(rospy.get_param("~allow_predicted", True))
        self.minimum_confidence = max(
            0.0, min(1.0, float(rospy.get_param("~minimum_confidence", 0.0)))
        )
        self.yaw_sign = self._read_sign("~gimbal_state/yaw_sign")
        self.pitch_sign = self._read_sign("~gimbal_state/pitch_sign")
        self.roll_sign = self._read_sign("~gimbal_state/roll_sign")

        self.last_tracks_receive_time = None
        self.bbox_is_valid = False
        self.state_lock = threading.Lock()

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
        self.shutdown_event = threading.Event()
        rospy.on_shutdown(self.shutdown_event.set)
        self.timeout_thread = threading.Thread(
            target=self._timeout_loop,
            name="xd-track-gimbal-timeout",
            daemon=True,
        )
        self.timeout_thread.start()

        rospy.loginfo(
            "xd track/gimbal bridge started: tracks=%s bbox=%s gm_state=%s track_state=%s",
            tracks_topic,
            bbox_topic,
            gm_state_topic,
            track_state_topic,
        )

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

        output = TrackGimbalState()
        output.header = message.header
        output.header.stamp = self._stamp_or_now(output.header.stamp)
        output.yaw_rad = self.yaw_sign * math.radians(message.yaw_deg) if finite else 0.0
        output.pitch_rad = (
            self.pitch_sign * math.radians(message.pitch_deg) if finite else 0.0
        )
        output.roll_rad = self.roll_sign * math.radians(message.roll_deg) if finite else 0.0
        output.valid = bool(message.valid and finite)
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
