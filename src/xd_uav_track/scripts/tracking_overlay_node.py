#!/usr/bin/env python3
"""Render xd_uav_track state on its capture-time camera image.

This node is presentation-only: it never changes detections, track IDs or
control references.  A short bounded image cache lets a delayed tracker frame
be overlaid on the exact image that produced it instead of on the newest frame.
"""

from __future__ import annotations

from collections import deque
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from xd_uav_track.msg import TrackStateArray, TrackStatus


class TrackingOverlay:
    def __init__(self) -> None:
        self._source = str(rospy.get_param("~image_source", "")).strip()
        if not self._source:
            raise ValueError("~image_source must not be empty")
        self._image_topic = str(rospy.get_param("~input_image_topic", "image_raw"))
        self._tracks_topic = str(rospy.get_param("~tracks_topic", "track/tracks"))
        self._status_topic = str(rospy.get_param("~status_topic", "track/status"))
        self._output_topic = str(rospy.get_param("~output_image_topic", "tracking_overlay"))
        self._maximum_age_sec = max(0.05, min(2.0, float(
            rospy.get_param("~maximum_pair_delta_sec", 0.45))))
        self._cache_size = max(4, min(128, int(rospy.get_param(
            "~image_cache_size", 32))))
        self._fallback_render_rate_hz = max(0.0, min(30.0, float(
            rospy.get_param("~fallback_render_rate_hz", 0.0))))
        # A lost track remains in tracker memory for ReID, but its stale
        # projection must not look like a live target to an operator. Short
        # "occluded" predictions remain visible; long-lost boxes are opt-in.
        self._show_lost = bool(rospy.get_param("~show_lost", False))
        self._images = deque(maxlen=self._cache_size)
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._status = None
        # Tracker output normally follows its capture image by one inference
        # interval.  Keep the most recent source-local snapshot and render it
        # on the next camera frame when it remains inside the bounded pairing
        # window.  This makes the GUI robust to asynchronous ROS delivery
        # without feeding anything back into detection/tracking.
        self._latest_tracks = None
        self._paired = 0
        self._dropped_no_image = 0
        self._last_publish_monotonic = 0.0
        self._publisher = rospy.Publisher(self._output_topic, Image, queue_size=1)
        self._image_sub = rospy.Subscriber(self._image_topic, Image,
                                           self._image_callback, queue_size=4,
                                           buff_size=16 * 1024 * 1024,
                                           tcp_nodelay=True)
        self._tracks_sub = rospy.Subscriber(self._tracks_topic, TrackStateArray,
                                            self._tracks_callback, queue_size=8,
                                            tcp_nodelay=True)
        self._status_sub = rospy.Subscriber(self._status_topic, TrackStatus,
                                            self._status_callback, queue_size=4,
                                            tcp_nodelay=True)
        self._fallback_timer = None
        if self._fallback_render_rate_hz > 0.0:
            self._fallback_timer = rospy.Timer(
                rospy.Duration(1.0 / self._fallback_render_rate_hz),
                self._fallback_render_callback)
        rospy.loginfo("tracking overlay ready: source=%s image=%s tracks=%s output=%s",
                      self._source, self._image_topic, self._tracks_topic,
                      self._output_topic)

    def _image_callback(self, message: Image) -> None:
        if message.header.stamp.is_zero():
            return
        with self._lock:
            self._images.append(message)
            tracks = self._latest_tracks
            status = self._status
        if tracks is None or self._publisher.get_num_connections() == 0:
            return
        delta = abs((message.header.stamp - tracks.header.stamp).to_sec())
        if delta > self._maximum_age_sec:
            return
        self._publish_overlay(message, tracks, status)

    def _status_callback(self, message: TrackStatus) -> None:
        with self._lock:
            self._status = message

    def _fallback_render_callback(self, _event) -> None:
        """Keep a GUI responsive when image and detection callbacks race.

        This is deliberately presentation-only and uses the newest bounded
        image/track pair.  It is disabled by default so deployed tracker
        pipelines pay no additional conversion cost.
        """
        if self._publisher.get_num_connections() == 0:
            return
        # The regular callbacks already render at camera rate in the common
        # case.  Fall back only after they have gone quiet, avoiding duplicate
        # BGR conversions and publishes when both paths are healthy.
        if (time.monotonic() - self._last_publish_monotonic) < \
                (0.75 / self._fallback_render_rate_hz):
            return
        with self._lock:
            if not self._images or self._latest_tracks is None:
                return
            image = self._images[-1]
            tracks = self._latest_tracks
            status = self._status
        if abs((image.header.stamp - tracks.header.stamp).to_sec()) > self._maximum_age_sec:
            return
        self._publish_overlay(image, tracks, status)

    def _matching_image(self, stamp: rospy.Time):
        with self._lock:
            if not self._images:
                return None, None
            candidate = min(self._images,
                            key=lambda image: abs((image.header.stamp - stamp).to_sec()))
            delta = abs((candidate.header.stamp - stamp).to_sec())
            status = self._status
        return (candidate, status) if delta <= self._maximum_age_sec else (None, status)

    @staticmethod
    def _color(track):
        if track.selected:
            return (0, 255, 0)
        if track.predicted or not track.detected:
            return (0, 190, 255)
        if track.lifecycle_state == "tentative":
            return (180, 180, 180)
        return (255, 210, 0)

    @staticmethod
    def _label(track) -> str:
        kind = "PRED" if track.predicted or not track.detected else "DET"
        ready = " READY" if track.control_measurement_ready else ""
        return "ID {} c{} {:.2f} {}{}".format(
            track.track_id, track.class_id, track.confidence, kind, ready)

    def _draw(self, image, tracks: TrackStateArray, status) -> None:
        height, width = image.shape[:2]
        visible_tracks = 0
        hidden_lost = 0
        for track in tracks.tracks:
            if track.lifecycle_state == "lost" and not self._show_lost:
                hidden_lost += 1
                continue
            x1, y1, x2, y2 = [int(value) for value in track.bbox]
            x1 = max(0, min(width - 1, x1))
            x2 = max(0, min(width - 1, x2))
            y1 = max(0, min(height - 1, y1))
            y2 = max(0, min(height - 1, y2))
            if x2 <= x1 or y2 <= y1:
                continue
            visible_tracks += 1
            color = self._color(track)
            thickness = 3 if track.selected else 2
            cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
            text = self._label(track)
            baseline = max(18, y1 - 6)
            cv2.putText(image, text, (x1, baseline), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, color, 2, cv2.LINE_AA)
            association = str(track.association_method or "")
            if association:
                cv2.putText(image, association, (x1, min(height - 5, y2 + 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        panel = "{} | shown={}/{} | paired={}".format(
            self._source, visible_tracks, len(tracks.tracks), self._paired)
        if hidden_lost:
            panel += " | lost_hidden={}".format(hidden_lost)
        if status is not None:
            panel += " | selected={} {}".format(status.track_id,
                                                   status.tracking_state)
        cv2.rectangle(image, (0, 0), (min(width, 620), 28), (20, 20, 20), -1)
        cv2.putText(image, panel, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (255, 255, 255), 1, cv2.LINE_AA)

    def _tracks_callback(self, message: TrackStateArray) -> None:
        if message.image_source != self._source:
            return
        with self._lock:
            self._latest_tracks = message
            status = self._status
        if self._publisher.get_num_connections() == 0:
            return
        image_message, status = self._matching_image(message.header.stamp)
        if image_message is None:
            self._dropped_no_image += 1
            rospy.logwarn_throttle(2.0,
                                   "tracking overlay source=%s lacks matching image (dropped=%d)",
                                   self._source, self._dropped_no_image)
            return
        self._publish_overlay(image_message, message, status)

    def _publish_overlay(self, image_message: Image,
                         tracks: TrackStateArray, status) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "tracking overlay image conversion failed: %s", error)
            return
        self._paired += 1
        self._draw(image, tracks, status)
        # Preserve capture timing and frame convention for downstream viewers/recorders.
        output = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = image_message.header
        self._publisher.publish(output)
        self._last_publish_monotonic = time.monotonic()


def main() -> None:
    rospy.init_node("tracking_overlay")
    try:
        TrackingOverlay()
    except ValueError as error:
        rospy.logfatal("invalid tracking overlay configuration: %s", error)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
