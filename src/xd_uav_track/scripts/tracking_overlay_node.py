#!/usr/bin/env python3
"""Low-load, latest-only visualization of source-local tracker output."""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from xd_uav_track.msg import TrackStateArray, TrackStatus
from xd_uav_track.pixel_lock import (blend_boxes, box_iou, coast_box, color_box,
                                      compatible_box, flow_box,
                                      turn_compatible_box)


class TrackingOverlay:
    def __init__(self) -> None:
        self._source = str(rospy.get_param("~image_source", "")).strip()
        if not self._source:
            raise ValueError("~image_source must not be empty")
        self._image_topic = str(rospy.get_param("~input_image_topic", "image_raw"))
        self._tracks_topic = str(rospy.get_param("~tracks_topic", "track/tracks"))
        self._status_topic = str(rospy.get_param("~status_topic", "track/status"))
        self._output_topic = str(rospy.get_param("~output_image_topic", "tracking_overlay"))
        self._maximum_pair_delta_sec = max(0.05, min(1.0, float(
            rospy.get_param("~maximum_pair_delta_sec", 0.40))))
        self._display_rate_hz = max(0.5, min(15.0, float(
            rospy.get_param("~display_rate_hz", 3.0))))
        self._show_lost = bool(rospy.get_param("~show_lost", False))
        self._show_confirmed_only = bool(rospy.get_param(
            "~show_confirmed_only", False))
        self._show_selected_lost = bool(rospy.get_param(
            "~show_selected_lost", False))
        self._selected_lost_max_frames = max(0, int(rospy.get_param(
            "~selected_lost_max_frames", 3)))

        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._latest_tracks = None
        self._status = None
        self._last_publish_monotonic = 0.0
        self._stale_frame_count = 0
        self._pixel_locks = {}
        self._pixel_predicted_ids = set()
        self._pixel_lock_max_tracks = max(1, min(16, int(rospy.get_param(
            "~pixel_lock_max_tracks", 8))))
        self._pixel_lock_max_gap_sec = max(0.2, min(2.0, float(
            rospy.get_param("~pixel_lock_max_gap_sec", 0.50))))
        self._pixel_lock_max_unanchored_sec = max(0.2, min(3.0, float(
            rospy.get_param("~pixel_lock_max_unanchored_sec", 2.00))))

        self._publisher = rospy.Publisher(self._output_topic, Image, queue_size=1)
        # Queue size one deliberately drops superseded frames instead of making
        # the display render an increasingly old backlog under CPU contention.
        self._image_sub = rospy.Subscriber(
            self._image_topic, Image, self._image_callback, queue_size=1,
            buff_size=16 * 1024 * 1024, tcp_nodelay=True)
        self._tracks_sub = rospy.Subscriber(
            self._tracks_topic, TrackStateArray, self._tracks_callback,
            queue_size=1, tcp_nodelay=True)
        self._status_sub = rospy.Subscriber(
            self._status_topic, TrackStatus, self._status_callback,
            queue_size=1, tcp_nodelay=True)
        rospy.loginfo(
            "latest-only tracking overlay ready: source=%s image=%s tracks=%s "
            "output=%s display_hz=%.1f max_pair_delta=%.3fs",
            self._source, self._image_topic, self._tracks_topic,
            self._output_topic, self._display_rate_hz,
            self._maximum_pair_delta_sec)

    def _tracks_callback(self, message: TrackStateArray) -> None:
        if message.image_source != self._source:
            return
        with self._lock:
            previous = self._latest_tracks
            # Do not let a delayed ROS delivery roll the display state backward.
            if (previous is not None and not message.header.stamp.is_zero() and
                    not previous.header.stamp.is_zero() and
                    message.header.stamp <= previous.header.stamp):
                return
            self._latest_tracks = message

    def _status_callback(self, message: TrackStatus) -> None:
        with self._lock:
            previous = self._status
            if (previous is not None and not message.header.stamp.is_zero() and
                    not previous.header.stamp.is_zero() and
                    message.header.stamp < previous.header.stamp):
                return
            self._status = message

    @staticmethod
    def _color(track, rgb_image=False):
        if track.selected:
            color = (0, 255, 0)
        elif track.predicted or not track.detected:
            color = (0, 190, 255)
        elif track.lifecycle_state == "tentative":
            color = (180, 180, 180)
        else:
            color = (255, 210, 0)
        return color[::-1] if rgb_image else color

    @staticmethod
    def _label(track) -> str:
        kind = "PRED" if track.predicted or not track.detected else "DET"
        ready = " READY" if track.control_measurement_ready else ""
        # Tentative tracks have only a provisional YOLO label. Show identity
        # and tracking state first; publish the latched class only once the
        # track manager confirms the entity.
        class_label = (" c{}".format(track.class_id)
                       if track.lifecycle_state in ("confirmed", "occluded", "lost")
                       and track.class_id >= 0 else "")
        return "ID {}{} {:.2f} {}{}".format(
            track.track_id, class_label, track.confidence, kind, ready)

    def _draw(self, image, tracks, status, age_sec, rgb_image=False,
              pixel_boxes=None) -> None:
        pixel_boxes = pixel_boxes or {}
        height, width = image.shape[:2]
        visible_tracks = 0
        hidden_lost = 0
        hidden_unconfirmed = 0
        track_count = len(tracks.tracks) if tracks is not None else 0
        if tracks is not None:
            for track in tracks.tracks:
                if age_sec is None and track.track_id not in pixel_boxes:
                    continue
                flowed = track.track_id in pixel_boxes
                selected_lost = (
                    track.lifecycle_state == "lost" and track.selected and
                    (flowed or (self._show_selected_lost and
                     track.frames_since_detection <= self._selected_lost_max_frames)))
                if (track.lifecycle_state == "lost" and not self._show_lost and
                        not selected_lost):
                    hidden_lost += 1
                    continue
                if self._show_confirmed_only:
                    if track.lifecycle_state == "tentative":
                        hidden_unconfirmed += 1
                        continue
                    # A confirmed identity remains useful during the tracker's
                    # short occlusion/coast window. Draw that estimate as PRED
                    # instead of making the box vanish on the first missed
                    # detector frame. Do not expose tentative or unresolved
                    # identity hypotheses as if they were confirmed targets.
                    confirmed_state = track.lifecycle_state in (
                        "confirmed", "occluded")
                    unresolved_identity = track.association_method in (
                        "stitch_pending", "stitch_ambiguous", "ambiguous",
                        "reacquiring")
                    if ((not confirmed_state and not selected_lost and not flowed) or
                            (unresolved_identity and not flowed)):
                        hidden_unconfirmed += 1
                        continue
                x1, y1, x2, y2 = (pixel_boxes[track.track_id] if flowed else
                                  [int(value) for value in track.bbox])
                x1 = max(0, min(width - 1, x1))
                x2 = max(0, min(width - 1, x2))
                y1 = max(0, min(height - 1, y1))
                y2 = max(0, min(height - 1, y2))
                if x2 <= x1 or y2 <= y1:
                    continue
                visible_tracks += 1
                predicted_display = track.track_id in self._pixel_predicted_ids
                color = ((0, 190, 255)[::-1] if rgb_image else (0, 190, 255)) \
                    if predicted_display else self._color(track, rgb_image)
                thickness = 3 if track.selected else 2
                cv2.rectangle(image, (x1, y1), (x2, y2), color,
                              thickness, cv2.LINE_AA)
                label = self._label(track)
                if flowed:
                    marker = " PRED" if predicted_display else " PIX"
                    label = label.replace(" DET", marker).replace(" PRED", marker)
                cv2.putText(image, label, (x1, max(18, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2,
                            cv2.LINE_AA)
                association = str(track.association_method or "")
                if association:
                    cv2.putText(image, association,
                                (x1, min(height - 5, y2 + 16)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1,
                                cv2.LINE_AA)

        if tracks is None:
            panel = "{} | no tracks".format(self._source)
        elif age_sec is None:
            panel = "{} | no fresh tracks".format(self._source)
        else:
            panel = "{} | shown={}/{} | age={:.2f}s".format(
                self._source, visible_tracks, track_count, age_sec)
        if hidden_lost:
            panel += " | lost_hidden={}".format(hidden_lost)
        if hidden_unconfirmed:
            panel += " | unconfirmed_hidden={}".format(hidden_unconfirmed)
        if status is not None:
            panel += " | selected={} {}".format(status.track_id,
                                                 status.tracking_state)
        cv2.rectangle(image, (0, 0), (min(width, 700), 28), (20, 20, 20), -1)
        cv2.putText(image, panel, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (255, 255, 255), 1, cv2.LINE_AA)

    def _advance_pixel_locks(self, gray, stamp_sec, tracks, age_sec,
                             color_frame=None, rgb_image=False):
        """Follow a bounded set of confirmed IDs between detector updates."""
        if tracks is None or stamp_sec <= 0.0:
            self._pixel_locks = {}
            self._pixel_predicted_ids = set()
            return {}
        ranked = sorted(tracks.tracks,
                        key=lambda item: (not item.selected,
                                          -float(item.tracking_quality)))
        track_stamp = tracks.header.stamp.to_sec()
        lag_sec = stamp_sec - track_stamp
        next_states = {}
        boxes = {}
        predicted_ids = set()
        for track in ranked:
            if len(next_states) >= self._pixel_lock_max_tracks:
                break
            state = self._pixel_locks.get(track.track_id)
            recognized = track.lifecycle_state in (
                "tentative", "confirmed", "occluded", "lost")
            if (not recognized and state is None) or track.frames_since_detection > 6 \
                    and not track.selected:
                continue
            observed = tuple(int(value) for value in track.bbox)
            if observed[2] <= observed[0] or observed[3] <= observed[1]:
                continue
            competitor_near_observation = any(
                other.track_id != track.track_id and
                box_iou(observed, other.bbox) >= 0.15
                for other in ranked)
            moved = None
            if state is not None:
                dt_sec = stamp_sec - state["stamp"]
                if 0.0 < dt_sec <= self._pixel_lock_max_gap_sec:
                    moved = flow_box(state["gray"], gray, state["box"], dt_sec,
                                     image_velocity=track.image_velocity)
                    if moved is None and color_frame is not None and \
                            state.get("frame") is not None:
                        moved = color_box(state["frame"], color_frame,
                                          state["box"], dt_sec, rgb_image)
            predicted = False
            if moved is None:
                box = None
                if state is not None and recognized and \
                        stamp_sec - state["anchor_stamp"] <= 0.80:
                    box = coast_box(state["box"], track.image_velocity,
                                    stamp_sec - state["stamp"], gray.shape)
                    if box is not None:
                        predicted = True
                        anchor_stamp = state["anchor_stamp"]
                        if (track_stamp > anchor_stamp and track.detected and
                                age_sec is not None and
                                -0.01 <= lag_sec <= 0.40 and
                                (compatible_box(box, observed) or
                                 (not competitor_near_observation and
                                  turn_compatible_box(box, observed)))):
                            if lag_sec <= 0.22:
                                box = blend_boxes(box, observed,
                                                  detector_weight=0.25)
                            anchor_stamp = track_stamp
                            predicted = False
                if box is None:
                    if (not recognized or not track.detected or
                            not -0.01 <= lag_sec <= 0.75):
                        continue
                    box = observed
                    predicted = lag_sec > 0.40
                    if predicted:
                        box = coast_box(observed, track.image_velocity,
                                        min(0.5, lag_sec), gray.shape)
                        if box is None:
                            continue
                    anchor_stamp = track_stamp
            else:
                box, anchor_stamp = moved, state["anchor_stamp"]
                if (track_stamp > anchor_stamp and age_sec is not None and
                        -0.01 <= lag_sec <= 0.40 and track.detected and
                        (compatible_box(moved, observed) or
                         (not competitor_near_observation and
                          turn_compatible_box(moved, observed)))):
                    if lag_sec <= 0.22:
                        box = blend_boxes(moved, observed, detector_weight=0.25)
                    anchor_stamp = track_stamp
                if stamp_sec - anchor_stamp > self._pixel_lock_max_unanchored_sec:
                    continue
            # Two nearby flow hypotheses must never display as two claims on
            # the same pixels. Let the tracker resolve that ambiguity.
            if any(box_iou(box, other) > 0.60 for other in boxes.values()):
                continue
            boxes[track.track_id] = box
            if predicted:
                predicted_ids.add(track.track_id)
            next_states[track.track_id] = {
                "gray": gray, "stamp": stamp_sec, "box": box,
                "anchor_stamp": anchor_stamp, "frame": color_frame}
        self._pixel_locks = next_states
        self._pixel_predicted_ids = predicted_ids
        return boxes

    def _image_callback(self, message: Image) -> None:
        if self._publisher.get_num_connections() == 0:
            return
        now = time.monotonic()
        with self._lock:
            period = 1.0 / self._display_rate_hz
            if now - self._last_publish_monotonic < period:
                return
            self._last_publish_monotonic = now
            tracks = self._latest_tracks
            status = self._status

        fresh_tracks = None
        fresh_status = None
        age_sec = None
        if (tracks is not None and not message.header.stamp.is_zero() and
                not tracks.header.stamp.is_zero()):
            age = abs((message.header.stamp - tracks.header.stamp).to_sec())
            if age <= self._maximum_pair_delta_sec:
                fresh_tracks = tracks
                age_sec = age
                if (status is not None and not status.header.stamp.is_zero() and
                        abs((message.header.stamp - status.header.stamp).to_sec()) <=
                        self._maximum_pair_delta_sec):
                    fresh_status = status

        output = None
        rgb_image = False
        if (message.encoding in ("rgb8", "bgr8", "8UC3") and
                message.step >= message.width * 3 and
                len(message.data) >= message.step * message.height):
            # Copy once into the outgoing ROS message and draw directly in its
            # buffer. This avoids a full RGB<->BGR conversion on both display
            # nodes while retaining the camera's original encoding.
            output = Image()
            output.header = message.header
            output.height = message.height
            output.width = message.width
            output.encoding = message.encoding
            output.is_bigendian = message.is_bigendian
            output.step = message.step
            output.data = bytearray(message.data)
            frame = np.ndarray(
                (message.height, message.width, 3), dtype=np.uint8,
                buffer=output.data, strides=(message.step, 3, 1))
            rgb_image = message.encoding == "rgb8"
        else:
            try:
                frame = self._bridge.imgmsg_to_cv2(
                    message, desired_encoding="bgr8")
            except CvBridgeError as error:
                rospy.logwarn_throttle(
                    2.0, "tracking overlay image conversion failed: %s", error)
                return

        if fresh_tracks is None:
            with self._lock:
                self._stale_frame_count += 1
                stale_count = self._stale_frame_count
            if tracks is not None:
                rospy.logwarn_throttle(
                    10.0,
                    "tracking overlay source=%s hides stale tracks (frames=%d)",
                    self._source, stale_count)
        current_stamp = message.header.stamp.to_sec()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY if rgb_image
                            else cv2.COLOR_BGR2GRAY)
        pixel_boxes = self._advance_pixel_locks(
            gray, current_stamp, tracks, age_sec,
            color_frame=frame.copy(), rgb_image=rgb_image)
        display_tracks = fresh_tracks if fresh_tracks is not None else (
            tracks if pixel_boxes else None)
        self._draw(frame, display_tracks, fresh_status, age_sec, rgb_image,
                   pixel_boxes)
        if output is None:
            output = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            output.header = message.header
        try:
            self._publisher.publish(output)
        except rospy.ROSException as error:
            if not rospy.is_shutdown():
                rospy.logwarn_throttle(2.0,
                                       "tracking overlay publish failed: %s",
                                       error)


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
