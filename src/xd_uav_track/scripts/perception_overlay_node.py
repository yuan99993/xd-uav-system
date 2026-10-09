#!/usr/bin/env python3
"""Draw the canonical PerceptionTargetArray on its exact source image."""

from collections import deque
from copy import deepcopy
import threading

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from xd_uav_track.msg import PerceptionTargetArray
from xd_uav_track.perception_evidence import target_status
from xd_uav_track.perception_video import LatestPerceptionVideoEncoder


def dashed_box(image, x1, y1, x2, y2, color):
    for start in range(x1, x2, 12):
        end = min(start + 6, x2 - 1)
        cv2.line(image, (start, y1), (end, y1), color, 1, cv2.LINE_AA)
        cv2.line(image, (start, y2 - 1), (end, y2 - 1), color, 1, cv2.LINE_AA)
    for start in range(y1, y2, 12):
        end = min(start + 6, y2 - 1)
        cv2.line(image, (x1, start), (x1, end), color, 1, cv2.LINE_AA)
        cv2.line(image, (x2 - 1, start), (x2 - 1, end), color, 1, cv2.LINE_AA)


class PerceptionOverlay:
    def __init__(self):
        self._source = str(rospy.get_param("~image_source", "")).strip()
        if not self._source:
            raise ValueError("~image_source is required")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._images = deque(maxlen=max(2, min(30, int(rospy.get_param(
            "~image_cache_size", 12)))))
        self._pending = None
        self._video_encoder = None
        self._publisher = rospy.Publisher(
            str(rospy.get_param("~output_image_topic", "tracking_overlay")),
            Image, queue_size=1)
        if bool(rospy.get_param("~enable_web_video", True)):
            from xd_uav_track.msg import PerceptionVideoFrame
            video_publisher = rospy.Publisher(
                str(rospy.get_param("~web_video_topic", "perception/video")),
                PerceptionVideoFrame, queue_size=1, latch=False)
            self._video_encoder = LatestPerceptionVideoEncoder(
                video_publisher,
                rate_hz=rospy.get_param("~video_rate_hz", 6.0),
                max_pixels=rospy.get_param("~video_max_pixels", 16777216),
                max_encoded_bytes=rospy.get_param("~video_max_encoded_bytes", 16777216),
                log_drop=lambda reason: rospy.logwarn_throttle(
                    2.0, "perception video %s: %s", self._source, reason))
            rospy.on_shutdown(self._video_encoder.close)
        self._image_subscriber = rospy.Subscriber(
            str(rospy.get_param("~input_image_topic", "image_raw")), Image,
            self._image_callback, queue_size=1,
            buff_size=16 * 1024 * 1024, tcp_nodelay=True)
        self._target_subscriber = rospy.Subscriber(
            str(rospy.get_param("~targets_topic", "perception/targets")),
            PerceptionTargetArray, self._target_callback, queue_size=10,
            tcp_nodelay=True)

    def _image_callback(self, message):
        with self._lock:
            self._images.append(message)
            pending = self._pending
        if pending is not None:
            self._try_publish(pending)

    def _target_callback(self, message):
        if message.image_source != self._source:
            return
        with self._lock:
            self._pending = message
        self._try_publish(message)

    def _matching_image(self, targets):
        with self._lock:
            images = tuple(self._images)
        if targets.header.stamp.is_zero():
            return None
        same_capture = [image for image in reversed(images)
                        if image.header.stamp == targets.header.stamp]
        exact = next((image for image in same_capture
                      if image.header.seq == targets.frame_seq), None)
        if exact is not None:
            return exact
        # A zero seq can mean that a driver/manager has no sequence metadata.
        # Timestamp-only fallback is then valid, but must still be exact: even
        # a sub-millisecond neighbour is a different image and cannot be drawn
        # as the capture identified by the target snapshot.
        return next((image for image in same_capture
                     if not targets.frame_seq or not image.header.seq), None)

    def _try_publish(self, targets):
        image_message = self._matching_image(targets)
        if image_message is None:
            return
        if (targets.image_width and targets.image_width != image_message.width) or \
                (targets.image_height and targets.image_height != image_message.height):
            rospy.logwarn_throttle(
                2.0, "perception overlay rejected image-size mismatch for %s",
                self._source)
            return
        try:
            image = self._bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8").copy()
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "perception overlay decode failed: %s", error)
            return
        height, width = image.shape[:2]
        lost_ids = [str(t.track_id) for t in targets.targets if target_status(t) == 'LOST']
        # Draw the status background first. Otherwise it erases valid boxes
        # when a turning target passes through the top 28/50 pixels.
        panel = "{} | {}".format(self._source, targets.mode)
        cv2.rectangle(image, (0, 0), (width, 50 if lost_ids else 28), (20, 20, 20), -1)
        cv2.putText(image, panel, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (255, 255, 255), 1, cv2.LINE_AA)
        if lost_ids:
            cv2.putText(image, "LOST ID " + ",".join(lost_ids[:4]), (8, 43),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 190, 255), 1, cv2.LINE_AA)
        for target in targets.targets:
            status = target_status(target)
            if status == "LOST":
                continue
            x1, y1, x2, y2 = [int(value) for value in target.bbox_xyxy]
            x1, y1 = max(0, min(width - 1, x1)), max(0, min(height - 1, y1))
            x2, y2 = max(0, min(width, x2)), max(0, min(height, y2))
            if x2 <= x1 or y2 <= y1:
                continue
            color = ((0, 190, 255) if status == "PRED" else
                     ((0, 255, 0) if target.selected else (255, 210, 0)))
            if status == "PRED":
                dashed_box(image, x1, y1, x2, y2, color)
            else:
                cv2.rectangle(image, (x1, y1), (x2 - 1, y2 - 1), color,
                              3 if target.selected else 2, cv2.LINE_AA)
            center = (max(0, min(width - 1, int(round(target.center_px[0])))),
                      max(0, min(height - 1, int(round(target.center_px[1])))))
            cv2.circle(image, center, 3, (255, 255, 255), 1 if status == "PRED" else -1, cv2.LINE_AA)
            label = ("{} ID {}".format(target.class_name, target.track_id)
                     if target.class_valid else "ID {}".format(target.track_id))
            cv2.putText(image, label, (x1, max(18, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2,
                        cv2.LINE_AA)
        output = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        # Never mutate a cached camera header. Capture identity belongs to the
        # target snapshot, even when a driver omitted/mismatched its seq field.
        output.header = deepcopy(targets.header)
        output.header.seq = targets.frame_seq
        with self._lock:
            # A newer target update may have arrived while drawing. Prevent an
            # older DET snapshot from overwriting its same-capture LOST update.
            if self._pending is not targets:
                return
            self._pending = None
            self._publisher.publish(output)
            video_encoder = getattr(self, "_video_encoder", None)
            if video_encoder is not None:
                video_encoder.submit(image, targets)


def main():
    rospy.init_node("perception_overlay")
    PerceptionOverlay()
    rospy.spin()


if __name__ == "__main__":
    main()
