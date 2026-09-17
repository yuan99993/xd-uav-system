#!/usr/bin/env python3
"""Attach optional appearance embeddings to detector observations.

No track id, gallery, lost-object memory, or matching decision is created here.
The node is deliberately a stateless image/ROI frontend for MultiTrackManager.
"""

from __future__ import annotations

from collections import deque
import threading
import time

import numpy as np
import rospy
from sensor_msgs.msg import Image

from xd_uav_track.msg import DetectionArray
from xd_uav_track.reid import AppearanceEncoder


class ReidEncoderNode:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._closing = False
        self._enabled = bool(rospy.get_param("~enabled", True))
        config = dict(rospy.get_param("~reid", {}))
        if rospy.has_param("~model_profile"):
            config["active_model_profile"] = rospy.get_param("~model_profile")
        synchronization = dict(config.get("synchronization", {}))
        runtime = dict(config.get("runtime", {}))
        self._max_sync_sec = max(0.0, float(rospy.get_param(
            "~maximum_image_delta_sec",
            synchronization.get("maximum_image_delta_sec", 0.050))))
        self._max_image_wait_sec = max(0.0, float(rospy.get_param(
            "~maximum_image_wait_sec",
            synchronization.get("maximum_image_wait_sec", 0.080))))
        self._maximum_output_age_sec = max(0.0, float(rospy.get_param(
            "~maximum_output_age_sec", runtime.get("maximum_output_age_sec", 0.30))))
        self._cache_size = max(2, int(rospy.get_param(
            "~image_cache_size", synchronization.get("image_cache_size", 4))))
        self._max_rois = max(1, int(rospy.get_param(
            "~maximum_rois_per_frame", runtime.get("maximum_rois_per_frame", 32))))
        self._preserve_existing = bool(rospy.get_param(
            "~preserve_existing_embedding", False))
        self._images = deque(maxlen=self._cache_size)
        self._pending = None
        self._received = 0
        self._published = 0
        self._dropped_backlog = 0
        self._sync_misses = 0
        self._stale_outputs = 0
        self._encoder = AppearanceEncoder(config) if self._enabled else None
        input_topic = rospy.get_param("~input_detections_topic", "detect/input/detections_raw")
        image_topic = rospy.get_param("~input_image_topic", "camera/image_raw")
        output_topic = rospy.get_param("~output_detections_topic", "detect/input/detections_2d")
        self._publisher = rospy.Publisher(output_topic, DetectionArray, queue_size=1)
        # In pass-through mode there is no reason to deserialize a second copy
        # of the high-bandwidth image stream.
        self._image_sub = rospy.Subscriber(
            image_topic, Image, self._image_callback, queue_size=1,
            buff_size=16 * 1024 * 1024, tcp_nodelay=True
        ) if self._enabled else None
        self._detections_sub = rospy.Subscriber(
            input_topic, DetectionArray, self._detections_callback,
            queue_size=1, tcp_nodelay=True)
        self._worker = threading.Thread(target=self._worker_loop, name="reid_encoder", daemon=True)
        self._worker.start()
        rospy.on_shutdown(self.close)
        rospy.loginfo("xd_uav_track ReID encoder: %s -> %s (%s)", input_topic, output_topic, "enabled" if self._enabled else "pass-through")

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify_all()
        worker = getattr(self, "_worker", None)
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2.0)
        with self._lock:
            if self._encoder is not None:
                self._encoder.close()
                self._encoder = None

    def _image_callback(self, message: Image) -> None:
        with self._condition:
            # Keep the serialized ROS message and decode only the frame that
            # actually matches a detection. This avoids converting and storing
            # every camera frame when YOLO runs at a lower rate.
            self._images.append((message.header.stamp, message))
            # Image and detector messages are transported independently. Wake
            # a detection worker which may be waiting briefly for this image.
            self._condition.notify_all()

    @staticmethod
    def _decode_image(message: Image) -> np.ndarray:
        """Decode common uncompressed ROS image encodings without cv_bridge.

        Keeping this small conversion local avoids an otherwise unnecessary
        Python ABI dependency on the OpenCV version used to build cv_bridge.
        The detector/ReID deployment contract intentionally accepts only
        uncompressed 8-bit images here; compressed transports must be decoded
        by image_transport before reaching this node.
        """
        encoding = message.encoding.lower()
        channels_by_encoding = {
            "mono8": 1,
            "8uc1": 1,
            "bgr8": 3,
            "rgb8": 3,
            "8uc3": 3,
            "bgra8": 4,
            "rgba8": 4,
            "8uc4": 4,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            raise ValueError("unsupported image encoding '%s'" % message.encoding)
        width = int(message.width)
        height = int(message.height)
        step = int(message.step)
        packed_step = width * channels
        if width <= 0 or height <= 0 or step < packed_step:
            raise ValueError("invalid image dimensions or row step")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        required = height * step
        if raw.size < required:
            raise ValueError("image data is shorter than height * step")
        rows = raw[:required].reshape(height, step)[:, :packed_step]
        pixels = rows.reshape(height, width, channels)
        if channels == 1:
            return np.ascontiguousarray(np.repeat(pixels, 3, axis=2))
        if encoding == "rgb8":
            pixels = pixels[:, :, ::-1]
        elif encoding == "rgba8":
            pixels = pixels[:, :, [2, 1, 0]]
        elif channels == 4:
            pixels = pixels[:, :, :3]
        return np.ascontiguousarray(pixels)

    def _closest_image_locked(self, stamp):
        if stamp.is_zero() or not self._images:
            return None, None
        candidate = min(
            self._images, key=lambda item: abs((item[0] - stamp).to_sec()))
        delta = abs((candidate[0] - stamp).to_sec())
        return (candidate[1], delta) if delta <= self._max_sync_sec else (None, delta)

    def _wait_for_image(self, stamp):
        deadline = time.monotonic() + self._max_image_wait_sec
        with self._condition:
            while not self._closing and not rospy.is_shutdown():
                image, delta = self._closest_image_locked(stamp)
                if image is not None:
                    return image, delta
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None, delta
                self._condition.wait(timeout=remaining)
        return None, None

    @staticmethod
    def _bbox(candidate):
        if not candidate.has_bbox:
            return None
        x1, y1, x2, y2 = [int(value) for value in candidate.bbox]
        return (x1, y1, x2, y2) if x2 > x1 and y2 > y1 else None

    def _detections_callback(self, message: DetectionArray) -> None:
        self._received += 1
        if not self._enabled or self._encoder is None:
            self._publisher.publish(message)
            self._published += 1
            return
        if not message.candidates:
            self._publisher.publish(message)
            self._published += 1
            return
        with self._condition:
            if self._pending is not None:
                self._dropped_backlog += 1
            self._pending = message
            self._condition.notify()

    def _worker_loop(self) -> None:
        while not rospy.is_shutdown():
            with self._condition:
                while self._pending is None and not self._closing and not rospy.is_shutdown():
                    self._condition.wait(timeout=0.25)
                if self._closing or rospy.is_shutdown():
                    return
                output = self._pending
                self._pending = None
            image_message, delta = self._wait_for_image(output.header.stamp)
            if image_message is None:
                self._sync_misses += 1
                for candidate in output.candidates:
                    if not self._preserve_existing:
                        candidate.appearance_embedding = []
                self._publisher.publish(output)
                self._published += 1
                rospy.logwarn_throttle(
                    2.0, "ReID pass-through: no synchronized image (nearest delta=%s)",
                    "unknown" if delta is None else "%.4fs" % delta)
                continue
            try:
                image = self._decode_image(image_message)
            except Exception as error:
                self._sync_misses += 1
                rospy.logwarn_throttle(
                    2.0, "ReID image conversion failed: %s", error)
                self._publisher.publish(output)
                self._published += 1
                continue
            stamp = output.header.stamp
            now = rospy.Time.now()
            if not stamp.is_zero() and not now.is_zero() and \
                    (now - stamp).to_sec() > self._maximum_output_age_sec:
                self._stale_outputs += 1
                rospy.logwarn_throttle(2.0, "ReID dropped stale detection frame")
                continue
            selected = output.candidates[:self._max_rois]
            observations = [(self._bbox(candidate), candidate.class_id)
                            for candidate in selected]
            features, qualities = self._encoder.encode_many_with_quality(
                image, observations)
            for candidate, feature, quality in zip(selected, features, qualities):
                candidate.appearance_embedding = [] if feature is None else \
                    feature.astype(np.float32, copy=False).tolist()
                candidate.appearance_quality = 0.0 if feature is None else \
                    max(0.001, float(quality))
            for candidate in output.candidates[self._max_rois:]:
                candidate.appearance_embedding = []
                candidate.appearance_quality = 0.0
            self._publisher.publish(output)
            self._published += 1
            rospy.loginfo_throttle(
                10.0,
                "ReID live stats received=%d published=%d backlog_drop=%d sync_miss=%d stale=%d",
                self._received, self._published, self._dropped_backlog,
                self._sync_misses, self._stale_outputs)


def main() -> None:
    rospy.init_node("reid_encoder")
    ReidEncoderNode()
    rospy.spin()


if __name__ == "__main__":
    main()
