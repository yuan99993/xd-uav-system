"""Bounded, asynchronous PNG delivery of canonical perception overlays.

Each source owns one encoder. One image may be encoding and at most one newer
image may wait; submitting a frame replaces that pending image. The encoder
receives an owned BGR image and never modifies it, resizes it or performs
perception. Rate limits use monotonic wall time, independent of ROS /clock.
"""

from copy import deepcopy
import math
import threading
import time

import cv2


class LatestPerceptionVideoEncoder:
    def __init__(self, publisher, rate_hz=6.0, max_pixels=16777216,
                 max_encoded_bytes=16777216, frame_factory=None, log_drop=None):
        if frame_factory is None:
            from xd_uav_track.msg import PerceptionVideoFrame
            frame_factory = PerceptionVideoFrame
        rate_hz = float(rate_hz)
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("video_rate_hz must be finite and positive")
        self._publisher = publisher
        self._period = 1.0 / min(rate_hz, 60.0)
        self._max_pixels = max(1, int(max_pixels))
        self._max_bytes = max(1, int(max_encoded_bytes))
        self._frame_factory = frame_factory
        self._log_drop = log_drop or (lambda reason: None)
        self._condition = threading.Condition()
        self._pending = None
        self._revision = 0
        self._closed = False
        self._thread = threading.Thread(target=self._run,
                                        name="perception-png-encoder", daemon=True)
        self._thread.start()

    def submit(self, image, targets):
        """Transfer ownership of a drawn image; metadata is snapshotted here.

        The caller must not mutate the image after a successful submission.
        There is no image copy and no encoding if nobody subscribes. No frame
        identity is deduplicated: LOST/selection updates can share a capture.
        """
        if self._publisher.get_num_connections() == 0:
            return False
        if (getattr(image, "ndim", 0) != 3 or image.shape[2] != 3 or
                str(image.dtype) != "uint8"):
            self._log_drop("video expects an 8-bit BGR image")
            return False
        height, width = image.shape[:2]
        if width * height > self._max_pixels:
            self._log_drop("video exceeds video_max_pixels; original pixels retained")
            return False
        if width != targets.image_width or height != targets.image_height:
            self._log_drop("video image dimensions differ from target snapshot")
            return False
        with self._condition:
            if self._closed:
                return False
            self._revision += 1
            self._pending = (self._revision, image, deepcopy(targets))
            self._condition.notify()
        return True

    def close(self):
        """Stop accepting work, discard pending work and bound shutdown time."""
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify_all()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        next_encode = 0.0
        while True:
            with self._condition:
                while not self._closed:
                    if self._pending is None:
                        self._condition.wait()
                        continue
                    wait_sec = next_encode - time.monotonic()
                    if wait_sec > 0:
                        self._condition.wait(wait_sec)
                        continue
                    work = self._pending
                    self._pending = None
                    break
                else:
                    return
            # A disconnected browser must not keep consuming encoder CPU.
            if self._publisher.get_num_connections() == 0:
                continue
            next_encode = time.monotonic() + self._period
            revision, image, targets = work
            try:
                ok, encoded = cv2.imencode(
                    ".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                if not ok:
                    self._log_drop("PNG encoding failed")
                    continue
                if encoded.nbytes > self._max_bytes:
                    self._log_drop("PNG exceeds video_max_encoded_bytes; frame dropped")
                    continue
                message = self._frame_factory()
                message.targets = targets
                message.header = deepcopy(targets.header)
                message.header.seq = targets.frame_seq
                message.frame_revision = revision
                message.image.header = deepcopy(message.header)
                message.image.format = "bgr8; png compressed bgr8"
                message.image.data = encoded.tobytes()
                with self._condition:
                    if not self._closed and self._publisher.get_num_connections() > 0:
                        self._publisher.publish(message)
            except Exception as error:
                # Encoder/publisher failure must not kill the source callbacks.
                self._log_drop("video encoding/publishing failed: {}".format(error))
