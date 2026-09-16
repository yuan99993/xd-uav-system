"""Small shared ROS-image helpers used by detector-only adapters.

Keeping this utility outside SmartTracker prevents a detection-only deployment
from constructing any temporal tracker, ReID gallery, or selection state.
"""

from __future__ import annotations

import numpy as np
import cv2


def image_to_bgr(message):
    """Decode the common ROS image encodings into a contiguous BGR image."""
    height, width, step = int(message.height), int(message.width), int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    encoding = str(message.encoding or "").strip().lower()
    formats = {
        "bgr8": (3, None), "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR), "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR), "8uc1": (1, cv2.COLOR_GRAY2BGR),
    }
    if encoding in formats:
        channels, conversion = formats[encoding]
        minimum_step = width * channels
        if step < minimum_step or len(message.data) < height * step:
            raise ValueError("invalid image stride or data length")
        raw = np.frombuffer(message.data, dtype=np.uint8, count=height * step)
        image = raw.reshape(height, step)[:, :minimum_step]
        image = image.reshape(height, width, channels) if channels > 1 else image.reshape(height, width)
        return np.ascontiguousarray(cv2.cvtColor(image, conversion) if conversion is not None else image)
    if encoding in {"mono16", "16uc1", "y16"}:
        if step < width * 2 or step % 2 or len(message.data) < height * step:
            raise ValueError("invalid 16-bit image stride or data length")
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        raw = np.frombuffer(message.data, dtype=dtype, count=height * step // 2)
        gray = raw.reshape(height, step // 2)[:, :width].astype(np.float32)
        low, high = np.percentile(gray, (1.0, 99.0))
        if high <= low:
            gray8 = np.zeros((height, width), dtype=np.uint8)
        else:
            gray8 = np.clip((gray - low) * (255.0 / (high - low)), 0, 255).astype(np.uint8)
        return cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)
    raise ValueError("unsupported image encoding: %s" % encoding)
