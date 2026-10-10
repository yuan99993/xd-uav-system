"""Bounded, frame-exact pairing and aspect-preserving UI coordinates."""
from collections import OrderedDict
import math
import threading


def image_point(x, y, view_width, view_height, width, height):
    """Map letterboxed widget coordinates to original pixels, rejecting margins."""
    if min(view_width, view_height, width, height) <= 0:
        return None
    if not all(math.isfinite(v) for v in (x, y)):
        return None
    scale = min(view_width / width, view_height / height)
    left, top = (view_width-width*scale)/2, (view_height-height*scale)/2
    px, py = (x-left)/scale, (y-top)/scale
    return (px, py) if 0 <= px < width and 0 <= py < height else None


class FramePair:
    """One pending image; bounded metadata. Never use publisher-rewritten seq."""
    def __init__(self, limit=32):
        self.limit = max(1, int(limit))
        self.lock = threading.Lock()
        self.metadata = OrderedDict()
        self.image = None

    def add_image(self, image):
        with self.lock:
            self.image = image

    def add_evidence(self, evidence):
        header = evidence['capture_header']
        stamp = header['stamp']
        key = (int(stamp['secs']), int(stamp['nsecs']), str(header['frame_id']))
        with self.lock:
            self.metadata[key] = evidence
            self.metadata.move_to_end(key)
            while len(self.metadata) > self.limit:
                self.metadata.popitem(last=False)

    def take(self):
        with self.lock:
            if self.image is None:
                return None
            header = self.image.header
            key = (header.stamp.secs, header.stamp.nsecs, header.frame_id)
            evidence = self.metadata.get(key)
            if evidence is None:
                return None
            image, self.image = self.image, None
            return image, evidence
