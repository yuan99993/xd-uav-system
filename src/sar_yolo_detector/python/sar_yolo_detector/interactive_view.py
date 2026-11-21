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
    def __init__(self, limit=32, image_limit=1, maximum_image_bytes=33554432):
        self.limit = max(1, int(limit))
        self.lock = threading.Lock()
        self.metadata = OrderedDict()
        self.image = None
        self.images=OrderedDict();self.image_limit=max(1,int(image_limit))
        self.maximum_image_bytes=int(maximum_image_bytes);self.delivered=None
        self.ready = None

    def add_image(self, image):
        with self.lock:
            self.image = image
            h=image.header;key=(h.stamp.secs,h.stamp.nsecs,h.frame_id)
            self.images[key]=image
            while len(self.images)>self.image_limit or sum(len(getattr(v,'data',b'')) for v in self.images.values())>self.maximum_image_bytes:
                self.images.popitem(last=False)
            self._match()

    def add_evidence(self, evidence):
        header = evidence['capture_header']
        stamp = header['stamp']
        key = (int(stamp['secs']), int(stamp['nsecs']), str(header['frame_id']))
        with self.lock:
            self.metadata[key] = evidence
            self.metadata.move_to_end(key)
            self._match()
            while len(self.metadata) > self.limit:
                self.metadata.popitem(last=False)

    def _match(self):
        matches=[k for k in self.images if k in self.metadata and (self.delivered is None or k[:2]>self.delivered)]
        if not matches:return
        key=max(matches,key=lambda k:k[:2])
        if self.ready is None or key[:2]>=(self.ready[0].header.stamp.secs,self.ready[0].header.stamp.nsecs):
            self.ready=(self.images[key],self.metadata[key])
        self.image=None

    def take(self):
        with self.lock:
            result,self.ready=self.ready,None
            if result is not None:
                h=result[0].header;self.delivered=(h.stamp.secs,h.stamp.nsecs)
                self.images=OrderedDict((k,v) for k,v in self.images.items() if k[:2]>self.delivered)
            return result
