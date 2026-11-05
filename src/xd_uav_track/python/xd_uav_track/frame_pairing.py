"""Bounded timestamp pairing; never put an old box on a newer camera frame."""
from collections import OrderedDict

class FramePairer:
    def __init__(self,wait=.3,capacity=8):
        self.wait=wait;self.capacity=capacity;self.frames=OrderedDict();self.results=OrderedDict();self.last=-1
    def _emit(self,stamp,tracks):
        image=self.frames[stamp][0];self.last=stamp
        for mapping in (self.frames,self.results):
            for key in list(mapping):
                if key<=stamp:del mapping[key]
        return stamp,image,tracks
    def image(self,stamp,image,now):
        if stamp<=self.last:return None
        self.frames[stamp]=(image,now)
        while len(self.frames)>self.capacity:self.frames.popitem(last=False)
        if stamp in self.results:return self._emit(stamp,self.results[stamp])
        return None
    def tracks(self,stamp,tracks):
        if stamp<=self.last:return None
        if stamp in self.frames:return self._emit(stamp,tracks)
        self.results[stamp]=tracks
        while len(self.results)>self.capacity:self.results.popitem(last=False)
        return None
    def expire(self,now):
        expired=[stamp for stamp,(_,received) in self.frames.items() if now-received>=self.wait]
        return self._emit(max(expired),None) if expired else None
