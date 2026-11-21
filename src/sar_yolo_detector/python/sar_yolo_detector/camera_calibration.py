"""Bounded immutable capture-stamped calibration, with explicit static fallback."""
from collections import OrderedDict
from dataclasses import dataclass
import threading
import numpy as np


@dataclass(frozen=True)
class Calibration:
    frame_id: str
    stamp_ns: int
    width: int
    height: int
    K: tuple


class CalibrationCache:
    def __init__(self,limit=256):
        self.limit=max(2,int(limit));self.values=OrderedDict();self.dynamic=set();self.lock=threading.RLock()
    @staticmethod
    def key(header):return str(header.frame_id),int(header.stamp.secs)*10**9+int(header.stamp.nsecs)
    def add(self,message):
        key=self.key(message.header)
        with self.lock:
            if key[1]:self.dynamic.add(key[0])
            try:
                k=np.asarray(message.K,dtype=float).reshape(3,3)
                valid=(message.width>0 and message.height>0 and np.isfinite(k).all()
                    and k[0,0]>0 and k[1,1]>0 and abs(k[2,2]-1.)<1e-8
                    and np.isfinite(message.D).all() and all(abs(v)<1e-8 for v in message.D))
                value=Calibration(key[0],key[1],int(message.width),int(message.height),tuple(k.ravel())) if valid else None
            except (ValueError,TypeError):value=None
            # An ambiguous duplicate timestamp is unusable, never silently replaced.
            if key in self.values and self.values[key]!=value:value=None
            self.values[key]=value;self.values.move_to_end(key)
            while len(self.values)>self.limit:self.values.popitem(last=False)
        return value is not None
    def get(self,header,width,height):
        key=self.key(header)
        with self.lock:
            value=self.values.get(key)
            if value is None and key[0] not in self.dynamic:value=self.values.get((key[0],0))
            return value if value is not None and (value.width,value.height)==(width,height) else None
