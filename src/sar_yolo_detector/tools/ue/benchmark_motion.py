"""Deterministic visual-target trajectories; never flight-controller feedback."""
import math
import threading
import time


def entity_position(entity,elapsed):
    trajectory=entity.get('trajectory')
    if trajectory is None:
        return [p+v*20*math.sin(elapsed/20) for p,v in zip(entity['position'],entity['velocity'])]
    if trajectory.get('kind')=='step_offsets':
        position=list(entity['position']);previous=-1.
        for step in trajectory['steps']:
            at=float(step['at_sec']);offset=step['offset_m']
            if not math.isfinite(at) or at<0 or at<=previous or len(offset)!=3 or not all(math.isfinite(v) for v in offset):
                raise ValueError('Invalid evaluation event')
            previous=at
            if elapsed>=at:position=[p+v for p,v in zip(entity['position'],offset)]
        return position
    if trajectory.get('kind')!='sine':raise ValueError('Unsupported benchmark trajectory')
    amplitude=trajectory['amplitude_m'];period=float(trajectory['period_sec'])
    phase=float(trajectory.get('phase_rad',0.))
    if len(amplitude)!=3 or not all(math.isfinite(v) for v in amplitude) or not math.isfinite(period) or not math.isfinite(phase) or period<=0:
        raise ValueError('Invalid benchmark trajectory')
    value=math.sin(2*math.pi*max(0.,elapsed)/period+phase)
    return [p+a*value for p,a in zip(entity['position'],amplitude)]


class BenchmarkClock:
    def __init__(self,manual=False):
        self.manual=manual;self.epoch=None;self.frozen=0.;self.lock=threading.Lock()
    def command(self,action,now=None):
        now=time.monotonic() if now is None else now
        with self.lock:
            if action=='start':
                if self.epoch is None:self.epoch=now-self.frozen
            elif action=='reset':self.epoch=None;self.frozen=0.
            elif action=='hold':
                if self.epoch is not None:self.frozen=max(0.,now-self.epoch)
                self.epoch=None
            else:raise ValueError('Benchmark command must be start, hold or reset')
    def sample(self,elapsed,now=None):
        if not self.manual:return elapsed
        now=time.monotonic() if now is None else now
        with self.lock:return self.frozen if self.epoch is None else max(0.,now-self.epoch)
