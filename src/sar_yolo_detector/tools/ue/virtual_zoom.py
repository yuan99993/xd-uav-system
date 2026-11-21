"""Explicit virtual lens tests. Default fixed lens; no target-driven auto zoom."""
import json
import math
from pathlib import Path
from virtual_gimbal import Axis


class FovProfile:
    def __init__(self,name='fixed'):
        if name=='fixed':data=dict(schedule=[[0.,90.]])
        elif name=='zoom_test':data=dict(schedule=[[0.,90.],[8.,60.],[20.,90.]])
        else:data=json.loads(Path(name).read_text())
        self.config=dict(minimum_deg=45.,maximum_deg=100.,initial_deg=90.,
            max_rate_deg_s=8.,max_accel_deg_s2=16.)
        self.config.update(data.get('actuator',{}))
        VirtualZoom(self.config)
        self.schedule=[tuple(map(float,item)) for item in data['schedule']]
        if not self.schedule or self.schedule[0][0]!=0:raise ValueError('FOV schedule must start at zero')
        previous=-1.
        for stamp,angle in self.schedule:
            if not all(math.isfinite(v) for v in (stamp,angle)) or stamp<=previous:
                raise ValueError('FOV schedule times must be finite and strictly increasing')
            if not self.config['minimum_deg']<=angle<=self.config['maximum_deg']:
                raise ValueError('FOV target outside virtual lens limits')
            previous=stamp
    def target(self,elapsed):
        return next(angle for stamp,angle in reversed(self.schedule) if stamp<=max(0.,elapsed))


class VirtualZoom:
    def __init__(self,config=None):
        c=dict(config or {})
        minimum=float(c.get('minimum_deg',45.));maximum=float(c.get('maximum_deg',100.))
        if not 45.<=minimum<maximum<=100.:raise ValueError('Virtual lens must stay within 45–100 degrees')
        rate=float(c.get('max_rate_deg_s',8.));accel=float(c.get('max_accel_deg_s2',16.))
        if not 0<rate<=8. or not 0<accel<=16.:raise ValueError('Virtual lens slew exceeds validated limits')
        self.axis=Axis(float(c.get('initial_deg',90.)),[minimum,maximum],rate,accel)
        self.last_time=None
    def step(self,now,target):
        if not all(math.isfinite(v) for v in (now,target)) or not self.axis.minimum<=target<=self.axis.maximum:
            raise ValueError('Invalid virtual lens command')
        if self.last_time is not None and now<self.last_time:raise ValueError('Virtual lens clock reversed')
        remaining=min(.25,now-self.last_time) if self.last_time is not None else 0.
        while remaining>1e-9:
            dt=min(.005,remaining);self.axis.step(target,dt);remaining-=dt
        self.last_time=now
        return self.axis.angle
