"""Deterministic two-axis virtual actuator, in degrees and monotonic seconds.

Positive yaw turns right in NED; negative UE pitch looks down. Targets are
world sight lines; body yaw is subtracted before applying physical pan limits.
No object position or simulator truth is consumed by this module.
"""
import math
from collections import deque


def angle_delta(a, b):
    return (a-b+180.) % 360.-180.


class Axis:
    def __init__(self, angle, limits, rate, acceleration):
        self.minimum, self.maximum = map(float, limits)
        self.rate, self.acceleration = float(rate), float(acceleration)
        if (not all(math.isfinite(v) for v in (angle,self.minimum,self.maximum,self.rate,self.acceleration))
                or not self.minimum <= angle <= self.maximum or min(self.rate, self.acceleration) <= 0):
            raise ValueError('Invalid gimbal axis parameters')
        self.angle, self.velocity = float(angle), 0.

    def step(self, target, dt):
        stopping = target is None
        target = self.angle if stopping else max(self.minimum, min(self.maximum, target))
        error = target-self.angle
        # Brake before the requested angle; preserve finite acceleration even
        # after reversals. Small bounded integration steps avoid stall jumps.
        wanted = math.copysign(min(self.rate, math.sqrt(2.*self.acceleration*abs(error))), error) if error else 0.
        old = self.velocity
        change = max(-self.acceleration*dt, min(self.acceleration*dt, wanted-old))
        self.velocity = max(-self.rate, min(self.rate, old+change))
        position = self.angle+(old+self.velocity)*.5*dt
        if position <= self.minimum or position >= self.maximum:
            position = max(self.minimum, min(self.maximum, position))
            self.velocity = 0.
        self.angle = position


class VirtualGimbal:
    def __init__(self, config=None):
        c = dict(config or {})
        self.yaw = Axis(float(c.get('home_yaw_deg', 0.)), c.get('yaw_limits_deg', [-160.,160.]),
                        c.get('max_rate_deg_s',30.), c.get('max_accel_deg_s2',90.))
        self.pitch = Axis(float(c.get('home_pitch_deg',-70.)), c.get('pitch_limits_deg',[-89.,45.]),
                          c.get('max_rate_deg_s',30.), c.get('max_accel_deg_s2',90.))
        self.delay = float(c.get('command_delay_sec',.08))
        self.timeout = float(c.get('measurement_timeout_sec',1.2))
        if not 0 <= self.delay <= 2 or not .05 <= self.timeout <= 5:
            raise ValueError('Invalid gimbal delay/measurement timeout')
        self.pending = deque(maxlen=256)
        self.last_time = None
        self.last_sequence = -1
        self.active = None
        self.mode = 'HOLD'
        self.sequence = -1
        self.saturated = False
        self.last_body_yaw = 0.

    def command(self, sequence, now, yaw_world=None, pitch_world=None, measurement_age=0.):
        values = [now, measurement_age] + ([] if yaw_world is None else [yaw_world,pitch_world])
        if not all(math.isfinite(v) for v in values) or sequence <= self.last_sequence:
            return False
        if yaw_world is not None and not 0 <= measurement_age <= self.timeout:
            return False
        self.last_sequence = sequence
        command = None if yaw_world is None else (float(yaw_world),float(pitch_world),now-measurement_age)
        self.pending.append((now+self.delay, sequence, command))
        return True

    def step(self, now, body_yaw=0.):
        if not all(math.isfinite(v) for v in (now,body_yaw)):
            raise ValueError('Nonfinite actuator time/body yaw')
        if self.last_time is None:
            self.last_time = now
        if now < self.last_time:
            raise ValueError('Actuator clock moved backwards')
        # Integrate at <=5 ms. A stalled editor does not teleport through a long
        # gap: at most 250 ms of motion is executed after a scheduling stall.
        remaining = min(now-self.last_time,.25)
        cursor = now-remaining
        while remaining > 1e-9:
            dt = min(.005, remaining)
            while self.pending and self.pending[0][0] <= cursor+1e-9:
                _,self.sequence,self.active = self.pending.popleft()
            fresh = self.active is not None and cursor-self.active[2] <= self.timeout
            if fresh:
                pan = angle_delta(self.active[0],body_yaw)
                tilt = self.active[1]
                self.mode = 'TRACKING'
            else:
                # Stopping follows the same acceleration bound as tracking.
                pan = self.yaw.angle+self.yaw.velocity*abs(self.yaw.velocity)/(2*self.yaw.acceleration)
                tilt = self.pitch.angle+self.pitch.velocity*abs(self.pitch.velocity)/(2*self.pitch.acceleration)
                self.mode = 'LOST' if self.active else 'HOLD'
            self.saturated = not (self.yaw.minimum <= pan <= self.yaw.maximum and
                                  self.pitch.minimum <= tilt <= self.pitch.maximum)
            self.yaw.step(pan if fresh else None,dt);self.pitch.step(tilt if fresh else None,dt)
            cursor += dt;remaining -= dt
        self.last_time,self.last_body_yaw = now,body_yaw
        return self.feedback()

    def feedback(self):
        return dict(mode=self.mode, applied_sequence=self.sequence,
                    yaw_deg=self.yaw.angle, pitch_deg=self.pitch.angle,
                    yaw_world_deg=self.last_body_yaw+self.yaw.angle,
                    yaw_rate_deg_s=self.yaw.velocity,pitch_rate_deg_s=self.pitch.velocity,
                    saturated=self.saturated,
                    limits=dict(yaw_deg=[self.yaw.minimum,self.yaw.maximum],
                                pitch_deg=[self.pitch.minimum,self.pitch.maximum],
                                max_rate_deg_s=self.yaw.rate,max_accel_deg_s2=self.yaw.acceleration),
                    command_delay_sec=self.delay)
