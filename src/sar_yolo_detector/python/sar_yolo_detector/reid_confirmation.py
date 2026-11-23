"""Two distinct detector captures and pre-occlusion direction are required."""
import math
import numpy as np

class ReIDConfirmation:
    def __init__(self,maximum_gap=.35):
        self.maximum_gap=maximum_gap;self.pending=None;self.reason='empty'
    def reset(self):self.pending=None;self.reason='empty'
    def update(self,box,stamp,velocity,identity=None):
        box=np.asarray(box,dtype=float);velocity=np.asarray(velocity,dtype=float)
        if box.shape!=(4,) or velocity.shape!=(2,) or not np.isfinite(np.r_[box,velocity,stamp]).all():
            self.reset();return False
        if self.pending is None:
            self.pending=(box.copy(),stamp,identity);self.reason='first_capture';return False
        old,previous_stamp,old_identity=self.pending;dt=stamp-previous_stamp
        if dt<=0:self.reason='non_fresh_capture';return False
        self.pending=(box.copy(),stamp,identity)
        if old_identity is not None and identity is not None and old_identity!=identity:
            self.reason='candidate_identity_changed';return False
        if dt>self.maximum_gap:self.reason='capture_gap';return False
        displacement=(box[:2]+box[2:]/2)-(old[:2]+old[2:]/2)
        length=float(np.linalg.norm(displacement));speed=float(np.linalg.norm(velocity))
        if length<=1e-6 or speed<=1e-6:
            self.reason='direction_unavailable';return False
        if length>max(12.,3.*speed*dt+.5*math.hypot(*old[2:])):
            self.reason='candidate_jump';return False
        cosine=float(np.dot(displacement,velocity)/(length*speed))
        if cosine<=math.cos(math.pi/4):self.reason='direction_mismatch';return False
        self.reason='two_capture_direction_confirmed';return True
