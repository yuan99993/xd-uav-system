"""Response peak diagnostics independent of a tracker implementation."""
import numpy as np
import math

def response_quality(response):
    values=np.asarray(response)
    if values.ndim!=2 or min(values.shape)<4:return dict(apce=0.,psr=0.)
    low=float(values.min());high=float(values.max())
    if not math.isfinite(low) or not math.isfinite(high):return dict(apce=0.,psr=0.)
    delta=(values-low).ravel()
    apce=(high-low)**2/max(1e-12,float(np.dot(delta,delta))/values.size)
    y,x=np.unravel_index(values.argmax(),values.shape)
    mask=np.ones(values.shape,bool)
    mask[max(0,y-1):y+2,max(0,x-1):x+2]=False
    side=values[mask].astype(np.float64,copy=False)
    mean=float(side.sum())/side.size
    variance=max(0.,float(np.dot(side,side))/side.size-mean*mean)
    psr=(high-mean)/max(1e-6,math.sqrt(variance))
    return dict(apce=float(apce),psr=float(psr))


class ResponseHysteresis:
    """Only simultaneous low APCE/PSR for three frames trips the guard."""
    def __init__(self,low_apce=1.8,low_psr=2.2,high_apce=2.5,high_psr=2.8):
        self.low_apce=float(low_apce);self.low_psr=float(low_psr)
        self.high_apce=float(high_apce);self.high_psr=float(high_psr)
        if not np.isfinite([low_apce,low_psr,high_apce,high_psr]).all():raise ValueError('Nonfinite response thresholds')
        self.reset()
    def reset(self):self.bad_frames=0;self.occluded=False
    @property
    def search_multiplier(self):return 1.5 if self.occluded else 1.
    def update(self,apce,psr):
        if not math.isfinite(apce) or not math.isfinite(psr):apce=psr=0.
        if self.occluded:
            if apce>self.high_apce and psr>self.high_psr:self.reset()
        else:
            self.bad_frames=self.bad_frames+1 if apce<self.low_apce and psr<self.low_psr else 0
            if self.bad_frames>=3:self.occluded=True
        return not self.occluded
