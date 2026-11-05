"""Optional CPU ROI background verifier; no implicit download or activation.

Scores are linear discriminant margins, not calibrated probabilities. An
invalid/tiny crop supplies no veto evidence and must never remove a target.
"""
import hashlib
from pathlib import Path

import cv2
import numpy as np

def _hog(gray):
    # Same small orientation/block descriptor across the ROS and CUDA OpenCV
    # builds; the CUDA environment's minimal OpenCV omits HOGDescriptor.
    gy,gx=np.gradient(gray.astype(np.float32))
    magnitude=np.hypot(gx,gy)
    direction=(np.arctan2(gy,gx)%np.pi)*(9./np.pi)
    lower=np.floor(direction).astype(int)%9
    fraction=direction-np.floor(direction)
    y,x=np.indices(gray.shape)
    hist=np.zeros((4,4,9),dtype=np.float32)
    np.add.at(hist,(y//8,x//8,lower),magnitude*(1.-fraction))
    np.add.at(hist,(y//8,x//8,(lower+1)%9),magnitude*fraction)
    blocks=[]
    for row in range(3):
        for col in range(3):
            block=hist[row:row+2,col:col+2].ravel()
            block=block/np.sqrt(float(block@block)+1e-6)
            block=np.minimum(block,.2)
            blocks.append(block/np.sqrt(float(block@block)+1e-6))
    return np.concatenate(blocks)


def features(frame, box):
    values=np.asarray(box,dtype=float)
    if values.shape!=(4,) or not np.isfinite(values).all():
        return None
    x1,y1,x2,y2=values
    if x2-x1<4 or y2-y1<4:
        return None
    px,py=.1*(x2-x1),.1*(y2-y1)
    left,top=max(0,int(x1-px)),max(0,int(y1-py))
    right,bottom=min(frame.shape[1],int(np.ceil(x2+px))),min(frame.shape[0],int(np.ceil(y2+py)))
    if right-left<4 or bottom-top<4:
        return None
    crop=cv2.resize(frame[top:bottom,left:right],(32,32),interpolation=cv2.INTER_AREA)
    gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    histograms=[]
    for channel,limit in ((0,180),(1,256),(2,256)):
        hist=cv2.calcHist([hsv],[channel],None,[16],[0,limit]).ravel()
        histograms.append(hist/max(1.,float(hist.sum())))
    moments=np.r_[hsv.mean(axis=(0,1))/[180.,255.,255.],hsv.std(axis=(0,1))/[180.,255.,255.]]
    return np.concatenate([_hog(gray)]+histograms+[moments]).astype(np.float32)


class ROIBackgroundVerifier:
    def __init__(self, path, expected_sha256):
        path=Path(path)
        if path.is_symlink() or path.stat().st_size>16*1024*1024:
            raise ValueError('Invalid ROI verifier artifact')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected_sha256:
            raise ValueError('ROI verifier checksum differs')
        with np.load(path,allow_pickle=False) as values:
            self.mean=values['mean'].copy()
            self.scale=values['scale'].copy()
            self.coefficients=values['coefficients'].copy()
        n=len(self.mean)
        if self.scale.shape!=(n,) or self.coefficients.shape!=(n+1,4) or n!=378:
            raise ValueError('ROI feature contract differs')
        if (not all(np.isfinite(v).all() for v in (self.mean,self.scale,self.coefficients)) or
                np.any(self.scale<=0)):
            raise ValueError('Invalid ROI verifier parameters')

    def background_margin(self, frame, box):
        vector=features(frame,box)
        if vector is None:
            return None
        standardized=(vector-self.mean)/self.scale
        scores=np.r_[standardized,1.]@self.coefficients
        return float(scores[0]-scores[1:].max())
