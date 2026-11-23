"""Target-local image motion witness, with no ground/depth assumption."""
import cv2
import numpy as np


def foreground_support(image, box, return_model=False):
    """Select ROI colours that are rarer in its surrounding background.

    The model uses only the previous capture, never truth or the next image.
    Quantized HSV jointly preserves neutral/black targets via saturation/value.
    """
    h,w=image.shape[:2];x,y,bw,bh=box
    x1,y1=max(0,int(x)),max(0,int(y));x2,y2=min(w,int(x+bw)),min(h,int(y+bh))
    margin=max(8,int(.35*max(bw,bh)))
    left,top=max(0,x1-margin),max(0,y1-margin)
    right,bottom=min(w,x2+margin),min(h,y2+margin)
    hsv=cv2.cvtColor(image[top:bottom,left:right],cv2.COLOR_BGR2HSV)
    bins=(hsv[:,:,0].astype(np.int32)//15)*64+(hsv[:,:,1].astype(np.int32)//32)*8+hsv[:,:,2].astype(np.int32)//32
    inside=np.zeros(bins.shape,bool);inside[y1-top:y2-top,x1-left:x2-left]=True
    if not inside.any() or (~inside).sum()<16:return None
    fg=np.bincount(bins[inside],minlength=768).astype(float);bg=np.bincount(bins[~inside],minlength=768).astype(float)
    fg/=fg.sum();bg/=bg.sum()
    # A background colour common in both areas is not target support.
    distinctive=(fg>=3.*np.maximum(bg,1./max(1,(~inside).sum()))) & (fg>0.)
    mask=np.zeros((h,w),np.uint8)
    patch=(inside & distinctive[bins]).astype(np.uint8)*255
    mask[top:bottom,left:right]=patch
    return (mask,distinctive) if return_model else mask


def selected_flow(previous, current, box, local_crop=False, foreground_only=False, initial_transform=None, window_size=None, pyramid_levels=None):
    """Return a current-pixel box only after bounded forward/backward LK + RANSAC.

    This is an independent witness to a local tracker, never a detector or a
    means of recovering identity. Textureless/occluded/ambiguous support fails.
    """
    if previous is None or current is None or previous.shape!=current.shape:
        return None,dict(valid=False,reason='missing_image_pair')
    h,w=previous.shape[:2];x,y,bw,bh=box
    x1,y1=max(0,int(x+.08*bw)),max(0,int(y+.08*bh))
    x2,y2=min(w,int(x+.92*bw)),min(h,int(y+.92*bh))
    if x2-x1<6 or y2-y1<6:return None,dict(valid=False,reason='small_support')
    left,top,right,bottom=0,0,w,h
    if local_crop:
        # Preserve native pixels and all target features. The 128px margin
        # exceeds the LK pyramid/window support for this stationary stage.
        left,top=max(0,x1-128),max(0,y1-128)
        right,bottom=min(w,x2+128),min(h,y2+128)
    before=cv2.cvtColor(previous[top:bottom,left:right],cv2.COLOR_BGR2GRAY)
    after=cv2.cvtColor(current[top:bottom,left:right],cv2.COLOR_BGR2GRAY)
    mask=np.zeros(before.shape,np.uint8);mask[y1-top:y2-top,x1-left:x2-left]=255
    foreground_fraction=None
    if foreground_only:
        model=foreground_support(previous[top:bottom,left:right],(x-left,y-top,bw,bh),return_model=True)
        if model is None:return None,dict(valid=False,reason='foreground_support_unavailable')
        support,lookup=model
        mask=cv2.bitwise_and(mask,support)
        foreground_fraction=float(np.count_nonzero(mask))/max(1,(x2-x1)*(y2-y1))
        if np.count_nonzero(mask)<6:return None,dict(valid=False,reason='insufficient_foreground_support')
    points=cv2.goodFeaturesToTrack(before,48,.02,3,mask=mask,blockSize=3)
    if points is None or len(points)<6:return None,dict(valid=False,reason='insufficient_target_texture')
    window=(9 if foreground_only else 15) if window_size is None else window_size
    levels=(1 if foreground_only else 3) if pyramid_levels is None else pyramid_levels
    if type(window) is not int or window<5 or window>31 or window%2!=1 or type(levels) is not int or not 0<=levels<=4:
        raise ValueError('Invalid selected-flow window/pyramid bounds')
    options=dict(winSize=(window,window),maxLevel=levels,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,20,.01))
    initial=None
    if initial_transform is not None:
        transform=np.asarray(initial_transform,dtype=np.float64)
        if transform.shape!=(3,3) or not np.isfinite(transform).all():
            return None,dict(valid=False,reason='invalid_motion_seed')
        origin=np.float32([left,top])
        initial=cv2.perspectiveTransform(points+origin,transform)-origin
        if not np.isfinite(initial).all():return None,dict(valid=False,reason='invalid_motion_seed')
    forward,status,_=cv2.calcOpticalFlowPyrLK(before,after,points,initial,
        flags=cv2.OPTFLOW_USE_INITIAL_FLOW if initial is not None else 0,**options)
    if forward is None:return None,dict(valid=False,reason='forward_failed')
    backward,back_status,_=cv2.calcOpticalFlowPyrLK(after,before,forward,points.copy() if initial is not None else None,
        flags=cv2.OPTFLOW_USE_INITIAL_FLOW if initial is not None else 0,**options)
    if backward is None:return None,dict(valid=False,reason='backward_failed')
    error=np.linalg.norm(backward-points,axis=2).ravel()
    valid=(status.ravel()!=0)&(back_status.ravel()!=0)&np.isfinite(error)&(error<.8)
    if valid.sum()<6 or valid.mean()<.7:return None,dict(valid=False,reason='inconsistent_target_flow')
    origin=np.float32([left,top])
    a=points.reshape(-1,2)[valid]+origin;b=forward.reshape(-1,2)[valid]+origin
    matrix,inliers=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=1.5)
    if matrix is None or inliers is None or inliers.sum()<6 or inliers.mean()<.8:
        return None,dict(valid=False,reason='inconsistent_target_transform')
    scale=float(np.hypot(matrix[0,0],matrix[1,0]))
    if not .85<=scale<=1.18:return None,dict(valid=False,reason='target_scale_jump')
    corners=np.float32([[x,y],[x+bw,y],[x+bw,y+bh],[x,y+bh]])
    projected=cv2.transform(corners[None],matrix)[0]
    left,top=projected.min(axis=0);right,bottom=projected.max(axis=0)
    if not (0<=left<right<=w and 0<=top<bottom<=h):
        return None,dict(valid=False,reason='target_outside_frame')
    box=tuple(float(v) for v in (left,top,right-left,bottom-top))
    return box,dict(valid=True,reason='current_target_pixels',points=int(valid.sum()),
        inlier_ratio=float(inliers.mean()),fb_error_median_px=float(np.median(error[valid])),scale=scale,
        foreground_only=bool(foreground_only),foreground_fraction=foreground_fraction,motion_seeded=initial is not None)
