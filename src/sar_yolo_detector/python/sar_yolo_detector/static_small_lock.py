"""Opt-in civilian, manually selected small-object pixel verification.

An immutable selection patch, a foreground component against its ring, local
negative matches and consecutive fresh captures corroborate KCF/CSRT. This is
not object classification, detector evidence, range or prediction. No online
anchor learning: an empty background must not replace the selected subject.
"""
import math
import cv2
import numpy as np


class StaticSmallLock:
    def __init__(self, config=None):
        self.config=dict(config or {});self.enabled=bool(self.config.get('enabled',False))
        self.blur_sigma=float(self.config.get('photometric_blur_sigma',0.))
        if not math.isfinite(self.blur_sigma) or not 0<=self.blur_sigma<=2.:raise ValueError('Invalid photometric blur sigma')
        self.reset()

    def gray(self,frame):
        result=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        return cv2.GaussianBlur(result,(0,0),self.blur_sigma) if self.blur_sigma else result

    def costs(self,search,anchor,weights,backgrounds=None,reference_background=0.):
        if not self.config.get('sparse_foreground_costs',False):
            windows=np.lib.stride_tricks.sliding_window_view(search,anchor.shape)
            difference=windows.astype(np.float32)-anchor
            if backgrounds is not None:difference-=backgrounds[:,:,None,None]-reference_background
            return (np.abs(difference)*weights).sum(axis=(-1,-2))/weights.sum()
        ys,xs=np.nonzero(weights)
        height,width=np.asarray(search.shape)-np.asarray(anchor.shape)+1
        samples=search[np.arange(height)[:,None,None]+ys,np.arange(width)[None,:,None]+xs]
        difference=samples.astype(np.float32)-anchor[ys,xs]
        if backgrounds is not None:difference-=backgrounds[:,:,None]-reference_background
        return (np.abs(difference)*weights[ys,xs]).sum(axis=-1)/weights.sum()

    def reset(self):
        self.anchor=None;self.mask=None;self.armed=False;self.confirmed=False;self.applicable=False
        self.count=0;self.last_stamp=None;self.first_stamp=None;self.kind=0;self.seed_stamp=None
        self.seed_status={};self.last_status={}
        self.reference_gray=None;self.camera_matrix=np.eye(3);self.reference_origin=None;self.crop_origin=None
        self.reference_box=None;self.ever_confirmed=False;self.retry_start=None;self.last_reference_box=None

    @staticmethod
    def patch(gray, box, margin=0):
        x,y,w,h=map(float,box);width=int(round(w))+2*margin;height=int(round(h))+2*margin
        if min(width,height)<1 or x-margin<0 or y-margin<0 or x+w+margin>=gray.shape[1] or y+h+margin>=gray.shape[0]:return None
        return cv2.getRectSubPix(gray,(width,height),(x+w/2-.5,y+h/2-.5))

    def initialize(self, frame, box, stamp):
        status=self._initialize(frame,box,stamp)
        self.seed_status=dict(status)
        return status

    def _initialize(self, frame, box, stamp):
        self.reset()
        if not self.enabled:return dict(valid=False,reason='disabled')
        x,y,w,h=map(float,box)
        if not all(map(math.isfinite,[x,y,w,h,stamp])) or min(w,h)<6 or max(w,h)>64:
            return dict(valid=False,reason='unsupported_pixel_size')
        self.applicable=True
        gray=self.gray(frame);patch=self.patch(gray,box);context=self.patch(gray,box,4)
        if patch is None or context is None:return dict(valid=False,reason='outside_image')
        ring=np.ones(context.shape,bool);ring[4:-4,4:-4]=False;bg=float(np.median(context[ring]));noise=float(np.median(np.abs(context[ring].astype(float)-bg)))
        threshold=max(10.,min(24.,1.5*noise));choices=[]
        for kind in [-1,1]:
            foreground=((patch.astype(float)-bg)*kind>=threshold).astype(np.uint8)
            n,labels,stats,centroids=cv2.connectedComponentsWithStats(foreground,8)
            for i in range(1,n):
                area=int(stats[i,cv2.CC_STAT_AREA]);cx,cy=centroids[i]
                if not 3<=area<=patch.size*.65:continue
                distance=math.hypot((cx-patch.shape[1]/2)/patch.shape[1],(cy-patch.shape[0]/2)/patch.shape[0])
                if distance>.4:continue
                choices.append((area/(1+8*distance),kind,labels==i))
        if not choices or float(patch.std())<4:return dict(valid=False,reason='no_distinct_foreground')
        _,self.kind,self.mask=max(choices,key=lambda v:v[0]);self.anchor=patch.astype(np.float32)
        if self.config.get('multi_component_foreground',False):
            # A single selected object can have disconnected rotors/wheels.
            # Keep its complete contrast silhouette, not one interchangeable
            # fragment that would match another part of the SAME object.
            foreground=((patch.astype(float)-bg)*self.kind>=threshold).astype(np.uint8)
            n,labels,stats,centroids=cv2.connectedComponentsWithStats(foreground,8)
            union=np.zeros(patch.shape,bool)
            for i in range(1,n):
                if stats[i,cv2.CC_STAT_AREA]>=2:union|=labels==i
            if 3<=union.sum()<=patch.size*.65:self.mask=union

        if self.mask.sum()>=8 and not self.config.get("multi_component_foreground",False):
            values=self.anchor[self.mask]*self.kind
            self.mask &= self.anchor*self.kind>=np.percentile(values,60)
        self.initial_mask_count=int(self.mask.sum())
        self.reference_background=bg
        self.contrast=(float(self.anchor[self.mask].mean())-bg)*self.kind
        pad=max(24,int(math.ceil(max(w,h))))
        left,top=max(0,int(x)-pad),max(0,int(y)-pad)
        right,bottom=min(gray.shape[1],int(math.ceil(x+w))+pad),min(gray.shape[0],int(math.ceil(y+h))+pad)
        self.reference_gray=gray[top:bottom,left:right].copy();self.crop_origin=(left,top)
        self.reference_box=(x,y,w,h)
        self.reference_origin=(x+(w-self.anchor.shape[1])/2,y+(h-self.anchor.shape[0])/2)
        self.box_size=(w,h);self.armed=True;self.applicable=True;self.last_stamp=float(stamp);self.seed_stamp=float(stamp)
        return dict(valid=False,reason='awaiting_fresh_captures',foreground_pixels=int(self.mask.sum()),contrast=self.contrast)

    def update(self, frame, box, stamp, geometry_supported=False, camera_transform=None, absolute_reference=False, image_reference_transform=None):
        box,status=self._update(frame,box,stamp,geometry_supported,camera_transform,absolute_reference,image_reference_transform)
        reason=status.get('reason')
        if (geometry_supported or image_reference_transform is not None) and reason in ('pixel_verification_failed','ambiguous_foreground'):
            if self.retry_start is None:self.retry_start=stamp
            if stamp-self.retry_start<=.75:
                # Missing measurements remain missing. A brief photometric
                # outlier must not destroy the immutable identity forever.
                self.armed=True;self.confirmed=False;self.count=0;self.first_stamp=None
                status['revalidation_pending']=True
        if self.retry_start is not None and stamp-self.retry_start>.75:
            self.armed=False;box=None;status.update(valid=False,reason='pixel_revalidation_expired')
        if status.get('valid'):
            self.ever_confirmed=True;self.retry_start=None
        if reason!='not_armed':self.last_status=dict(status)
        if self.last_reference_box is not None:status['projected_reference_box']=self.last_reference_box
        return box,dict(status,initialization=self.seed_status,last_evidence=self.last_status)

    def _update(self, frame, box, stamp, geometry_supported=False, camera_transform=None, absolute_reference=False, image_reference_transform=None):
        status=dict(valid=False,reason='not_armed',confirmed=False)
        if not self.armed:return None,status
        if not math.isfinite(stamp) or stamp<=self.last_stamp:return None,dict(status,reason='non_fresh_capture')
        gap=stamp-self.last_stamp;self.last_stamp=stamp
        initial_pose_bootstrap=(geometry_supported and self.count==0 and not self.ever_confirmed
                                and self.first_stamp is None and gap<=2.)
        if absolute_reference and not self.ever_confirmed and stamp-self.seed_stamp>5.:
            self.armed=False;return None,dict(status,reason='initial_confirmation_timeout')
        if gap>(.75 if not self.ever_confirmed else .5) and not initial_pose_bootstrap:
            if absolute_reference and not self.ever_confirmed:
                # No identity has been trusted yet. Restart the five-current-
                # frame test after startup stalls, never bridge a locked gap.
                self.count=0;self.first_stamp=None
                status['initial_confirmation_restarted']=True
            else:
                self.armed=False;return None,dict(status,reason='capture_gap')
        if box is None:
            self.armed=False;return None,dict(status,reason='local_tracker_failed')
        x,y,w,h=map(float,box);aw,ah=self.box_size
        if not all(map(math.isfinite,[x,y,w,h])) or (not geometry_supported and image_reference_transform is None and (not .85<=w/aw<=1.18 or not .85<=h/ah<=1.18)) or max(w,h)>96:
            self.armed=False;return None,dict(status,reason='scale_or_numeric_jump')
        if image_reference_transform is not None:
            # An image-space SOT proposal supplies no calibrated world or
            # stationary-target geometry. Retain all dynamic negative gates.
            transform=np.asarray(image_reference_transform,dtype=float)
            if transform.shape!=(3,3) or not np.isfinite(transform).all() or abs(np.linalg.det(transform))<1e-10:
                self.armed=False;return None,dict(status,reason='invalid_image_reference_transform')
            if not .5<=w/aw<=2. or not .5<=h/ah<=2.:
                self.armed=False;return None,dict(status,reason='image_template_scale_limit')
            self.camera_matrix=transform
        elif geometry_supported:
            if camera_transform is None or np.shape(camera_transform)!=(3,3) or not np.isfinite(camera_transform).all():
                self.armed=False;return None,dict(status,reason='missing_calibrated_transform')
            self.camera_matrix=(np.asarray(camera_transform) if absolute_reference else
                                np.asarray(camera_transform)@self.camera_matrix)
        if geometry_supported or image_reference_transform is not None:
            rx,ry,rw,rh=self.reference_box
            corners=np.float64([[rx,ry],[rx+rw,ry],[rx+rw,ry+rh],[rx,ry+rh]])
            projected=cv2.perspectiveTransform(corners[None],self.camera_matrix)[0]
            left,top=projected.min(axis=0);right,bottom=projected.max(axis=0)
            x,y,w,h=map(float,(left,top,right-left,bottom-top))
            self.last_reference_box=(x,y,w,h)
            if not np.isfinite(projected).all() or min(w,h)<4 or max(w,h)>96:
                self.armed=False;return None,dict(status,reason='projected_geometry_invalid')
        # Fixed-FOV stationary stage: a small bounded translation search only.
        if geometry_supported or image_reference_transform is not None:aw,ah=w,h
        radius=2 if geometry_supported else max(4,min(12,int(math.ceil(.65*max(aw,ah)))))
        gray=self.gray(frame);search=self.patch(gray,(x,y,aw,ah),radius)
        if search is None:self.armed=False;return None,dict(status,reason='outside_image')
        anchor=self.anchor;weights=self.mask.astype(np.float32)
        reference_background=self.reference_background
        if geometry_supported or image_reference_transform is not None:
            # Reproject the immutable selection pixels, rather than comparing
            # unrotated pixels after the gimbal turns. This uses measured K/R,
            # not a ground plane, target depth or simulator target position.
            ox,oy=self.reference_origin;placement=np.array([[1.,0.,ox],[0.,1.,oy],[0.,0.,1.]])
            width,height=int(round(aw)),int(round(ah))
            origin_x=x+(aw-width)/2-4;origin_y=y+(ah-height)/2-4
            destination=np.array([[1.,0.,-origin_x],[0.,1.,-origin_y],[0.,0.,1.]])
            cx,cy=self.crop_origin;source=np.array([[1.,0.,cx],[0.,1.,cy],[0.,0.,1.]])
            shape=(width+8,height+8)
            context=cv2.warpPerspective(self.reference_gray,destination@self.camera_matrix@source,shape)
            projected=cv2.warpPerspective(self.mask.astype(np.float32),destination@self.camera_matrix@placement,shape)
            anchor=context[4:-4,4:-4].astype(np.float32);weights=projected[4:-4,4:-4]
            if context is None or weights.sum()<max(2.,.18*self.initial_mask_count):
                self.armed=False;return None,dict(status,reason='insufficient_projected_foreground')
            ring=np.ones(context.shape,bool);ring[4:-4,4:-4]=False
            reference_background=float(np.median(context[ring]))
        # Foreground matching, not background NCC. The selection is immutable;
        # bootstrap can only remove inconsistent foreground pixels, never learn
        # background pixels or renew a vanished identity.
        windows=np.lib.stride_tricks.sliding_window_view(search,anchor.shape)
        backgrounds=None
        if self.config.get('contrast_normalized_search',False):
            outer=self.patch(gray,(x,y,aw,ah),radius+4)
            if outer is None:self.armed=False;return None,dict(status,reason='outside_image')
            context_shape=(anchor.shape[0]+8,anchor.shape[1]+8)
            contexts=np.lib.stride_tricks.sliding_window_view(outer,context_shape)
            ring_mask=np.ones(context_shape,bool);ring_mask[4:-4,4:-4]=False
            backgrounds=np.median(contexts[:,:,ring_mask],axis=-1).astype(np.float32)
        costs=self.costs(search,anchor,weights,backgrounds,reference_background)
        yy,xx=np.indices(costs.shape)
        allowed=(np.abs(yy-radius)<=2)&(np.abs(xx-radius)<=2) if geometry_supported else np.ones(costs.shape,bool)
        best=float(np.min(costs[allowed]))
        close=(costs<=best+1.)&allowed
        distance=(yy-radius)**2+(xx-radius)**2
        py,px=np.unravel_index(np.argmin(np.where(close,distance,float('inf'))),costs.shape)
        current=windows[py,px].astype(float)
        proposed=(x+px-radius,y+py-radius,aw,ah);context=self.patch(gray,proposed,4)
        if context is None:self.armed=False;return None,dict(status,reason='outside_image')
        ring=np.ones(context.shape,bool);ring[4:-4,4:-4]=False;bg=float(np.median(context[ring]))
        if not self.confirmed and not geometry_supported and image_reference_transform is None:
            self.mask &= (np.abs(current-self.anchor)<=8.) & ((current-bg)*self.kind>=8.)
        support=int(self.mask.sum())
        if support<max(3,int(math.ceil(.2*self.initial_mask_count))):
            self.armed=False;return None,dict(status,reason='no_stable_foreground',foreground_pixels=support)
        if not geometry_supported and image_reference_transform is None:weights=self.mask.astype(np.float32)
        absolute_mae=float((np.abs(current-anchor)*weights).sum()/weights.sum())
        # Compare contrast against the surrounding ring. A uniform exposure
        # change is not identity loss; the current silhouette must still exist.
        foreground_mae=float((np.abs((current-bg)-(anchor-reference_background))*weights).sum()/weights.sum())
        contrast=(float((current*weights).sum()/weights.sum())-bg)*self.kind
        expected_contrast=self.contrast if reference_background is None else (float((anchor*weights).sum()/weights.sum())-reference_background)*self.kind
        costs=self.costs(search,anchor,weights,backgrounds,reference_background)
        alternatives=costs.copy();ex=max(1,int(.35*aw));ey=max(1,int(.35*ah))
        alternatives[max(0,py-ey):py+ey+1,max(0,px-ex):px+ex+1]=float('inf')
        # Sparse surrounding negatives stay outside the target core even
        # when a large ROI covers the entire bounded translation search.
        negative_costs=[float(alternatives.min())]
        dx,dy=(int(math.ceil(aw))+4,int(math.ceil(ah))+4) if self.config.get("multi_component_foreground",False) else (max(ex+2,int(aw/2)+4),max(ey+2,int(ah/2)+4))
        for nx,ny in ((-dx,0),(dx,0),(0,-dy),(0,dy),(-dx,-dy),(-dx,dy),(dx,-dy),(dx,dy)):
            negative=self.patch(gray,(proposed[0]+nx,proposed[1]+ny,aw,ah))
            if negative is not None:
                difference=negative.astype(np.float32)-anchor
                if backgrounds is not None:
                    negative_context=self.patch(gray,(proposed[0]+nx,proposed[1]+ny,aw,ah),4)
                    if negative_context is None:continue
                    negative_ring=np.ones(negative_context.shape,bool);negative_ring[4:-4,4:-4]=False
                    difference-=float(np.median(negative_context[negative_ring]))-reference_background
                negative_costs.append(float((np.abs(difference)*weights).sum()/weights.sum()))
        margin=float(min(negative_costs)-costs[py,px])
        if not math.isfinite(margin):
            self.armed=False;return None,dict(status,reason='insufficient_negative_support')
        status.update(reason='pixel_verification_failed',negative_mae_margin=margin,
                      foreground_mae=foreground_mae,absolute_foreground_mae=absolute_mae,contrast=contrast,expected_contrast=expected_contrast,foreground_pixels=support)
        # Retain at least half the immutable silhouette contrast. The
        # absolute floor covers low-contrast codec noise; the upper bound
        # prevents a high-contrast seed from accepting arbitrary pixels.
        error_limit=max(8.,min(16.,.5*expected_contrast))
        contrast_limit=max(8.,.5*expected_contrast)
        status.update(foreground_error_limit=error_limit,minimum_contrast=contrast_limit)
        valid=(foreground_mae<=error_limit and contrast>=contrast_limit)
        if not valid:
            self.armed=False;return None,status
        if self.first_stamp is None:self.first_stamp=stamp
        self.count+=1
        confirmed=self.count>=(2 if self.ever_confirmed else 5) and stamp-self.first_stamp>=(.025 if self.ever_confirmed else .1)
        if confirmed and margin<max(1.,.08*self.contrast) and not geometry_supported:
            self.armed=False;return None,dict(status,reason='ambiguous_foreground')
        status.update(valid=confirmed,confirmed=confirmed,reason='current_foreground_pixels' if confirmed else 'confirming_fresh_captures',
                      confirmations=self.count,score=min(1.-foreground_mae/40.,.8+min(.2,margin/40.)))
        status['stationary_geometry_supported']=bool(geometry_supported)
        status['image_template_scaled']=image_reference_transform is not None
        status['ambiguous_response']=bool(margin<max(1.,.08*self.contrast))
        if status['ambiguous_response']:status['score']=min(.55,status['score'])
        self.confirmed=confirmed
        return (proposed if confirmed else None),status
