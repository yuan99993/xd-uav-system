"""Display-only sparse visual tracks; no ROS publishers, model, or control output."""
import math
import cv2
import numpy as np


def overlap(a,b):
    area=max(0,min(a[0]+a[2],b[0]+b[2])-max(a[0],b[0]))*max(0,min(a[1]+a[3],b[1]+b[3])-max(a[1],b[1]))
    return area/max(1.,a[2]*a[3]+b[2]*b[3]-area)


def candidates(objects,proposals):
    rows=[]
    for original in list(objects)+list(proposals):
        if original.get('kind')=='manual_track':continue
        box=original.get('box')
        if box is None and 'aabb_xyxy' in original:
            x,y,r,b=original['aabb_xyxy'];box=[x,y,r-x,b-y]
        category=int(original.get('class_id',-1));confidence=float(original.get('confidence',0))
        if box is None or category not in (0,1,2) or confidence<.50:continue
        if not np.isfinite(list(box)+[confidence]).all() or min(box[2:])<6:continue
        row=dict(box=list(map(float,box)),class_id=category,confidence=confidence,
                 source_track_id=original.get('id') if original.get('kind')=='detector_track' else None)
        rows.append(row)
    kept=[]
    for row in sorted(rows,key=lambda x:x['confidence'],reverse=True):
        if not any(overlap(row['box'],other['box'])>.45 for other in kept):kept.append(row)
    return kept[:24]


class SoftTrackLayer:
    # Visual support cannot indefinitely renew its own detection evidence.
    max_perception_gap_sec=0.35
    max_tracks=12

    @staticmethod
    def corners(box):
        x,y,w,h=box
        return np.array([[[x,y],[x+w,y],[x+w,y+h],[x,y+h]]],np.float32)

    def project_geometry(self,track,matrix,shape):
        # Transform the persistent quadrilateral, never the previous AABB.
        # Re-bounding the AABB every frame compounds even pure rotation.
        quad=cv2.transform(track['quad'],matrix)
        points=quad[0]
        edge_w=float(np.linalg.norm(points[1]-points[0]))
        edge_h=float(np.linalg.norm(points[3]-points[0]))
        seed_w,seed_h=track['seed_size']
        if not (.65<=edge_w/seed_w<=1.5 and .65<=edge_h/seed_h<=1.5):return None
        lo=points.min(axis=0);hi=points.max(axis=0)
        width,height=hi-lo
        # Drop visibly inflated overlays even when the rotated edges pass.
        original_w,original_h=track.get('initial_size',track['seed_size'])
        if width*height>min(2.5*seed_w*seed_h,2.5*original_w*original_h):return None
        box=self.clip([lo[0],lo[1],width,height],shape)
        return (box,quad) if box is not None else None

    def __init__(self,labels):
        self.labels=list(labels);self.next_id=1;self.tracks={};self.previous=None;self.stamp=None;self.epoch=None

    @staticmethod
    def clip(box,shape):
        height, width = shape[:2]
        x, y, box_width, box_height = map(float, box)
        if not (0 <= x + box_width / 2 < width and 0 <= y + box_height / 2 < height):
            return None
        left = max(0., x)
        top = max(0., y)
        right = min(float(width), x + box_width)
        bottom = min(float(height), y + box_height)
        clipped_width, clipped_height = right - left, bottom - top
        if min(clipped_width, clipped_height) < 3:
            return None
        return [left, top, clipped_width, clipped_height]

    @staticmethod
    def crop(image,box):
        x,y,w,h=map(int,box);return image[y:y+h,x:x+w]

    def points(self,gray,box):
        roi=self.crop(gray,box)
        if min(roi.shape[:2],default=0)<6:return None
        p=cv2.goodFeaturesToTrack(roi,24,.015,2,blockSize=3)
        if p is not None:p+=np.array([[[int(box[0]),int(box[1])]]],np.float32)
        return p

    def histogram(self,image,box):
        roi=self.crop(image,box)
        if roi.size==0:return None
        hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        hist=cv2.calcHist([hsv],[0,1],None,[12,8],[0,180,0,256]).astype(np.float32)
        hist/=max(1.,float(hist.sum()));return hist

    def associate(self,image,gray,rows,stamp):
        used=set()
        for row in rows:
            box=self.clip(row['box'],gray.shape)
            if box is None:continue
            replaced_identifier=None
            ranked=sorted([(overlap(box,t['box']),identifier) for identifier,t in self.tracks.items()
                if t['class_id']==row['class_id'] and identifier not in used],reverse=True)
            if ranked and ranked[0][0]>=.30:
                if len(ranked)>1 and ranked[1][0]>=.30 and ranked[0][0]-ranked[1][0]<.12:continue
                identifier=ranked[0][1]
                original_size=self.tracks[identifier]['initial_size']
                if (box[2]>1.6*original_size[0] or box[3]>1.6*original_size[1]
                        or box[2]*box[3]>2.0*original_size[0]*original_size[1]):
                    replaced_identifier=identifier
                    identifier=self.next_id
            else:
                if any(overlap(box,t['box'])>.30 for t in self.tracks.values()):continue
                if len(self.tracks)>=self.max_tracks:continue
                identifier=self.next_id
            points=self.points(gray,box)
            if points is None or len(points)<4:continue
            if identifier == self.next_id:
                self.next_id+=1
            if replaced_identifier is not None:
                del self.tracks[replaced_identifier]
            old=self.tracks.get(identifier)
            initial_size=old['initial_size'] if old else box[2:]
            velocity=np.zeros(2,dtype=np.float32)
            if old and stamp>old['last_perception_stamp']:
                dt=stamp-old['last_perception_stamp']
                old_center=np.array(old['last_detection_center'])
                center=np.array([box[0]+box[2]/2,box[1]+box[3]/2])
                measured=(center-old_center)/dt
                if np.linalg.norm(measured) <= 500:
                    velocity=.5*old['velocity']+.5*measured
            self.tracks[identifier]=dict(box=box,initial_size=initial_size,class_id=row['class_id'],confidence=row['confidence'],
                points=points,quad=self.corners(box),seed_size=box[2:],histogram=self.histogram(image,box),
                last_detection_center=[box[0]+box[2]/2,box[1]+box[3]/2],velocity=velocity,
                source_track_id=row.get('source_track_id'),
                last_perception_stamp=stamp,visual_quality=1.)
            used.add(identifier)

    def propagate(self,gray,image):
        valid=[(identifier,t) for identifier,t in self.tracks.items() if t.get('points') is not None and len(t['points'])>=4]
        if not valid:self.tracks={};return
        p0=np.concatenate([t['points'] for _,t in valid]).astype(np.float32)
        p1,s1,_=cv2.calcOpticalFlowPyrLK(self.previous,gray,p0,None,winSize=(15,15),maxLevel=2)
        if p1 is None:self.tracks={};return
        pback,sback,_=cv2.calcOpticalFlowPyrLK(gray,self.previous,p1,None,winSize=(15,15),maxLevel=2)
        if pback is None:self.tracks={};return
        good=(s1.reshape(-1)>0)&(sback.reshape(-1)>0)&np.isfinite(p1.reshape(-1,2)).all(axis=1)
        good&=np.linalg.norm((pback-p0).reshape(-1,2),axis=1)<1.5
        updated={};offset=0
        for identifier,t in valid:
            count=len(t['points']);mask=good[offset:offset+count];old=p0[offset:offset+count][mask];new=p1[offset:offset+count][mask];offset+=count
            if len(new)<4 or float(mask.mean())<.55:continue
            matrix,inliers=cv2.estimateAffinePartial2D(old,new,method=cv2.RANSAC,ransacReprojThreshold=2.)
            if matrix is None or inliers is None or float(inliers.mean())<.65:continue
            scale=math.hypot(float(matrix[0,0]),float(matrix[1,0]))
            if not .80<=scale<=1.25:continue
            geometry=self.project_geometry(t,matrix,gray.shape)
            if geometry is None:continue
            box,quad=geometry
            hist=self.histogram(image,box);anchor=t['histogram']
            if hist is None or anchor is None or float(np.sqrt(hist*anchor).sum())<.60:continue
            t=dict(t,box=box,quad=quad,points=new.reshape(-1,1,2),visual_quality=float(mask.mean()))
            updated[identifier]=t
        self.tracks=updated

    def step(self,image,stamp,epoch,current_candidates,previous_candidates=()):
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        if len(self.tracks)>self.max_tracks:
            keep=sorted(self.tracks,key=lambda identifier:self.tracks[identifier]['confidence'],reverse=True)[:self.max_tracks]
            self.tracks={identifier:self.tracks[identifier] for identifier in keep}
        if self.epoch!=epoch or self.previous is None or stamp-self.stamp>.25 or stamp<=self.stamp:
            self.tracks={};self.previous=None
        elif previous_candidates:
            # Late numeric outputs are applied to their OWN previous image,
            # then transported by current pixels. Never relabel old boxes fresh.
            self.associate(self.previous_image,self.previous,previous_candidates,self.stamp)
        self.tracks={identifier:t for identifier,t in self.tracks.items()
            if stamp-t['last_perception_stamp']<=self.max_perception_gap_sec}
        if self.previous is not None:
            before=dict(self.tracks)
            self.propagate(gray,image)
            dt=stamp-self.stamp
            for identifier,track in before.items():
                if identifier in self.tracks or stamp-track['last_perception_stamp']>self.max_perception_gap_sec:
                    continue
                dx,dy=track['velocity']*dt
                matrix=np.array([[1.,0.,dx],[0.,1.,dy]],dtype=np.float32)
                geometry=self.project_geometry(track,matrix,gray.shape)
                if geometry is None:continue
                box,quad=geometry
                points=track['points']+np.array([[[dx,dy]]],np.float32)
                self.tracks[identifier]=dict(track,box=box,quad=quad,points=points,visual_quality=.5)
        self.associate(image,gray,current_candidates,stamp)
        self.previous=gray;self.previous_image=image;self.stamp=stamp;self.epoch=epoch
        rows=[]
        for identifier,t in self.tracks.items():
            category=t['class_id'];name=self.labels[category] if category<len(self.labels) else 'unknown'
            x,y,w,h=t['box']
            rows.append(dict(id=700000000+identifier,name=name,class_id=category,
                kind='soft_track',soft_lock=True,box=list(t['box']),center=[x+w/2,y+h/2],
                confidence=min(t['confidence'],t['visual_quality']),confidence_kind='visual_support',
                evidence_kind='current_pixels_with_perception_refresh',capture_stamp=stamp,
                last_perception_stamp=t['last_perception_stamp'],source_track_id=t.get('source_track_id')))
        return rows


def compose_display(soft,primary_id,primary_box,original):
    # Auxiliary numeric IDs are display-only and never become a control observation.
    result=[]
    for row in soft:
        b=row['box']
        if primary_id>=0 and primary_box:
            cx,cy,w,h=primary_box;hard=[cx-w/2,cy-h/2,w,h]
            inside=(hard[0]<=row['center'][0]<=hard[0]+w and hard[1]<=row['center'][1]<=hard[1]+h)
            if inside or overlap(b,hard)>.20:continue
        result.append(dict(row))
    result.extend(dict(o) for o in original if o['id']==primary_id and primary_id>=0)
    return result
