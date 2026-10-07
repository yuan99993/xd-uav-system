"""Bounded, latest-only multi-target image observations, independent of YOLO.

No identity allocation, detector-time refresh or metric inference occurs here.
Source states belong exclusively to this worker; callbacks never hold its lock.
"""
from collections import OrderedDict, deque
from copy import deepcopy
import threading
import time

import cv2

from .pixel_lock import flow_boxes, flow_box, box_iou, color_box, coast_box, compatible_box
from .perception_evidence import identity_only


class LatestPixelWorker:
    def __init__(self, sources, publish, recovery=None, maximum_tracks=8,
                 maximum_anchor_age=2.2, motion=None, error=None):
        self.sources = frozenset(sources)
        self.publish, self.recovery, self.motion = publish, recovery, motion
        self.error = error
        self.maximum_tracks = max(1,min(16,int(maximum_tracks)))
        self.maximum_anchor_age = max(.1,min(5.,float(maximum_anchor_age)))
        self.condition = threading.Condition()
        self.pending, self.anchors = OrderedDict(), {}
        self.generation, self.closing = 0, False
        self.processing_generation = 0
        self.stats = dict(received=0,processed=0,dropped=0,observed=0,failures=0,ms_ewma=0.)
        self.processing_ms = deque(maxlen=128)
        self.states = {}
        self.thread = threading.Thread(target=self._run,name='perception_pixels',daemon=True)
        self.thread.start()

    def submit(self, source, header, frame, epoch):
        if source not in self.sources:
            return
        with self.condition:
            if self.closing:
                return
            self.stats['received'] += 1
            if source in self.pending:
                self.stats['dropped'] += 1
            self.pending[source] = (deepcopy(header),frame,epoch,self.generation)
            self.condition.notify()

    def seed(self, source, header, frame, targets, epoch):
        if source not in self.sources:
            return
        selected = sorted((t for t in targets if t.lifecycle_state=='confirmed'
                    and t.measurement_source=='detector'),
                    key=lambda t:(not t.selected,-t.tracking_quality))[:self.maximum_tracks]
        observed = [deepcopy(t) for t in selected]
        with self.condition:
            if self.closing:
                return
            self.anchors[source] = (deepcopy(header),frame,observed,epoch,self.generation)
            self.condition.notify()

    def clear(self):
        with self.condition:
            self.generation += 1
            self.pending.clear(); self.anchors.clear()
            self.condition.notify()

    def close(self):
        with self.condition:
            self.closing = True
            self.pending.clear(); self.anchors.clear()
            self.condition.notify_all()
        if self.thread is not threading.current_thread():
            self.thread.join(timeout=3)

    def snapshot(self):
        with self.condition:
            result = dict(self.stats)
            timings = tuple(self.processing_ms)
            result.update(pending_sources=len(self.pending),generation=self.generation,
                ready=self.thread.is_alive() and not self.closing)
        timings = sorted(timings)
        result['processing_p95_ms'] = timings[min(len(timings)-1,int(.95*len(timings)))] if timings else 0.
        return result

    def _run(self):
        active_generation = -1
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.closing or self.pending)
                if self.closing:
                    return
                source, job = self.pending.popitem(last=False)
                anchor = self.anchors.pop(source,None)
            header,frame,epoch,generation = job
            if generation != self.generation:
                continue
            if generation != active_generation:
                self.states.clear(); active_generation=generation
            start = time.perf_counter()
            self.processing_generation = generation
            try:
                targets = self._advance(source,header,frame,epoch,anchor)
                if generation == self.generation and targets is not None:
                    self.publish(source,header,frame.shape[1],frame.shape[0],targets,epoch,generation)
            except Exception as error:
                self.stats['failures'] += 1
                # Fail closed; the next detector anchor can reinitialize the source.
                self.states.pop(source,None)
                if self.error is not None:
                    self.error(source, error)
            elapsed = 1000*(time.perf_counter()-start)
            with self.condition:
                self.processing_ms.append(elapsed)
                self.stats['processed'] += 1
                self.stats['ms_ewma'] += .1*(elapsed-self.stats['ms_ewma'])

    def _advance(self, source, header, frame, epoch, anchor):
        stamp = header.stamp.to_sec()
        state = self.states.get(source)
        if state and (stamp<=state['stamp'] or epoch!=state['epoch']):
            return None
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        if state is None:
            state=dict(gray=gray,frame=frame,stamp=stamp,epoch=epoch,tracks={},
                history=deque(maxlen=15), failed=set(), velocities={}, last_valid={})
            self.states[source]=state
        if state['gray'].shape != gray.shape:
            state['tracks'].clear(); state['history'].clear()
            state['failed'].clear(); state['velocities'].clear(); state['last_valid'].clear()
        seeded = set()
        if anchor and anchor[3:]==(epoch,self.generation):
            ah,af,targets,_,_=anchor
            historical = next((entry for entry in state['history'] if entry[0]==ah.stamp.to_sec()),None)
            old = historical[1] if historical is not None else {}
            seed_gray = historical[2] if historical is not None else None
            for target in targets:
                identity=target.track_id
                current=state['tracks'].get(identity)
                if current and identity in old:
                    if not compatible_box(old[identity], target.bbox_xyxy, minimum_iou=.18):
                        if self.recovery:
                            self.recovery(source, header, target, 'anchor_conflict', epoch,self.processing_generation)
                        continue
                    delta=[a-b for a,b in zip(target.bbox_xyxy,old[identity])]
                    target.bbox_xyxy=[int(a+b) for a,b in zip(current.bbox_xyxy,delta)]
                elif ah.stamp.to_sec()<state['stamp']:
                    if seed_gray is None:
                        seed_gray=cv2.cvtColor(af,cv2.COLOR_BGR2GRAY)
                    moved=flow_box(seed_gray,state['gray'],target.bbox_xyxy,
                                   state['stamp']-ah.stamp.to_sec())
                    if moved is None:
                        continue
                    target.bbox_xyxy=list(moved)
                    if stamp == state['stamp']:
                        seeded.add(identity)
                if ah.stamp.to_sec()<=stamp:
                    state['tracks'][identity]=target
                    state['failed'].discard(identity)
                    state['last_valid'][identity] = (state['stamp'], tuple(target.bbox_xyxy))
                    if ah.stamp.to_sec() == stamp:
                        seeded.add(identity)
        for identity,target in list(state['tracks'].items()):
            if target.last_detection_stamp.is_zero() or stamp-target.last_detection_stamp.to_sec()>self.maximum_anchor_age:
                del state['tracks'][identity]
                state['failed'].discard(identity)
                state['velocities'].pop(identity,None); state['last_valid'].pop(identity,None)
        boxes={i:tuple(t.bbox_xyxy) for i,t in state['tracks'].items()}
        dt=stamp-state['stamp']
        transform = self.motion(source,state['stamp'],stamp,state['gray'],gray,boxes) if self.motion and dt>0 else None
        rejected = {}
        active_boxes = {i:box for i,box in boxes.items() if i not in state['failed']}
        observations=flow_boxes(state['gray'],gray,active_boxes,dt,
            initial_transform=transform,rejections=rejected) if dt>0 else {}
        for identity,box in active_boxes.items():
            if identity not in observations and identity not in rejected and 0. < dt <= .8:
                moved = color_box(state['frame'],frame,box,dt)
                if moved is not None:
                    observations[identity] = (moved,.45)
        observations.update({identity:(boxes[identity], .5) for identity in seeded if identity in boxes})
        output=[]
        for identity,target in list(state['tracks'].items()):
            item=deepcopy(target)
            if identity in observations:
                box,quality=observations[identity]
                ambiguous=any(other!=identity and box_iou(box,value[0])>.6 and
                    box_iou(boxes[identity],boxes[other])<.1 for other,value in observations.items())
                if not ambiguous:
                    item.bbox_xyxy=list(box)
                    item.center_px=[.5*(box[0]+box[2]),.5*(box[1]+box[3])]
                    item.bbox_cxcywh_norm=[item.center_px[0]/frame.shape[1],item.center_px[1]/frame.shape[0],
                        (box[2]-box[0])/frame.shape[1],(box[3]-box[1])/frame.shape[0]]
                    item.measurement_source='visual_tracker'; item.lifecycle_state='confirmed'
                    item.tracking_quality=quality; item.position_valid=False
                    item.age_ms=0.
                    state['tracks'][identity]=deepcopy(item)
                    if dt > 0 and identity not in seeded:
                        state['velocities'][identity] = (
                            (item.center_px[0]-.5*(boxes[identity][0]+boxes[identity][2]))/dt,
                            (item.center_px[1]-.5*(boxes[identity][1]+boxes[identity][3]))/dt)
                    state['last_valid'][identity]=(stamp,box)
                    self.stats['observed']+=1
                else:
                    identity_only(item)
            else:
                identity_only(item)
            failed = item.measurement_source == 'stale'
            if failed and identity not in rejected and identity in state['velocities']:
                captured, last_box = state['last_valid'][identity]
                predicted = coast_box(last_box,state['velocities'][identity],stamp-captured,frame.shape)
                if predicted is not None:
                    item.bbox_xyxy=list(predicted)
                    item.center_px=[.5*(predicted[0]+predicted[2]),.5*(predicted[1]+predicted[3])]
                    item.bbox_cxcywh_norm=[item.center_px[0]/frame.shape[1],item.center_px[1]/frame.shape[0],
                        (predicted[2]-predicted[0])/frame.shape[1],(predicted[3]-predicted[1])/frame.shape[0]]
                    item.lifecycle_state='occluded'; item.measurement_source='prediction'
                    item.detection_confidence=0.; item.age_ms=1000*(stamp-captured)
                    state['failed'].add(identity)
            if item.measurement_source == 'stale':
                # Do not advance an obsolete ROI against a new background on
                # subsequent frames. Only a detector anchor may restore it.
                state['tracks'].pop(identity, None)
                state['failed'].discard(identity)
                state['velocities'].pop(identity,None); state['last_valid'].pop(identity,None)
            norm=item.bbox_cxcywh_norm
            edge = item.measurement_source=='visual_tracker' and (norm[0]-norm[2]/2<.04 or
                norm[1]-norm[3]/2<.04 or norm[0]+norm[2]/2>.96 or norm[1]+norm[3]/2>.96)
            if self.recovery and (failed or edge):
                self.recovery(source,header,deepcopy(target),rejected.get(identity,
                    'edge' if edge else 'flow_failed'),epoch,self.processing_generation)
            output.append(item)
        state['gray'],state['frame'],state['stamp']=gray,frame,stamp
        state['history'].append((stamp,{i:tuple(t.bbox_xyxy) for i,t in state['tracks'].items()},gray))
        return output
