"""Bounded, time-based YOLO/local-feature fusion; no ROS or model dependency.

Local tracker success is not a probability. Appearance and motion validate it.
Only compatible observations are blended; prediction is never an observation.
"""
from dataclasses import dataclass, replace
from collections import Counter
import time
import math
from typing import Optional, Tuple

import cv2
import numpy as np

from .manual_box_tracker import ManualBoxTracker
from .selected_flow import selected_flow
from .static_small_lock import StaticSmallLock
from .camera_motion import CameraMotion, transform_window, correct_window_support


@dataclass(frozen=True)
class Candidate:
    box: Tuple[float, float, float, float]  # x, y, width, height, original pixels
    class_id: int
    confidence: float
    local_id: int
    stable: bool = True
    detector_stamp: Optional[float] = None
    camera_projected: bool = False
    selected_public_id: int = -1
    capture_box: Optional[Tuple[float, float, float, float]] = None


@dataclass(frozen=True)
class FusionResult:
    box: Optional[Tuple[float, float, float, float]]
    public_id: int
    class_id: int
    confidence: float
    source: str  # fused / yolo / visual / prediction / lost / ambiguous / idle
    reason: str
    last_detection_stamp: float
    local_id: int

    @property
    def observed(self):
        return self.source in ("fused", "yolo", "visual") and self.box is not None


def iou(a, b):
    x, y = max(a[0], b[0]), max(a[1], b[1])
    right = min(a[0] + a[2], b[0] + b[2])
    bottom = min(a[1] + a[3], b[1] + b[3])
    area = max(0., right - x) * max(0., bottom - y)
    return area / max(1., a[2] * a[3] + b[2] * b[3] - area)


def class_duplicate_indices(candidates, class_groups, preferred_id=-1,
                            overlap=.75, center_fraction=.20,
                            suppress_selected_same_class=False):
    """Keep original indices; suppress only near-identical sibling-class boxes.

    This is output suppression, not identity merging. Same-class competitors
    remain available to ambiguity gating. At the final output boundary only,
    an already validated selected box can suppress a near-identical same-class
    duplicate. This never merges identities or affects input candidates.
    """
    def duplicate(a, b):
        same_class = a.class_id == b.class_id
        if same_class:
            if not (suppress_selected_same_class and preferred_id >= 0
                    and preferred_id in (a.local_id, b.local_id)):
                return False
        elif not any(a.class_id in group and b.class_id in group for group in class_groups):
            return False
        if iou(a.box, b.box) < (max(.85, overlap) if same_class else overlap):
            return False
        ax, ay, aw, ah = a.box
        bx, by, bw, bh = b.box
        return (abs(ax + aw / 2. - bx - bw / 2.) <= center_fraction * min(aw, bw)
                and abs(ay + ah / 2. - by - bh / 2.) <= center_fraction * min(ah, bh))

    ranked = sorted(range(len(candidates)), key=lambda index: (
        preferred_id >= 0 and candidates[index].local_id == preferred_id,
        candidates[index].confidence), reverse=True)
    kept = []
    for index in ranked:
        if not any(duplicate(candidates[index], candidates[other]) for other in kept):
            kept.append(index)
    return sorted(kept)


def center(box):
    return np.array((box[0] + box[2] / 2., box[1] + box[3] / 2.))


def valid_box(box, shape):
    if box is None or len(box) != 4 or not np.all(np.isfinite(box)):
        return False
    x, y, w, h = box
    return w >= 4 and h >= 4 and x >= 0 and y >= 0 and x + w <= shape[1] + .01 and y + h <= shape[0] + .01


def appearance(frame, box):
    if not valid_box(box, frame.shape):
        return None, 0.
    x, y, w, h = box
    roi = frame[int(y):int(y + h), int(x):int(x + w)]
    roi = cv2.resize(roi, (48, 48))
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    # Featureless background cannot maintain an identity or update its anchor.
    quality = min(1., float(gray.std()) / 18.) * min(1., min(w, h) / 24.)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Learn the subject against its surrounding background, rather than
    # assuming the centre or all chromatic pixels belong to the vehicle.
    # This keeps a partially off-centre black car from learning green grass.
    left,top,right,bottom=int(x),int(y),int(x+w),int(y+h)
    margin=max(4,int(.25*max(w,h)))
    l,t=max(0,left-margin),max(0,top-margin)
    r,b=min(frame.shape[1],right+margin),min(frame.shape[0],bottom+margin)
    surrounding=frame[t:b,l:r]
    ring=np.ones(surrounding.shape[:2],np.uint8)
    ring[top-t:bottom-t,left-l:right-l]=0
    samples=surrounding[ring>0]
    weights=np.ones((48,48),np.float64)
    if len(samples)>=16:
        samples=samples[::max(1,len(samples)//2048)]
        background=np.median(cv2.cvtColor(samples.reshape(-1,1,3),cv2.COLOR_BGR2HSV).reshape(-1,3),axis=0)
        values=hsv.astype(float)
        hue=np.abs(values[:,:,0]-background[0]);hue=np.minimum(hue,180-hue)/90.
        hue*=((values[:,:,1]>40)&(background[1]>40))
        saturation=np.abs(values[:,:,1]-background[1])/255.
        brightness_delta=np.abs(values[:,:,2]-background[2])/255.
        foreground=(hue>12/90.)|(saturation>50/255.)|(brightness_delta>25/255.)
        if np.count_nonzero(foreground)>=max(16,int(.10*foreground.size)):
            # Squared contrast suppresses weak ground shadows without excluding
            # achromatic subject pixels. It does not increase tracking quality.
            weights=np.maximum.reduce([hue,saturation,brightness_delta])**2*foreground
    hbin=np.minimum(15,hsv[:,:,0].astype(int)*16//180)
    sbin=np.minimum(7,hsv[:,:,1].astype(int)*8//256)
    hist=np.bincount((hbin*8+sbin).ravel(),weights=weights.ravel(),minlength=128).reshape(16,8)
    # Adjacent quantisation bins must not make one vehicle appear unrelated
    # after resampling. Hue wraps; saturation/value have bounded endpoints.
    hist=.6*hist+.2*np.roll(hist,1,axis=0)+.2*np.roll(hist,-1,axis=0)
    padded=np.pad(hist,((0,0),(1,1)),mode='edge')
    hist=.6*padded[:,1:-1]+.2*padded[:,:-2]+.2*padded[:,2:]
    brightness=np.bincount((hsv[:,:,2]//16).ravel(),weights=weights.ravel(),minlength=16)
    padded=np.pad(brightness,(1,1),mode='edge')
    brightness=.6*padded[1:-1]+.2*padded[:-2]+.2*padded[2:]
    hist=hist.ravel()/max(1e-9,np.linalg.norm(hist))
    brightness=brightness/max(1e-9,np.linalg.norm(brightness))
    feature=np.r_[.75*hist,math.sqrt(1-.75**2)*brightness]
    return feature/max(1e-9,np.linalg.norm(feature)),quality



class BoxFusion:
    def __init__(self, config=None, manual_config=None, tracker=None):
        self.config = dict(config or {})
        self.visual_hold = self._positive("visual_hold_sec", 1.5)
        self.predict_hold = self._positive("prediction_hold_sec", .35)
        self.reacquire_hold = self._positive("reacquire_hold_sec", 8.)
        self.identity_ttl = self._positive("identity_memory_ttl_sec", 60.)
        if self.identity_ttl <= self.reacquire_hold or self.identity_ttl > 300.:
            raise ValueError("Identity TTL must exceed short recovery and be <= 300 seconds")
        # One selected identity per camera instance, not an unbounded gallery.
        self.identity_capacity = 1
        self.long_appearance_gate = self._unit("long_reacquire_appearance_gate", .85)
        self.long_quality_gate = self._unit("long_reacquire_quality_gate", .65)
        self.long_score_gate = self._unit("long_reacquire_minimum_score", .65)
        self.long_motion_required = bool(self.config.get("long_reacquire_require_camera_motion", True))
        self.motion_horizon = self._positive("reacquire_motion_horizon_sec", 2.)
        self.max_gap = self._positive("max_frame_gap_sec", .75)
        self.area_ratio = self._positive("max_area_ratio", 2.5)
        self.center_gate = self._positive("center_gate_sizes", 1.4)
        self.appearance_gate = self._unit("appearance_gate", .70)
        self.quality_gate = self._unit("quality_gate", .35)
        self.margin = self._unit("ambiguity_margin", .12)
        self.confirmations = max(2, int(self.config.get("reacquire_confirm_frames", 3)))
        self.detector_weight = self._unit("detector_weight", .70)
        self.detector_authority = self._unit("detector_authority_confidence", .85)
        self.min_detector_conf = self._unit("minimum_detector_confidence", .50)
        self.maintenance_min_iou = self._unit("maintenance_min_iou", .35)
        self.min_maintenance_conf = self._unit("minimum_maintenance_confidence", .10)
        self.local_refresh_sec = self._positive("local_template_refresh_sec", .75)
        if self.min_maintenance_conf > self.min_detector_conf:
            raise ValueError("Maintenance confidence must not exceed identity confidence")
        self.class_agnostic_lock=bool(self.config.get("class_agnostic_lock",False))
        self.class_groups = self.config.get("class_groups", [[0, 1, 2, 3]])
        if not isinstance(self.class_groups, list) or any(not isinstance(group, list) or any(not isinstance(c, int) for c in group) for group in self.class_groups):
            raise ValueError("FusionTracker/class_groups must be a list of integer lists")
        options = dict(manual_config or {})
        options["tracker_type"] = "feature_tracker"
        self.tracker = tracker if tracker is not None else ManualBoxTracker(options)
        self.small_lock=StaticSmallLock(self.config.get('civil_static_small_lock',{}))
        # Same immutable pixel/negative verifier, in SOT image coordinates.
        # This instance has no stationary-world or ground-plane constraint.
        self.moving_pixel_lock=StaticSmallLock(self.config.get('moving_small_pixel_witness',{}))
        self.reset()

    def _positive(self, key, default):
        value = float(self.config.get(key, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError("FusionTracker/%s must be finite and positive" % key)
        return value

    def _unit(self, key, default):
        value = float(self.config.get(key, default))
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError("FusionTracker/%s must be between zero and one" % key)
        return value

    def reset(self):
        self.small_lock.reset()
        self.moving_pixel_lock.reset()
        self.tracker.reset()
        self.public_id = -1
        self.local_id = -1
        self.class_id = -1
        self.last_detector_candidate = None
        self.box = None
        self._camera_support=None
        self.anchor = None
        self._visual_reference=None
        self.anchor_area = None
        self.velocity = np.zeros(2)
        self._detector_reference_box=None
        self._detector_reference_support=None
        self._detector_reference_stamp=0.
        self._detector_velocity=np.zeros(2)
        self._detector_anchor_area=None
        self._detector_anchor_box=None
        self._detector_anchor_support=None
        self._strong_pending_box=None
        self._strong_pending_stamp=0.
        self._strong_pending_count=0
        self.stamp = 0.
        self.selection_stamp = 0.
        self.last_detection_stamp = 0.
        self.last_observation_stamp = 0.
        self.last_local_seed_stamp = 0.
        self.last_confidence = 0.
        self.pending_id = None
        self.pending_count = 0
        self.pending_stamp = 0.
        self.pending_box = None
        self.shape = None
        self._visual_box = None
        self._visual_quality = 0.
        self._previous_frame = None
        self._deferred_local_seed=None
        self.identity_archive = None
        self._motion_continuous = True
        self._long_recovery = False
        self._visual_bootstrap_pending=False;self._visual_bootstrap_count=0
        self._visual_hypothesis_box=None;self._visual_hypothesis_stamp=None;self._visual_revalidation_count=0
        self.diagnostics = {}
        self.result = FusionResult(None, -1, -1, 0., "idle", "no_selection", 0., -1)

    def initialize(self, frame, box, stamp, public_id, candidate=None):
        if not math.isfinite(stamp) or stamp <= 0 or public_id < 0 or not valid_box(box, frame.shape):
            raise ValueError("Invalid original-frame selection")
        self.reset()
        self.public_id = int(public_id)
        self.box = tuple(float(v) for v in box)
        self._camera_support=transform_window(self.box,np.eye(3))[1]
        self.stamp = self.last_observation_stamp = float(stamp)
        self.selection_stamp = float(stamp)
        self.shape = frame.shape[:2]
        self._previous_frame = frame
        self.anchor, quality = appearance(frame, box)
        self._visual_reference=self.anchor.copy()
        self.anchor_area = box[2] * box[3]
        self.tracker.initialize(frame, self.box)
        self.last_local_seed_stamp = stamp
        if candidate is None or candidate.class_id==2 or self.class_agnostic_lock:
            self.small_lock.initialize(frame,self.box,stamp)
        self.moving_pixel_lock.initialize(frame,self.box,stamp)
        if self.tracker.tracker_name == "template" and not self.config.get("allow_template_fallback", False):
            self.reset()
            raise ValueError("Fusion requires OpenCV contrib KCF/CSRT; template fallback is disabled")
        if candidate is not None:
            self.local_id = candidate.local_id
            self.class_id = candidate.class_id
            self.last_detection_stamp = stamp if candidate.detector_stamp is None else candidate.detector_stamp
            self.last_confidence = candidate.confidence
            source = "yolo" if self.last_detection_stamp == stamp else "visual"
        else:
            self.last_confidence = min(.65, quality)
            source = "visual" if quality >= self.quality_gate and not self.small_lock.applicable else "lost"
        if self.small_lock.applicable or self.moving_pixel_lock.applicable:
            # A detector association supplies a label/identity, not five fresh
            # small-target pixel confirmations. Selection is pending first.
            source = "lost"
        self._visual_bootstrap_pending=bool(self.config.get("joint_current_visual_witnesses",False) and source=="lost" and not self.small_lock.applicable)
        self._visual_bootstrap_count=0
        self.result = self._result(self.box if source != "lost" else None, source, "initialized")
        return self.result

    def _result(self, box, source, reason, confidence=None):
        self.result = FusionResult(box, self.public_id, self.class_id,
                                   self.last_confidence if confidence is None else confidence,
                                   source, reason, self.last_detection_stamp, self.local_id)
        if (self.public_id >= 0 and source in ("lost", "ambiguous", "prediction")
                and reason != "identity_memory_expired" and self.identity_archive is None):
            self.identity_archive = dict(public_id=self.public_id, local_id=self.local_id,
                class_id=self.class_id, last_seen=self.last_observation_stamp,
                expires_at=self.last_observation_stamp + self.identity_ttl,
                box=self.box, velocity=self.velocity.copy(),
                appearance=self.anchor.copy() if self.anchor is not None else None)
        elif self.result.observed:
            self.identity_archive = None
            self._motion_continuous = True
        return self.result

    def _class_compatible(self, class_id):
        return self.class_agnostic_lock or self.class_id < 0 or class_id == self.class_id or any(
            self.class_id in group and class_id in group for group in self.class_groups)

    def _compatible(self, box, reference):
        ratio = box[2] * box[3] / max(1., reference[2] * reference[3])
        distance = np.linalg.norm(center(box) - center(reference)) / max(8., math.hypot(reference[2], reference[3]))
        return 1. / self.area_ratio <= ratio <= self.area_ratio and distance <= self.center_gate

    def step(self, frame, stamp, detections, *, detector_updated=True, same_frame_correction=False,
             camera_transform=None, camera_motion_reliable=None, camera_calibration_changed=False,
             visual_observation=None, visual_quality=0., visual_scale_ratio=None,
             color_observation=None, color_quality=0., static_reference=None):
        self.diagnostics = dict(detector_updated=bool(detector_updated), candidates=len(detections),
            rejected={}, local_tracker="not_run", camera_motion_available=False,
            branches={name:dict(accepted=False,score=0.,reason='not_run')
                      for name in ('feature','flow','color','detector')},
            selected_target_flow=dict(valid=False,reason='not_eligible'),
            frame_gap_sec=max(0., stamp-self.stamp) if math.isfinite(stamp) else None)
        previous_result=self.result
        correction = same_frame_correction and detector_updated and stamp == self.stamp
        if not math.isfinite(stamp) or stamp < self.stamp or (stamp == self.stamp and not correction):
            # Duplicate captures must not advance reacquisition or the tracker.
            return FusionResult(None, self.public_id, self.class_id, 0., "lost", "non_increasing_capture", self.last_detection_stamp, self.local_id)
        if self.box is None:
            return self._result(None, "idle", "no_selection", 0.)
        dt = stamp - self.stamp
        if (not correction and self._detector_reference_box is not None
                and CameraMotion.validate(camera_transform,frame.shape)):
            reference=self._detector_reference_box
            points=np.float32([center(reference),center(reference)+self._detector_velocity*.01])
            mapped=cv2.perspectiveTransform(points.reshape(-1,1,2),camera_transform).reshape(-1,2)
            self._detector_velocity=(mapped[1]-mapped[0])/.01
            self._detector_reference_box,self._detector_reference_support=transform_window(
                reference,camera_transform,self._detector_reference_support)
            if self._detector_anchor_box is not None:
                anchor_box,anchor_support=transform_window(self._detector_anchor_box,
                    camera_transform,self._detector_anchor_support)
                if anchor_box is not None:
                    self._detector_anchor_area*=anchor_box[2]*anchor_box[3]/max(
                        1.,self._detector_anchor_box[2]*self._detector_anchor_box[3])
                    self._detector_anchor_box,self._detector_anchor_support=anchor_box,anchor_support
            if self._strong_pending_box is not None:
                self._strong_pending_box=transform_window(self._strong_pending_box,camera_transform)[0]
        if detector_updated and self.config.get('strong_takeover_safety_enabled', False):
            # Filter once for BOTH strong and ordinary association. A denied raw
            # selected proposal cannot re-enter below as an ordinary match.
            from .takeover_safety import rejection_reason
            allowed = []
            vetoes = Counter()
            safety_reference = self._detector_reference_box or self.box
            safety_prediction = list(safety_reference)
            safety_elapsed = max(0., stamp-(self._detector_reference_stamp
                if self._detector_reference_box is not None else
                (self.last_detection_stamp or self.selection_stamp)))
            if self._detector_reference_box is not None:
                safety_prediction[:2] = np.asarray(safety_prediction[:2]) + self._detector_velocity * min(
                    safety_elapsed, self.motion_horizon)
            strong_peers = [c for c in detections
                if c.selected_public_id == self.public_id
                and math.isfinite(c.confidence) and c.confidence >= .50
                and valid_box(c.box, frame.shape)]
            for candidate in detections:
                reason = None
                if (candidate.selected_public_id == self.public_id
                        and math.isfinite(candidate.confidence)
                        and valid_box(candidate.box, frame.shape)):
                    reason = rejection_reason(candidate, strong_peers, stamp=stamp,
                        last_detection_stamp=self.last_detection_stamp or self.selection_stamp,
                        independent_history=self._detector_reference_box is not None,
                        reference=safety_reference, anchor_area=self._detector_anchor_area,
                        predicted_reference=safety_prediction,
                        public_id=self.public_id, class_groups=self.class_groups)
                if reason == 'transported_detection_not_current':
                    if correction and not self.result.observed:
                        reason = 'stale_correction_after_loss'
                    else:
                        # Old evidence has no strong authority. The ordinary
                        # projected-candidate path requires current independent
                        # KCF/visual witnesses, motion, geometry and appearance.
                        candidate = replace(candidate, camera_projected=True,
                            local_id=self.local_id, stable=self.local_id >= 0)
                        reason = None
                if reason is None:
                    allowed.append(candidate)
                else:
                    vetoes[reason] += 1
            self.diagnostics['strong_takeover_vetoes'] = dict(vetoes)
            detections = allowed
        if detector_updated:
            takeover=self._strong_takeover(frame,stamp,detections,camera_transform)
            if takeover is not None:
                self.stamp=stamp
                return takeover
        if not correction and self._deferred_local_seed is not None:
            seed_frame,seed_box,seed_stamp=self._deferred_local_seed
            self._deferred_local_seed=None
            if 0<stamp-seed_stamp<=self.max_gap:
                self.tracker.initialize(seed_frame,seed_box)
                self.diagnostics['deferred_local_seed_initialized']=True
        revalidate_visual=bool(self.config.get("joint_current_visual_witnesses",False)
            and not self.result.observed and 0<stamp-self.last_observation_stamp<=.5
            and self._visual_hypothesis_stamp==self.stamp and self._visual_hypothesis_box is not None)
        target_flow_box=None
        target_flow=dict(valid=False,reason='not_eligible')
        if (self.config.get('continuous_selected_flow',False) and not correction
                and ((self.result.observed and self.last_observation_stamp==self.stamp)
                     or (self._visual_bootstrap_pending and stamp-self.selection_stamp<=3.) or revalidate_visual)
                and 0<dt<=self.max_gap and self.pending_id is None):
            flow_seed=None
            if self.config.get("selected_flow_foreground_only",False):
                flow_seed=np.asarray(camera_transform,dtype=float).copy() if CameraMotion.validate(camera_transform,frame.shape) else np.eye(3)
                flow_seed[:2,2]+=self.velocity*dt
            flow_started=time.perf_counter()
            target_flow_box,target_flow=selected_flow(self._previous_frame,frame,self._visual_hypothesis_box if revalidate_visual else self.box,local_crop=bool(self.config.get("selected_flow_local_crop",self.small_lock.enabled)),
                foreground_only=bool(self.config.get("selected_flow_foreground_only",False)),initial_transform=flow_seed)
            self.diagnostics["selected_lk_ms"]=(time.perf_counter()-flow_started)*1000.
        strong_target_flow=bool(target_flow.get("valid") and target_flow.get("points",0)>=6
            and target_flow.get("inlier_ratio",0.)>=.90 and target_flow.get("fb_error_median_px",float("inf"))<=.35)
        self.diagnostics['selected_target_flow']=target_flow
        self.stamp = stamp
        if frame.shape[:2] != self.shape:
            self.tracker.reset()
            return self._result(None, "lost", "image_dimensions_changed", 0.)
        absence = stamp - self.last_observation_stamp
        self._long_recovery = absence > self.reacquire_hold
        if absence > self.identity_ttl:
            self.tracker.reset()
            self.pending_count, self.pending_id = 0, None
            result = self._result(None, "lost", "identity_memory_expired", 0.)
            self.identity_archive = None
            self.anchor = None
            return result
        motion_valid = CameraMotion.validate(camera_transform, frame.shape)
        motion_reliable = motion_valid and (camera_motion_reliable is None or bool(camera_motion_reliable))
        self.diagnostics.update(camera_calibration_changed=bool(camera_calibration_changed),
            camera_motion_available=bool(motion_valid),
            camera_motion_reliable=bool(motion_reliable),
            recovery_phase="identity_memory" if self._long_recovery else "short",
            identity_memory_capacity=self.identity_capacity,
            identity_memory_remaining_sec=max(0., self.identity_ttl-absence))
        initial_pose_bootstrap=(self.small_lock.armed and not self.small_lock.ever_confirmed
            and self.small_lock.count==0 and self.small_lock.first_stamp is None
            and stamp-self.selection_stamp<=2. and motion_reliable)
        if not correction and (not motion_reliable or (dt > self.max_gap and not initial_pose_bootstrap)):
            self._motion_continuous = False
        self.diagnostics['camera_motion_continuous'] = self._motion_continuous
        compensation_transform=None
        if not correction and motion_valid:
            transformed,projected = transform_window(self.box,camera_transform,self._camera_support)
            if transformed is not None:
                compensation_transform=camera_transform
                # Velocity is target residual motion; camera displacement must
                # not be learned as a target acceleration.
                points = np.float32([center(self.box), center(self.box)+self.velocity*.01])
                rotated = cv2.perspectiveTransform(points.reshape(-1, 1, 2), camera_transform).reshape(-1, 2)
                self.velocity = (rotated[1]-rotated[0])/.01
                old_area=max(1.,self.box[2]*self.box[3])
                self.anchor_area*=transformed[2]*transformed[3]/old_area
                self.box=transformed;self._camera_support=projected
                if self.pending_box is not None:
                    self.pending_box = transform_window(self.pending_box,camera_transform)[0]
        if not correction and self._previous_frame is not None and hasattr(self.tracker,"compensate_motion"):
            search_transform=compensation_transform
            hint_used=False
            if (self.config.get("target_motion_search_hint",False) and strong_target_flow
                    and previous_result.observed and 0<dt<=min(.25,self.max_gap)
                    and valid_box(target_flow_box,frame.shape)):
                # LK predicts where KCF should SEARCH. It does not create an
                # observation: the actual current-image KCF response and the
                # immutable appearance anchor still gate the result below.
                tracker_box=getattr(self.tracker,"_last_bbox",None)
                projected=transform_window(tracker_box,search_transform)[0] if tracker_box is not None and search_transform is not None else tracker_box
                if projected is not None:
                    delta=center(target_flow_box)-center(projected)
                    if np.linalg.norm(delta)<=max(projected[2:]):
                        shift=np.eye(3);shift[:2,2]=delta
                        search_transform=shift@(search_transform if search_transform is not None else np.eye(3))
                        hint_used=True
            self.diagnostics["target_motion_search_hint"]=dict(used=hint_used)
            if search_transform is not None:
                changed=self.tracker.compensate_motion(self._previous_frame,search_transform,force_scale=True) if camera_calibration_changed else self.tracker.compensate_motion(self._previous_frame,search_transform)
                if camera_calibration_changed:self.diagnostics["lens_template_refreshed"]=bool(changed)
        if not correction:
            self._previous_frame = frame
        predicted = list(self.box)
        # Do not extrapolate a stale pixel velocity for a minute or grow gates
        # without limit. Camera displacement is handled separately above.
        predicted[:2] = np.asarray(predicted[:2]) + self.velocity * min(absence, self.motion_horizon)
        predicted = tuple(predicted)
        # No advancing a local tracker across a large dropped-frame interval.
        if dt > self.max_gap or self._long_recovery:
            self.tracker.reset()
            if dt > self.max_gap:
                self.pending_count = 0
        self.diagnostics['local_tracker_executed'] = not correction and not self._long_recovery
        local = None
        if self.diagnostics['local_tracker_executed']:
            local_started=time.perf_counter()
            local=self.tracker.update(frame)
            self.diagnostics['local_tracker_ms']=(time.perf_counter()-local_started)*1000.
        self.diagnostics['local_tracker'] = ("cached_same_capture" if correction else
            "short_tracking_ended" if self._long_recovery else
            "frame_gap_reset" if dt > self.max_gap else
            "valid" if local is not None and local.valid else
            str(getattr(local, "reason", "local_tracker_failed")))
        moving_box=None;moving_status=dict(valid=False,reason='disabled')
        if not correction and self.moving_pixel_lock.enabled and not self._long_recovery:
            proposal=local.bbox if local is not None and local.valid else predicted if absence<=self.max_gap else None
            if proposal is not None and valid_box(proposal,frame.shape) and self._compatible(proposal,predicted):
                rx,ry,rw,rh=self.moving_pixel_lock.reference_box or proposal
                px,py,pw,ph=proposal
                transform=np.array([[pw/rw,0.,px-rx*pw/rw],[0.,ph/rh,py-ry*ph/rh],[0.,0.,1.]])
                moving_box,moving_status=self.moving_pixel_lock.update(frame,proposal,stamp,image_reference_transform=transform)
                if moving_box is not None and (not valid_box(moving_box,frame.shape) or not self._compatible(moving_box,predicted)):
                    moving_box=None;moving_status.update(valid=False,reason='pixel_proposal_geometry_rejected')
            else:moving_status=dict(valid=False,reason='no_bounded_local_proposal')
        self.diagnostics['moving_small_pixel_witness']=moving_status
        small_geometry=motion_reliable and self._motion_continuous
        small_transform=camera_transform
        if static_reference is not None:
            small_transform=static_reference.get('matrix')
            translation=static_reference.get('translation_m')
            # This maps the INITIAL capture to now, not adjacent frames. A
            # legitimate accumulated pan can exceed the per-frame full-image
            # GMC bounds; the immutable target ROI is validated below instead.
            small_geometry=(np.shape(small_transform)==(3,3) and np.isfinite(small_transform).all()
                and abs(np.linalg.det(small_transform))>1e-10)
            if translation is not None and translation>static_reference['maximum_translation_m']:
                self.small_lock.armed=False
                small_box,small_status=None,dict(valid=False,reason='observer_left_hover_scope',translation_m=translation)
            elif not small_geometry:
                # Do not substitute a ground/background homography for an
                # airborne target's immutable camera-pose reference.
                self.small_lock.confirmed=False;self.small_lock.count=0;self.small_lock.first_stamp=None
                small_box,small_status=None,dict(valid=False,reason='calibrated_reference_unavailable')
            else:
                small_box,small_status=self.small_lock.update(frame,self.box,stamp,
                    geometry_supported=True,camera_transform=small_transform,absolute_reference=True)
                small_status.update(reference_source='capture_calibration_rotation',observer_translation_m=translation)
        else:
            small_box,small_status=(self.small_lock.update(frame,self.box if small_geometry else
                    local.bbox if local is not None and local.valid else None,stamp,
                    geometry_supported=small_geometry,camera_transform=small_transform)
                if not correction else (None,dict(valid=False,reason='same_capture_correction')))
        if small_box is not None and (not valid_box(small_box,frame.shape)
                or not self._compatible(small_box,predicted)
                or np.linalg.norm(center(small_box)-center(small_status.get('projected_reference_box',self.box)))>max(3.,.25*min(self.box[2:]))):
            self.small_lock.armed=False
            small_box=None
            small_status.update(valid=False,reason='stationary_geometry_disagreement')
        self.diagnostics['civil_static_small_lock']=small_status
        flow_quality = visual_quality
        feature_similarity = 0.
        reference_similarity=0.
        joint_feature=None
        joint_appearance_ok=False
        visual_box = self._visual_box if correction else local.bbox if local is not None else None
        visual_quality = self._visual_quality if correction else 0.
        local_score = float(getattr(local,'confidence',1.)) if local is not None else 1.
        if correction:
            pass
        elif local is not None and local.valid and math.isfinite(local_score) and local_score >= self.quality_gate and valid_box(visual_box, frame.shape) and self._compatible(visual_box, predicted):
            feature, visual_quality = appearance(frame, visual_box)
            similarity = float(np.dot(feature, self.anchor)) if self.anchor is not None else 0.
            feature_similarity = similarity
            joint_feature=feature
            adaptive=bool(self.config.get('adaptive_joint_appearance',False))
            reference_similarity=float(np.dot(feature,self._visual_reference)) if self._visual_reference is not None else similarity
            joint_appearance_ok=bool(similarity>=(.90 if adaptive else .95)
                and (reference_similarity>=.95 if adaptive else True))
            anchor_ratio = visual_box[2] * visual_box[3] / max(1., self.anchor_area)
            joint_quality=bool(self.config.get("joint_current_visual_witnesses",False) and strong_target_flow
                and joint_appearance_ok and local_score>=.75 and target_flow_box is not None
                and iou(visual_box,target_flow_box)>=.60 and self._compatible(visual_box,target_flow_box))
            if similarity < self.appearance_gate or (visual_quality < self.quality_gate and not joint_quality) or not 1. / self.area_ratio <= anchor_ratio <= self.area_ratio:
                visual_box = None
                self.diagnostics['local_tracker'] = ("appearance_rejected" if similarity < self.appearance_gate else
                    "quality_rejected" if visual_quality < self.quality_gate else "scale_rejected")
        else:
            visual_box = None
            if local is not None and local.valid:
                self.diagnostics['local_tracker'] = ('low_feature_response'
                    if not math.isfinite(local_score) or local_score < self.quality_gate else 'position_or_bounds_rejected')
        joint_current=bool(self.config.get("joint_current_visual_witnesses",False) and strong_target_flow
            and visual_box is not None and joint_appearance_ok and local_score>=.75
            and target_flow_box is not None and iou(visual_box,target_flow_box)>=.60
            and self._compatible(visual_box,target_flow_box))
        if self.config.get("joint_current_visual_witnesses",False) and not correction:
            # A single SOT response may seed an UNTRUSTED next-frame test;
            # it cannot publish or renew observation time on its own. Requiring
            # the already fused box here would prevent low-texture targets
            # from ever collecting the second independent pixel witness.
            hypothesis=getattr(local,'bbox',None)
            self._visual_hypothesis_box=(tuple(hypothesis) if local is not None and local.valid
                and joint_appearance_ok and local_score>=.75
                and valid_box(hypothesis,frame.shape) and self._compatible(hypothesis,predicted) else None)
            self._visual_hypothesis_stamp=stamp if self._visual_hypothesis_box is not None else None
            self._visual_revalidation_count=(0 if previous_result.observed else
                self._visual_revalidation_count+1 if joint_current else 0)
        if self._visual_bootstrap_pending and not correction:
            self._visual_bootstrap_count=self._visual_bootstrap_count+1 if joint_current else 0
            if self._visual_bootstrap_count>=3:self._visual_bootstrap_pending=False
            elif joint_current:
                self._camera_support=correct_window_support(self._camera_support,self.box,visual_box)
                self.box=tuple(visual_box)
        joint_confirmed=bool(joint_current and not self._visual_bootstrap_pending
            and (previous_result.observed or self._visual_revalidation_count>=2))
        self.diagnostics["joint_current_visual_witnesses"]=dict(valid=joint_current,confirmed=joint_confirmed,
            identity_reference_similarity=feature_similarity,short_reference_similarity=reference_similarity,
            revalidation_frames=self._visual_revalidation_count,
            bootstrap_pending=self._visual_bootstrap_pending,confirmations=self._visual_bootstrap_count)
        feature_score = min(visual_quality,feature_similarity,
                            float(getattr(local,'confidence',1.))) if visual_box is not None else 0.
        self.diagnostics['branch_proposals'] = dict(
            coordinate_format='xywh_original_capture_pixels',
            feature=(list(local.bbox) if local is not None and valid_box(local.bbox, frame.shape) else None),
            selected_flow=(list(target_flow_box) if valid_box(target_flow_box, frame.shape) else None),
            associated_flow=(list(visual_observation.box) if visual_observation is not None
                             and valid_box(visual_observation.box, frame.shape) else None),
            color=(list(color_observation.box) if color_observation is not None
                   and valid_box(color_observation.box, frame.shape) else None),
            prediction=list(predicted) if valid_box(predicted, frame.shape) else None)
        self.diagnostics['branches']['feature'] = dict(accepted=visual_box is not None and not correction,
            score=max(0.,min(1.,feature_score)),tracking_response_score=local_score,appearance_similarity=feature_similarity,
            texture_quality=visual_quality,reason=self.diagnostics['local_tracker'])
        # The per-target LK worker already checked forward/backward support
        # and coherent motion. Use only this bound identity's CURRENT visual
        # observation, never a new YOLO box disguised as independent evidence.
        # It cannot rebind an ID, renew detector time, or extend the hold budget.
        flow = visual_observation
        flow_valid = False
        if flow is not None:
            measured = flow.detector_stamp
            flow_valid = (not self._long_recovery and dt <= self.max_gap
                and flow.local_id == self.local_id and self.local_id >= 0 and flow.stable
                and measured is not None and math.isfinite(measured) and 0 <= stamp-measured <= self.max_gap
                and math.isfinite(flow_quality) and flow_quality >= .60
                and self._class_compatible(flow.class_id) and valid_box(flow.box, frame.shape)
                and self._compatible(flow.box, predicted))
            if flow_valid:
                feature, quality = appearance(frame, flow.box)
                similarity = float(np.dot(feature, self.anchor)) if self.anchor is not None else 0.
                ratio = flow.box[2]*flow.box[3]/max(1., self.anchor_area)
                # Rotation of a long vehicle enlarges the AABB without a real
                # scale change. Prefer the validated LK footprint's bounded
                # area ratio; never relax the same per-frame position gate.
                if visual_scale_ratio is not None:
                    ratio = visual_scale_ratio if math.isfinite(visual_scale_ratio) else float('inf')
                flow_valid = (quality >= self.quality_gate and similarity >= self.appearance_gate
                    and 1./self.area_ratio <= ratio <= self.area_ratio)
                if flow_valid:
                    if visual_box is not None and iou(visual_box, flow.box) < .15:
                        visual_box = None
                        self.diagnostics['local_tracker'] = 'conflicting_visual_witnesses'
                    else:
                        visual_box, visual_quality = flow.box, min(flow_quality, quality)
        self.diagnostics['lk_selected_observation'] = bool(flow_valid and visual_box is not None)
        self.diagnostics['branches']['flow'] = dict(accepted=bool(flow_valid and visual_box is not None),
            score=max(0.,min(1.,float(flow_quality))) if flow_valid and visual_box is not None and math.isfinite(flow_quality) else 0.,
            input_score=float(flow_quality) if math.isfinite(flow_quality) else 0.,
            reason=('current_same_identity_flow' if flow_valid and visual_box is not None else
                    'conflicting_visual_witnesses' if flow_valid else 'absent_or_failed_flow_gates'))

        # ROI-learned colour is an independent cue from KCF/CSRT and LK. It
        # may corroborate their location, or maintain the same bounded track
        # briefly when both feature cues fail. It cannot change the target ID,
        # class, detector timestamp, or bypass the appearance/position gates.
        color_valid = False
        color_box = None
        color_reliability = 0.
        if color_observation is not None:
            color_box = color_observation.box
            color_valid = (not self._long_recovery and dt <= self.max_gap
                and color_observation.local_id == self.local_id and self.local_id >= 0
                and color_observation.stable and self._class_compatible(color_observation.class_id)
                and math.isfinite(color_quality) and color_quality >= .05
                and valid_box(color_box, frame.shape) and self._compatible(color_box, predicted))
            if color_valid:
                feature, quality = appearance(frame, color_box)
                similarity = float(np.dot(feature, self.anchor)) if self.anchor is not None else 0.
                color_valid = quality >= self.quality_gate and similarity >= max(.55, self.appearance_gate - .10)
                self.diagnostics['color_appearance'] = similarity
                self.diagnostics['color_quality'] = quality
                if color_valid:color_reliability = min(color_quality,quality,similarity)
        # Fuse color only when it spatially corroborates a different witness.
        # If KCF and LK are unavailable, require a close match to the bounded
        # prediction and strong appearance before accepting color as visual.
        if color_valid:
            if visual_box is not None and iou(color_box, visual_box) >= .15:
                color_weight = .20*color_reliability
                visual_box = tuple((1.-color_weight)*np.asarray(visual_box) + color_weight*np.asarray(color_box))
                visual_quality = max(visual_quality, min(.65, color_reliability))
                self.diagnostics['color_blend_weight'] = color_weight
                self.diagnostics['color_selected_observation'] = True
                self.diagnostics['color_support'] = 'local_visual_agreement'
            elif visual_box is None and iou(color_box, predicted) >= .30:
                visual_box = color_box
                visual_quality = min(.65, color_reliability)
                self.diagnostics['color_selected_observation'] = True
                self.diagnostics['color_support'] = 'bounded_prediction_agreement'
            else:
                self.diagnostics['color_selected_observation'] = False
                self.diagnostics['color_support'] = 'conflict_or_low_overlap'
        else:
            self.diagnostics['color_selected_observation'] = False
            self.diagnostics['color_support'] = 'invalid_or_appearance_rejected'
        self.diagnostics['branches']['color'] = dict(
            accepted=bool(self.diagnostics['color_selected_observation']),score=float(color_reliability),
            input_score=float(color_quality) if math.isfinite(color_quality) else 0.,
            reason=self.diagnostics['color_support'],blend_weight=self.diagnostics.get('color_blend_weight',0.))
        self._visual_box, self._visual_quality = visual_box, visual_quality

        ranked = []
        rejected = Counter()
        for candidate in detections:
            raw_selected = (self.public_id >= 0 and candidate.selected_public_id == self.public_id)
            if candidate.selected_public_id >= 0 and not raw_selected:
                rejected["different_selection"] += 1
                continue
            if self.small_lock.applicable and (small_box is None or iou(candidate.box,small_box)<.4):
                rejected['unverified_small_target_pixels'] += 1
                continue
            measured = stamp if candidate.detector_stamp is None else candidate.detector_stamp
            if not detector_updated or not math.isfinite(measured) or measured > stamp or measured <= self.last_detection_stamp:
                rejected['non_fresh_detector'] += 1
                continue
            if not (math.isfinite(candidate.confidence) and self.min_maintenance_conf <= candidate.confidence <= 1):
                rejected['confidence'] += 1
                continue
            # Weak detections only maintain an already bound, currently trusted
            # track when the independent local box agrees. Never recover an ID.
            if candidate.confidence < self.min_detector_conf and not (
                    (raw_selected or candidate.local_id == self.local_id) and self.local_id >= 0
                    and self.public_id >= 0 and self.class_id >= 0
                    and self.result.source not in ("lost", "ambiguous", "idle")
                    and self.pending_count == 0
                    and self._class_compatible(candidate.class_id)
                    and iou(predicted, candidate.box) >= self.maintenance_min_iou
                    and .60 <= candidate.box[2] * candidate.box[3] / max(1., predicted[2] * predicted[3]) <= 1.60):
                rejected['weak_identity_evidence'] += 1
                continue
            if (candidate.local_id < 0 or not candidate.stable) and not raw_selected:
                rejected['unstable_identity'] += 1
                continue
            if not valid_box(candidate.box, frame.shape):
                rejected['box_bounds'] += 1
                continue
            if not self._class_compatible(candidate.class_id):
                rejected['class'] += 1
                continue
            if candidate.camera_projected:
                # A real, manager-accepted old detector observation may be
                # handed to CURRENT pixels using calibrated camera geometry.
                # It cannot recover/rebind an ID or replace failed local vision.
                local_box=local.bbox if local is not None and local.valid else None
                if (correction or not self.result.observed or self.pending_count or self._long_recovery
                        or candidate.local_id!=self.local_id or not motion_reliable
                        or local_box is None or feature_score<.85 or visual_box is None
                        or iou(local_box,candidate.box)<.60 or iou(visual_box,candidate.box)<.60):
                    rejected['camera_projection_lacks_current_witness']+=1
                    continue
            if not self._compatible(candidate.box, predicted):
                rejected['position_or_size'] += 1
                detail=self.diagnostics.setdefault('rejected_geometry',[])
                if len(detail)<4:
                    detail.append(dict(box=list(candidate.box),reference=list(predicted),
                        area_ratio=float(candidate.box[2]*candidate.box[3]/max(1.,predicted[2]*predicted[3])),
                        distance_norm=float(np.linalg.norm(center(candidate.box)-center(predicted))/
                            max(8.,math.hypot(predicted[2],predicted[3])))))
                continue
            feature, quality = appearance(frame, candidate.box)
            sim = float(np.dot(feature, self.anchor)) if self.anchor is not None else 0.
            # Same BoT-SORT ID is supporting evidence, never a bypass of gates.
            quality_gate = max(self.quality_gate, self.long_quality_gate) if self._long_recovery else self.quality_gate
            appearance_gate = max(self.appearance_gate, self.long_appearance_gate) if self._long_recovery else self.appearance_gate
            if quality < quality_gate or sim < appearance_gate:
                rejected['quality' if quality < quality_gate else 'appearance'] += 1
                continue
            if self._long_recovery and self.long_motion_required and not self._motion_continuous:
                rejected['camera_motion_chain_untrusted'] += 1
                continue
            proximity = max(0., 1. - np.linalg.norm(center(candidate.box) - center(predicted)) / max(8., math.hypot(predicted[2], predicted[3])))
            score = .45 * proximity + .25 * iou(candidate.box, predicted) + .30 * sim
            if self._long_recovery and score < self.long_score_gate:
                rejected['long_identity_score'] += 1
                continue
            if raw_selected:
                candidate = replace(candidate, local_id=self.local_id if self.local_id >= 0 else self.public_id,
                                    stable=True)
            ranked.append((float(score), candidate, quality))
        if self.config.get('strong_detector_takeover_enabled',False) and len(ranked)>1:
            ordered=sorted(ranked,key=lambda item:(iou(item[1].box,predicted),item[1].confidence),reverse=True)
            unique=[]
            for item in ordered:
                candidate=item[1]
                duplicate=False
                for prior in unique:
                    other=prior[1]
                    if candidate.selected_public_id!=self.public_id or other.selected_public_id!=self.public_id:continue
                    if candidate.class_id!=other.class_id and not any(candidate.class_id in group and other.class_id in group for group in self.class_groups):continue
                    overlap=iou(candidate.box,other.box)
                    intersection=overlap*(candidate.box[2]*candidate.box[3]+other.box[2]*other.box[3])/(1.+overlap)
                    contained=intersection/max(1.,min(candidate.box[2]*candidate.box[3],other.box[2]*other.box[3]))>=.90
                    close=np.linalg.norm(center(candidate.box)-center(other.box))<=.5*min(math.hypot(*candidate.box[2:]),math.hypot(*other.box[2:]))
                    if overlap>=.50 or contained and close:
                        duplicate=True;break
                if not duplicate:unique.append(item)
            self.diagnostics['local_duplicate_candidates']=len(ranked)-len(unique)
            ranked=unique
        if (len(ranked) > 1 and not correction and strong_target_flow
                and visual_box is not None and target_flow_box is not None
                and self.diagnostics['branches']['feature']['accepted']
                and iou(visual_box, target_flow_box) >= .60):
            supported = [item for item in ranked
                if item[1].selected_public_id == self.public_id
                and iou(item[1].box, visual_box) >= .60
                and iou(item[1].box, target_flow_box) >= .60]
            if len(supported) == 1:
                self.diagnostics['current_pixels_isolated_candidate'] = True
                ranked = supported
        ranked.sort(key=lambda item: item[0], reverse=True)
        self.diagnostics.update(rejected=dict(rejected), accepted_candidates=len(ranked),
            no_detector_candidates=bool(detector_updated and not detections),
            best_score=ranked[0][0] if ranked else None,
            second_score=ranked[1][0] if len(ranked) > 1 else None)
        self.diagnostics['branch_proposals']['detector_accepted_candidates'] = [
            dict(box=list(candidate.box), local_id=int(candidate.local_id),
                 class_id=int(candidate.class_id), confidence=float(candidate.confidence))
            for _, candidate, _ in ranked]
        self.diagnostics['branches']['detector'] = dict(accepted=bool(ranked),
            score=float(ranked[0][1].confidence) if ranked else 0.,
            association_score=float(ranked[0][0]) if ranked else 0.,
            reason='fresh_identity_and_appearance_gates' if ranked else 'absent_or_failed_detector_gates')
        if len(ranked) > 1 and ranked[0][0] - ranked[1][0] < self.margin:
            scoped_competitors = all(candidate.selected_public_id == self.public_id
                for _, candidate, _ in ranked)
            current_pixels = (not correction and strong_target_flow and visual_box is not None
                and target_flow_box is not None
                and self.diagnostics['branches']['feature']['accepted']
                and iou(visual_box, target_flow_box) >= .60)
            if scoped_competitors and current_pixels:
                # Ambiguous detector evidence cannot override independent
                # current KCF and verified target LK. Discard every weak
                # proposal; the ordinary visual gates below still apply.
                ranked = []
                self.diagnostics['detector_competitors_discarded'] = True
                self.diagnostics['accepted_candidates'] = 0
                self.diagnostics['branches']['detector'].update(accepted=False,
                    reason='ambiguous_detector_evidence_discarded')
            else:
                self.pending_count = 0
                self.pending_id = None
                return self._result(None, "ambiguous", "competing_detections", 0.)
        match = ranked[0][1] if ranked else None
        if match is not None and self.local_id >= 0 and match.local_id != self.local_id and any(
                c.local_id == self.local_id for c in detections):
            self.pending_count = 0
            self.pending_id = None
            return self._result(None, "ambiguous", "old_and_new_tracks_simultaneously_visible", 0.)
        # One failed visual frame is not an identity change. A fresh, strongly
        # gated SAME-ID detector observation may resume short continuity
        # immediately. New IDs, prolonged loss and competing hypotheses still
        # need the full consecutive-confirmation procedure.
        recent_same_identity = (match is not None and match.local_id == self.local_id
            and match.confidence >= self.min_detector_conf and ranked[0][0] >= .65
            and iou(match.box, predicted) >= .30
            and absence <= self.max_gap
            and stamp-self.last_detection_stamp <= self.max_gap
            and self.result.source != "ambiguous")
        rebound = match is not None and (self.local_id < 0 or match.local_id != self.local_id or self.pending_count > 0
                                        or self._long_recovery or dt > self.max_gap
                                        or (self.config.get('strong_takeover_safety_enabled', False)
                                            and stamp-(self.last_detection_stamp or self.selection_stamp) > 1.0)
                                        or (self.result.source in ("lost", "ambiguous", "prediction") and not recent_same_identity))
        if rebound:
            measured = stamp if match.detector_stamp is None else match.detector_stamp
            # Same backend ID is not sufficient if consecutive candidate
            # positions jump. Validate its continuation in the current camera
            # plane before accepting the confirmation sequence.
            pending_jump = (self.pending_id == match.local_id and self.pending_box is not None
                and np.linalg.norm(center(match.box)-center(self.pending_box)) >
                2. + self.center_gate * math.hypot(match.box[2], match.box[3]) *
                min(1., max(0., measured-self.pending_stamp)/self.max_gap))
            if self.pending_id != match.local_id or measured - self.pending_stamp > self.max_gap or dt > self.max_gap:
                self.pending_count = 0
            if pending_jump:
                self.pending_count = 0
                self.diagnostics['pending_motion_rejected'] = True
            self.pending_id = match.local_id
            if measured > self.pending_stamp:
                self.pending_count += 1
                self.pending_stamp = measured
                self.pending_box = match.box
            if self.pending_count < self.confirmations:
                # Position continuity and identity rebinding are separate. A
                # current, independently gated local observation can maintain
                # the OLD public/local identity while the new ID is pending.
                # Do not refresh detector time, appearance, or its hold budget.
                if (visual_box is not None
                        and stamp - (self.last_detection_stamp or self.selection_stamp) <= self.visual_hold
                        and iou(visual_box, match.box) >= .30
                        and self._compatible(visual_box, match.box)):
                    return self._commit_observation(
                        visual_box, stamp, "visual", "reacquisition_confirmation_visual_hold",
                        min(.65, visual_quality, self.last_confidence))
                return self._result(None, "ambiguous", "reacquisition_confirmation", 0.)
        elif match is None and (detector_updated or stamp - self.pending_stamp > self.max_gap):
            self.pending_id = None
            self.pending_count = 0

        if match is not None:
            # Two witnesses must agree before coordinates are blended.
            visual_agrees = (visual_box is not None and iou(visual_box, match.box) >= .30
                             and self._compatible(visual_box, match.box))
            color_agrees = (color_valid and iou(color_box, match.box) >= .30
                            and self._compatible(color_box, match.box))
            agree = visual_agrees or color_agrees
            if agree:
                evidence_quality = visual_quality if visual_agrees else min(.65, color_reliability)
                evidence_box = visual_box if visual_agrees else color_box
                weight = max(.5, self.detector_weight * match.confidence / max(.01, self.detector_weight * match.confidence + (1. - self.detector_weight) * evidence_quality))
                # Do not pull a strong, independently validated detector box
                # backwards by KCF's small temporal lag. Fusion is evidence
                # arbitration, not an obligation to average every coordinate.
                if match.camera_projected:
                    # Projection verifies a real detector's identity/time only;
                    # coordinates remain the independently validated visual box.
                    fused,source=visual_box,"visual"
                    self.diagnostics['camera_projection_verified']=True
                else:
                    fused = match.box if match.confidence >= self.detector_authority else tuple(
                        weight * np.asarray(match.box) + (1. - weight) * np.asarray(evidence_box))
                    source = "fused"
            else:
                fused, source = match.box, "yolo"
            # Rescale/reinitialize only for drift, failure, or confirmed new ID.
            # KCF's old template can fail abruptly during a turn even while
            # YOLO and LK agree. Refresh from a trusted CURRENT detector at a
            # bounded cadence, not on every frame or on predictions.
            refresh = (agree and stamp-self.last_local_seed_stamp >= self.local_refresh_sec
                and match.confidence >= self.min_detector_conf
                and (match.detector_stamp is None or match.detector_stamp == stamp))
            if not agree or rebound or refresh:
                self.tracker.initialize(frame, match.box)
                self.last_local_seed_stamp = stamp
                self.diagnostics['local_reinitialized'] = True
            if rebound:
                # The old derivative is no longer valid after an identity
                # rebind. Bootstrap velocity from subsequent fresh evidence.
                self.velocity[:] = 0.
                self._visual_box, self._visual_quality = None, 0.
                self._previous_frame = frame
                self.diagnostics.update(identity_rebound=True, previous_local_id=self.local_id,
                                        rebound_box=tuple(match.box))
            self.local_id = match.local_id
            self.last_detector_candidate = match
            if self.class_id < 0 or (self.config.get('follow_upstream_class_consensus',False)
                    and match.detector_stamp is not None
                    and self.last_detection_stamp < match.detector_stamp <= stamp):
                self.class_id = match.class_id
            feature, quality = appearance(frame, match.box)
            measured = stamp if match.detector_stamp is None else match.detector_stamp
            if quality >= .65 and match.confidence >= self.min_detector_conf and measured == stamp:
                self.anchor = .95 * self.anchor + .05 * feature
                self.anchor /= max(1e-9, np.linalg.norm(self.anchor))
                self.anchor_area = .95 * self.anchor_area + .05 * match.box[2] * match.box[3]
            self.last_detection_stamp = measured
            if (self.config.get('strong_detector_takeover_enabled',False)
                    and match.selected_public_id==self.public_id):
                self._update_detector_reference(match.box,measured)
            self._visual_bootstrap_pending=False;self._visual_bootstrap_count=0
            if measured < stamp:
                source = "visual"
            confidence = match.confidence
            self.pending_count = 0
            self.pending_id = None
        elif correction and previous_result.observed and self.last_observation_stamp==stamp:
            # An unconfirmed delayed detector packet is not negative visual
            # evidence. Keep this capture's accepted result without another
            # tracker update, clock renewal or control measurement. Explicit
            # competing/ambiguous identities already returned above.
            self.diagnostics['kept_current_visual_on_unconfirmed_correction']=True
            return previous_result
        elif small_box is not None and not correction and self.pending_id is None and not self._long_recovery:
            fused,source=small_box,'visual'
            confidence=min(.65,float(small_status['score']))
            self.diagnostics['civil_static_small_lock_verified']=True
            if local is None or not local.valid:
                self.tracker.initialize(frame,small_box)
                self.last_local_seed_stamp=stamp
                self.diagnostics['civil_static_small_lock_reseeded_kcf']=True
        elif (moving_box is not None and not correction and self.pending_id is None and not self._long_recovery
                and (local is not None and local.valid or previous_result.observed and absence<=self.max_gap)):
            fused,source=moving_box,'visual'
            confidence=min(.65,float(moving_status['score']))
            self._visual_bootstrap_pending=False;self._visual_bootstrap_count=0
            self.diagnostics['moving_small_pixel_verified']=True
            # Align the next real KCF/CSRT update with the independently
            # verified pixels; do not train a template on an unverified guess.
            if local is None or not local.valid or iou(local.bbox,moving_box)<.95:
                self.tracker.initialize(frame,moving_box)
                self.last_local_seed_stamp=stamp
                self.diagnostics['moving_small_pixel_reseeded_local']=True
        elif visual_box is not None and not self.small_lock.applicable and not self._visual_bootstrap_pending and (not self.moving_pixel_lock.applicable) and (stamp - (self.last_detection_stamp or self.selection_stamp) <= self.visual_hold
                or (target_flow_box is not None and not correction and self.pending_id is None
                    and not self._long_recovery and (joint_confirmed if self.config.get("joint_current_visual_witnesses",False) else feature_score>=.85)
                    and iou(visual_box,target_flow_box)>=.60
                    and self._compatible(visual_box,target_flow_box))):
            self.diagnostics['continuous_selected_flow_verified']=bool(
                stamp-(self.last_detection_stamp or self.selection_stamp)>self.visual_hold)
            fused, source = visual_box, "visual"
            confidence = (min(.65,local_score,feature_similarity,float(target_flow["inlier_ratio"]),
                1.-float(target_flow["fb_error_median_px"])/.8) if joint_current else
                min(.65, visual_quality, self.last_confidence))
        else:
            age = stamp - self.last_observation_stamp
            if age <= self.predict_hold and valid_box(predicted, frame.shape):
                return self._result(predicted, "prediction", "no_trusted_visual_observation", min(.25, self.last_confidence))
            return self._result(None, "lost", "identity_memory_waiting" if self._long_recovery else "no_trusted_visual_observation", 0.)
        if (self.config.get('adaptive_joint_appearance',False) and joint_confirmed
                and not correction and joint_feature is not None):
            # Only mutually consistent CURRENT KCF+FB-LK pixels can update
            # this short reference. The identity anchor and detector clock
            # are not renewed; original similarity must remain >= .90.
            self._visual_reference=.9*self._visual_reference+.1*joint_feature
            self._visual_reference/=max(1e-9,np.linalg.norm(self._visual_reference))
            self.diagnostics['joint_appearance_adapted']=True
        return self._commit_observation(
            fused, stamp, source, "compatible_observations" if source == "fused" else source,
            confidence, update_velocity=not rebound)

    def _strong_takeover(self, frame, stamp, detections, camera_transform):
        if (not self.config.get('strong_detector_takeover_enabled', False)
                or frame.shape[:2] != self.shape):
            return None
        if self.config.get('strong_takeover_safety_enabled', False):
            # Only original current-capture measurements can take this bypass.
            # Long gaps retain the ordinary identity/confirmation path below.
            detections = [c for c in detections
                if not c.camera_projected and (c.detector_stamp is None or c.detector_stamp == stamp)
                and stamp - (self.last_detection_stamp or self.selection_stamp) <= 1.0]
            if not detections:
                return None
        independent=self._detector_reference_box is not None
        reference = self._detector_reference_box if independent else self.box
        if not independent and CameraMotion.validate(camera_transform, frame.shape):
            reference = transform_window(self.box, camera_transform, self._camera_support)[0]
        if reference is None:
            return None
        predicted = list(reference)
        observation_stamps=[c.detector_stamp if c.detector_stamp is not None else stamp for c in detections
            if c.selected_public_id==self.public_id and math.isfinite(c.confidence) and c.confidence>=.50
            and (c.detector_stamp is None or math.isfinite(c.detector_stamp))
            and self.last_detection_stamp < (c.detector_stamp if c.detector_stamp is not None else stamp) <= stamp]
        observation_stamp=max(observation_stamps) if observation_stamps else stamp
        elapsed=max(0.,observation_stamp-(self._detector_reference_stamp if independent else self.last_observation_stamp))
        if independent and elapsed>self.reacquire_hold:
            self.diagnostics['strong_takeover_requires_identity_memory']=True
            return None
        predicted[:2] = np.asarray(predicted[:2]) + (self._detector_velocity if independent else self.velocity) * min(
            elapsed, self.motion_horizon)
        ranked = []
        innovation_gate=3.+.5*math.hypot(*reference[2:])*min(elapsed,self.max_gap)/self.max_gap
        if self.config.get('adaptive_detector_motion_gate',False):
            speed=float(np.linalg.norm(self._detector_velocity if independent else self.velocity))
            innovation_gate+=min(.5*math.hypot(*reference[2:]),.25*speed*min(elapsed,self.max_gap))
        if elapsed>self.max_gap:
            diagonal=math.hypot(*reference[2:])
            innovation_gate+=.5*diagonal*(elapsed-self.max_gap)+.25*diagonal*min(elapsed,self.reacquire_hold)**2
        self.diagnostics.update(strong_detector_reference_box=list(reference),
            strong_detector_prediction=list(predicted),strong_detector_innovation_gate_px=float(innovation_gate))
        if self._detector_anchor_area is None:
            self._detector_anchor_area=float(self.anchor_area or reference[2]*reference[3])
            self._detector_anchor_box=tuple(reference)
            self._detector_anchor_support=None
        minimum_area=min(reference[2]*reference[3],self._detector_anchor_area)/self.area_ratio
        recovery_area=(math.sqrt(self._detector_anchor_area)*float(self.config.get('roi_crop_scale',3.))+2.)**2
        maximum_area=max(reference[2]*reference[3]*self.area_ratio,self._detector_anchor_area*self.area_ratio)
        if elapsed>self.max_gap:maximum_area=max(maximum_area,recovery_area)
        self.diagnostics['strong_detector_area_bounds']=[minimum_area,maximum_area]
        for candidate in detections:
            measured = stamp if candidate.detector_stamp is None else candidate.detector_stamp
            if (candidate.selected_public_id != self.public_id
                    or not math.isfinite(candidate.confidence) or candidate.confidence < .50
                    or not math.isfinite(measured) or not self.last_detection_stamp < measured <= stamp
                    or stamp-measured > self.max_gap
                    or not valid_box(candidate.box, frame.shape)
                    or not self._class_compatible(candidate.class_id)
                    or not minimum_area <= candidate.box[2]*candidate.box[3] <= maximum_area
                    or np.linalg.norm(center(candidate.box)-center(predicted)) > innovation_gate + (
                        .5*(abs(candidate.box[2]-reference[2])+abs(candidate.box[3]-reference[3]))
                        if iou(candidate.box,predicted)>=.35 else 0.)):
                continue
            similarity=0.
            if elapsed>self.max_gap:
                feature,_=appearance(frame,candidate.box)
                similarity=float(np.dot(feature,self.anchor)) if feature is not None and self.anchor is not None else 0.
            distance = np.linalg.norm(center(candidate.box)-center(predicted))/max(4.,innovation_gate)
            score = .6*iou(candidate.box,predicted)+.3*max(0.,1.-distance)+.1*candidate.confidence
            if elapsed>self.max_gap:
                distance=np.linalg.norm(center(candidate.box)-center(predicted))/max(8.,innovation_gate)
                score=.5*similarity+.35*max(0.,1.-distance)+.15*candidate.confidence
            ranked.append((score,candidate))
        ranked.sort(key=lambda item:item[0],reverse=True)
        kept=[]
        for item in ranked:
            box=item[1].box
            def duplicate(other):
                if item[1].class_id!=other[1].class_id and not any(item[1].class_id in group and other[1].class_id in group for group in self.class_groups):
                    return False
                other=other[1].box
                intersection=iou(box,other)*(box[2]*box[3]+other[2]*other[3])/(1.+iou(box,other))
                contained=intersection/max(1.,min(box[2]*box[3],other[2]*other[3])) >= .90
                close=np.linalg.norm(center(box)-center(other)) <= .5*min(math.hypot(*box[2:]),math.hypot(*other[2:]))
                return iou(box,other)>=.50 or contained and close
            if not any(duplicate(other) for other in kept):kept.append(item)
        self.diagnostics['strong_takeover_candidates']=len(ranked)
        self.diagnostics['strong_takeover_duplicates']=len(ranked)-len(kept)
        if elapsed>self.max_gap and len(kept)>1:
            self._strong_pending_count=0
            self.diagnostics['strong_recovery_multiple_identities']=True
            return self._result(None,'ambiguous','strong_recovery_multiple_identities',0.)
        if not kept or len(kept)>1 and kept[0][0]-kept[1][0]<self.margin:
            return None
        candidate=kept[0][1]
        measured=stamp if candidate.detector_stamp is None else candidate.detector_stamp
        fast_recovery=(elapsed>self.max_gap and candidate.confidence>=float(self.config.get('fast_reacquire_confidence',1.01))
            and len(kept)==1 and elapsed<=float(self.config.get('fast_reacquire_max_gap_sec',0.)))
        if fast_recovery:self.diagnostics['fast_single_frame_recovery']=True
        if elapsed>self.max_gap and not fast_recovery:
            gap=measured-self._strong_pending_stamp
            consistent=(self._strong_pending_box is not None and 0<=gap<=self.max_gap
                and np.linalg.norm(center(candidate.box)-center(self._strong_pending_box)) <=
                    3.+.5*math.hypot(*candidate.box[2:])+np.linalg.norm(self._detector_velocity)*gap)
            if not consistent:self._strong_pending_count=0
            if measured>self._strong_pending_stamp:
                self._strong_pending_count+=1
                self._strong_pending_stamp=measured
                self._strong_pending_box=candidate.box
            self.diagnostics['strong_recovery_confirmations']=self._strong_pending_count
            if self._strong_pending_count<self.confirmations:return None
        self._strong_pending_count=0
        candidate=replace(candidate,local_id=self.local_id if self.local_id>=0 else self.public_id,stable=True)
        saved_reference=self._detector_reference_box
        saved_support=self._detector_reference_support
        saved_stamp=self._detector_reference_stamp
        saved_velocity=self._detector_velocity.copy()
        self._update_detector_reference(candidate.box,measured)
        current_box=list(candidate.box)
        current_box[:2]=np.asarray(current_box[:2])+self._detector_velocity*min(stamp-measured,self.predict_hold)
        if not valid_box(current_box,frame.shape):
            self._detector_reference_box=saved_reference
            self._detector_reference_support=saved_support
            self._detector_reference_stamp=saved_stamp
            self._detector_velocity=saved_velocity
            return None
        candidate=replace(candidate,box=tuple(current_box))
        self.box=tuple(reference)
        self.local_id=candidate.local_id
        if self.class_id<0:self.class_id=candidate.class_id
        self.last_detector_candidate=candidate
        self.last_detection_stamp=measured
        self.pending_count=0;self.pending_id=None
        self._long_recovery=False
        self._visual_bootstrap_pending=False;self._visual_bootstrap_count=0
        self.tracker.reset()
        if self.config.get('defer_strong_local_seed',False):
            self._deferred_local_seed=(frame,candidate.box,stamp)
        else:
            self.tracker.initialize(frame,candidate.box)
        self.last_local_seed_stamp=stamp
        self._previous_frame=frame
        self._camera_support=None
        self._visual_box=candidate.box
        self.diagnostics.update(strong_detector_takeover=True,
            local_reinitialized=not self.config.get('defer_strong_local_seed',False),
            local_seed_deferred=bool(self.config.get('defer_strong_local_seed',False)),
            accepted_candidates=1,detector_capture_stamp=measured)
        self.diagnostics['branches']['detector']=dict(accepted=True,score=candidate.confidence,
            reason='strong_family_and_physical_reachability')
        self.velocity=self._detector_velocity.copy()
        return self._commit_observation(candidate.box,stamp,
            'yolo' if measured==stamp else 'visual','strong_detector_takeover',candidate.confidence,update_velocity=False)

    def _update_detector_reference(self,box,stamp):
        elapsed=stamp-self._detector_reference_stamp
        if self._detector_reference_box is not None and elapsed>0:
            reference=self._detector_reference_box
            innovation=center(box)-center(reference)-self._detector_velocity*elapsed
            uncertainty=.5*(abs(box[2]-reference[2])+abs(box[3]-reference[3]))
            magnitude=float(np.linalg.norm(innovation))
            residual=innovation*max(0.,magnitude-uncertainty)/max(1e-9,magnitude)
            smoothing=float(self.config.get('detector_velocity_time_constant_sec',0.))
            alpha=1.-math.exp(-elapsed/smoothing) if smoothing>0 else .4
            self._detector_velocity=self._detector_velocity+alpha*residual/elapsed
        elif self._detector_reference_box is None:
            self._detector_velocity=np.zeros(2)
        self._detector_reference_box=tuple(box)
        self._detector_reference_support=None
        self._detector_reference_stamp=stamp

    def _commit_observation(self, box, stamp, source, reason, confidence, update_velocity=True):
        observation_dt = stamp - self.last_observation_stamp
        if observation_dt > 0 and update_velocity:
            measured_velocity = (center(box) - center(self.box)) / observation_dt
            self.velocity = .6 * self.velocity + .4 * measured_velocity
        reset_support=self.diagnostics.get('local_reinitialized') or self.diagnostics.get('identity_rebound')
        self._camera_support=correct_window_support(None if reset_support else self._camera_support,self.box,box)
        self.box = tuple(box)
        self.last_observation_stamp = stamp
        self.last_confidence = confidence
        return self._result(self.box, source, reason)
