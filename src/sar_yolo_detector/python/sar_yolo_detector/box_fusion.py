"""Bounded, time-based YOLO/local-feature fusion; no ROS or model dependency.

Local tracker success is not a probability. Appearance and motion validate it.
Only compatible observations are blended; prediction is never an observation.
"""
from dataclasses import dataclass
from collections import Counter
import math
from typing import Optional, Tuple

import cv2
import numpy as np

from .manual_box_tracker import ManualBoxTracker
from .camera_motion import CameraMotion, transform_box


@dataclass(frozen=True)
class Candidate:
    box: Tuple[float, float, float, float]  # x, y, width, height, original pixels
    class_id: int
    confidence: float
    local_id: int
    stable: bool = True
    detector_stamp: Optional[float] = None


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
    # A turning rectangular vehicle occupies a changing fraction of its AABB.
    # Flat, low-saturation ground used to dominate the normalized histogram
    # and reject the same vehicle after a perfectly coherent rotation. Use
    # chromatic support if sufficient; preserve the original full-ROI fallback
    # for gray/white vehicles. This remains weak appearance evidence, not ReID.
    mask = cv2.inRange(hsv, np.array([0, 40, 20], np.uint8), np.array([179, 255, 255], np.uint8))
    if cv2.countNonZero(mask) < max(16, int(.10*mask.size)):
        mask = None
    hist = cv2.calcHist([hsv], [0, 1], mask, [16, 8], [0, 180, 0, 256]).ravel()
    return hist / max(1e-9, np.linalg.norm(hist)), quality


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
        self.min_maintenance_conf = self._unit("minimum_maintenance_confidence", .10)
        self.local_refresh_sec = self._positive("local_template_refresh_sec", .75)
        self.manual_seed_visual_without_yolo = bool(
            self.config.get("manual_seed_visual_without_yolo", True))
        if self.min_maintenance_conf > self.min_detector_conf:
            raise ValueError("Maintenance confidence must not exceed identity confidence")
        self.class_groups = self.config.get("class_groups", [[0, 1, 2, 3]])
        if not isinstance(self.class_groups, list) or any(not isinstance(group, list) or any(not isinstance(c, int) for c in group) for group in self.class_groups):
            raise ValueError("FusionTracker/class_groups must be a list of integer lists")
        options = dict(manual_config or {})
        options["tracker_type"] = "feature_tracker"
        self.tracker = tracker if tracker is not None else ManualBoxTracker(options)
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
        self.tracker.reset()
        self.public_id = -1
        self.local_id = -1
        self.class_id = -1
        self.box = None
        self.anchor = None
        self.anchor_area = None
        self.velocity = np.zeros(2)
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
        self.identity_archive = None
        self._motion_continuous = True
        self._long_recovery = False
        self.manual_seed_without_yolo = False
        self.manual_seed_region = None
        self.diagnostics = {}
        self.result = FusionResult(None, -1, -1, 0., "idle", "no_selection", 0., -1)

    def initialize(self, frame, box, stamp, public_id, candidate=None, manual_seed=False):
        if not math.isfinite(stamp) or stamp <= 0 or public_id < 0 or not valid_box(box, frame.shape):
            raise ValueError("Invalid original-frame selection")
        self.reset()
        self.manual_seed_without_yolo = bool(manual_seed and candidate is None)
        self.manual_seed_region = tuple(float(v) for v in box) if self.manual_seed_without_yolo else None
        self.public_id = int(public_id)
        self.box = tuple(float(v) for v in box)
        self.stamp = self.last_observation_stamp = float(stamp)
        self.selection_stamp = float(stamp)
        self.shape = frame.shape[:2]
        self._previous_frame = frame
        self.anchor, quality = appearance(frame, box)
        self.anchor_area = box[2] * box[3]
        self.tracker.initialize(frame, self.box)
        self.last_local_seed_stamp = stamp
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
            source = "visual" if quality >= self.quality_gate else "lost"
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
        return self.class_id < 0 or class_id == self.class_id or any(
            self.class_id in group and class_id in group for group in self.class_groups)

    def _compatible(self, box, reference):
        ratio = box[2] * box[3] / max(1., reference[2] * reference[3])
        distance = np.linalg.norm(center(box) - center(reference)) / max(8., math.hypot(reference[2], reference[3]))
        return 1. / self.area_ratio <= ratio <= self.area_ratio and distance <= self.center_gate

    def step(self, frame, stamp, detections, *, detector_updated=True, same_frame_correction=False,
             camera_transform=None, camera_motion_reliable=None,
             visual_observation=None, visual_quality=0., visual_scale_ratio=None):
        self.diagnostics = dict(detector_updated=bool(detector_updated), candidates=len(detections),
            rejected={}, local_tracker="not_run", camera_motion_available=False,
            frame_gap_sec=max(0., stamp-self.stamp) if math.isfinite(stamp) else None)
        correction = same_frame_correction and detector_updated and stamp == self.stamp
        if not math.isfinite(stamp) or stamp < self.stamp or (stamp == self.stamp and not correction):
            # Duplicate captures must not advance reacquisition or the tracker.
            return FusionResult(None, self.public_id, self.class_id, 0., "lost", "non_increasing_capture", self.last_detection_stamp, self.local_id)
        if self.box is None:
            return self._result(None, "idle", "no_selection", 0.)
        dt = stamp - self.stamp
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
        self.diagnostics.update(camera_motion_available=bool(motion_valid),
            camera_motion_reliable=bool(motion_reliable),
            recovery_phase="identity_memory" if self._long_recovery else "short",
            identity_memory_capacity=self.identity_capacity,
            identity_memory_remaining_sec=max(0., self.identity_ttl-absence))
        if not correction and (not motion_reliable or dt > self.max_gap):
            self._motion_continuous = False
        self.diagnostics['camera_motion_continuous'] = self._motion_continuous
        if not correction and motion_valid:
            transformed = transform_box(self.box, camera_transform)
            if transformed is not None:
                # Velocity is target residual motion; camera displacement must
                # not be learned as a target acceleration.
                points = np.float32([center(self.box), center(self.box)+self.velocity*.01])
                rotated = cv2.perspectiveTransform(points.reshape(-1, 1, 2), camera_transform).reshape(-1, 2)
                self.velocity = (rotated[1]-rotated[0])/.01
                old_area = max(1., self.box[2]*self.box[3])
                x, y, w, h = self.box
                corners = np.float32([[x, y], [x+w, y], [x+w, y+h], [x, y+h]])
                projected = cv2.perspectiveTransform(corners.reshape(-1, 1, 2), camera_transform)
                self.anchor_area *= abs(cv2.contourArea(projected))/old_area
                self.box = transformed
                if self.pending_box is not None:
                    self.pending_box = transform_box(self.pending_box, camera_transform)
                if self._previous_frame is not None and hasattr(self.tracker, "compensate_motion"):
                    self.tracker.compensate_motion(self._previous_frame, camera_transform)
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
        local = self.tracker.update(frame) if not correction and not self._long_recovery else None
        self.diagnostics['local_tracker'] = ("cached_same_capture" if correction else
            "short_tracking_ended" if self._long_recovery else
            "frame_gap_reset" if dt > self.max_gap else
            "valid" if local is not None and local.valid else
            str(getattr(local, "reason", "local_tracker_failed")))
        flow_quality = visual_quality
        visual_box = self._visual_box if correction else local.bbox if local is not None else None
        visual_quality = self._visual_quality if correction else 0.
        if correction:
            pass
        elif (self.manual_seed_without_yolo and local is not None and local.valid
                and valid_box(visual_box, frame.shape)):
            # Before YOLO has associated the manually selected target, expose
            # KCF's own per-frame box. ManualBoxTracker already applies bounds,
            # size and jump safety checks; fusion appearance gates must not
            # freeze the operator's KCF-only box while waiting for YOLO.
            _, visual_quality = appearance(frame, visual_box)
            self.diagnostics['local_tracker'] = 'manual_seed_kcf'
        elif local is not None and local.valid and valid_box(visual_box, frame.shape) and self._compatible(visual_box, predicted):
            feature, visual_quality = appearance(frame, visual_box)
            similarity = float(np.dot(feature, self.anchor)) if self.anchor is not None else 0.
            anchor_ratio = visual_box[2] * visual_box[3] / max(1., self.anchor_area)
            if similarity < self.appearance_gate or visual_quality < self.quality_gate or not 1. / self.area_ratio <= anchor_ratio <= self.area_ratio:
                visual_box = None
                self.diagnostics['local_tracker'] = ("appearance_rejected" if similarity < self.appearance_gate else
                    "quality_rejected" if visual_quality < self.quality_gate else "scale_rejected")
        else:
            visual_box = None
            if local is not None and local.valid:
                self.diagnostics['local_tracker'] = "position_or_bounds_rejected"
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
        self.diagnostics['lk_selected_observation'] = bool(flow_valid)
        self._visual_box, self._visual_quality = visual_box, visual_quality

        ranked = []
        rejected = Counter()
        for candidate in detections:
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
                    candidate.local_id == self.local_id and self.local_id >= 0
                    and self.result.observed and self.pending_count == 0
                    and visual_box is not None and iou(visual_box, candidate.box) >= .30):
                rejected['weak_identity_evidence'] += 1
                continue
            if candidate.local_id < 0 or not candidate.stable:
                rejected['unstable_identity'] += 1
                continue
            if not valid_box(candidate.box, frame.shape):
                rejected['box_bounds'] += 1
                continue
            # For a manual KCF seed, limit the first YOLO association to the
            # operator's ROI. Once the detector identity is confirmed, discard
            # this ROI and use the normal fusion association gates.
            if self.manual_seed_without_yolo:
                region = self.manual_seed_region
                cx, cy = center(candidate.box)
                inside_region = (region is not None and (
                    region[0] <= cx <= region[0] + region[2]
                    and region[1] <= cy <= region[1] + region[3]
                    or iou(candidate.box, region) >= .15))
                # Once a candidate inside the ROI starts identity confirmation,
                # let that same YOLO ID finish confirming if its box moves out
                # of the seed ROI during the confirmation frames.
                continuing_seed_identity = (self.pending_count > 0
                    and candidate.local_id == self.pending_id)
                inside_region = inside_region or continuing_seed_identity
                if not inside_region:
                    rejected['outside_manual_seed_region'] += 1
                    continue
            if not self._class_compatible(candidate.class_id):
                rejected['class'] += 1
                continue
            if not self.manual_seed_without_yolo and not self._compatible(candidate.box, predicted):
                rejected['position_or_size'] += 1
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
            ranked.append((float(score), candidate, quality))
        ranked.sort(key=lambda item: item[0], reverse=True)
        self.diagnostics.update(rejected=dict(rejected), accepted_candidates=len(ranked),
            no_detector_candidates=bool(detector_updated and not detections),
            best_score=ranked[0][0] if ranked else None,
            second_score=ranked[1][0] if len(ranked) > 1 else None)
        visual_reference_stamp = max(self.last_detection_stamp, self.selection_stamp)
        if len(ranked) > 1 and ranked[0][0] - ranked[1][0] < self.margin:
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
                # the selected identity while the new detector ID is pending.
                # A fresh operator selection starts its own short visual hold,
                # even when the displayed track's last YOLO stamp is older.
                # Do not refresh detector time or appearance, or extend this
                # fixed hold again on each frame.
                manual_visual_hold = (self.manual_seed_without_yolo
                    and self.manual_seed_visual_without_yolo)
                agrees_with_pending = (visual_box is not None
                    and stamp - visual_reference_stamp <= self.visual_hold
                    and iou(visual_box, match.box) >= .30
                    and self._compatible(visual_box, match.box))
                if visual_box is not None and (manual_visual_hold or agrees_with_pending):
                    return self._commit_observation(
                        visual_box, stamp, "visual", "reacquisition_confirmation_visual_hold",
                        min(.65, visual_quality, self.last_confidence))
                return self._result(None, "ambiguous", "reacquisition_confirmation", 0.)
        elif match is None and (detector_updated or stamp - self.pending_stamp > self.max_gap):
            self.pending_id = None
            self.pending_count = 0

        if match is not None:
            # The manual-only visual exception ends once YOLO has associated
            # a detection with the selected target. From here, the normal
            # detector-backed visual hold applies.
            self.manual_seed_without_yolo = False
            self.manual_seed_region = None
            # Two witnesses must agree before coordinates are blended.
            agree = visual_box is not None and iou(visual_box, match.box) >= .30 and self._compatible(visual_box, match.box)
            if agree:
                weight = max(.5, self.detector_weight * match.confidence / max(.01, self.detector_weight * match.confidence + (1. - self.detector_weight) * visual_quality))
                # Do not pull a strong, independently validated detector box
                # backwards by KCF's small temporal lag. Fusion is evidence
                # arbitration, not an obligation to average every coordinate.
                fused = match.box if match.confidence >= self.detector_authority else tuple(
                    weight * np.asarray(match.box) + (1. - weight) * np.asarray(visual_box))
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
            if self.class_id < 0:
                self.class_id = match.class_id
            feature, quality = appearance(frame, match.box)
            measured = stamp if match.detector_stamp is None else match.detector_stamp
            if quality >= .65 and match.confidence >= self.min_detector_conf and measured == stamp:
                self.anchor = .95 * self.anchor + .05 * feature
                self.anchor /= max(1e-9, np.linalg.norm(self.anchor))
                self.anchor_area = .95 * self.anchor_area + .05 * match.box[2] * match.box[3]
            self.last_detection_stamp = measured
            if measured < stamp:
                source = "visual"
            confidence = match.confidence
            self.pending_count = 0
            self.pending_id = None
        elif visual_box is not None and (
                self.manual_seed_without_yolo and self.manual_seed_visual_without_yolo
                or stamp - visual_reference_stamp <= self.visual_hold):
            fused, source = visual_box, "visual"
            confidence = min(.65, visual_quality, self.last_confidence)
        else:
            age = stamp - self.last_observation_stamp
            if age <= self.predict_hold and valid_box(predicted, frame.shape):
                return self._result(predicted, "prediction", "no_trusted_visual_observation", min(.25, self.last_confidence))
            return self._result(None, "lost", "identity_memory_waiting" if self._long_recovery else "no_trusted_visual_observation", 0.)
        return self._commit_observation(
            fused, stamp, source, "compatible_observations" if source == "fused" else source,
            confidence, update_velocity=not rebound)

    def _commit_observation(self, box, stamp, source, reason, confidence, update_velocity=True):
        observation_dt = stamp - self.last_observation_stamp
        if observation_dt > 0 and update_velocity:
            measured_velocity = (center(box) - center(self.box)) / observation_dt
            self.velocity = .6 * self.velocity + .4 * measured_velocity
        self.box = tuple(box)
        self.last_observation_stamp = stamp
        self.last_confidence = confidence
        return self._result(self.box, source, reason)
