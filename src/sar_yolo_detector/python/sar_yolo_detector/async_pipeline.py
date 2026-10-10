"""Bounded latest-only scheduling and frame-bound optical-flow propagation.

No ROS/model dependency. Images are immutable references; no frame FIFO and no
unbounded history. Propagated boxes are visual observations, never new YOLO IDs.
"""
from dataclasses import dataclass
from types import SimpleNamespace
import threading

import cv2
import numpy as np


class LatestMailbox:
    def __init__(self):
        self._condition = threading.Condition()
        self._item = None
        self._wake = False
        self.closed = False
        self.replaced = 0

    def put(self, item):
        with self._condition:
            if self.closed:
                return False
            if self._item is not None:
                self.replaced += 1
            self._item = item
            self._condition.notify()
            return True

    def wake(self):
        with self._condition:
            self._wake = True
            self._condition.notify()

    def get(self, timeout=0.):
        with self._condition:
            self._condition.wait_for(
                lambda: self.closed or self._wake or self._item is not None, timeout)
            item, self._item = self._item, None
            self._wake = False
            return item

    def clear(self):
        with self._condition:
            self._item = None
            self._wake = False

    def close(self):
        with self._condition:
            self.closed = True
            self._item = None
            self._condition.notify_all()


@dataclass(frozen=True)
class InferencePacket:
    image: object
    frame: object
    detections: tuple
    epoch: int
    processing_ms: float
    error: str = ""


class FlowDetections:
    """One gray conversion/paired LK call per update for all bounded tracklets."""
    def __init__(self, max_age=.75, max_tracks=30, points_per_box=16):
        if not np.isfinite(max_age) or max_age <= 0 or not 1 <= max_tracks <= 100:
            raise ValueError("Invalid optical-flow bounds")
        self.max_age = float(max_age)
        self.max_tracks = int(max_tracks)
        self.points_per_box = max(6, min(64, int(points_per_box)))
        self.reset()

    def reset(self):
        self.gray = None
        self.stamp = 0.
        self.detector_stamp = 0.
        self.items = []
        self.visual_observations = {}
        self.diagnostics = {}

    def _reject_flow(self, reason, count=1):
        rejected = self.diagnostics.setdefault('rejected', {})
        rejected[reason] = rejected.get(reason, 0) + count

    def reseed_identity(self, frame, stamp, old_id, new_id, box):
        """Discard old selected LK points, seed only a confirmed new track.

        Other vehicles keep their points and detector timestamps. This does
        not create an identity or renew its detector TTL.
        """
        from .box_fusion import valid_box
        if not valid_box(box, frame.shape) or stamp != self.stamp:
            return False
        gray = self.gray
        replacement = next((item for item in self.items if item[0].track_id == new_id), None)
        self.items = [item for item in self.items if item[0].track_id not in (old_id, new_id)]
        if replacement is None or gray is None:
            return False
        x, y, width, height = box
        left, top = int(x), int(y)
        crop = gray[top:int(y+height), left:int(x+width)]
        points = cv2.goodFeaturesToTrack(crop, self.points_per_box, .01, 3) if crop.size else None
        if points is not None:
            points += np.float32([left, top])
        values = dict(vars(replacement[0]),
            _flow_corners=self._corners((x, y, x+width, y+height), points))
        values['_flow_seed_area'] = max(1., abs(cv2.contourArea(values['_flow_corners'])))
        self.items.append((SimpleNamespace(**values), np.float64([x, y, x+width, y+height]), points))
        return points is not None

    @staticmethod
    def _gray(frame):
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def _corners(box, points):
        x1, y1, x2, y2 = box
        fallback = np.float32([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        if points is None or len(points) < 6:
            return fallback
        # An oblique target's detector AABB includes empty corner background.
        # Preserve a supported oriented footprint rather than rotating those
        # empty corners into an ever larger envelope. Never shrink the seed
        # below its detector box; reject sparse/degenerate footprint estimates.
        corners = cv2.boxPoints(cv2.minAreaRect(points))
        extent = np.ptp(corners, axis=0)
        if min(extent) < 4.:
            return fallback
        expansion = max((x2-x1)/extent[0], (y2-y1)/extent[1])
        if not 1. <= expansion <= 1.5:
            return fallback
        corners = (corners-corners.mean(axis=0))*expansion + np.float32([(x1+x2)/2, (y1+y2)/2])
        if np.any(np.ptp(corners, axis=0) > 1.2*np.float32([x2-x1, y2-y1])):
            return fallback
        return corners

    def _seed(self, gray, detections, stamp):
        self.gray, self.stamp, self.detector_stamp = gray, stamp, stamp
        self.items = []
        for detection in detections[:self.max_tracks]:
            x1, y1, x2, y2 = detection.aabb_xyxy
            if not np.all(np.isfinite((x1, y1, x2, y2))) or x2-x1 < 4 or y2-y1 < 4:
                continue
            left, top = max(0, int(x1)), max(0, int(y1))
            right, bottom = min(gray.shape[1], int(x2)), min(gray.shape[0], int(y2))
            crop = gray[top:bottom, left:right]
            points = cv2.goodFeaturesToTrack(crop, self.points_per_box, .01, 3) if crop.size else None
            if points is not None:
                points += np.float32([left, top])
            values = dict(vars(detection), detector_stamp=stamp)
            values['_flow_corners'] = self._corners((x1, y1, x2, y2), points)
            values['_flow_seed_area'] = max(1., abs(cv2.contourArea(values['_flow_corners'])))
            self.items.append((SimpleNamespace(**values), np.float64((x1, y1, x2, y2)), points))

    def update(self, frame, stamp, detections=None, detector_frame=None, detector_stamp=None, motion=None):
        self.diagnostics = dict(tracks_before=len(self.items), rejected={})
        if stamp != self.stamp:
            self.visual_observations = {}
        gray = self._gray(frame)
        if detections is not None:
            measured = float(detector_stamp)
            if measured > stamp or stamp - measured > self.max_age or measured <= self.detector_stamp:
                self._reject_flow('non_fresh_or_expired_packet')
                detections = None
            else:
                # Advance previous tracklets before merging a detector miss.
                # Their individual detector time/TTL must NOT be refreshed by
                # another vehicle's detection or an empty inference packet.
                if self.gray is not None and self.gray.shape == gray.shape and stamp > self.stamp:
                    self._advance(gray, motion.between(self.stamp, stamp) if motion is not None else None)
                previous = self.items
                self._remember_visual(previous, stamp)
                source_gray = gray if measured == stamp else self._gray(detector_frame)
                self._seed(source_gray, tuple(detections), measured)
                if measured < stamp and self.items:
                    self._advance(gray, motion.between(measured, stamp) if motion is not None else None)
                self.gray, self.stamp = gray, stamp
                ids = {item[0].track_id for item in self.items}
                from .box_fusion import iou
                for item in previous:
                    detection, box, _ = item
                    if (len(self.items) >= self.max_tracks or detection.track_id in ids
                            or not detection.track_id_is_stable or detection.track_id < 0
                            or stamp - detection.detector_stamp > self.max_age):
                        continue
                    as_box = (box[0], box[1], box[2]-box[0], box[3]-box[1])
                    if any(iou(as_box, (other[1][0], other[1][1], other[1][2]-other[1][0],
                                        other[1][3]-other[1][1])) > .3 for other in self.items):
                        continue
                    self.items.append(item)
        if self.gray is None or self.gray.shape != gray.shape or stamp < self.stamp:
            self.reset()
            return []
        if stamp - self.detector_stamp > self.max_age:
            self._reject_flow('detector_ttl_expired', len(self.items))
            self.items = []
        if stamp > self.stamp and self.items:
            self._advance(gray, motion.between(self.stamp, stamp) if motion is not None else None)
        self.gray, self.stamp = gray, stamp
        output = []
        for detection, box, _ in self.items:
            if stamp - detection.detector_stamp > self.max_age:
                self._reject_flow('track_detector_ttl_expired')
                continue
            observed_now = stamp == detection.detector_stamp
            # Unstable list-position IDs cannot become persistent via LK.
            if not observed_now and (not detection.track_id_is_stable or detection.track_id < 0):
                continue
            values = dict(vars(detection))
            values.update(aabb_xyxy=tuple(box), center_xy=tuple((box[:2]+box[2:])/2),
                          measurement_source="detector" if observed_now else "visual_tracker",
                          detector_stamp=detection.detector_stamp)
            if not observed_now:
                values.update(geometry_type="aabb", obb_xywhr=None, polygon_xy=None, rotation_deg=0.,
                              confidence=min(.65, float(detection.confidence)))
            output.append(SimpleNamespace(**values))
        self._remember_visual(self.items, stamp)
        self.diagnostics['tracks_after'] = len(output)
        return output

    def _remember_visual(self, items, stamp):
        # Preserve the independent forward/backward observation BEFORE a new
        # detector packet replaces its tracklet. The cache is capture-bound,
        # max_tracks bounded, contains no images and never renews detector TTL.
        for detection, box, _ in items:
            if (not detection.track_id_is_stable or detection.track_id < 0
                    or not 0 < stamp-detection.detector_stamp <= self.max_age
                    or getattr(detection, 'flow_quality', 0.) < .60):
                continue
            values = dict(vars(detection), aabb_xyxy=tuple(box),
                center_xy=tuple((box[:2]+box[2:])/2), measurement_source='visual_tracker')
            if detection.track_id not in self.visual_observations and len(self.visual_observations) >= self.max_tracks:
                continue
            self.visual_observations[detection.track_id] = SimpleNamespace(**values)

    def _advance(self, gray, transform=None):
        from .camera_motion import CameraMotion
        transform = transform if CameraMotion.validate(transform, gray.shape) else None
        groups = [(index, item[2]) for index, item in enumerate(self.items)
                  if item[2] is not None and len(item[2]) >= 4]
        if not groups:
            self._reject_flow('no_usable_features', len(self.items))
            self.items = []
            return
        before = np.concatenate([points for _, points in groups]).astype(np.float32)
        try:
            initial = cv2.perspectiveTransform(before, transform) if transform is not None else None
            scale = min(1., 480./gray.shape[1])
            size = (round(gray.shape[1]*scale), round(gray.shape[0]*scale))
            previous_gray = cv2.resize(self.gray, size, interpolation=cv2.INTER_AREA) if scale < 1. else self.gray
            current_gray = cv2.resize(gray, size, interpolation=cv2.INTER_AREA) if scale < 1. else gray
            after, status, _ = cv2.calcOpticalFlowPyrLK(previous_gray, current_gray, before*scale,
                initial*scale if initial is not None else None, winSize=(21, 21), maxLevel=2,
                flags=cv2.OPTFLOW_USE_INITIAL_FLOW if initial is not None else 0)
            if after is None:
                self._reject_flow('forward_flow_failed', len(self.items))
                self.items = []
                return
            back, back_status, _ = cv2.calcOpticalFlowPyrLK(current_gray, previous_gray, after,
                before*scale if initial is not None else None, winSize=(21, 21), maxLevel=2,
                flags=cv2.OPTFLOW_USE_INITIAL_FLOW if initial is not None else 0)
            if back is None:
                self._reject_flow('backward_flow_failed', len(self.items))
                self.items = []
                return
            after, back = after/scale, back/scale
        except cv2.error:
            self._reject_flow('opencv_flow_error', len(self.items))
            self.items = []
            return
        accepted = (status.ravel() > 0) & (back_status.ravel() > 0)
        accepted &= np.linalg.norm((back-before).reshape(-1, 2), axis=1) <= 1.5
        accepted &= np.all(np.isfinite(after.reshape(-1, 2)), axis=1)
        kept, offset = [], 0
        for index, points in groups:
            count = len(points)
            good = accepted[offset:offset+count]
            old = before[offset:offset+count].reshape(-1, 2)[good]
            new = after[offset:offset+count].reshape(-1, 2)[good]
            offset += count
            if len(new) < max(4, int(.5*count)):
                self._reject_flow('insufficient_forward_backward_support')
                continue
            expected = cv2.perspectiveTransform(old.reshape(-1, 1, 2), transform).reshape(-1, 2) if transform is not None else old
            delta = np.median(new-expected, axis=0)
            residual = np.linalg.norm(new-expected-delta, axis=1)
            detection, box, _ = self.items[index]
            if np.linalg.norm(delta) > max(16., np.linalg.norm(box[2:]-box[:2])):
                self._reject_flow('excessive_residual_displacement')
                continue
            # Tanks turn: coherent rotation is not an outlier to a translation
            # model. Fit a bounded similarity transform, after camera motion.
            affine, inliers = cv2.estimateAffinePartial2D(expected, new,
                method=cv2.RANSAC, ransacReprojThreshold=2., maxIters=100, confidence=.99)
            motion = np.float64([[1, 0, delta[0]], [0, 1, delta[1]], [0, 0, 1]])
            support = np.ones(len(new), dtype=bool)
            if affine is not None and inliers is not None:
                support = inliers.ravel().astype(bool)
                scale_change = np.hypot(affine[0, 0], affine[1, 0])
                angle = abs(np.arctan2(affine[1, 0], affine[0, 0]))
                spread = np.ptp(expected[support], axis=0) if support.any() else np.zeros(2)
                if (np.count_nonzero(support) >= max(4, int(np.ceil(.6*len(new))))
                        and min(spread) >= 3. and .80 <= scale_change <= 1.25
                        and angle <= np.deg2rad(35)):
                    motion[:2] = affine
                    residual = np.linalg.norm(cv2.transform(expected.reshape(-1, 1, 2), affine).reshape(-1, 2)-new, axis=1)
                else:
                    support[:] = True
            if np.median(residual[support]) > 3.:
                self._reject_flow('non_rigid_or_inconsistent_flow')
                continue
            # Carry the original quadrilateral, not its axis-aligned envelope:
            # repeatedly rotating AABBs causes runaway box enlargement.
            corners = getattr(detection, '_flow_corners',
                np.float32([[box[0], box[1]], [box[2], box[1]], [box[2], box[3]], [box[0], box[3]]]))
            matrix = motion @ transform if transform is not None else motion
            corners = cv2.perspectiveTransform(corners.reshape(-1, 1, 2), matrix).reshape(-1, 2)
            left, top = corners.min(axis=0)
            right, bottom = corners.max(axis=0)
            clipped = np.float64([max(0., left), max(0., top), min(gray.shape[1], right), min(gray.shape[0], bottom)])
            visible_area = max(0., clipped[2]-clipped[0])*max(0., clipped[3]-clipped[1])
            if (min(clipped[2:]-clipped[:2]) < 4.
                    or visible_area < .25*max(1., (right-left)*(bottom-top))):
                self._reject_flow('out_of_image')
                continue
            inside = support & (new[:, 0] >= 0) & (new[:, 0] < gray.shape[1]) & (new[:, 1] >= 0) & (new[:, 1] < gray.shape[0])
            if np.count_nonzero(inside) < 4:
                self._reject_flow('insufficient_in_image_support')
                continue
            values = dict(vars(detection), _flow_corners=corners,
                flow_scale_ratio=abs(cv2.contourArea(corners))/getattr(detection, '_flow_seed_area', max(1., abs(cv2.contourArea(corners)))),
                # Survival already passed the >=50% FB hard gate. Quality
                # measures the coherent surviving evidence, rather than
                # penalizing the same rejected points twice during rotation.
                flow_quality=float(np.count_nonzero(inside)/len(new) / (1.+np.median(residual[inside])/3.)))
            kept.append((SimpleNamespace(**values), clipped, new[inside].reshape(-1, 1, 2)))
        self.items = kept
