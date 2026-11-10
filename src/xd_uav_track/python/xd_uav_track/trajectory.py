"""One selected identity's bounded capture-time trajectory. No image retention."""
from collections import deque
from xd_uav_track.msg import TargetTrajectoryPoint
from xd_uav_track.perception_evidence import target_status


class TargetTrajectory:
    def __init__(self, retention_sec=60.0, maximum_points=600, sample_hz=10.0, gap_sec=1.0):
        self.retention_sec = max(.1, min(120.0, float(retention_sec)))
        self.maximum_points = max(1, min(1200, int(maximum_points)))
        self.sample_interval = 1.0 / max(.1, min(30.0, float(sample_hz)))
        self.gap_sec = max(.05, min(5.0, float(gap_sec)))
        self.points = deque(maxlen=self.maximum_points)
        self.uid = ""
        self.track_id = self.class_id = -1
        self.class_name = "unknown"
        self.class_valid = self.active = False
        self.segment_id = 0
        self.pending_reason = "selected"
        self.status = "WAITING_SELECTION"
        self.last_source = ""
        self.last_capture = None

    def close(self, reason="deselected"):
        self.active, self.status, self.pending_reason = False, "INACTIVE", reason

    def begin(self, target):
        if self.uid != target.target_uid:
            self.points.clear()
            self.uid, self.track_id = target.target_uid, target.track_id
            self.segment_id, self.pending_reason = 0, "selected"
            self.last_source, self.last_capture = "", None
            self.active, self.status = False, "WAITING_OBSERVATION"
        self.class_id, self.class_name, self.class_valid = target.class_id, target.class_name, target.class_valid

    def expire(self, now):
        while self.points and (now - self.points[0].capture_stamp).to_sec() > self.retention_sec:
            self.points.popleft()

    def remap(self, old, new, session):
        if self.track_id == old:
            self.track_id, self.uid = new, "{}:{}".format(session, new)

    def append(self, target, header, source, frame_seq):
        stamp = header.stamp
        if stamp.is_zero() or (self.uid == target.target_uid and self.last_capture is not None
                              and stamp < self.last_capture):
            return False
        self.expire(stamp)
        self.begin(target)
        status = target_status(target)
        if status == "LOST":
            self.last_capture = stamp
            self.pending_reason, self.status, self.active = "loss", "LOST", False
            return False
        self.active, self.status = True, "PREDICTING" if status == "PRED" else "TRACKING"
        if self.last_source and self.last_source != source:
            self.pending_reason = "source_change"
        if self.last_capture is not None and (stamp - self.last_capture).to_sec() > self.gap_sec:
            self.pending_reason = self.pending_reason or "observation_gap"
        self.last_source, self.last_capture = source, stamp
        same_capture = bool(self.points and self.points[-1].capture_stamp == stamp and
                            self.points[-1].image_source == source)
        if not self.pending_reason and not same_capture and self.points and \
                (stamp - self.points[-1].capture_stamp).to_sec() + 1e-6 < self.sample_interval:
            return False
        if same_capture and self.points[-1].observed and status == "PRED":
            return False
        point = TargetTrajectoryPoint()
        point.capture_stamp, point.frame_seq = stamp, frame_seq
        if self.pending_reason:
            self.segment_id += 1
            point.segment_reason, self.pending_reason = self.pending_reason, ""
        point.segment_id = self.segment_id
        point.image_source, point.measurement_source = source, target.measurement_source
        point.observed, point.predicted = status in ("DET", "SOT"), status == "PRED"
        point.center_px, point.center_norm = list(target.center_px), list(target.bbox_cxcywh_norm[:2])
        point.position_valid = target.position_valid and point.observed
        if point.position_valid:
            point.position_frame, point.position_stamp = target.position_frame, target.position_stamp
            point.position_xyz, point.position_covariance = list(target.position_xyz), list(target.position_covariance)
        if same_capture:
            point.segment_id, point.segment_reason = self.points[-1].segment_id, self.points[-1].segment_reason
            self.points[-1] = point
        else:
            self.points.append(point)
        return True
