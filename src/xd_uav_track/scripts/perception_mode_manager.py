#!/usr/bin/env python3
"""Runtime detect/track/fusion contract independent from flight-follow state."""

from collections import OrderedDict, deque
from copy import deepcopy
import hashlib
import json
import math
import threading
import time
import uuid

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from xd_uav_track.msg import (
    PerceptionCommandResult, PerceptionState, PerceptionTarget,
    PerceptionTargetArray, SelectPerceptionTargetCommand,
    SetPerceptionModeCommand, TrackStateArray, TargetTrajectoryArray,
    SetDetectorModelCommand, DetectorModelCommit, DetectionArray)
from xd_uav_track.srv import (ManageVisualIdentity, ManageVisualIdentityRequest,
                              GetPerceptionState, GetPerceptionStateResponse)
from xd_uav_track.sot_tracker import SotTracker
from xd_uav_track.pixel_worker import LatestPixelWorker
from xd_uav_track.perception_policy import image_signature, evidence_gate
from xd_uav_track.trajectory import TargetTrajectory
from xd_uav_track.command_queue import CommandQueue
from xd_uav_track.perception_evidence import (
    bound_prediction, identity_only, target_status, observation_age_ms, metric_position_valid)


class PerceptionModeManager:
    MODES = {"detect", "track", "fusion"}
    SELECTIONS = {"clear", "track_id", "point", "roi"}

    def __init__(self):
        # CSRT otherwise creates a large OpenCV pool that competes with Gazebo
        # and shared inference. This changes CPU scheduling, not the algorithm.
        cv2.setNumThreads(max(1, min(4, int(rospy.get_param("~opencv_threads", 1)))))
        self._lock = threading.RLock()
        self._bridge = CvBridge()
        self._session_id = uuid.uuid4().hex
        self._version = 1
        self._mode = str(rospy.get_param("~initial_mode", "detect")).lower()
        if self._mode not in self.MODES:
            raise ValueError("~initial_mode must be detect, track, or fusion")
        self._class_names = [str(value) for value in rospy.get_param(
            "~class_names", ["car", "ar-car", "tank", "m142"])]
        self._frame_tolerance = max(0.0, min(0.001, float(rospy.get_param(
            "~selection_frame_tolerance_sec", 0.001))))
        self._frame_cache_size = max(2, min(60, int(rospy.get_param(
            "~frame_cache_size", 15))))
        self._source_clock_reset_jump = max(0.1, float(rospy.get_param(
            "~source_clock_reset_jump_sec", 5.0)))
        self._selected_uid = ""
        self._selected_track_id = -1
        self._selected_source = ""
        self._requested_algorithm = "auto"
        self._active_algorithm = "none"
        self._manual_tracker = None
        self._registration_number = 0
        self._visual_targets = {}
        self._background_visual = OrderedDict()
        self._identity_aliases = OrderedDict()
        self._image_evidence = OrderedDict()
        self._prediction_horizon = max(0.0, min(2.0, float(rospy.get_param(
            "~prediction_horizon_sec", 0.5))))
        namespace = rospy.get_namespace().strip("/")
        self._position_frame = str(rospy.get_param("~body_frd_frame",
            (namespace + "/" if namespace else "") + "base_link_frd"))
        if not self._position_frame:
            raise ValueError("~body_frd_frame must identify the FRD position frame")
        self._position_timeout = max(0.02, min(5.0, float(rospy.get_param(
            "~position_timeout_sec", 1.0))))
        self._stale_notified = set()
        self._visual_ttl = max(1.0, min(120.0, float(rospy.get_param("~visual_identity_ttl_sec", 60.0))))
        self._identity_client = rospy.ServiceProxy(
            str(rospy.get_param("~identity_service", "track/manage_visual_identity")),
            ManageVisualIdentity)
        self._manual_target = None
        self._selected_output = None
        self._raw_sot = None
        self._sot_failure = ''
        self._metric_recovery = None
        self._manual_last_valid_wall = 0.0
        self._last_error = ""
        self._latest_targets = {}
        self._latest_headers = {}
        self._track_history = {}
        self._sot_history = deque(maxlen=self._frame_cache_size)
        self._sot_manual = False
        self._identity_attached = False
        self._identity_update_stamp = None
        self._identity_bind_stamp = None
        self._last_identity_rejection = ''
        self._reference_signature = None
        self._fusion_reason = ""
        self._fusion_conflict = False
        self._last_source_wall = {}
        self._source_timeout = max(0.2, float(rospy.get_param("~source_timeout_sec", 1.0)))
        self._frames = {}
        self._frame_receipts = {}
        self._presentation_ms = deque(maxlen=128)
        self._command_cache = OrderedDict()
        self._model_transaction = None
        self._transaction_event = threading.Event()
        self._closing = False
        self._trajectory = TargetTrajectory(
            rospy.get_param("~trajectory/retention_sec", 60.0),
            rospy.get_param("~trajectory/maximum_points", 600),
            rospy.get_param("~trajectory/sample_hz", 10.0),
            rospy.get_param("~trajectory/gap_sec", 1.0))
        self._trajectory_publish_interval = 1.0 / max(.1, min(10.0, float(rospy.get_param(
            "~trajectory/publish_hz", 2.0))))
        self._last_trajectory_publish = 0.0
        self._model = {
            "id": "unknown", "sha256": "", "backend": "unknown",
            "epoch": 0, "ready": False}

        self._result_pub = rospy.Publisher(
            "perception/command_result", PerceptionCommandResult,
            queue_size=20)
        self._state_pub = rospy.Publisher(
            "perception/state", PerceptionState, queue_size=1, latch=True)
        self._targets_pub = rospy.Publisher(
            "perception/targets", PerceptionTargetArray, queue_size=10)
        self._trajectory_pub = rospy.Publisher(
            "perception/trajectory", TargetTrajectoryArray, queue_size=1, latch=True)
        self._mode_sub = rospy.Subscriber(
            "perception/mode/set", SetPerceptionModeCommand,
            lambda message:self._enqueue_command("mode", message), queue_size=10)
        self._selection_sub = rospy.Subscriber(
            "perception/target/select", SelectPerceptionTargetCommand,
            lambda message:self._enqueue_command("target_select", message), queue_size=10)
        self._model_request_pub = rospy.Publisher(
            "perception/internal/model/set", SetDetectorModelCommand, queue_size=1)
        self._model_command_sub = rospy.Subscriber(
            "perception/model/set", SetDetectorModelCommand,
            lambda message:self._enqueue_command("model", message), queue_size=10)
        self._model_commit_sub = rospy.Subscriber(
            "perception/internal/model/result", DetectorModelCommit,
            self._model_commit_callback, queue_size=10)
        self._tracks_sub = rospy.Subscriber(
            str(rospy.get_param("~tracks_topic", "track/tracks_by_source")),
            TrackStateArray, self._tracks_callback, queue_size=10,
            tcp_nodelay=True)
        self._visual_sub = rospy.Subscriber(
            "track/visual_tracks_by_source", TrackStateArray,
            self._visual_tracks_callback, queue_size=10, tcp_nodelay=True)
        self._model_sub = rospy.Subscriber(
            str(rospy.get_param("~model_status_topic",
                                "sar_yolo_multi_source_detection/model_status")),
            DiagnosticArray, self._model_callback, queue_size=1)
        self._image_subscribers = []
        sources = rospy.get_param("~sources", [])
        for item in sources:
            source = str(item.get("image_source", "")).strip()
            topic = str(item.get("image_topic", "")).strip()
            if not source or not topic or source in self._frames:
                raise ValueError("each perception source needs unique image_source and image_topic")
            self._frames[source] = deque(maxlen=self._frame_cache_size)
            self._image_subscribers.append(rospy.Subscriber(
                topic, Image, self._image_callback, callback_args=source,
                queue_size=1, buff_size=16 * 1024 * 1024,
                tcp_nodelay=True))
        if not self._frames:
            raise ValueError("~sources must contain at least one camera")
        self._pixel_targets = {}
        self._pixel_history = {}
        self._output_stamps = None
        self._pixel_recovery_wall = {}
        self._pixel_recovery_pub = rospy.Publisher('perception/internal/roi_request', String, queue_size=1)
        self._pixel_observations_pub = rospy.Publisher('perception/internal/pixel_observations',
            PerceptionTargetArray, queue_size=1)
        self._pixel_diagnostics_pub = rospy.Publisher('perception/pixel_diagnostics',
            DiagnosticArray,queue_size=1)
        self._pixel_worker = None
        self._pixel_anchors = {}
        self._pixel_anchor_subscriber = None
        if bool(rospy.get_param("perception/high_frequency_pixel_enabled", False)):
            self._output_stamps = {}
            self._pixel_worker = LatestPixelWorker(self._frames, self._pixel_callback, self._pixel_recovery,
                maximum_tracks=rospy.get_param("~pixel_max_tracks", 8),
                maximum_anchor_age=rospy.get_param("~pixel_anchor_timeout", 2.2),
                error=lambda source,error:rospy.logerr_throttle(2.,
                    'pixel worker failed closed for %s: %s',source,error))
            self._pixel_anchor_subscriber = rospy.Subscriber('track/internal/observations_by_source',
                DetectionArray,self._pixel_anchor_callback,queue_size=1)
        self._commands = CommandQueue(self._execute_command,
            rospy.get_param("~command_queue_size", 16))
        self._state_service = rospy.Service("perception/state/get", GetPerceptionState,
                                            self._get_state)
        self._state_timer = rospy.Timer(rospy.Duration(min(0.2, self._source_timeout / 2)),
                                        self._heartbeat)
        self._publish_state()
        rospy.on_shutdown(self.close)
        rospy.loginfo("perception mode manager ready: session=%s mode=%s sources=%s",
                      self._session_id, self._mode,
                      ",".join(sorted(self._frames)))

    def close(self):
        self._closing = True
        self._transaction_event.set()
        self._commands.close()
        if getattr(self, "_pixel_worker", None) is not None:
            self._pixel_worker.close()

    def _clear_pixels(self):
        self._raw_sot = None
        self._metric_recovery = None
        if getattr(self, "_pixel_worker", None) is not None:
            self._pixel_worker.clear()
        getattr(self, "_pixel_targets", {}).clear()
        getattr(self, "_pixel_history", {}).clear()
        (getattr(self, "_output_stamps", None) or {}).clear()
        getattr(self, "_pixel_recovery_wall", {}).clear()
        getattr(self, "_pixel_anchors", {}).clear()
        getattr(self,'_frame_receipts',{}).clear()
        if hasattr(self,'_presentation_ms'):
            self._presentation_ms.clear()

    def _pixel_anchor_callback(self, message):
        with self._lock:
            source=message.image_source
            if source not in self._frames or message.model_epoch != self._model['epoch']:
                return
            history=self._pixel_anchors.setdefault(source,deque(maxlen=8))
            if history and message.header.stamp <= history[-1].header.stamp:
                return
            history.append(message)
            header=self._latest_headers.get(source)
            if header is not None and header.stamp == message.header.stamp:
                self._seed_pixels(source,header,self._latest_targets.get(source,[]))
                if (self._manual_target is not None and source == self._selected_source
                        and self._fusion_conflict):
                    # Track and raw-anchor topics may arrive in either order.
                    self._validate_anchor(message,self._latest_targets.get(source,[]))

    def _seed_pixels(self, source, header, targets):
        corrected = self._measured_targets(source, header, targets)
        captured = self._find_frame(source,0,header.stamp)
        if corrected and captured is not None:
            self._pixel_worker.seed(source,header,captured[2],corrected,self._model['epoch'])

    def _measured_targets(self, source, header, targets):
        """Same-capture associated YOLO geometry, never Kalman-smoothed boxes."""
        captured=self._find_frame(source,0,header.stamp)
        anchor=next((a for a in reversed(getattr(self,'_pixel_anchors',{}).get(source,()))
                     if a.header.stamp == header.stamp),None)
        if captured is None or anchor is None:
            return []
        height,width=captured[2].shape[:2]
        if anchor.image_width != width or anchor.image_height != height:
            return []
        measured={self._canonical_id(c.track_id):c for c in anchor.candidates if c.track_id>=0 and
                  c.measurement_source in ('','detector') and (c.has_bbox or c.has_normalized_bbox)}
        corrected=[]
        for target in targets:
            candidate=measured.get(target.track_id)
            if candidate is None:
                continue
            item=deepcopy(target)
            item.detection_confidence = candidate.confidence
            if candidate.has_bbox:
                box=list(candidate.bbox)
            else:
                cx,cy,bw,bh=candidate.normalized_bbox
                box=[round((cx-bw/2)*width),round((cy-bh/2)*height),
                     round((cx+bw/2)*width),round((cy+bh/2)*height)]
            if not 0<=box[0]<box[2]<=width or not 0<=box[1]<box[3]<=height:
                continue
            item.bbox_xyxy=box
            item.center_px=[.5*(box[0]+box[2]),.5*(box[1]+box[3])]
            item.bbox_cxcywh_norm=[item.center_px[0]/width,item.center_px[1]/height,
                (box[2]-box[0])/width,(box[3]-box[1])/height]
            corrected.append(item)
        return corrected

    def _pixel_recovery(self, source, header, target, reason, epoch, generation):
        with self._lock:
            if (self._closing or epoch != self._model['epoch'] or
                    generation != self._pixel_worker.generation):
                return
            now = time.monotonic()
            if now - self._pixel_recovery_wall.get(source, -10.) < 1.:
                return
            self._pixel_recovery_wall[source] = now
            payload = dict(image_source=source, capture_stamp=header.stamp.to_sec(),
                model_epoch=epoch, box=list(target.bbox_cxcywh_norm),
                class_id=target.class_id, reason=reason)
        self._pixel_recovery_pub.publish(String(data=json.dumps(payload)))

    def _pixel_callback(self, source, header, width, height, targets, epoch, generation):
        with self._lock:
            if (self._closing or epoch != self._model['epoch'] or
                    generation != self._pixel_worker.generation):
                return
            for target in targets:
                canonical = self._canonical_id(target.track_id)
                if canonical != target.track_id:
                    target.track_id = canonical
                    target.target_uid = "{}:{}".format(self._session_id, canonical)
                target.selected = (source == self._selected_source and
                                   target.target_uid == self._selected_uid)
            targets = self._unique_identity_targets(targets)
            self._pixel_targets[source] = (header.stamp, targets)
            history = self._pixel_history.setdefault(source,deque(maxlen=self._frame_cache_size))
            history.append((header.stamp,targets))
            # Keep independent SOT evidence separate from its rejected display
            # state. A prior detector geometry outlier must not poison future
            # visual observations and force a new user selection.
            raw = getattr(self,'_raw_sot',None)
            peer = next((t for t in targets if t.target_uid == self._selected_uid and
                t.lifecycle_state == 'confirmed' and t.measurement_source == 'visual_tracker'
                and t.tracking_quality >= .6 and not t.last_detection_stamp.is_zero()
                and t.last_detection_stamp < header.stamp),None)
            if (self._fusion_conflict and self._fusion_reason in ('motion_conflict','geometry_conflict')
                    and source == self._selected_source and raw is not None
                    and raw[0] == header.stamp and peer is not None):
                captured = self._find_frame(source,0,header.stamp)
                competitors = [self._normalized_iou(peer.bbox_cxcywh_norm,t.bbox_cxcywh_norm)
                    for t in targets if t.track_id != peer.track_id and t.lifecycle_state == 'confirmed']
                if (captured is not None and not any(v >= .15 for v in competitors)
                        and self._visual_consensus(raw[1],peer,captured[2])):
                    self._fusion_conflict = False
                    self._fusion_reason = 'visual_consensus_recovered'
                    self._manual_target = deepcopy(raw[1])
                    for index,(stamp,_) in enumerate(self._sot_history):
                        if stamp == header.stamp:
                            self._sot_history[index] = (stamp,deepcopy(raw[1]))
                            break
            observations = PerceptionTargetArray(header=header,image_source=source,
                image_width=width,image_height=height,model_epoch=epoch,targets=deepcopy(targets))
            publisher = getattr(self, '_pixel_observations_pub', None)
            if publisher is not None:
                publisher.publish(observations)
            self._cache_selection_snapshot(source, header.stamp, targets)
            detector_header = self._latest_headers.get(source)
            if (self._manual_target is not None and source == self._selected_source
                    and detector_header is not None and detector_header.stamp == header.stamp):
                # Detector/SOT can finish before the independent LK worker.
                # Re-evaluate that capture when its visual evidence arrives;
                # unique timestamps still govern identity confirmation.
                self._validate_anchor(TrackStateArray(header=header,image_source=source),
                                      self._latest_targets.get(source, []))
            self._publish_targets(source, header, width, height)

    def _enqueue_command(self, kind, command):
        with self._lock:
            if not str(command.request_id).strip() or len(command.request_id) > 128:
                self._reject(command, kind, "INVALID_REQUEST", "request_id must contain 1..128 characters")
                return
            cached = self._command_cache.get(command.request_id)
            if cached:
                if cached[0] == self._fingerprint(command):
                    self._result_pub.publish(cached[1])
                else:
                    self._reject(command, kind, "REQUEST_CONFLICT", "request_id content differs")
                return
            queue = getattr(self, "_commands", None)
            status = queue.submit(kind, command) if queue else "UNAVAILABLE"
            if status in ("QUEUED", "DUPLICATE"):
                self._publish_result(command.request_id, kind, "QUEUED", "", "queued for serial execution")
            else:
                # Never overwrite an active transaction's eventual result.
                self._reject(command, kind, status, "transaction queue: " + status)

    def _execute_command(self, kind, command):
        try:
            handler = {"mode":self._mode_callback, "model":self._model_command_callback,
                       "target_select":self._selection_callback}[kind]
            handler(command)
            if kind == "model":
                while not self._closing:
                    with self._lock:
                        pending = self._model_transaction
                        if pending is None or pending[0].request_id != command.request_id:
                            break
                        if time.monotonic() > self._model_deadline:
                            self._heartbeat(None)
                            break
                    self._transaction_event.wait(.1)
                    self._transaction_event.clear()
        except Exception as error:
            with self._lock:
                self._model_transaction = None if kind == "model" else self._model_transaction
                self._publish_result(command.request_id, kind, "FAILED", "INTERNAL_ERROR",
                                     str(error), self._fingerprint(command))
                self._last_error = str(error)
                self._publish_state()

    @staticmethod
    def _fingerprint(message):
        return hashlib.sha256((getattr(message, "_type", type(message).__name__) +
                               str(message)).encode("utf-8")).hexdigest()

    def _validate_command(self, message, command_type):
        request_id = str(message.request_id).strip()
        if not request_id or len(request_id) > 128:
            return False, "INVALID_REQUEST", "request_id must contain 1..128 characters", None
        fingerprint = self._fingerprint(message)
        cached = self._command_cache.get(request_id)
        if cached is not None:
            if cached[0] != fingerprint:
                return False, "REQUEST_CONFLICT", \
                    "request_id was already used with different content", None
            self._result_pub.publish(cached[1])
            return False, "DUPLICATE", "duplicate command replayed", cached[1]
        if message.session_id != self._session_id:
            return False, "STALE_SESSION", "session_id does not match current node", None
        pending = getattr(self, "_model_transaction", None)
        if pending is not None:
            if pending[0].request_id == request_id:
                if pending[1] != fingerprint:
                    return False, "REQUEST_CONFLICT", "in-flight request content differs", None
                self._publish_result(request_id, command_type, "ACCEPTED", "", "already preparing")
                return False, "DUPLICATE", "in-flight duplicate", None
            return False, "BUSY", "model transaction is in progress", None
        if int(message.expected_state_version) != self._version:
            return False, "STALE_STATE", "expected_state_version does not match", None
        now = rospy.Time.now()
        if not math.isfinite(message.ttl_sec) or not 0.0 <= message.ttl_sec <= 120.0:
            return False, "INVALID_TTL", "TTL must be finite within 0..120 seconds", None
        if not message.issued_at.is_zero() and (message.issued_at - now).to_sec() > .5:
            return False, "INVALID_TIME", "issued_at is in the future", None
        if (message.ttl_sec > 0.0 and not message.issued_at.is_zero() and
                (now - message.issued_at).to_sec() > message.ttl_sec):
            return False, "EXPIRED", "command TTL elapsed", None
        self._publish_result(message.request_id, command_type, "ACCEPTED", "", "accepted")
        return True, "", "", fingerprint

    def _publish_result(self, request_id, command_type, stage, error, message,
                        cache_fingerprint=None):
        result = PerceptionCommandResult()
        result.header.stamp = rospy.Time.now()
        result.request_id = request_id
        result.session_id = self._session_id
        result.command_type = command_type
        result.stage = stage
        result.error_code = error
        result.message = message
        result.state_version = self._version
        self._result_pub.publish(result)
        if cache_fingerprint is not None and stage in ("APPLIED", "REJECTED", "FAILED"):
            self._command_cache[request_id] = (cache_fingerprint, result)
            self._command_cache.move_to_end(request_id)
            while len(self._command_cache) > 128:
                self._command_cache.popitem(last=False)
        return result

    def _reject(self, message, command_type, error, reason, fingerprint=None):
        self._last_error = reason
        self._publish_result(message.request_id, command_type, "REJECTED",
                             error, reason, fingerprint)
        self._publish_state()

    def _mode_callback(self, message):
        with self._lock:
            valid, error, reason, fingerprint = self._validate_command(message, "mode")
            if not valid:
                if fingerprint is None and error != "DUPLICATE":
                    self._reject(message, "mode", error, reason,
                                 self._fingerprint(message))
                return
            mode = str(message.mode).lower().strip()
            if mode not in self.MODES:
                self._reject(message, "mode", "INVALID_MODE",
                             "mode must be detect, track, or fusion", fingerprint)
                return
            if mode != self._mode:
                self._mode = mode
                self._version += 1
                if mode == "detect":
                    self._clear_selection(publish=False)
                self._initialize_switch_runtime("mode_switch")
            self._last_error = ""
            self._publish_result(message.request_id, "mode", "APPLIED", "",
                                 "perception mode applied", fingerprint)
            self._publish_state()
            self._publish_trajectory(force=True)

    def _selection_callback(self, message):
        with self._lock:
            valid, error, reason, fingerprint = self._validate_command(
                message, "target_select")
            if not valid:
                if fingerprint is None and error != "DUPLICATE":
                    self._reject(message, "target_select", error, reason,
                                 self._fingerprint(message))
                return
            kind = str(message.selection_type).lower().strip()
            if kind not in self.SELECTIONS:
                self._reject(message, "target_select", "INVALID_SELECTION",
                             "selection_type must be clear, track_id, point, or roi",
                             fingerprint)
                return
            if kind == "clear":
                self._clear_selection(publish=False)
            elif message.image_source not in self._frames:
                self._reject(message, "target_select", "UNKNOWN_SOURCE",
                             "image_source is not configured", fingerprint)
                return
            elif kind in ("track_id", "point"):
                uid = str(message.target_uid).strip()
                if kind == "track_id" and uid:
                    prefix, separator, number = uid.rpartition(":")
                    if (prefix != self._session_id or separator != ":" or
                            not 1 <= len(number) <= 10 or not number.isascii() or
                            not number.isdecimal() or not 0 < int(number) <= 2147483647):
                        self._reject(message, "target_select", "INVALID_TARGET_UID",
                                     "target_uid must belong to this session", fingerprint)
                        return
                    resolved = self._canonical_id(int(number))
                    if message.track_id > 0 and self._canonical_id(message.track_id) != resolved:
                        self._reject(message, "target_select", "TARGET_CONFLICT",
                                     "target_uid and track_id disagree", fingerprint)
                        return
                frame_entry, candidates = self._selection_evidence(message, kind)
                if frame_entry is None:
                    self._reject(message, "target_select", "STALE_FRAME",
                                 self._last_error, fingerprint)
                    return
                target = self._find_existing_target(message, kind, candidates)
                if target is None:
                    self._reject(message, "target_select", "TARGET_LOST",
                                 "requested active target was not found", fingerprint)
                    return
                if not self._select_existing(message.image_source, target,
                                             message.tracker_algorithm or "auto", frame_entry):
                    self._reject(message, "target_select", "ALGORITHM_UNAVAILABLE",
                                 self._last_error, fingerprint)
                    return
            else:
                frame_entry, candidates = self._selection_evidence(message, kind)
                if frame_entry is None:
                    self._reject(message, "target_select", "STALE_FRAME",
                                 self._last_error, fingerprint)
                    return
                x,y,w,h = message.roi_norm
                if not (0 <= x < 1 and 0 <= y < 1 and w > 0 and h > 0
                        and x+w <= 1+1e-6 and y+h <= 1+1e-6):
                    self._reject(message, "target_select", "INVALID_ROI",
                                 "ROI must be inside the referenced source frame", fingerprint)
                    return
                existing = self._find_roi_target(message, candidates)
                if existing is None:
                    contained = self._roi_contained_targets(message, candidates)
                    x,y,w,h = message.roi_norm
                    roi = (x+w*.5,y+h*.5,w,h)
                    overlapping = {t.track_id for t in candidates if t.lifecycle_state == 'confirmed'
                        and self._normalized_iou(roi,t.bbox_cxcywh_norm) >= .25}
                    if len(contained) > 1 or len(overlapping) > 1:
                        self._reject(message, 'target_select', 'AMBIGUOUS_TARGET',
                            'ROI contains multiple target identities; use a tighter ROI or select an ID',
                            fingerprint)
                        return
                if existing is not None:
                    if not self._select_existing(message.image_source, existing,
                                                 message.tracker_algorithm or "auto", frame_entry):
                        self._reject(message, "target_select", "ALGORITHM_UNAVAILABLE",
                                     self._last_error, fingerprint)
                        return
                elif not self._initialize_roi(message):
                    self._reject(message, "target_select", "STALE_FRAME",
                                 self._last_error, fingerprint)
                    return
            if kind != "clear" and self._mode == "detect":
                self._mode = "track"
            self._version += 1
            self._last_error = ""
            self._publish_result(message.request_id, "target_select", "APPLIED", "",
                                 "visual target selection applied", fingerprint)
            self._publish_state()
            self._publish_trajectory(force=True)

    def _selection_evidence(self, message, kind):
        last_image = self._last_source_wall.get(message.image_source)
        if last_image is not None and time.monotonic() - last_image > self._source_timeout:
            self._last_error = "referenced camera is no longer producing fresh images"
            return None, []
        sequence, stamp = message.frame_seq, message.capture_stamp
        if not sequence and stamp.is_zero():
            if kind != "track_id":
                self._last_error = "click and ROI require an original frame reference"
                return None, []
            history = self._track_history.get(message.image_source)
            if not history:
                self._last_error = "source has no fresh track snapshot"
                return None, []
            stamp = history[-1][0]
        entry = self._find_frame(message.image_source, sequence, stamp)
        if entry is None:
            self._last_error = "referenced image frame is missing or mismatched"
            return None, []
        snapshots = self._track_history.get(message.image_source, ())
        snapshot = next((targets for captured, targets in reversed(snapshots)
                         if captured == entry[1]), None)
        if snapshot is None and kind != "roi":
            self._last_error = "referenced frame has no corresponding track snapshot"
            return None, []
        return entry, list(snapshot or [])

    def _find_existing_target(self, message, kind, candidates=None):
        candidates = candidates if candidates is not None else self._latest_targets.get(message.image_source, [])
        candidates = [t for t in candidates if t.lifecycle_state in ("confirmed", "occluded")]
        if kind == "track_id":
            uid = getattr(message, "target_uid", "")
            requested = int(uid.rpartition(":")[2]) if uid else message.track_id
            return next((item for item in candidates
                         if item.track_id == self._canonical_id(requested)), None)
        x, y = message.point_norm
        if not 0.0 <= x <= 1.0 or not 0.0 <= y <= 1.0:
            return None
        ranked = []
        for item in candidates:
            cx, cy, width, height = item.bbox_cxcywh_norm
            if (cx - width * 0.5 <= x <= cx + width * 0.5 and
                    cy - height * 0.5 <= y <= cy + height * 0.5):
                ranked.append((width * height, item))
        return min(ranked, default=(0.0, None), key=lambda pair: pair[0])[1]

    def _select_existing(self, source, target, algorithm="auto", frame_entry=None):
        header = self._latest_headers.get(source)
        frame_entry = frame_entry or (self._find_frame(
            source, 0, header.stamp) if header is not None else None)
        if frame_entry is None:
            self._last_error = "selected target image is no longer cached"
            return False
        seq, stamp, frame = frame_entry
        try:
            tracker = SotTracker(algorithm)
        except ValueError as error:
            self._last_error = str(error)
            return False
        x1, y1, x2, y2 = target.bbox_xyxy
        result = tracker.initialize(frame, (x1, y1, x2-x1, y2-y1), stamp.to_sec())
        if not result.valid:
            self._last_error = result.reason
            return False
        self._retire_unclassified_background(target.track_id)
        self._park_visual(preserve_unclassified=target.track_id == self._selected_track_id)
        self._background_visual.pop(target.track_id, None)
        self._selected_track_id = target.track_id
        self._selected_output = None
        self._raw_sot = None
        self._sot_failure = ''
        self._metric_recovery = None
        self._selected_uid = target.target_uid
        self._selected_source = source
        self._requested_algorithm = algorithm
        self._active_algorithm = result.algorithm
        self._manual_tracker = tracker
        self._manual_target = deepcopy(target)
        self._manual_target.selected = True
        self._trajectory.begin(self._manual_target)
        self._sot_manual = target.track_id in {
            t.track_id for t in self._visual_targets.get(source, (None, []))[1]}
        self._fusion_conflict = False
        self._reference_signature = image_signature(frame, target.bbox_xyxy)
        self._identity_attached = False
        self._identity_update_stamp = None
        self._identity_bind_stamp = None
        if not self._sot_manual:
            self._attach_selected_identity(source, target, stamp)
        self._manual_last_valid_wall = time.monotonic()
        self._sot_history.clear()
        self._sot_history.append((stamp, deepcopy(self._manual_target)))
        self._replay_sot(source, stamp)
        return True

    def _attach_selected_identity(self, source, target, selected_stamp):
        # The web often displays an intermediate LK frame, not a YOLO frame.
        # Ownership must be established with a real historical detector
        # anchor, without pretending the old detection is the current image.
        self._identity_attach_retry_wall = time.monotonic()
        anchors = [(selected_stamp, target)]
        for captured, targets in reversed(self._track_history.get(source, ())):
            gap = (selected_stamp-captured).to_sec()
            if not 0. <= gap <= 1.5 or captured == selected_stamp:
                continue
            anchors.extend((captured,t) for t in targets if t.track_id==target.track_id
                and t.lifecycle_state=='confirmed' and t.measurement_source=='detector'
                and t.detection_confidence>=.5)
        for captured, anchor in anchors[:4]:
            frame = self._find_frame(source,0,captured)
            if frame is None or frame[1] != captured:
                continue
            signature = image_signature(frame[2],anchor.bbox_xyxy)
            try:
                result = self._identity_client(self._identity_request(
                    'attach',anchor,source,captured,signature))
            except rospy.ServiceException as error:
                self._last_identity_rejection = str(error)
                break
            if getattr(result,'success',False):
                self._identity_attached=True
                self._identity_update_stamp=captured
                self._last_identity_rejection=''
                return
            self._last_identity_rejection=getattr(result,'message','identity service unavailable')

    def _replay_sot(self, source, after_stamp):
        for seq, stamp, frame in self._frames[source]:
            if stamp > after_stamp:
                self._update_sot(frame, stamp)

    def _update_sot(self, frame, stamp):
        result = self._manual_tracker.update(frame, stamp.to_sec())
        self._sot_failure = '' if result.valid else result.reason
        self._raw_sot = None
        previous = self._manual_target
        if result.valid:
            updated = self._manual_target_from_result(
                result, frame.shape[1], frame.shape[0], stamp)
            if previous is not None:
                for field in ("class_id", "class_name", "class_valid",
                              "detection_confidence", "last_detection_stamp"):
                    setattr(updated, field, getattr(previous, field))
            self._manual_target = updated
            self._manual_last_valid_wall = time.monotonic()
            self._raw_sot = (stamp,deepcopy(updated))
            if self._fusion_conflict:
                updated.lifecycle_state = "lost"
                updated.measurement_source = "stale"
                updated.tracking_quality = 0.0
        elif previous is not None:
            self._manual_target = deepcopy(previous)
            self._manual_target.lifecycle_state = "lost"
            self._manual_target.measurement_source = "stale"
            self._manual_target.tracking_quality = 0.0
            self._manual_target.position_valid = False
        self._sot_history.append((stamp, deepcopy(self._manual_target)))
        if self._sot_manual:
            self._sync_visual(self._manual_target, self._selected_source, stamp)

    def _identity_request(self, action, target, source, stamp, signature=None,
                          candidate_id=-1, key=""):
        request = ManageVisualIdentityRequest()
        request.action = action
        request.owner_session = self._session_id
        request.registration_key = key
        request.image_source = source
        request.global_id = target.track_id
        request.candidate_global_id = candidate_id
        track = request.observation
        track.header.stamp = stamp
        track.track_id = target.track_id
        track.class_id = target.class_id
        track.bbox = target.bbox_xyxy
        track.normalized_bbox = target.bbox_cxcywh_norm
        captured = self._find_frame(source, 0, stamp)
        if captured is not None and captured[1] == stamp:
            height, width = captured[2].shape[:2]
            # A Kalman rectangle can extend beyond the image at its edge.
            # The identity service receives only visible, same-frame pixels,
            # not negative coordinates or fabricated off-image evidence.
            x1, y1, x2, y2 = target.bbox_xyxy
            track.bbox = [max(0,min(width,x1)), max(0,min(height,y1)),
                          max(0,min(width,x2)), max(0,min(height,y2))]
            x1, y1, x2, y2 = track.bbox
            track.normalized_bbox = [(x1+x2)/(2.*width), (y1+y2)/(2.*height),
                                     (x2-x1)/float(width), (y2-y1)/float(height)]
        track.lifecycle_state = target.lifecycle_state
        track.detected = target.lifecycle_state == "confirmed"
        track.predicted = not track.detected
        track.confidence = target.tracking_quality
        track.tracking_quality = target.tracking_quality
        track.measurement_source = "visual_tracker"
        track.last_detection_stamp = target.last_detection_stamp
        if signature is not None:
            request.image_signature = [float(value) for value in signature]
        return request

    def _sync_visual(self, target, source, stamp, signature=None):
        if (self._identity_attached and self._identity_update_stamp is not None and
                stamp <= self._identity_update_stamp):
            return
        try:
            response = self._identity_client(self._identity_request(
                "update", target, source, stamp, signature))
            if not response.success:
                self._last_error = response.message
            elif self._identity_attached:
                self._identity_update_stamp = stamp
                if self._last_error == 'invalid visual identity observation':
                    self._last_error = ''
        except rospy.ServiceException as error:
            self._last_error = "global identity update failed: {}".format(error)

    def _retire_visual_observer(self, target, source, stamp):
        retired = deepcopy(target)
        identity_only(retired)
        cached = self._visual_targets.get(source)
        if cached is not None:
            for index, item in enumerate(cached[1]):
                if item.track_id == target.track_id:
                    cached[1][index] = deepcopy(retired)
        try:
            self._identity_client(self._identity_request('retire', target, source, stamp))
        except rospy.ServiceException as error:
            self._last_error = 'visual retirement failed: {}'.format(error)

    def _retire_unclassified_background(self, keep_id=-1):
        for identity, state in list(self._background_visual.items()):
            if identity == keep_id or state['target'].class_valid:
                continue
            self._retire_visual_observer(state['target'], state['source'],
                state['target'].last_detection_stamp)
            state['tracker'].reset()
            del self._background_visual[identity]

    def _park_visual(self, preserve_unclassified=False):
        if self._identity_attached and not self._sot_manual and self._manual_target is not None:
            stamp = self._sot_history[-1][0] if self._sot_history else rospy.Time.now()
            try:
                self._identity_client(self._identity_request(
                    'detach', self._manual_target, self._selected_source, stamp))
            except rospy.ServiceException:
                pass
            self._identity_attached = False
        if not self._sot_manual or self._manual_tracker is None or self._manual_target is None:
            return
        if not preserve_unclassified and not self._manual_target.class_valid:
            stamp = self._sot_history[-1][0] if self._sot_history else rospy.Time.now()
            self._retire_visual_observer(self._manual_target, self._selected_source, stamp)
            self._manual_tracker.reset()
            return
        self._background_visual[self._manual_target.track_id] = {
            "source": self._selected_source, "tracker": self._manual_tracker,
            "target": deepcopy(self._manual_target), "updated": time.monotonic(),
            "signature": self._reference_signature}
        while len(self._background_visual) > 8:
            self._background_visual.popitem(last=False)

    def _advance_background(self, source, frame, stamp):
        now = time.monotonic()
        for identity, item in list(self._background_visual.items()):
            if now-item["updated"] > self._visual_ttl:
                del self._background_visual[identity]
                continue
            if item["source"] != source:
                continue
            result = item["tracker"].update(frame, stamp.to_sec())
            previous = item["target"]
            if result.valid:
                updated = self._manual_target_from_result(result, frame.shape[1],frame.shape[0],stamp)
                updated.track_id = previous.track_id
                updated.target_uid = previous.target_uid
                updated.selected = False
                for field in ("class_id","class_name","class_valid", "last_detection_stamp"):
                    setattr(updated, field, getattr(previous, field))
                item["target"] = updated
                item["updated"] = now
            else:
                item["target"].lifecycle_state = "lost"
                item["target"].measurement_source = "stale"
                item["target"].tracking_quality = 0.0
            self._sync_visual(item["target"],source,stamp)

    def _sot_at(self, stamp):
        return next((target for time_stamp, target in reversed(self._sot_history)
                     if time_stamp == stamp), None)

    @staticmethod
    def _normalized_iou(first, second):
        acx, acy, aw, ah = first
        bcx, bcy, bw, bh = second
        ax1, ay1, ax2, ay2 = acx - aw * 0.5, acy - ah * 0.5, acx + aw * 0.5, acy + ah * 0.5
        bx1, by1, bx2, by2 = bcx - bw * 0.5, bcy - bh * 0.5, bcx + bw * 0.5, bcy + bh * 0.5
        intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * \
            max(0.0, min(ay2, by2) - max(ay1, by1))
        union = aw * ah + bw * bh - intersection
        return intersection / union if union > 1e-9 else 0.0

    def _find_roi_target(self, message, candidates=None):
        x, y, width, height = [float(value) for value in message.roi_norm]
        roi = (x + 0.5 * width, y + 0.5 * height, width, height)
        scores = sorted(((self._normalized_iou(roi, target.bbox_cxcywh_norm), target)
                         for target in (candidates if candidates is not None else
                                        self._latest_targets.get(message.image_source, []))
                         if target.lifecycle_state in ("confirmed", "occluded")),
                        key=lambda pair: pair[0], reverse=True)
        contained = self._roi_contained_targets(message, candidates)
        if scores and not scores[0][1].class_valid and contained:
            # A previously parked, unclassified manual/background region must
            # not outrank the recognized vehicle merely because it is larger.
            return contained[0] if len(contained) == 1 else None
        if not scores or scores[0][0] < 0.25:
            return contained[0] if len(contained) == 1 else None
        if len(scores) > 1 and scores[0][0] - scores[1][0] < 0.10:
            return None
        return scores[0][1]

    def _roi_contained_targets(self, message, candidates=None):
        """Coarse ROI is a selection region, not necessarily an object box.

        Use only the already frame-validated snapshot. One clearly contained
        recognized entity can reuse its ID and tight box, even if padding makes
        IoU small. Distinct entities remain ambiguous, never overlap-merged.
        """
        x,y,w,h = message.roi_norm
        if w <= 0 or h <= 0:
            return []
        unique = {}
        for target in (candidates if candidates is not None else
                       self._latest_targets.get(message.image_source, [])):
            if (target.lifecycle_state != 'confirmed' or not target.class_valid or
                    target.measurement_source not in ('detector','visual_tracker')):
                continue
            cx,cy,bw,bh = target.bbox_cxcywh_norm
            area = bw*bh
            if area <= 0 or area/(w*h) < .05 or not (x <= cx <= x+w and y <= cy <= y+h):
                continue
            intersection = max(0.,min(x+w,cx+bw*.5)-max(x,cx-bw*.5)) * \
                           max(0.,min(y+h,cy+bh*.5)-max(y,cy-bh*.5))
            if intersection/area >= .8:
                unique[target.track_id] = target
        return list(unique.values())

    def _find_frame(self, source, sequence, stamp):
        frames = self._frames.get(source, ())
        if sequence:
            for item in reversed(frames):
                if item[0] == sequence:
                    if not stamp.is_zero() and abs((item[1]-stamp).to_sec()) > self._frame_tolerance:
                        return None
                    return item
            # A sequence-only command refers to one exact source frame.  Never
            # initialize an ROI on the newest image merely because the
            # requested frame has already left the bounded cache.
            return None
        if not stamp.is_zero():
            nearest = min(frames, default=None,
                          key=lambda item: abs((item[1]-stamp).to_sec()))
            return (nearest if nearest is not None and
                    abs((nearest[1]-stamp).to_sec()) <= self._frame_tolerance else None)
        return frames[-1] if frames else None

    def _initialize_roi(self, message):
        if not message.frame_seq and message.capture_stamp.is_zero():
            self._last_error = "ROI selection requires frame_seq or capture_stamp"
            return False
        frame_entry = self._find_frame(message.image_source,
                                       message.frame_seq,
                                       message.capture_stamp)
        if frame_entry is None:
            self._last_error = "requested image frame is not in the bounded cache"
            return False
        sequence, stamp, frame = frame_entry
        if (not message.capture_stamp.is_zero() and
                abs((stamp - message.capture_stamp).to_sec()) >
                self._frame_tolerance):
            self._last_error = "requested image timestamp is outside cache tolerance"
            return False
        x, y, width, height = [float(value) for value in message.roi_norm]
        if not (0.0 <= x < 1.0 and 0.0 <= y < 1.0 and width > 0.0 and
                height > 0.0 and x + width <= 1.0+1e-6 and y + height <= 1.0+1e-6):
            self._last_error = "roi_norm must be [x,y,width,height] inside the source image"
            return False
        # ROS float32 may round x + width just above one at the image edge.
        width, height = min(width, 1.-x), min(height, 1.-y)
        algorithm = str(message.tracker_algorithm or "auto").lower()
        try:
            tracker = SotTracker(algorithm=algorithm)
        except ValueError as error:
            self._last_error = str(error)
            return False
        pixel_roi = (x * frame.shape[1], y * frame.shape[0],
                     width * frame.shape[1], height * frame.shape[0])
        result = tracker.initialize(frame, pixel_roi, stamp.to_sec())
        if not result.valid:
            self._last_error = result.reason
            return False
        # Allocate identity only through the same C++ entity table as detector tracks.
        prospective = self._manual_target_from_result(result,frame.shape[1],frame.shape[0],stamp)
        signature = image_signature(frame,prospective.bbox_xyxy)
        self._registration_number += 1
        try:
            response = self._identity_client(self._identity_request(
                "register", prospective, message.image_source, stamp, signature,
                key=str(self._registration_number)))
        except rospy.ServiceException as error:
            self._last_error = "global identity registration unavailable: {}".format(error)
            return False
        if not response.success:
            self._last_error = response.message
            return False
        self._retire_unclassified_background()
        self._park_visual()
        self._manual_tracker = tracker
        self._sot_manual = True
        self._fusion_conflict = False
        self._selected_track_id = response.global_id
        self._selected_uid = "{}:{}".format(self._session_id, response.global_id)
        self._selected_source = message.image_source
        self._requested_algorithm = algorithm
        self._active_algorithm = result.algorithm
        self._manual_target = self._manual_target_from_result(
            result, frame.shape[1], frame.shape[0], stamp)
        self._trajectory.begin(self._manual_target)
        self._manual_last_valid_wall = time.monotonic()
        self._reference_signature = image_signature(frame, self._manual_target.bbox_xyxy)
        self._sot_history.clear()
        self._sot_history.append((stamp, deepcopy(self._manual_target)))
        self._replay_sot(message.image_source, stamp)
        return True

    def _clear_selection(self, publish=True):
        self._trajectory.close()
        if publish:
            self._publish_trajectory(force=True)
        self._park_visual(preserve_unclassified=self._mode == 'detect')
        if self._manual_tracker is not None and not self._sot_manual:
            self._manual_tracker.reset()
        self._manual_tracker = None
        self._manual_target = None
        self._selected_output = None
        self._raw_sot = None
        self._metric_recovery = None
        self._selected_uid = ""
        self._selected_track_id = -1
        self._selected_source = ""
        self._requested_algorithm = "auto"
        self._active_algorithm = "none"
        self._sot_history.clear()
        self._reference_signature = None
        self._sot_manual = False
        self._identity_attached = False
        self._identity_update_stamp = None
        self._identity_bind_stamp = None

    def _image_callback(self, message, source):
        callback_start=time.monotonic()
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "perception frame decode failed: %s", error)
            return
        with self._lock:
            if message.header.stamp.is_zero():
                return
            now = rospy.Time.now()
            if not now.is_zero() and (message.header.stamp - now).to_sec() > 1.0:
                return  # delayed old-segment frames after a ROS clock rewind
            if self._frames[source] and message.header.stamp <= self._frames[source][-1][1]:
                jump = (self._frames[source][-1][1] - message.header.stamp).to_sec()
                if jump <= self._source_clock_reset_jump:
                    return  # duplicates and ordinary late frames are not restarts
                self._restart_source_clock(source)
            self._frames[source].append((message.header.seq,
                                         message.header.stamp, frame.copy()))
            receipts=self._frame_receipts.setdefault(source,OrderedDict())
            receipts[message.header.stamp.to_nsec()]=callback_start
            while len(receipts)>self._frame_cache_size:
                receipts.popitem(last=False)
            self._last_source_wall[source] = time.monotonic()
            self._stale_notified.discard(source)
            if getattr(self, "_pixel_worker", None) is not None:
                self._pixel_worker.submit(source, message.header, frame, self._model['epoch'])
            self._advance_background(source, frame, message.header.stamp)
            if (self._manual_tracker is None or source != self._selected_source or
                    self._mode not in ("track", "fusion")):
                return
            if self._sot_history and message.header.stamp <= self._sot_history[-1][0]:
                return
            self._update_sot(frame, message.header.stamp)
            self._publish_targets(source, message.header, message.width,
                                  message.height)

    def _restart_source_clock(self, source):
        """Invalidate only this camera's evidence; never re-use an old SOT ROI.

        Public identities in other sources survive. Reacquisition must establish
        fresh evidence, rather than pretending pre-restart boxes are observed.
        Caller holds _lock. Threshold matches the C++ source clock policy.
        """
        self._clear_pixels()
        self._frames[source].clear()
        getattr(self,'_frame_receipts',{}).pop(source,None)
        for cache in (self._latest_headers, self._latest_targets,
                      self._track_history, self._visual_targets):
            cache.pop(source, None)
        for key in list(self._image_evidence):
            if key[0] == source:
                del self._image_evidence[key]
        for identity, visual in list(self._background_visual.items()):
            if visual["source"] == source:
                visual["tracker"].reset()
                del self._background_visual[identity]
        if self._selected_source == source:
            self._selected_output = None
            if self._manual_tracker is not None:
                self._manual_tracker.reset()
                self._manual_tracker = None
            if self._manual_target is not None:
                self._manual_target.lifecycle_state = "lost"
                self._manual_target.measurement_source = "stale"
                self._manual_target.position_valid = False
                self._manual_target.tracking_quality = 0.0
            self._sot_history.clear()
            self._reference_signature = None
            self._active_algorithm = "none"
            self._trajectory.close("source_clock_reset")
        self._version += 1
        self._publish_state()

    def _manual_target_from_result(self, result, width, height, stamp):
        x, y, box_width, box_height = result.bbox
        target = PerceptionTarget()
        target.target_uid = self._selected_uid
        target.track_id = self._selected_track_id
        target.class_id = -1
        target.class_name = "unknown"
        target.class_valid = False
        target.detection_confidence = 0.0
        target.tracking_quality = result.confidence
        target.bbox_xyxy = [int(round(x)), int(round(y)),
                            int(round(x + box_width)),
                            int(round(y + box_height))]
        target.center_px = [float(x + 0.5 * box_width),
                            float(y + 0.5 * box_height)]
        target.bbox_cxcywh_norm = [target.center_px[0] / width,
                                   target.center_px[1] / height,
                                   box_width / width, box_height / height]
        target.selected = True
        target.lifecycle_state = "confirmed"
        target.measurement_source = "visual_tracker"
        target.last_detection_stamp = rospy.Time(0)
        target.age_ms = 0.0
        target.position_valid = False
        target.position_stamp = rospy.Time(0)
        return target

    def _tracks_callback(self, message):
        with self._lock:
            if message.model_epoch and message.model_epoch < self._model["epoch"]:
                return
            if message.model_epoch > self._model["epoch"]:
                # A track array has no verified model hash/id. Wait for runtime
                # status or commit, rather than label new-epoch data with the
                # previous model's identity when ROS topics arrive out of order.
                return
            source = str(message.image_source)
            if source not in self._frames:
                return
            previous = self._latest_headers.get(source)
            if previous is not None and message.header.stamp <= previous.stamp:
                return
            frame = self._frames.get(source)
            if frame and (message.header.stamp - frame[-1][1]).to_sec() > 1.0:
                return  # pre-restart in-flight data must not poison the new segment
            if frame and frame[-1][2] is not None:
                height, width = frame[-1][2].shape[:2]
            else:
                width = height = 0
            targets = []
            for track in message.tracks:
                if track.lifecycle_state not in ("confirmed", "occluded", "lost"):
                    continue
                target = PerceptionTarget()
                target.track_id = self._canonical_id(track.track_id)
                target.target_uid = "{}:{}".format(self._session_id, target.track_id)
                target.class_id = track.class_id
                target.class_valid = track.class_id >= 0
                target.class_name = (self._class_names[track.class_id]
                                     if 0 <= track.class_id < len(self._class_names)
                                     else "unknown")
                target.detection_confidence = track.confidence
                target.tracking_quality = track.tracking_quality
                target.bbox_xyxy = list(track.bbox)
                target.center_px = [0.5 * (track.bbox[0] + track.bbox[2]),
                                    0.5 * (track.bbox[1] + track.bbox[3])]
                target.bbox_cxcywh_norm = list(track.normalized_bbox)
                target.selected = (target.target_uid == self._selected_uid and
                                   source == self._selected_source)
                target.lifecycle_state = track.lifecycle_state
                target.measurement_source = track.measurement_source or (
                    "detector" if track.detected and not track.predicted
                    else "prediction")
                target.last_detection_stamp = track.last_detection_stamp
                target.age_ms = observation_age_ms(message.header.stamp,
                    track.last_observation_stamp, rospy.Time.now())
                target.position_valid = metric_position_valid(track, message.header.stamp,
                    self._position_frame, rospy.Time.now(), self._position_timeout)
                target.position_frame = track.position_frame
                target.position_stamp = track.position_stamp
                target.position_xyz = list(track.relative_position_body)
                target.position_covariance = list(track.position_covariance)
                targets.append(target)
            targets = self._unique_identity_targets(targets)
            self._latest_targets[source] = targets
            self._latest_headers[source] = deepcopy(message.header)
            worker = getattr(self, "_pixel_worker", None)
            if worker is not None:
                self._seed_pixels(source,message.header,targets)
            self._cache_selection_snapshot(source, message.header.stamp, targets)
            if self._manual_target is not None and source == self._selected_source:
                self._validate_anchor(message, targets)
            self._bind_registered_visuals(message, targets)
            self._publish_targets(source, message.header, width, height)

    def _cache_selection_snapshot(self, source, stamp, targets):
        history = self._track_history.setdefault(source, deque(maxlen=self._frame_cache_size))
        snapshot = next((items for captured, items in history if captured == stamp), None)
        if snapshot is not None:
            by_id = {item.track_id: item for item in snapshot}
            for item in targets:
                previous = by_id.get(item.track_id)
                # A same-capture LK publication must not erase the genuine
                # detector evidence needed to attach the public identity.
                if (previous is not None and previous.measurement_source == 'detector'
                        and item.measurement_source != 'detector'):
                    continue
                by_id[item.track_id] = deepcopy(item)
            snapshot[:] = list(by_id.values())
        else:
            history.append((stamp, deepcopy(targets)))
            # Callbacks may arrive in a different order, but selection uses capture order.
            ordered = sorted(history, key=lambda item: item[0])
            history.clear()
            history.extend(ordered)

    def _visual_tracks_callback(self, message):
        if message.model_epoch and message.model_epoch < self._model["epoch"]:
            return
        with self._lock:
            if message.image_source not in self._frames:
                return
            previous = self._visual_targets.get(message.image_source)
            if previous is not None and message.header.stamp < previous[0]:
                return
            targets = []
            for track in message.tracks:
                item = PerceptionTarget()
                item.track_id = track.track_id
                item.target_uid = "{}:{}".format(self._session_id,track.track_id)
                item.class_id = track.class_id
                item.class_valid = 0 <= track.class_id < len(self._class_names)
                item.class_name = self._class_names[track.class_id] if item.class_valid else "unknown"
                item.bbox_xyxy = list(track.bbox)
                item.bbox_cxcywh_norm = list(track.normalized_bbox)
                item.center_px = [(track.bbox[0]+track.bbox[2])*0.5,
                                  (track.bbox[1]+track.bbox[3])*0.5]
                item.lifecycle_state = track.lifecycle_state
                item.measurement_source = "visual_tracker" if track.detected else "stale"
                item.tracking_quality = track.tracking_quality
                item.last_detection_stamp = track.last_detection_stamp
                item.selected = track.track_id == self._selected_track_id
                targets.append(item)
                background = self._background_visual.get(track.track_id)
                if background and item.class_valid:
                    background["target"].class_id = item.class_id
                    background["target"].class_name = item.class_name
                    background["target"].class_valid = True
                if self._sot_manual and self._selected_source == message.image_source and \
                        track.track_id == self._selected_track_id and item.class_valid:
                    for field in ("class_id", "class_name", "class_valid"):
                        setattr(self._manual_target, field, getattr(item, field))
            self._visual_targets[message.image_source] = (message.header.stamp,targets)
            self._cache_selection_snapshot(message.image_source, message.header.stamp, targets)
            entry = self._find_frame(message.image_source,0,message.header.stamp)
            if entry is not None:
                self._publish_targets(message.image_source,message.header,
                                      entry[2].shape[1],entry[2].shape[0])

    def _bind_registered_visuals(self, message, targets):
        entry = self._find_frame(message.image_source,0,message.header.stamp)
        if entry is None:
            return
        registered = self._visual_targets.get(message.image_source)
        visual_targets = list(registered[1]) if registered and registered[0] == message.header.stamp else []
        selected = self._sot_at(message.header.stamp)
        if self._sot_manual and selected is not None and not any(
                t.track_id == selected.track_id for t in visual_targets):
            visual_targets.append(selected)
        for visual in visual_targets:
            if visual.lifecycle_state != "confirmed":
                self._reset_visual_binding(visual, message)
                continue
            scored = sorted(((self._normalized_iou(visual.bbox_cxcywh_norm,t.bbox_cxcywh_norm),t)
                             for t in targets if t.measurement_source == "detector"
                             and t.detection_confidence >= .5),key=lambda x:x[0],reverse=True)
            if not scored or scored[0][0]<.5 or (len(scored)>1 and scored[0][0]-scored[1][0]<.15):
                self._reset_visual_binding(visual, message)
                continue
            candidate = scored[0][1]
            if candidate.track_id == visual.track_id:
                continue
            signature = image_signature(entry[2],candidate.bbox_xyxy)
            try:
                result = self._identity_client(self._identity_request(
                    "bind",visual,message.image_source,message.header.stamp,
                    signature,candidate_id=candidate.track_id))
            except rospy.ServiceException:
                continue
            if result.success and result.bound:
                self._remap_identity(candidate.track_id, result.global_id)
                candidate.track_id = result.global_id
                candidate.target_uid = "{}:{}".format(self._session_id,result.global_id)

    def _canonical_id(self, identity):
        # A bounded, acyclic alias set also resolves IDs on delayed web frames.
        for _ in range(len(self._identity_aliases)):
            replacement = self._identity_aliases.get(identity)
            if replacement is None:
                break
            identity = replacement
        return identity

    @staticmethod
    def _unique_identity_targets(targets):
        # Only identical *authorized public IDs* are deduplicated. Overlap of
        # distinct IDs is never sufficient. Prefer measured geometry to a
        # dormant fragment's stale/predicted box after a service-approved bind.
        unique = {}
        def quality(t):
            return (t.lifecycle_state == 'confirmed', t.measurement_source == 'detector',
                    t.detection_confidence, t.tracking_quality)
        for target in targets:
            current = unique.get(target.track_id)
            if current is None or quality(target) > quality(current):
                unique[target.track_id] = target
        return list(unique.values())

    def _remap_identity(self, old, new):
        if old == new:
            return
        self._trajectory.remap(old, new, self._session_id)
        self._identity_aliases[old] = new
        for alias, value in list(self._identity_aliases.items()):
            if value == old:
                self._identity_aliases[alias] = new
        while len(self._identity_aliases) > 64:
            self._identity_aliases.popitem(last=False)
        snapshots = list(self._latest_targets.values())
        snapshots.extend(items for history in self._track_history.values() for _, items in history)
        snapshots.extend(items for _, items in self._visual_targets.values())
        snapshots.extend(items for _, items in getattr(self, '_pixel_targets', {}).values())
        snapshots.extend(items for history in getattr(self, '_pixel_history', {}).values()
                         for _, items in history)
        snapshots.append([item for _, item in self._sot_history])
        if getattr(self, '_raw_sot', None) is not None:
            snapshots.append([self._raw_sot[1]])
        if getattr(self, '_selected_output', None) is not None:
            snapshots.append([self._selected_output[1]])
        if self._manual_target is not None:
            snapshots.append([self._manual_target])
        for items in snapshots:
            for item in items:
                if item.track_id == old:
                    item.track_id = new
                    item.target_uid = "{}:{}".format(self._session_id, new)
            # A cache may already contain both provisional and registered observations.
            items[:] = self._unique_identity_targets(items)
        if self._selected_track_id == old:
            self._selected_track_id = new
            self._selected_uid = "{}:{}".format(self._session_id, new)
            self._sot_manual = new in {item.track_id for item in
                self._visual_targets.get(self._selected_source, (None, []))[1]}
            self._background_visual.pop(new, None)
        for (source, identity), stamp in list(self._image_evidence.items()):
            if identity == old:
                existing = self._image_evidence.get((source, new))
                self._image_evidence[(source, new)] = max(stamp, existing) if existing else stamp
                del self._image_evidence[(source, old)]

    def _selected_identity_conflict(self, visual, reason):
        self._fusion_conflict = True
        self._fusion_reason = reason
        self._manual_target.lifecycle_state = "lost"
        self._manual_target.measurement_source = "stale"
        visual.lifecycle_state = "lost"
        visual.measurement_source = "stale"

    def _rebind_selected_fragment(self, message, targets, visual, comparison, frame):
        """Ask the global owner to confirm an ID fork; never merge by overlap.

        During confirmation the old selection is explicitly lost, while the
        unselected detector entity remains visible. A successful service bind
        is the only permission to rewrite its public identity.
        """
        scored = sorted(((self._normalized_iou(comparison.bbox_cxcywh_norm,
                         t.bbox_cxcywh_norm), t) for t in targets
                         if t.measurement_source == 'detector' and
                         t.lifecycle_state == 'confirmed' and t.detection_confidence >= .5),
                        key=lambda item: item[0], reverse=True)
        if not scored or scored[0][0] < .15:
            return None
        if (comparison.lifecycle_state != 'confirmed' or
                comparison.measurement_source not in ('detector', 'visual_tracker') or
                comparison.tracking_quality < .5):
            self._selected_identity_conflict(visual, 'identity_unavailable')
            if self._identity_attached:
                self._reset_visual_binding(comparison, message)
            return None
        if scored[0][0] < .5 or (len(scored) > 1 and scored[0][0]-scored[1][0] < .15):
            self._selected_identity_conflict(visual, 'ambiguous_competitor')
            if self._identity_attached:
                self._reset_visual_binding(comparison, message)
            return None
        candidate = scored[0][1]
        signature = image_signature(frame, candidate.bbox_xyxy)
        accepted, reason = evidence_gate(comparison.bbox_xyxy, candidate.bbox_xyxy,
            0., self._reference_signature, signature, same_capture=True)
        if not accepted or not self._identity_attached:
            self._selected_identity_conflict(visual, reason if not accepted else 'identity_unavailable')
            if self._identity_attached:
                self._reset_visual_binding(comparison, message)
            return None
        # LK can supply full-object geometry if CSRT has collapsed onto a part.
        # It has already passed same-capture, selected-ID and quality checks.
        evidence = deepcopy(comparison)
        evidence.lifecycle_state = 'confirmed'
        evidence.measurement_source = 'visual_tracker'
        if (getattr(self, '_identity_bind_stamp', None) is not None and
                message.header.stamp <= self._identity_bind_stamp):
            self._selected_identity_conflict(visual, 'identity_rebind_pending')
            return None
        self._identity_bind_stamp = message.header.stamp
        try:
            self._sync_visual(evidence, message.image_source, message.header.stamp)
            result = self._identity_client(self._identity_request(
                'bind', evidence, message.image_source, message.header.stamp,
                signature, candidate_id=candidate.track_id))
        except rospy.ServiceException:
            result = None
        if not getattr(result, 'success', False) or not getattr(result, 'bound', False):
            if not getattr(result, 'success', False):
                self._last_identity_rejection=getattr(result,'message','identity service unavailable')
                self._reset_visual_binding(comparison, message)
            self._selected_identity_conflict(visual, 'identity_rebind_pending')
            return None
        self._remap_identity(candidate.track_id, result.global_id)
        candidate.track_id = result.global_id
        candidate.target_uid = self._selected_uid
        self._fusion_conflict = False
        self._last_identity_rejection = ''
        self._fusion_reason = 'identity_rebound'
        return candidate

    def _reset_visual_binding(self, visual, message):
        try:
            self._identity_client(self._identity_request(
                "reset_bind", visual, message.image_source, message.header.stamp))
        except rospy.ServiceException as error:
            self._last_error = "global binding reset failed: {}".format(error)

    def _visual_consensus(self, first, second, frame):
        if (first.target_uid != second.target_uid or first.track_id != second.track_id
                or first.lifecycle_state != 'confirmed'
                or first.measurement_source not in ('detector','visual_tracker')
                or self._normalized_iou(first.bbox_cxcywh_norm,second.bbox_cxcywh_norm) < .5):
            return False
        accepted,_ = evidence_gate(first.bbox_xyxy,second.bbox_xyxy,0.,
            image_signature(frame,first.bbox_xyxy),image_signature(frame,second.bbox_xyxy),same_capture=True)
        return accepted

    def _metric_anchor_verified(self, candidate, stamp):
        # The C++ global entity must already have accepted two fresh, tightly
        # bounded metric observations. Never bridge a new ID by proximity.
        covariance=np.asarray(candidate.position_covariance,dtype=float).reshape(3,3)
        eligible=(not self._sot_manual and candidate.target_uid==self._selected_uid
            and candidate.track_id==self._selected_track_id and candidate.position_valid
            and candidate.position_frame==self._position_frame and candidate.position_stamp==stamp
            and candidate.detection_confidence>=.5 and np.isfinite(covariance).all()
            and np.allclose(covariance,covariance.T,atol=1e-6)
            and np.linalg.eigvalsh(covariance).min()>=0. and np.trace(covariance)<=4.)
        if not eligible:
            self._metric_recovery=None
            return False
        previous=getattr(self,'_metric_recovery',None)
        count=1
        if previous is not None and previous[0]==candidate.target_uid:
            dt=(stamp-previous[1]).to_sec()
            count=previous[2] if dt==0. else previous[2]+1 if 0.<dt<=1. else 1
        self._metric_recovery=(candidate.target_uid,stamp,min(count,2))
        return count>=2

    def _validate_anchor(self, message, targets):
        if self._mode not in ("track", "fusion"):
            return
        frame_entry = self._find_frame(message.image_source, 0,
                                       message.header.stamp)
        visual = self._sot_at(message.header.stamp)
        if frame_entry is None or visual is None:
            return
        hard_conflict = (self._fusion_conflict and self._fusion_reason in (
            'ambiguous_competitor','appearance_conflict','appearance_unavailable',
            'identity_rebind_pending','identity_unavailable'))
        hard_conflict_reason = self._fusion_reason if hard_conflict else None
        measurements = {t.track_id:t for t in self._measured_targets(
            message.image_source,message.header,targets)}
        targets = [measurements.get(t.track_id,t) for t in targets]
        # An independently advanced LK observation can recover a failed SOT.
        # It must belong to this exact capture and public identity, not be an
        # old box, a prediction, or an anchor seeded by this very YOLO frame.
        pixel = next((entry for entry in reversed(
            getattr(self,'_pixel_history',{}).get(message.image_source,()))
            if entry[0] == message.header.stamp), None)
        if pixel is None:
            pixel = getattr(self, '_pixel_targets', {}).get(message.image_source)
        peer = next((t for t in pixel[1] if
                     t.track_id == self._selected_track_id and
                     t.target_uid == self._selected_uid and
                     t.lifecycle_state == 'confirmed' and
                     t.measurement_source == 'visual_tracker' and
                     t.tracking_quality >= .6 and
                     not t.last_detection_stamp.is_zero() and
                     t.last_detection_stamp < message.header.stamp), None) \
            if pixel is not None and pixel[0] == message.header.stamp else None
        raw = getattr(self, '_raw_sot', None)
        comparison = peer if peer is not None else (
            raw[1] if raw is not None and raw[0] == message.header.stamp else visual)
        candidates = [target for target in targets if target.measurement_source == "detector"
                      and target.lifecycle_state == 'confirmed'
                      and target.detection_confidence >= 0.5
                      and (self._sot_manual or target.track_id == self._selected_track_id)]
        ranked = sorted(((self._normalized_iou(comparison.bbox_cxcywh_norm,
                         target.bbox_cxcywh_norm), target) for target in candidates),
                        key=lambda item: item[0], reverse=True)
        if not ranked:
            self._metric_recovery=None
            candidate = self._rebind_selected_fragment(message, targets, visual,
                                                       comparison, frame_entry[2])
            if candidate is None:
                return
            ranked = [(self._normalized_iou(comparison.bbox_cxcywh_norm,
                                           candidate.bbox_cxcywh_norm), candidate)]
            hard_conflict = False
        candidate = ranked[0][1]
        # A small selected detection contained in another full-object
        # detection is not independent proof of two entities. A part/full
        # split may otherwise have a deceptively large best/second IoU margin.
        a = candidate.bbox_xyxy
        area = max(1, (a[2]-a[0])*(a[3]-a[1]))
        contained = False
        for other in targets:
            if (other.track_id == candidate.track_id or other.lifecycle_state != 'confirmed'
                    or other.measurement_source != 'detector' or other.detection_confidence < .5):
                continue
            b = other.bbox_xyxy
            other_area = max(1, (b[2]-b[0])*(b[3]-b[1]))
            intersection = max(0,min(a[2],b[2])-max(a[0],b[0])) * \
                           max(0,min(a[3],b[3])-max(a[1],b[1]))
            if area/other_area < .65 and intersection/area >= .8:
                contained = True
                break
        if contained:
            self._metric_recovery = None
            self._selected_identity_conflict(visual, 'ambiguous_competitor')
            return
        # Competitors of any identity participate in the ambiguity check.
        competing = [self._normalized_iou(comparison.bbox_cxcywh_norm, t.bbox_cxcywh_norm)
                     for t in targets if t.track_id != candidate.track_id
                     and t.lifecycle_state == "confirmed"]
        if competing and max(competing) >= .15 and ranked[0][0]-max(competing) < 0.15:
            self._metric_recovery=None
            self._fusion_reason = "ambiguous_competitor"
            self._fusion_conflict = True
            self._manual_target.lifecycle_state = "lost"
            self._manual_target.measurement_source = "stale"
            visual.lifecycle_state = "lost"
            return
        observed_signature = image_signature(frame_entry[2], candidate.bbox_xyxy)
        reference = (image_signature(frame_entry[2], peer.bbox_xyxy)
                     if peer is not None else self._reference_signature)
        accepted, reason = evidence_gate(
            comparison.bbox_xyxy, candidate.bbox_xyxy, 0.0,
            reference, observed_signature,same_capture=True)
        if accepted:
            self._metric_recovery=None
        elif self._metric_anchor_verified(candidate,message.header.stamp):
            accepted,reason=True,'metric_identity_recovery'
        self._fusion_reason = reason
        if not accepted:
            if hard_conflict and reason in ('motion_conflict','geometry_conflict'):
                # A later geometric rejection is not evidence resolving an
                # earlier identity conflict. Keep the latch for future frames.
                self._fusion_reason = hard_conflict_reason
            # Reject a geometric detector correction, not an actual, current
            # SOT observation. Its appearance must still match the last trusted
            # entity; hard identity conflicts cannot use this continuation.
            raw=getattr(self,'_raw_sot',None)
            current=raw[1] if raw is not None and raw[0]==message.header.stamp else visual
            current_signature=image_signature(frame_entry[2],current.bbox_xyxy)
            if (reason in ('motion_conflict','geometry_conflict') and not hard_conflict
                    and current.lifecycle_state=='confirmed'
                    and current.measurement_source=='visual_tracker'
                    and current.tracking_quality>=.5
                    and current_signature is not None and self._reference_signature is not None
                    and float(np.dot(current_signature,self._reference_signature))>=.55
                    and (peer is None or self._visual_consensus(current,peer,frame_entry[2]))):
                self._fusion_conflict=False
                self._fusion_reason='detector_geometry_correction_rejected'
                self._manual_target=deepcopy(current)
                for index,(stamp,_) in enumerate(self._sot_history):
                    if stamp==message.header.stamp:
                        self._sot_history[index]=(stamp,deepcopy(current))
                        break
                return
            if (not hard_conflict and reason in ('motion_conflict','geometry_conflict')
                    and peer is not None and visual.lifecycle_state == 'confirmed' and
                    visual.measurement_source in ('detector','visual_tracker') and
                    self._normalized_iou(visual.bbox_cxcywh_norm,
                                         peer.bbox_cxcywh_norm) >= .5):
                # A disagreeing YOLO box is not proof that two current visual
                # observers have lost the entity. Keep their verified consensus,
                # reject the detector correction and never update appearance.
                consistent, _ = evidence_gate(visual.bbox_xyxy,peer.bbox_xyxy,0.,
                    image_signature(frame_entry[2],visual.bbox_xyxy),reference,same_capture=True)
                if consistent:
                    self._fusion_reason = 'detector_outlier_' + reason
                    self._fusion_conflict = False
                    return
            self._fusion_conflict = True
            self._manual_target.lifecycle_state = "lost"
            self._manual_target.measurement_source = "stale"
            visual.lifecycle_state = "lost"
            return
        # Adapt only after identity, geometry, ambiguity and appearance gates.
        # Do not freeze the selected tank's appearance at its initial heading.
        if observed_signature is not None:
            previous = self._reference_signature
            adapted = observed_signature if previous is None or reason == 'metric_identity_recovery' else \
                .9 * previous + .1 * observed_signature
            norm = float(np.linalg.norm(adapted))
            if norm > 0:
                self._reference_signature = adapted / norm
        # Keep the selected public identity; the detector is a geometric anchor.
        anchored = deepcopy(candidate)
        if self._sot_manual:
            # Class is committed only by the global entity's three-frame binding.
            for field in ("class_id", "class_name", "class_valid"):
                setattr(anchored, field, getattr(self._manual_target, field))
        anchored.track_id = self._selected_track_id
        anchored.target_uid = self._selected_uid
        anchored.selected = True
        if (not self._sot_manual and not self._identity_attached and
                time.monotonic()-getattr(self,'_identity_attach_retry_wall',0.) >= .5):
            # Selection may precede the C++ detector-history callback or all
            # available anchors may be weak. Retry only after an accepted,
            # real same-ID detection, not on blind SOT or a new entity.
            self._attach_selected_identity(message.image_source, anchored, message.header.stamp)
        if self._identity_attached:
            # One authoritative geometric sample per detector capture; LK may
            # provide its geometry during an ID fork, before service binding.
            self._sync_visual(anchored, message.image_source, message.header.stamp,
                              self._reference_signature)
        visual_area = max(1., (visual.bbox_xyxy[2]-visual.bbox_xyxy[0]) *
                           (visual.bbox_xyxy[3]-visual.bbox_xyxy[1]))
        anchor_area = max(1., (candidate.bbox_xyxy[2]-candidate.bbox_xyxy[0]) *
                           (candidate.bbox_xyxy[3]-candidate.bbox_xyxy[1]))
        scale_consistent = .65 <= visual_area / anchor_area <= 1.55
        if (not self._fusion_conflict and
                visual.measurement_source in ("visual_tracker", "detector") and
                visual.lifecycle_state == "confirmed" and
                self._manual_target.lifecycle_state == "confirmed" and
                scale_consistent and
                self._normalized_iou(visual.bbox_cxcywh_norm,
                                     candidate.bbox_cxcywh_norm) >= 0.5):
            # A healthy SOT needs validation, not recreation on every YOLO
            # frame. Reinitializing CSRT then replaying images while holding
            # _lock stalls the entire scene and repeatedly trains its filters.
            for index, (captured, _) in enumerate(self._sot_history):
                if captured == message.header.stamp:
                    self._sot_history[index] = (captured, deepcopy(anchored))
                    break
            for field in ("class_id", "class_name", "class_valid",
                          "detection_confidence", "last_detection_stamp"):
                setattr(self._manual_target, field, getattr(anchored, field))
            x1, y1, x2, y2 = candidate.bbox_xyxy
            self._manual_tracker.accept_anchor((x1, y1, x2-x1, y2-y1))
            return
        tracker = SotTracker(self._requested_algorithm)
        x1, y1, x2, y2 = candidate.bbox_xyxy
        result = tracker.initialize(frame_entry[2], (x1,y1,x2-x1,y2-y1),
                                    message.header.stamp.to_sec())
        if result.valid:
            self._fusion_conflict = False
            self._manual_tracker = tracker
            self._manual_target = anchored
            self._active_algorithm = result.algorithm
            self._sot_history.clear()
            self._sot_history.append((message.header.stamp, deepcopy(anchored)))
            self._manual_last_valid_wall = time.monotonic()
            self._replay_sot(message.image_source, message.header.stamp)

    def _publish_targets(self, source, header, width, height):
        output_stamps = getattr(self, '_output_stamps', None)
        if output_stamps is not None and header.stamp < output_stamps.get(source, rospy.Time(0)):
            return
        output = PerceptionTargetArray()
        output.header = header
        paired = self._find_frame(source, 0, header.stamp)
        output.frame_seq = paired[0] if paired is not None else 0
        if paired is not None:
            height, width = paired[2].shape[:2]
        output.image_source = source
        output.image_width = width
        output.image_height = height
        output.session_id = self._session_id
        output.state_version = self._version
        output.model_epoch = self._model["epoch"]
        output.model_id = self._model["id"]
        output.mode = self._mode
        latest_header = self._latest_headers.get(source)
        pixel = getattr(self, '_pixel_targets', {}).get(source)
        pixel_paired = pixel is not None and pixel[0] == header.stamp
        # Fusion renders a complete, same-capture scene, like detect. The SOT
        # still updates on every image, but an early SOT-only publication would
        # erase all unselected targets until this frame's detector result arrives.
        # Never fill that gap by relabelling old boxes as current observations.
        source_expired = source in self._last_source_wall and \
            time.monotonic() - self._last_source_wall[source] > self._source_timeout
        if self._mode == "fusion" and not source_expired and not pixel_paired and (latest_header is None or
                latest_header.stamp != header.stamp):
            return
        ordinary = (deepcopy(self._latest_targets.get(source, [])) if latest_header
                    and latest_header.stamp == header.stamp else [])
        if pixel_paired:
            if latest_header is None or latest_header.stamp != header.stamp:
                ordinary = deepcopy(pixel[1])
            else:
                # A detector miss is not a veto of a real same-capture LK
                # measurement. Preserve actual detections; fill only missing
                # or predicted identities with independently advanced pixels.
                by_id = {t.track_id:t for t in ordinary}
                for t in pixel[1]:
                    previous = by_id.get(t.track_id)
                    measured = (t.lifecycle_state == 'confirmed' and
                        t.measurement_source == 'visual_tracker' and t.tracking_quality >= .6
                        and not t.last_detection_stamp.is_zero()
                        and t.last_detection_stamp < header.stamp)
                    if measured and (previous is None or previous.lifecycle_state != 'confirmed'
                            or previous.measurement_source not in ('detector','visual_tracker')):
                        by_id[t.track_id] = deepcopy(t)
                ordinary = list(by_id.values())
        if output_stamps is not None:
            output_stamps[source] = header.stamp
        registered = self._visual_targets.get(source)
        if registered and registered[0] == header.stamp:
            ordinary_ids = {t.track_id for t in ordinary}
            ordinary += [deepcopy(t) for t in registered[1] if t.track_id not in ordinary_ids]
        visual = deepcopy(self._sot_at(header.stamp)) if source == self._selected_source else None
        if self._mode == "detect":
            output.targets = ordinary
        elif self._mode == "track":
            observation = next((t for t in ordinary if t.target_uid == self._selected_uid
                and t.track_id == self._selected_track_id and t.lifecycle_state == 'confirmed'
                and t.measurement_source in ('detector','visual_tracker')), None)
            if (observation is not None and not self._fusion_conflict and
                    (visual is None or visual.lifecycle_state != 'confirmed'
                     or visual.measurement_source not in ('detector','visual_tracker'))):
                observation.selected = True
                output.targets = [observation]
            else:
                output.targets = [visual] if visual is not None else [t for t in ordinary if t.selected]
        else:
            output.targets = ordinary
            if visual is not None:
                selected_observation = next((t for t in ordinary if
                    t.track_id == self._selected_track_id and
                    t.target_uid == self._selected_uid and
                    t.lifecycle_state == 'confirmed' and
                    t.measurement_source in ('detector', 'visual_tracker')), None)
                # A failed local SOT must not erase valid same-frame evidence
                # of the same globally managed entity. Hard identity conflicts
                # still suppress recovery; no nearest-box or new-ID fallback.
                prefer_observation = (selected_observation is not None and
                    not self._fusion_conflict and
                    (visual.lifecycle_state != 'confirmed' or
                     visual.measurement_source not in ('detector', 'visual_tracker')))
                if prefer_observation:
                    selected_observation.selected = True
                else:
                    output.targets = [target for target in output.targets
                                      if target.track_id != self._selected_track_id]
                    output.targets.append(visual)
        stale_source = source_expired
        for target in output.targets:
            key = (source, target.track_id)
            declared_observation = target.last_detection_stamp
            previous = self._image_evidence.get(key)
            if not declared_observation.is_zero() and declared_observation <= header.stamp and \
                    (previous is None or declared_observation > previous):
                self._image_evidence[key] = declared_observation
                self._image_evidence.move_to_end(key)
            if not stale_source and target_status(target) in ("DET", "SOT"):
                previous = self._image_evidence.get(key)
                if previous is None or header.stamp > previous:
                    self._image_evidence[key] = header.stamp
                    self._image_evidence.move_to_end(key)
            bound_prediction(target, header.stamp, self._image_evidence.get(key),
                             self._prediction_horizon, stale_source)
            target.age_ms = observation_age_ms(header.stamp,
                self._image_evidence.get(key), rospy.Time.now())
            if target_status(target) == "PRED" and (target.age_ms < 0.0 or
                    target.age_ms > self._prediction_horizon * 1000.0):
                identity_only(target)
            if target.position_valid and observation_age_ms(header.stamp,
                    target.position_stamp, rospy.Time.now()) > self._position_timeout * 1000.0:
                target.position_valid = False
        while len(self._image_evidence) > 512:
            self._image_evidence.popitem(last=False)
        received=getattr(self,'_frame_receipts',{}).get(source,{}).get(header.stamp.to_nsec())
        if received is not None:
            self._presentation_ms.append(1000*(time.monotonic()-received))
        self._targets_pub.publish(output)
        if self._mode in ("track", "fusion") and source == self._selected_source:
            selected = next((item for item in output.targets if item.target_uid == self._selected_uid), None)
            if selected is not None:
                self._selected_output = (time.monotonic(), deepcopy(selected))
                previous_status = self._trajectory.status
                self._trajectory.append(selected, header, source, output.frame_seq)
                self._publish_trajectory(force=previous_status != self._trajectory.status)
        self._publish_state()

    def _publish_trajectory(self, force=False):
        now_wall = time.monotonic()
        if not force and now_wall - self._last_trajectory_publish < self._trajectory_publish_interval:
            return
        now = rospy.Time.now()
        self._trajectory.expire(now)
        output = TargetTrajectoryArray()
        output.header.stamp = now
        output.session_id, output.state_version = self._session_id, self._version
        output.mode, output.model_epoch = self._mode, self._model["epoch"]
        output.target_uid, output.track_id = self._trajectory.uid, self._trajectory.track_id
        output.class_id, output.class_name, output.class_valid = (
            self._trajectory.class_id, self._trajectory.class_name, self._trajectory.class_valid)
        output.active, output.status = self._trajectory.active, self._trajectory.status
        output.retention_sec, output.maximum_points = self._trajectory.retention_sec, self._trajectory.maximum_points
        output.points = list(self._trajectory.points)
        self._trajectory_pub.publish(output)
        self._last_trajectory_publish = now_wall

    def _heartbeat(self, _event):
        with self._lock:
            pending = getattr(self, "_model_transaction", None)
            if pending is not None and time.monotonic() > self._model_deadline:
                self._model_transaction = None
                self._transaction_event.set()
                self._last_error = "model completion timed out; query runtime state before retrying"
                self._publish_result(pending[0].request_id, "model", "FAILED", "TIMEOUT",
                                     self._last_error, pending[1])
            for source, last_image in list(self._last_source_wall.items()):
                if time.monotonic() - last_image <= self._source_timeout or source in self._stale_notified:
                    continue
                self._stale_notified.add(source)
                frames = self._frames[source]
                if frames:
                    header = self._latest_headers.get(source)
                    if header is None or header.stamp != frames[-1][1]:
                        header = Header(seq=frames[-1][0], stamp=frames[-1][1])
                    self._publish_targets(source, header, frames[-1][2].shape[1], frames[-1][2].shape[0])
            self._publish_state()

            self._publish_trajectory()

            worker = getattr(self, '_pixel_worker', None)
            statuses=[]
            if self._selected_uid:
                raw = getattr(self,'_raw_sot',None)
                pixel = getattr(self,'_pixel_targets',{}).get(self._selected_source)
                peer = next((t for t in pixel[1] if t.target_uid == self._selected_uid),None) \
                    if pixel is not None else None
                values = dict(global_id=self._selected_track_id, image_source=self._selected_source,
                    observer_attached=self._identity_attached, manual_roi=self._sot_manual,
                    fusion_conflict=self._fusion_conflict, fusion_reason=self._fusion_reason,
                    identity_rejection=getattr(self,'_last_identity_rejection',''),
                    raw_sot_valid=bool(raw and raw[1].lifecycle_state=='confirmed'),
                    sot_failure=getattr(self,'_sot_failure','not_updated'),
                    pixel_lifecycle=peer.lifecycle_state if peer else 'absent',
                    pixel_quality=peer.tracking_quality if peer else 0.,
                    pixel_capture_stamp=pixel[0].to_sec() if pixel else 0.)
                statuses.append(DiagnosticStatus(name='perception/selected_identity',
                    level=DiagnosticStatus.WARN if self._fusion_conflict else DiagnosticStatus.OK,
                    message='selected identity evidence; no stale-box substitution',
                    values=[KeyValue(key=str(k),value=str(v)) for k,v in values.items()]))
            if worker is not None:
                status = DiagnosticStatus(name='perception/high_frequency_pixels',
                    level=DiagnosticStatus.OK if worker.thread.is_alive() else DiagnosticStatus.ERROR,
                    message='latest-only image observations; never detector freshness')
                status.values=[KeyValue(key=str(key),value=str(value)) for key,value in worker.snapshot().items()]
                statuses.append(status)
            if hasattr(self,'_pixel_diagnostics_pub') and self._presentation_ms:
                timings=sorted(self._presentation_ms)
                status=DiagnosticStatus(name='perception/image_to_targets',level=DiagnosticStatus.OK,
                    message='application camera callback to target publication; excludes capture/network latency',
                    values=[KeyValue(key='callback_to_targets_p95_ms',value=str(timings[
                        min(len(timings)-1,int(.95*len(timings)))]))])
                statuses.append(status)
            if statuses:
                self._pixel_diagnostics_pub.publish(DiagnosticArray(header=Header(stamp=rospy.Time.now()),
                    status=statuses))

    def _model_callback(self, message):
        if not message.status:
            return
        values = {item.key: item.value for item in message.status[0].values}
        with self._lock:
            if getattr(self, "_model_transaction", None) is not None:
                return
            try:
                epoch = int(values.get("model_epoch", "0"))
                classes = json.loads(values.get("class_names", "[]"))
                if epoch < self._model["epoch"]:
                    return
                if epoch > self._model["epoch"] and self._model["epoch"] > 0:
                    self._version += 1
                if classes:
                    self._apply_model_contract(epoch, classes)
            except (ValueError, TypeError):
                return
            self._model["id"] = values.get("active_model_id", "unknown")
            self._model["sha256"] = values.get(
                "source_model_sha256", values.get("loaded_artifact_sha256", ""))
            self._model["backend"] = values.get("backend", "unknown")
            try:
                self._model["epoch"] = int(values.get("model_epoch", "0"))
            except ValueError:
                self._model["epoch"] = 0
            self._model["ready"] = values.get("state") == "READY"
            self._publish_state()

    def _model_command_callback(self, message):
        with self._lock:
            valid, error, reason, fingerprint = self._validate_command(message, "model")
            if not valid:
                if error != "DUPLICATE":
                    self._reject(message, "model", error, reason, self._fingerprint(message))
                return
            if not message.model_id.strip():
                self._reject(message, "model", "INVALID_MODEL", "model_id is required", fingerprint)
                return
            if self._model_request_pub.get_num_connections() == 0:
                self._reject(message, "model", "UNAVAILABLE", "shared detector is not connected", fingerprint)
                return
            self._model_transaction = (deepcopy(message), fingerprint)
            forwarded = deepcopy(message)
            # A missing client TTL must not allow an unbounded preparation.
            if forwarded.issued_at.is_zero():
                forwarded.issued_at = rospy.Time.now()
            forwarded.ttl_sec = min(28.0, forwarded.ttl_sec) if forwarded.ttl_sec > 0 else 28.0
            self._model_deadline = time.monotonic() + 30.0
            self._model_request_pub.publish(forwarded)

    def _initialize_switch_runtime(self, reason):
        """Soft initialization: release frame/history buffers, not entity IDs.

        Keep only each source's newest image to seed trustworthy SOT templates.
        Old capture headers remain as rejection barriers for delayed track arrays.
        A switch is not an observation and must not extend the lock lifetime.
        """
        self._clear_pixels()
        sot_stamp = (self._sot_history[-1][0] if self._sot_history else
                     self._manual_target.last_detection_stamp if self._manual_target else None)
        for source, frames in self._frames.items():
            if frames:
                newest = frames[-1]
                frames.clear()
                frames.append(newest)
                previous = self._latest_headers.get(source)
                if previous is None or previous.stamp < newest[1]:
                    self._latest_headers[source] = Header(seq=newest[0], stamp=newest[1])
        for cache in (self._latest_targets, self._track_history,
                      self._visual_targets, self._image_evidence):
            cache.clear()
        self._sot_history.clear()
        self._fusion_conflict, self._fusion_reason = False, ""
        self._selected_output = None
        self._trajectory.points.clear()
        self._trajectory.last_capture, self._trajectory.last_source = None, ""
        self._trajectory.close(reason)
        if self._manual_tracker is not None and self._manual_target is not None:
            target = self._manual_target
            captured = self._find_frame(self._selected_source, 0, sot_stamp)
            old = self._manual_tracker
            replacement = SotTracker(self._requested_algorithm)
            valid = False
            if captured is not None and target.lifecycle_state == "confirmed":
                x1, y1, x2, y2 = target.bbox_xyxy
                result = replacement.initialize(captured[2],
                    (x1, y1, x2-x1, y2-y1), captured[1].to_sec())
                valid = result.valid
            old.reset()
            self._manual_tracker = replacement
            if valid:
                self._active_algorithm = result.algorithm
                self._sot_history.append((captured[1], deepcopy(target)))
                self._reference_signature = image_signature(captured[2], target.bbox_xyxy)
            else:
                target.lifecycle_state, target.measurement_source = "lost", "stale"
                target.position_valid, target.tracking_quality = False, 0.0
                self._reference_signature = None

    def _apply_model_contract(self, epoch, class_names):
        if epoch <= self._model["epoch"]:
            return
        self._class_names = list(class_names)
        self._latest_targets.clear()
        self._latest_headers.clear()
        self._track_history.clear()
        self._visual_targets.clear()
        self._sot_history.clear()
        for target in ([self._manual_target] if self._manual_target else []) + \
                [item["target"] for item in self._background_visual.values()]:
            target.class_id, target.class_name, target.class_valid = -1, "unknown", False
        self._trajectory.class_id = -1
        self._trajectory.class_name = "unknown"
        self._trajectory.class_valid = False
        self._initialize_switch_runtime("model_switch")

    def _model_commit_callback(self, message):
        with self._lock:
            pending = self._model_transaction
            if (pending is None or message.session_id != self._session_id or
                    message.request_id != pending[0].request_id):
                return
            command, fingerprint = pending
            self._model_transaction = None
            if hasattr(self, "_transaction_event"):
                self._transaction_event.set()
            if message.success:
                self._apply_model_contract(message.model_epoch, message.class_names)
                self._model.update(id=message.model_id, sha256=message.source_sha256,
                    backend=message.backend, epoch=message.model_epoch, ready=True)
                self._version += 1
                self._last_error = ""
                self._publish_result(command.request_id, "model", "APPLIED", "",
                                     message.message, fingerprint)
                self._publish_trajectory(force=True)
            else:
                self._last_error = message.message
                self._publish_result(command.request_id, "model", "FAILED",
                    message.error_code, message.message, fingerprint)
            self._publish_state()

    def _operating_state(self):
        if self._mode == "detect":
            return "ACTIVE"
        if not self._selected_uid:
            return "WAITING_SELECTION" if self._mode == "track" else "ACTIVE"
        selected_output = getattr(self, '_selected_output', None)
        if (selected_output is not None and not self._fusion_conflict and
                time.monotonic()-selected_output[0] <= self._source_timeout and
                selected_output[1].target_uid == self._selected_uid and
                selected_output[1].lifecycle_state == 'confirmed' and
                selected_output[1].measurement_source in ('detector', 'visual_tracker')):
            return 'TRACKING'
        if self._manual_target is not None:
            if time.monotonic()-self._manual_last_valid_wall > self._source_timeout:
                return "LOST"
            return ("TRACKING" if self._manual_target.lifecycle_state == "confirmed"
                    else "LOST")
        targets = self._latest_targets.get(self._selected_source, [])
        selected = next((item for item in targets
                         if item.target_uid == self._selected_uid), None)
        return ("TRACKING" if selected is not None and
                selected.lifecycle_state in ("confirmed", "occluded") else "LOST")

    def _publish_state(self):
        self._state_pub.publish(self._state_snapshot())

    def _get_state(self, _request):
        return GetPerceptionStateResponse(success=True, message="current runtime state",
                                          state=self._state_snapshot())

    def _state_snapshot(self):
        with self._lock:
            state = PerceptionState()
            state.header.stamp = rospy.Time.now()
            state.session_id = self._session_id
            state.state_version = self._version
            state.mode = self._mode
            state.operating_state = self._operating_state()
            state.selection_active = bool(self._selected_uid)
            state.selected_target_uid = self._selected_uid
            state.selected_track_id = self._selected_track_id
            state.selected_image_source = self._selected_source
            state.requested_tracker_algorithm = self._requested_algorithm
            state.active_tracker_algorithm = self._active_algorithm
            state.active_model_id = self._model["id"]
            state.active_model_sha256 = self._model["sha256"]
            state.active_model_backend = self._model["backend"]
            state.model_epoch = self._model["epoch"]
            state.model_ready = self._model["ready"]
            state.last_error = self._last_error or self._fusion_reason
            commands = getattr(self, "_commands", None)
            state.active_request_id, state.queued_commands = commands.snapshot() if commands else ("", 0)
            state.transaction_state = ("MODEL_PREPARING" if getattr(self, "_model_transaction", None) is not None
                else ("EXECUTING" if state.active_request_id else "IDLE"))
            return state


def main():
    rospy.init_node("perception_mode_manager")
    PerceptionModeManager()
    rospy.spin()


if __name__ == "__main__":
    main()
