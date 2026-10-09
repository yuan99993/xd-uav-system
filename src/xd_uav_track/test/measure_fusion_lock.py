#!/usr/bin/env python3
"""Bounded selected-ID continuity probe; does not assert ground-truth identity."""
import argparse
from collections import Counter, OrderedDict
import json
import threading
import time
import uuid
import cv2
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from xd_uav_track.msg import (PerceptionState, PerceptionTargetArray,
    SetPerceptionModeCommand, SelectPerceptionTargetCommand, PerceptionVideoFrame)


class RevisionContinuity:
    """Count every update, especially a later LOST revision of a good capture."""
    def __init__(self):
        self.captures = OrderedDict()
        self.updates = self.missing = self.downgrades = 0
        self.failures = []

    def record(self, message, uid, revision=None):
        key = message.header.stamp.to_nsec()
        target = next((t for t in message.targets if t.target_uid == uid),None)
        valid = bool(target is not None and target.lifecycle_state == 'confirmed' and
            target.measurement_source in ('detector','visual_tracker') and
            target.bbox_xyxy[2]>target.bbox_xyxy[0] and target.bbox_xyxy[3]>target.bbox_xyxy[1])
        if self.captures.get(key) is True and not valid:
            self.downgrades += 1
        self.captures[key] = valid
        while len(self.captures)>600:
            self.captures.popitem(last=False)
        self.updates += 1
        self.missing += int(not valid)
        if not valid and len(self.failures)<32:
            self.failures.append({'stamp':message.header.stamp.to_sec(),'revision':revision,
                'source':target.measurement_source if target else 'absent',
                'lifecycle':target.lifecycle_state if target else 'absent',
                'box':list(target.bbox_xyxy) if target else None})

    def report(self):
        return {'updates':self.updates,'missing_updates':self.missing,
                'same_capture_downgrades':self.downgrades,
                'retained_captures':len(self.captures),
                'final_missing_captures':sum(not v for v in self.captures.values()),
                'first_missing_updates':self.failures}


def overlapping_confirmed_peers(message, uid):
    """Diagnostic overlap only, not a truth-based duplicate/ID error metric."""
    chosen = next((t for t in message.targets if t.target_uid == uid and
                   t.lifecycle_state == 'confirmed'), None)
    if chosen is None:
        return 0
    def area(box):
        return max(0, box[2]-box[0])*max(0, box[3]-box[1])
    count = 0
    a = chosen.bbox_xyxy
    for peer in message.targets:
        if peer.target_uid == uid or peer.lifecycle_state != 'confirmed':
            continue
        b = peer.bbox_xyxy
        intersection = max(0,min(a[2],b[2])-max(a[0],b[0])) * \
                       max(0,min(a[3],b[3])-max(a[1],b[1]))
        if intersection / max(1,area(a)+area(b)-intersection) >= .15:
            count += 1
    return count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=45.)
    parser.add_argument('--output', required=True)
    parser.add_argument('--select-first', action='store_true')
    parser.add_argument('--track-id',type=int,default=None)
    parser.add_argument('--stable-detections',type=int,default=3,
                        help='distinct real detector captures before automatic test selection')
    parser.add_argument('--image-source', default='search_fixed')
    parser.add_argument('--roi-padding', type=float, default=0.,
                        help='test coarse ROI selection instead of ID (e.g. 3)')
    args = parser.parse_args()
    rospy.init_node('fusion_lock_probe', anonymous=True)
    lock = threading.Lock()
    runtime = {'state': None, 'targets': None, 'selection_targets': None}
    stable = {}
    stats = {'frames': 0, 'observed': 0, 'missing': 0, 'prediction': 0,
             'max_missing_capture_sec': 0., 'max_observed_gap_capture_sec': 0.}
    sources, reasons = Counter(), Counter()
    identity_diagnostics = Counter()
    external_selections = set()
    target_revisions, video_revisions = RevisionContinuity(), RevisionContinuity()
    draw_checks = {'near_header_checked':0,'near_header_without_green_pixels':0}
    overlaps = {'target_updates_with_overlap':0, 'video_updates_with_overlap':0}
    selected = {'uid': '', 'source': '', 'last_stamp': None,
                'last_observed': None, 'missing_start': None}

    def state(message):
        with lock:
            runtime['state'] = message
            if selected['uid']:
                reasons[message.last_error] += 1
                if (message.selected_target_uid != selected['uid'] or
                        message.selected_image_source != selected['source'] or message.mode != 'fusion'):
                    external_selections.add((message.mode,message.selected_image_source,message.selected_target_uid))

    def targets(message):
        with lock:
            runtime['targets'] = message if message.image_source == args.image_source else runtime['targets']
            if message.image_source == args.image_source:
                measured=[t for t in message.targets if t.lifecycle_state=='confirmed'
                    and t.measurement_source=='detector' and t.detection_confidence>=.5]
                if measured:
                    runtime['selection_targets']=message
                    for t in measured:
                        count,previous=stable.get(t.target_uid,(0,None))
                        if previous is None or message.header.stamp>previous:
                            count=count+1 if previous is not None and (message.header.stamp-previous).to_sec()<=1.25 else 1
                            stable[t.target_uid]=(count,message.header.stamp)
                expired=[uid for uid,(_,stamp) in stable.items()
                         if (message.header.stamp-stamp).to_sec()>1.25]
                for uid in expired: del stable[uid]
            if not selected['uid'] or message.image_source != selected['source']:
                return
            target_revisions.record(message,selected['uid'])
            overlaps['target_updates_with_overlap'] += int(overlapping_confirmed_peers(message, selected['uid']) > 0)
            stamp = message.header.stamp.to_sec()
            if selected['last_stamp'] is not None and stamp <= selected['last_stamp']:
                return
            selected['last_stamp'] = stamp
            stats['frames'] += 1
            target = next((t for t in message.targets if t.target_uid == selected['uid']), None)
            valid = target is not None and target.lifecycle_state == 'confirmed' and \
                target.measurement_source in ('detector', 'visual_tracker')
            if valid:
                stats['observed'] += 1
                sources[target.measurement_source] += 1
                if selected['last_observed'] is not None:
                    stats['max_observed_gap_capture_sec'] = max(stats['max_observed_gap_capture_sec'],
                        stamp-selected['last_observed'])
                selected['last_observed'] = stamp
                if selected['missing_start'] is not None:
                    stats['max_missing_capture_sec'] = max(stats['max_missing_capture_sec'],
                        stamp-selected['missing_start'])
                    selected['missing_start'] = None
            else:
                stats['missing'] += 1
                if target is not None and target.measurement_source == 'prediction':
                    stats['prediction'] += 1
                if selected['missing_start'] is None:
                    selected['missing_start'] = stamp
                stats['max_missing_capture_sec'] = max(stats['max_missing_capture_sec'],
                    stamp-selected['missing_start'])

    def video(message):
        with lock:
            if selected['uid'] and message.targets.image_source == selected['source']:
                video_revisions.record(message.targets,selected['uid'],message.frame_revision)
                overlaps['video_updates_with_overlap'] += int(overlapping_confirmed_peers(message.targets, selected['uid']) > 0)
                item=next((t for t in message.targets.targets if t.target_uid==selected['uid']),None)
                if (item is not None and item.selected and item.lifecycle_state=='confirmed'
                        and item.measurement_source in ('detector','visual_tracker')
                        and item.bbox_xyxy[1]<50):
                    image=cv2.imdecode(np.frombuffer(bytes(message.image.data),np.uint8),cv2.IMREAD_COLOR)
                    if image is not None:
                        x1,y1,x2,y2=item.bbox_xyxy
                        crop=image[max(0,y1):min(image.shape[0],y2),max(0,x1):min(image.shape[1],x2)]
                        if crop.size:
                            draw_checks['near_header_checked']+=1
                            green=(crop[:,:,1]>180)&(crop[:,:,0]<100)&(crop[:,:,2]<100)
                            draw_checks['near_header_without_green_pixels']+=int(not green.any())

    def diagnostics(message):
        with lock:
            if not selected['uid']:
                return
            for status in message.status:
                if status.name != 'perception/selected_identity':
                    continue
                values = {item.key:item.value for item in status.values}
                if values.get('image_source') != selected['source']:
                    continue
                # Aggregate evidence states, not every unique camera stamp
                # and floating-point quality value (bounded diagnostic load).
                values.pop('pixel_capture_stamp',None)
                if 'pixel_quality' in values:
                    values['pixel_quality_band']='qualified' if float(values.pop('pixel_quality'))>=.6 else 'weak'
                identity_diagnostics[json.dumps(values, sort_keys=True)] += 1

    rospy.Subscriber('/moving_uav_search/perception/video',PerceptionVideoFrame,video,queue_size=5)
    rospy.Subscriber('/moving_uav_search/perception/state', PerceptionState, state, queue_size=5)
    rospy.Subscriber('/moving_uav_search/perception/targets', PerceptionTargetArray, targets, queue_size=5)
    rospy.Subscriber('/moving_uav_search/perception/pixel_diagnostics',DiagnosticArray,diagnostics,queue_size=2)
    mode_pub = rospy.Publisher('/moving_uav_search/perception/mode/set', SetPerceptionModeCommand, queue_size=1)
    select_pub = rospy.Publisher('/moving_uav_search/perception/target/select', SelectPerceptionTargetCommand, queue_size=1)
    deadline = time.monotonic()+45.
    issued_mode, issued_selection = False, False
    requested_uid = None
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        with lock:
            current, snapshot = runtime['state'], runtime['selection_targets']
            qualified={uid for uid,(count,_) in stable.items()
                       if count>=max(1,args.stable_detections)}
        if current is None:
            time.sleep(.1)
            continue
        if args.select_first:
            if current.mode != 'fusion':
                if not issued_mode and mode_pub.get_num_connections():
                    mode_pub.publish(SetPerceptionModeCommand(request_id=uuid.uuid4().hex,
                        session_id=current.session_id,expected_state_version=current.state_version,
                        issued_at=rospy.Time.now(),ttl_sec=10.,mode='fusion'))
                    issued_mode = True
            elif (not issued_selection and snapshot is not None and select_pub.get_num_connections()
                  and snapshot.session_id == current.session_id
                  and snapshot.state_version == current.state_version
                  and snapshot.model_epoch == current.model_epoch):
                candidate = next((t for t in snapshot.targets if t.lifecycle_state == 'confirmed' and
                    t.class_valid and
                    0 < t.bbox_cxcywh_norm[0] < 1 and 0 < t.bbox_cxcywh_norm[1] < 1 and
                    t.bbox_cxcywh_norm[2] > 0 and t.bbox_cxcywh_norm[3] > 0 and
                    (args.track_id is None or t.track_id==args.track_id) and
                    t.measurement_source == 'detector' and t.detection_confidence>=.5 and
                    t.target_uid in qualified),None)
                if candidate is not None:
                    selection_type='track_id'
                    roi=[0.,0.,0.,0.]
                    if args.roi_padding > 0:
                        cx,cy,bw,bh=candidate.bbox_cxcywh_norm
                        left=max(0.,cx-.5*bw*args.roi_padding)
                        top=max(0.,cy-.5*bh*args.roi_padding)
                        right=min(1.,cx+.5*bw*args.roi_padding)
                        bottom=min(1.,cy+.5*bh*args.roi_padding)
                        roi=[left,top,right-left,bottom-top]
                        selection_type='roi'
                    select_pub.publish(SelectPerceptionTargetCommand(request_id=uuid.uuid4().hex,
                        session_id=current.session_id, expected_state_version=current.state_version,
                        issued_at=rospy.Time.now(),ttl_sec=10.,selection_type=selection_type,
                        track_id=candidate.track_id,target_uid=candidate.target_uid,
                        frame_seq=snapshot.frame_seq,capture_stamp=snapshot.header.stamp,
                        roi_norm=roi,image_source=snapshot.image_source,tracker_algorithm='auto'))
                    issued_selection = True
                    requested_uid = candidate.target_uid
        if current.mode == 'fusion' and current.selection_active and \
                (not args.select_first or (issued_selection and
                 current.selected_target_uid == requested_uid and
                 current.selected_image_source == args.image_source)):
            with lock:
                selected['uid'],selected['source'] = current.selected_target_uid,current.selected_image_source
            break
        time.sleep(.1)
    if not selected['uid']:
        raise RuntimeError('no active fusion selection within 45 seconds')
    start = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic()-start < args.seconds:
        time.sleep(.1)
    with lock:
        result = dict(stats, selected_uid=selected['uid'], image_source=selected['source'],
            schema_version=2,target_revisions=target_revisions.report(),video_revisions=video_revisions.report(),
            draw_checks=draw_checks, overlap_diagnostics=overlaps,
            external_selection_changes=[list(item) for item in sorted(external_selections)],
            selection_unchanged=not external_selections,
            duration_wall_sec=time.monotonic()-start, measurement_sources=dict(sources),
            state_reasons=dict(reasons),scope='selected public-ID continuity only; not true identity or visibility accuracy')
        result['identity_diagnostics']=[dict(values=json.loads(key),samples=count)
            for key,count in identity_diagnostics.items()]
    with open(args.output,'w') as handle:
        json.dump(result,handle,indent=2,sort_keys=True)
    print(json.dumps(result,indent=2,sort_keys=True))


if __name__ == '__main__':
    main()
