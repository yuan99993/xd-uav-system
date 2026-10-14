#!/usr/bin/env python3
"""Bounded scene plumbing observation, not a truth-labelled accuracy score."""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import threading
import time

import rospy
from sensor_msgs.msg import Image
from sar_yolo_detector.msg import SmartTrackerState, TrackedDetection2DArray
from sar_yolo_detector.srv import SelectSmartTrack, SelectSmartTrackRequest
from std_msgs.msg import String
from xd_uav_track.msg import DetectionArray, TrackStateArray
from sar_yolo_detector.fusion_metrics import FusionAcceptance, capture_key


def load_truth(path, target_id):
    if not path:
        return {}
    source = Path(path)
    if not target_id or source.stat().st_size > 8*1024*1024:
        raise ValueError('Truth requires --truth-target-id and a file <= 8 MiB')
    rows = json.loads(source.read_text(encoding='utf-8'))
    if not isinstance(rows, list) or len(rows) > 10000:
        raise ValueError('Truth must be a list of <= 10000 capture records')
    truth = {}
    for row in rows:
        key = capture_key(row)
        if key in truth or not isinstance(row.get('objects'), list) or len(row['objects']) > 100:
            raise ValueError('Duplicate capture or invalid truth object count')
        identities = set()
        for obj in row['objects']:
            identity = str(obj['id'])
            if identity in identities or not isinstance(obj.get('visible'), bool):
                raise ValueError('Duplicate identity or missing boolean visibility')
            identities.add(identity)
            if obj['visible']:
                box = obj.get('box', [])
                if (len(box) != 4 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in box)
                        or box[2] <= 0 or box[3] <= 0):
                    raise ValueError('Visible truth needs a finite x,y,width,height box')
        truth[key] = row
    return truth


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=45.)
    parser.add_argument('--output', required=True)
    parser.add_argument('--snapshot', default='')
    parser.add_argument('--auto-select', action='store_true')
    parser.add_argument('--selected-id', type=int)
    parser.add_argument('--truth-annotations', default='')
    parser.add_argument('--truth-target-id', default='')
    parser.add_argument('--require-tracking', action='store_true')
    parser.add_argument('--minimum-in-view-rate', type=float, default=.95)
    parser.add_argument('--minimum-truth-coverage', type=float, default=.95)
    parser.add_argument('--minimum-recovered-events', type=int, default=0)
    args = parser.parse_args()
    if not 5 <= args.duration <= 120:
        parser.error('duration must be 5..120 seconds')
    if not 0 <= args.minimum_in_view_rate <= 1 or not 0 <= args.minimum_truth_coverage <= 1:
        parser.error('coverage thresholds must be 0..1')
    if not 0 <= args.minimum_recovered_events <= 1000:
        parser.error('minimum-recovered-events must be 0..1000')
    try:
        truth = load_truth(args.truth_annotations, args.truth_target_id)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))
    acceptance = FusionAcceptance(truth=truth, truth_target_id=args.truth_target_id)
    rospy.init_node('formal_fusion_observer', anonymous=True)
    ns = '/moving_uav_search'
    base = ns + '/sar_yolo_detector/fusion_fixed'
    node = ns + '/sar_yolo_smart_tracker_fusion_fixed'
    lock = threading.RLock()
    counts, sources = Counter(), Counter()
    processing = deque(maxlen=600)
    latest, identities = {}, set()
    dimensions, raw_dimensions = set(), set()
    metric_valid = 0
    duplicate_frames = 0
    class_overlap_frames = 0
    suppressed_class_duplicates = 0

    def overlapping_classes(a, b):
        if not a.detection.results or not b.detection.results:
            return False
        ca, cb = a.detection.results[0].id, b.detection.results[0].id
        if ca == cb or ca not in (0, 1, 2, 3) or cb not in (0, 1, 2, 3):
            return False
        a, b = a.detection.bbox, b.detection.bbox
        intersection = (max(0., min(a.center.x+a.size_x/2., b.center.x+b.size_x/2.) -
                            max(a.center.x-a.size_x/2., b.center.x-b.size_x/2.)) *
                        max(0., min(a.center.y+a.size_y/2., b.center.y+b.size_y/2.) -
                            max(a.center.y-a.size_y/2., b.center.y-b.size_y/2.)))
        union = a.size_x*a.size_y + b.size_x*b.size_y - intersection
        return (intersection / max(1., union) >= .75 and
                abs(a.center.x-b.center.x) <= .20*min(a.size_x, b.size_x) and
                abs(a.center.y-b.center.y) <= .20*min(a.size_y, b.size_y))

    def receive(kind):
        def callback(message):
            nonlocal metric_valid, duplicate_frames, class_overlap_frames, suppressed_class_duplicates
            with lock:
                counts[kind] += 1
                latest[kind] = message
                if kind == 'state':
                    processing.append(message.frame_processing_ms)
                    if message.has_selection:
                        identities.add(message.selected_track_id)
                elif kind == 'evidence':
                    evidence = json.loads(message.data)
                    acceptance.observe(evidence)
                    sources[evidence['source']] += 1
                    suppressed_class_duplicates += evidence.get('suppressed_class_duplicates', 0)
                elif kind == 'projected':
                    metric_valid += sum(c.has_relative_position_body for c in message.candidates)
                elif kind == 'image':
                    dimensions.add((message.width, message.height))
                elif kind == 'raw':
                    raw_dimensions.add((message.width, message.height))
                    acceptance.add_raw(dict(frame_id=message.header.frame_id,
                        stamp=dict(secs=message.header.stamp.secs, nsecs=message.header.stamp.nsecs)))
                elif kind == 'detections':
                    ids = [d.track_id for d in message.detections if d.track_id >= 0]
                    duplicate_frames += len(ids) != len(set(ids))
                    detections = message.detections
                    class_overlap_frames += any(overlapping_classes(a, b)
                        for index, a in enumerate(detections) for b in detections[index+1:])
        return callback

    specifications = [('raw', ns+'/fixed_camera/image_raw', Image),
                      ('image', base+'/annotated', Image),
                      ('detections', base+'/tracked_detections', TrackedDetection2DArray),
                      ('state', base+'/state', SmartTrackerState),
                      ('evidence', node+'/fusion_evidence', String),
                      ('bridge', ns+'/detect/input/fixed/detections_2d', DetectionArray),
                      ('projected', ns+'/track/detections', DetectionArray),
                      ('tracks', ns+'/track/tracks_by_source', TrackStateArray)]
    subscribers = [rospy.Subscriber(topic, cls, receive(kind), queue_size=1, tcp_nodelay=True)
                   for kind, topic, cls in specifications]
    selected = None
    errors = deque(maxlen=6)
    select = rospy.ServiceProxy(node+'/select', SelectSmartTrack)
    start = time.monotonic()
    while time.monotonic() - start < args.duration and not rospy.is_shutdown():
        if (args.auto_select or args.selected_id is not None) and selected is None:
            with lock:
                detections = latest.get('detections')
                candidates = [] if detections is None else [d for d in detections.detections
                    if d.track_id >= 0 and d.track_id_is_stable and d.detection.results
                    and d.detection.results[0].score >= .5
                    and (args.selected_id is None or d.track_id == args.selected_id)]
                fresh = detections is not None and 0 <= (rospy.Time.now()-detections.header.stamp).to_sec() < .5
            if candidates and fresh:
                try:
                    candidate = max(candidates, key=lambda d: d.detection.results[0].score)
                    result = select(SelectSmartTrackRequest(use_target_id=True, target_id=candidate.track_id))
                    if result.success:
                        selected = result.selected_target_id
                    else:
                        errors.append(result.message)
                except rospy.ServiceException as exc:
                    errors.append(str(exc))
        time.sleep(.1)
    with lock:
        state = latest.get('state')
        latencies = sorted(processing)
        report = dict(kind='gazebo_plumbing_not_identity_accuracy', duration_sec=time.monotonic()-start,
                      message_counts=dict(counts), fusion_sources=dict(sources), selected_id=selected,
                      selected_ids_seen=sorted(identities), duplicate_id_frames=duplicate_frames,
                      different_class_overlap_frames=class_overlap_frames,
                      suppressed_class_duplicates=suppressed_class_duplicates,
                      projected_valid_positions=metric_valid, dimensions=sorted(dimensions),
                      raw_dimensions=sorted(raw_dimensions), selection_errors=list(errors),
                      device=state.device if state else '', model=state.model_path if state else '',
                      processing_p95_ms=latencies[min(len(latencies)-1, int(.95*len(latencies)))] if latencies else None,
                      tracker_tracks_latest=len(latest['tracks'].tracks) if 'tracks' in latest else 0)
        report['capture_acceptance'] = acceptance.report()
        report['plumbing_passed'] = (all(counts[k] > 0 for k in ('raw','image','state','bridge','projected','tracks'))
                            and dimensions == raw_dimensions and metric_valid > 0
                            and duplicate_frames == 0 and class_overlap_frames == 0
                            and (not (args.auto_select or args.selected_id is not None)
                                 or (selected is not None and sources['fused'] > 0)))
        metrics = report['capture_acceptance']
        rate = metrics['in_view_effective_tracking_rate']
        report['tracking_acceptance_passed'] = (rate >= args.minimum_in_view_rate
            and metrics['truth_annotation_coverage'] >= args.minimum_truth_coverage
            and metrics['recovered_events'] >= args.minimum_recovered_events
            and metrics['wrong_target_frames'] == 0 and metrics['public_identity_switches'] == 0
            and metrics['window_evicted_frames'] == 0 and metrics['window_evicted_raw'] == 0
            if rate is not None else None)
        report['passed'] = (report['plumbing_passed'] and report['tracking_acceptance_passed']
                            if report['tracking_acceptance_passed'] is not None else None)
        report['acceptance_note'] = ('No truth: tracking accuracy is NOT certified; '
            'message source counters include revisions, capture_acceptance deduplicates them. '
            'Truth identity metrics require one intentionally selected physical target.')
        image = latest.get('image')
    for subscriber in subscribers:
        subscriber.unregister()
    if args.snapshot and image is not None and image.encoding == 'bgr8':
        import cv2
        import numpy as np
        frame = np.frombuffer(image.data, np.uint8).reshape(image.height, image.step)[:, :image.width*3]
        cv2.imwrite(args.snapshot, frame.reshape(image.height, image.width, 3))
    Path(args.output).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
    return 0 if (report['passed'] if args.require_tracking else report['plumbing_passed']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
