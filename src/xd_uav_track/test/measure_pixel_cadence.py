#!/usr/bin/env python3
"""Bounded transport cadence probe. Not a substitute for labelled accuracy metrics."""
import argparse
import collections
import hashlib
import json
import time
import threading
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from xd_uav_track.msg import PerceptionTargetArray, TrackStateArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=45.)
    parser.add_argument('--output', required=True)
    parser.add_argument('--world', required=True)
    args = parser.parse_args()
    rospy.init_node('pixel_cadence_probe', anonymous=True)
    lock = threading.Lock()
    rows = collections.defaultdict(lambda: {'stamps': collections.deque(maxlen=1200),
                                            'frames': 0, 'observed_frames': 0,
                                            'sources': collections.Counter()})
    diagnostics = {}

    def record(message, kind):
        stamp = message.header.stamp.to_sec()
        with lock:
            row = rows[kind + '/' + message.image_source]
            if row['stamps'] and stamp <= row['stamps'][-1]:
                return
            row['stamps'].append(stamp)
            row['frames'] += 1
            targets = getattr(message, 'targets', getattr(message, 'tracks', []))
            observed = False
            for target in targets:
                source = target.measurement_source
                row['sources'][source] += 1
                observed |= (target.lifecycle_state == 'confirmed' and
                             source in ('detector', 'visual_tracker'))
            row['observed_frames'] += int(observed)

    def diag(message):
        with lock:
            for status in message.status:
                diagnostics[status.name] = {v.key: v.value for v in status.values}

    rospy.Subscriber('/moving_uav_search/perception/targets', PerceptionTargetArray,
                     lambda m: record(m, 'presentation'), queue_size=10)
    rospy.Subscriber('/moving_uav_search/track/tracks_by_source', TrackStateArray,
                     lambda m: record(m, 'anchor'), queue_size=10)
    rospy.Subscriber('/moving_uav_search/perception/pixel_diagnostics', DiagnosticArray,
                     diag, queue_size=2)
    start = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic()-start < args.seconds:
        time.sleep(.1)
    with lock:
        result = {'world': args.world, 'duration_wall_sec': time.monotonic()-start,
                  'scope': 'cadence only; coverage/identity require capture-time truth visibility',
                  'diagnostics': diagnostics, 'streams': {}}
        with open(args.world, 'rb') as handle:
            result['world_sha256'] = hashlib.sha256(handle.read()).hexdigest()
        for key, row in rows.items():
            stamps = list(row['stamps'])
            gaps = sorted(b-a for a,b in zip(stamps, stamps[1:]))
            result['streams'][key] = {
                'frames': row['frames'], 'observed_frames': row['observed_frames'],
                'measurement_sources': dict(row['sources']),
                'max_frame_gap_sec': max(gaps) if gaps else None,
                'p95_frame_gap_sec': gaps[int(.95*(len(gaps)-1))] if gaps else None,
                'capture_fps': (len(stamps)-1)/(stamps[-1]-stamps[0]) if gaps else None}
    with open(args.output, 'w') as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
