#!/usr/bin/python3
"""Join recorded RGB captures with civilian vehicle truth and tracker evidence."""
import argparse
import json
import math
from bisect import bisect_left
from pathlib import Path

import cv2
import rosbag
from cv_bridge import CvBridge


def read(path):
    return json.loads(path.read_text())


def nearby(keys, stamp):
    index = bisect_left(keys, stamp)
    options = keys[max(0, index - 1):index + 1]
    match = min(options, key=lambda key: abs(key - stamp), default=None)
    return match if match is not None and abs(match - stamp) <= 2 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('bag', 'evaluation', 'capture', 'output'):
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()
    manifest = read(Path(str(args.bag) + '.manifest.json'))
    run = read(args.capture / 'run.json')
    if run['session'] != manifest['source']['session']:
        parser.error('Bag and capture metadata belong to different sessions')
    evaluation = args.evaluation
    selection = read(evaluation / 'selection.json')
    primary = selection['primary_entity']
    cars = {obj['id'] for obj in run['scene']['entities'] if obj['class'] == 'car'}
    if primary not in cars:
        parser.error('The selected entity must be a civilian car')
    offset = run['clock_alignment']['ue_minus_host_sec']
    truth = {round((row['capture_wall_start'] - offset) * 1e6): row
             for row in map(json.loads, (evaluation / 'truth.jsonl').read_text().splitlines())}
    comparison = {round(row['stamp'] * 1e6): row for row in read(evaluation / 'comparison.json')}
    if not comparison:
        parser.error('No scored states in this evaluation')
    states = {}
    for row in read(evaluation / 'states.json'):
        key = round(row['stamp'] * 1e6)
        if key not in states or row['current']:
            states[key] = row
    truth_keys = sorted(truth)
    score_keys = sorted(comparison)
    state_keys = sorted(states)
    args.output.mkdir(parents=True, exist_ok=False)
    bridge = CvBridge()
    previous = None
    rows = []
    first_failure = None
    first_visible_failure = None
    correct_seen = False
    topic = manifest['source']['image_topic']
    with rosbag.Bag(str(args.bag)) as bag:
        for _, image, _ in bag.read_messages(topics=[topic]):
            ns = image.header.stamp.to_nsec()
            stamp = round(ns / 1000)
            annotation = truth.get(nearby(truth_keys, stamp))
            score = comparison.get(nearby(score_keys, stamp))
            state = states.get(nearby(state_keys, stamp))
            target = None
            objects = []
            if annotation:
                objects = [obj for obj in annotation['objects'] if obj['entity_id'] in cars]
                target = next(obj for obj in objects if obj['entity_id'] == primary)
            visible = bool(target and target['on_screen'] and
                           target['visibility_proxy'] == 'centre_ray_clear')
            speed_px = speed_mps = None
            if target and previous:
                old_ns, old = previous
                dt = (ns - old_ns) / 1e9
                if dt > 0:
                    speed_mps = math.dist(target['centre_world_ned_m'], old['centre_world_ned_m']) / dt
                    if target['centre_px'] and old['centre_px']:
                        speed_px = math.dist(target['centre_px'], old['centre_px']) / dt
            previous = (ns, target) if target else None
            outcome = 'outside_scored_interval'
            if score_keys[0] <= stamp <= score_keys[-1] and annotation:
                if visible:
                    outcome = ('no_matched_output' if score is None else 'correct' if score['correct']
                               else 'wrong_current_box' if score['current'] else 'no_current_box')
                else:
                    outcome = 'target_not_visible'
            row = dict(capture_nsec=ns, image_seq=image.header.seq,
                       truth_seq=annotation['frame_seq'] if annotation else None,
                       cars=objects, target_mps=speed_mps, target_px_per_sec=speed_px,
                       outcome=outcome, score=score, tracker_state=state)
            rows.append(row)
            if outcome == 'correct':
                correct_seen = True
            failed = outcome in ('no_matched_output', 'wrong_current_box', 'no_current_box')
            evidence_names = []
            if failed and first_visible_failure is None:
                first_visible_failure = row
                evidence_names.append('first_visible_failure_raw.png')
            if failed and correct_seen and first_failure is None:
                first_failure = row
                evidence_names.append('first_failure_after_correct_raw.png')
            if evidence_names:
                rgb = bridge.imgmsg_to_cv2(image, 'bgr8')
                for name in evidence_names:
                    if not cv2.imwrite(str(args.output / name), rgb):
                        raise OSError('Could not write evidence image')
    summary = dict(bag=str(args.bag.resolve()), bag_sha256=manifest['bag_sha256'],
                   selection=selection, frames=len(rows),
                   truth_matches=sum(row['truth_seq'] is not None for row in rows),
                   scored_matches=sum(row['score'] is not None for row in rows),
                   first_visible_failure=first_visible_failure,
                   first_failure_after_correct=first_failure)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    with (args.output / 'aligned.jsonl').open('w') as stream:
        for row in rows:
            stream.write(json.dumps(row) + '\n')


if __name__ == '__main__':
    main()
