#!/usr/bin/python3
"""Score civilian-car footage and independently collected canvas samples."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


def stamp(value):
    return value['secs'] * 1_000_000_000 + value['nsecs'] if value else None


def iou(a, b):
    x = max(0, min(a[0]+a[2], b[0]+b[2])-max(a[0], b[0]))
    y = max(0, min(a[1]+a[3], b[1]+b[3])-max(a[1], b[1]))
    union = a[2]*a[3]+b[2]*b[3]-x*y
    return x*y/union if union > 0 else 0


def visible(car):
    return car['on_screen'] and car['visibility_proxy'] == 'centre_ray_clear'


def classify(box, cars, primary):
    overlaps = {car['entity_id']: iou(box, car['projected_world_aabb']) for car in cars
                if visible(car) and car['projected_world_aabb']}
    focal = overlaps.pop(primary, 0)
    other = max(overlaps.values(), default=0)
    if focal >= .25 and other >= .25 and abs(focal-other) < .05:
        return 'ambiguous'
    if other >= .25 and other > focal:
        return 'neighbor_car'
    if focal >= .25:
        return 'target_car'
    if max(focal, other) > 0:
        return 'poor_alignment'
    return 'background_proxy'


def p95(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v) and v >= 0)
    return values[math.ceil(.95*len(values))-1] if values else None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--review', type=Path, required=True)
    ap.add_argument('--evaluation', type=Path, required=True)
    ap.add_argument('--browser', type=Path)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    read = lambda path: json.loads(path.read_text())
    primary = read(args.review/'summary.json')['selection']['primary_entity']
    result = read(args.evaluation/'result.json')
    identities = result.get('initial_selected_id')
    fusion_file = args.evaluation/'fusion.json'
    conflicts = set()
    diagnostic_frames = set()
    if fusion_file.exists():
        for row in read(fusion_file):
            diag = row.get('diagnostics', {})
            key = stamp(row.get('capture_header'))
            if key is not None and 'branches' in diag:
                diagnostic_frames.add(key)
            if (diag.get('branches', {}).get('feature', {}).get('accepted') and
                    diag.get('selected_target_flow', {}).get('reason') == 'inconsistent_target_flow'):
                if key is not None:
                    conflicts.add(key)
    frames = [json.loads(line) for line in (args.review/'aligned.jsonl').read_text().splitlines()]
    frames.sort(key=lambda row: row['capture_nsec'])
    # Historical evaluations have no scoring-window endpoints. In that case
    # the aligned review's scored span is the observable time boundary.
    counts = Counter(dict(background_proxy=0, neighbor_car=0, ambiguous=0,
                          poor_alignment=0, current_boxes=0, visible_frames=0,
                          correct_frames=0, identity_switch_frames=0,
                          missing_scored_state_frames=0))
    frame_index = {}
    gap_start = false_start = blocked = recovery = None
    longest_gap = longest_false = 0
    recovery_events = []
    for row in frames:
        now = row['capture_nsec']
        if row['outcome'] == 'outside_scored_interval':
            gap_start = false_start = blocked = recovery = None
            continue
        cars = row['cars']
        target = next(car for car in cars if car['entity_id'] == primary)
        state = row['tracker_state'] if row['score'] is not None else None
        actual = state and state['current']
        category = classify(state['box'], cars, primary) if actual else 'no_current_box'
        correct = bool(row['outcome'] == 'correct' and category == 'target_car'
                       and (identities is None or state['id'] == identities))
        wrong = bool(actual and not correct and category != 'ambiguous')
        counts['missing_scored_state_frames'] += state is None
        if actual:
            counts['current_boxes'] += 1
            if identities is not None and state['id'] != identities:
                counts['identity_switch_frames'] += 1
            counts[category] += 1
            if category == 'target_car' and not correct:
                counts['target_box_failed_geometry_or_identity'] += 1
        frame_index[now] = row
        if visible(target):
            counts['visible_frames'] += 1
            counts['correct_frames'] += correct
            if blocked:
                recovery = dict(visible_again_capture_nsec=now, first_correct_capture_nsec=None,
                                recovery_sec=None)
                recovery_events.append(recovery)
            blocked = False
            if correct and recovery is not None and recovery['recovery_sec'] is None and identities is not None:
                recovery.update(first_correct_capture_nsec=now,
                                recovery_sec=(now-recovery['visible_again_capture_nsec'])/1e9)
        else:
            if recovery is not None:
                recovery['visible_interval_end_nsec'] = now
            recovery = None
            blocked = target['on_screen'] and target['visibility_proxy'] == 'centre_ray_blocked'
        if gap_start is not None:
            longest_gap = max(longest_gap, (now-gap_start)/1e9)
        if false_start is not None:
            longest_false = max(longest_false, (now-false_start)/1e9)
        gap_start = (now if gap_start is None else gap_start) if visible(target) and not correct else None
        false_start = (now if false_start is None else false_start) if wrong else None
    browser = None
    if args.browser:
        samples = [s for s in read(args.browser) if stamp(s.get('image_capture_stamp')) in frame_index]
        confirmed = [s for s in samples if s.get('confirmed') is True]
        comparable = [s for s in confirmed if s.get('overlay_capture_stamp')]
        disagreed = [s for s in samples if stamp(s.get('image_capture_stamp')) in conflicts
                     and s.get('confirmed') is not None]
        browser_counts = Counter(dict(frame_mismatch=0, neighbor_car=0,
                                      background_proxy=0, ambiguous=0, poor_alignment=0))
        correct_presentations = []
        for sample in comparable:
            image = stamp(sample['image_capture_stamp'])
            if image != stamp(sample['overlay_capture_stamp']):
                browser_counts['frame_mismatch'] += 1
                continue
            cx, cy, w, h = sample['selected_box']
            category = classify([cx-w/2, cy-h/2, w, h], frame_index[image]['cars'], primary)
            browser_counts[category] += 1
            if category == 'target_car' and identities is not None and sample['selected_id'] == identities:
                correct_presentations.append((image, sample))
        for event in recovery_events:
            after = event['visible_again_capture_nsec']
            until = event.get('visible_interval_end_nsec', math.inf)
            candidates = [(image, sample) for image, sample in correct_presentations if after <= image < until]
            if candidates:
                image, sample = candidates[0]
                elapsed = (image-after)/1e9
                event['browser_recovery_sec_bounds'] = [
                    elapsed+sample['lower_ms']/1000, elapsed+sample['upper_ms']/1000]
            else:
                event['browser_recovery_sec_bounds'] = None
        first_paints = {}
        for sample in samples:
            first_paints.setdefault(sample['frame_token'], sample)
        browser = dict(matched_samples=len(samples), confirmed_samples=len(confirmed),
                       comparable_confirmed_samples=len(comparable), counts=dict(browser_counts),
                       conflicting_samples=len(disagreed),
                       conflicting_confirmed_samples=sum(s['confirmed'] for s in disagreed),
                       frame_age_lower_p95_ms=p95(s.get('lower_ms') for s in first_paints.values()),
                       frame_age_upper_p95_ms=p95(s.get('upper_ms') for s in first_paints.values()))
    summary = dict(counts=dict(counts),
                   correct_coverage=counts['correct_frames']/counts['visible_frames'] if counts['visible_frames'] else None,
                   longest_visible_gap_sec=longest_gap if counts['visible_frames'] else None,
                   longest_false_lock_sec=longest_false if frame_index else None,
                   initial_selected_id=identities, occlusion_recovery=recovery_events,
                   fusion_branch_diagnostic_frames=len(diagnostic_frames.intersection(frame_index)),
                   conflicting_fusion_frames=len(conflicts.intersection(frame_index)),
                   browser=browser, scored_frames=len(frame_index))
    with args.output.open('x') as stream:
        json.dump(summary, stream, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
