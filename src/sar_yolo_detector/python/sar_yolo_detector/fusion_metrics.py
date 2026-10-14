"""Finite-window, capture-deduplicated fusion acceptance statistics.

The final revision of a capture wins. Visibility/identity accuracy requires
external truth; a detector miss is never used as a visibility annotation.
"""
from collections import Counter, OrderedDict
import math

from .box_fusion import iou
from .loss_journal import LossJournal


def capture_key(record):
    header = record.get('capture_header', record)
    stamp = header.get('stamp')
    if isinstance(stamp, dict):
        secs, nsecs = int(stamp['secs']), int(stamp['nsecs'])
        if not 0 <= nsecs < 1000000000:
            raise ValueError('Invalid nanoseconds')
        timestamp = secs*1000000000+nsecs
    else:
        if not isinstance(stamp, (float, int)) or not math.isfinite(stamp):
            raise ValueError('Invalid capture stamp')
        timestamp = round(stamp*1000000000)
    return str(header.get('frame_id', '')), timestamp


class FusionAcceptance:
    def __init__(self, max_frames=4096, truth=None, truth_target_id=None):
        if not 16 <= max_frames <= 10000:
            raise ValueError('Invalid acceptance window')
        self.max_frames = max_frames
        self.frames, self.raw = OrderedDict(), OrderedDict()
        self.evicted_frames = self.evicted_raw = 0
        self.revision_messages = 0
        self.truth = truth or {}
        self.truth_target_id = truth_target_id

    def _put(self, table, key, value):
        table[key] = value
        if len(table) > self.max_frames:
            table.popitem(last=False)
            return 1
        return 0

    def add_raw(self, header):
        self.evicted_raw += self._put(self.raw, capture_key(header), True)

    def observe(self, evidence):
        key = (evidence.get('pipeline_epoch', 0), evidence.get('capture_segment', 0), capture_key(evidence))
        self.revision_messages += key in self.frames
        # Keep only scalars/bounded gate counters, no images or recursive logs.
        record = {field: evidence.get(field) for field in (
            'stamp', 'frame_id', 'selected_id', 'local_id', 'observed', 'source', 'reason',
            'pipeline_epoch', 'capture_segment', 'selected_box', 'diagnostics')}
        self.evicted_frames += self._put(self.frames, key, record)

    def report(self):
        selected = [(key, sample) for key, sample in self.frames.items()
                    if sample.get('selected_id', -1) >= 0]
        sources, reasons, rejects = Counter(), Counter(), Counter()
        journal = LossJournal(128, 1)
        for sample in self.frames.values():
            journal.observe(sample)
            if sample.get('selected_id', -1) < 0:
                continue
            sources[sample['source']] += 1
            if not sample.get('observed'):
                reasons[sample['reason']] += 1
                rejects.update((sample.get('diagnostics') or {}).get('rejected', {}))
        journal.flush()
        events = list(journal.events)
        recovered = [event for event in events if event['recovered']]
        # Include raw captures without a worker result, but exclude periods
        # explicitly unselected. A continuing loss extends to the last raw
        # capture, not merely the last published result.
        windows, active = [], None
        for key, sample in self.frames.items():
            source, timestamp = key[2]
            if sample.get('selected_id', -1) >= 0:
                if active is None:
                    active = (source, timestamp)
                elif active[0] != source or timestamp < active[1]:
                    windows.append((active[0], active[1], timestamp))
                    active = (source, timestamp)
            elif active is not None:
                windows.append((active[0], active[1], timestamp-1))
                active = None
        if active is not None:
            windows.append((active[0], active[1], max(
                (stamp for source, stamp in self.raw if source == active[0]), default=active[1])))
        raw_selected = sum(any(source == camera and start <= timestamp <= end
            for camera, start, end in windows) for source, timestamp in self.raw)
        observed = sum(bool(sample['observed']) for _, sample in selected)
        denominator = max(len(selected), raw_selected)
        result = dict(unique_capture_frames=len(self.frames), selected_capture_frames=len(selected),
            raw_frames_in_selection_window=raw_selected, same_capture_revisions=self.revision_messages,
            raw_frames_without_selected_output=max(0, raw_selected-len(selected)),
            window_evicted_frames=self.evicted_frames, window_evicted_raw=self.evicted_raw,
            unique_capture_sources=dict(sources), absence_reasons=dict(reasons), rejected_gates=dict(rejects),
            trusted_observation_coverage=observed/denominator if denominator else None,
            detector_coverage=(sources['yolo']+sources['fused'])/denominator if denominator else None,
            visual_coverage=sources['visual']/denominator if denominator else None,
            prediction_coverage=sources['prediction']/denominator if denominator else None,
            loss_events=journal.event_count, recovered_events=journal.recovered_count,
            interrupted_events=journal.interrupted_count,
            recovery_rate=journal.recovered_count/journal.event_count if journal.event_count else None,
            event_durations_sec=[event.get('duration_sec', 0.) for event in events],
            recovered_durations_sec=[event['duration_sec'] for event in recovered],
            active_loss=journal.active,
            unfinished_loss_events=1 if journal.active else 0,
            # Ground truth is optional, absence is explicitly unknown.
            visible_truth_frames=0, in_view_effective_tracking_rate=None,
            truth_annotation_coverage=None,
            wrong_target_frames=None, public_identity_switches=None,
            truth_status='not_provided', event_detail_window=128)
        if not self.truth or self.truth_target_id is None:
            return result
        if len({key[:2] for key, _ in selected}) > 1:
            result['truth_status'] = 'multiple_timelines_need_separate_runs'
            return result
        visible = effective = wrong = switches = 0
        previous_public_id = None
        labelled = 0
        by_capture = {key[2]: sample for key, sample in selected}
        # A labelled raw image with no result is a miss, not an omitted item
        # from the in-view denominator.
        truth_keys = set(by_capture) | {key for key in self.raw if any(
            key[0] == source and start <= key[1] <= end for source, start, end in windows)}
        for key in sorted(truth_keys):
            row = self.truth.get(key)
            if row is None:
                continue
            objects = row.get('objects', [])
            target = next((obj for obj in objects if str(obj['id']) == str(self.truth_target_id)), None)
            if target is None:
                continue  # missing annotation is not "out of view"
            labelled += 1
            visible += bool(target['visible'])
            sample = by_capture.get(key)
            if sample is None or not sample.get('observed') or not sample.get('selected_box'):
                continue
            box = sample['selected_box']
            correct = target['visible'] and iou(box, target['box']) >= .5
            if correct:
                effective += 1
                if previous_public_id is not None and previous_public_id != sample['selected_id']:
                    switches += 1
                previous_public_id = sample['selected_id']
            elif any(obj['visible'] and str(obj['id']) != str(self.truth_target_id)
                     and iou(box, obj['box']) >= .5 for obj in objects):
                wrong += 1
        result.update(truth_status='annotated' if labelled else 'no_matching_annotations',
            visible_truth_frames=visible, annotated_selected_frames=labelled,
            truth_annotation_coverage=labelled/len(truth_keys) if truth_keys else None,
            in_view_effective_tracking_rate=effective/visible if visible else None,
            wrong_target_frames=wrong if labelled else None,
            public_identity_switches=switches if labelled else None)
        return result
