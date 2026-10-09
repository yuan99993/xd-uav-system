"""Bounded loss events. Same-capture revisions replace a pending sample.

Finalize on the next capture, so an async YOLO correction cannot count a
temporary pre-inference loss as a separate incident. No images are retained.
"""
from collections import deque
import copy
import math


class LossJournal:
    def __init__(self, max_events=32, context_frames=12):
        if not 1 <= max_events <= 128 or not 1 <= context_frames <= 32:
            raise ValueError("Invalid loss journal bounds")
        self.events = deque(maxlen=max_events)
        self.context = deque(maxlen=context_frames)
        self.context_frames = context_frames
        self.pending = None
        self.active = None
        self.key = None
        self.finalized_stamp = None
        self.event_count = self.recovered_count = self.interrupted_count = 0

    def observe(self, evidence):
        stamp = evidence.get('stamp')
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return False
        key = (evidence.get('pipeline_epoch', 0), evidence.get('capture_segment', 0),
               evidence.get('frame_id', ''), evidence.get('selected_id', -1))
        if key != self.key:
            self.flush()
            if self.active is not None:
                self.active['end_reason'] = 'selection_or_timeline_changed'
                self.events.append(self.active)
                self.interrupted_count += 1
                self.active = None
            self.context.clear()
            self.key = key
            self.finalized_stamp = None
        if self.finalized_stamp is not None and stamp <= self.finalized_stamp:
            return False
        if self.pending is not None:
            if stamp < self.pending['stamp']:
                return False
            if stamp > self.pending['stamp']:
                self.flush()
        self.pending = copy.deepcopy(evidence)
        return True

    def flush(self):
        if self.pending is None:
            return
        sample, self.pending = self.pending, None
        self.finalized_stamp = sample['stamp']
        if sample.get('selected_id', -1) < 0:
            return
        observed = bool(sample.get('observed'))
        if not observed and self.active is None:
            self.event_count += 1
            self.active = dict(event_id=self.event_count, selected_id=sample['selected_id'],
                start_stamp=sample['stamp'], first_reason=sample.get('reason', ''),
                first_diagnostics=sample.get('diagnostics', {}),
                before=list(self.context), after=[], recovered=False)
        if self.active is not None:
            self.active['last_stamp'] = sample['stamp']
            self.active['last_reason'] = sample.get('reason', '')
            self.active['last_diagnostics'] = sample.get('diagnostics', {})
            self.active['duration_sec'] = max(0., sample['stamp']-self.active['start_stamp'])
            if len(self.active['after']) < self.context_frames:
                self.active['after'].append(sample)
            if observed:
                self.active.update(recovered=True, end_reason='trusted_observation',
                    recovered_local_id=sample.get('local_id', -1))
                self.recovered_count += 1
                self.events.append(self.active)
                self.active = None
        self.context.append(sample)

    def snapshot(self, recent_limit=None):
        recent = list(self.events)
        if recent_limit is not None:
            recent = recent[-max(1, int(recent_limit)):]
        return copy.deepcopy(dict(loss_events=self.event_count, recovered_events=self.recovered_count,
            interrupted_events=self.interrupted_count, active=self.active,
            recent_events=recent, finalized_through_stamp=self.finalized_stamp))

    def summary(self):
        event = self.active or (self.events[-1] if self.events else None)
        return dict(loss_events=self.event_count, recovered_events=self.recovered_count,
            active=self.active is not None, event_id=event['event_id'] if event else None,
            first_reason=event['first_reason'] if event else None,
            duration_sec=event.get('duration_sec', 0.) if event else 0.,
            finalized_through_stamp=self.finalized_stamp)
