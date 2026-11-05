"""Capture-causal assistance after BoT-SORT, shared by runtime and replay.

Does not invent boxes, confidence, identities, or detector observations. Only
distinct capture times count as evidence. The UE deployment enables class
consensus only; extra birth confirmation remains an experimental option.
"""
from collections import Counter, deque
from dataclasses import dataclass, field
import math


@dataclass
class Evidence:
    box: tuple
    stamp: float
    label: int
    confirmed: bool = False
    observations: deque = field(default_factory=lambda: deque(maxlen=5))


class TemporalDetectionEvidence:
    def __init__(self, birth_confirmation=True, class_consensus=True):
        self.birth_confirmation = birth_confirmation
        self.class_consensus = class_consensus
        self.reset()

    def reset(self):
        self.tracks = {}
        self.last_stamp = None
        self.frame = 0
        self.stats = Counter()

    @staticmethod
    def family(label):
        return 0 if label in (0, 1) else label

    @staticmethod
    def continuous(previous, current):
        a, b = previous.box, current
        aw, ah = a[2]-a[0], a[3]-a[1]
        bw, bh = b[2]-b[0], b[3]-b[1]
        scale = max(aw/bw, bw/aw, ah/bh, bh/ah)
        distance = math.hypot((a[0]+a[2]-b[0]-b[2])/2., (a[1]+a[3]-b[1]-b[3])/2.)
        return scale <= 2.5 and distance <= 3.*max(1., math.hypot(aw, ah))

    def process(self, detections, stamp):
        if not math.isfinite(stamp) or (self.last_stamp is not None and stamp <= self.last_stamp):
            self.stats['rejected_capture'] += 1
            return []
        self.last_stamp = stamp
        self.frame += 1
        self.tracks = {identity: state for identity, state in self.tracks.items() if stamp-state.stamp <= 3.}
        output = []
        seen = set()
        for raw in sorted(detections, key=lambda row: -row['score']):
            row = dict(raw)
            label, score, box = row['label'], row['score'], tuple(row['box'])
            if (not math.isfinite(score) or not 0. <= score <= 1. or
                    len(box) != 4 or not all(math.isfinite(v) for v in box) or
                    box[2] <= box[0] or box[3] <= box[1]):
                self.stats['invalid_detection'] += 1
                continue
            identity = row.get('track_id', -1)
            if identity < 0 or not row.get('stable', False):
                # Position/list-index IDs cannot safely accumulate identity evidence.
                output.append(row)
                continue
            if identity in seen:
                continue
            seen.add(identity)
            state = self.tracks.get(identity)
            visual = row.get('measurement_source', 'detector') != 'detector'
            if visual:
                if state is not None and state.confirmed:
                    output.append(row)
                self.stats['visual_not_counted'] += 1
                continue
            if (state is None or not self.continuous(state, box) or
                    self.family(state.label) != self.family(label)):
                state = Evidence(box, stamp, label)
                self.tracks[identity] = state
            state.box, state.stamp = box, stamp
            state.observations.append((self.frame, stamp, label, score))
            while state.observations and stamp-state.observations[0][1] > 2.:
                state.observations.popleft()
            if not state.confirmed:
                recent = [item for item in state.observations if self.frame-item[0] <= 2]
                # Strong proposals have a no-wait path. Weak births need 2/3
                # distinct captures; once confirmed, .10 updates remain valid.
                state.confirmed = (not self.birth_confirmation or score >= .85 or
                                   (score >= .25 and len(recent) >= 2))
            if not state.confirmed:
                self.stats['pending_birth'] += 1
                continue
            if self.class_consensus and label != state.label:
                weights = Counter()
                support = Counter()
                for _, _, category, confidence in state.observations:
                    weights[category] += confidence
                    support[category] += 1
                winner = max(weights, key=lambda category: (weights[category], category == state.label))
                tail = list(state.observations)[-2:]
                consecutive = len(tail) == 2 and all(item[2] == winner for item in tail)
                if (winner != state.label and support[winner] >= 2 and consecutive and
                        weights[winner] >= .60*sum(weights.values())):
                    state.label = winner
                    self.stats['confirmed_class_change'] += 1
                if row['label'] != state.label:
                    # Preserve raw evidence explicitly. The unchanged score is
                    # the current proposal score, not a calibrated probability
                    # of the history-derived stable class.
                    row['raw_label'] = row['label']
                    row['label'] = state.label
                    row['label_source'] = 'temporal_consensus'
                    self.stats['single_frame_label_change_held'] += 1
            else:
                state.label = label
            output.append(row)
        if len(self.tracks) > 128:
            keep = sorted(self.tracks, key=lambda identity: self.tracks[identity].stamp, reverse=True)[:128]
            self.tracks = {identity: self.tracks[identity] for identity in keep}
        return output
