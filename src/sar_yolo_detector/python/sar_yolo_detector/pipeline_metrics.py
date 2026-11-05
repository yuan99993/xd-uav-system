"""Bounded, passive measurements of the SAR bridge -> XD tracking path."""
import threading
import time
from collections import Counter, OrderedDict, deque


class XDChainMetrics:
    def __init__(self):
        self.lock = threading.Lock()
        self.inputs = OrderedDict()
        self.counts = Counter()
        self.sources = Counter()
        self.lifecycle = Counter()
        self.bridge_ms = deque(maxlen=10000)
        self.capture_ms = deque(maxlen=10000)

    @staticmethod
    def key(message):
        return message.image_source, message.header.stamp.to_nsec()

    def receive_input(self, message):
        with self.lock:
            self.inputs[self.key(message)] = (time.monotonic(),
                {int(item.track_id):int(item.class_id) for item in message.candidates
                 if item.track_id_is_stable and item.measurement_source in ('','detector')})
            while len(self.inputs) > 256:
                self.inputs.popitem(last=False)
            self.counts['input_frames'] += 1
            self.counts['input_candidates'] += len(message.candidates)
            self.sources.update(item.measurement_source for item in message.candidates)

    def receive_output(self, message):
        import rospy
        now = time.monotonic()
        with self.lock:
            self.counts['output_frames'] += 1
            self.counts['output_tracks'] += len(message.tracks)
            self.lifecycle.update(item.lifecycle_state for item in message.tracks)
            captured = self.inputs.pop(self.key(message), None)
            if captured is not None:
                started,classes=captured
                self.bridge_ms.append((now - started) * 1000.)
                for track in message.tracks:
                    if (track.detected and track.class_id>=0 and track.detector_id_stable and
                            track.detector_track_id in classes):
                        equal=track.class_id==classes[track.detector_track_id]
                        self.counts['classification_matches' if equal else 'classification_mismatches']+=1
            age = (rospy.Time.now() - message.header.stamp).to_sec()
            if 0 <= age < 60:
                self.capture_ms.append(age * 1000.)

    @staticmethod
    def percentiles(values):
        values = sorted(values)
        return {name: values[min(len(values)-1, int(fraction*(len(values)-1)))] if values else None
                for name, fraction in (('p50_ms', .5), ('p95_ms', .95))}

    def summary(self):
        with self.lock:
            return dict(counts=dict(self.counts), candidate_sources=dict(self.sources),
                        lifecycle_observations=dict(self.lifecycle),
                        bridge_to_xd=self.percentiles(self.bridge_ms),
                        capture_to_xd=self.percentiles(self.capture_ms),
                        matched_latency_samples=len(self.bridge_ms),
                        notes='Passive topic arrival timings; lifecycle counts are observations, not unique IDs. '
                              'Standalone manual visual ROIs do not create detector identities in XD.')
