import unittest

from sar_yolo_detector.temporal_detection_evidence import TemporalDetectionEvidence


def detection(label=0, score=.6, identity=7, box=(20.,20.,60.,50.), **kwargs):
    return dict(label=label, score=score, track_id=identity, stable=True, box=list(box), **kwargs)


class TemporalEvidenceTest(unittest.TestCase):
    def test_weak_single_frame_proposal_is_not_confirmed(self):
        tracker = TemporalDetectionEvidence()
        self.assertFalse(tracker.process([detection()], 1.))
        self.assertFalse(tracker.process([], 1.5))
        self.assertEqual(tracker.stats['pending_birth'], 1)

    def test_static_target_can_confirm_and_keep_low_score_updates(self):
        tracker = TemporalDetectionEvidence()
        self.assertFalse(tracker.process([detection()], 1.))
        self.assertEqual(len(tracker.process([detection()], 1.5)), 1)
        result = tracker.process([detection(score=.12)], 2.)
        self.assertEqual(result[0]['score'], .12)

    def test_same_capture_cannot_create_confirmation(self):
        tracker = TemporalDetectionEvidence()
        tracker.process([detection()], 1.)
        self.assertFalse(tracker.process([detection()], 1.))
        self.assertEqual(len(tracker.process([detection()], 1.5)), 1)

    def test_high_score_birth_needs_no_extra_frame(self):
        tracker = TemporalDetectionEvidence()
        self.assertEqual(len(tracker.process([detection(score=.9)], 1.)), 1)

    def test_one_wrong_class_does_not_change_established_class(self):
        tracker = TemporalDetectionEvidence()
        tracker.process([detection(score=.9)], 1.)
        tracker.process([detection()], 1.5)
        result = tracker.process([detection(label=1)], 2.)
        self.assertEqual(result[0]['label'], 0)
        self.assertEqual(result[0]['raw_label'], 1)

    def test_sustained_new_class_can_correct_wrong_initial_class(self):
        tracker = TemporalDetectionEvidence()
        tracker.process([detection(label=1, score=.9)], 1.)
        tracker.process([detection(label=0, score=.9)], 1.5)
        result = tracker.process([detection(label=0, score=.9)], 2.)
        self.assertEqual(result[0]['label'], 0)

    def test_ground_history_does_not_relabel_drone(self):
        tracker = TemporalDetectionEvidence()
        tracker.process([detection(score=.9)], 1.)
        tracker.process([detection()], 1.5)
        self.assertFalse(tracker.process([detection(label=2)], 2.))
        result = tracker.process([detection(label=2)], 2.5)
        self.assertEqual(result[0]['label'], 2)

    def test_recycled_id_far_away_requires_confirmation(self):
        tracker = TemporalDetectionEvidence()
        tracker.process([detection(score=.9)], 1.)
        self.assertFalse(tracker.process([detection(box=(800.,400.,840.,430.))], 1.5))

    def test_visual_updates_cannot_confirm_detector_birth(self):
        tracker = TemporalDetectionEvidence()
        tracker.process([detection()], 1.)
        self.assertFalse(tracker.process([detection(measurement_source='visual_tracker')], 1.5))
        self.assertEqual(len(tracker.tracks[7].observations), 1)

    def test_long_absence_expires_history(self):
        tracker = TemporalDetectionEvidence()
        tracker.process([detection(score=.9)], 1.)
        self.assertFalse(tracker.process([detection()], 5.))


if __name__ == '__main__':
    unittest.main()
