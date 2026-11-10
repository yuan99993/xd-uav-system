import unittest
from sar_yolo_detector.loss_journal import LossJournal


def sample(stamp, observed=False, reason='no_trusted_visual_observation', identity=7):
    return dict(stamp=stamp, selected_id=identity, local_id=5, observed=observed,
        reason=reason, source='yolo' if observed else 'lost',
        diagnostics=dict(rejected={'appearance': 1}))


class LossJournalTest(unittest.TestCase):
    def test_same_frame_correction_does_not_create_loss(self):
        journal = LossJournal()
        journal.observe(sample(1.))
        journal.observe(sample(1., True))
        journal.observe(sample(2., True))
        journal.flush()
        self.assertEqual(journal.event_count, 0)

    def test_one_prolonged_loss_retains_initial_cause(self):
        journal = LossJournal()
        journal.observe(sample(1., True))
        journal.observe(sample(2., reason='local_tracker_failed'))
        for stamp in range(3, 100):
            journal.observe(sample(float(stamp), reason='identity_memory_waiting'))
        journal.observe(sample(100., True))
        journal.flush()
        self.assertEqual(journal.event_count, 1)
        self.assertEqual(journal.recovered_count, 1)
        event = journal.events[0]
        self.assertEqual(event['first_reason'], 'local_tracker_failed')
        self.assertEqual(event['duration_sec'], 98.)
        self.assertEqual(event['before'][0]['stamp'], 1.)
        self.assertEqual(len(event['after']), 12)

    def test_selection_change_is_not_recovery(self):
        journal = LossJournal()
        journal.observe(sample(1.))
        journal.observe(sample(2., True, identity=8))
        journal.flush()
        self.assertEqual(journal.recovered_count, 0)
        self.assertEqual(journal.interrupted_count, 1)

    def test_bounds_and_out_of_order(self):
        journal = LossJournal(2, 2)
        for stamp in range(1, 20):
            journal.observe(sample(float(stamp), bool(stamp % 2)))
        self.assertFalse(journal.observe(sample(18.)))
        journal.flush()
        self.assertLessEqual(len(journal.events), 2)
        self.assertLessEqual(len(journal.context), 2)

    def test_finalized_captures_cannot_be_counted_again(self):
        journal = LossJournal()
        journal.observe(sample(1.))
        journal.flush()
        self.assertFalse(journal.observe(sample(1., True)))
        self.assertFalse(journal.observe(sample(.9)))
        self.assertEqual(journal.event_count, 1)

    def test_unselected_frames_never_count_as_loss(self):
        journal = LossJournal()
        journal.observe(sample(1., identity=-1))
        journal.flush()
        self.assertEqual(journal.summary()['loss_events'], 0)

    def test_many_incidents_keep_bounded_context_and_history(self):
        journal = LossJournal()
        for stamp in range(1, 2001):
            journal.observe(sample(float(stamp), bool(stamp % 2)))
        journal.flush()
        self.assertEqual(journal.event_count, 1000)
        self.assertEqual(len(journal.events), 32)
        self.assertEqual(len(journal.context), 12)
        self.assertLessEqual(len(journal.snapshot(recent_limit=2)['recent_events']), 2)
        for event in journal.events:
            self.assertLessEqual(len(event['before']), 12)
            self.assertLessEqual(len(event['after']), 12)


if __name__ == '__main__':
    unittest.main()
