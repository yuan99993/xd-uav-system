import unittest
from sar_yolo_detector.fusion_metrics import FusionAcceptance, capture_key


def evidence(stamp, source='yolo', identity=7, box=(10, 10, 20, 20)):
    return dict(stamp=float(stamp), frame_id='camera', selected_id=identity, local_id=5,
        source=source, observed=source in ('yolo', 'fused', 'visual'),
        reason='ok' if source in ('yolo', 'fused', 'visual') else 'no_trusted_visual_observation',
        selected_box=box)


class FusionMetricsTest(unittest.TestCase):
    def test_revisions_are_not_independent_frames_or_losses(self):
        metrics = FusionAcceptance()
        metrics.observe(evidence(1., 'lost'))
        metrics.observe(evidence(1., 'yolo'))
        metrics.observe(evidence(2., 'visual'))
        report = metrics.report()
        self.assertEqual(report['selected_capture_frames'], 2)
        self.assertEqual(report['same_capture_revisions'], 1)
        self.assertEqual(report['loss_events'], 0)
        self.assertEqual(report['detector_coverage'], .5)
        self.assertIsNone(report['in_view_effective_tracking_rate'])
        self.assertIsNone(report['wrong_target_frames'])

    def test_loss_events_recovery_time_and_no_manual_reselect_recovery(self):
        metrics = FusionAcceptance()
        for stamp, source, identity in ((1, 'yolo', 7), (2, 'prediction', 7),
                (3, 'lost', 7), (4, 'visual', 7), (5, 'lost', 7), (6, 'idle', -1), (7, 'yolo', 7)):
            metrics.observe(evidence(stamp, source, identity))
        report = metrics.report()
        self.assertEqual(report['loss_events'], 2)
        self.assertEqual(report['recovered_events'], 1)
        self.assertEqual(report['interrupted_events'], 1)
        self.assertEqual(report['recovered_durations_sec'], [2.])

    def test_truth_distinguishes_out_of_view_wrong_target_and_id_switch(self):
        truth = {}
        for stamp in range(1, 6):
            truth[capture_key(evidence(stamp))] = dict(objects=[
                dict(id='tank_a', visible=stamp != 2, box=(10, 10, 20, 20)),
                dict(id='tank_b', visible=True, box=(80, 10, 20, 20))])
        metrics = FusionAcceptance(truth=truth, truth_target_id='tank_a')
        for row in (evidence(1), evidence(2, 'lost'), evidence(3, 'lost'),
                    evidence(4, 'yolo', box=(80, 10, 20, 20)), evidence(5, 'yolo', identity=17)):
            metrics.observe(row)
        report = metrics.report()
        self.assertEqual(report['visible_truth_frames'], 4)
        self.assertEqual(report['in_view_effective_tracking_rate'], .5)
        self.assertEqual(report['wrong_target_frames'], 1)
        self.assertEqual(report['public_identity_switches'], 1)

    def test_missing_annotations_are_not_out_of_view(self):
        metrics = FusionAcceptance(truth={capture_key(evidence(1)): dict(objects=[])}, truth_target_id='a')
        metrics.observe(evidence(1))
        report = metrics.report()
        self.assertEqual(report['truth_status'], 'no_matching_annotations')
        self.assertIsNone(report['in_view_effective_tracking_rate'])

    def test_raw_captures_without_results_count_as_in_view_misses(self):
        truth = {capture_key(evidence(stamp)): dict(objects=[
            dict(id='a', visible=True, box=(10, 10, 20, 20))]) for stamp in (1, 2, 3)}
        metrics = FusionAcceptance(truth=truth, truth_target_id='a')
        for stamp in (1, 2, 3):
            metrics.add_raw(evidence(stamp))
        metrics.observe(evidence(1))
        report = metrics.report()
        self.assertEqual(report['raw_frames_in_selection_window'], 3)
        self.assertAlmostEqual(report['trusted_observation_coverage'], 1/3)
        self.assertAlmostEqual(report['in_view_effective_tracking_rate'], 1/3)

    def test_cache_bounds_and_capture_stamp_precision(self):
        metrics = FusionAcceptance(max_frames=16)
        for stamp in range(32):
            metrics.observe(evidence(stamp))
            metrics.add_raw(evidence(stamp))
        self.assertEqual(len(metrics.frames), 16)
        self.assertEqual(metrics.evicted_frames, 16)
        self.assertEqual(metrics.report()['window_evicted_raw'], 16)
        a = dict(capture_header=dict(frame_id='camera', stamp=dict(secs=1000000000, nsecs=1)))
        b = dict(capture_header=dict(frame_id='camera', stamp=dict(secs=1000000000, nsecs=2)))
        self.assertNotEqual(capture_key(a), capture_key(b))


if __name__ == '__main__':
    unittest.main()
