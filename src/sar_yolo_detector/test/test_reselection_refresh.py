"""Real adapter selection regressions and immutable refresh job boundaries."""
import copy
import unittest

import rospy

import test_fusion_adapter as adapter
import test_lost_detector_dispatch as dispatch
from sar_yolo_detector.async_pipeline import LatestMailbox, InferencePacket
from sar_yolo_detector.box_fusion import Candidate, iou
from sar_yolo_detector.roi_selection import unique_roi_matches


class ReselectionRefreshTest(unittest.TestCase):
    def setUp(self):
        self.fixture = adapter.FusionAdapterTest()
        self.fixture.setUp()
        self.node = self.fixture.node

    def test_recorded_weak_duplicate_does_not_reject_tight_roi(self):
        # Round-six selected ROI and same-family raw boxes, all below .60.
        roi = (342.21615054729125, 291.4096609536631, 18.661382766400436, 16.481257596682553)
        candidates = [Candidate(box, 0, confidence, -1, False)
                      for box, confidence in [((341, 291, 21, 17), .482),
                          ((341, 292, 19, 16), .079), ((341, 284, 24, 24), .247),
                          ((341, 291, 21, 17), .055)]]
        matches = unique_roi_matches([(c, iou(c.box, roi)) for c in candidates], ((0, 1, 2),))
        self.assertEqual(2, len(matches))
        self.assertGreaterEqual(matches[0][1], .65)
        self.assertGreaterEqual(matches[0][1] - matches[1][1], .20)
        # Exercise the real initializer at in-image coordinates with the same
        # geometry; no ground-truth labels are injected into the adapter.
        shifted = tuple([roi[0] - 300, roi[1] - 250] + list(roi[2:]))
        local = [Candidate((c.box[0] - 300, c.box[1] - 250, *c.box[2:]),
                           c.class_id, c.confidence, -1, False) for c in candidates]
        result = self.node._initialize_fusion_selection(
            (self.fixture.image.header, self.fixture.frame, local), shifted)
        self.assertGreaterEqual(result.public_id, 0)
        self.assertEqual(0, result.class_id)

    def test_distinct_stable_identities_never_collapsed(self):
        a = Candidate((50, 50, 40, 40), 0, .9, 5, True)
        b = Candidate((51, 50, 40, 40), 0, .8, 6, True)
        self.assertEqual(2, len(unique_roi_matches([(a, .9), (b, .85)], ((0, 1),))))

    def test_exact_recorded_switchback_capture_duplicates(self):
        # New evidence includes the exact rejected capture 1795170079.0282798.
        roi = (316.1602658203233, 298.9844751282058, 17.69351855122028, 17.090392769389553)
        candidates = [Candidate(box, 0, confidence, -1, False)
                      for box, confidence in [((315, 298, 23, 19), .5555483103),
                          ((312, 299, 25, 19), .0887609348), ((312, 299, 27, 22), .0794760287)]]
        matches = unique_roi_matches([(c, iou(c.box, roi)) for c in candidates], ((0, 1, 2),))
        self.assertEqual(1, len(matches))
        self.assertEqual(candidates[0], matches[0][0])

    def test_part_whole_and_displaced_competitors_remain(self):
        a = Candidate((50, 50, 40, 40), 0, .8, -1)
        for box in [(50, 50, 30, 30), (60, 55, 40, 40)]:
            b = Candidate(box, 0, .9, -1)
            self.assertEqual(2, len(unique_roi_matches([(a, .6), (b, .5)], ((0, 1),))))

    def prepare_refresh(self):
        n, f = self.node, self.fixture
        n._tracker.backend.backend_name = 'dfine'
        n._inference_mailbox = LatestMailbox()
        n._pipeline_current = (n._epoch, f.image, f.frame)
        n._fusion.initialize(f.frame, (50, 50, 40, 40), f.image.header.stamp.to_sec(), 101)
        return n

    def test_new_selection_replaces_obsolete_job_without_backend_call(self):
        n = self.prepare_refresh()
        n._inference_mailbox.put(('obsolete',))
        self.assertTrue(n._refresh_selected_detector())
        epoch, message, frame, selection = n._inference_mailbox.get()
        self.assertEqual(n._epoch, epoch)
        self.assertIs(message, self.fixture.image)
        self.assertIs(frame, self.fixture.frame)
        self.assertEqual(101, selection[7])
        anchor = selection[4].copy()
        n._fusion.anchor[:] = 0
        self.assertTrue((anchor == selection[4]).all())
        self.assertIsNone(n._inference_mailbox.get())

    def test_refresh_rejects_old_epoch_old_capture_and_shutdown(self):
        n = self.prepare_refresh()
        original = n._pipeline_current
        n._pipeline_current = (n._epoch - 1, *original[1:])
        self.assertFalse(n._refresh_selected_detector())
        image = copy.deepcopy(original[1])
        image.header.stamp -= rospy.Duration(.001)
        n._pipeline_current = (n._epoch, image, original[2])
        self.assertFalse(n._refresh_selected_detector())
        n._pipeline_current = original
        n._closing = True
        self.assertFalse(n._refresh_selected_detector())
        self.assertIsNone(n._inference_mailbox.get())

    def test_unmatched_roi_is_not_given_old_target_class(self):
        n = self.prepare_refresh()
        result = n._initialize_fusion_selection(
            (self.fixture.image.header, self.fixture.frame, []), (50, 50, 40, 40))
        self.assertEqual(-1, result.class_id)
        self.assertEqual(result.public_id, n._inference_mailbox.get()[3][7])

    def test_first_selection_keeps_original_scheduling(self):
        n = self.prepare_refresh()
        n._fusion.reset()
        n._initialize_fusion_selection(
            (self.fixture.image.header, self.fixture.frame, []), (50, 50, 40, 40))
        self.assertIsNone(n._inference_mailbox.get())

    def unknown_selection(self, packet_epoch=None):
        n, packet, image = dispatch.LostDetectorDispatchTest().setup_node(packet_epoch, stamp=10.32)
        n._fusion.class_id = -1
        n._fusion.last_detection_stamp = 0.
        n._fusion._result(n._fusion.box, 'visual', 'initialized')
        return n, packet, image

    def test_unknown_roi_does_not_advance_clock_ahead_of_first_detection(self):
        n, _, _ = self.unknown_selection()
        n._frame_loop()
        n._process_image.assert_not_called()
        self.assertEqual(rospy.Time.from_sec(10.), n._last_capture_stamp)
        self.assertIsNotNone(n._inference_mailbox.get())

    def test_unknown_roi_accepts_first_original_detection_packet(self):
        n, packet, _ = self.unknown_selection(3)
        n._frame_loop()
        self.assertIs(packet.image, n._process_image.call_args.args[0])
        self.assertIs(packet, n._process_image.call_args.kwargs['packet'])

    def test_queued_old_identity_result_cannot_advance_new_selection_clock(self):
        n, _, _ = self.unknown_selection(3)
        n._fusion.public_id = 100
        n._frame_loop()
        n._process_image.assert_not_called()
        self.assertEqual(rospy.Time.from_sec(10.), n._last_capture_stamp)
        self.assertIn('old_selection_packet_discarded',
                      [c.args[0] for c in n._record_stage.call_args_list])

    def test_process_rechecks_packet_identity_under_original_lock(self):
        self.fixture.process()
        n = self.node
        n._fusion.initialize(self.fixture.frame, (50, 50, 40, 40),
                            self.fixture.image.header.stamp.to_sec(), 101)
        before = n._last_capture_stamp = self.fixture.image.header.stamp
        n._maximum_age = 0.
        n._reject_out_of_order = False
        packet = InferencePacket(self.fixture.image, self.fixture.frame, (), n._epoch, 25., '', 100)
        n._process_image(self.fixture.image, self.fixture.frame, packet=packet,
                         expected_epoch=n._epoch, result_only=True)
        self.assertEqual(before, n._last_capture_stamp)
        self.assertIsNone(n._publication_context.batch)


if __name__ == '__main__':
    unittest.main()
