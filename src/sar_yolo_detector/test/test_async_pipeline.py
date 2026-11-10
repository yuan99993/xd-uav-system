import threading
import time
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from sar_yolo_detector.async_pipeline import LatestMailbox, FlowDetections
from sar_yolo_detector.box_fusion import BoxFusion, Candidate, iou


def textured(shift=0):
    frame = np.zeros((160, 220, 3), np.uint8)
    frame[50:90, 50+shift:90+shift] = np.random.RandomState(7).randint(0, 240, (40, 40, 3))
    return frame


def detection(identity=5, stable=True):
    return SimpleNamespace(track_id=identity, class_id=2, confidence=.9,
        aabb_xyxy=(50, 50, 90, 90), center_xy=(70, 70), track_id_is_stable=stable)


class AsyncPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._previous_cv_threads = cv2.getNumThreads()
        cv2.setNumThreads(1)

    @classmethod
    def tearDownClass(cls):
        cv2.setNumThreads(cls._previous_cv_threads)

    def test_two_turning_targets_keep_geometry_between_detections(self):
        texture = np.random.RandomState(23).randint(40, 220, (36, 80, 3), dtype=np.uint8)
        def scene(index):
            frame = np.full((240, 360, 3), 20, np.uint8)
            detections = []
            for identity, cx, cy, sign in ((5, 95, 80, 1), (17, 255, 165, -1)):
                matrix = cv2.getRotationMatrix2D((40, 18), sign*index*7., 1.)
                matrix[:, 2] += (cx-40+index*.5, cy-18)
                corners = cv2.transform(np.float32([[[0, 0], [80, 0], [80, 36], [0, 36]]]), matrix)[0]
                left, top = corners.min(axis=0)
                right, bottom = corners.max(axis=0)
                warped = cv2.warpAffine(texture, matrix, (360, 240))
                mask = cv2.warpAffine(np.full((36, 80), 255, np.uint8), matrix, (360, 240))
                frame[mask > 0] = warped[mask > 0]
                detections.append(SimpleNamespace(track_id=identity, class_id=2, confidence=.9,
                    aabb_xyxy=(left, top, right, bottom), center_xy=((left+right)/2, (top+bottom)/2),
                    track_id_is_stable=True))
            return frame, detections
        flow = FlowDetections()
        from test_box_fusion import LocalTracker
        local = LocalTracker()
        fusion = BoxFusion(tracker=local)
        for index in range(31):
            frame, truth = scene(index)
            stamp = 10.+index*.1
            output = flow.update(frame, stamp, truth if index % 5 == 0 else None,
                frame if index % 5 == 0 else None, stamp if index % 5 == 0 else None)
            self.assertEqual({item.track_id for item in output}, {5, 17}, (index, flow.diagnostics))
            for item, target in zip(sorted(output, key=lambda d: d.track_id), truth):
                a, b = item.aabb_xyxy, target.aabb_xyxy
                self.assertGreater(iou((a[0], a[1], a[2]-a[0], a[3]-a[1]),
                                       (b[0], b[1], b[2]-b[0], b[3]-b[1])), .70, index)
            target = truth[0]
            x1, y1, x2, y2 = target.aabb_xyxy
            measured = Candidate((x1, y1, x2-x1, y2-y1), 2,
                .9 if index == 0 else .2, 5, True, stamp)
            if index == 0:
                fusion.initialize(frame, measured.box, stamp, 99, measured)
                local.valid = False
                continue
            cue = flow.visual_observations.get(5)
            visual = None
            if cue is not None:
                a, b, c, d = cue.aabb_xyxy
                visual = Candidate((a, b, c-a, d-b), 2, .65, 5, True, cue.detector_stamp)
            result = fusion.step(frame, stamp, [measured] if index % 5 == 0 else [],
                detector_updated=index % 5 == 0, visual_observation=visual,
                visual_quality=getattr(cue, 'flow_quality', 0.),
                visual_scale_ratio=getattr(cue, 'flow_scale_ratio', None))
            self.assertTrue(result.observed, (index, result.reason, fusion.diagnostics))
            self.assertEqual(result.public_id, 99)
            self.assertGreater(iou(result.box, measured.box), .70, index)

    def test_partially_visible_target_is_clipped_not_discarded(self):
        frame = textured()
        flow = FlowDetections()
        flow.update(frame, 10., [detection()], frame, 10.)
        # Known image rotation/translation may put a portion of a real target
        # outside the sensor. Keep only the observed, in-image portion.
        matrix = np.float64([[1, 0, -55], [0, 1, 0], [0, 0, 1]])
        moved = cv2.warpPerspective(frame, matrix, (220, 160))
        class Motion:
            def between(self, earlier, later):
                return matrix
        output = flow.update(moved, 10.1, motion=Motion())
        self.assertEqual(len(output), 1, flow.diagnostics)
        self.assertEqual(output[0].aabb_xyxy[0], 0.)
        self.assertGreater(output[0].aabb_xyxy[2], 30.)

    def test_latest_only_and_bounded_shutdown(self):
        queue = LatestMailbox()
        for index in range(1000):
            queue.put(index)
        self.assertEqual(queue.get(), 999)
        self.assertEqual(queue.replaced, 999)
        reader = threading.Thread(target=lambda: queue.get(5.))
        reader.start()
        queue.close()
        reader.join(.2)
        self.assertFalse(reader.is_alive())
        self.assertFalse(queue.put("after close"))

    def test_wake_does_not_overwrite_new_image(self):
        queue = LatestMailbox()
        queue.put("capture")
        queue.wake()
        self.assertEqual(queue.get(), "capture")
        self.assertIsNone(queue.get())

    def test_rebound_reseeds_selected_points_without_resetting_other_targets(self):
        flow = FlowDetections()
        flow.update(textured(), 10., [detection(5), detection(17), detection(21)], textured(), 10.)
        other = next(item for item in flow.items if item[0].track_id == 21)
        self.assertTrue(flow.reseed_identity(textured(), 10., 5, 17, (51, 50, 39, 40)))
        self.assertEqual({item[0].track_id for item in flow.items}, {17, 21})
        self.assertIs(next(item for item in flow.items if item[0].track_id == 21), other)
        rebound = next(item for item in flow.items if item[0].track_id == 17)
        self.assertEqual(rebound[0].detector_stamp, 10.)
        self.assertIsNotNone(rebound[2])
        self.assertFalse(flow.reseed_identity(textured(), 11., 17, 17, (51, 50, 39, 40)))

    def test_delayed_detection_is_aligned_not_relabelled_as_yolo(self):
        flow = FlowDetections()
        output = flow.update(textured(5), 10.1, [detection()], textured(), 10.)
        self.assertEqual(len(output), 1)
        item = output[0]
        self.assertEqual(item.measurement_source, "visual_tracker")
        self.assertEqual(item.detector_stamp, 10.)
        self.assertGreater(iou((item.aabb_xyxy[0], item.aabb_xyxy[1], 40, 40), (55, 50, 40, 40)), .9)

    def test_all_targets_advance_between_detector_frames(self):
        flow = FlowDetections()
        flow.update(textured(), 10., [detection()], textured(), 10.)
        for index in range(1, 6):
            output = flow.update(textured(index), 10.+index*.03)
            self.assertEqual(len(output), 1)
            self.assertEqual(output[0].track_id, 5)
            self.assertEqual(output[0].measurement_source, "visual_tracker")
        self.assertAlmostEqual(output[0].aabb_xyxy[0], 55, delta=.7)

    def test_flow_expires_and_never_creates_stable_id(self):
        flow = FlowDetections(max_age=.5)
        flow.update(textured(), 10., [detection()], textured(), 10.)
        self.assertFalse(flow.update(textured(), 10.6))
        flow.reset()
        self.assertTrue(flow.update(textured(), 11., [detection(-1, False)], textured(), 11.))
        self.assertFalse(flow.update(textured(), 11.1))

    def test_empty_detector_packets_do_not_drop_visual_or_refresh_its_ttl(self):
        flow = FlowDetections(max_age=.5)
        flow.update(textured(), 10., [detection()], textured(), 10.)
        for index in range(1, 5):
            stamp = 10.+index*.1
            output = flow.update(textured(index), stamp, [], textured(index), stamp)
            self.assertEqual(len(output), 1)
            self.assertEqual(output[0].measurement_source, "visual_tracker")
            self.assertEqual(output[0].detector_stamp, 10.)
        self.assertFalse(flow.update(textured(6), 10.6, [], textured(6), 10.6))

    def test_fresh_weak_detector_does_not_erase_independent_same_capture_flow(self):
        flow = FlowDetections()
        flow.update(textured(), 10., [detection()], textured(), 10.)
        weak = detection()
        weak.confidence = .2
        weak.aabb_xyxy = (51, 50, 91, 90)
        output = flow.update(textured(1), 10.1, [weak], textured(1), 10.1)
        self.assertEqual(output[0].measurement_source, 'detector')
        cue = flow.visual_observations[5]
        self.assertEqual(cue.measurement_source, 'visual_tracker')
        self.assertEqual(cue.detector_stamp, 10.)
        self.assertGreaterEqual(cue.flow_quality, .60)
        self.assertAlmostEqual(cue.aabb_xyxy[0], 51., delta=.5)
        flow.update(textured(2), 10.2)
        self.assertEqual(flow.visual_observations[5].detector_stamp, 10.1)
        flow.update(textured(2), 11.5)
        self.assertFalse(flow.visual_observations)

    def test_old_future_or_textureless_detection_cannot_create_current_box(self):
        for measured in (8., 11.):
            flow = FlowDetections()
            self.assertFalse(flow.update(textured(), 10., [detection()], textured(), measured))
        flow = FlowDetections()
        self.assertFalse(flow.update(np.zeros_like(textured()), 10.1,
            [detection()], np.zeros_like(textured()), 10.))

    def test_same_frame_result_correction_does_not_advance_kcf_twice(self):
        from test_box_fusion import LocalTracker, image
        local = LocalTracker()
        fusion = BoxFusion(tracker=local)
        frame = image()
        old = Candidate((60, 60, 40, 40), 2, .9, 5)
        fusion.initialize(frame, old.box, 10., 99, old)
        fusion.step(frame, 10.1, [], detector_updated=False)
        updates = local.updates
        new = Candidate(old.box, 2, .9, 5, True, 10.1)
        result = fusion.step(frame, 10.1, [new], same_frame_correction=True)
        self.assertTrue(result.observed)
        self.assertEqual(local.updates, updates)
        self.assertEqual(result.last_detection_stamp, 10.1)

    def test_intermediate_images_do_not_break_or_accelerate_id_confirmation(self):
        from test_box_fusion import LocalTracker, image
        fusion = BoxFusion(tracker=LocalTracker())
        frame = image()
        old = Candidate((60, 60, 40, 40), 2, .9, 5)
        fusion.initialize(frame, old.box, 10., 99, old)
        for index in range(1, 4):
            measured = 10. + index*.15
            new = Candidate(old.box, 2, .9, 7, True, measured)
            fusion.step(frame, measured, [new])
            if index < 3:
                self.assertEqual(fusion.local_id, 5)
                for delta in (.03, .06):
                    fusion.step(frame, measured+delta, [], detector_updated=False)
                self.assertEqual(fusion.pending_count, index)
        self.assertEqual(fusion.local_id, 7)
        self.assertEqual(fusion.public_id, 99)


if __name__ == "__main__":
    unittest.main()
