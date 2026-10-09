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
