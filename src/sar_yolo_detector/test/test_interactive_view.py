import unittest
from types import SimpleNamespace as NS
from sar_yolo_detector.interactive_view import FramePair, image_point
from sar_yolo_detector.box_fusion import Candidate, class_duplicate_indices


class InteractiveViewTest(unittest.TestCase):
    def image(self, sec=1, seq=999):
        return NS(header=NS(stamp=NS(secs=sec, nsecs=2), frame_id='camera', seq=seq))

    def evidence(self, sec=1):
        return dict(capture_header=dict(seq=7, frame_id='camera', stamp=dict(secs=sec, nsecs=2)))

    def test_original_pixel_mapping(self):
        self.assertEqual(image_point(400, 300, 800, 600, 640, 480), (320, 240))
        self.assertEqual(image_point(0, 0, 640, 480, 640, 480), (0, 0))

    def test_letterbox_is_not_selectable(self):
        self.assertIsNone(image_point(20, 10, 1000, 480, 640, 480))
        self.assertEqual(image_point(180, 0, 1000, 480, 640, 480), (0, 0))
        self.assertIsNone(image_point(820, 0, 1000, 480, 640, 480))
        self.assertIsNone(image_point(float('nan'), 5, 640, 480, 640, 480))

    def test_match_ignores_rewritten_image_sequence(self):
        cache = FramePair()
        cache.add_image(self.image())
        cache.add_evidence(self.evidence())
        self.assertEqual(cache.take()[1]['capture_header']['seq'], 7)
        self.assertIsNone(cache.take())

    def test_never_substitute_latest_metadata(self):
        cache = FramePair()
        cache.add_image(self.image())
        cache.add_evidence(self.evidence(2))
        self.assertIsNone(cache.take())
        cache.add_evidence(self.evidence())
        self.assertIsNotNone(cache.take())

    def test_bounded_latest_only(self):
        cache = FramePair(3)
        for sec in range(100):
            cache.add_image(self.image(sec))
            cache.add_evidence(self.evidence(sec))
        self.assertEqual(len(cache.metadata), 3)
        self.assertEqual(cache.take()[0].header.stamp.secs, 99)

    def test_selected_output_duplicate_does_not_change_input_gates(self):
        boxes = [Candidate((10, 10, 40, 40), 2, .8, 5),
                 Candidate((11, 10, 40, 40), 2, .9, 7)]
        self.assertEqual(class_duplicate_indices(boxes, [[0, 1, 2, 3]], 5), [0, 1])
        self.assertEqual(class_duplicate_indices(boxes, [[0, 1, 2, 3]], 5,
                         suppress_selected_same_class=True), [0])
        boxes[1] = Candidate((25, 10, 40, 40), 2, .9, 7)
        self.assertEqual(class_duplicate_indices(boxes, [[0, 1, 2, 3]], 5,
                         suppress_selected_same_class=True), [0, 1])


if __name__ == '__main__':
    unittest.main()
