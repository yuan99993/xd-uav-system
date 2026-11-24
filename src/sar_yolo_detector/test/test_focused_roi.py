import unittest

import numpy as np

from sar_yolo_detector.focused_roi import crop_focused_roi


class FocusedROITest(unittest.TestCase):
    def test_motion_margin_grows_into_bounded_graph_sizes(self):
        frame=np.zeros((640,640,3),np.uint8)
        crop,offset=crop_focused_roi(frame,(280,280,60,60),motion_margin_px=80,
                                     size_bins=(256,384,512,640))
        self.assertEqual((384,384,3),crop.shape)
        self.assertEqual((118,118),offset)
        crop,_=crop_focused_roi(frame,(280,280,20,20),motion_margin_px=8,size_bins=(256,384,512,640))
        self.assertEqual((256,256,3),crop.shape)

    def test_minimum_square_crop_preserves_pixels(self):
        image = np.zeros((1080, 1920, 3), np.uint8)
        crop, offset = crop_focused_roi(image, (900, 500, 50, 50))
        self.assertEqual((256, 256, 3), crop.shape)
        self.assertEqual((797, 397), offset)
        self.assertTrue(np.shares_memory(crop, image))

    def test_edges_keep_square_window(self):
        image = np.zeros((600, 800, 3), np.uint8)
        crop, offset = crop_focused_roi(image, (770, 570, 30, 30))
        self.assertEqual((657, 457), offset)
        self.assertEqual((256, 256, 3), crop.shape)

    def test_short_image_bounds_crop(self):
        crop, offset, mask = crop_focused_roi(np.zeros((100, 200, 3), np.uint8),
            (0, 0, 20, 20), return_mask=True)
        self.assertEqual((256, 256, 3), crop.shape)
        self.assertEqual((-118, -118), offset)
        self.assertEqual(255, int(mask[118, 118]))
        self.assertEqual(0, int(mask[0, 0]))

    def test_invalid_box_rejected(self):
        with self.assertRaises(ValueError):
            crop_focused_roi(np.zeros((100, 100, 3), np.uint8), (0, 0, float('nan'), 20))

    def test_edge_crop_uses_gray_padding_and_validity_mask(self):
        image = np.arange(100*200*3, dtype=np.uint8).reshape(100,200,3)
        crop, offset, mask = crop_focused_roi(image, (0,0,20,20), min_crop_size=80,
                                               reflect_padding=True, return_mask=True)
        self.assertEqual(crop.shape, (80,80,3))
        self.assertEqual(offset, (-30,-30))
        np.testing.assert_array_equal(crop[30:80,30:80], image[:50,:50])
        np.testing.assert_array_equal(crop[29,30],np.array([114,114,114],np.uint8))
        self.assertEqual(0, int(mask[29,30]))
        self.assertEqual(255, int(mask[30,30]))
