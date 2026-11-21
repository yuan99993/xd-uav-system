#!/usr/bin/env python3
"""Tests for the cv_bridge-free SmartTracker ROS image boundary."""

import importlib.util
import unittest
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs.msg import Image


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "smart_tracker_node.py"
SPEC = importlib.util.spec_from_file_location("smart_tracker_node", str(SCRIPT))
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


class SmartTrackerImageCodecTest(unittest.TestCase):
    def test_cuda_jpeg_path_preserves_bgr_pixels_dimensions_and_capture_header(self):
        try:
            import torch
        except ImportError:self.skipTest('Optional CUDA runtime unavailable')
        if not torch.cuda.is_available():self.skipTest('CUDA unavailable')
        import cv2
        from sensor_msgs.msg import CompressedImage
        from sar_yolo_detector.jpeg_decode import JPEGDecoder
        source=np.full((64,96,3),(15,60,180),np.uint8)
        _,encoded=cv2.imencode('.jpg',source,[cv2.IMWRITE_JPEG_QUALITY,98])
        decoder=JPEGDecoder('cuda',100000)
        message=CompressedImage(format='jpeg',data=encoded.tobytes())
        message.header.stamp=rospy.Time(123,45);message.header.frame_id='camera'
        raw,pixels=NODE._decode_pipeline_image(message,100000,decoder)
        self.assertEqual(raw.header,message.header);self.assertEqual(pixels.shape,source.shape)
        self.assertEqual(pixels.dtype,np.uint8)
        reference=cv2.imdecode(encoded,cv2.IMREAD_COLOR)
        self.assertLess(np.abs(pixels.astype(float)-reference).mean(),2.)
        with self.assertRaises(ValueError):JPEGDecoder('cpu',10)(message.data)

    def test_decoder_keeps_latest_frame_while_processing_consumer_is_busy(self):
        import cv2
        import threading
        import time
        from sensor_msgs.msg import CompressedImage
        from sar_yolo_detector.async_pipeline import LatestMailbox
        node=NODE.SmartTrackerNode.__new__(NODE.SmartTrackerNode)
        node._image_mailbox=LatestMailbox();node._decoded_mailbox=LatestMailbox()
        node._maximum_image_bytes=100000;node._lock=threading.RLock();node._rejected_frames=0
        _,encoded=cv2.imencode('.jpg',np.full((48,64,3),90,np.uint8))
        worker=threading.Thread(target=node._decode_loop);worker.start()
        try:
            for seq in range(20):
                message=CompressedImage(format='jpeg',data=encoded.tobytes())
                message.header.seq=seq;message.header.frame_id='camera';message.header.stamp=rospy.Time(100,seq)
                node._image_callback(message);time.sleep(.005)
            deadline=time.monotonic()+2
            while node._decoded_mailbox.replaced<19 and time.monotonic()<deadline:time.sleep(.005)
            raw,pixels=node._decoded_mailbox.get(.2)
            self.assertEqual(raw.header.seq,19)
            self.assertEqual(raw.header.stamp,rospy.Time(100,19))
            self.assertGreater(node._decoded_mailbox.replaced,0)
            np.testing.assert_array_equal(pixels,cv2.imdecode(encoded,cv2.IMREAD_COLOR))
        finally:
            node._image_mailbox.close();node._decoded_mailbox.close();worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_slow_preview_has_bounded_pending_work_and_keeps_capture_header(self):
        import threading
        import time
        from sar_yolo_detector.async_pipeline import LatestMailbox
        node=NODE.SmartTrackerNode.__new__(NODE.SmartTrackerNode)
        node._preview_mailbox=LatestMailbox();entered=threading.Event();release=threading.Event();seen=[]
        def render(message,frame,overlay,epoch):
            seen.append((message.header.seq,int(frame[0,0,0]),overlay,epoch))
            if len(seen)==1:entered.set();release.wait(2)
        node._publish_preview=render
        worker=threading.Thread(target=node._preview_loop);worker.start()
        try:
            for seq in range(20):
                message=Image();message.header.seq=seq
                node._preview_mailbox.put((message,np.full((1,1,3),seq,np.uint8),('capture',seq),7))
                if seq==0:self.assertTrue(entered.wait(1))
            self.assertGreater(node._preview_mailbox.replaced,0)
            release.set();deadline=time.monotonic()+2
            while len(seen)<2 and time.monotonic()<deadline:time.sleep(.005)
            self.assertEqual(seen,[(0,0,('capture',0),7),(19,19,('capture',19),7)])
        finally:
            release.set();node._preview_mailbox.close();worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_compressed_capture_is_decoded_once_without_changing_header_or_pixels(self):
        import cv2
        from sensor_msgs.msg import CompressedImage
        original=np.random.RandomState(42).randint(0,255,(48,64,3),dtype=np.uint8)
        ok,encoded=cv2.imencode('.jpg',original,[cv2.IMWRITE_JPEG_QUALITY,98])
        self.assertTrue(ok)
        source=CompressedImage(format='jpeg',data=encoded.tobytes())
        source.header.seq=12;source.header.frame_id='rgb';source.header.stamp=rospy.Time(123,456)
        raw,pixels=NODE._decode_pipeline_image(source,100000)
        self.assertEqual(raw.header,source.header)
        self.assertIsNot(raw.header,source.header)
        np.testing.assert_array_equal(pixels,cv2.imdecode(encoded,cv2.IMREAD_COLOR))
        np.testing.assert_array_equal(NODE._image_to_bgr(raw),pixels)
        with self.assertRaises(ValueError):NODE._decode_pipeline_image(source,10)
        source.data=b'corrupt jpeg'
        with self.assertRaises(ValueError):NODE._decode_pipeline_image(source,100000)

    def test_bgr8_padding_is_removed(self):
        message = Image()
        message.height = 2
        message.width = 2
        message.encoding = "bgr8"
        message.step = 8
        message.data = bytes([
            1, 2, 3, 4, 5, 6, 99, 99,
            7, 8, 9, 10, 11, 12, 99, 99,
        ])
        decoded = NODE._image_to_bgr(message)
        self.assertEqual(decoded.shape, (2, 2, 3))
        self.assertEqual(decoded.tolist(), [
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]],
        ])

    def test_rgb_and_mono16_are_converted_to_bgr(self):
        rgb = Image()
        rgb.height = 1
        rgb.width = 1
        rgb.encoding = "rgb8"
        rgb.step = 3
        rgb.data = bytes([10, 20, 30])
        self.assertEqual(NODE._image_to_bgr(rgb).tolist(), [[[30, 20, 10]]])

        mono = Image()
        mono.height = 2
        mono.width = 2
        mono.encoding = "mono16"
        mono.step = 4
        mono.is_bigendian = 0
        mono.data = np.array([[0, 100], [200, 300]], dtype="<u2").tobytes()
        decoded = NODE._image_to_bgr(mono)
        self.assertEqual(decoded.shape, (2, 2, 3))
        self.assertEqual(decoded.dtype, np.uint8)
        self.assertTrue(np.array_equal(decoded[:, :, 0], decoded[:, :, 1]))

    def test_malformed_image_fails_closed(self):
        message = Image()
        message.height = 2
        message.width = 2
        message.encoding = "bgr8"
        message.step = 6
        message.data = bytes(6)
        with self.assertRaisesRegex(ValueError, "shorter"):
            NODE._image_to_bgr(message)

    def test_annotated_output_is_tightly_packed(self):
        header = Image().header
        header.stamp = rospy.Time.from_sec(123.5)
        frame = np.zeros((3, 4, 3), dtype=np.uint8)
        message = NODE._bgr_to_image(frame, header)
        self.assertEqual(message.header.stamp, header.stamp)
        self.assertEqual(message.encoding, "bgr8")
        self.assertEqual(message.step, 12)
        self.assertEqual(len(message.data), 36)


if __name__ == "__main__":
    unittest.main()
