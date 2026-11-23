import gc
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np
from sar_yolo_detector.shared_image import SharedImagePublisher,read_shared_image,StaleSharedImage
class SharedImageTest(unittest.TestCase):
    def test_allocation_failure_removes_partial_slots_and_closes_descriptor(self):
        import os
        with tempfile.TemporaryDirectory() as directory:
            before=len(list(Path('/proc/self/fd').iterdir()))
            with patch('sar_yolo_detector.shared_image.os.ftruncate',side_effect=OSError('allocation failed')):
                with self.assertRaises(OSError):SharedImagePublisher(12,16,directory)
            self.assertEqual(list(Path(directory).iterdir()),[])
            self.assertEqual(len(list(Path('/proc/self/fd').iterdir())),before)
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.writer=SharedImagePublisher(12,16,self.temp.name)
    def tearDown(self):self.writer.close();self.temp.cleanup()
    def put(self,value):return self.writer.publish(np.full((12,16,3),value,np.uint8),1000000000+value,'camera',value)
    def test_four_leases_block_overwrite_and_numpy_views_retain_lease(self):
        frames=[]
        for value in range(4):frames.append(read_shared_image(self.put(value),self.temp.name))
        self.assertIsNone(self.put(8))
        view=np.asarray(frames[0])[:,2:5];frames[0]=None;gc.collect()
        self.assertIsNone(self.put(8));self.assertTrue((view==0).all())
        del view;gc.collect()
        self.assertIsNotNone(self.put(8))
        self.assertTrue((frames[1]==1).all())
    def test_stale_slot_is_rejected_and_copy_does_not_hold_lease(self):
        first=self.put(1)
        for value in range(2,6):self.put(value)
        with self.assertRaises(StaleSharedImage):read_shared_image(first,self.temp.name)
        d=self.put(7);frame=read_shared_image(d,self.temp.name);copy=frame.copy();del frame;gc.collect()
        for value in range(4):self.assertIsNotNone(self.put(value))
        self.assertTrue((copy==7).all())
    def test_invalid_descriptor_rejected_without_opening_arbitrary_files(self):
        d=self.put(1);d['name']='../etc/passwd'
        with self.assertRaises(ValueError):read_shared_image(d,self.temp.name)
        d=self.put(2);d['width']=10000000
        with self.assertRaises(ValueError):read_shared_image(d,self.temp.name)

class SharedCodecTest(unittest.TestCase):
    def test_metadata_decode_preserves_exact_capture_and_pixels_after_publisher_closes(self):
        import json
        from std_msgs.msg import String
        from sar_yolo_detector.shared_image import metadata_image
        import test_smart_tracker_image_codec as codec
        writer=SharedImagePublisher(12,16)
        try:
            pixels=np.arange(12*16*3,dtype=np.uint8).reshape(12,16,3)
            d=writer.publish(pixels,10000000001,'camera',17)
            message=metadata_image(String(data=json.dumps(d)))
            out,frame=codec.NODE._decode_pipeline_image(message,33554432)
            self.assertEqual(out.header.stamp.to_nsec(),10000000001)
            self.assertEqual(out.header.frame_id,'camera')
            self.assertEqual(bytes(out.data),pixels.tobytes())
            writer.close()
            np.testing.assert_array_equal(frame,pixels)
        finally:writer.close()

class SharedHistoryTest(unittest.TestCase):
    def test_reselection_history_does_not_pin_live_ring_slots(self):
        from collections import deque
        import json
        from std_msgs.msg import String
        from sar_yolo_detector.shared_image import metadata_image
        import test_smart_tracker_image_codec as codec
        node=codec.NODE.SmartTrackerNode.__new__(codec.NODE.SmartTrackerNode)
        node._fusion_snapshots=deque();node._fusion_snapshot_max_frames=24
        node._fusion_snapshot_max_age=1.;node._fusion_snapshot_max_bytes=33554432
        writer=SharedImagePublisher(12,16)
        try:
            for value in range(4):
                d=writer.publish(np.full((12,16,3),value,np.uint8),10000000000+value,'camera',value)
                msg,frame=codec.NODE._decode_pipeline_image(metadata_image(String(data=json.dumps(d))),33554432)
                node._cache_fusion_snapshot(msg,frame,())
                del msg,frame
            gc.collect()
            self.assertIsNotNone(writer.publish(np.zeros((12,16,3),np.uint8),10000000010,'camera',10))
            for i,snapshot in enumerate(node._fusion_snapshots):self.assertTrue((snapshot[1]==i).all())
        finally:writer.close()
