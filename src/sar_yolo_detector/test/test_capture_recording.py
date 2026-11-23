import unittest
import time
import json
import rosbag
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from sar_yolo_detector.capture_recording import AcknowledgedBag,capture_header
from sar_yolo_detector.rgb_recording import inspect_bag
import test_rgb_recording as recording_fixture
class CaptureRecordingTest(unittest.TestCase):
    def setUp(self):self.fixture=recording_fixture.RecordingTests();self.fixture.setUp()
    def tearDown(self):self.fixture.tearDown()
    def test_ack_requires_pixels_and_exact_info_and_pose_and_roundtrips_bag(self):
        f=self.fixture;acks=[];im,info,tf=f.messages(1)
        with rosbag.Bag(str(f.path),'w') as bag:
            w=AcknowledgedBag(bag,f.source,lambda *args:acks.append(args))
            w.typed(f.source['image_topic'],im,rospy.Time(101));self.assertEqual(acks,[])
            # Wrong-capture calibration must not qualify this image.
            _,wrong,_=f.messages(2)
            w.typed(f.source['camera_info_topic'],wrong,rospy.Time(101));self.assertEqual(acks,[])
            w.typed('/tf',TFMessage(transforms=[tf]),rospy.Time(101));self.assertEqual(acks,[])
            w.typed(f.source['camera_info_topic'],info,rospy.Time(101))
            self.assertEqual(acks,[])
            def evidence(image):
                return String(data=json.dumps(dict(capture_header=dict(frame_id='capture_optical',stamp=dict(secs=image.header.stamp.secs,nsecs=image.header.stamp.nsecs)))))
            w.typed('/tracker/fusion_evidence',evidence(im),rospy.Time(101))
            self.assertEqual(acks,[(im.header.stamp.to_nsec(),'capture_optical')])
            self.assertFalse(w.images)
            im2,_,tf2=f.messages(2)
            w.typed('/tf',TFMessage(transforms=[tf2]),rospy.Time(102))
            w.typed(f.source['image_topic'],im2,rospy.Time(102))
            w.typed('/tracker/fusion_evidence',evidence(im2),rospy.Time(102))
        with rosbag.Bag(str(f.path)) as bag:
            summary=inspect_bag(bag,f.source)
            images=list(bag.read_messages(topics=[f.source['image_topic']]))
            self.assertEqual(bytes(images[0].message.data),bytes(im.data))
            self.assertEqual(images[0].timestamp,rospy.Time(101))
    def test_buffer_overflow_fails_without_ack(self):
        f=self.fixture;acks=[]
        with rosbag.Bag(str(f.path),'w') as bag:
            w=AcknowledgedBag(bag,f.source,lambda *args:acks.append(args),maximum_images=1)
            w.typed(f.source['image_topic'],f.messages(1)[0],rospy.Time(101))
            with self.assertRaises(RuntimeError):w.typed(f.source['image_topic'],f.messages(2)[0],rospy.Time(102))
            self.assertEqual(acks,[])
    def test_metadata_timeout_and_bad_header_fail(self):
        f=self.fixture
        with rosbag.Bag(str(f.path),'w') as bag:
            w=AcknowledgedBag(bag,f.source,lambda *_:None)
            im=f.messages(1)[0];w.typed(f.source['image_topic'],im,rospy.Time(101))
            stamp=im.header.stamp.to_nsec();entry=w.images[stamp]
            w.images[stamp]=entry[:3]+(time.monotonic()-3.,)
            with self.assertRaises(RuntimeError):w.check()
        with self.assertRaises(ValueError):capture_header(b'bad')

class RecordingChannelTest(unittest.TestCase):
    def test_dedicated_index_keeps_live_provenance_and_rejects_missing_channel(self):
        from sar_yolo_detector.rgb_recording import recording_source
        f=recording_fixture.RecordingTests();f.setUp()
        try:
            source=dict(f.source,record_image_topic='/capture/image_record')
            indexed=recording_source(source)
            self.assertEqual(source['image_topic'],'/capture/image')
            self.assertEqual(indexed['image_topic'],'/capture/image_record')
            self.assertEqual(indexed['live_image_topic'],'/capture/image')
            with self.assertRaises(ValueError):recording_source(f.source)
        finally:f.tearDown()
