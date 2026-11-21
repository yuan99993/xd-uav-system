import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from sar_yolo_detector.rgb_recording import (validate_source,inspect_bag,write_manifest,
    load_manifest,remap_capture,image_bgr,source_from_bag)

class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.path=Path(self.temp.name)/'rgb.bag'
        self.source=dict(modality='rgb',thermal_sensor_model=False,encoding='bgr8',
            producer='UE.SceneCapture2D.FinalColorLDR',is_replay=False,
            image_topic='/capture/image',camera_info_topic='/capture/info',frame_id='capture_optical')
    def tearDown(self):self.temp.cleanup()
    def messages(self,index):
        stamp=rospy.Time(100,index*100000000)
        im=Image();im.header.stamp=stamp;im.header.frame_id='capture_optical'
        im.width=16;im.height=12;im.encoding='bgr8';im.step=48;im.data=bytes([20,30,40])*192
        info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='capture_optical'
        info.width=16;info.height=12;info.K=[8,0,8,0,8,6,0,0,1];info.D=[0.]*5
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='source_world'
        tf.child_frame_id='capture_optical';tf.transform.rotation.w=1.;tf.transform.translation.z=-30
        return im,info,tf
    def bag(self,condition='valid'):
        with rosbag.Bag(str(self.path),'w') as bag:
            bag.write('/camera/source',String(data=json.dumps(self.source)),rospy.Time(100))
            for i in range(2):
                im,info,tf=self.messages(i)
                if condition=='latest_k' and i==1:info.header.stamp=rospy.Time(100)
                if condition=='wrong_frame':im.header.frame_id='wrong_optical'
                if condition=='mono':im.encoding='mono8'
                if condition=='duplicate_capture' and i==1:im.header.stamp=rospy.Time(100)
                if condition=='bad_pose':tf.transform.rotation.w=2
                bag.write(self.source['camera_info_topic'],info,rospy.Time(101,i*100000000))
                if condition!='missing_pose':bag.write('/tf',TFMessage(transforms=[tf]),rospy.Time(101,i*100000000))
                bag.write(self.source['image_topic'],im,rospy.Time(101,i*100000000))
        return self.path
    def test_exact_rgb_pairing_and_manifest(self):
        self.bag();m=write_manifest(self.path,self.source)
        self.assertEqual(m['summary']['frames'],2);self.assertFalse(m['infrared_validated'])
        self.assertEqual(load_manifest(self.path)['bag_sha256'],m['bag_sha256'])
    def test_no_latest_calibration_no_missing_pose_no_implicit_gray_conversion(self):
        for condition in ['latest_k','missing_pose','mono','wrong_frame','duplicate_capture','bad_pose']:
            self.bag(condition)
            with rosbag.Bag(str(self.path)) as bag:
                with self.assertRaises(ValueError,msg=condition):inspect_bag(bag,self.source)
    def test_no_infrared_claim_from_three_channel_or_gray_data(self):
        for change in [dict(modality='infrared'),dict(thermal_sensor_model=True),dict(encoding='mono8'),dict(modality='pseudo_color_ir')]:
            with self.assertRaises(ValueError):validate_source(dict(self.source,**change))
        with self.assertRaises(ValueError):validate_source({'encoding':'bgr8'})
    def test_recording_integrity_is_checked_before_replay(self):
        self.bag();write_manifest(self.path,self.source)
        with self.path.open('ab') as stream:stream.write(b'corruption')
        with self.assertRaises(ValueError):load_manifest(self.path)
    def test_retime_tuple_is_exact_and_does_not_mutate_original(self):
        im,info,tf=self.messages(1);stamp=rospy.Time(900,123456789)
        ri,rc,rt=remap_capture(im,info,tf,stamp,'isolated_optical','isolated_world')
        self.assertEqual(ri.header.stamp,rc.header.stamp);self.assertEqual(rc.header.stamp,rt.header.stamp)
        self.assertEqual(rt.header.frame_id,'isolated_world');self.assertEqual(rt.child_frame_id,'isolated_optical')
        self.assertEqual(im.header.stamp,rospy.Time(100,100000000));self.assertEqual(tf.header.frame_id,'source_world')
    def test_bgr_pixels_and_row_padding_preserved(self):
        im,_,_=self.messages(1);im.step=50;im.data=(bytes([20,30,40])*16+b'xx')*12
        values=image_bgr(im);self.assertEqual(values.shape,(12,16,3));self.assertEqual(values[0,0].tolist(),[20,30,40])
        im.encoding='rgb8';self.assertEqual(image_bgr(im)[0,0].tolist(),[40,30,20])
    def test_recorded_source_is_required(self):
        self.bag()
        with rosbag.Bag(str(self.path)) as bag:
            self.assertEqual(source_from_bag(bag,'/camera/source'),self.source)
            with self.assertRaises(ValueError):source_from_bag(bag,'/absent')
    def test_metadata_capacity_is_bounded(self):
        from sar_yolo_detector.rgb_recording import calibration_index
        self.bag()
        with rosbag.Bag(str(self.path)) as bag:
            with self.assertRaises(ValueError):calibration_index(bag,self.source,maximum=1)

if __name__=='__main__':unittest.main()
