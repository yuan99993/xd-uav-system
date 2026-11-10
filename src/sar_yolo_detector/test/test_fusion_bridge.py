#!/usr/bin/env python3
"""The additive fusion input must retain the unchanged XD output contract."""
import copy
import threading
import time
import unittest

import rospy
import rostest
from sar_yolo_detector.msg import FusionDetection2D, FusionDetection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from xd_uav_track.msg import DetectionArray


class FusionBridgeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.outputs = []
        self.sub = rospy.Subscriber('/fusion_bridge_test/output', DetectionArray, self.receive, queue_size=2)
        self.pub = rospy.Publisher('/fusion_bridge_test/input', FusionDetection2DArray, queue_size=1)

    def receive(self, message):
        with self.lock:
            self.outputs.append(message)
            self.outputs = self.outputs[-12:]

    def tearDown(self):
        self.sub.unregister()
        self.pub.unregister()

    def send(self, message):
        deadline = time.monotonic() + 4.
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.pub.publish(message)
            time.sleep(.04)
            with self.lock:
                # ROS1 publishers maintain their own header.seq counters.
                matching = [m for m in self.outputs if m.header.stamp == message.header.stamp
                            and m.header.frame_id == message.header.frame_id]
                if matching:
                    return matching[-1]
        self.fail('Bridge output timeout')

    @staticmethod
    def message():
        message = FusionDetection2DArray(image_width=320, image_height=240)
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'camera_optical'
        message.provenance.sensor_id = 'gimbal_camera'
        message.provenance.model_version = 'train7'
        entry = FusionDetection2D(measurement_source='visual_tracker')
        entry.last_detector_stamp = message.header.stamp - rospy.Duration(.2)
        entry.target.track_id = 73
        entry.target.track_id_is_stable = True
        entry.target.detection.header = message.header
        box = entry.target.detection.bbox
        box.center.x, box.center.y = 100.4, 70.8
        box.size_x, box.size_y = 40., 30.
        entry.target.detection.results = [ObjectHypothesisWithPose(id=2, score=.6)]
        message.detections = [entry]
        return message

    def test_visual_source_original_pixels_and_identity(self):
        source = self.message()
        output = self.send(source)
        self.assertEqual((output.image_width, output.image_height), (320, 240))
        self.assertEqual(output.image_source, 'gimbal_rgb')
        self.assertEqual(output.sensor_id, 'gimbal_camera')
        self.assertEqual(output.model_version, 'train7')
        self.assertEqual(len(output.candidates), 1)
        target = output.candidates[0]
        self.assertEqual(target.track_id, 73)
        self.assertTrue(target.track_id_is_stable)
        self.assertEqual(target.measurement_source, 'visual_tracker')
        self.assertEqual(list(target.bbox), [80, 55, 121, 86])
        self.assertFalse(target.has_relative_position_body)

    def test_invalid_source_and_future_detector_stamp_are_rejected(self):
        message = self.message()
        message.detections[0].measurement_source = 'prediction'
        future = copy.deepcopy(message.detections[0])
        future.measurement_source = 'visual_tracker'
        future.last_detector_stamp = message.header.stamp + rospy.Duration(1.)
        message.detections.append(future)
        self.assertEqual(self.send(message).candidates, [])

    def test_detector_and_empty_arrays_keep_contract(self):
        message = self.message()
        message.detections[0].measurement_source = 'detector'
        self.assertEqual(self.send(message).candidates[0].measurement_source, 'detector')
        message.header.stamp = rospy.Time.now()
        message.detections = []
        self.assertEqual(self.send(message).candidates, [])


if __name__ == '__main__':
    rospy.init_node('test_fusion_bridge')
    rostest.rosrun('sar_yolo_detector', 'fusion_bridge', FusionBridgeTest)
