#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest

from sensor_msgs.msg import CameraInfo
from sar_yolo_detector.msg import (
    PerceptionIdentity,
    TrackedDetection2D,
    TrackedDetection2DArray,
)
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from xd_uav_track.msg import DetectionArray


class VisionToXdBridgeTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._outputs = []
        self._tracked_outputs = []
        self._input = rospy.Publisher(
            '/vision_to_xd_test/input', Detection2DArray, queue_size=2)
        self._tracked_input = rospy.Publisher(
            '/vision_to_xd_test/tracked_input', TrackedDetection2DArray,
            queue_size=2)
        self._camera_info = rospy.Publisher(
            '/vision_to_xd_test/camera_info', CameraInfo, queue_size=1,
            latch=True)
        self._output = rospy.Subscriber(
            '/vision_to_xd_test/output', DetectionArray,
            self._output_callback, queue_size=2)
        self._tracked_output = rospy.Subscriber(
            '/vision_to_xd_test/tracked_output', DetectionArray,
            self._tracked_output_callback, queue_size=2)

    def _output_callback(self, message):
        with self._lock:
            self._outputs.append(message)

    def _tracked_output_callback(self, message):
        with self._lock:
            self._tracked_outputs.append(message)

    def _wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.rostime.wallsleep(0.01)
        return False

    def test_standard_detection_is_converted_for_xd_detect(self):
        self.assertTrue(self._wait(
            lambda: self._input.get_num_connections() > 0 and
            self._output.get_num_connections() > 0))

        camera_info = CameraInfo()
        camera_info.width = 640
        camera_info.height = 480
        self._camera_info.publish(camera_info)
        rospy.rostime.wallsleep(0.1)

        message = Detection2DArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'down_camera_optical_frame'
        detection = Detection2D()
        detection.bbox.center.x = 620.0
        detection.bbox.center.y = 200.0
        detection.bbox.size_x = 60.0
        detection.bbox.size_y = 80.0
        weak = ObjectHypothesisWithPose(id=2, score=0.4)
        best = ObjectHypothesisWithPose(id=7, score=0.9)
        detection.results = [weak, best]
        message.detections = [detection]
        self._input.publish(message)

        self.assertTrue(self._wait(lambda: len(self._outputs) > 0))
        with self._lock:
            output = self._outputs[-1]
        # rospy assigns the publisher-side sequence number during publish;
        # capture time and optical frame are the safety-critical fields that
        # the bridge must preserve.
        self.assertEqual(output.header.stamp, message.header.stamp)
        self.assertEqual(output.header.frame_id, message.header.frame_id)
        self.assertEqual(output.image_width, 640)
        self.assertEqual(output.image_height, 480)
        self.assertEqual(output.image_source, 'eo')
        self.assertEqual(output.sensor_id, 'down_camera')
        self.assertEqual(output.detector_name, 'sar_yolo_test')
        self.assertEqual(output.model_version, 'test_v1')
        self.assertEqual(len(output.candidates), 1)
        candidate = output.candidates[0]
        self.assertEqual(candidate.class_id, 7)
        self.assertAlmostEqual(candidate.confidence, 0.9, places=5)
        self.assertEqual(list(candidate.bbox), [590, 160, 640, 240])
        self.assertTrue(candidate.has_bbox)
        self.assertFalse(candidate.track_id_is_stable)
        self.assertEqual(candidate.track_id, -1)
        self.assertFalse(candidate.range_valid)

    def test_tracked_detection_preserves_identity_and_provenance(self):
        self.assertTrue(self._wait(
            lambda: self._tracked_input.get_num_connections() > 0 and
            self._tracked_output.get_num_connections() > 0))

        message = TrackedDetection2DArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'down_camera_optical_frame'
        message.provenance = PerceptionIdentity(
            sensor_id='down_camera', model_version='yolo11n_coco80')
        tracked = TrackedDetection2D()
        tracked.track_id = 42
        tracked.track_id_is_stable = True
        tracked.detection.bbox.center.x = 100.0
        tracked.detection.bbox.center.y = 80.0
        tracked.detection.bbox.size_x = 40.0
        tracked.detection.bbox.size_y = 20.0
        tracked.detection.results = [
            ObjectHypothesisWithPose(id=0, score=0.88)]
        message.detections = [tracked]
        self._tracked_input.publish(message)

        self.assertTrue(self._wait(lambda: len(self._tracked_outputs) > 0))
        with self._lock:
            output = self._tracked_outputs[-1]
        self.assertEqual(output.header.stamp, message.header.stamp)
        self.assertEqual(output.sensor_id, 'down_camera')
        self.assertEqual(output.detector_name, 'sar_yolo_smart_tracker')
        self.assertEqual(output.model_version, 'yolo11n_coco80')
        self.assertEqual(len(output.candidates), 1)
        candidate = output.candidates[0]
        self.assertEqual(candidate.track_id, 42)
        self.assertTrue(candidate.track_id_is_stable)
        self.assertEqual(candidate.class_id, 0)
        self.assertAlmostEqual(candidate.confidence, 0.88, places=5)
        self.assertEqual(list(candidate.bbox), [80, 70, 120, 90])


if __name__ == '__main__':
    rospy.init_node('vision_to_xd_bridge_interface_test')
    rostest.rosrun('sar_yolo_detector', 'vision_to_xd_bridge',
                   VisionToXdBridgeTest)
