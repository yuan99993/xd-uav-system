#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, VisionInfo


class DetectorInterfacesTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._detections = []
        self._vision_info = None
        self._publisher = rospy.Publisher(
            '/sar_yolo_detector_test/image', Image, queue_size=5)
        self._subscriber = rospy.Subscriber(
            '/sar_yolo_detector_test/detections', Detection2DArray,
            self._detections_callback, queue_size=5)
        self._vision_info_subscriber = rospy.Subscriber(
            '/sar_yolo_detector_test/vision_info', VisionInfo,
            self._vision_info_callback, queue_size=1)

    def _detections_callback(self, message):
        with self._lock:
            self._detections.append(message)

    def _vision_info_callback(self, message):
        with self._lock:
            self._vision_info = message

    def _wait(self, predicate, timeout=5.0):
        # Isolated rostests may inherit /use_sim_time without publishing a
        # clock.  Use wall time for test progress so a startup race cannot
        # turn into an infinite ROS-time wait.
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.rostime.wallsleep(0.01)
        return False

    @staticmethod
    def _image(stamp):
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = 'camera_optical_frame'
        message.height = 480
        message.width = 640
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = bytes(message.height * message.step)
        return message

    def test_capture_timestamp_and_standard_detection_output(self):
        self.assertTrue(self._wait(
            lambda: self._publisher.get_num_connections() > 0 and
            self._subscriber.get_num_connections() > 0))
        self.assertTrue(self._wait(lambda: self._vision_info is not None))
        stamp = rospy.Time.now()
        self._publisher.publish(self._image(stamp))
        self.assertTrue(self._wait(lambda: len(self._detections) >= 1))
        with self._lock:
            result = self._detections[-1]
            vision_info = self._vision_info
        self.assertEqual(result.header.stamp, stamp)
        self.assertEqual(result.header.frame_id, 'camera_optical_frame')
        self.assertEqual(len(result.detections), 1)
        detection = result.detections[0]
        self.assertEqual(detection.header.stamp, stamp)
        self.assertAlmostEqual(detection.bbox.center.x, 320.0, places=3)
        # 640x480 source is letterboxed with 80 px vertical padding, so the
        # deterministic model-space centre y=320 maps back to source y=240.
        self.assertAlmostEqual(detection.bbox.center.y, 240.0, places=3)
        self.assertAlmostEqual(detection.bbox.size_x, 140.0, places=3)
        self.assertAlmostEqual(detection.bbox.size_y, 220.0, places=3)
        self.assertEqual(detection.results[0].id, 0)
        self.assertAlmostEqual(detection.results[0].score, 0.95, places=3)
        self.assertIn('sar_yolo', vision_info.method)

        # A duplicate capture timestamp is deliberately rejected before it can
        # enter the latest-frame queue or create a second control measurement.
        with self._lock:
            previous_count = len(self._detections)
        self._publisher.publish(self._image(stamp))
        rospy.sleep(0.3)
        with self._lock:
            self.assertEqual(previous_count, len(self._detections))


if __name__ == '__main__':
    rospy.init_node('sar_yolo_detector_interfaces_test')
    rostest.rosrun('sar_yolo_detector', 'detector_interfaces',
                   DetectorInterfacesTest)
