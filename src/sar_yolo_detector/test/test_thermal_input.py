#!/usr/bin/env python3
import struct
import threading
import time
import unittest

import rospy
import rostest

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from sar_yolo_detector.msg import ThermalImageInfo


class ThermalInputTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._detections = []
        self._info = []
        self._publisher = rospy.Publisher('/sar_thermal_test/image', Image,
                                          queue_size=2)
        self._detection_sub = rospy.Subscriber('/sar_thermal_test/detections',
                                               Detection2DArray,
                                               self._detection_callback)
        self._info_sub = rospy.Subscriber('/sar_thermal_test/info',
                                          ThermalImageInfo,
                                          self._info_callback)

    def _detection_callback(self, message):
        with self._lock:
            self._detections.append(message)

    def _info_callback(self, message):
        with self._lock:
            self._info.append(message)

    def _wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.rostime.wallsleep(0.01)
        return False

    def test_mono16_preserves_radiometric_contract(self):
        self.assertTrue(self._wait(lambda:
            self._publisher.get_num_connections() > 0 and
            self._detection_sub.get_num_connections() > 0 and
            self._info_sub.get_num_connections() > 0))
        width, height = 64, 48
        values = [1000 + x * 20 + y * 5
                  for y in range(height) for x in range(width)]
        message = Image()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'thermal_optical'
        message.width = width
        message.height = height
        message.encoding = 'mono16'
        message.is_bigendian = 0
        message.step = width * 2
        message.data = struct.pack('<{}H'.format(len(values)), *values)
        self._publisher.publish(message)
        self.assertTrue(self._wait(lambda: self._detections and self._info))
        with self._lock:
            detection = self._detections[-1]
            info = self._info[-1]
        self.assertEqual(detection.header.stamp, message.header.stamp)
        self.assertEqual(len(detection.detections), 1)
        self.assertEqual(info.raw_encoding, 'mono16')
        self.assertTrue(info.radiometric_input)
        self.assertGreater(info.upper_raw_value, info.lower_raw_value)
        self.assertTrue(info.bad_pixel_correction_applied)
        self.assertEqual(info.provenance.uav_id, 'uav_thermal')


if __name__ == '__main__':
    rospy.init_node('sar_thermal_input_test')
    rostest.rosrun('sar_yolo_detector', 'thermal_input', ThermalInputTest)
