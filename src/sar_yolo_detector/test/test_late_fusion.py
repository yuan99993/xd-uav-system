#!/usr/bin/env python3
import threading
import time
import unittest
import copy

import rospy
import rostest

from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)


class LateFusionTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._outputs = []
        self._eo = rospy.Publisher('/sar_late_fusion_test/eo', Detection2DArray,
                                   queue_size=2)
        self._ir = rospy.Publisher('/sar_late_fusion_test/ir', Detection2DArray,
                                   queue_size=2)
        self._subscription = rospy.Subscriber('/sar_late_fusion_test/fused',
                                              Detection2DArray,
                                              self._callback, queue_size=4)

    def _callback(self, message):
        with self._lock:
            self._outputs.append(message)

    def _wait(self, predicate, timeout=4.0):
        # The test must also be valid when the workspace has a global
        # /use_sim_time setting but this isolated rostest intentionally does
        # not publish /clock.  A ROS-time deadline could then never elapse.
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.rostime.wallsleep(0.01)
        return False

    @staticmethod
    def _frame(stamp, score, center_x=200.0, center_y=150.0,
               size_x=80.0, size_y=120.0):
        detection = Detection2D()
        detection.header.stamp = stamp
        detection.header.frame_id = 'test_camera'
        detection.bbox = BoundingBox2D()
        detection.bbox.center.x = center_x
        detection.bbox.center.y = center_y
        detection.bbox.size_x = size_x
        detection.bbox.size_y = size_y
        result = ObjectHypothesisWithPose()
        result.id = 0
        result.score = score
        detection.results = [result]
        frame = Detection2DArray()
        frame.header.stamp = stamp
        frame.header.frame_id = 'test_camera'
        frame.detections = [detection]
        return frame

    @classmethod
    def _two_eo_frame(cls, stamp):
        frame = cls._frame(stamp, 0.70, center_x=195.0)
        second = copy.deepcopy(frame.detections[0])
        second.bbox.center.x = 205.0
        frame.detections.append(second)
        return frame

    def test_calibrated_pair_and_unimodal_fallback(self):
        self.assertTrue(self._wait(
            lambda: self._eo.get_num_connections() > 0 and
            self._ir.get_num_connections() > 0 and
            # Waiting for the reverse output connection matters on a loaded
            # CI host: a valid fused result is otherwise published before
            # this test's callback TCP connection is established.
            self._subscription.get_num_connections() > 0))
        paired_stamp = rospy.Time.now()
        self._eo.publish(self._frame(paired_stamp, 0.70))
        self._ir.publish(self._frame(paired_stamp + rospy.Duration(0.02), 0.80))
        self.assertTrue(self._wait(lambda: len(self._outputs) >= 1))
        with self._lock:
            paired = self._outputs[-1]
        self.assertEqual(paired.header.frame_id, 'test_eo_optical')
        self.assertEqual(len(paired.detections), 1)
        result = paired.detections[0]
        self.assertEqual(result.results[0].id, 0)
        # Independent-support fusion: 1-(1-0.7)*(1-0.8) = 0.94.
        self.assertAlmostEqual(result.results[0].score, 0.94, places=4)
        self.assertAlmostEqual(result.bbox.center.x, 200.0, places=3)
        self.assertAlmostEqual(result.bbox.center.y, 150.0, places=3)

        with self._lock:
            count = len(self._outputs)
        self._eo.publish(self._frame(rospy.Time.now(), 0.70))
        self.assertTrue(self._wait(lambda: len(self._outputs) > count))
        with self._lock:
            fallback = self._outputs[-1]
        self.assertEqual(len(fallback.detections), 1)
        self.assertAlmostEqual(fallback.detections[0].results[0].score,
                               0.70, places=4)

        # One IR detection may support at most one EO box. The second EO box
        # must remain unimodal instead of receiving the same IR evidence.
        with self._lock:
            count = len(self._outputs)
        unique_stamp = rospy.Time.now()
        self._eo.publish(self._two_eo_frame(unique_stamp))
        self._ir.publish(self._frame(unique_stamp + rospy.Duration(0.01),
                                     0.80))
        self.assertTrue(self._wait(lambda: len(self._outputs) > count))
        with self._lock:
            unique = self._outputs[-1]
        self.assertEqual(len(unique.detections), 2)
        scores = sorted(d.results[0].score for d in unique.detections)
        self.assertAlmostEqual(scores[0], 0.70, places=4)
        self.assertAlmostEqual(scores[1], 0.94, places=4)


if __name__ == '__main__':
    rospy.init_node('sar_late_fusion_test')
    rostest.rosrun('sar_yolo_detector', 'late_fusion', LateFusionTest)
