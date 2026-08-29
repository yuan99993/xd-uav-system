#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from sar_yolo_detector.msg import TaskCandidateArray
from sar_yolo_detector.srv import GetTaskCandidates


class TaskGenerationGateTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._arrays = []
        self._publisher = rospy.Publisher('/sar_task_gate_test/detections',
                                          Detection2DArray, queue_size=2)
        self._subscriber = rospy.Subscriber('/sar_task_gate_test/candidates',
                                             TaskCandidateArray,
                                             self._callback, queue_size=5)

    def _callback(self, message):
        with self._lock:
            self._arrays.append(message)

    def _wait(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.rostime.wallsleep(0.01)
        return False

    def test_unvalidated_model_cannot_generate_candidate(self):
        self.assertTrue(self._wait(lambda:
            self._publisher.get_num_connections() > 0 and
            self._subscriber.get_num_connections() > 0))
        message = Detection2DArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'camera_optical'
        detection = Detection2D()
        detection.bbox.center.x = 100.0
        detection.bbox.center.y = 100.0
        detection.bbox.size_x = 20.0
        detection.bbox.size_y = 40.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = 0
        hypothesis.score = 0.99
        detection.results.append(hypothesis)
        message.detections.append(detection)
        self._publisher.publish(message)
        self.assertTrue(self._wait(lambda: len(self._arrays) >= 2))
        with self._lock:
            self.assertTrue(all(not array.candidates for array in self._arrays))
        rospy.wait_for_service('/sar_task_gate_test/candidates/get_snapshot', 2.0)
        response = rospy.ServiceProxy(
            '/sar_task_gate_test/candidates/get_snapshot',
            GetTaskCandidates)()
        self.assertTrue(response.snapshot.full_snapshot)
        self.assertEqual(len(response.snapshot.candidates), 0)


if __name__ == '__main__':
    rospy.init_node('sar_task_generation_gate_test')
    rostest.rosrun('sar_yolo_detector', 'task_generation_gate',
                   TaskGenerationGateTest)
