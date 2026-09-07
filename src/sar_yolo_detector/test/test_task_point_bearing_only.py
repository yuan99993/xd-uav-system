#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from sar_yolo_detector.msg import TaskCandidateArray


class TaskPointBearingOnlyTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._candidates = []
        self._poses = []
        self._detections_pub = rospy.Publisher(
            '/sar_task_bearing_test/detections', Detection2DArray, queue_size=2)
        self._camera_info_pub = rospy.Publisher(
            '/sar_task_bearing_test/camera_info', CameraInfo, queue_size=1,
            latch=True)
        self._candidate_sub = rospy.Subscriber(
            '/sar_task_bearing_test/candidates', TaskCandidateArray,
            self._candidate_callback, queue_size=2)
        self._pose_sub = rospy.Subscriber(
            '/sar_task_bearing_test/task_point', PoseStamped,
            self._pose_callback, queue_size=2)

    def _candidate_callback(self, message):
        with self._lock:
            self._candidates.extend(message.candidates)

    def _pose_callback(self, message):
        with self._lock:
            self._poses.append(message)

    def _wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.rostime.wallsleep(0.01)
        return False

    def test_bearing_only_never_emits_navigation_pose(self):
        self.assertTrue(self._wait(
            lambda: self._detections_pub.get_num_connections() > 0 and
            self._camera_info_pub.get_num_connections() > 0 and
            self._candidate_sub.get_num_connections() > 0))
        stamp = rospy.Time.now()
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'thermal_optical'
        info.width = 640
        info.height = 480
        info.K = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
        info.P = [400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self._camera_info_pub.publish(info)
        rospy.rostime.wallsleep(0.1)

        detections = Detection2DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = 'thermal_optical'
        detection = Detection2D()
        detection.bbox.center.x = 320.0
        detection.bbox.center.y = 240.0
        detection.bbox.size_x = 20.0
        detection.bbox.size_y = 40.0
        result = ObjectHypothesisWithPose()
        result.id = 0
        result.score = 0.95
        detection.results.append(result)
        detections.detections.append(detection)
        self._detections_pub.publish(detections)

        self.assertTrue(self._wait(lambda: len(self._candidates) >= 1))
        rospy.rostime.wallsleep(0.3)
        with self._lock:
            candidate = self._candidates[-1]
            pose_count = len(self._poses)
        self.assertFalse(candidate.localization_valid)
        self.assertEqual(candidate.bearing.header.frame_id, 'thermal_optical')
        self.assertGreater(candidate.bearing.vector.z, 0.9)
        self.assertEqual(pose_count, 0)


if __name__ == '__main__':
    rospy.init_node('sar_task_point_bearing_only_test')
    rostest.rosrun('sar_yolo_detector', 'task_point_bearing_only',
                   TaskPointBearingOnlyTest)
