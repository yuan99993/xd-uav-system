#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from sar_yolo_detector.msg import TaskCandidate, TaskCandidateArray


class TaskPointGeneratorTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._candidate_messages = []
        self._poses = []
        self._detections_pub = rospy.Publisher(
            '/sar_task_point_test/detections', Detection2DArray, queue_size=5)
        self._camera_info_pub = rospy.Publisher(
            '/sar_task_point_test/camera_info', CameraInfo, queue_size=2,
            latch=True)
        self._candidate_sub = rospy.Subscriber(
            '/sar_task_point_test/candidates', TaskCandidateArray,
            self._candidate_callback, queue_size=10)
        self._pose_sub = rospy.Subscriber(
            '/sar_task_point_test/task_point', PoseStamped,
            self._pose_callback, queue_size=10)
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def _candidate_callback(self, message):
        with self._lock:
            self._candidate_messages.append(message)

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

    @staticmethod
    def _camera_info(stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera_optical'
        info.width = 640
        info.height = 480
        info.K = [400.0, 0.0, 320.0,
                  0.0, 400.0, 240.0,
                  0.0, 0.0, 1.0]
        info.P = [400.0, 0.0, 320.0, 0.0,
                  0.0, 400.0, 240.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    @staticmethod
    def _detection_message(stamp):
        message = Detection2DArray()
        message.header.stamp = stamp
        message.header.frame_id = 'camera_optical'
        detection = Detection2D()
        detection.header = message.header
        detection.bbox.center.x = 320.0
        detection.bbox.center.y = 240.0
        detection.bbox.size_x = 40.0
        detection.bbox.size_y = 80.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = 0
        hypothesis.score = 0.92
        detection.results.append(hypothesis)
        message.detections.append(detection)
        return message

    def _publish_static_camera_transform(self):
        # Optical +Z points straight down from a camera at map Z=20 m.
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'camera_optical'
        transform.transform.translation.z = 20.0
        transform.transform.rotation.x = 1.0
        transform.transform.rotation.w = 0.0
        self._tf_broadcaster.sendTransform(transform)

    def test_confirmed_detection_becomes_localized_task_candidate_then_expires(self):
        self.assertTrue(self._wait(
            lambda: self._detections_pub.get_num_connections() > 0 and
            self._camera_info_pub.get_num_connections() > 0 and
            self._candidate_sub.get_num_connections() > 0))
        self._publish_static_camera_transform()
        stamp = rospy.Time.now()
        self._camera_info_pub.publish(self._camera_info(stamp))
        rospy.rostime.wallsleep(0.1)

        self._detections_pub.publish(self._detection_message(stamp))
        self.assertTrue(self._wait(
            lambda: any(m.candidates for m in self._candidate_messages)))

        second_stamp = rospy.Time.now()
        self._camera_info_pub.publish(self._camera_info(second_stamp))
        self._detections_pub.publish(self._detection_message(second_stamp))
        self.assertTrue(self._wait(
            lambda: any(any(c.status == TaskCandidate.CONFIRMED
                            for c in m.candidates)
                        for m in self._candidate_messages)))
        self.assertTrue(self._wait(lambda: len(self._poses) >= 1))

        with self._lock:
            confirmed = next(c for message in self._candidate_messages
                             for c in message.candidates
                             if c.status == TaskCandidate.CONFIRMED)
            pose = self._poses[-1]
        self.assertEqual(confirmed.class_name, 'person')
        self.assertTrue(confirmed.localization_valid)
        self.assertEqual(confirmed.task_pose.header.frame_id, 'map')
        self.assertAlmostEqual(confirmed.task_pose.pose.position.x, 0.0,
                               places=3)
        self.assertAlmostEqual(confirmed.task_pose.pose.position.y, 0.0,
                               places=3)
        self.assertAlmostEqual(confirmed.task_pose.pose.position.z, 0.0,
                               places=3)
        self.assertGreater(confirmed.priority, 0.55)
        self.assertEqual(pose.header.frame_id, 'map')
        self.assertEqual(pose.header.stamp, confirmed.task_pose.header.stamp)

        self.assertTrue(self._wait(
            lambda: any(any(c.status == TaskCandidate.EXPIRED
                            for c in m.candidates)
                        for m in self._candidate_messages), timeout=2.0))


if __name__ == '__main__':
    rospy.init_node('sar_task_point_generator_test')
    rostest.rosrun('sar_yolo_detector', 'task_point_generator',
                   TaskPointGeneratorTest)
