#!/usr/bin/env python3
import threading
import unittest

import rospy
import rostest

from pod_msgs.msg import SelectedTarget, TargetTrackArray
from pod_msgs.srv import SelectTarget, SelectTargetRequest
from tracker.msg import DetectionArray, DetectionCandidate


class TrackerProductIntegrationTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._tracks = None
        self._selected = None
        self._detection_publisher = rospy.Publisher(
            '/product_test/pod/perception/detections', DetectionArray,
            queue_size=1)
        self._tracks_subscriber = rospy.Subscriber(
            '/product_test/pod/perception/tracks', TargetTrackArray,
            self._tracks_callback, queue_size=1)
        self._selected_subscriber = rospy.Subscriber(
            '/product_test/pod/target/selected', SelectedTarget,
            self._selected_callback, queue_size=1)

    def _tracks_callback(self, message):
        with self._lock:
            self._tracks = message

    def _selected_callback(self, message):
        with self._lock:
            self._selected = message

    def _wait(self, predicate, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self._lock:
                if predicate():
                    return True
            rate.sleep()
        return False

    @staticmethod
    def _candidate(track_id, center_x):
        result = DetectionCandidate()
        result.track_id = track_id
        result.track_id_is_stable = True
        result.class_id = 4
        result.confidence = 0.9
        result.has_normalized_bbox = True
        result.normalized_bbox = [center_x, 0.5, 0.2, 0.2]
        return result

    def _publish_frame(self):
        frame = DetectionArray()
        frame.header.stamp = rospy.Time.now()
        frame.header.frame_id = 'eo_optical_frame'
        frame.candidates = [self._candidate(21, 0.3),
                            self._candidate(22, 0.7)]
        self._detection_publisher.publish(frame)

    def test_persistent_tracks_and_product_selection(self):
        self.assertTrue(self._wait(
            lambda: self._detection_publisher.get_num_connections() > 0))
        self._publish_frame()
        rospy.sleep(0.05)
        self._publish_frame()
        self.assertTrue(self._wait(
            lambda: self._tracks is not None and len(self._tracks.tracks) == 2 and
                    all(track.age_frames >= 2 for track in self._tracks.tracks)))

        rospy.wait_for_service('/product_test/pod/mission/select_target', 5.0)
        select = rospy.ServiceProxy(
            '/product_test/pod/mission/select_target', SelectTarget)
        request = SelectTargetRequest()
        request.target_id = 22
        request.start_tracking = True
        response = select(request)
        self.assertTrue(response.success, response.message)
        self.assertEqual(22, response.selected_target.target_id)

        self._publish_frame()
        self.assertTrue(self._wait(lambda: any(
            track.target_id == 22 and track.selected
            for track in self._tracks.tracks)))
        self.assertTrue(self._wait(
            lambda: self._selected is not None and
                    self._selected.target_id == 22 and
                    self._selected.selected))


if __name__ == '__main__':
    rospy.init_node('tracker_product_integration_test')
    rostest.rosrun('pod_tracker_adapter', 'tracker_product_integration',
                   TrackerProductIntegrationTest)
