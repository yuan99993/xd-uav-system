#!/usr/bin/env python3
import threading
import unittest

import rospy
import rostest

from pod_msgs.msg import TargetTrackArray
from tracker.msg import DetectionArray, DetectionCandidate, TrackingOutput


class TrackerAdapterTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.forwarded = None
        self.tracks = None
        self.forward_sub = rospy.Subscriber(
            '/adapter_test/tracker/detection_candidates', DetectionArray,
            self._forward_callback, queue_size=1)
        self.tracks_sub = rospy.Subscriber(
            '/adapter_test/pod/perception/tracks', TargetTrackArray,
            self._tracks_callback, queue_size=1)
        self.detection_pub = rospy.Publisher(
            '/adapter_test/pod/perception/detections', DetectionArray,
            queue_size=1)
        self.selected_pub = rospy.Publisher(
            '/adapter_test/tracker/tracking_output', TrackingOutput,
            queue_size=1)

    def _forward_callback(self, message):
        with self.lock:
            self.forwarded = message

    def _tracks_callback(self, message):
        with self.lock:
            self.tracks = message

    def _wait_for(self, predicate, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if predicate():
                    return True
            rate.sleep()
        return False

    @staticmethod
    def _candidate(track_id, class_id, confidence, bbox):
        candidate = DetectionCandidate()
        candidate.track_id = track_id
        candidate.class_id = class_id
        candidate.track_id_is_stable = True
        candidate.bbox = bbox
        candidate.has_bbox = True
        candidate.normalized_bbox = [0.5, 0.5, 0.2, 0.3]
        candidate.has_normalized_bbox = True
        candidate.confidence = confidence
        return candidate

    def test_detector_passthrough_and_all_tracks(self):
        self.assertTrue(self._wait_for(
            lambda: self.detection_pub.get_num_connections() > 0))

        detections = DetectionArray()
        detections.header.stamp = rospy.Time.now()
        detections.header.frame_id = 'eo_camera'
        detections.command = 'start_track'
        detections.candidates = [
            self._candidate(41, 7, 0.80, [10, 20, 110, 220]),
            self._candidate(42, 3, 0.70, [30, 40, 130, 240]),
        ]
        self.detection_pub.publish(detections)

        self.assertTrue(self._wait_for(
            lambda: self.forwarded is not None and self.tracks is not None))
        with self.lock:
            forwarded = self.forwarded
            tracks = self.tracks
        self.assertEqual(forwarded.command, 'start_track')
        self.assertEqual(len(forwarded.candidates), 2)
        self.assertEqual(len(tracks.tracks), 2)
        self.assertEqual(tracks.header.frame_id, 'eo_camera')
        self.assertEqual(tracks.tracks[0].image_source, 'eo_test')
        self.assertEqual(tracks.tracks[0].class_label, 'class_7')
        self.assertEqual(list(tracks.tracks[0].bbox_px), [10, 20, 110, 220])
        self.assertTrue(tracks.tracks[0].detected)
        self.assertFalse(tracks.tracks[0].selected)

        selected = TrackingOutput()
        selected.tracking_active = True
        selected.target_id = 42
        selected.tracking_quality = 0.91
        selected.association_method = 'id'
        selected.control_measurement_ready = True
        self.selected_pub.publish(selected)

        self.assertTrue(self._wait_for(
            lambda: self.tracks is not None and len(self.tracks.tracks) == 2 and
            self.tracks.tracks[1].selected))
        with self.lock:
            selected_track = self.tracks.tracks[1]
        self.assertTrue(selected_track.selected)
        self.assertTrue(selected_track.control_measurement_ready)
        self.assertEqual(selected_track.association_method, 'id')
        self.assertAlmostEqual(selected_track.tracking_quality, 0.91, places=5)


if __name__ == '__main__':
    rospy.init_node('tracker_adapter_test')
    rostest.rosrun('pod_tracker_adapter', 'tracker_adapter', TrackerAdapterTest)
