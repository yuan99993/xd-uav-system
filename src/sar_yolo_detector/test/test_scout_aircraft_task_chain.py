#!/usr/bin/env python3
"""Verify tracked COCO airplanes, and only airplanes, become scout tasks."""

import threading
import time
import unittest

import rospy
import rostest
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

from sar_yolo_detector.msg import (
    PerceptionIdentity,
    TaskCandidate,
    TaskCandidateArray,
    TrackedDetection2D,
    TrackedDetection2DArray,
)


class ScoutAircraftTaskChainTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._candidate_messages = []
        self._publisher = rospy.Publisher(
            "/scout_aircraft_chain/tracked_detections",
            TrackedDetection2DArray,
            queue_size=3,
        )
        rospy.Subscriber(
            "/scout_aircraft_chain/task_candidates",
            TaskCandidateArray,
            self._candidate_callback,
            queue_size=10,
        )

    def _candidate_callback(self, message):
        with self._lock:
            self._candidate_messages.append(message)

    def _wait(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.rostime.wallsleep(0.02)
        return False

    @staticmethod
    def _identity():
        identity = PerceptionIdentity()
        identity.mission_id = "mission_aircraft_test"
        identity.uav_id = "scout_01"
        identity.session_uuid = "scout_aircraft_test_session"
        identity.sensor_id = "scout_eo_primary"
        identity.profile = "aircraft_coco"
        identity.model_version = "yolo11n_coco80_baseline"
        identity.model_sha256 = (
            "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1"
        )
        identity.calibration_version = "test_calibration"
        identity.coordinate_contract = "CAMERA_RAY"
        return identity

    @staticmethod
    def _tracked(class_id, score, track_id, stamp, center_x):
        detection = Detection2D()
        detection.header.stamp = stamp
        detection.header.frame_id = "scout_eo_optical"
        detection.bbox.center.x = center_x
        detection.bbox.center.y = 220.0
        detection.bbox.size_x = 90.0
        detection.bbox.size_y = 50.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = class_id
        hypothesis.score = score
        detection.results.append(hypothesis)

        tracked = TrackedDetection2D()
        tracked.detection = detection
        tracked.track_id = track_id
        tracked.track_id_is_stable = True
        return tracked

    def _message(self, stamp):
        message = TrackedDetection2DArray()
        message.header.stamp = stamp
        message.header.frame_id = "scout_eo_optical"
        message.provenance = self._identity()
        # COCO class 2 must be ignored by the aircraft task profile.
        message.detections.append(self._tracked(2, 0.99, 202, stamp, 180.0))
        message.detections.append(self._tracked(4, 0.91, 404, stamp, 420.0))
        return message

    def test_only_stable_airplane_track_becomes_candidate(self):
        self.assertTrue(self._wait(lambda: self._publisher.get_num_connections() > 0))

        for _ in range(3):
            self._publisher.publish(self._message(rospy.Time.now()))
            rospy.rostime.wallsleep(0.08)

        self.assertTrue(
            self._wait(
                lambda: any(
                    any(candidate.status == TaskCandidate.CONFIRMED
                        for candidate in message.candidates)
                    for message in self._candidate_messages
                )
            )
        )
        with self._lock:
            candidates = [
                candidate
                for message in self._candidate_messages
                for candidate in message.candidates
            ]

        self.assertTrue(candidates)
        self.assertTrue(all(candidate.class_id == 4 for candidate in candidates))
        confirmed = next(
            candidate
            for candidate in candidates
            if candidate.status == TaskCandidate.CONFIRMED
        )
        self.assertEqual(confirmed.class_name, "airplane")
        self.assertEqual(confirmed.semantic_type, TaskCandidate.VEHICLE)
        self.assertTrue(confirmed.source_track_id_valid)
        self.assertEqual(confirmed.source_track_id, 404)
        self.assertFalse(confirmed.localization_valid)
        self.assertEqual(confirmed.bearing.header.frame_id, "scout_eo_optical")
        self.assertEqual(confirmed.provenance.uav_id, "scout_01")


if __name__ == "__main__":
    rospy.init_node("scout_aircraft_task_chain_test")
    rostest.rosrun(
        "sar_yolo_detector",
        "scout_aircraft_task_chain",
        ScoutAircraftTaskChainTest,
    )
