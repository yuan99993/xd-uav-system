#!/usr/bin/env python3
"""Verify that a scout bridge is perception-up only."""

import threading
import time
import unittest

import rosservice
import rospy
import rostest
from sar_yolo_detector.msg import (
    PerceptionCandidate,
    PerceptionCandidateArray,
    UavDecisionHeartbeat,
)
from sar_yolo_detector.msg import PerceptionIdentity, TaskCandidate, TaskCandidateArray


class DecisionBridgeScoutTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._candidates = []
        self._heartbeat = None
        self._publisher = rospy.Publisher(
            "/scout_01/test/task_candidates", TaskCandidateArray, queue_size=2
        )
        rospy.Subscriber(
            "/scout_01/mission_interface/perception_candidates",
            PerceptionCandidateArray,
            self._candidate_callback,
            queue_size=2,
        )
        rospy.Subscriber(
            "/scout_01/mission_interface/heartbeat",
            UavDecisionHeartbeat,
            self._heartbeat_callback,
            queue_size=1,
        )

    def _candidate_callback(self, message):
        with self._lock:
            self._candidates.append(message)

    def _heartbeat_callback(self, message):
        with self._lock:
            self._heartbeat = message

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
        identity.mission_id = "mission_test"
        identity.uav_id = "scout_01"
        identity.session_uuid = "scout_session_test"
        identity.sensor_id = "scout_eo_primary"
        identity.profile = "aircraft_coco"
        identity.model_version = "yolo11n_coco80_baseline"
        identity.model_sha256 = (
            "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1"
        )
        identity.calibration_version = "test_calibration"
        identity.coordinate_contract = "LOCAL_ENU:map"
        return identity

    def test_airplane_candidate_uplink_without_assignment_downlink(self):
        self.assertTrue(self._wait(lambda: self._publisher.get_num_connections() > 0))
        self.assertTrue(self._wait(lambda: self._heartbeat is not None))
        with self._lock:
            heartbeat = self._heartbeat
        self.assertEqual(heartbeat.bridge_state, UavDecisionHeartbeat.READY)
        self.assertFalse(heartbeat.ready_for_assignments)
        self.assertEqual(heartbeat.not_ready_reason, "scout_uplink_only")

        services = rosservice.get_service_list()
        self.assertNotIn("/scout_01/mission_interface/submit_task", services)
        self.assertNotIn("/scout_01/mission_interface/submit_area_task", services)

        now = rospy.Time.now()
        message = TaskCandidateArray()
        message.header.stamp = now
        message.header.frame_id = "map"
        message.array_sequence = 1
        message.full_snapshot = False
        message.provenance = self._identity()
        candidate = TaskCandidate()
        candidate.header = message.header
        candidate.track_id = 17
        candidate.observation_uuid = "airplane-observation-17"
        candidate.event_sequence = 1
        candidate.provenance = self._identity()
        candidate.class_id = 4
        candidate.class_name = "airplane"
        candidate.semantic_type = TaskCandidate.VEHICLE
        candidate.status = TaskCandidate.CONFIRMED
        candidate.source_track_id_valid = True
        candidate.source_track_id = 71
        candidate.confidence = 0.91
        candidate.stability = 0.88
        candidate.priority = 0.89
        candidate.observation_count = 4
        candidate.consecutive_hits = 4
        candidate.first_seen = now
        candidate.last_seen = now
        candidate.localization_valid = True
        candidate.task_pose.header = message.header
        candidate.task_pose.pose.position.x = 20.0
        candidate.task_pose.pose.position.y = 10.0
        candidate.task_pose.pose.position.z = 50.0
        candidate.task_pose.pose.orientation.w = 1.0
        candidate.image_detection.bbox.center.x = 320.0
        candidate.image_detection.bbox.center.y = 240.0
        candidate.image_detection.bbox.size_x = 80.0
        candidate.image_detection.bbox.size_y = 45.0
        candidate.sensor_id = "scout_eo_primary"
        message.candidates.append(candidate)
        self._publisher.publish(message)

        self.assertTrue(self._wait(lambda: len(self._candidates) > 0))
        with self._lock:
            uplink = self._candidates[-1]
        self.assertEqual(uplink.source.uav_id, "scout_01")
        self.assertEqual(len(uplink.candidates), 1)
        result = uplink.candidates[0]
        self.assertEqual(result.raw_class_id, 4)
        self.assertEqual(result.raw_class_name, "airplane")
        self.assertEqual(result.lifecycle_state, PerceptionCandidate.CONFIRMED)
        self.assertTrue(result.localization_valid)
        self.assertEqual(result.target_pose.header.frame_id, "map")


if __name__ == "__main__":
    rospy.init_node("decision_bridge_scout_test")
    rostest.rosrun(
        "sar_yolo_detector",
        "decision_bridge_scout",
        DecisionBridgeScoutTest,
    )
