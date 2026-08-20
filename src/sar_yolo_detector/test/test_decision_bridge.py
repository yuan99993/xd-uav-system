#!/usr/bin/env python3

import copy
import threading
import unittest

import rospy
import rostest

from sar_yolo_detector.msg import (
    ExecutorHeartbeat,
    GeoReference,
    MissionIdentity,
    TaskAssignment,
    TaskExecutionStatus,
    UavDecisionHeartbeat,
)
from sar_yolo_detector.srv import (
    GetPerceptionSnapshot,
    GetTaskStatuses,
    SubmitTaskAssignment,
)
from sar_yolo_detector.msg import PerceptionIdentity, TaskCandidate, TaskCandidateArray


class DecisionBridgeTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self.executor_assignments = []
        self.outbound_statuses = []
        self.outbound_candidates = []
        self.bridge_heartbeats = []

        self.executor_subscriber = rospy.Subscriber(
            "/uav_01/mission_interface/executor_assignments",
            TaskAssignment,
            self._on_executor_assignment,
            queue_size=10,
        )
        self.status_subscriber = rospy.Subscriber(
            "/uav_01/mission_interface/task_status",
            TaskExecutionStatus,
            self._on_status,
            queue_size=20,
        )
        from sar_yolo_detector.msg import PerceptionCandidateArray

        self.candidate_subscriber = rospy.Subscriber(
            "/uav_01/mission_interface/perception_candidates",
            PerceptionCandidateArray,
            self._on_candidate,
            queue_size=10,
        )
        self.source_publisher = rospy.Publisher(
            "/uav_01/test/task_candidates",
            TaskCandidateArray,
            queue_size=1,
            latch=True,
        )
        self.executor_status_publisher = rospy.Publisher(
            "/uav_01/mission_interface/executor_status",
            TaskExecutionStatus,
            queue_size=10,
        )
        self.executor_heartbeat_publisher = rospy.Publisher(
            "/uav_01/mission_interface/executor_heartbeat",
            ExecutorHeartbeat,
            queue_size=1,
            latch=True,
        )
        self.bridge_heartbeat_subscriber = rospy.Subscriber(
            "/uav_01/mission_interface/heartbeat",
            UavDecisionHeartbeat,
            self._on_bridge_heartbeat,
            queue_size=2,
        )

        rospy.wait_for_service("/uav_01/mission_interface/submit_task", 10.0)
        rospy.wait_for_service(
            "/uav_01/mission_interface/get_perception_snapshot", 10.0
        )
        rospy.wait_for_service(
            "/uav_01/mission_interface/get_task_statuses", 10.0
        )
        self.submit = rospy.ServiceProxy(
            "/uav_01/mission_interface/submit_task", SubmitTaskAssignment
        )
        self.get_snapshot = rospy.ServiceProxy(
            "/uav_01/mission_interface/get_perception_snapshot",
            GetPerceptionSnapshot,
        )
        self.get_statuses = rospy.ServiceProxy(
            "/uav_01/mission_interface/get_task_statuses", GetTaskStatuses
        )
        self.assertTrue(
            self._wait_for(
                lambda: self.source_publisher.get_num_connections() > 0
                and self.executor_status_publisher.get_num_connections() > 0
                and self.executor_heartbeat_publisher.get_num_connections() > 0,
                10.0,
            )
        )
        self._executor_heartbeat_sequence = 0
        self._executor_timer = rospy.Timer(
            rospy.Duration(0.2), self._publish_executor_heartbeat
        )
        self._publish_executor_heartbeat(None)
        self.assertTrue(
            self._wait_for(
                lambda: self.bridge_heartbeats
                and self.bridge_heartbeats[-1].ready_for_assignments,
                5.0,
            )
        )

    def _on_executor_assignment(self, message):
        with self._lock:
            self.executor_assignments.append(message)

    def _on_status(self, message):
        with self._lock:
            self.outbound_statuses.append(message)

    def _on_candidate(self, message):
        with self._lock:
            self.outbound_candidates.append(message)

    def _on_bridge_heartbeat(self, message):
        with self._lock:
            self.bridge_heartbeats.append(message)

    def _publish_executor_heartbeat(self, _event):
        self._executor_heartbeat_sequence += 1
        message = ExecutorHeartbeat()
        message.header.stamp = rospy.Time.now()
        message.aircraft = MissionIdentity(
            mission_id="mission_test",
            uav_id="uav_01",
            session_uuid="session_test",
        )
        message.executor_id = "fake_mrs_executor"
        message.executor_session_uuid = "executor_session_test"
        message.heartbeat_sequence = self._executor_heartbeat_sequence
        message.ready = True
        message.flight_control_ok = True
        message.localization_ok = True
        message.battery_percent = 80.0
        message.capabilities.executor_id = message.executor_id
        message.capabilities.executor_session_uuid = message.executor_session_uuid
        message.capabilities.capability_version = 1
        message.capabilities.protocol_major = TaskAssignment.PROTOCOL_MAJOR
        message.capabilities.protocol_minor = TaskAssignment.PROTOCOL_MINOR
        message.capabilities.supported_task_types = [TaskAssignment.RESCUE]
        message.capabilities.supported_executor_profiles = ["mrs-rescue"]
        message.capabilities.known_geofence_ids = ["geofence-test"]
        message.capabilities.geofence_version = "geofence-test-v1"
        message.capabilities.available_payloads = ["rescue-kit"]
        message.capabilities.maximum_active_tasks = 1
        message.capabilities.supports_area_tasks = True
        message.capabilities.maximum_speed_mps = 8.0
        message.capabilities.minimum_arrival_tolerance_m = 0.5
        message.capabilities.maximum_arrival_tolerance_m = 20.0
        message.capabilities.minimum_survey_altitude_m = 5.0
        message.capabilities.maximum_survey_altitude_m = 120.0
        self.executor_heartbeat_publisher.publish(message)

    @staticmethod
    def _wait_for(predicate, timeout):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return True
            rate.sleep()
        return predicate()

    @staticmethod
    def _provenance():
        result = PerceptionIdentity()
        result.mission_id = "mission_test"
        result.uav_id = "uav_01"
        result.session_uuid = "session_test"
        result.sensor_id = "eo_01"
        result.profile = "thermal_uav"
        result.model_version = "test-v1"
        result.model_sha256 = "abc123"
        result.calibration_version = "cal-test"
        result.coordinate_contract = "LOCAL_ENU:map"
        return result

    def _publish_candidate(self):
        now = rospy.Time.now()
        candidate = TaskCandidate()
        candidate.header.stamp = now
        candidate.header.frame_id = "map"
        candidate.track_id = 7
        candidate.observation_uuid = "observation-7"
        candidate.event_sequence = 1
        candidate.provenance = self._provenance()
        candidate.class_id = 0
        candidate.class_name = "person"
        candidate.semantic_type = TaskCandidate.VICTIM
        candidate.status = TaskCandidate.CONFIRMED
        candidate.confidence = 0.91
        candidate.stability = 0.88
        candidate.priority = 0.95
        candidate.observation_count = 3
        candidate.first_seen = now - rospy.Duration(0.2)
        candidate.last_seen = now
        candidate.localization_valid = True
        candidate.task_pose.header = candidate.header
        candidate.task_pose.pose.position.x = 10.0
        candidate.task_pose.pose.position.y = -2.0
        candidate.task_pose.pose.position.z = 0.0
        candidate.task_pose.pose.orientation.w = 1.0
        candidate.bearing.header.stamp = now
        candidate.bearing.header.frame_id = "camera"
        candidate.bearing.vector.z = 1.0
        candidate.sensor_id = "eo_01"
        candidate.image_detection.bbox.center.x = 100.0
        candidate.image_detection.bbox.center.y = 120.0
        candidate.image_detection.bbox.size_x = 20.0
        candidate.image_detection.bbox.size_y = 40.0

        array = TaskCandidateArray()
        array.header = candidate.header
        array.array_sequence = 1
        array.full_snapshot = True
        array.provenance = candidate.provenance
        array.candidates = [candidate]
        self.source_publisher.publish(array)

    @staticmethod
    def _assignment():
        now = rospy.Time.now()
        assignment = TaskAssignment()
        assignment.protocol_major = TaskAssignment.PROTOCOL_MAJOR
        assignment.protocol_minor = TaskAssignment.PROTOCOL_MINOR
        assignment.header.stamp = now
        assignment.assignment_uuid = "assignment-1"
        assignment.assignment_sequence = 1
        assignment.command = TaskAssignment.ASSIGN
        assignment.task_type = TaskAssignment.RESCUE
        assignment.decision_id = "decision_test"
        assignment.decision_session_uuid = "decision_session_test"
        assignment.destination = MissionIdentity(
            mission_id="mission_test",
            uav_id="uav_01",
            session_uuid="session_test",
        )
        assignment.observation_uuid = "observation-7"
        assignment.semantic_type = TaskCandidate.VICTIM
        assignment.priority = 0.95
        assignment.target_pose_valid = False
        assignment.geo_reference = GeoReference(
            coordinate_type=GeoReference.LOCAL_ENU,
            frame_id="map",
            map_uuid="mission-test-map",
            origin_latitude_deg=31.2304,
            origin_longitude_deg=121.4737,
            origin_altitude_m=5.0,
            vertical_datum="WGS84_ELLIPSOID",
            transform_version="geo-test-v1",
        )
        assignment.arrival_tolerance_m = 3.0
        assignment.maximum_speed_mps = 4.0
        assignment.maximum_task_duration = rospy.Duration(120.0)
        assignment.valid_until = now + rospy.Duration(30.0)
        assignment.geofence_id = "geofence-test"
        assignment.geofence_version = "geofence-test-v1"
        assignment.operator_authorized = True
        assignment.policy_version = "policy-test"
        assignment.executor_profile = "mrs-rescue"
        assignment.required_payload = "rescue-kit"
        return assignment

    def test_bridge_contract_and_idempotency(self):
        self._publish_candidate()
        self.assertTrue(
            self._wait_for(lambda: len(self.outbound_candidates) >= 1, 5.0)
        )
        outbound = self.outbound_candidates[-1].candidates[0]
        self.assertEqual(outbound.observation_uuid, "observation-7")
        self.assertTrue(outbound.localization_valid)
        self.assertEqual(outbound.source.session_uuid, "session_test")

        snapshot = self.get_snapshot()
        self.assertTrue(snapshot.available)
        self.assertEqual(len(snapshot.snapshot.candidates), 1)

        assignment = self._assignment()
        response = self.submit(assignment)
        self.assertTrue(response.accepted)
        self.assertEqual(
            response.acknowledgment.state,
            TaskExecutionStatus.DISPATCHED_TO_EXECUTOR,
        )
        first_status_sequence = response.acknowledgment.status_sequence
        self.assertTrue(
            self._wait_for(lambda: len(self.executor_assignments) == 1, 5.0)
        )
        forwarded = self.executor_assignments[0]
        self.assertTrue(forwarded.target_pose_valid)
        self.assertEqual(forwarded.coordinate_contract, "LOCAL_ENU:map")
        self.assertEqual(forwarded.target_pose.header.frame_id, "map")

        # An identical network retry returns the original ACK and is not
        # dispatched to the aircraft a second time.
        replay = self.submit(copy.deepcopy(assignment))
        self.assertTrue(replay.accepted)
        self.assertEqual(replay.acknowledgment.status_sequence, first_status_sequence)
        rospy.sleep(0.2)
        self.assertEqual(len(self.executor_assignments), 1)

        wrong_uav = copy.deepcopy(assignment)
        wrong_uav.assignment_uuid = "assignment-wrong-uav"
        wrong_uav.assignment_sequence = 2
        wrong_uav.destination.uav_id = "uav_99"
        rejected = self.submit(wrong_uav)
        self.assertFalse(rejected.accepted)
        self.assertEqual(
            rejected.acknowledgment.reason_code,
            TaskExecutionStatus.INVALID_IDENTITY,
        )

        executor_status = TaskExecutionStatus()
        executor_status.header.stamp = rospy.Time.now()
        executor_status.assignment_uuid = "assignment-1"
        executor_status.assignment_sequence = 1
        executor_status.state = TaskExecutionStatus.ACCEPTED_BY_EXECUTOR
        executor_status.aircraft = MissionIdentity(
            mission_id="mission_test",
            uav_id="uav_01",
            session_uuid="session_test",
        )
        executor_status.executor_id = "fake_mrs_executor"
        executor_status.executor_session_uuid = "executor_session_test"
        executor_status.executor_status_sequence = 1
        executor_status.progress = 0.1
        self.executor_status_publisher.publish(executor_status)
        self.assertTrue(
            self._wait_for(
                lambda: any(
                    status.state == TaskExecutionStatus.ACCEPTED_BY_EXECUTOR
                    for status in self.outbound_statuses
                ),
                5.0,
            )
        )

        history = self.get_statuses(
            assignment_uuid="assignment-1", since_status_sequence=0, limit=100
        )
        self.assertTrue(
            any(
                status.state == TaskExecutionStatus.ACCEPTED_BY_EXECUTOR
                for status in history.statuses
            )
        )


if __name__ == "__main__":
    rospy.init_node("test_decision_bridge")
    rostest.rosrun(
        "sar_yolo_detector", "test_decision_bridge", DecisionBridgeTest
    )
