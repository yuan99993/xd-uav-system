#!/usr/bin/env python3
"""Explicit, signed point-task submission example for decision developers."""

import argparse
import hashlib
import hmac
import io
import os
import uuid

import rospy

from sar_yolo_detector.msg import TaskAssignment, UavDecisionHeartbeat
from sar_yolo_detector.srv import SubmitTaskAssignment


def signed(message, key):
    message.authorization_signature = ""
    serialized = io.BytesIO()
    message.serialize(serialized)
    message.authorization_signature = hmac.new(
        key, serialized.getvalue(), hashlib.sha256
    ).hexdigest()
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uav-namespace", required=True)
    parser.add_argument("--decision-id", required=True)
    parser.add_argument("--decision-session", default=str(uuid.uuid4()))
    parser.add_argument("--key-id", required=True)
    parser.add_argument("--key-file", required=True)
    parser.add_argument("--observation-uuid", required=True)
    parser.add_argument("--semantic-type", type=int, required=True)
    parser.add_argument("--task-type", type=int, required=True)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--sequence", type=int, required=True)
    parser.add_argument("--geofence-id", required=True)
    parser.add_argument("--geofence-version", required=True)
    parser.add_argument("--executor-profile", required=True)
    parser.add_argument("--required-payload", default="")
    parser.add_argument("--maximum-speed", type=float, default=4.0)
    parser.add_argument("--arrival-tolerance", type=float, default=3.0)
    parser.add_argument("--valid-for-sec", type=float, default=10.0)
    parser.add_argument("--operator-authorized", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    if not args.operator_authorized:
        parser.error("--operator-authorized is required for this explicit send tool")
    if os.stat(args.key_file).st_mode & 0o077:
        parser.error("key file must not be accessible by group/other users")
    with open(args.key_file, "rb") as stream:
        key = stream.read().strip()
    if len(key) < 32:
        parser.error("key must contain at least 32 bytes")

    rospy.init_node("sar_signed_task_submitter", anonymous=True)
    namespace = "/" + args.uav_namespace.strip("/") + "/mission_interface"
    heartbeat = rospy.wait_for_message(
        namespace + "/heartbeat", UavDecisionHeartbeat, timeout=5.0
    )
    if not heartbeat.ready_for_assignments:
        raise RuntimeError("aircraft is not ready: " + heartbeat.not_ready_reason)

    now = rospy.Time.now()
    assignment = TaskAssignment()
    assignment.protocol_major = heartbeat.protocol_major
    assignment.protocol_minor = heartbeat.protocol_minor
    assignment.header.stamp = now
    assignment.assignment_uuid = str(uuid.uuid4())
    assignment.assignment_sequence = args.sequence
    assignment.command = TaskAssignment.ASSIGN
    assignment.task_type = args.task_type
    assignment.decision_id = args.decision_id
    assignment.decision_session_uuid = args.decision_session
    assignment.destination = heartbeat.aircraft
    assignment.observation_uuid = args.observation_uuid
    assignment.semantic_type = args.semantic_type
    assignment.priority = 1.0
    assignment.target_pose_valid = True
    assignment.target_pose.header.stamp = now
    assignment.target_pose.header.frame_id = heartbeat.geo_reference.frame_id
    assignment.target_pose.pose.position.x = args.x
    assignment.target_pose.pose.position.y = args.y
    assignment.target_pose.pose.position.z = args.z
    assignment.target_pose.pose.orientation.w = 1.0
    assignment.coordinate_contract = heartbeat.coordinate_contract
    assignment.geo_reference = heartbeat.geo_reference
    assignment.arrival_tolerance_m = args.arrival_tolerance
    assignment.maximum_speed_mps = args.maximum_speed
    assignment.maximum_task_duration = rospy.Duration(600.0)
    assignment.valid_until = now + rospy.Duration(args.valid_for_sec)
    assignment.geofence_id = args.geofence_id
    assignment.geofence_version = args.geofence_version
    assignment.operator_authorized = True
    assignment.policy_version = "explicit-cli-v1"
    assignment.executor_profile = args.executor_profile
    assignment.required_payload = args.required_payload
    assignment.authorization_key_id = args.key_id
    assignment.authorization_nonce = str(uuid.uuid4())
    assignment.authorization_issued_at = now
    signed(assignment, key)

    rospy.wait_for_service(namespace + "/submit_task", 5.0)
    response = rospy.ServiceProxy(
        namespace + "/submit_task", SubmitTaskAssignment
    )(assignment)
    print(
        "accepted={} state={} reason_code={} reason={}".format(
            response.accepted,
            response.acknowledgment.state,
            response.acknowledgment.reason_code,
            response.acknowledgment.reason,
        )
    )


if __name__ == "__main__":
    main()
