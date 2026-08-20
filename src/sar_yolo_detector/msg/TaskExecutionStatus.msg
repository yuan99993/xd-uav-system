# Adapter states are deliberately distinct from executor states. A decision
# layer must not interpret DISPATCHED_TO_EXECUTOR as an aircraft acceptance.
uint8 RECEIVED_BY_ADAPTER=0
uint8 REJECTED_BY_ADAPTER=1
uint8 DISPATCHED_TO_EXECUTOR=2
uint8 ACCEPTED_BY_EXECUTOR=3
uint8 QUEUED=4
uint8 EXECUTING=5
uint8 SUCCEEDED=6
uint8 FAILED=7
uint8 CANCELLED=8
uint8 PREEMPTED=9
uint8 EVIDENCE_STALE=10
uint8 EXECUTOR_TIMEOUT=11
uint8 RECOVERY_REQUIRED=12

uint16 REASON_NONE=0
uint16 INVALID_IDENTITY=1
uint16 STALE_ASSIGNMENT=2
uint16 DUPLICATE_OR_OUT_OF_ORDER=3
uint16 AUTHORIZATION_REQUIRED=4
uint16 UNKNOWN_OBSERVATION=5
uint16 OBSERVATION_EXPIRED=6
uint16 LOCALIZATION_REQUIRED=7
uint16 COORDINATE_CONTRACT_MISMATCH=8
uint16 INVALID_TARGET_POSE=9
uint16 EXECUTOR_UNAVAILABLE=10
uint16 UNKNOWN_ASSIGNMENT=11
uint16 INVALID_COMMAND=12
uint16 EXECUTOR_REJECTED=13
uint16 INVALID_PROTOCOL_VERSION=14
uint16 INVALID_AUTHENTICATION=15
uint16 EXECUTOR_NOT_READY=16
uint16 UNSUPPORTED_TASK=17
uint16 INVALID_CONSTRAINT=18
uint16 CAPACITY_EXCEEDED=19
uint16 GEOFENCE_UNKNOWN=20
uint16 PAYLOAD_UNAVAILABLE=21
uint16 INVALID_STATE_TRANSITION=22
uint16 EXECUTOR_ACK_TIMEOUT=23
uint16 RECOVERY_MISMATCH=24
uint16 EVIDENCE_BECAME_STALE=25
uint16 INTERNAL_ERROR=255

std_msgs/Header header
string assignment_uuid
uint64 assignment_sequence
uint64 status_sequence
uint8 state
uint16 reason_code
string reason

sar_yolo_detector/MissionIdentity aircraft
string decision_id
string decision_session_uuid
string observation_uuid
string executor_id
string executor_session_uuid
uint64 executor_status_sequence

float32 progress
bool current_pose_valid
geometry_msgs/PoseStamped current_pose
time updated_at
