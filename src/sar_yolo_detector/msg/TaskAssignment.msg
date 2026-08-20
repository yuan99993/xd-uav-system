# Lifecycle command for one stable assignment_uuid. ASSIGN creates a task,
# UPDATE replaces its parameters, and CANCEL stops it. assignment_sequence is
# monotonic within (mission_id, decision_id, decision_session_uuid).
uint8 ASSIGN=0
uint8 UPDATE=1
uint8 CANCEL=2

# Mission task ontology. Extend by adding constants without changing meanings.
uint8 TASK_UNKNOWN=0
uint8 INSPECT=1
uint8 SEARCH=2
uint8 RESCUE=3
uint8 DELIVER_SUPPLIES=4
uint8 MONITOR=5

uint16 PROTOCOL_MAJOR=1
uint16 PROTOCOL_MINOR=0

uint16 protocol_major
uint16 protocol_minor
std_msgs/Header header
string assignment_uuid
uint64 assignment_sequence
uint8 command
uint8 task_type

string decision_id
string decision_session_uuid
sar_yolo_detector/MissionIdentity destination

# Links the task to perception evidence. It may be empty for an operator-made
# waypoint when the bridge is configured to permit such assignments.
string observation_uuid
uint8 semantic_type
float32 priority

bool target_pose_valid
geometry_msgs/PoseStamped target_pose
string coordinate_contract
sar_yolo_detector/GeoReference geo_reference

float32 arrival_tolerance_m
float32 maximum_speed_mps
duration maximum_task_duration
time valid_until
string geofence_id
string geofence_version
bool require_line_of_sight

# This is an authorization assertion from the decision layer, not a substitute
# for the executor's geofence, collision, energy, or flight-state checks.
bool operator_authorized
string policy_version
string executor_profile
string required_payload
bool allow_preemption

# HMAC-SHA256 over the ROS1 serialized TaskAssignment after setting this
# signature field to an empty string. The key is selected by authorization_key_id.
string authorization_key_id
string authorization_nonce
time authorization_issued_at
string authorization_signature
