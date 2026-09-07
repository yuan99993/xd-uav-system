# Area equivalent of TaskAssignment. The common command fields intentionally
# remain explicit so the signed wire contract is self-contained.
uint8 ASSIGN=0
uint8 UPDATE=1
uint8 CANCEL=2

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
string observation_uuid
# Set by the multi-UAV decision layer after merging overlapping world polygons.
# source_observation_uuids preserves every local item of supporting evidence.
string global_region_uuid
string[] source_observation_uuids
uint8 semantic_type
float32 priority

geometry_msgs/PolygonStamped target_area
sar_yolo_detector/GeoReference geo_reference
bool preferred_entry_pose_valid
geometry_msgs/PoseStamped preferred_entry_pose

float32 survey_altitude_m
float32 target_gsd_m_per_px
float32 lane_spacing_m
float32 forward_overlap
float32 side_overlap
float32 maximum_speed_mps
duration maximum_task_duration
time valid_until
string geofence_id
string geofence_version
bool require_line_of_sight

bool operator_authorized
string policy_version
string executor_profile
string required_payload
bool allow_preemption

string authorization_key_id
string authorization_nonce
time authorization_issued_at
string authorization_signature
