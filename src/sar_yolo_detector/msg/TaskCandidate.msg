# Mission-decision candidate produced from temporally confirmed visual evidence.
# This message is advisory only: a task decision layer must still enforce
# geofences, vehicle feasibility, airspace rules, and operator authorization.

uint8 PENDING=0
uint8 CONFIRMED=1
uint8 UPDATED=2
uint8 EXPIRED=3

# Mission-wide semantic ontology. raw class_id/class_name remain available for
# model-specific audit while semantic_type is stable across profiles.
uint8 SEMANTIC_UNKNOWN=0
uint8 VICTIM=1
uint8 FIRE=2
uint8 SMOKE=3
uint8 FLOODED_BUILDING=4
uint8 FLOODED_ROAD=5
uint8 FLOOD_WATER=6
uint8 RESCUE_ASSET=7
uint8 VEHICLE=8

std_msgs/Header header
uint32 track_id
string observation_uuid
uint64 event_sequence
sar_yolo_detector/PerceptionIdentity provenance
uint8 class_id
string class_name
uint8 semantic_type
uint8 status

# Stable ID supplied by an upstream Tracker when available. The generator uses
# it before geometric association but never assumes an absent ID is stable.
bool source_track_id_valid
int64 source_track_id

# confidence is the exponentially smoothed detector confidence; stability also
# includes temporal confirmation. priority is provided for task arbitration.
float32 confidence
float32 stability
float32 priority
uint32 observation_count
uint32 consecutive_hits
time first_seen
time last_seen

# A confirmed localised target has task_pose in task_frame.  When false, the
# pose is intentionally not a navigation command; use bearing for inspection.
bool localization_valid
geometry_msgs/PoseStamped task_pose
geometry_msgs/Vector3Stamped bearing
float64[9] position_covariance

# Image-space box retained for audit, operator display, and Tracker hand-off.
vision_msgs/Detection2D image_detection
string sensor_id
