# Candidate shape.
uint8 POINT=0
uint8 IMAGE_REGION=1

# Lifecycle events. event_sequence is monotonic for one observation_uuid.
uint8 PENDING=0
uint8 CONFIRMED=1
uint8 UPDATED=2
uint8 EXPIRED=3

# Mission-wide semantic ontology. raw_class_id/name are retained for audit.
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
string observation_uuid
uint64 event_sequence
sar_yolo_detector/MissionIdentity source

uint8 candidate_shape
uint32 local_track_id
bool source_track_id_valid
int64 source_track_id
uint8 raw_class_id
string raw_class_name
uint8 semantic_type
uint8 lifecycle_state

float32 confidence
float32 stability
float32 priority
uint32 observation_count
time first_seen
time last_seen

# A false localization_valid explicitly means that target_pose is not a
# navigation target. bearing is still useful for a follow-up observation.
bool localization_valid
geometry_msgs/PoseStamped target_pose
geometry_msgs/Vector3Stamped bearing
float64[9] position_covariance

# Optional image evidence. POINT candidates normally use image_roi while
# IMAGE_REGION candidates may additionally carry image_contour.
bool image_region_valid
sensor_msgs/RegionOfInterest image_roi
geometry_msgs/PolygonStamped image_contour
geometry_msgs/PolygonStamped world_polygon
float64 world_area_m2
float32 world_projection_coverage
sar_yolo_detector/EvidenceReference evidence

string sensor_id
string profile
string model_version
string model_sha256
string calibration_version
string coordinate_contract
sar_yolo_detector/GeoReference geo_reference
