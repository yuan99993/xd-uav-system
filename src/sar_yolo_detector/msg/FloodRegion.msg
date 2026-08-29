# Mission-decision evidence from one stable semantic-mask component.
uint8 PENDING=0
uint8 CONFIRMED=1
uint8 UPDATED=2
uint8 EXPIRED=3

uint8 SEMANTIC_UNKNOWN=0
uint8 FLOODED_BUILDING=4
uint8 FLOODED_ROAD=5
uint8 FLOOD_WATER=6

std_msgs/Header header
uint32 track_id
string observation_uuid
uint64 event_sequence
sar_yolo_detector/PerceptionIdentity provenance
uint8 class_id
string class_name
uint8 semantic_type
uint8 status
float32 confidence
float32 stability
float32 priority
uint32 observation_count
uint32 consecutive_hits
time first_seen
time last_seen
uint32 pixel_area
float32 image_area_ratio

geometry_msgs/PolygonStamped image_contour
sensor_msgs/RegionOfInterest image_roi
geometry_msgs/PolygonStamped world_polygon
float64 world_area_m2
float32 world_projection_coverage

# False unless capture-time CameraInfo/TF and the configured terrain or plane
# projection all satisfy the coverage and freshness gates.
bool localization_valid
geometry_msgs/PoseStamped map_centroid
float64[9] position_covariance
string evidence_uuid
string evidence_source_topic
string evidence_image_sha256
string evidence_retrieval_uri
string sensor_id
