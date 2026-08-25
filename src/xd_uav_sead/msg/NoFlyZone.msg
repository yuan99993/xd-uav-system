# Versioned dynamic no-fly-zone command in a local ENU control frame.
uint8 CURRENT_SCHEMA_VERSION=1
uint8 OP_UPSERT=0
uint8 OP_REMOVE=1
uint8 OP_CLEAR=2
uint8 TYPE_NO_FLY=0

std_msgs/Header header
uint8 schema_version
uint8 operation
uint32 zone_id
bool enabled
uint8 zone_type
float64 min_altitude
float64 max_altitude
# Zero means permanent until OP_REMOVE/OP_CLEAR; otherwise absolute ROS expiry time.
time valid_until
geometry_msgs/Polygon polygon
