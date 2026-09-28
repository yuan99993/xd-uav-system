uint32 source_candidate_index
int32 track_id
int32 class_id
float32 confidence

string image_source
string sensor_id
string detector_name
string model_version

bool position_valid
# WGS84 latitude/longitude in degrees and ellipsoidal altitude in metres.
geographic_msgs/GeoPoint position
# Row-major 3x3 covariance in the array header.frame_id local ENU frame, m^2.
float64[9] position_covariance_enu
